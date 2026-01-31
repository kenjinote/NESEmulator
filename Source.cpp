#include <windows.h>
#include <commdlg.h>
#include <shellapi.h>
#include <d2d1.h>
#include <dsound.h>
#include <vector>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <string>
#include <ctime>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <memory>
#include <functional>
#pragma comment(lib, "shell32.lib")
#pragma comment(lib, "d2d1.lib")
#pragma comment(lib, "dsound.lib")
#pragma comment(lib, "dxguid.lib")
#pragma comment(lib, "winmm.lib")
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "user32.lib")
using Byte = uint8_t; using Word = uint16_t;
enum MIRROR { HARDWARE, HORIZONTAL, VERTICAL, ONESCREEN_LO, ONESCREEN_HI };
const int NES_WIDTH = 256, NES_HEIGHT = 240, SAMPLE_RATE = 44100, NUM_CHANNELS = 2;
const double NES_CPU_CLOCK = 1789773.0;
#define IDM_FILE_OPEN 1001
#define IDM_FILE_EXIT 1002
#define IDM_FILE_FULLSCREEN 1003
template<class T> void SafeRelease(T** ppT) { if (*ppT) { (*ppT)->Release(); *ppT = NULL; } }
class Bus; class CPU; class PPU; class APU; class Cartridge; class Mapper;
class AudioDriver {
	IDirectSound8* m_pDS = NULL; IDirectSoundBuffer* m_pPrimary = NULL, * m_pSecondary = NULL; int m_bufferSize = 0, m_nextWriteOffset = 0;
public:
	~AudioDriver() { SafeRelease(&m_pSecondary); SafeRelease(&m_pPrimary); SafeRelease(&m_pDS); }
	bool Initialize(HWND h) {
		if (FAILED(DirectSoundCreate8(NULL, &m_pDS, NULL)) || FAILED(m_pDS->SetCooperativeLevel(h, DSSCL_PRIORITY)))return false;
		DSBUFFERDESC d = { sizeof(DSBUFFERDESC),DSBCAPS_PRIMARYBUFFER,0,0,NULL };
		if (FAILED(m_pDS->CreateSoundBuffer(&d, &m_pPrimary, NULL)))return false;
		WAVEFORMATEX w = { WAVE_FORMAT_PCM,NUM_CHANNELS,SAMPLE_RATE,SAMPLE_RATE * (NUM_CHANNELS * 16 / 8),(NUM_CHANNELS * 16 / 8),16,0 };
		if (FAILED(m_pPrimary->SetFormat(&w)))return false;
		m_bufferSize = w.nAvgBytesPerSec; d.dwFlags = DSBCAPS_GETCURRENTPOSITION2 | DSBCAPS_GLOBALFOCUS | DSBCAPS_CTRLVOLUME; d.dwBufferBytes = m_bufferSize; d.lpwfxFormat = &w;
		if (FAILED(m_pDS->CreateSoundBuffer(&d, &m_pSecondary, NULL)))return false;
		ClearBuffer(); m_pSecondary->Play(0, 0, DSBPLAY_LOOPING); return true;
	}
	void ClearBuffer() { void* p1, * p2; DWORD l1, l2; if (SUCCEEDED(m_pSecondary->Lock(0, m_bufferSize, &p1, &l1, &p2, &l2, 0))) { ZeroMemory(p1, l1); if (p2)ZeroMemory(p2, l2); m_pSecondary->Unlock(p1, l1, p2, l2); }m_nextWriteOffset = 0; }
	void Stop() { if (m_pSecondary)m_pSecondary->Stop(); }
	void Resume() { if (m_pSecondary)m_pSecondary->Play(0, 0, DSBPLAY_LOOPING); }
	void PushSamples(const std::vector<int16_t>& s) {
		if (!m_pSecondary || s.empty())return;
		int b = (int)s.size() * sizeof(int16_t), sm = m_bufferSize / 20; DWORD p, w; if (FAILED(m_pSecondary->GetCurrentPosition(&p, &w)))return;
		int l = m_nextWriteOffset - (int)p; if (l < 0)l += m_bufferSize; if (l<sm || l>m_bufferSize / 2)m_nextWriteOffset = ((int)p + sm) % m_bufferSize;
		void* p1, * p2; DWORD l1, l2; if (SUCCEEDED(m_pSecondary->Lock(m_nextWriteOffset, b, &p1, &l1, &p2, &l2, 0))) { memcpy(p1, s.data(), l1); if (p2)memcpy(p2, (uint8_t*)s.data() + l1, l2); m_pSecondary->Unlock(p1, l1, p2, l2); m_nextWriteOffset = (m_nextWriteOffset + b) % m_bufferSize; }
	}
};
struct LengthUnit {
	uint8_t counter = 0; bool enabled = false, halt = false; static constexpr uint8_t lookup[32] = { 10,254,20,2,40,4,80,6,160,8,60,10,14,12,26,14,12,16,24,18,48,20,96,22,192,24,72,26,16,28,32,30 };
	void Load(uint8_t c) { if (enabled)counter = lookup[c & 0x1F]; } void Tick() { if (!enabled)counter = 0; else if (counter > 0 && !halt)counter--; }
};
struct EnvelopeUnit {
	bool start = false, loop = false, constant = false; uint8_t volume = 0, output = 0, decay = 0, divider = 0;
	void Tick() { if (!start) { if (divider == 0) { divider = volume; if (decay == 0) { if (loop)decay = 15; } else decay--; } else divider--; } else { start = false; decay = 15; divider = volume; }output = constant ? volume : decay; }
};
struct SweepUnit {
	bool enabled = false, negate = false, reload = false, mute = false; uint8_t period = 0, shift = 0, timer = 0; uint16_t target_period = 0;
	void Track(uint16_t c, int i) { int ch = c >> shift; if (negate) { ch = -ch; if (i == 0)ch--; }int t = c + ch; mute = (c < 8) || (t > 0x7FF); target_period = (uint16_t)((t < 0) ? 0 : t); }
	bool Tick(uint16_t& c, int i) { bool ch = false; Track(c, i); if (timer == 0 && enabled && !mute && shift > 0) { c = target_period; Track(c, i); ch = true; }if (timer == 0 || reload) { timer = period; reload = false; } else timer--; return ch; }
};
struct ChannelPulse {
	int id; uint16_t timer = 0, timer_reload = 0; uint8_t duty = 0, sequence_pos = 0; LengthUnit length; EnvelopeUnit envelope; SweepUnit sweep;
	static constexpr uint8_t duty_lookup[4][8] = { {0,1,0,0,0,0,0,0},{0,1,1,0,0,0,0,0},{0,1,1,1,1,0,0,0},{1,0,0,1,1,1,1,1} };
	void TickTimer() { if (timer > 0)timer--; else { timer = timer_reload; sequence_pos = (sequence_pos + 1) & 7; } }
	uint8_t GetSample() { return(length.counter == 0 || sweep.mute || !duty_lookup[duty][sequence_pos]) ? 0 : envelope.output; }
};
struct ChannelTriangle {
	uint16_t timer = 0, timer_reload = 0; uint8_t sequence_pos = 0, linear_counter = 0, linear_reload_val = 0; bool linear_reload_flag = false, control_flag = false; LengthUnit length;
	static constexpr uint8_t sequence_lookup[32] = { 15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15 };
	void TickTimer() { if (timer > 0)timer--; else { timer = timer_reload; if (length.counter > 0 && linear_counter > 0)sequence_pos = (sequence_pos + 1) & 31; } }
	void TickLinear() { if (linear_reload_flag)linear_counter = linear_reload_val; else if (linear_counter > 0)linear_counter--; if (!control_flag)linear_reload_flag = false; }
	uint8_t GetSample() { return sequence_lookup[sequence_pos]; }
};
struct ChannelNoise {
	uint16_t timer = 0, timer_reload = 0, lfsr = 1; uint8_t mode = 0; LengthUnit length; EnvelopeUnit envelope;
	static constexpr uint16_t period_lookup[16] = { 4,8,16,32,64,96,128,160,202,254,380,508,762,1016,2034,4068 };
	void TickTimer() { if (timer > 0)timer--; else { timer = timer_reload; uint8_t f = (lfsr & 1) ^ ((lfsr >> (mode ? 6 : 1)) & 1); lfsr >>= 1; lfsr |= (f << 14); } }
	uint8_t GetSample() { return(length.counter == 0 || (lfsr & 1)) ? 0 : envelope.output; }
};
class APU {
	Bus* bus; double pulse_table[31], tnd_table[203], clock_time = 0, sample_accumulator = 0, high_pass_capacitor = 0, low_pass_accumulator = 0; int frame_clock_counter = 0, sample_count = 0;
public:
	ChannelPulse p1, p2; ChannelTriangle tri; ChannelNoise noise; std::vector<int16_t> buffer;
	explicit APU(Bus& b) :bus(&b) { InitializeMixerTables(); Reset(); }
	void InitializeMixerTables() { pulse_table[0] = 0; for (int i = 1; i < 31; i++)pulse_table[i] = 95.52 / (8128.0 / i + 100); tnd_table[0] = 0; for (int i = 1; i < 203; i++)tnd_table[i] = 159.79 / (100 + (1.0 / (i / 10000.0))); }
	void Reset() { p1 = ChannelPulse(); p1.id = 0; p2 = ChannelPulse(); p2.id = 1; tri = ChannelTriangle(); noise = ChannelNoise(); p1.length.enabled = p2.length.enabled = tri.length.enabled = noise.length.enabled = true; buffer.clear(); buffer.reserve(4096); frame_clock_counter = 0; clock_time = 0; sample_accumulator = 0; sample_count = 0; low_pass_accumulator = 0; high_pass_capacitor = 0; }
	uint8_t Read(uint16_t) { return 0; }
	void Write(uint16_t addr, uint8_t data) {
		switch (addr) {
		case 0x4000:p1.duty = (data >> 6) & 3; p1.length.halt = p1.envelope.loop = (data & 0x20); p1.envelope.constant = (data & 0x10); p1.envelope.volume = (data & 0x0F); break;
		case 0x4001:p1.sweep.enabled = (data & 0x80); p1.sweep.period = (data >> 4) & 7; p1.sweep.negate = (data & 0x08); p1.sweep.shift = (data & 0x07); p1.sweep.reload = true; break;
		case 0x4002:p1.timer_reload = (p1.timer_reload & 0xFF00) | data; break;
		case 0x4003:p1.timer_reload = (p1.timer_reload & 0x00FF) | ((data & 7) << 8); p1.sequence_pos = 0; p1.length.Load(data >> 3); p1.envelope.start = true; break;
		case 0x4004:p2.duty = (data >> 6) & 3; p2.length.halt = p2.envelope.loop = (data & 0x20); p2.envelope.constant = (data & 0x10); p2.envelope.volume = (data & 0x0F); break;
		case 0x4005:p2.sweep.enabled = (data & 0x80); p2.sweep.period = (data >> 4) & 7; p2.sweep.negate = (data & 0x08); p2.sweep.shift = (data & 0x07); p2.sweep.reload = true; break;
		case 0x4006:p2.timer_reload = (p2.timer_reload & 0xFF00) | data; break;
		case 0x4007:p2.timer_reload = (p2.timer_reload & 0x00FF) | ((data & 7) << 8); p2.sequence_pos = 0; p2.length.Load(data >> 3); p2.envelope.start = true; break;
		case 0x4008:tri.control_flag = (data & 0x80); tri.linear_reload_val = (data & 0x7F); break;
		case 0x400A:tri.timer_reload = (tri.timer_reload & 0xFF00) | data; break;
		case 0x400B:tri.timer_reload = (tri.timer_reload & 0x00FF) | ((data & 7) << 8); tri.length.Load(data >> 3); tri.linear_reload_flag = true; break;
		case 0x400C:noise.length.halt = noise.envelope.loop = (data & 0x20); noise.envelope.constant = (data & 0x10); noise.envelope.volume = (data & 0x0F); break;
		case 0x400E:noise.mode = (data & 0x80); noise.timer_reload = ChannelNoise::period_lookup[data & 0x0F]; break;
		case 0x400F:noise.length.Load(data >> 3); noise.envelope.start = true; break;
		case 0x4015:p1.length.enabled = (data & 1); if (!p1.length.enabled)p1.length.counter = 0; p2.length.enabled = (data & 2); if (!p2.length.enabled)p2.length.counter = 0; tri.length.enabled = (data & 4); if (!tri.length.enabled)tri.length.counter = 0; noise.length.enabled = (data & 8); if (!noise.length.enabled)noise.length.counter = 0; break;
		case 0x4017:frame_clock_counter = 0; if (data & 0x80)TickFrameCounter(); break;
		}
	}
	void TickFrameCounter() { frame_clock_counter++; if (frame_clock_counter == 1 || frame_clock_counter == 3) { p1.envelope.Tick(); p2.envelope.Tick(); noise.envelope.Tick(); tri.TickLinear(); } else if (frame_clock_counter == 2 || frame_clock_counter == 4) { p1.envelope.Tick(); p2.envelope.Tick(); noise.envelope.Tick(); tri.TickLinear(); p1.length.Tick(); p2.length.Tick(); tri.length.Tick(); noise.length.Tick(); p1.sweep.Tick(p1.timer_reload, 0); p2.sweep.Tick(p2.timer_reload, 1); }if (frame_clock_counter >= 4)frame_clock_counter = 0; }
	void Step(int cycles) {
		for (int i = 0; i < cycles; i++) {
			if (i % 2 == 0) { p1.TickTimer(); p2.TickTimer(); noise.TickTimer(); } tri.TickTimer(); clock_time += 1.0;
			if (clock_time >= 7457.3875) { clock_time -= 7457.3875; TickFrameCounter(); }
			uint8_t p1o = p1.GetSample(), p2o = p2.GetSample(), to = tri.GetSample(), no = noise.GetSample();
			double pm = 0, tnd = 0; if (p1o + p2o > 0)pm = 95.52 / (8128.0 / (p1o + p2o) + 100.0); if (to > 0 || no > 0)tnd = 159.79 / (1.0 / (to / 8227.0 + no / 12241.0) + 100.0);
			sample_accumulator += pm + tnd; sample_count++;
			if (sample_count >= 40) { double m = sample_accumulator / sample_count, hpf = m - low_pass_accumulator; low_pass_accumulator = m - hpf * 0.995; int16_t s = (int16_t)(hpf * 30000.0); buffer.push_back(s); buffer.push_back(s); sample_accumulator = 0; sample_count = 0; }
		}
	}
};
union PPU_CTRL { struct { uint8_t nametable_x : 1, nametable_y : 1, vram_inc : 1, sprite_pattern : 1, bg_pattern : 1, sprite_size : 1, slave_mode : 1, nmi_enable : 1; }bits; uint8_t raw; };
union PPU_MASK { struct { uint8_t grayscale : 1, render_bg_left : 1, render_spr_left : 1, render_bg : 1, render_spr : 1, enhance_red : 1, enhance_green : 1, enhance_blue : 1; }bits; uint8_t raw; };
union PPU_STATUS { struct { uint8_t unused : 5, sprite_overflow : 1, sprite_zero_hit : 1, vblank : 1; }bits; uint8_t raw; };
union PPU_ADDR_REG { struct { uint16_t coarse_x : 5, coarse_y : 5, nametable_x : 1, nametable_y : 1, fine_y : 3, unused : 1; }bits; uint16_t raw; };
struct OAM_ENTRY { uint8_t y, tile_id; struct { uint8_t palette : 2, unused : 3, priority : 1, flip_h : 1, flip_v : 1; }attr; uint8_t x; };
class PPU {
	Bus* pBus = NULL; Cartridge* pCart = NULL; uint32_t* pScreenBuffer = NULL;
public:
	uint8_t nameTables[2][1024], paletteRam[32], * pNameTables[4], oamAddr = 0, fineX = 0, readBuffer = 0; OAM_ENTRY primaryOAM[64]; MIRROR currentMirrorMode = HARDWARE;
	PPU_CTRL ctrl; PPU_MASK mask; PPU_STATUS status; PPU_ADDR_REG vAddr, tAddr; bool writeToggle = false, frameComplete = false, nmiOccurred = false; int16_t scanline = 0, cycle = 0;
	const uint32_t SYSTEM_PALETTE[64] = { 0xFF7C7C7C,0xFF0000FC,0xFF0000BC,0xFF4428BC,0xFF940084,0xFFA80020,0xFFA81000,0xFF881400,0xFF503000,0xFF007800,0xFF006800,0xFF005800,0xFF004058,0xFF000000,0xFF000000,0xFF000000,0xFFBCBCBC,0xFF0078F8,0xFF0058F8,0xFF6844FC,0xFFD800CC,0xFFE40058,0xFFF83800,0xFFE45C10,0xFFAC7C00,0xFF00B800,0xFF00A800,0xFF00A844,0xFF008888,0xFF000000,0xFF000000,0xFF000000,0xFFF8F8F8,0xFF3CBCFC,0xFF6888FC,0xFF9878F8,0xFFF878F8,0xFFF85898,0xFFF87858,0xFFFCA044,0xFFF8B800,0xFFB8F818,0xFF58D854,0xFF58F898,0xFF00E8D8,0xFF787878,0xFF000000,0xFF000000,0xFFFCFCFC,0xFFA4E4FC,0xFFB8B8F8,0xFFD8B8F8,0xFFF8B8F8,0xFFF8A4C0,0xFFF0D0B0,0xFFFCE0A8,0xFFF8D878,0xFFD8F878,0xFFB8F8B8,0xFFB8F8D8,0xFF00FCFC,0xFFF8D8F8,0xFF000000,0xFF000000 };
	explicit PPU(Bus& b); void SetCart(Cartridge* c); void SetScreenBuffer(uint32_t* b); void Reset(); uint8_t cpuRead(uint16_t a); void cpuWrite(uint16_t a, uint8_t d); uint8_t InternalRead(uint16_t a); void InternalWrite(uint16_t a, uint8_t d); void RenderScanline(); void Clock(); void UpdateNametables();
};
class CPU {
	Bus* bus;
public:
	uint8_t A = 0, X = 0, Y = 0, S = 0xFD, P = 0; uint16_t PC = 0; int cyclesLeft = 0; enum FLAGS { C = 1, Z = 2, I = 4, D = 8, B = 16, U = 32, V = 64, N = 128 };
	explicit CPU(Bus& b); void Reset(); void IRQ(); void NMI(); void Run(int c); uint8_t Read(uint16_t a); void Write(uint16_t a, uint8_t d); void Push(uint8_t v); uint8_t Pop(); void SetFlag(FLAGS f, bool v); void UpdateZN(uint8_t v);
	inline bool CheckPageCross(uint16_t a, uint16_t b) { return(a & 0xFF00) != (b & 0xFF00); }
	inline uint16_t GetAddr_ZP(); inline uint16_t GetAddr_ZPX(); inline uint16_t GetAddr_ZPY(); inline uint16_t GetAddr_ABS(); inline uint16_t GetAddr_ABX(bool& p); inline uint16_t GetAddr_ABY(bool& p); inline uint16_t GetAddr_IZX(); inline uint16_t GetAddr_IZY(bool& p);
	void Op_ADC(uint8_t v); void Op_SBC(uint8_t v); void Op_AND(uint8_t v); void Op_ORA(uint8_t v); void Op_EOR(uint8_t v); void Op_CMP(uint8_t r, uint8_t v); void Op_BIT(uint8_t v); void Op_INC(uint16_t a); void Op_DEC(uint16_t a);
	void Op_ASL(uint16_t a); void Op_LSR(uint16_t a); void Op_ROL(uint16_t a); void Op_ROR(uint16_t a); void Op_ASL_A(); void Op_LSR_A(); void Op_ROL_A(); void Op_ROR_A(); void Op_Branch(bool c);
	void Op_LAX(uint8_t v); void Op_SAX(uint16_t a); void Op_DCP(uint16_t a); void Op_ISB(uint16_t a); void Op_SLO(uint16_t a); void Op_RLA(uint16_t a); void Op_SRE(uint16_t a); void Op_RRA(uint16_t a);
};
using ReadHandler = std::function<Byte(Word)>; using WriteHandler = std::function<void(Word, Byte)>;
class Bus {
	CPU* cpu; PPU* ppu; APU* apu; void InitializeMemoryMap();
public:
	Cartridge* cart = NULL; Byte ram[2048], controller[2], controller_state[2], dma_page = 0, dma_addr = 0; bool dma_transfer = false; Byte* pageReadPtrs[256]; WriteHandler pageWriteHandlers[256]; ReadHandler pageReadHandlers[256];
	Bus(CPU& c, PPU& p, APU& a); void SetCart(Cartridge* c); void ClearMap(); void MapReadPage(Byte p, Byte* d) { pageReadPtrs[p] = d; } void MapWriteHandler(Byte p, WriteHandler h) { pageWriteHandlers[p] = h; } void MapReadHandler(Byte p, ReadHandler h) { pageReadPtrs[p] = NULL; pageReadHandlers[p] = h; }
	Byte cpuRead(Word a); void cpuWrite(Word a, Byte d); CPU* GetCPU() { return cpu; }
};
class Mapper {
protected:
	Byte nPRGBanks, nCHRBanks; Bus* pBus; std::vector<Byte>& PRGData, & CHRData;
	void MapPRG(int s, int n, int b, int sz) { int t = static_cast<int>(PRGData.size()); for (int i = 0; i < n; i++)pBus->MapReadPage(s + i, &PRGData[((b * sz) + (i * 256)) & (t - 1)]); }
public:
	Mapper(Byte p, Byte c, Bus* b, std::vector<Byte>& pg, std::vector<Byte>& ch) :nPRGBanks(p), nCHRBanks(c), pBus(b), PRGData(pg), CHRData(ch) {}
	virtual ~Mapper() {} virtual void Reset() = 0; virtual bool ppuMapRead(Word a, uint32_t& m) = 0; virtual bool ppuMapWrite(Word a, uint32_t& m) = 0; virtual MIRROR mirror() { return HARDWARE; } virtual bool irqState() { return false; } virtual void irqClear() {} virtual void tick(int) {}
};
class Mapper_000 :public Mapper {
public:
	using Mapper::Mapper; void Reset()override { MapPRG(0x80, 128, 0, 32768); for (int p = 0x80; p <= 0xFF; p++)pBus->MapWriteHandler(p, [](Word, Byte) {}); }
	bool ppuMapRead(Word a, uint32_t& m)override { if (a <= 0x1FFF) { m = a; return true; }return false; } bool ppuMapWrite(Word a, uint32_t& m)override { if (a <= 0x1FFF) { m = a; return true; }return false; }
};
class Mapper_001 : public Mapper {
	Byte nLoadReg = 0, nLoadRegCount = 0, nCtrlReg = 0x0C, nCHR4Lo = 0, nCHR4Hi = 0, nCHR8 = 0, nPRG16Lo = 0, nPRG16Hi = 0, nPRG32 = 0;
	std::vector<Byte> vRAM;
public:
	using Mapper::Mapper;
	void Reset() override {
		vRAM.resize(8192); std::fill(vRAM.begin(), vRAM.end(), 0);
		nLoadReg = 0; nLoadRegCount = 0; nCtrlReg = 0x0C; nPRG16Lo = 0; nPRG16Hi = nPRGBanks - 1; nPRG32 = 0;
		for (int p = 0x60; p < 0x80; p++) { pBus->MapReadPage(p, &vRAM[(p - 0x60) * 256]); pBus->MapWriteHandler(p, [this](Word a, Byte d) { vRAM[a & 0x1FFF] = d; }); }
		for (int p = 0x80; p <= 0xFF; p++) { pBus->MapWriteHandler(p, [this](Word a, Byte d) { Write(a, d); }); }
		UpdateState();
	}
	void Write(Word addr, Byte data) {
		if (data & 0x80) { nLoadReg = 0; nLoadRegCount = 0; nCtrlReg |= 0x0C; UpdateState(); }
		else {
			nLoadReg >>= 1; nLoadReg |= (data & 1) << 4; nLoadRegCount++;
			if (nLoadRegCount == 5) {
				Byte target = (addr >> 13) & 3;
				if (target == 0) { nCtrlReg = nLoadReg & 0x1F; }
				else if (target == 1) { if (nCtrlReg & 0x10) nCHR4Lo = nLoadReg & 0x1F; else nCHR8 = (nLoadReg & 0x1E) >> 1; }
				else if (target == 2) { if (nCtrlReg & 0x10) nCHR4Hi = nLoadReg & 0x1F; }
				else if (target == 3) {
					Byte mode = (nCtrlReg >> 2) & 3;
					if (mode <= 1) { nPRG32 = (nLoadReg & 0x0E) >> 1; }
					else if (mode == 2) { nPRG16Lo = 0; nPRG16Hi = nLoadReg & 0x0F; }
					else if (mode == 3) { nPRG16Lo = nLoadReg & 0x0F; nPRG16Hi = nPRGBanks - 1; }
				}
				nLoadReg = 0; nLoadRegCount = 0; UpdateState();
			}
		}
	}
	void UpdateState() {
		switch ((nCtrlReg >> 2) & 3) {
		case 0: case 1: MapPRG(0x80, 128, nPRG32, 32768); break;
		case 2: MapPRG(0x80, 64, 0, 16384); MapPRG(0xC0, 64, nPRG16Hi, 16384); break;
		case 3: MapPRG(0x80, 64, nPRG16Lo, 16384); MapPRG(0xC0, 64, nPRGBanks - 1, 16384); break;
		}
	}
	MIRROR mirror() override {
switch (nCtrlReg & 3) { case 0: return ONESCREEN_LO; case 1: return ONESCREEN_HI; case 2: return VERTICAL; case 3: return HORIZONTAL; } return HARDWARE;
	}
	bool ppuMapRead(Word addr, uint32_t& m) override {
		if (addr < 0x2000) {
			if (nCHRBanks == 0) { m = addr; return true; }
			if (nCtrlReg & 0x10) { if (addr < 0x1000) m = (nCHR4Lo * 0x1000) + (addr & 0x0FFF); else m = (nCHR4Hi * 0x1000) + (addr & 0x0FFF); }
			else { m = (nCHR8 * 0x2000) + (addr & 0x1FFF); } return true;
		} return false;
	}
	bool ppuMapWrite(Word addr, uint32_t& m) override { if (addr < 0x2000 && nCHRBanks == 0) { m = addr; return true; } return false; }
};
class Mapper_002 : public Mapper {
	Byte nPRGBankSelectLo = 0;
public:
	using Mapper::Mapper;
	void Reset() override {
		nPRGBankSelectLo = 0; MapPRG(0xC0, 64, nPRGBanks - 1, 16384); MapPRG(0x80, 64, 0, 16384);
		for (int p = 0x80; p <= 0xFF; p++) pBus->MapWriteHandler(p, [this](Word a, Byte d) { nPRGBankSelectLo = d; MapPRG(0x80, 64, nPRGBankSelectLo, 16384); });
	}
	bool ppuMapRead(Word a, uint32_t& m) override { if (a < 0x2000) { m = a; return true; } return false; }
	bool ppuMapWrite(Word a, uint32_t& m) override { if (a < 0x2000) { m = a; return true; } return false; }
};
class Mapper_003 : public Mapper {
	Byte nCHRBankSelect = 0;
public:
	using Mapper::Mapper;
	void Reset() override {
		nCHRBankSelect = 0; MapPRG(0x80, 128, 0, 32768);
		for (int p = 0x80; p <= 0xFF; p++) pBus->MapWriteHandler(p, [this](Word a, Byte d) { nCHRBankSelect = d & 0x03; });
	}
	bool ppuMapRead(Word a, uint32_t& m) override { if (a < 0x2000) { m = (nCHRBankSelect * 0x2000) + a; return true; } return false; }
	bool ppuMapWrite(Word a, uint32_t& m) override { return ppuMapRead(a, m); }
};
class Mapper_004 : public Mapper {
	uint8_t nTargetRegister = 0; bool bPRGBankMode = false, bCHRInversion = false, bIRQActive = false, bIRQEnable = false, bIRQReload = false;
	MIRROR mirroringMode = HORIZONTAL; uint32_t pRegister[8], pCHRBank[8]; uint8_t nIRQCounter = 0, nIRQLatch = 0;
	std::vector<Byte> vRAM; int irq_filter_delay = 0;
public:
	using Mapper::Mapper;
	void Reset() override {
		vRAM.resize(8192); std::fill(vRAM.begin(), vRAM.end(), 0);
		nTargetRegister = 0; bPRGBankMode = false; bCHRInversion = false; mirroringMode = HORIZONTAL;
		bIRQActive = false; bIRQEnable = false; nIRQCounter = 0; nIRQLatch = 0; bIRQReload = false; irq_filter_delay = 0;
		memset(pRegister, 0, sizeof(pRegister)); memset(pCHRBank, 0, sizeof(pCHRBank)); pRegister[6] = 0; pRegister[7] = 1;
		for (int p = 0x60; p < 0x80; p++) { pBus->MapReadPage(p, &vRAM[(p - 0x60) * 256]); pBus->MapWriteHandler(p, [this](Word a, Byte d) { vRAM[a & 0x1FFF] = d; }); }
		for (int p = 0x80; p <= 0xFF; p++) { pBus->MapWriteHandler(p, [this](Word a, Byte d) { Write(a, d); }); }
		UpdateBanks();
	}
	void UpdateBanks() {
		if (bPRGBankMode) { MapPRG(0x80, 32, nPRGBanks * 2 - 2, 8192); MapPRG(0xA0, 32, pRegister[7], 8192); MapPRG(0xC0, 32, pRegister[6], 8192); MapPRG(0xE0, 32, nPRGBanks * 2 - 1, 8192); }
		else { MapPRG(0x80, 32, pRegister[6], 8192); MapPRG(0xA0, 32, pRegister[7], 8192); MapPRG(0xC0, 32, nPRGBanks * 2 - 2, 8192); MapPRG(0xE0, 32, nPRGBanks * 2 - 1, 8192); }
		if (bCHRInversion) {
			pCHRBank[0] = pRegister[2]; pCHRBank[1] = pRegister[3]; pCHRBank[2] = pRegister[4]; pCHRBank[3] = pRegister[5];
			pCHRBank[4] = pRegister[0] & 0xFE; pCHRBank[5] = pRegister[0] | 1; pCHRBank[6] = pRegister[1] & 0xFE; pCHRBank[7] = pRegister[1] | 1;
		}
		else {
			pCHRBank[0] = pRegister[0] & 0xFE; pCHRBank[1] = pRegister[0] | 1; pCHRBank[2] = pRegister[1] & 0xFE; pCHRBank[3] = pRegister[1] | 1;
			pCHRBank[4] = pRegister[2]; pCHRBank[5] = pRegister[3]; pCHRBank[6] = pRegister[4]; pCHRBank[7] = pRegister[5];
		}
	}
	void Write(Word addr, Byte data) {
		if (addr >= 0x8000 && addr <= 0x9FFF) {
			if (!(addr & 1)) { nTargetRegister = data & 0x07; bPRGBankMode = (data & 0x40); bCHRInversion = (data & 0x80); }
			else { pRegister[nTargetRegister] = data; } UpdateBanks();
		}
		else if (addr >= 0xA000 && addr <= 0xBFFF) { if (!(addr & 1)) mirroringMode = (data & 1) ? HORIZONTAL : VERTICAL; }
		else if (addr >= 0xC000 && addr <= 0xDFFF) { if (!(addr & 1)) nIRQLatch = data; else bIRQReload = true; }
		else if (addr >= 0xE000 && addr <= 0xFFFF) { if (!(addr & 1)) { bIRQEnable = false; bIRQActive = false; } else bIRQEnable = true; }
	}
	bool ppuMapRead(Word addr, uint32_t& m) override {
		if (addr < 0x3F00 && (addr & 0x1000)) {
			if (irq_filter_delay <= 0) {
				if (nIRQCounter == 0 || bIRQReload) { nIRQCounter = nIRQLatch; bIRQReload = false; }
				else nIRQCounter--;
				if (nIRQCounter == 0 && bIRQEnable) bIRQActive = true; irq_filter_delay = 18;
			}
		}
		if (addr < 0x2000) { uint32_t bank = pCHRBank[addr / 0x400]; if (nCHRBanks > 0) bank %= (nCHRBanks * 8); m = (bank * 0x400) + (addr & 0x3FF); return true; }
		return false;
	}
	bool ppuMapWrite(Word a, uint32_t& m) override { return ppuMapRead(a, m); }
	MIRROR mirror() override { return mirroringMode; }
	bool irqState() override { return bIRQActive; }
	void irqClear() override { bIRQActive = false; }
	void tick(int cycles) override { if (irq_filter_delay > 0) irq_filter_delay -= cycles; }
};
class Mapper_016 : public Mapper {
	std::vector<uint8_t> wram; uint8_t eeprom_mem[256];
	int nPRG = 0, nCHR[8], i2c_state = 0, i2c_bit = 0;
	uint16_t nIRQCounter = 0; uint8_t i2c_shift = 0, i2c_ptr = 0;
	bool bIRQEnable = false, bIRQActive = false, bIRQPending = false;
	bool i2c_sda = true, i2c_scl = false, i2c_out = true, i2c_readMode = false, i2c_addrPhase = true;
	MIRROR mirrorMode = VERTICAL;
public:
	using Mapper::Mapper;
	void Reset() override {
		nPRG = 0; for (int i = 0; i < 8; i++) nCHR[i] = i;
		nIRQCounter = 0; bIRQEnable = bIRQActive = bIRQPending = false;
		wram.assign(8192, 0); memset(eeprom_mem, 0xFF, 256);
		i2c_state = i2c_bit = 0; i2c_sda = i2c_out = i2c_addrPhase = true; i2c_scl = i2c_readMode = false;
		for (int p = 0x60; p <= 0xFF; p++) {
			pBus->MapReadHandler(p, [this](Word a) { return Read(a); });
			pBus->MapWriteHandler(p, [this](Word a, Byte d) { Write(a, d); });
		}
	}
	Byte Read(Word addr) {
		if (addr == 0x800D || (addr >= 0x6000 && addr < 0x8000 && (addr & 0xF) == 0xD)) return (i2c_sda && i2c_out) ? 0x10 : 0;
		if (addr >= 0x6000 && addr < 0x8000) return wram[addr & 0x1FFF];
		int bank = (addr < 0xC000) ? nPRG : (nPRGBanks - 1);
		return PRGData[((size_t)bank * 16384 + (addr & 0x3FFF)) & (PRGData.size() - 1)];
	}
	void Write(Word addr, Byte d) {
		if (addr >= 0x6000 && addr < 0x8000) wram[addr & 0x1FFF] = d;
		switch (addr & 0xF) {
		case 0x00: case 0x01: case 0x02: case 0x03: case 0x04: case 0x05: case 0x06: case 0x07: nCHR[addr & 7] = d; break;
		case 0x08: nPRG = d; break;
		case 0x09: switch (d & 3) { case 0: mirrorMode = VERTICAL; break; case 1: mirrorMode = HORIZONTAL; break; case 2: mirrorMode = ONESCREEN_LO; break; case 3: mirrorMode = ONESCREEN_HI; break; } break;
		case 0x0A: bIRQEnable = (d & 1); bIRQActive = bIRQPending = false; break;
		case 0x0B: nIRQCounter = (nIRQCounter & 0xFF00) | d; break;
		case 0x0C: nIRQCounter = (nIRQCounter & 0x00FF) | (d << 8); break;
		case 0x0D: RunI2C(d); break;
		}
	}
	void RunI2C(uint8_t val) {
		bool sda = (val & 0x40), scl = (val & 0x20);
		if (i2c_scl && scl) {
			if (i2c_sda && !sda) { i2c_state = 1; i2c_bit = 0; i2c_shift = 0; i2c_out = true; }
			else if (!i2c_sda && sda) { i2c_state = 0; i2c_out = true; }
		}
		else if (!i2c_scl && scl && i2c_state == 1) {
			i2c_shift = (i2c_shift << 1) | (sda ? 1 : 0); i2c_bit++;
			if (i2c_bit == 9) { i2c_out = true; i2c_bit = 0; if (i2c_readMode) i2c_shift = eeprom_mem[i2c_ptr]; }
		}
		else if (i2c_scl && !scl && i2c_state == 1) {
			if (i2c_bit == 8) {
				bool ack = false; i2c_bit = 0;
				if ((i2c_shift & 0xF0) == 0xA0) {
					i2c_readMode = (i2c_shift & 1);
					if (i2c_readMode) i2c_shift = eeprom_mem[i2c_ptr]; else i2c_addrPhase = true;
					ack = true;
				}
				else if (!i2c_readMode) {
					if (i2c_addrPhase) { i2c_ptr = i2c_shift; i2c_addrPhase = false; ack = true; }
					else { eeprom_mem[i2c_ptr++] = i2c_shift; i2c_addrPhase = true; ack = true; }
				}
				else { i2c_ptr++; i2c_shift = eeprom_mem[i2c_ptr]; }
				i2c_out = !ack;
			}
			else {
				if (i2c_readMode) { i2c_out = (i2c_shift & 0x80); i2c_shift <<= 1; }
				else i2c_out = true;
			}
		}
		i2c_sda = sda; i2c_scl = scl;
	}
	void tick(int cycles) override {
		if (bIRQEnable && !bIRQPending) {
			if (nIRQCounter > 0) { if ((int)nIRQCounter - cycles <= 0) { nIRQCounter = 0; bIRQActive = bIRQPending = true; } else nIRQCounter -= cycles; }
		}
	}
	bool ppuMapRead(Word a, uint32_t& m) override {
		if (a < 0x2000) { m = (nCHRBanks == 0) ? a : (nCHR[a / 0x400] * 0x400 + (a & 0x3FF)); return true; } return false;
	}
	bool ppuMapWrite(Word a, uint32_t& m) override { return ppuMapRead(a, m); }
	MIRROR mirror() override { return mirrorMode; }
	bool irqState() override { return bIRQActive; }
	void irqClear() override { bIRQActive = false; }
};
class Cartridge {
public:
	std::vector<Byte> PRG, CHR; Byte mapperID = 0, prgBanks = 0, chrBanks = 0; bool verticalMirroring = false;
	std::shared_ptr<Mapper> pMapper; Bus* pBus = nullptr;
	Cartridge(Bus* bus) : pBus(bus) {}
	void Tick(int cycles) { if (pMapper) pMapper->tick(cycles); }
	bool Load(const std::wstring& path) {
		FILE* fp = NULL; _wfopen_s(&fp, path.c_str(), L"rb"); if (!fp) return false;
		uint8_t h[16]; if (fread(h, 1, 16, fp) != 16) { fclose(fp); return false; }
		if (h[0] != 'N' || h[1] != 'E' || h[2] != 'S' || h[3] != 0x1A) { fclose(fp); return false; }
		prgBanks = h[4]; chrBanks = h[5]; mapperID = ((h[6] >> 4) & 0x0F) | (h[7] & 0xF0); verticalMirroring = (h[6] & 1);
		if (h[6] & 4) fseek(fp, 512, SEEK_CUR);
		PRG.resize((size_t)prgBanks * 16384); fread(PRG.data(), 1, PRG.size(), fp);
		if (chrBanks == 0) { CHR.resize(8192); std::fill(CHR.begin(), CHR.end(), 0); }
		else { CHR.resize((size_t)chrBanks * 8192); fread(CHR.data(), 1, CHR.size(), fp); }
		fclose(fp);
		switch (mapperID) {
		case 0: pMapper = std::make_shared<Mapper_000>(prgBanks, chrBanks, pBus, PRG, CHR); break;
		case 1: pMapper = std::make_shared<Mapper_001>(prgBanks, chrBanks, pBus, PRG, CHR); break;
		case 2: pMapper = std::make_shared<Mapper_002>(prgBanks, chrBanks, pBus, PRG, CHR); break;
		case 3: pMapper = std::make_shared<Mapper_003>(prgBanks, chrBanks, pBus, PRG, CHR); break;
		case 4: pMapper = std::make_shared<Mapper_004>(prgBanks, chrBanks, pBus, PRG, CHR); break;
		case 16: pMapper = std::make_shared<Mapper_016>(prgBanks, chrBanks, pBus, PRG, CHR); break;
		default: return false;
		}
		if (pMapper) pMapper->Reset(); return true;
	}
	bool ppuRead(Word a, Byte& d) { uint32_t m = 0; if (pMapper && pMapper->ppuMapRead(a, m)) { d = CHR[m]; return true; } return false; }
	bool ppuWrite(Word a, Byte d) { uint32_t m = 0; if (pMapper && pMapper->ppuMapWrite(a, m)) { CHR[m] = d; return true; } return false; }
	bool GetIRQ() { return pMapper ? pMapper->irqState() : false; }
	void ClearIRQ() { if (pMapper) pMapper->irqClear(); }
	MIRROR GetMirror() { MIRROR m = pMapper ? pMapper->mirror() : HARDWARE; return (m == HARDWARE) ? (verticalMirroring ? VERTICAL : HORIZONTAL) : m; }
};
Bus::Bus(CPU& c, PPU& p, APU& a) : cpu(&c), ppu(&p), apu(&a) { InitializeMemoryMap(); }
void Bus::SetCart(Cartridge* c) { cart = c; InitializeMemoryMap(); if (cart && cart->pMapper) cart->pMapper->Reset(); }
void Bus::ClearMap() { for (int i = 0; i < 256; i++) { pageReadPtrs[i] = nullptr; pageReadHandlers[i] = [](Word) { return 0; }; pageWriteHandlers[i] = [](Word, Byte) {}; } }
void Bus::InitializeMemoryMap() {
	ClearMap();
	for (int p = 0; p < 32; p++) { MapReadPage(p, &ram[(p & 7) * 256]); MapWriteHandler(p, [this](Word a, Byte d) { ram[a & 0x7FF] = d; }); }
	for (int p = 0x20; p < 0x40; p++) { MapReadHandler(p, [this](Word a) { return ppu->cpuRead(a & 7); }); MapWriteHandler(p, [this](Word a, Byte d) { ppu->cpuWrite(a & 7, d); }); }
	MapReadHandler(0x40, [this](Word a) -> Byte {
		if (a == 0x4015) return apu->Read(a); if (a == 0x4016) { Byte d = (controller_state[0] & 1); controller_state[0] >>= 1; return d; } return 0;
		});
	MapWriteHandler(0x40, [this](Word a, Byte d) {
		if (a == 0x4014) { dma_page = d; dma_addr = 0; dma_transfer = true; }
		else if (a == 0x4016) { if (d & 1) controller_state[0] = controller[0]; }
		else if (a <= 0x4017) apu->Write(a, d);
		});
}
Byte Bus::cpuRead(Word addr) { Byte p = addr >> 8; if (pageReadPtrs[p]) return pageReadPtrs[p][addr & 0xFF]; return pageReadHandlers[p](addr); }
void Bus::cpuWrite(Word addr, Byte data) { pageWriteHandlers[addr >> 8](addr, data); }
PPU::PPU(Bus& b) : pBus(&b) { Reset(); }
void PPU::SetCart(Cartridge* cart) { pCart = cart; }
void PPU::SetScreenBuffer(uint32_t* buffer) { pScreenBuffer = buffer; }
void PPU::Reset() {
	ctrl.raw = 0; mask.raw = 0; status.raw = 0; oamAddr = 0; vAddr.raw = 0; tAddr.raw = 0; fineX = 0; writeToggle = false; readBuffer = 0;
	scanline = 0; cycle = 0; frameComplete = false; nmiOccurred = false;
	memset(nameTables, 0, sizeof(nameTables)); memset(paletteRam, 0, sizeof(paletteRam)); memset(primaryOAM, 0, sizeof(primaryOAM));
	currentMirrorMode = HARDWARE; UpdateNametables();
}
uint8_t PPU::cpuRead(uint16_t addr) {
	uint8_t data = 0;
	switch (addr) {
	case 0x0002: data = (status.raw & 0xE0) | (readBuffer & 0x1F); status.bits.vblank = 0; writeToggle = false; break;
	case 0x0004: data = ((uint8_t*)primaryOAM)[oamAddr]; break;
	case 0x0007: data = readBuffer; readBuffer = InternalRead(vAddr.raw); if (vAddr.raw >= 0x3F00) data = readBuffer; vAddr.raw += (ctrl.bits.vram_inc ? 32 : 1); break;
	} return data;
}
void PPU::cpuWrite(uint16_t addr, uint8_t data) {
	switch (addr) {
	case 0x0000: ctrl.raw = data; tAddr.bits.nametable_x = ctrl.bits.nametable_x; tAddr.bits.nametable_y = ctrl.bits.nametable_y; break;
	case 0x0001: mask.raw = data; break;
	case 0x0003: oamAddr = data; break;
	case 0x0004: ((uint8_t*)primaryOAM)[oamAddr] = data; oamAddr++; break;
	case 0x0005: if (!writeToggle) { fineX = data & 0x07; tAddr.bits.coarse_x = data >> 3; writeToggle = true; }
			   else { tAddr.bits.fine_y = data & 0x07; tAddr.bits.coarse_y = data >> 3; writeToggle = false; } break;
	case 0x0006: if (!writeToggle) { tAddr.raw = (tAddr.raw & 0x00FF) | ((uint16_t)(data & 0x3F) << 8); writeToggle = true; }
			   else { tAddr.raw = (tAddr.raw & 0xFF00) | data; vAddr.raw = tAddr.raw; writeToggle = false; } break;
	case 0x0007: InternalWrite(vAddr.raw, data); vAddr.raw += (ctrl.bits.vram_inc ? 32 : 1); break;
	}
}
uint8_t PPU::InternalRead(uint16_t addr) {
	addr &= 0x3FFF; uint8_t data = 0; if (pCart->ppuRead(addr, data)) return data;
	if (addr >= 0x2000 && addr <= 0x3EFF) { if (pCart->GetMirror() != currentMirrorMode) UpdateNametables(); return pNameTables[(addr >> 10) & 3][addr & 0x3FF]; }
	else if (addr >= 0x3F00 && addr <= 0x3FFF) {
		addr &= 0x001F; if (addr == 0x0010) addr = 0x0000; if (addr == 0x0014) addr = 0x0004; if (addr == 0x0018) addr = 0x0008; if (addr == 0x001C) addr = 0x000C;
		return paletteRam[addr] & (mask.bits.grayscale ? 0x30 : 0x3F);
	} return 0;
}
void PPU::InternalWrite(uint16_t addr, uint8_t data) {
	addr &= 0x3FFF; if (pCart->ppuWrite(addr, data)) return;
	if (addr >= 0x2000 && addr <= 0x3EFF) { if (pCart->GetMirror() != currentMirrorMode) UpdateNametables(); pNameTables[(addr >> 10) & 3][addr & 0x3FF] = data; }
	else if (addr >= 0x3F00 && addr <= 0x3FFF) {
		addr &= 0x001F; if (addr == 0x0010) addr = 0x0000; if (addr == 0x0014) addr = 0x0004; if (addr == 0x0018) addr = 0x0008; if (addr == 0x001C) addr = 0x000C;
		paletteRam[addr] = data;
	}
}
void PPU::RenderScanline() {
	if (!pScreenBuffer) return;
	int y = scanline; bool bgOpacity[256]; memset(bgOpacity, 0, 256);
	if (mask.bits.render_bg) {
		uint16_t coarseX = vAddr.bits.coarse_x, coarseY = vAddr.bits.coarse_y, nt = (vAddr.bits.nametable_y << 1) | vAddr.bits.nametable_x, fineY = vAddr.bits.fine_y;
		for (int col = 0; col < 256; col++) {
			if (col < 8 && !mask.bits.render_bg_left) { pScreenBuffer[y * 256 + col] = 0xFF000000; continue; }
			int sc_x = (coarseX << 3) + fineX + col, eff_nt = nt; if (sc_x >= 256) { sc_x -= 256; eff_nt ^= 1; } if (sc_x >= 256) { sc_x -= 256; eff_nt ^= 1; }
			int tile_col = sc_x / 8, tile_row = coarseY; uint16_t nt_base = 0x2000 + (eff_nt * 0x400);
			uint8_t tileID = InternalRead(nt_base + (tile_row * 32) + tile_col);
			uint8_t attrByte = InternalRead((nt_base + 0x3C0) + ((tile_row >> 2) << 3) + (tile_col >> 2));
			if (tile_row & 0x02) attrByte >>= 4; if (tile_col & 0x02) attrByte >>= 2;
			uint16_t patternAddr = (ctrl.bits.bg_pattern << 12) + ((uint16_t)tileID << 4) + fineY;
			uint8_t pLo = InternalRead(patternAddr), pHi = InternalRead(patternAddr + 8);
			int pixelBit = 7 - (sc_x % 8); uint8_t pixelVal = ((pLo >> pixelBit) & 1) | (((pHi >> pixelBit) & 1) << 1);
			if (pixelVal != 0) { bgOpacity[col] = true; pScreenBuffer[y * 256 + col] = SYSTEM_PALETTE[InternalRead(0x3F00 + ((attrByte & 3) << 2) + pixelVal) & 0x3F]; }
			else { pScreenBuffer[y * 256 + col] = SYSTEM_PALETTE[InternalRead(0x3F00) & 0x3F]; }
		}
	}
	if (mask.bits.render_spr) {
		int spriteHeight = ctrl.bits.sprite_size ? 16 : 8;
		for (int i = 63; i >= 0; i--) {
			OAM_ENTRY& spr = primaryOAM[i]; int sprY = spr.y + 1;
			if (y >= sprY && y < sprY + spriteHeight) {
				int row = y - sprY; if (spr.attr.flip_v) row = (spriteHeight - 1) - row;
				uint16_t addrPattern;
				if (ctrl.bits.sprite_size == 0) addrPattern = (ctrl.bits.sprite_pattern << 12) | (spr.tile_id << 4) | (row & 7);
				else { uint8_t table = spr.tile_id & 1, tile = spr.tile_id & 0xFE; if (row >= 8) tile++; addrPattern = (table << 12) | (tile << 4) | (row & 7); }
				uint8_t pLo = InternalRead(addrPattern), pHi = InternalRead(addrPattern + 8);
				for (int cx = 0; cx < 8; cx++) {
					int px = spr.x + cx; if (px < 0 || px >= 256) continue; if (px < 8 && !mask.bits.render_spr_left) continue;
					int bit = spr.attr.flip_h ? cx : (7 - cx); uint8_t pixelVal = ((pLo >> bit) & 1) | (((pHi >> bit) & 1) << 1);
					if (pixelVal != 0) {
						if (i == 0 && mask.bits.render_bg && mask.bits.render_spr && bgOpacity[px] && px != 255) status.bits.sprite_zero_hit = 1;
						if (!spr.attr.priority || !bgOpacity[px]) { pScreenBuffer[y * 256 + px] = SYSTEM_PALETTE[InternalRead(0x3F10 + (spr.attr.palette << 2) + pixelVal) & 0x3F]; }
					}
				}
			}
		}
	}
}
void PPU::Clock() {
	if (scanline >= 0 && scanline < 240) { if (cycle == 256) RenderScanline(); }
	if (scanline == 241 && cycle == 1) { status.bits.vblank = 1; if (ctrl.bits.nmi_enable) nmiOccurred = true; }
	if (scanline == 261 && cycle == 1) { status.bits.vblank = 0; status.bits.sprite_zero_hit = 0; status.bits.sprite_overflow = 0; frameComplete = true; }
	if ((scanline >= 0 && scanline < 240) || scanline == 261) {
		if (mask.bits.render_bg || mask.bits.render_spr) {
			if (cycle == 256) {
				if (vAddr.bits.fine_y < 7) vAddr.bits.fine_y++; else { vAddr.bits.fine_y = 0; uint8_t y = vAddr.bits.coarse_y; if (y == 29) { y = 0; vAddr.bits.nametable_y ^= 1; } else if (y == 31) y = 0; else y++; vAddr.bits.coarse_y = y; }
			}
			if (cycle == 257) { vAddr.bits.nametable_x = tAddr.bits.nametable_x; vAddr.bits.coarse_x = tAddr.bits.coarse_x; }
		}
	}
	if (scanline == 261 && (cycle >= 280 && cycle <= 304)) { if (mask.bits.render_bg || mask.bits.render_spr) { vAddr.bits.fine_y = tAddr.bits.fine_y; vAddr.bits.nametable_y = tAddr.bits.nametable_y; vAddr.bits.coarse_y = tAddr.bits.coarse_y; } }
	cycle++; if (cycle >= 341) { cycle = 0; scanline++; if (scanline >= 262) { scanline = 0; frameComplete = false; } }
}
void PPU::UpdateNametables() {
	MIRROR m = pCart ? pCart->GetMirror() : VERTICAL; currentMirrorMode = m;
	switch (m) {
	case HORIZONTAL: pNameTables[0] = nameTables[0]; pNameTables[1] = nameTables[0]; pNameTables[2] = nameTables[1]; pNameTables[3] = nameTables[1]; break;
	case VERTICAL: pNameTables[0] = nameTables[0]; pNameTables[1] = nameTables[1]; pNameTables[2] = nameTables[0]; pNameTables[3] = nameTables[1]; break;
	case ONESCREEN_LO: pNameTables[0] = pNameTables[1] = pNameTables[2] = pNameTables[3] = nameTables[0]; break;
	case ONESCREEN_HI: pNameTables[0] = pNameTables[1] = pNameTables[2] = pNameTables[3] = nameTables[1]; break;
	default: pNameTables[0] = nameTables[0]; pNameTables[1] = nameTables[1]; pNameTables[2] = nameTables[0]; pNameTables[3] = nameTables[1]; break;
	}
}
CPU::CPU(Bus& b) : bus(&b) {}
uint8_t CPU::Read(uint16_t addr) { return bus->cpuRead(addr); }
void CPU::Write(uint16_t addr, uint8_t data) { bus->cpuWrite(addr, data); }
void CPU::Push(uint8_t v) { Write(0x100 + S, v); S--; }
uint8_t CPU::Pop() { S++; return Read(0x100 + S); }
void CPU::SetFlag(FLAGS f, bool v) { if (v) P |= f; else P &= ~f; }
void CPU::UpdateZN(uint8_t v) { SetFlag(Z, v == 0); SetFlag(N, v & 0x80); }
void CPU::Reset() { A = 0; X = 0; Y = 0; S = 0xFD; P = 0 | U | I; uint16_t lo = Read(0xFFFC), hi = Read(0xFFFD); PC = (hi << 8) | lo; cyclesLeft = 0; }
void CPU::IRQ() { if (!(P & I)) { Push((PC >> 8) & 0xFF); Push(PC & 0xFF); Push((P & ~B) | U); SetFlag(I, true); uint16_t lo = Read(0xFFFE), hi = Read(0xFFFF); PC = (hi << 8) | lo; cyclesLeft -= 7; } }
void CPU::NMI() { Push((PC >> 8) & 0xFF); Push(PC & 0xFF); Push((P & ~B) | U); SetFlag(I, true); uint16_t lo = Read(0xFFFA), hi = Read(0xFFFB); PC = (hi << 8) | lo; cyclesLeft -= 8; }
uint16_t CPU::GetAddr_ZP() { return Read(PC++); }
uint16_t CPU::GetAddr_ZPX() { return (Read(PC++) + X) & 0xFF; }
uint16_t CPU::GetAddr_ZPY() { return (Read(PC++) + Y) & 0xFF; }
uint16_t CPU::GetAddr_ABS() { uint16_t lo = Read(PC++); uint16_t hi = Read(PC++); return (hi << 8) | lo; }
uint16_t CPU::GetAddr_ABX(bool& p) { uint16_t base = GetAddr_ABS(); uint16_t addr = base + X; if (CheckPageCross(base, addr)) p = true; return addr; }
uint16_t CPU::GetAddr_ABY(bool& p) { uint16_t base = GetAddr_ABS(); uint16_t addr = base + Y; if (CheckPageCross(base, addr)) p = true; return addr; }
uint16_t CPU::GetAddr_IZX() { uint16_t t = Read(PC++), lo = Read((uint16_t)(t + X) & 0xFF), hi = Read((uint16_t)(t + X + 1) & 0xFF); return (hi << 8) | lo; }
uint16_t CPU::GetAddr_IZY(bool& p) { uint16_t t = Read(PC++), lo = Read(t & 0xFF), hi = Read((t + 1) & 0xFF), base = (hi << 8) | lo, addr = base + Y; if (CheckPageCross(base, addr)) p = true; return addr; }
void CPU::Op_ADC(uint8_t v) { uint16_t t = (uint16_t)A + v + (P & C ? 1 : 0); SetFlag(C, t > 255); SetFlag(Z, (t & 0xFF) == 0); SetFlag(N, t & 0x80); SetFlag(V, (~((uint16_t)A ^ (uint16_t)v) & ((uint16_t)A ^ t)) & 0x0080); A = (uint8_t)t; }
void CPU::Op_SBC(uint8_t v) { Op_ADC(v ^ 0xFF); }
void CPU::Op_AND(uint8_t v) { A &= v; UpdateZN(A); }
void CPU::Op_ORA(uint8_t v) { A |= v; UpdateZN(A); }
void CPU::Op_EOR(uint8_t v) { A ^= v; UpdateZN(A); }
void CPU::Op_CMP(uint8_t reg, uint8_t v) { uint16_t t = (uint16_t)reg - v; SetFlag(C, reg >= v); SetFlag(Z, (t & 0xFF) == 0); SetFlag(N, t & 0x80); }
void CPU::Op_BIT(uint8_t v) { SetFlag(Z, (A & v) == 0); SetFlag(N, v & 0x80); SetFlag(V, v & 0x40); }
void CPU::Op_INC(uint16_t addr) { uint8_t v = Read(addr); v++; Write(addr, v); UpdateZN(v); }
void CPU::Op_DEC(uint16_t addr) { uint8_t v = Read(addr); v--; Write(addr, v); UpdateZN(v); }
void CPU::Op_ASL(uint16_t addr) { uint8_t v = Read(addr); SetFlag(C, v & 0x80); v <<= 1; Write(addr, v); UpdateZN(v); }
void CPU::Op_LSR(uint16_t addr) { uint8_t v = Read(addr); SetFlag(C, v & 1); v >>= 1; Write(addr, v); UpdateZN(v); }
void CPU::Op_ROL(uint16_t addr) { uint8_t v = Read(addr); uint16_t t = ((uint16_t)v << 1) | (P & C ? 1 : 0); SetFlag(C, t > 255); v = (uint8_t)t; Write(addr, v); UpdateZN(v); }
void CPU::Op_ROR(uint16_t addr) { uint8_t v = Read(addr); uint16_t t = ((uint16_t)v >> 1) | (P & C ? 0x80 : 0); SetFlag(C, v & 1); v = (uint8_t)t; Write(addr, v); UpdateZN(v); }
void CPU::Op_ASL_A() { SetFlag(C, A & 0x80); A <<= 1; UpdateZN(A); }
void CPU::Op_LSR_A() { SetFlag(C, A & 1); A >>= 1; UpdateZN(A); }
void CPU::Op_ROL_A() { uint16_t t = ((uint16_t)A << 1) | (P & C ? 1 : 0); SetFlag(C, t > 255); A = (uint8_t)t; UpdateZN(A); }
void CPU::Op_ROR_A() { uint16_t t = ((uint16_t)A >> 1) | (P & C ? 0x80 : 0); SetFlag(C, A & 1); A = (uint8_t)t; UpdateZN(A); }
void CPU::Op_Branch(bool condition) { int8_t off = (int8_t)Read(PC++); if (condition) { cyclesLeft--; uint16_t newPC = PC + off; if ((PC & 0xFF00) != (newPC & 0xFF00)) cyclesLeft--; PC = newPC; } cyclesLeft -= 2; }
void CPU::Op_LAX(uint8_t v) { A = X = v; UpdateZN(A); } void CPU::Op_SAX(uint16_t addr) { Write(addr, A & X); }
void CPU::Op_DCP(uint16_t addr) { uint8_t v = Read(addr) - 1; Write(addr, v); Op_CMP(A, v); }
void CPU::Op_ISB(uint16_t addr) { uint8_t v = Read(addr) + 1; Write(addr, v); Op_SBC(v); }
void CPU::Op_SLO(uint16_t addr) { Op_ASL(addr); A |= Read(addr); UpdateZN(A); } void CPU::Op_RLA(uint16_t addr) { Op_ROL(addr); A &= Read(addr); UpdateZN(A); }
void CPU::Op_SRE(uint16_t addr) { Op_LSR(addr); A ^= Read(addr); UpdateZN(A); } void CPU::Op_RRA(uint16_t addr) { Op_ROR(addr); Op_ADC(Read(addr)); }
void CPU::Run(int cyclesToRun) {
	cyclesLeft += cyclesToRun;
	while (cyclesLeft > 0) {
		uint8_t op = Read(PC++); bool p = false;
		switch (op) {
		case 0xA9: A = Read(PC++); UpdateZN(A); cyclesLeft -= 2; break; case 0xA5: A = Read(GetAddr_ZP()); UpdateZN(A); cyclesLeft -= 3; break; case 0xB5: A = Read(GetAddr_ZPX()); UpdateZN(A); cyclesLeft -= 4; break; case 0xAD: A = Read(GetAddr_ABS()); UpdateZN(A); cyclesLeft -= 4; break; case 0xBD: A = Read(GetAddr_ABX(p)); UpdateZN(A); cyclesLeft -= 4; if (p)cyclesLeft--; break; case 0xB9: A = Read(GetAddr_ABY(p)); UpdateZN(A); cyclesLeft -= 4; if (p)cyclesLeft--; break; case 0xA1: A = Read(GetAddr_IZX()); UpdateZN(A); cyclesLeft -= 6; break; case 0xB1: A = Read(GetAddr_IZY(p)); UpdateZN(A); cyclesLeft -= 5; if (p)cyclesLeft--; break;
		case 0xA2: X = Read(PC++); UpdateZN(X); cyclesLeft -= 2; break; case 0xA6: X = Read(GetAddr_ZP()); UpdateZN(X); cyclesLeft -= 3; break; case 0xB6: X = Read(GetAddr_ZPY()); UpdateZN(X); cyclesLeft -= 4; break; case 0xAE: X = Read(GetAddr_ABS()); UpdateZN(X); cyclesLeft -= 4; break; case 0xBE: X = Read(GetAddr_ABY(p)); UpdateZN(X); cyclesLeft -= 4; if (p)cyclesLeft--; break;
		case 0xA0: Y = Read(PC++); UpdateZN(Y); cyclesLeft -= 2; break; case 0xA4: Y = Read(GetAddr_ZP()); UpdateZN(Y); cyclesLeft -= 3; break; case 0xB4: Y = Read(GetAddr_ZPX()); UpdateZN(Y); cyclesLeft -= 4; break; case 0xAC: Y = Read(GetAddr_ABS()); UpdateZN(Y); cyclesLeft -= 4; break; case 0xBC: Y = Read(GetAddr_ABX(p)); UpdateZN(Y); cyclesLeft -= 4; if (p)cyclesLeft--; break;
		case 0x85: Write(GetAddr_ZP(), A); cyclesLeft -= 3; break; case 0x95: Write(GetAddr_ZPX(), A); cyclesLeft -= 4; break; case 0x8D: Write(GetAddr_ABS(), A); cyclesLeft -= 4; break; case 0x9D: Write(GetAddr_ABX(p), A); cyclesLeft -= 5; break; case 0x99: Write(GetAddr_ABY(p), A); cyclesLeft -= 5; break; case 0x81: Write(GetAddr_IZX(), A); cyclesLeft -= 6; break; case 0x91: Write(GetAddr_IZY(p), A); cyclesLeft -= 6; break;
		case 0x86: Write(GetAddr_ZP(), X); cyclesLeft -= 3; break; case 0x96: Write(GetAddr_ZPY(), X); cyclesLeft -= 4; break; case 0x8E: Write(GetAddr_ABS(), X); cyclesLeft -= 4; break;
		case 0x84: Write(GetAddr_ZP(), Y); cyclesLeft -= 3; break; case 0x94: Write(GetAddr_ZPX(), Y); cyclesLeft -= 4; break; case 0x8C: Write(GetAddr_ABS(), Y); cyclesLeft -= 4; break;
		case 0xAA: X = A; UpdateZN(X); cyclesLeft -= 2; break; case 0xA8: Y = A; UpdateZN(Y); cyclesLeft -= 2; break; case 0x8A: A = X; UpdateZN(A); cyclesLeft -= 2; break; case 0x98: A = Y; UpdateZN(A); cyclesLeft -= 2; break; case 0x9A: S = X; cyclesLeft -= 2; break; case 0xBA: X = S; UpdateZN(X); cyclesLeft -= 2; break;
		case 0x69: Op_ADC(Read(PC++)); cyclesLeft -= 2; break; case 0x65: Op_ADC(Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0x75: Op_ADC(Read(GetAddr_ZPX())); cyclesLeft -= 4; break; case 0x6D: Op_ADC(Read(GetAddr_ABS())); cyclesLeft -= 4; break; case 0x7D: Op_ADC(Read(GetAddr_ABX(p))); cyclesLeft -= 4; if (p)cyclesLeft--; break; case 0x79: Op_ADC(Read(GetAddr_ABY(p))); cyclesLeft -= 4; if (p)cyclesLeft--; break; case 0x61: Op_ADC(Read(GetAddr_IZX())); cyclesLeft -= 6; break; case 0x71: Op_ADC(Read(GetAddr_IZY(p))); cyclesLeft -= 5; if (p)cyclesLeft--; break;
		case 0xE9: Op_SBC(Read(PC++)); cyclesLeft -= 2; break; case 0xE5: Op_SBC(Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0xF5: Op_SBC(Read(GetAddr_ZPX())); cyclesLeft -= 4; break; case 0xED: Op_SBC(Read(GetAddr_ABS())); cyclesLeft -= 4; break; case 0xFD: Op_SBC(Read(GetAddr_ABX(p))); cyclesLeft -= 4; if (p)cyclesLeft--; break; case 0xF9: Op_SBC(Read(GetAddr_ABY(p))); cyclesLeft -= 4; if (p)cyclesLeft--; break; case 0xE1: Op_SBC(Read(GetAddr_IZX())); cyclesLeft -= 6; break; case 0xF1: Op_SBC(Read(GetAddr_IZY(p))); cyclesLeft -= 5; if (p)cyclesLeft--; break;
		case 0x29: Op_AND(Read(PC++)); cyclesLeft -= 2; break; case 0x25: Op_AND(Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0x35: Op_AND(Read(GetAddr_ZPX())); cyclesLeft -= 4; break; case 0x2D: Op_AND(Read(GetAddr_ABS())); cyclesLeft -= 4; break; case 0x3D: Op_AND(Read(GetAddr_ABX(p))); cyclesLeft -= 4; if (p)cyclesLeft--; break; case 0x39: Op_AND(Read(GetAddr_ABY(p))); cyclesLeft -= 4; if (p)cyclesLeft--; break; case 0x21: Op_AND(Read(GetAddr_IZX())); cyclesLeft -= 6; break; case 0x31: Op_AND(Read(GetAddr_IZY(p))); cyclesLeft -= 5; if (p)cyclesLeft--; break;
		case 0x09: Op_ORA(Read(PC++)); cyclesLeft -= 2; break; case 0x05: Op_ORA(Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0x15: Op_ORA(Read(GetAddr_ZPX())); cyclesLeft -= 4; break; case 0x0D: Op_ORA(Read(GetAddr_ABS())); cyclesLeft -= 4; break; case 0x1D: Op_ORA(Read(GetAddr_ABX(p))); cyclesLeft -= 4; if (p)cyclesLeft--; break; case 0x19: Op_ORA(Read(GetAddr_ABY(p))); cyclesLeft -= 4; if (p)cyclesLeft--; break; case 0x01: Op_ORA(Read(GetAddr_IZX())); cyclesLeft -= 6; break; case 0x11: Op_ORA(Read(GetAddr_IZY(p))); cyclesLeft -= 5; if (p)cyclesLeft--; break;
		case 0x49: Op_EOR(Read(PC++)); cyclesLeft -= 2; break; case 0x45: Op_EOR(Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0x55: Op_EOR(Read(GetAddr_ZPX())); cyclesLeft -= 4; break; case 0x4D: Op_EOR(Read(GetAddr_ABS())); cyclesLeft -= 4; break; case 0x5D: Op_EOR(Read(GetAddr_ABX(p))); cyclesLeft -= 4; if (p)cyclesLeft--; break; case 0x59: Op_EOR(Read(GetAddr_ABY(p))); cyclesLeft -= 4; if (p)cyclesLeft--; break; case 0x41: Op_EOR(Read(GetAddr_IZX())); cyclesLeft -= 6; break; case 0x51: Op_EOR(Read(GetAddr_IZY(p))); cyclesLeft -= 5; if (p)cyclesLeft--; break;
		case 0xC9: Op_CMP(A, Read(PC++)); cyclesLeft -= 2; break; case 0xC5: Op_CMP(A, Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0xD5: Op_CMP(A, Read(GetAddr_ZPX())); cyclesLeft -= 4; break; case 0xCD: Op_CMP(A, Read(GetAddr_ABS())); cyclesLeft -= 4; break; case 0xDD: Op_CMP(A, Read(GetAddr_ABX(p))); cyclesLeft -= 4; if (p)cyclesLeft--; break; case 0xD9: Op_CMP(A, Read(GetAddr_ABY(p))); cyclesLeft -= 4; if (p)cyclesLeft--; break; case 0xC1: Op_CMP(A, Read(GetAddr_IZX())); cyclesLeft -= 6; break; case 0xD1: Op_CMP(A, Read(GetAddr_IZY(p))); cyclesLeft -= 5; if (p)cyclesLeft--; break;
		case 0xE0: Op_CMP(X, Read(PC++)); cyclesLeft -= 2; break; case 0xE4: Op_CMP(X, Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0xEC: Op_CMP(X, Read(GetAddr_ABS())); cyclesLeft -= 4; break; case 0xC0: Op_CMP(Y, Read(PC++)); cyclesLeft -= 2; break; case 0xC4: Op_CMP(Y, Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0xCC: Op_CMP(Y, Read(GetAddr_ABS())); cyclesLeft -= 4; break;
		case 0x24: Op_BIT(Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0x2C: Op_BIT(Read(GetAddr_ABS())); cyclesLeft -= 4; break;
		case 0xE6: Op_INC(GetAddr_ZP()); cyclesLeft -= 5; break; case 0xF6: Op_INC(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0xEE: Op_INC(GetAddr_ABS()); cyclesLeft -= 6; break; case 0xFE: Op_INC(GetAddr_ABX(p)); cyclesLeft -= 7; break; case 0xE8: X++; UpdateZN(X); cyclesLeft -= 2; break; case 0xC8: Y++; UpdateZN(Y); cyclesLeft -= 2; break;
		case 0xC6: Op_DEC(GetAddr_ZP()); cyclesLeft -= 5; break; case 0xD6: Op_DEC(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0xCE: Op_DEC(GetAddr_ABS()); cyclesLeft -= 6; break; case 0xDE: Op_DEC(GetAddr_ABX(p)); cyclesLeft -= 7; break; case 0xCA: X--; UpdateZN(X); cyclesLeft -= 2; break; case 0x88: Y--; UpdateZN(Y); cyclesLeft -= 2; break;
		case 0x0A: Op_ASL_A(); cyclesLeft -= 2; break; case 0x06: Op_ASL(GetAddr_ZP()); cyclesLeft -= 5; break; case 0x16: Op_ASL(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0x0E: Op_ASL(GetAddr_ABS()); cyclesLeft -= 6; break; case 0x1E: Op_ASL(GetAddr_ABX(p)); cyclesLeft -= 7; break;
		case 0x4A: Op_LSR_A(); cyclesLeft -= 2; break; case 0x46: Op_LSR(GetAddr_ZP()); cyclesLeft -= 5; break; case 0x56: Op_LSR(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0x4E: Op_LSR(GetAddr_ABS()); cyclesLeft -= 6; break; case 0x5E: Op_LSR(GetAddr_ABX(p)); cyclesLeft -= 7; break;
		case 0x2A: Op_ROL_A(); cyclesLeft -= 2; break; case 0x26: Op_ROL(GetAddr_ZP()); cyclesLeft -= 5; break; case 0x36: Op_ROL(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0x2E: Op_ROL(GetAddr_ABS()); cyclesLeft -= 6; break; case 0x3E: Op_ROL(GetAddr_ABX(p)); cyclesLeft -= 7; break;
		case 0x6A: Op_ROR_A(); cyclesLeft -= 2; break; case 0x66: Op_ROR(GetAddr_ZP()); cyclesLeft -= 5; break; case 0x76: Op_ROR(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0x6E: Op_ROR(GetAddr_ABS()); cyclesLeft -= 6; break; case 0x7E: Op_ROR(GetAddr_ABX(p)); cyclesLeft -= 7; break;
		case 0x10: Op_Branch(!(P & N)); break; case 0x30: Op_Branch(!!(P & N)); break; case 0x50: Op_Branch(!(P & V)); break; case 0x70: Op_Branch(!!(P & V)); break; case 0x90: Op_Branch(!(P & C)); break; case 0xB0: Op_Branch(!!(P & C)); break; case 0xD0: Op_Branch(!(P & Z)); break; case 0xF0: Op_Branch(!!(P & Z)); break;
		case 0x4C: PC = GetAddr_ABS(); cyclesLeft -= 3; break; case 0x6C: { uint16_t a = GetAddr_ABS(), l = Read(a), h = Read((a & 0xFF00) | ((a + 1) & 0x00FF)); PC = (h << 8) | l; cyclesLeft -= 5; } break;
		case 0x20: { uint16_t t = GetAddr_ABS(); Push(((PC - 1) >> 8) & 0xFF); Push((PC - 1) & 0xFF); PC = t; cyclesLeft -= 6; } break; case 0x60: { uint16_t l = Pop(), h = Pop(); PC = ((h << 8) | l) + 1; cyclesLeft -= 6; } break;
		case 0x00: { PC++; Push((PC >> 8) & 0xFF); Push(PC & 0xFF); Push(P | U | B); SetFlag(I, true); PC = (Read(0xFFFF) << 8) | Read(0xFFFE); cyclesLeft -= 7; } break; case 0x40: { P = Pop(); SetFlag(U, true); uint16_t l = Pop(), h = Pop(); PC = (h << 8) | l; cyclesLeft -= 6; } break;
		case 0x48: Push(A); cyclesLeft -= 3; break; case 0x08: Push(P | B | U); cyclesLeft -= 3; break; case 0x68: A = Pop(); UpdateZN(A); cyclesLeft -= 4; break; case 0x28: P = Pop(); SetFlag(U, true); cyclesLeft -= 4; break;
		case 0x18: SetFlag(C, false); cyclesLeft -= 2; break; case 0x38: SetFlag(C, true); cyclesLeft -= 2; break; case 0x58: SetFlag(I, false); cyclesLeft -= 2; break; case 0x78: SetFlag(I, true); cyclesLeft -= 2; break; case 0xB8: SetFlag(V, false); cyclesLeft -= 2; break; case 0xD8: SetFlag(D, false); cyclesLeft -= 2; break; case 0xF8: SetFlag(D, true); cyclesLeft -= 2; break; case 0xEA: cyclesLeft -= 2; break;
		case 0xA7: Op_LAX(Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0xB7: Op_LAX(Read(GetAddr_ZPY())); cyclesLeft -= 4; break; case 0xAF: Op_LAX(Read(GetAddr_ABS())); cyclesLeft -= 4; break; case 0xBF: Op_LAX(Read(GetAddr_ABY(p))); cyclesLeft -= 4; if (p)cyclesLeft--; break; case 0xA3: Op_LAX(Read(GetAddr_IZX())); cyclesLeft -= 6; break; case 0xB3: Op_LAX(Read(GetAddr_IZY(p))); cyclesLeft -= 5; if (p)cyclesLeft--; break;
		case 0x87: Op_SAX(GetAddr_ZP()); cyclesLeft -= 3; break; case 0x97: Op_SAX(GetAddr_ZPY()); cyclesLeft -= 4; break; case 0x8F: Op_SAX(GetAddr_ABS()); cyclesLeft -= 4; break; case 0x83: Op_SAX(GetAddr_IZX()); cyclesLeft -= 6; break;
		case 0xC7: Op_DCP(GetAddr_ZP()); cyclesLeft -= 5; break; case 0xD7: Op_DCP(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0xCF: Op_DCP(GetAddr_ABS()); cyclesLeft -= 6; break; case 0xDF: Op_DCP(GetAddr_ABX(p)); cyclesLeft -= 7; break; case 0xDB: Op_DCP(GetAddr_ABY(p)); cyclesLeft -= 7; break; case 0xC3: Op_DCP(GetAddr_IZX()); cyclesLeft -= 8; break; case 0xD3: Op_DCP(GetAddr_IZY(p)); cyclesLeft -= 8; break;
		case 0xE7: Op_ISB(GetAddr_ZP()); cyclesLeft -= 5; break; case 0xF7: Op_ISB(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0xEF: Op_ISB(GetAddr_ABS()); cyclesLeft -= 6; break; case 0xFF: Op_ISB(GetAddr_ABX(p)); cyclesLeft -= 7; break; case 0xFB: Op_ISB(GetAddr_ABY(p)); cyclesLeft -= 7; break; case 0xE3: Op_ISB(GetAddr_IZX()); cyclesLeft -= 8; break; case 0xF3: Op_ISB(GetAddr_IZY(p)); cyclesLeft -= 8; break;
		case 0x07: Op_SLO(GetAddr_ZP()); cyclesLeft -= 5; break; case 0x17: Op_SLO(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0x0F: Op_SLO(GetAddr_ABS()); cyclesLeft -= 6; break; case 0x1F: Op_SLO(GetAddr_ABX(p)); cyclesLeft -= 7; break; case 0x1B: Op_SLO(GetAddr_ABY(p)); cyclesLeft -= 7; break; case 0x03: Op_SLO(GetAddr_IZX()); cyclesLeft -= 8; break; case 0x13: Op_SLO(GetAddr_IZY(p)); cyclesLeft -= 8; break;
		case 0x27: Op_RLA(GetAddr_ZP()); cyclesLeft -= 5; break; case 0x37: Op_RLA(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0x2F: Op_RLA(GetAddr_ABS()); cyclesLeft -= 6; break; case 0x3F: Op_RLA(GetAddr_ABX(p)); cyclesLeft -= 7; break; case 0x3B: Op_RLA(GetAddr_ABY(p)); cyclesLeft -= 7; break; case 0x23: Op_RLA(GetAddr_IZX()); cyclesLeft -= 8; break; case 0x33: Op_RLA(GetAddr_IZY(p)); cyclesLeft -= 8; break;
		case 0x47: Op_SRE(GetAddr_ZP()); cyclesLeft -= 5; break; case 0x57: Op_SRE(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0x4F: Op_SRE(GetAddr_ABS()); cyclesLeft -= 6; break; case 0x5F: Op_SRE(GetAddr_ABX(p)); cyclesLeft -= 7; break; case 0x5B: Op_SRE(GetAddr_ABY(p)); cyclesLeft -= 7; break; case 0x43: Op_SRE(GetAddr_IZX()); cyclesLeft -= 8; break; case 0x53: Op_SRE(GetAddr_IZY(p)); cyclesLeft -= 8; break;
		case 0x67: Op_RRA(GetAddr_ZP()); cyclesLeft -= 5; break; case 0x77: Op_RRA(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0x6F: Op_RRA(GetAddr_ABS()); cyclesLeft -= 6; break; case 0x7F: Op_RRA(GetAddr_ABX(p)); cyclesLeft -= 7; break; case 0x7B: Op_RRA(GetAddr_ABY(p)); cyclesLeft -= 7; break; case 0x63: Op_RRA(GetAddr_IZX()); cyclesLeft -= 8; break; case 0x73: Op_RRA(GetAddr_IZY(p)); cyclesLeft -= 8; break;
		case 0x1A: case 0x3A: case 0x5A: case 0x7A: case 0xDA: case 0xFA: cyclesLeft -= 2; break; case 0x80: case 0x82: case 0x89: case 0xC2: case 0xE2: Read(PC++); cyclesLeft -= 2; break; case 0x04: case 0x44: case 0x64: GetAddr_ZP(); cyclesLeft -= 3; break; case 0x14: case 0x34: case 0x54: case 0x74: case 0xD4: case 0xF4: GetAddr_ZPX(); cyclesLeft -= 4; break; case 0x0C: GetAddr_ABS(); cyclesLeft -= 4; break; case 0x1C: case 0x3C: case 0x5C: case 0x7C: case 0xDC: case 0xFC: GetAddr_ABX(p); cyclesLeft -= 4; if (p)cyclesLeft--; break; case 0xEB: Op_SBC(Read(PC++)); cyclesLeft -= 2; break;
		default: cyclesLeft -= 2; break;
		}
	}
}
class NESCore {
public:
	CPU cpu; PPU ppu; APU apu; Bus bus; std::shared_ptr<Cartridge> cart; std::vector<uint32_t> displayBuffer; bool isLoaded = false;
	NESCore() : cpu(bus), ppu(bus), apu(bus), bus(cpu, ppu, apu) { displayBuffer.resize(NES_WIDTH * NES_HEIGHT); ppu.SetScreenBuffer(displayBuffer.data()); }
	bool LoadRom(const std::wstring& path) { cart = std::make_shared<Cartridge>(&bus); if (!cart->Load(path)) return false; bus.SetCart(cart.get()); ppu.SetCart(cart.get()); cpu.Reset(); ppu.Reset(); apu.Reset(); return (isLoaded = true); }
	void InputKey(int id, bool pressed) { if (id >= 0 && id < 8) { if (pressed) bus.controller[0] |= (1 << id); else bus.controller[0] &= ~(1 << id); } }
	void StepFrame() {
		if (!isLoaded) return;
		const int CPU_CYCLES = 29780; int cycles = 0, step = 1;
		while (cycles < CPU_CYCLES) {
			for (int i = 0; i < step * 3; i++) { ppu.Clock(); if (ppu.nmiOccurred) { ppu.nmiOccurred = false; cpu.NMI(); } }
			cart->Tick(step); if (cart->GetIRQ()) cpu.IRQ();
			if (bus.dma_transfer) {
				bus.dma_transfer = false; uint16_t off = (uint16_t)bus.dma_page << 8;
				for (int i = 0; i < 256; i++) ((uint8_t*)ppu.primaryOAM)[(bus.dma_addr + i) & 0xFF] = bus.cpuRead(off + i);
				for (int i = 0; i < 513 * 3; i++) { ppu.Clock(); if (ppu.nmiOccurred) { ppu.nmiOccurred = false; cpu.NMI(); } }
				cycles += 513; cart->Tick(513);
			}
			else { cpu.Run(step); cycles += step; }
		} ppu.frameComplete = false;
	}
};
class App {
	HWND m_hwnd; ID2D1Factory* m_pD2DF; ID2D1HwndRenderTarget* m_pRT; ID2D1Bitmap* m_pBmp; NESCore m_nes; AudioDriver m_aud; BOOL m_fs; WINDOWPLACEMENT m_wp; HMENU m_hm;
public:
	App() : m_hwnd(0), m_pD2DF(0), m_pRT(0), m_pBmp(0), m_fs(0), m_hm(0) { ZeroMemory(&m_wp, sizeof(m_wp)); }
	~App() { SafeRelease(&m_pBmp); SafeRelease(&m_pRT); SafeRelease(&m_pD2DF); }
	HRESULT Initialize(HINSTANCE hI, int nC) {
		D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DF);
		WNDCLASSEX wc = { sizeof(wc), CS_HREDRAW | CS_VREDRAW, App::WndProc, 0, sizeof(LONG_PTR), hI, 0, LoadCursor(0, IDC_ARROW), (HBRUSH)GetStockObject(BLACK_BRUSH), 0, L"D2DNESWnd" }; RegisterClassEx(&wc);
		HMENU hM = CreateMenu(), hS = CreatePopupMenu(); AppendMenu(hS, MF_STRING, IDM_FILE_OPEN, L"Open..."); AppendMenu(hS, MF_STRING, IDM_FILE_FULLSCREEN, L"Fullscreen\tF11"); AppendMenu(hS, MF_SEPARATOR, 0, 0); AppendMenu(hS, MF_STRING, IDM_FILE_EXIT, L"Exit"); AppendMenu(hM, MF_STRING | MF_POPUP, (UINT_PTR)hS, L"File");
		int w = NES_WIDTH * 3, h = NES_HEIGHT * 3; RECT rc = { 0, 0, w, h }; AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, TRUE);
		m_hwnd = CreateWindow(L"D2DNESWnd", L"NES Emulator", WS_OVERLAPPEDWINDOW, (GetSystemMetrics(SM_CXSCREEN) - (rc.right - rc.left)) / 2, (GetSystemMetrics(SM_CYSCREEN) - (rc.bottom - rc.top)) / 2, rc.right - rc.left, rc.bottom - rc.top, 0, hM, hI, this);
		if (!m_hwnd) return E_FAIL; DragAcceptFiles(m_hwnd, TRUE); m_hm = GetMenu(m_hwnd); ShowWindow(m_hwnd, nC); UpdateWindow(m_hwnd); m_aud.Initialize(m_hwnd); return S_OK;
	}
	void OpenRomFile(const std::wstring& path) {
		std::wstring p = path; if (!p.empty() && p.front() == L'\"') p.erase(0, 1); if (!p.empty() && p.back() == L'\"') p.pop_back();
		if (m_nes.LoadRom(p)) { size_t s = p.find_last_of(L"\\/"), d = p.find_last_of(L"."); SetWindowText(m_hwnd, (L"NES Emulator - " + ((s == std::wstring::npos) ? p : p.substr(s + 1)).substr(0, (d != std::wstring::npos) ? d : std::wstring::npos)).c_str()); }
	}
	void ToggleFullscreen() {
		DWORD s = GetWindowLong(m_hwnd, GWL_STYLE);
		if (m_fs) { SetWindowLong(m_hwnd, GWL_STYLE, s | WS_OVERLAPPEDWINDOW); SetWindowPlacement(m_hwnd, &m_wp); SetWindowPos(m_hwnd, 0, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_FRAMECHANGED); SetMenu(m_hwnd, m_hm); CheckMenuItem(m_hm, IDM_FILE_FULLSCREEN, MF_UNCHECKED); ShowCursor(TRUE); m_fs = FALSE; }
		else { m_wp.length = sizeof(m_wp); GetWindowPlacement(m_hwnd, &m_wp); SetWindowLong(m_hwnd, GWL_STYLE, s & ~WS_OVERLAPPEDWINDOW); CheckMenuItem(m_hm, IDM_FILE_FULLSCREEN, MF_CHECKED); SetMenu(m_hwnd, 0); HMONITOR hM = MonitorFromWindow(m_hwnd, MONITOR_DEFAULTTOPRIMARY); MONITORINFO mi = { sizeof(mi) }; GetMonitorInfo(hM, &mi); SetWindowPos(m_hwnd, HWND_TOP, mi.rcMonitor.left, mi.rcMonitor.top, mi.rcMonitor.right - mi.rcMonitor.left, mi.rcMonitor.bottom - mi.rcMonitor.top, SWP_NOOWNERZORDER | SWP_FRAMECHANGED); ShowCursor(FALSE); m_fs = TRUE; }
	}
	void Run() {
		timeBeginPeriod(1); MSG msg = { 0 }; LARGE_INTEGER f, l, c; QueryPerformanceFrequency(&f); QueryPerformanceCounter(&l);
		const double SPF = 1.0 / 60.0988; const int CPF = (int)(NES_CPU_CLOCK / 60.0988);
		while (msg.message != WM_QUIT) {
			if (PeekMessage(&msg, 0, 0, 0, PM_REMOVE)) { TranslateMessage(&msg); DispatchMessage(&msg); continue; }
			QueryPerformanceCounter(&c); double el = (double)(c.QuadPart - l.QuadPart) / f.QuadPart;
			if (el >= SPF) {
				l.QuadPart += (LONGLONG)(SPF * f.QuadPart); if ((double)(c.QuadPart - l.QuadPart) / f.QuadPart > SPF) l.QuadPart = c.QuadPart;
				if (m_nes.isLoaded) { m_nes.StepFrame(); m_nes.apu.Step(CPF); if (!m_nes.apu.buffer.empty()) { m_aud.PushSamples(m_nes.apu.buffer); m_nes.apu.buffer.clear(); } }
				if (m_pRT || (CreateDeviceResources(), m_pRT)) {
					if (!(m_pRT->CheckWindowState() & D2D1_WINDOW_STATE_OCCLUDED)) {
						m_pRT->BeginDraw(); m_pRT->Clear(D2D1::ColorF(D2D1::ColorF::Black));
						if (m_pBmp) { m_pBmp->CopyFromMemory(0, m_nes.displayBuffer.data(), NES_WIDTH * 4); D2D1_SIZE_F s = m_pRT->GetSize(); float sc = (std::min)(s.width / NES_WIDTH, s.height / NES_HEIGHT), dw = NES_WIDTH * sc, dh = NES_HEIGHT * sc; m_pRT->DrawBitmap(m_pBmp, D2D1::RectF((s.width - dw) / 2, (s.height - dh) / 2, (s.width + dw) / 2, (s.height + dh) / 2), 1.0f, D2D1_BITMAP_INTERPOLATION_MODE_NEAREST_NEIGHBOR, 0); }
						if (m_pRT->EndDraw() == D2DERR_RECREATE_TARGET) { SafeRelease(&m_pBmp); SafeRelease(&m_pRT); }
					}
				}
			}
			else if (SPF - el > 0.002) Sleep(1);
		} timeEndPeriod(1);
	}
	void CreateDeviceResources() { RECT rc; GetClientRect(m_hwnd, &rc); m_pD2DF->CreateHwndRenderTarget(D2D1::RenderTargetProperties(), D2D1::HwndRenderTargetProperties(m_hwnd, D2D1::SizeU(rc.right - rc.left, rc.bottom - rc.top)), &m_pRT); if (m_pRT) { D2D1_BITMAP_PROPERTIES p = { D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE),96.f,96.f }; m_pRT->CreateBitmap(D2D1::SizeU(NES_WIDTH, NES_HEIGHT), p, &m_pBmp); } }
	static LRESULT CALLBACK WndProc(HWND h, UINT m, WPARAM w, LPARAM l) {
		App* p = (App*)GetWindowLongPtr(h, GWLP_USERDATA);
		switch (m) {
		case WM_CREATE: SetWindowLongPtr(h, GWLP_USERDATA, (LONG_PTR)((LPCREATESTRUCT)l)->lpCreateParams); return 0;
		case WM_ENTERMENULOOP: case WM_ENTERSIZEMOVE: if (p) p->m_aud.Stop(); return 0;
		case WM_EXITMENULOOP: case WM_EXITSIZEMOVE: if (p && p->m_nes.isLoaded) p->m_aud.Resume(); return 0;
		case WM_COMMAND: if (LOWORD(w) == IDM_FILE_OPEN && p) { p->m_aud.Stop(); OPENFILENAME ofn = { sizeof(ofn) }; wchar_t f[260] = { 0 }; ofn.hwndOwner = h; ofn.lpstrFile = f; ofn.nMaxFile = 260; ofn.lpstrFilter = L"NES ROMs\0*.nes\0All Files\0*.*\0"; ofn.nFilterIndex = 1; ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST; if (GetOpenFileName(&ofn)) p->OpenRomFile(f); p->m_aud.Resume(); }
					   else if (LOWORD(w) == IDM_FILE_EXIT) DestroyWindow(h); else if (LOWORD(w) == IDM_FILE_FULLSCREEN && p) p->ToggleFullscreen(); return 0;
		case WM_NCHITTEST: { LRESULT r = DefWindowProc(h, m, w, l); return (r == HTCLIENT && p && !p->m_fs) ? HTCAPTION : r; }
		case WM_SIZE: if (p && p->m_pRT) p->m_pRT->Resize(D2D1::SizeU(LOWORD(l), HIWORD(l))); return 0;
		case WM_KEYDOWN: case WM_KEYUP: if (p) { if (m == WM_KEYDOWN && w == VK_F11) { p->ToggleFullscreen(); return 0; } if (m == WM_KEYDOWN && w == VK_ESCAPE && p->m_fs) { p->ToggleFullscreen(); return 0; } int id = (w == 'X') ? 0 : (w == 'Z') ? 1 : (w == VK_SHIFT) ? 2 : (w == VK_RETURN) ? 3 : (w == VK_UP) ? 4 : (w == VK_DOWN) ? 5 : (w == VK_LEFT) ? 6 : (w == VK_RIGHT) ? 7 : -1; p->m_nes.InputKey(id, m == WM_KEYDOWN); } return 0;
		case WM_DROPFILES: if (p) { wchar_t f[MAX_PATH]; if (DragQueryFile((HDROP)w, 0, f, MAX_PATH)) { p->OpenRomFile(f); SetForegroundWindow(h); SetFocus(h); } DragFinish((HDROP)w); } return 0;
		case WM_DESTROY: PostQuitMessage(0); return 0;
		} return DefWindowProc(h, m, w, l);
	}
};
int WINAPI wWinMain(_In_ HINSTANCE hI, _In_opt_ HINSTANCE, _In_ LPWSTR, _In_ int nC) {
	SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2); CoInitialize(0); App a;
	if (SUCCEEDED(a.Initialize(hI, nC))) { int c; LPWSTR* v = CommandLineToArgvW(GetCommandLineW(), &c); if (v && c > 1) a.OpenRomFile(v[1]); LocalFree(v); a.Run(); }
	CoUninitialize(); return 0;
}