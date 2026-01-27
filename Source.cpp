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
template <class T> void SafeRelease(T** ppT) { if (*ppT) { (*ppT)->Release(); *ppT = NULL; } }
class Bus;
class CPU;
class PPU;
class APU;
class Cartridge;
class Mapper;
class AudioDriver {
    IDirectSound8* m_pDS; IDirectSoundBuffer* m_pPrimary; IDirectSoundBuffer* m_pSecondary;
    int m_bufferSize, m_nextWriteOffset;
public:
    AudioDriver() : m_pDS(NULL), m_pPrimary(NULL), m_pSecondary(NULL), m_bufferSize(0), m_nextWriteOffset(0) {}
    ~AudioDriver() { SafeRelease(&m_pSecondary); SafeRelease(&m_pPrimary); SafeRelease(&m_pDS); }
    bool Initialize(HWND hwnd) {
        if (FAILED(DirectSoundCreate8(NULL, &m_pDS, NULL))) return false;
        if (FAILED(m_pDS->SetCooperativeLevel(hwnd, DSSCL_PRIORITY))) return false;
        DSBUFFERDESC dsbd = { 0 }; dsbd.dwSize = sizeof(DSBUFFERDESC); dsbd.dwFlags = DSBCAPS_PRIMARYBUFFER;
        if (FAILED(m_pDS->CreateSoundBuffer(&dsbd, &m_pPrimary, NULL))) return false;
        WAVEFORMATEX wfx = { 0 }; wfx.wFormatTag = WAVE_FORMAT_PCM; wfx.nChannels = NUM_CHANNELS;
        wfx.nSamplesPerSec = SAMPLE_RATE; wfx.wBitsPerSample = 16;
        wfx.nBlockAlign = (wfx.nChannels * wfx.wBitsPerSample) / 8;
        wfx.nAvgBytesPerSec = wfx.nSamplesPerSec * wfx.nBlockAlign;
        if (FAILED(m_pPrimary->SetFormat(&wfx))) return false;
        m_bufferSize = wfx.nAvgBytesPerSec;
        dsbd.dwFlags = DSBCAPS_GETCURRENTPOSITION2 | DSBCAPS_GLOBALFOCUS | DSBCAPS_CTRLVOLUME;
        dsbd.dwBufferBytes = m_bufferSize; dsbd.lpwfxFormat = &wfx;
        if (FAILED(m_pDS->CreateSoundBuffer(&dsbd, &m_pSecondary, NULL))) return false;
        ClearBuffer(); m_pSecondary->Play(0, 0, DSBPLAY_LOOPING); return true;
    }
    void ClearBuffer() {
        void* p1, * p2; DWORD l1, l2;
        if (SUCCEEDED(m_pSecondary->Lock(0, m_bufferSize, &p1, &l1, &p2, &l2, 0))) {
            ZeroMemory(p1, l1); if (p2) ZeroMemory(p2, l2); m_pSecondary->Unlock(p1, l1, p2, l2);
        }
        m_nextWriteOffset = 0;
    }
    void Stop() { if (m_pSecondary) m_pSecondary->Stop(); }
    void Resume() { if (m_pSecondary) m_pSecondary->Play(0, 0, DSBPLAY_LOOPING); }
    void PushSamples(const std::vector<int16_t>& samples) {
        if (!m_pSecondary || samples.empty()) return;
        int bytesToWrite = (int)samples.size() * sizeof(int16_t);
        DWORD playPos, writePos;
        if (FAILED(m_pSecondary->GetCurrentPosition(&playPos, &writePos))) return;
        int safeMargin = m_bufferSize / 20;
        int latency = m_nextWriteOffset - (int)playPos;
        if (latency < 0) latency += m_bufferSize;
        if (latency < safeMargin || latency > m_bufferSize / 2) {
            m_nextWriteOffset = ((int)playPos + safeMargin) % m_bufferSize;
        }
        void* p1, * p2; DWORD l1, l2;
        if (SUCCEEDED(m_pSecondary->Lock(m_nextWriteOffset, bytesToWrite, &p1, &l1, &p2, &l2, 0))) {
            memcpy(p1, samples.data(), l1);
            if (p2) memcpy(p2, (uint8_t*)samples.data() + l1, l2);
            m_pSecondary->Unlock(p1, l1, p2, l2);
            m_nextWriteOffset = (m_nextWriteOffset + bytesToWrite) % m_bufferSize;
        }
    }
};
class APU {
public:
    static constexpr int length_table[32] = { 10,254,20,2,40,4,80,6,160,8,60,10,14,12,26,14,12,16,24,18,48,20,96,22,192,24,72,26,16,28,32,30 };
    static constexpr int duty_seq[4][8] = { {0,1,0,0,0,0,0,0},{0,1,1,0,0,0,0,0},{0,1,1,1,1,0,0,0},{1,0,0,1,1,1,1,1} };
    static constexpr int noise_period[16] = { 4,8,16,32,64,96,128,160,202,254,380,508,762,1016,2034,4068 };
    struct LengthCounter {
        int counter = 0; bool enabled = false, halt = false;
        void Load(int code) { if (enabled) counter = length_table[code & 0x1F]; }
        void Clock() { if (!enabled) counter = 0; else if (counter > 0 && !halt) counter--; }
    };
    struct Envelope {
        bool start = false, loop = false, constant = false; int volume = 0, output = 0, decayCount = 0, divider = 0;
        void Clock() {
            if (!start) { if (divider == 0) { divider = volume; if (decayCount == 0) { if (loop) decayCount = 15; } else decayCount--; } else divider--; }
            else { start = false; decayCount = 15; divider = volume; }
            output = constant ? volume : decayCount;
        }
    };
    struct Sweep {
        bool enabled = false, negate = false, reload = false; int period = 0, shift = 0, timer = 0;
        int CalculateTarget(int current_period, int channelID) {
            int change = current_period >> shift;
            if (negate) { change = -change; if (channelID == 0) change--; }
            return current_period + change;
        }
        bool IsMuted(int current_period, int channelID) { return (current_period < 8) || (!negate && CalculateTarget(current_period, channelID) > 0x7FF); }
        int Clock(int current_period, int channelID) {
            int target = CalculateTarget(current_period, channelID);
            bool should_update = (timer == 0 && enabled && shift > 0 && !IsMuted(current_period, channelID));
            if (timer == 0 || reload) { timer = period; reload = false; }
            else timer--;
            return should_update ? target : -1;
        }
    };
    struct PulseChannel {
        int id = 0, timer = 0, timer_period = 0, duty_mode = 0, sequence = 0;
        LengthCounter lc; Envelope env; Sweep sweep;
        void StepTimer() { if (timer > 0) timer--; else { timer = timer_period; sequence = (sequence + 1) % 8; } }
        int GetOutput() { return (lc.counter == 0 || sweep.IsMuted(timer_period, id) || !duty_seq[duty_mode][sequence]) ? 0 : env.output; }
    } pulse1, pulse2;
    struct TriangleChannel {
        int timer = 0, timer_period = 0, sequence = 0, linear_counter = 0, linear_reload_val = 0;
        bool linear_reload_flag = false, control_flag = false; LengthCounter lc;
        static constexpr int sequence_table[32] = { 15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15 };
        void StepTimer() { if (timer > 0) timer--; else { timer = (timer_period < 2) ? 2 : timer_period; if (lc.counter > 0 && linear_counter > 0) sequence = (sequence + 1) % 32; } }
        void ClockLinear() { if (linear_reload_flag) linear_counter = linear_reload_val; else if (linear_counter > 0) linear_counter--; if (!control_flag) linear_reload_flag = false; }
        int GetOutput() { return sequence_table[sequence]; }
    } triangle;
    struct NoiseChannel {
        int timer = 0, timer_period = 0, mode = 0; uint16_t lfsr = 1; LengthCounter lc; Envelope env;
        void StepTimer() { if (timer > 0) timer--; else { timer = timer_period; int f = (lfsr & 1) ^ ((lfsr >> (mode ? 6 : 1)) & 1); lfsr >>= 1; lfsr |= (f << 14); } }
        int GetOutput() { return (lc.counter == 0 || (lfsr & 1)) ? 0 : env.output; }
    } noise;
    double frameSequencerTime = 0.0, accCount = 0, filter_capacitor = 0.0, sampleSum = 0.0;
    int frameStep = 0, sampleCount = 0;
    double lpfVal = 0.0;
    std::vector<int16_t> buffer;
    APU() { Reset(); }
    void Reset() {
        pulse1 = PulseChannel(); pulse1.id = 0; pulse2 = PulseChannel(); pulse2.id = 1; triangle = TriangleChannel(); noise = NoiseChannel(); noise.lfsr = 1;
        pulse1.lc.enabled = pulse2.lc.enabled = triangle.lc.enabled = noise.lc.enabled = true;
        buffer.clear(); buffer.reserve(4096); frameSequencerTime = 0; frameStep = 0;
        sampleSum = 0.0; sampleCount = 0; filter_capacitor = 0.0;
    }
    void Write(Word addr, Byte data) {
        switch (addr) {
        case 0x4000: pulse1.duty_mode = (data >> 6) & 3; pulse1.lc.halt = pulse1.env.loop = (data & 0x20); pulse1.env.constant = (data & 0x10); pulse1.env.volume = (data & 0x0F); break;
        case 0x4001: pulse1.sweep.enabled = (data & 0x80); pulse1.sweep.period = (data >> 4) & 7; pulse1.sweep.negate = (data & 0x08); pulse1.sweep.shift = (data & 0x07); pulse1.sweep.reload = true; break;
        case 0x4002: pulse1.timer_period = (pulse1.timer_period & 0xFF00) | data; break;
        case 0x4003: pulse1.timer_period = (pulse1.timer_period & 0x00FF) | ((data & 7) << 8); pulse1.sequence = 0; pulse1.lc.Load(data >> 3); pulse1.env.start = true; break;
        case 0x4004: pulse2.duty_mode = (data >> 6) & 3; pulse2.lc.halt = pulse2.env.loop = (data & 0x20); pulse2.env.constant = (data & 0x10); pulse2.env.volume = (data & 0x0F); break;
        case 0x4005: pulse2.sweep.enabled = (data & 0x80); pulse2.sweep.period = (data >> 4) & 7; pulse2.sweep.negate = (data & 0x08); pulse2.sweep.shift = (data & 0x07); pulse2.sweep.reload = true; break;
        case 0x4006: pulse2.timer_period = (pulse2.timer_period & 0xFF00) | data; break;
        case 0x4007: pulse2.timer_period = (pulse2.timer_period & 0x00FF) | ((data & 7) << 8); pulse2.sequence = 0; pulse2.lc.Load(data >> 3); pulse2.env.start = true; break;
        case 0x4008: triangle.control_flag = (data & 0x80); triangle.linear_reload_val = (data & 0x7F); break;
        case 0x400A: triangle.timer_period = (triangle.timer_period & 0xFF00) | data; break;
        case 0x400B: triangle.timer_period = (triangle.timer_period & 0x00FF) | ((data & 7) << 8); triangle.lc.Load(data >> 3); triangle.linear_reload_flag = true; break;
        case 0x400C: noise.lc.halt = noise.env.loop = (data & 0x20); noise.env.constant = (data & 0x10); noise.env.volume = (data & 0x0F); break;
        case 0x400E: noise.mode = (data & 0x80); noise.timer_period = noise_period[data & 0x0F]; break;
        case 0x400F: noise.lc.Load(data >> 3); noise.env.start = true; break;
        case 0x4015: pulse1.lc.enabled = (data & 1); if (!pulse1.lc.enabled) pulse1.lc.counter = 0; pulse2.lc.enabled = (data & 2); if (!pulse2.lc.enabled) pulse2.lc.counter = 0; triangle.lc.enabled = (data & 4); if (!triangle.lc.enabled) triangle.lc.counter = 0; noise.lc.enabled = (data & 8); if (!noise.lc.enabled) noise.lc.counter = 0; break;
        case 0x4017: frameStep = 0; if (data & 0x80) ClockFrameCounter(); break;
        }
    }
    void ClockFrameCounter() {
        frameStep++;
        if (frameStep == 1 || frameStep == 3) { pulse1.env.Clock(); pulse2.env.Clock(); noise.env.Clock(); triangle.ClockLinear(); }
        else if (frameStep == 2 || frameStep == 4) {
            pulse1.env.Clock(); pulse2.env.Clock(); noise.env.Clock(); triangle.ClockLinear();
            pulse1.lc.Clock(); pulse2.lc.Clock(); triangle.lc.Clock(); noise.lc.Clock();
            int p1 = pulse1.sweep.Clock(pulse1.timer_period, 0); if (p1 >= 0 && pulse1.sweep.enabled && pulse1.sweep.shift > 0) pulse1.timer_period = p1;
            int p2 = pulse2.sweep.Clock(pulse2.timer_period, 1); if (p2 >= 0 && pulse2.sweep.enabled && pulse2.sweep.shift > 0) pulse2.timer_period = p2;
        }
        if (frameStep >= 4) frameStep = 0;
    }
    void Step(int cycles) {
        const double CYCLES_PER_SAMPLE = 1789773.0 / 44100.0;
        const double CYCLES_PER_FRAME_STEP = 1789773.0 / 240.0;
        for (int i = 0; i < cycles; i++) {
            if (i % 2 == 0) { pulse1.StepTimer(); pulse2.StepTimer(); noise.StepTimer(); }
            triangle.StepTimer();
            frameSequencerTime += 1.0;
            if (frameSequencerTime >= CYCLES_PER_FRAME_STEP) {
                frameSequencerTime -= CYCLES_PER_FRAME_STEP;
                ClockFrameCounter();
            }
            int p1 = (pulse1.lc.counter > 0 && !pulse1.sweep.IsMuted(pulse1.timer_period, 0) && duty_seq[pulse1.duty_mode][pulse1.sequence]) ? pulse1.env.output : 0;
            int p2 = (pulse2.lc.counter > 0 && !pulse2.sweep.IsMuted(pulse2.timer_period, 1) && duty_seq[pulse2.duty_mode][pulse2.sequence]) ? pulse2.env.output : 0;
            int t = (triangle.lc.counter > 0 && triangle.linear_counter > 0) ? triangle.sequence_table[triangle.sequence] : 0;
            int n = (noise.lc.counter > 0 && !(noise.lfsr & 1)) ? noise.env.output : 0;
            double pulseOut = 0.00752 * (p1 + p2);
            double tndOut = 0.00851 * t + 0.00494 * n;
            double output = pulseOut + tndOut;
            sampleSum += output;
            sampleCount++;
            accCount += 1.0;
            if (accCount >= CYCLES_PER_SAMPLE) {
                accCount -= CYCLES_PER_SAMPLE;
                if (sampleCount > 0) {
                    double mixed = sampleSum / sampleCount;
                    sampleSum = 0.0; sampleCount = 0;
                    lpfVal = lpfVal * 0.8 + mixed * 0.2;
                    mixed = lpfVal;
                    double hpf = mixed - filter_capacitor;
                    filter_capacitor = mixed - hpf * 0.995;
                    double sd = hpf * 28000.0;
                    if (sd > 32000.0) sd = 32000.0;
                    if (sd < -32000.0) sd = -32000.0;
                    int16_t s = (int16_t)sd;
                    buffer.push_back(s);
                    buffer.push_back(s);
                }
            }
        }
    }
};
class PPU {
public:
    Bus* bus; Cartridge* cart; uint32_t* screenBuffer;
    Byte tblName[2][1024], tblPalette[32];
    struct { Byte control, mask, status, oam_addr, oam_data, scroll, address, data; } reg;
    Word vram_addr = 0, temp_vram_addr = 0; Byte fine_x = 0, address_latch = 0, data_buffer = 0;
    int scanline = 0, cycle = 0; bool frameComplete = false, nmi = false;
    struct OAMEntry { Byte y, id, attribute, x; } OAM[64];
    const uint32_t PALETTE[64] = {
        0xFF7C7C7C,0xFF0000FC,0xFF0000BC,0xFF4428BC,0xFF940084,0xFFA80020,0xFFA81000,0xFF881400,0xFF503000,0xFF007800,0xFF006800,0xFF005800,0xFF004058,0xFF000000,0xFF000000,0xFF000000,
        0xFFBCBCBC,0xFF0078F8,0xFF0058F8,0xFF6844FC,0xFFD800CC,0xFFE40058,0xFFF83800,0xFFE45C10,0xFFAC7C00,0xFF00B800,0xFF00A800,0xFF00A844,0xFF008888,0xFF000000,0xFF000000,0xFF000000,
        0xFFF8F8F8,0xFF3CBCFC,0xFF6888FC,0xFF9878F8,0xFFF878F8,0xFFF85898,0xFFF87858,0xFFFCA044,0xFFF8B800,0xFFB8F818,0xFF58D854,0xFF58F898,0xFF00E8D8,0xFF787878,0xFF000000,0xFF000000,
        0xFFFCFCFC,0xFFA4E4FC,0xFFB8B8F8,0xFFD8B8F8,0xFFF8B8F8,0xFFF8A4C0,0xFFF0D0B0,0xFFFCE0A8,0xFFF8D878,0xFFD8F878,0xFFB8F8B8,0xFFB8F8D8,0xFF00FCFC,0xFFF8D8F8,0xFF000000,0xFF000000
    };
    PPU();
    void ConnectBus(Bus* n);
    void SetCart(Cartridge* c);
    void SetScreenBuffer(uint32_t* buff);
    void Reset();
    Byte cpuRead(Word addr);
    void cpuWrite(Word addr, Byte data);
    Byte ppuRead(Word addr);
    void ppuWrite(Word addr, Byte data);
    void DrawScanline();
    void Clock();
};
class Bus {
public:
    CPU* cpu; PPU* ppu; APU* apu; Cartridge* cart;
    Byte ram[2048], controller[2], controller_state[2], dma_page = 0, dma_addr = 0;
    bool dma_transfer = false;
    uint8_t* prgReadMap[4];
    Bus();
    void SetCPU(CPU* c) { cpu = c; } void SetPPU(PPU* p) { ppu = p; } void SetAPU(APU* a) { apu = a; }
    void SetCart(Cartridge* c);
    Byte cpuRead(Word addr);
    void cpuWrite(Word addr, Byte data);
};
class Mapper {
protected:
    Byte nPRGBanks = 0, nCHRBanks = 0;
    Bus* pBus = nullptr;
    std::vector<Byte>& PRGData;
    std::vector<Byte>& CHRData;
public:
    Mapper(Byte prg, Byte chr, Bus* bus, std::vector<Byte>& p, std::vector<Byte>& c)
        : nPRGBanks(prg), nCHRBanks(chr), pBus(bus), PRGData(p), CHRData(c) {
    }
    virtual ~Mapper() {}
    void MapPRG(int slot, int bank) {
        if (bank < 0) bank = 0;
        pBus->prgReadMap[slot] = &PRGData[(bank * 0x2000) % PRGData.size()];
    }
    virtual void Reset() = 0;
    virtual bool cpuMapRead(Word addr, Byte& data) { return false; }
    virtual bool cpuMapWrite(Word addr, Byte data) = 0;
    virtual bool ppuMapRead(Word addr, uint32_t& mapped_addr) = 0;
    virtual bool ppuMapWrite(Word addr, uint32_t& mapped_addr) = 0;
    virtual MIRROR mirror() { return HARDWARE; }
    virtual bool irqState() { return false; }
    virtual void irqClear() {}
    virtual void scanline() {}
    virtual void tick(int cycles) {}
};
class Mapper_000 : public Mapper {
public:
    using Mapper::Mapper;
    void Reset() override {
        MapPRG(0, 0);
        MapPRG(1, 1);
        if (nPRGBanks > 1) {
            MapPRG(2, 2);
            MapPRG(3, 3);
        }
        else {
            MapPRG(2, 0);
            MapPRG(3, 1);
        }
    }
    bool cpuMapWrite(Word addr, Byte d) override { return false; }
    bool ppuMapRead(Word addr, uint32_t& m) override { if (addr <= 0x1FFF) { m = addr; return true; } return false; }
    bool ppuMapWrite(Word addr, uint32_t& m) override { if (addr <= 0x1FFF) { m = addr; return true; } return false; }
};
class Mapper_001 : public Mapper {
    Byte nLoadReg = 0, nLoadRegCount = 0, nCtrlReg = 0x0C;
    Byte nCHR4Lo = 0, nCHR4Hi = 0, nCHR8 = 0;
    Byte nPRG16Lo = 0, nPRG16Hi = 0, nPRG32 = 0;
    std::vector<Byte> vRAM;
public:
    using Mapper::Mapper;
    void Reset() override {
        vRAM.resize(8192);
        std::fill(vRAM.begin(), vRAM.end(), 0);
        nCtrlReg = 0x0C;
        nLoadReg = 0; nLoadRegCount = 0;
        nCHR4Lo = 0; nCHR4Hi = 0; nCHR8 = 0;
        nPRG32 = 0; nPRG16Lo = 0; nPRG16Hi = nPRGBanks - 1;
        UpdateState();
    }
    void UpdateState() {
        uint32_t prgMask = (nPRGBanks * 2) - 1;
        switch ((nCtrlReg >> 2) & 3) {
        case 0: case 1:
            MapPRG(0, (nPRG32 * 4 + 0) & prgMask);
            MapPRG(1, (nPRG32 * 4 + 1) & prgMask);
            MapPRG(2, (nPRG32 * 4 + 2) & prgMask);
            MapPRG(3, (nPRG32 * 4 + 3) & prgMask);
            break;
        case 2:
            MapPRG(0, 0);
            MapPRG(1, 1);
            MapPRG(2, (nPRG16Hi * 2 + 0) & prgMask);
            MapPRG(3, (nPRG16Hi * 2 + 1) & prgMask);
            break;
        case 3:
            MapPRG(0, (nPRG16Lo * 2 + 0) & prgMask);
            MapPRG(1, (nPRG16Lo * 2 + 1) & prgMask);
            MapPRG(2, (nPRGBanks * 2 - 2) & prgMask);
            MapPRG(3, (nPRGBanks * 2 - 1) & prgMask);
            break;
        }
    }
    MIRROR mirror() override {
        switch (nCtrlReg & 3) {
        case 0: return ONESCREEN_LO;
        case 1: return ONESCREEN_HI;
        case 2: return VERTICAL;
        case 3: return HORIZONTAL;
        }
        return HARDWARE;
    }
    bool cpuMapRead(Word addr, Byte& data) override {
        if (addr >= 0x6000 && addr <= 0x7FFF) {
            data = vRAM[addr & 0x1FFF];
            return true;
        }
        return false;
    }
    bool cpuMapWrite(Word addr, Byte d) override {
        if (addr >= 0x6000 && addr <= 0x7FFF) {
            vRAM[addr & 0x1FFF] = d;
            return true;
        }
        if (addr >= 0x8000) {
            if (d & 0x80) {
                nLoadReg = 0; nLoadRegCount = 0; nCtrlReg |= 0x0C;
                UpdateState();
            }
            else {
                nLoadReg >>= 1; nLoadReg |= (d & 1) << 4; nLoadRegCount++;
                if (nLoadRegCount == 5) {
                    Byte target = (addr >> 13) & 3;
                    if (target == 0) {
                        nCtrlReg = nLoadReg & 0x1F;
                    }
                    else if (target == 1) {
                        if (nCtrlReg & 0x10) nCHR4Lo = nLoadReg & 0x1F;
                        else                 nCHR8 = (nLoadReg & 0x1E) >> 1;
                    }
                    else if (target == 2) {
                        if (nCtrlReg & 0x10) nCHR4Hi = nLoadReg & 0x1F;
                    }
                    else if (target == 3) {
                        Byte mode = (nCtrlReg >> 2) & 3;
                        if (mode <= 1) { nPRG32 = (nLoadReg & 0x0E) >> 1; }
                        else if (mode == 2) { nPRG16Lo = 0; nPRG16Hi = nLoadReg & 0x0F; }
                        else if (mode == 3) { nPRG16Lo = nLoadReg & 0x0F; nPRG16Hi = nPRGBanks - 1; }
                    }
                    nLoadReg = 0; nLoadRegCount = 0;
                    UpdateState();
                }
            }
            return true;
        }
        return false;
    }
    bool ppuMapRead(Word addr, uint32_t& m) override {
        if (addr < 0x2000) {
            uint32_t bank = 0, offset = 0;
            if (nCtrlReg & 0x10) {
                if (addr < 0x1000) {
                    bank = nCHR4Lo; offset = addr & 0x0FFF;
                }
                else {
                    bank = nCHR4Hi; offset = addr & 0x0FFF;
                }
            }
            else {
                bank = nCHR8; offset = addr & 0x1FFF;
            }
            if (nCtrlReg & 0x10) {
                if (nCHRBanks > 0) bank %= (nCHRBanks * 2);
                m = (bank * 0x1000) + offset;
            }
            else {
                if (nCHRBanks > 0) bank %= nCHRBanks;
                m = (bank * 0x2000) + offset;
            }
            if (nCHRBanks == 0) {
                m &= 0x1FFF;
            }
            return true;
        }
        return false;
    }
    bool ppuMapWrite(Word addr, uint32_t& m) override {
        if (addr < 0x2000) {
            if (nCHRBanks == 0) {
                return ppuMapRead(addr, m);
            }
            return false;
        }
        return false;
    }
};
class Mapper_002 : public Mapper {
    Byte nPRGBankSelectLo = 0;
public:
    using Mapper::Mapper;
    void Reset() override {
        nPRGBankSelectLo = 0;
        MapPRG(0, 0);
        MapPRG(1, 1);
        MapPRG(2, nPRGBanks * 2 - 2);
        MapPRG(3, nPRGBanks * 2 - 1);
    }
    bool cpuMapWrite(Word addr, Byte d) override {
        if (addr >= 0x8000) {
            nPRGBankSelectLo = d;
            MapPRG(0, nPRGBankSelectLo * 2);
            MapPRG(1, nPRGBankSelectLo * 2 + 1);
            return true;
        }
        return false;
    }
    bool ppuMapRead(Word addr, uint32_t& m) override {
        if (addr < 0x2000) { m = addr; return true; }
        return false;
    }
    bool ppuMapWrite(Word addr, uint32_t& m) override {
        if (addr < 0x2000) { m = addr; return true; }
        return false;
    }
};
class Mapper_003 : public Mapper {
    Byte nCHRBankSelect = 0;
public:
    using Mapper::Mapper;
    void Reset() override {
        nCHRBankSelect = 0;
        MapPRG(0, 0); MapPRG(1, 1);
        if (nPRGBanks > 1) { MapPRG(2, 2); MapPRG(3, 3); }
        else { MapPRG(2, 0); MapPRG(3, 1); }
    }
    bool cpuMapWrite(Word addr, Byte d) override {
        if (addr >= 0x8000) {
            nCHRBankSelect = d & 0x03;
            return true;
        }
        return false;
    }
    bool ppuMapRead(Word addr, uint32_t& m) override {
        if (addr < 0x2000) {
            m = (nCHRBankSelect * 0x2000) + addr;
            return true;
        }
        return false;
    }
    bool ppuMapWrite(Word addr, uint32_t& m) override {
        if (addr < 0x2000) { m = (nCHRBankSelect * 0x2000) + addr; return true; }
        return false;
    }
};
class Mapper_004 : public Mapper {
    uint8_t nTargetRegister = 0;
    bool bPRGBankMode = false;
    bool bCHRInversion = false;
    MIRROR mirroringMode = HORIZONTAL;
    uint32_t pRegister[8];
    uint32_t pCHRBank[8];
    bool bIRQActive = false;
    bool bIRQEnable = false;
    uint8_t nIRQCounter = 0;
    uint8_t nIRQLatch = 0;
    bool bIRQReload = false;
    std::vector<Byte> vRAM;
    int irq_filter_delay = 0;
public:
    using Mapper::Mapper;
    void Reset() override {
        irq_filter_delay = 0;
        vRAM.resize(8192);
        std::fill(vRAM.begin(), vRAM.end(), 0);
        nTargetRegister = 0; bPRGBankMode = false; bCHRInversion = false;
        mirroringMode = HORIZONTAL;
        bIRQActive = false; bIRQEnable = false;
        nIRQCounter = 0; nIRQLatch = 0; bIRQReload = false;
        memset(pRegister, 0, sizeof(pRegister));
        memset(pCHRBank, 0, sizeof(pCHRBank));
        pRegister[6] = 0; pRegister[7] = 1;
        UpdateBanks();
    }
    void UpdateBanks() {
        if (bPRGBankMode) {
            MapPRG(0, nPRGBanks * 2 - 2);
            MapPRG(1, pRegister[7]);
            MapPRG(2, pRegister[6]);
            MapPRG(3, nPRGBanks * 2 - 1);
        }
        else {
            MapPRG(0, pRegister[6]);
            MapPRG(1, pRegister[7]);
            MapPRG(2, nPRGBanks * 2 - 2);
            MapPRG(3, nPRGBanks * 2 - 1);
        }
        if (bCHRInversion) {
            pCHRBank[0] = pRegister[2]; pCHRBank[1] = pRegister[3];
            pCHRBank[2] = pRegister[4]; pCHRBank[3] = pRegister[5];
            pCHRBank[4] = pRegister[0] & 0xFE; pCHRBank[5] = pRegister[0] | 1;
            pCHRBank[6] = pRegister[1] & 0xFE; pCHRBank[7] = pRegister[1] | 1;
        }
        else {
            pCHRBank[0] = pRegister[0] & 0xFE; pCHRBank[1] = pRegister[0] | 1;
            pCHRBank[2] = pRegister[1] & 0xFE; pCHRBank[3] = pRegister[1] | 1;
            pCHRBank[4] = pRegister[2]; pCHRBank[5] = pRegister[3];
            pCHRBank[6] = pRegister[4]; pCHRBank[7] = pRegister[5];
        }
    }
    bool cpuMapRead(Word addr, Byte& data) override {
        if (addr >= 0x6000 && addr <= 0x7FFF) {
            data = vRAM[addr & 0x1FFF];
            return true;
        }
        return false;
    }
    bool cpuMapWrite(Word addr, Byte data) override {
        if (addr >= 0x6000 && addr <= 0x7FFF) {
            vRAM[addr & 0x1FFF] = data;
            return true;
        }
        bool handled = false;
        if (addr >= 0x8000 && addr <= 0x9FFF) {
            if (!(addr & 1)) {
                nTargetRegister = data & 0x07;
                bPRGBankMode = (data & 0x40);
                bCHRInversion = (data & 0x80);
                UpdateBanks();
            }
            else {
                pRegister[nTargetRegister] = data;
                UpdateBanks();
            }
            handled = true;
        }
        else if (addr >= 0xA000 && addr <= 0xBFFF) {
            if (!(addr & 1)) {
                mirroringMode = (data & 1) ? HORIZONTAL : VERTICAL;
            }
            handled = true;
        }
        else if (addr >= 0xC000 && addr <= 0xDFFF) {
            if (!(addr & 1)) {
                nIRQLatch = data;
            }
            else {
                bIRQReload = true;
            }
            handled = true;
        }
        else if (addr >= 0xE000 && addr <= 0xFFFF) {
            if (!(addr & 1)) {
                bIRQEnable = false; bIRQActive = false;
            }
            else {
                bIRQEnable = true;
            }
            handled = true;
        }
        return handled;
    }
    bool ppuMapRead(Word addr, uint32_t& m) override {
        if (addr < 0x3F00) {
            if ((addr & 0x1000)) {
                if (irq_filter_delay <= 0) {
                    if (nIRQCounter == 0 || bIRQReload) {
                        nIRQCounter = nIRQLatch;
                        bIRQReload = false;
                    }
                    else {
                        nIRQCounter--;
                    }
                    if (nIRQCounter == 0 && bIRQEnable) {
                        bIRQActive = true;
                    }
                    irq_filter_delay = 18;
                }
            }
        }
        if (addr < 0x2000) {
            uint32_t bank = pCHRBank[addr / 0x400];
            if (nCHRBanks > 0) {
                bank %= (nCHRBanks * 8);
            }
            m = (bank * 0x400) + (addr & 0x3FF);
            return true;
        }
        return false;
    }
    bool ppuMapWrite(Word addr, uint32_t& m) override {
        if (addr < 0x2000) {
            m = (pCHRBank[addr / 0x400] * 0x400) + (addr & 0x3FF);
            return true;
        }
        return false;
    }
    MIRROR mirror() override { return mirroringMode; }
    bool irqState() override { return bIRQActive; }
    void irqClear() override { bIRQActive = false; }
    void tick(int cycles) override {
        if (irq_filter_delay > 0) {
            irq_filter_delay -= cycles;
        }
    }
};
class Mapper_016 : public Mapper {
    int nPRG = 0;
    int nCHR[8];
    uint16_t nIRQCounter = 0;
    bool bIRQEnable = false;
    bool bIRQActive = false;
    MIRROR mirrorMode = VERTICAL;
public:
    using Mapper::Mapper;
    void Reset() override {
        nPRG = 0;
        for (int i = 0; i < 8; i++) nCHR[i] = i;
        nIRQCounter = 0; bIRQEnable = false; bIRQActive = false;
        MapPRG(0, 0); MapPRG(1, 1);
        MapPRG(2, nPRGBanks * 2 - 2); MapPRG(3, nPRGBanks * 2 - 1);
    }
    void tick(int cycles) override {
        if (bIRQEnable) {
            if ((int)nIRQCounter - cycles <= 0) {
                nIRQCounter = 0;
                bIRQActive = true;
            }
            else {
                nIRQCounter -= cycles;
            }
        }
    }
    bool cpuMapWrite(Word addr, Byte d) override {
        switch (addr & 0x000F) {
        case 0x00: case 0x01: case 0x02: case 0x03:
        case 0x04: case 0x05: case 0x06: case 0x07:
            nCHR[addr & 0x07] = d; return true;
        case 0x08:
            nPRG = d;
            MapPRG(0, nPRG * 2); MapPRG(1, nPRG * 2 + 1); return true;
        case 0x09:
            switch (d & 3) {
            case 0: mirrorMode = VERTICAL; break; case 1: mirrorMode = HORIZONTAL; break;
            case 2: mirrorMode = ONESCREEN_LO; break; case 3: mirrorMode = ONESCREEN_HI; break;
            } return true;
        case 0x0A:
            bIRQEnable = (d & 1); bIRQActive = false; return true;
        case 0x0B:
            nIRQCounter = (nIRQCounter & 0xFF00) | d; return true;
        case 0x0C:
            nIRQCounter = (nIRQCounter & 0x00FF) | (d << 8); return true;
        case 0x0D:
            return true;
        }
        return false;
    }
    bool ppuMapRead(Word addr, uint32_t& m) override {
        if (addr < 0x2000) { m = nCHR[addr / 0x400] * 0x400 + (addr & 0x3FF); return true; }
        return false;
    }
    bool ppuMapWrite(Word addr, uint32_t& m) override {
        if (addr < 0x2000) { m = nCHR[addr / 0x400] * 0x400 + (addr & 0x3FF); return true; }
        return false;
    }
    MIRROR mirror() override { return mirrorMode; }
    bool irqState() override { return bIRQActive; }
    void irqClear() override { bIRQActive = false; }
};
class Cartridge {
public:
    std::vector<Byte> PRG, CHR;
    Byte mapperID = 0, prgBanks = 0, chrBanks = 0;
    bool verticalMirroring = false;
    std::shared_ptr<Mapper> pMapper;
    Bus* pBus = nullptr;
    Cartridge(Bus* bus) : pBus(bus) {}
    void Tick(int cycles) { if (pMapper) pMapper->tick(cycles); }
    bool Load(const std::wstring& path) {
        FILE* fp = NULL; _wfopen_s(&fp, path.c_str(), L"rb"); if (!fp) return false;
        uint8_t h[16]; if (fread(h, 1, 16, fp) != 16) { fclose(fp); return false; }
        if (h[0] != 'N' || h[1] != 'E' || h[2] != 'S' || h[3] != 0x1A) { fclose(fp); return false; }
        prgBanks = h[4]; chrBanks = h[5]; mapperID = ((h[6] >> 4) & 0x0F) | (h[7] & 0xF0); verticalMirroring = (h[6] & 1);
        if (h[6] & 4) fseek(fp, 512, SEEK_CUR);
        PRG.resize((size_t)prgBanks * 16384); fread(PRG.data(), 1, PRG.size(), fp);
        if (chrBanks == 0) {
            CHR.resize(8192);
            std::fill(CHR.begin(), CHR.end(), 0);
        }
        else {
            CHR.resize((size_t)chrBanks * 8192);
            fread(CHR.data(), 1, CHR.size(), fp);
        }
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
        if (pMapper) pMapper->Reset();
        return true;
    }
    bool cpuRead(Word addr, Byte& data) {
        if (pMapper) return pMapper->cpuMapRead(addr, data);
        return false;
    }
    bool cpuWrite(Word addr, Byte data) { if (pMapper) return pMapper->cpuMapWrite(addr, data); return false; }
    bool ppuRead(Word addr, Byte& data) { uint32_t m = 0; if (pMapper && pMapper->ppuMapRead(addr, m)) { data = CHR[m]; return true; } return false; }
    bool ppuWrite(Word addr, Byte data) { uint32_t m = 0; if (pMapper && pMapper->ppuMapWrite(addr, m)) { CHR[m] = data; return true; } return false; }
    bool GetIRQ() { return pMapper ? pMapper->irqState() : false; }
    void ClearIRQ() { if (pMapper) pMapper->irqClear(); }
    MIRROR GetMirror() { MIRROR m = pMapper ? pMapper->mirror() : HARDWARE; return (m == HARDWARE) ? (verticalMirroring ? VERTICAL : HORIZONTAL) : m; }
};
Bus::Bus() { memset(ram, 0, 2048); memset(controller, 0, 2); memset(controller_state, 0, 2); for (int i = 0; i < 4; i++) prgReadMap[i] = nullptr; }
void Bus::SetCart(Cartridge* c) { cart = c; }
void Bus::cpuWrite(Word addr, Byte data) {
    if (addr >= 0x4020) { cart->cpuWrite(addr, data); return; }
    if (addr < 0x2000) ram[addr & 0x7FF] = data;
    else if (addr < 0x4000) ppu->cpuWrite(addr & 7, data);
    else if (addr == 0x4014) { dma_page = data; dma_addr = 0; dma_transfer = true; }
    else if (addr == 0x4016) controller_state[0] = controller[0];
    else if (addr >= 0x4000 && addr <= 0x4017) apu->Write(addr, data);
}
Byte Bus::cpuRead(Word addr) {
    if (addr >= 0x8000) {
        if (prgReadMap[(addr >> 13) & 3])
            return prgReadMap[(addr >> 13) & 3][addr & 0x1FFF];
        return 0;
    }
    if (addr < 0x2000) return ram[addr & 0x7FF];
    if (addr < 0x4000) return ppu->cpuRead(addr & 7);
    if (addr == 0x4016) { Byte data = (controller_state[0] & 1); controller_state[0] >>= 1; return data; }
    if (addr >= 0x4020) {
        Byte data = 0;
        if (cart && cart->cpuRead(addr, data)) return data;
    }
    return 0;
}
PPU::PPU() { Reset(); }
void PPU::ConnectBus(Bus* n) { bus = n; }
void PPU::SetCart(Cartridge* c) { cart = c; }
void PPU::SetScreenBuffer(uint32_t* buff) { screenBuffer = buff; }
void PPU::Reset() {
    memset(&reg, 0, sizeof(reg));
    memset(tblName, 0, sizeof(tblName));
    memset(tblPalette, 0, sizeof(tblPalette));
    memset(OAM, 0xFF, sizeof(OAM));
    scanline = 0;
    cycle = 0;
    frameComplete = false;
    address_latch = 0;
    fine_x = 0;
    temp_vram_addr = 0;
    vram_addr = 0;
    data_buffer = 0;
}
Byte PPU::cpuRead(Word addr) {
    Byte data = 0;
    switch (addr) {
    case 0x0002:
        data = (reg.status & 0xE0) | (data_buffer & 0x1F);
        reg.status &= 0x7F;
        address_latch = 0;
        break;
    case 0x0004: data = ((Byte*)OAM)[reg.oam_addr]; break;
    case 0x0007: data = data_buffer; data_buffer = ppuRead(vram_addr); if (vram_addr >= 0x3F00) data = data_buffer; vram_addr += (reg.control & 4) ? 32 : 1; break;
    }
    return data;
}
void PPU::cpuWrite(Word addr, Byte data) {
    switch (addr) {
    case 0x0000: reg.control = data; temp_vram_addr = (temp_vram_addr & 0xF3FF) | ((data & 3) << 10); break;
    case 0x0001: reg.mask = data; break;
    case 0x0003: reg.oam_addr = data; break;
    case 0x0004: ((Byte*)OAM)[reg.oam_addr] = data; reg.oam_addr++; break;
    case 0x0005: if (!address_latch) { fine_x = data & 7; temp_vram_addr = (temp_vram_addr & 0xFFE0) | (data >> 3); address_latch = 1; }
               else { temp_vram_addr = (temp_vram_addr & 0x0C1F) | ((data & 7) << 12) | ((data & 0xF8) << 2); address_latch = 0; } break;
    case 0x0006: if (!address_latch) { temp_vram_addr = (temp_vram_addr & 0x00FF) | ((data & 0x3F) << 8); address_latch = 1; }
               else { temp_vram_addr = (temp_vram_addr & 0xFF00) | data; vram_addr = temp_vram_addr; address_latch = 0; } break;
    case 0x0007: ppuWrite(vram_addr, data); vram_addr += (reg.control & 4) ? 32 : 1; break;
    }
}
Byte PPU::ppuRead(Word addr) {
    addr &= 0x3FFF;
    Byte data = 0;
    if (cart->ppuRead(addr, data)) return data;
    if (addr >= 0x2000 && addr <= 0x3EFF) {
        addr &= 0x0FFF;
        MIRROR m = cart->GetMirror();
        if (m == VERTICAL) {
            if (addr & 0x0400) return tblName[1][addr & 0x3FF];
            else                return tblName[0][addr & 0x3FF];
        }
        else if (m == HORIZONTAL) {
            if (addr & 0x0800) return tblName[1][addr & 0x3FF];
            else                return tblName[0][addr & 0x3FF];
        }
        else if (m == ONESCREEN_LO) return tblName[0][addr & 0x3FF];
        else if (m == ONESCREEN_HI) return tblName[1][addr & 0x3FF];
    }
    else if (addr >= 0x3F00 && addr <= 0x3FFF) {
        addr &= 0x001F;
        if (addr == 0x0010 || addr == 0x0014 || addr == 0x0018 || addr == 0x001C) addr -= 0x10;
        return tblPalette[addr] & (reg.mask & 1 ? 0x30 : 0x3F);
    }
    return 0;
}
void PPU::ppuWrite(Word addr, Byte data) {
    addr &= 0x3FFF;
    if (cart->ppuWrite(addr, data)) return;
    if (addr >= 0x2000 && addr <= 0x3EFF) {
        addr &= 0x0FFF;
        MIRROR m = cart->GetMirror();
        if (m == VERTICAL) {
            if (addr & 0x0400) tblName[1][addr & 0x3FF] = data;
            else                tblName[0][addr & 0x3FF] = data;
        }
        else if (m == HORIZONTAL) {
            if (addr & 0x0800) tblName[1][addr & 0x3FF] = data;
            else                tblName[0][addr & 0x3FF] = data;
        }
        else if (m == ONESCREEN_LO) tblName[0][addr & 0x3FF] = data;
        else if (m == ONESCREEN_HI) tblName[1][addr & 0x3FF] = data;
    }
    else if (addr >= 0x3F00 && addr <= 0x3FFF) {
        addr &= 0x001F;
        if (addr == 0x0010 || addr == 0x0014 || addr == 0x0018 || addr == 0x001C) addr -= 0x10;
        tblPalette[addr] = data;
    }
}
void PPU::DrawScanline() {
    if (!screenBuffer) return;
    int y = scanline;
    bool bgOpaque[256];
    memset(bgOpaque, 0, 256);
    int coarse_x = vram_addr & 0x1F; int coarse_y = (vram_addr >> 5) & 0x1F; int fine_y = (vram_addr >> 12) & 7; int nametable = (vram_addr >> 10) & 3;
    int sx_base = (coarse_x << 3) | fine_x; if (nametable & 1) sx_base += 256;
    int sy_base = (coarse_y << 3) | fine_y; if (nametable & 2) sy_base += 240;
    if (reg.mask & 0x08) {
        bool maskLeft = !(reg.mask & 2);
        for (int x = 0; x < 256; x++) {
            if (x < 8 && maskLeft) { screenBuffer[y * 256 + x] = 0xFF000000; continue; }
            int scx = (sx_base + x) % 512;
            int scy = sy_base % 480;
            int ntIdx = (scx >= 256 ? 1 : 0) + (scy >= 240 ? 2 : 0);
            int col = (scx % 256) / 8;
            int row = (scy % 240) / 8;
            Byte tile = ppuRead(0x2000 + (ntIdx * 0x400) + (row * 32) + col);
            Byte attr = ppuRead(0x23C0 + (ntIdx * 0x400) + ((row / 4) * 8) + (col / 4));
            if ((row % 4) >= 2) attr >>= 4; if ((col % 4) >= 2) attr >>= 2; attr &= 3;
            Word pAddr = ((reg.control & 0x10) ? 0x1000 : 0) + (tile * 16) + (scy % 8);
            Byte lo = ppuRead(pAddr);
            Byte hi = ppuRead(pAddr + 8);
            int bit = 7 - (scx % 8);
            Byte px = ((lo >> bit) & 1) | (((hi >> bit) & 1) << 1);
            if (px) {
                bgOpaque[x] = true;
                screenBuffer[y * 256 + x] = PALETTE[ppuRead(0x3F00 + (attr << 2) + px) & 0x3F];
            }
            else screenBuffer[y * 256 + x] = PALETTE[ppuRead(0x3F00) & 0x3F];
        }
    }
    bool spriteFound = false;
    if (reg.mask & 0x10) {
        bool maskLeft = !(reg.mask & 4);
        for (int i = 63; i >= 0; i--) {
            OAMEntry& s = OAM[i]; int sy = s.y + 1, h = (reg.control & 0x20) ? 16 : 8;
            if (y >= sy && y < sy + h) {
                spriteFound = true;
                int row = (s.attribute & 0x80) ? (h - 1) - (y - sy) : y - sy;
                Word pAddr;
                if (h == 16) pAddr = ((s.id & 1) ? 0x1000 : 0) + ((s.id & 0xFE) * 16) + row + (row >= 8 ? 8 : 0);
                else         pAddr = ((reg.control & 8) ? 0x1000 : 0) + (s.id * 16) + row;
                Byte lo = ppuRead(pAddr), hi = ppuRead(pAddr + 8), attr = s.attribute & 3; bool prio = !(s.attribute & 0x20);
                for (int px = 0; px < 8; px++) {
                    int sx = s.x + px; if (sx < 0 || sx >= 256 || (sx < 8 && maskLeft)) continue;
                    int bit = (s.attribute & 0x40) ? px : (7 - px); Byte pix = ((lo >> bit) & 1) | (((hi >> bit) & 1) << 1);
                    if (pix) {
                        if (i == 0 && bgOpaque[sx] && sx < 255 && (reg.mask & 0x18) == 0x18 && !((sx < 8) && (!(reg.mask & 2) || !(reg.mask & 4)))) reg.status |= 0x40;
                        if (prio || !bgOpaque[sx]) screenBuffer[y * 256 + sx] = PALETTE[ppuRead(0x3F10 + (attr << 2) + pix) & 0x3F];
                    }
                }
            }
        }
    }
    if ((reg.mask & 0x18) && !spriteFound) {
        ppuRead(0x1000);
    }
}
void PPU::Clock() {
    if (scanline >= -1 && scanline < 240) {
        if (scanline == 0 && cycle == 0) cycle = 0;
        if (scanline == -1 && cycle == 304 && (reg.mask & 0x18)) { vram_addr = (vram_addr & 0x841F) | (temp_vram_addr & 0x7BE0); }
        if (scanline > -1 && cycle == 256) {
            DrawScanline();
            if (reg.mask & 0x18) {
                if ((vram_addr & 0x7000) != 0x7000) vram_addr += 0x1000;
                else { vram_addr &= ~0x7000; int y = (vram_addr & 0x03E0) >> 5; if (y == 29) { y = 0; vram_addr ^= 0x0800; } else if (y == 31) y = 0; else y++; vram_addr = (vram_addr & ~0x03E0) | (y << 5); }
            }
        }
        if (scanline > -1 && cycle == 257 && (reg.mask & 0x18)) { vram_addr = (vram_addr & 0xFBE0) | (temp_vram_addr & 0x041F); }
    }
    if (scanline == 241 && cycle == 1) { reg.status |= 0x80; if (reg.control & 0x80) nmi = true; }
    if (scanline == 261 && cycle == 1) { reg.status &= ~0xC0; reg.status &= ~0x40; frameComplete = true; }
    if (++cycle >= 341) { cycle = 0; if (++scanline >= 262) { scanline = -1; frameComplete = false; } }
}
class CPU {
public:
    uint8_t A = 0, X = 0, Y = 0, S = 0xFD, P = 0;
    uint16_t PC = 0;
    int cyclesLeft = 0;
    Bus* bus = nullptr;
    enum FLAGS { C = (1 << 0), Z = (1 << 1), I = (1 << 2), D = (1 << 3), B = (1 << 4), U = (1 << 5), V = (1 << 6), N = (1 << 7) };
    CPU() {}
    void ConnectBus(Bus* n) { bus = n; }
    uint8_t Read(uint16_t addr) { return bus->cpuRead(addr); }
    void Write(uint16_t addr, uint8_t data) { bus->cpuWrite(addr, data); }
    void Push(uint8_t v) { Write(0x100 + S, v); S--; }
    uint8_t Pop() { S++; return Read(0x100 + S); }
    void SetFlag(FLAGS f, bool v) { if (v) P |= f; else P &= ~f; }
    void UpdateZN(uint8_t v) { SetFlag(Z, v == 0); SetFlag(N, v & 0x80); }
    void Reset() { A = 0; X = 0; Y = 0; S = 0xFD; P = 0 | U | I; uint16_t lo = Read(0xFFFC); uint16_t hi = Read(0xFFFD); PC = (hi << 8) | lo; cyclesLeft = 0; }
    void IRQ() {
        if (!(P & I)) {
            Push((PC >> 8) & 0xFF);
            Push(PC & 0xFF);
            uint8_t statusToPush = P;
            statusToPush &= ~B;
            statusToPush |= U;
            Push(statusToPush);
            SetFlag(I, true);
            uint16_t lo = Read(0xFFFE);
            uint16_t hi = Read(0xFFFF);
            PC = (hi << 8) | lo;
            cyclesLeft -= 7;
        }
    }
    void NMI() {
        Push((PC >> 8) & 0xFF);
        Push(PC & 0xFF);
        uint8_t statusToPush = P;
        statusToPush &= ~B;
        statusToPush |= U;
        Push(statusToPush);
        SetFlag(I, true);
        uint16_t lo = Read(0xFFFA);
        uint16_t hi = Read(0xFFFB);
        PC = (hi << 8) | lo;
        cyclesLeft -= 8;
    }
    inline bool CheckPageCross(uint16_t a, uint16_t b) { return (a & 0xFF00) != (b & 0xFF00); }
    inline uint16_t GetAddr_ZP() { return Read(PC++); }
    inline uint16_t GetAddr_ZPX() { return (Read(PC++) + X) & 0xFF; }
    inline uint16_t GetAddr_ZPY() { return (Read(PC++) + Y) & 0xFF; }
    inline uint16_t GetAddr_ABS() { uint16_t lo = Read(PC++); uint16_t hi = Read(PC++); return (hi << 8) | lo; }
    inline uint16_t GetAddr_ABX(bool& pageCrossed) { uint16_t base = GetAddr_ABS(); uint16_t addr = base + X; if (CheckPageCross(base, addr)) pageCrossed = true; return addr; }
    inline uint16_t GetAddr_ABY(bool& pageCrossed) { uint16_t base = GetAddr_ABS(); uint16_t addr = base + Y; if (CheckPageCross(base, addr)) pageCrossed = true; return addr; }
    inline uint16_t GetAddr_IZX() { uint16_t t = Read(PC++); uint16_t lo = Read((uint16_t)(t + X) & 0xFF); uint16_t hi = Read((uint16_t)(t + X + 1) & 0xFF); return (hi << 8) | lo; }
    inline uint16_t GetAddr_IZY(bool& pageCrossed) { uint16_t t = Read(PC++); uint16_t lo = Read(t & 0xFF); uint16_t hi = Read((t + 1) & 0xFF); uint16_t base = (hi << 8) | lo; uint16_t addr = base + Y; if (CheckPageCross(base, addr)) pageCrossed = true; return addr; }
    void Op_ADC(uint8_t v) { uint16_t t = (uint16_t)A + v + (P & C ? 1 : 0); SetFlag(C, t > 255); SetFlag(Z, (t & 0xFF) == 0); SetFlag(N, t & 0x80); SetFlag(V, (~((uint16_t)A ^ (uint16_t)v) & ((uint16_t)A ^ t)) & 0x0080); A = (uint8_t)t; }
    void Op_SBC(uint8_t v) { Op_ADC(v ^ 0xFF); }
    void Op_AND(uint8_t v) { A &= v; UpdateZN(A); }
    void Op_ORA(uint8_t v) { A |= v; UpdateZN(A); }
    void Op_EOR(uint8_t v) { A ^= v; UpdateZN(A); }
    void Op_CMP(uint8_t reg, uint8_t v) { uint16_t t = (uint16_t)reg - v; SetFlag(C, reg >= v); SetFlag(Z, (t & 0xFF) == 0); SetFlag(N, t & 0x80); }
    void Op_BIT(uint8_t v) { SetFlag(Z, (A & v) == 0); SetFlag(N, v & 0x80); SetFlag(V, v & 0x40); }
    void Op_INC(uint16_t addr) { uint8_t v = Read(addr); v++; Write(addr, v); UpdateZN(v); }
    void Op_DEC(uint16_t addr) { uint8_t v = Read(addr); v--; Write(addr, v); UpdateZN(v); }
    void Op_ASL(uint16_t addr) { uint8_t v = Read(addr); SetFlag(C, v & 0x80); v <<= 1; Write(addr, v); UpdateZN(v); }
    void Op_LSR(uint16_t addr) { uint8_t v = Read(addr); SetFlag(C, v & 1); v >>= 1; Write(addr, v); UpdateZN(v); }
    void Op_ROL(uint16_t addr) { uint8_t v = Read(addr); uint16_t t = ((uint16_t)v << 1) | (P & C ? 1 : 0); SetFlag(C, t > 255); v = (uint8_t)t; Write(addr, v); UpdateZN(v); }
    void Op_ROR(uint16_t addr) { uint8_t v = Read(addr); uint16_t t = ((uint16_t)v >> 1) | (P & C ? 0x80 : 0); SetFlag(C, v & 1); v = (uint8_t)t; Write(addr, v); UpdateZN(v); }
    void Op_ASL_A() { SetFlag(C, A & 0x80); A <<= 1; UpdateZN(A); }
    void Op_LSR_A() { SetFlag(C, A & 1); A >>= 1; UpdateZN(A); }
    void Op_ROL_A() { uint16_t t = ((uint16_t)A << 1) | (P & C ? 1 : 0); SetFlag(C, t > 255); A = (uint8_t)t; UpdateZN(A); }
    void Op_ROR_A() { uint16_t t = ((uint16_t)A >> 1) | (P & C ? 0x80 : 0); SetFlag(C, A & 1); A = (uint8_t)t; UpdateZN(A); }
    void Op_Branch(bool condition) { int8_t offset = (int8_t)Read(PC++); if (condition) { cyclesLeft -= 1; uint16_t newPC = PC + offset; if ((PC & 0xFF00) != (newPC & 0xFF00)) cyclesLeft -= 1; PC = newPC; } cyclesLeft -= 2; }
    void Op_LAX(uint8_t v) { A = X = v; UpdateZN(A); }
    void Op_SAX(uint16_t addr) { Write(addr, A & X); }
    void Op_DCP(uint16_t addr) { uint8_t v = Read(addr); v--; Write(addr, v); Op_CMP(A, v); }
    void Op_ISB(uint16_t addr) { uint8_t v = Read(addr); v++; Write(addr, v); Op_SBC(v); }
    void Op_SLO(uint16_t addr) { Op_ASL(addr); A |= Read(addr); UpdateZN(A); }
    void Op_RLA(uint16_t addr) { Op_ROL(addr); A &= Read(addr); UpdateZN(A); }
    void Op_SRE(uint16_t addr) { Op_LSR(addr); A ^= Read(addr); UpdateZN(A); }
    void Op_RRA(uint16_t addr) { Op_ROR(addr); Op_ADC(Read(addr)); }
    void Run(int cyclesToRun) {
        cyclesLeft += cyclesToRun;
        while (cyclesLeft > 0) {
            uint8_t opcode = Read(PC++);
            bool p = false;
            switch (opcode) {
            case 0xA9: A = Read(PC++); UpdateZN(A); cyclesLeft -= 2; break; case 0xA5: A = Read(GetAddr_ZP()); UpdateZN(A); cyclesLeft -= 3; break; case 0xB5: A = Read(GetAddr_ZPX()); UpdateZN(A); cyclesLeft -= 4; break; case 0xAD: A = Read(GetAddr_ABS()); UpdateZN(A); cyclesLeft -= 4; break; case 0xBD: A = Read(GetAddr_ABX(p)); UpdateZN(A); cyclesLeft -= 4; if (p) cyclesLeft--; break; case 0xB9: A = Read(GetAddr_ABY(p)); UpdateZN(A); cyclesLeft -= 4; if (p) cyclesLeft--; break; case 0xA1: A = Read(GetAddr_IZX()); UpdateZN(A); cyclesLeft -= 6; break; case 0xB1: A = Read(GetAddr_IZY(p)); UpdateZN(A); cyclesLeft -= 5; if (p) cyclesLeft--; break;
            case 0xA2: X = Read(PC++); UpdateZN(X); cyclesLeft -= 2; break; case 0xA6: X = Read(GetAddr_ZP()); UpdateZN(X); cyclesLeft -= 3; break; case 0xB6: X = Read(GetAddr_ZPY()); UpdateZN(X); cyclesLeft -= 4; break; case 0xAE: X = Read(GetAddr_ABS()); UpdateZN(X); cyclesLeft -= 4; break; case 0xBE: X = Read(GetAddr_ABY(p)); UpdateZN(X); cyclesLeft -= 4; if (p) cyclesLeft--; break;
            case 0xA0: Y = Read(PC++); UpdateZN(Y); cyclesLeft -= 2; break; case 0xA4: Y = Read(GetAddr_ZP()); UpdateZN(Y); cyclesLeft -= 3; break; case 0xB4: Y = Read(GetAddr_ZPX()); UpdateZN(Y); cyclesLeft -= 4; break; case 0xAC: Y = Read(GetAddr_ABS()); UpdateZN(Y); cyclesLeft -= 4; break; case 0xBC: Y = Read(GetAddr_ABX(p)); UpdateZN(Y); cyclesLeft -= 4; if (p) cyclesLeft--; break;
            case 0x85: Write(GetAddr_ZP(), A); cyclesLeft -= 3; break; case 0x95: Write(GetAddr_ZPX(), A); cyclesLeft -= 4; break; case 0x8D: Write(GetAddr_ABS(), A); cyclesLeft -= 4; break; case 0x9D: Write(GetAddr_ABX(p), A); cyclesLeft -= 5; break; case 0x99: Write(GetAddr_ABY(p), A); cyclesLeft -= 5; break; case 0x81: Write(GetAddr_IZX(), A); cyclesLeft -= 6; break; case 0x91: Write(GetAddr_IZY(p), A); cyclesLeft -= 6; break;
            case 0x86: Write(GetAddr_ZP(), X); cyclesLeft -= 3; break; case 0x96: Write(GetAddr_ZPY(), X); cyclesLeft -= 4; break; case 0x8E: Write(GetAddr_ABS(), X); cyclesLeft -= 4; break;
            case 0x84: Write(GetAddr_ZP(), Y); cyclesLeft -= 3; break; case 0x94: Write(GetAddr_ZPX(), Y); cyclesLeft -= 4; break; case 0x8C: Write(GetAddr_ABS(), Y); cyclesLeft -= 4; break;
            case 0xAA: X = A; UpdateZN(X); cyclesLeft -= 2; break; case 0xA8: Y = A; UpdateZN(Y); cyclesLeft -= 2; break; case 0x8A: A = X; UpdateZN(A); cyclesLeft -= 2; break; case 0x98: A = Y; UpdateZN(A); cyclesLeft -= 2; break; case 0x9A: S = X; cyclesLeft -= 2; break; case 0xBA: X = S; UpdateZN(X); cyclesLeft -= 2; break;
            case 0x69: Op_ADC(Read(PC++)); cyclesLeft -= 2; break; case 0x65: Op_ADC(Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0x75: Op_ADC(Read(GetAddr_ZPX())); cyclesLeft -= 4; break; case 0x6D: Op_ADC(Read(GetAddr_ABS())); cyclesLeft -= 4; break; case 0x7D: Op_ADC(Read(GetAddr_ABX(p))); cyclesLeft -= 4; if (p) cyclesLeft--; break; case 0x79: Op_ADC(Read(GetAddr_ABY(p))); cyclesLeft -= 4; if (p) cyclesLeft--; break; case 0x61: Op_ADC(Read(GetAddr_IZX())); cyclesLeft -= 6; break; case 0x71: Op_ADC(Read(GetAddr_IZY(p))); cyclesLeft -= 5; if (p) cyclesLeft--; break;
            case 0xE9: Op_SBC(Read(PC++)); cyclesLeft -= 2; break; case 0xE5: Op_SBC(Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0xF5: Op_SBC(Read(GetAddr_ZPX())); cyclesLeft -= 4; break; case 0xED: Op_SBC(Read(GetAddr_ABS())); cyclesLeft -= 4; break; case 0xFD: Op_SBC(Read(GetAddr_ABX(p))); cyclesLeft -= 4; if (p) cyclesLeft--; break; case 0xF9: Op_SBC(Read(GetAddr_ABY(p))); cyclesLeft -= 4; if (p) cyclesLeft--; break; case 0xE1: Op_SBC(Read(GetAddr_IZX())); cyclesLeft -= 6; break; case 0xF1: Op_SBC(Read(GetAddr_IZY(p))); cyclesLeft -= 5; if (p) cyclesLeft--; break;
            case 0x29: Op_AND(Read(PC++)); cyclesLeft -= 2; break; case 0x25: Op_AND(Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0x35: Op_AND(Read(GetAddr_ZPX())); cyclesLeft -= 4; break; case 0x2D: Op_AND(Read(GetAddr_ABS())); cyclesLeft -= 4; break; case 0x3D: Op_AND(Read(GetAddr_ABX(p))); cyclesLeft -= 4; if (p) cyclesLeft--; break; case 0x39: Op_AND(Read(GetAddr_ABY(p))); cyclesLeft -= 4; if (p) cyclesLeft--; break; case 0x21: Op_AND(Read(GetAddr_IZX())); cyclesLeft -= 6; break; case 0x31: Op_AND(Read(GetAddr_IZY(p))); cyclesLeft -= 5; if (p) cyclesLeft--; break;
            case 0x09: Op_ORA(Read(PC++)); cyclesLeft -= 2; break; case 0x05: Op_ORA(Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0x15: Op_ORA(Read(GetAddr_ZPX())); cyclesLeft -= 4; break; case 0x0D: Op_ORA(Read(GetAddr_ABS())); cyclesLeft -= 4; break; case 0x1D: Op_ORA(Read(GetAddr_ABX(p))); cyclesLeft -= 4; if (p) cyclesLeft--; break; case 0x19: Op_ORA(Read(GetAddr_ABY(p))); cyclesLeft -= 4; if (p) cyclesLeft--; break; case 0x01: Op_ORA(Read(GetAddr_IZX())); cyclesLeft -= 6; break; case 0x11: Op_ORA(Read(GetAddr_IZY(p))); cyclesLeft -= 5; if (p) cyclesLeft--; break;
            case 0x49: Op_EOR(Read(PC++)); cyclesLeft -= 2; break; case 0x45: Op_EOR(Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0x55: Op_EOR(Read(GetAddr_ZPX())); cyclesLeft -= 4; break; case 0x4D: Op_EOR(Read(GetAddr_ABS())); cyclesLeft -= 4; break; case 0x5D: Op_EOR(Read(GetAddr_ABX(p))); cyclesLeft -= 4; if (p) cyclesLeft--; break; case 0x59: Op_EOR(Read(GetAddr_ABY(p))); cyclesLeft -= 4; if (p) cyclesLeft--; break; case 0x41: Op_EOR(Read(GetAddr_IZX())); cyclesLeft -= 6; break; case 0x51: Op_EOR(Read(GetAddr_IZY(p))); cyclesLeft -= 5; if (p) cyclesLeft--; break;
            case 0xC9: Op_CMP(A, Read(PC++)); cyclesLeft -= 2; break; case 0xC5: Op_CMP(A, Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0xD5: Op_CMP(A, Read(GetAddr_ZPX())); cyclesLeft -= 4; break; case 0xCD: Op_CMP(A, Read(GetAddr_ABS())); cyclesLeft -= 4; break; case 0xDD: Op_CMP(A, Read(GetAddr_ABX(p))); cyclesLeft -= 4; if (p) cyclesLeft--; break; case 0xD9: Op_CMP(A, Read(GetAddr_ABY(p))); cyclesLeft -= 4; if (p) cyclesLeft--; break; case 0xC1: Op_CMP(A, Read(GetAddr_IZX())); cyclesLeft -= 6; break; case 0xD1: Op_CMP(A, Read(GetAddr_IZY(p))); cyclesLeft -= 5; if (p) cyclesLeft--; break;
            case 0xE0: Op_CMP(X, Read(PC++)); cyclesLeft -= 2; break; case 0xE4: Op_CMP(X, Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0xEC: Op_CMP(X, Read(GetAddr_ABS())); cyclesLeft -= 4; break;
            case 0xC0: Op_CMP(Y, Read(PC++)); cyclesLeft -= 2; break; case 0xC4: Op_CMP(Y, Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0xCC: Op_CMP(Y, Read(GetAddr_ABS())); cyclesLeft -= 4; break;
            case 0x24: Op_BIT(Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0x2C: Op_BIT(Read(GetAddr_ABS())); cyclesLeft -= 4; break;
            case 0xE6: Op_INC(GetAddr_ZP()); cyclesLeft -= 5; break; case 0xF6: Op_INC(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0xEE: Op_INC(GetAddr_ABS()); cyclesLeft -= 6; break; case 0xFE: Op_INC(GetAddr_ABX(p)); cyclesLeft -= 7; break; case 0xE8: X++; UpdateZN(X); cyclesLeft -= 2; break; case 0xC8: Y++; UpdateZN(Y); cyclesLeft -= 2; break;
            case 0xC6: Op_DEC(GetAddr_ZP()); cyclesLeft -= 5; break; case 0xD6: Op_DEC(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0xCE: Op_DEC(GetAddr_ABS()); cyclesLeft -= 6; break; case 0xDE: Op_DEC(GetAddr_ABX(p)); cyclesLeft -= 7; break; case 0xCA: X--; UpdateZN(X); cyclesLeft -= 2; break; case 0x88: Y--; UpdateZN(Y); cyclesLeft -= 2; break;
            case 0x0A: Op_ASL_A(); cyclesLeft -= 2; break; case 0x06: Op_ASL(GetAddr_ZP()); cyclesLeft -= 5; break; case 0x16: Op_ASL(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0x0E: Op_ASL(GetAddr_ABS()); cyclesLeft -= 6; break; case 0x1E: Op_ASL(GetAddr_ABX(p)); cyclesLeft -= 7; break;
            case 0x4A: Op_LSR_A(); cyclesLeft -= 2; break; case 0x46: Op_LSR(GetAddr_ZP()); cyclesLeft -= 5; break; case 0x56: Op_LSR(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0x4E: Op_LSR(GetAddr_ABS()); cyclesLeft -= 6; break; case 0x5E: Op_LSR(GetAddr_ABX(p)); cyclesLeft -= 7; break;
            case 0x2A: Op_ROL_A(); cyclesLeft -= 2; break; case 0x26: Op_ROL(GetAddr_ZP()); cyclesLeft -= 5; break; case 0x36: Op_ROL(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0x2E: Op_ROL(GetAddr_ABS()); cyclesLeft -= 6; break; case 0x3E: Op_ROL(GetAddr_ABX(p)); cyclesLeft -= 7; break;
            case 0x6A: Op_ROR_A(); cyclesLeft -= 2; break; case 0x66: Op_ROR(GetAddr_ZP()); cyclesLeft -= 5; break; case 0x76: Op_ROR(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0x6E: Op_ROR(GetAddr_ABS()); cyclesLeft -= 6; break; case 0x7E: Op_ROR(GetAddr_ABX(p)); cyclesLeft -= 7; break;
            case 0x10: Op_Branch(!(P & N)); break; case 0x30: Op_Branch(!!(P & N)); break; case 0x50: Op_Branch(!(P & V)); break; case 0x70: Op_Branch(!!(P & V)); break; case 0x90: Op_Branch(!(P & C)); break; case 0xB0: Op_Branch(!!(P & C)); break; case 0xD0: Op_Branch(!(P & Z)); break; case 0xF0: Op_Branch(!!(P & Z)); break;
            case 0x4C: PC = GetAddr_ABS(); cyclesLeft -= 3; break; case 0x6C: { uint16_t a = GetAddr_ABS(); uint16_t lo = Read(a); uint16_t hi = Read((a & 0xFF00) | ((a + 1) & 0x00FF)); PC = (hi << 8) | lo; cyclesLeft -= 5; } break;
            case 0x20: { uint16_t t = GetAddr_ABS(); Push(((PC - 1) >> 8) & 0xFF); Push((PC - 1) & 0xFF); PC = t; cyclesLeft -= 6; } break; case 0x60: { uint16_t lo = Pop(); uint16_t hi = Pop(); PC = ((hi << 8) | lo) + 1; cyclesLeft -= 6; } break;
            case 0x00: { PC++; Push((PC >> 8) & 0xFF); Push(PC & 0xFF); Push(P | U | B); SetFlag(I, true); uint16_t lo = Read(0xFFFE); uint16_t hi = Read(0xFFFF); PC = (hi << 8) | lo; cyclesLeft -= 7; } break; case 0x40: { P = Pop(); SetFlag(U, true); uint16_t lo = Pop(); uint16_t hi = Pop(); PC = (hi << 8) | lo; cyclesLeft -= 6; } break;
            case 0x48: Push(A); cyclesLeft -= 3; break; case 0x08: Push(P | B | U); cyclesLeft -= 3; break; case 0x68: A = Pop(); UpdateZN(A); cyclesLeft -= 4; break; case 0x28: P = Pop(); SetFlag(U, true); cyclesLeft -= 4; break;
            case 0x18: SetFlag(C, false); cyclesLeft -= 2; break; case 0x38: SetFlag(C, true); cyclesLeft -= 2; break; case 0x58: SetFlag(I, false); cyclesLeft -= 2; break; case 0x78: SetFlag(I, true); cyclesLeft -= 2; break; case 0xB8: SetFlag(V, false); cyclesLeft -= 2; break; case 0xD8: SetFlag(D, false); cyclesLeft -= 2; break; case 0xF8: SetFlag(D, true); cyclesLeft -= 2; break; case 0xEA: cyclesLeft -= 2; break;
            case 0xA7: Op_LAX(Read(GetAddr_ZP())); cyclesLeft -= 3; break; case 0xB7: Op_LAX(Read(GetAddr_ZPY())); cyclesLeft -= 4; break; case 0xAF: Op_LAX(Read(GetAddr_ABS())); cyclesLeft -= 4; break; case 0xBF: Op_LAX(Read(GetAddr_ABY(p))); cyclesLeft -= 4; if (p) cyclesLeft--; break; case 0xA3: Op_LAX(Read(GetAddr_IZX())); cyclesLeft -= 6; break; case 0xB3: Op_LAX(Read(GetAddr_IZY(p))); cyclesLeft -= 5; if (p) cyclesLeft--; break;
            case 0x87: Op_SAX(GetAddr_ZP()); cyclesLeft -= 3; break; case 0x97: Op_SAX(GetAddr_ZPY()); cyclesLeft -= 4; break; case 0x8F: Op_SAX(GetAddr_ABS()); cyclesLeft -= 4; break; case 0x83: Op_SAX(GetAddr_IZX()); cyclesLeft -= 6; break;
            case 0xC7: Op_DCP(GetAddr_ZP()); cyclesLeft -= 5; break; case 0xD7: Op_DCP(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0xCF: Op_DCP(GetAddr_ABS()); cyclesLeft -= 6; break; case 0xDF: Op_DCP(GetAddr_ABX(p)); cyclesLeft -= 7; break; case 0xDB: Op_DCP(GetAddr_ABY(p)); cyclesLeft -= 7; break; case 0xC3: Op_DCP(GetAddr_IZX()); cyclesLeft -= 8; break; case 0xD3: Op_DCP(GetAddr_IZY(p)); cyclesLeft -= 8; break;
            case 0xE7: Op_ISB(GetAddr_ZP()); cyclesLeft -= 5; break; case 0xF7: Op_ISB(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0xEF: Op_ISB(GetAddr_ABS()); cyclesLeft -= 6; break; case 0xFF: Op_ISB(GetAddr_ABX(p)); cyclesLeft -= 7; break; case 0xFB: Op_ISB(GetAddr_ABY(p)); cyclesLeft -= 7; break; case 0xE3: Op_ISB(GetAddr_IZX()); cyclesLeft -= 8; break; case 0xF3: Op_ISB(GetAddr_IZY(p)); cyclesLeft -= 8; break;
            case 0x07: Op_SLO(GetAddr_ZP()); cyclesLeft -= 5; break; case 0x17: Op_SLO(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0x0F: Op_SLO(GetAddr_ABS()); cyclesLeft -= 6; break; case 0x1F: Op_SLO(GetAddr_ABX(p)); cyclesLeft -= 7; break; case 0x1B: Op_SLO(GetAddr_ABY(p)); cyclesLeft -= 7; break; case 0x03: Op_SLO(GetAddr_IZX()); cyclesLeft -= 8; break; case 0x13: Op_SLO(GetAddr_IZY(p)); cyclesLeft -= 8; break;
            case 0x27: Op_RLA(GetAddr_ZP()); cyclesLeft -= 5; break; case 0x37: Op_RLA(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0x2F: Op_RLA(GetAddr_ABS()); cyclesLeft -= 6; break; case 0x3F: Op_RLA(GetAddr_ABX(p)); cyclesLeft -= 7; break; case 0x3B: Op_RLA(GetAddr_ABY(p)); cyclesLeft -= 7; break; case 0x23: Op_RLA(GetAddr_IZX()); cyclesLeft -= 8; break; case 0x33: Op_RLA(GetAddr_IZY(p)); cyclesLeft -= 8; break;
            case 0x47: Op_SRE(GetAddr_ZP()); cyclesLeft -= 5; break; case 0x57: Op_SRE(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0x4F: Op_SRE(GetAddr_ABS()); cyclesLeft -= 6; break; case 0x5F: Op_SRE(GetAddr_ABX(p)); cyclesLeft -= 7; break; case 0x5B: Op_SRE(GetAddr_ABY(p)); cyclesLeft -= 7; break; case 0x43: Op_SRE(GetAddr_IZX()); cyclesLeft -= 8; break; case 0x53: Op_SRE(GetAddr_IZY(p)); cyclesLeft -= 8; break;
            case 0x67: Op_RRA(GetAddr_ZP()); cyclesLeft -= 5; break; case 0x77: Op_RRA(GetAddr_ZPX()); cyclesLeft -= 6; break; case 0x6F: Op_RRA(GetAddr_ABS()); cyclesLeft -= 6; break; case 0x7F: Op_RRA(GetAddr_ABX(p)); cyclesLeft -= 7; break; case 0x7B: Op_RRA(GetAddr_ABY(p)); cyclesLeft -= 7; break; case 0x63: Op_RRA(GetAddr_IZX()); cyclesLeft -= 8; break; case 0x73: Op_RRA(GetAddr_IZY(p)); cyclesLeft -= 8; break;
            case 0x1A: case 0x3A: case 0x5A: case 0x7A: case 0xDA: case 0xFA: cyclesLeft -= 2; break; case 0x80: case 0x82: case 0x89: case 0xC2: case 0xE2: Read(PC++); cyclesLeft -= 2; break; case 0x04: case 0x44: case 0x64: GetAddr_ZP(); cyclesLeft -= 3; break; case 0x14: case 0x34: case 0x54: case 0x74: case 0xD4: case 0xF4: GetAddr_ZPX(); cyclesLeft -= 4; break; case 0x0C: GetAddr_ABS(); cyclesLeft -= 4; break; case 0x1C: case 0x3C: case 0x5C: case 0x7C: case 0xDC: case 0xFC: GetAddr_ABX(p); cyclesLeft -= 4; if (p) cyclesLeft--; break; case 0xEB: Op_SBC(Read(PC++)); cyclesLeft -= 2; break;
            default: cyclesLeft -= 2; break;
            }
        }
    }
};
class NESCore {
public:
    Bus bus; CPU cpu; PPU ppu; APU apu;
    std::shared_ptr<Cartridge> cart;
    std::vector<uint32_t> displayBuffer;
    bool isLoaded = false;
    NESCore() {
        displayBuffer.resize(NES_WIDTH * NES_HEIGHT);
        bus.SetCPU(&cpu); bus.SetPPU(&ppu); bus.SetAPU(&apu);
        cpu.ConnectBus(&bus); ppu.ConnectBus(&bus); ppu.SetScreenBuffer(displayBuffer.data());
    }
    bool LoadRom(const std::wstring& path) {
        cart = std::make_shared<Cartridge>(&bus);
        if (!cart->Load(path)) return false;
        bus.SetCart(cart.get()); ppu.SetCart(cart.get());
        cpu.Reset(); ppu.Reset(); apu.Reset();
        return (isLoaded = true);
    }
    void InputKey(int id, bool pressed) { if (id >= 0 && id < 8) { if (pressed) bus.controller[0] |= (1 << id); else bus.controller[0] &= ~(1 << id); } }
    void StepFrame() {
        if (!isLoaded) return;
        const int CPU_CYCLES_PER_FRAME = 29780;
        int cyclesExecuted = 0;
        int step = 1;
        while (cyclesExecuted < CPU_CYCLES_PER_FRAME) {
            for (int i = 0; i < step * 3; i++) {
                ppu.Clock();
                if (ppu.nmi) {
                    ppu.nmi = false;
                    cpu.NMI();
                }
            }
            cart->Tick(step);
            if (cart->GetIRQ()) {
                cpu.IRQ();
            }
            if (bus.dma_transfer) {
                bus.dma_transfer = false;
                uint16_t offset = (uint16_t)bus.dma_page << 8;
                for (int i = 0; i < 256; i++) {
                    ((uint8_t*)ppu.OAM)[(bus.dma_addr + i) & 0xFF] = bus.cpuRead(offset + i);
                }
                int dma_cycles = 513;
                for (int i = 0; i < dma_cycles * 3; i++) {
                    ppu.Clock();
                    if (ppu.nmi) {
                        ppu.nmi = false;
                        cpu.NMI();
                    }
                }
                cyclesExecuted += dma_cycles;
                cart->Tick(dma_cycles);
            }
            else {
                cpu.Run(step);
                cyclesExecuted += step;
            }
        }
        ppu.frameComplete = false;
    }
};
class App {
private:
    HWND m_hwnd; ID2D1Factory* m_pDirect2dFactory; ID2D1HwndRenderTarget* m_pRenderTarget; ID2D1Bitmap* m_pBitmap;
    NESCore m_nesCore; AudioDriver m_audio; BOOL m_isFullscreen; WINDOWPLACEMENT m_wpPrev; HMENU m_hMenu;
public:
    App() : m_hwnd(NULL), m_pDirect2dFactory(NULL), m_pRenderTarget(NULL), m_pBitmap(NULL), m_isFullscreen(FALSE), m_hMenu(NULL) { ZeroMemory(&m_wpPrev, sizeof(m_wpPrev)); }
    ~App() { SafeRelease(&m_pBitmap); SafeRelease(&m_pRenderTarget); SafeRelease(&m_pDirect2dFactory); }
    HRESULT Initialize(HINSTANCE hInstance, int nCmdShow) {
        D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pDirect2dFactory);
        WNDCLASSEX wcex = { sizeof(WNDCLASSEX), CS_HREDRAW | CS_VREDRAW, App::WndProc, 0, sizeof(LONG_PTR), hInstance, 0, LoadCursor(NULL, IDC_ARROW), (HBRUSH)GetStockObject(BLACK_BRUSH), NULL, L"D2DNESWnd" };
        RegisterClassEx(&wcex);
        HMENU hMenu = CreateMenu(), hSubMenu = CreatePopupMenu();
        AppendMenu(hSubMenu, MF_STRING, IDM_FILE_OPEN, L"Open NES ROM..."); AppendMenu(hSubMenu, MF_STRING, IDM_FILE_FULLSCREEN, L"Fullscreen\tF11");
        AppendMenu(hSubMenu, MF_SEPARATOR, 0, NULL); AppendMenu(hSubMenu, MF_STRING, IDM_FILE_EXIT, L"Exit");
        AppendMenu(hMenu, MF_STRING | MF_POPUP, (UINT_PTR)hSubMenu, L"File");
        int w = NES_WIDTH * 3, h = NES_HEIGHT * 3; RECT rc = { 0, 0, w, h }; AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, TRUE);
        m_hwnd = CreateWindow(L"D2DNESWnd", L"NES Emulator", WS_OVERLAPPEDWINDOW, (GetSystemMetrics(SM_CXSCREEN) - (rc.right - rc.left)) / 2, (GetSystemMetrics(SM_CYSCREEN) - (rc.bottom - rc.top)) / 2, rc.right - rc.left, rc.bottom - rc.top, NULL, hMenu, hInstance, this);
        if (!m_hwnd) return E_FAIL;
        DragAcceptFiles(m_hwnd, TRUE); m_hMenu = GetMenu(m_hwnd); ShowWindow(m_hwnd, nCmdShow); UpdateWindow(m_hwnd); m_audio.Initialize(m_hwnd);
        return S_OK;
    }
    void OpenRomFile(const std::wstring& path) {
        std::wstring p = path; if (!p.empty() && p.front() == L'\"') p.erase(0, 1); if (!p.empty() && p.back() == L'\"') p.pop_back();
        if (m_nesCore.LoadRom(p)) {
            size_t s = p.find_last_of(L"\\/"), d = p.find_last_of(L"."); std::wstring t = L"NES Emulator - " + ((s == std::wstring::npos) ? p : p.substr(s + 1)).substr(0, (d != std::wstring::npos) ? d : std::wstring::npos);
            SetWindowText(m_hwnd, t.c_str());
        }
    }
    void OnDropFiles(HDROP hDrop) { wchar_t f[MAX_PATH]; if (DragQueryFile(hDrop, 0, f, MAX_PATH) > 0) { OpenRomFile(f); SetForegroundWindow(m_hwnd); SetFocus(m_hwnd); } DragFinish(hDrop); }
    void PauseAudio() { m_audio.Stop(); }
    void ResumeAudio() { if (m_nesCore.isLoaded) m_audio.Resume(); }
    void ToggleFullscreen() {
        DWORD s = GetWindowLong(m_hwnd, GWL_STYLE);
        if (m_isFullscreen) {
            SetWindowLong(m_hwnd, GWL_STYLE, s | WS_OVERLAPPEDWINDOW); SetWindowPlacement(m_hwnd, &m_wpPrev);
            SetWindowPos(m_hwnd, NULL, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_FRAMECHANGED);
            SetMenu(m_hwnd, m_hMenu); CheckMenuItem(m_hMenu, IDM_FILE_FULLSCREEN, MF_UNCHECKED); ShowCursor(TRUE); m_isFullscreen = FALSE;
        }
        else {
            m_wpPrev.length = sizeof(WINDOWPLACEMENT); GetWindowPlacement(m_hwnd, &m_wpPrev);
            SetWindowLong(m_hwnd, GWL_STYLE, s & ~WS_OVERLAPPEDWINDOW); CheckMenuItem(m_hMenu, IDM_FILE_FULLSCREEN, MF_CHECKED); SetMenu(m_hwnd, NULL);
            HMONITOR hM = MonitorFromWindow(m_hwnd, MONITOR_DEFAULTTOPRIMARY); MONITORINFO mi = { sizeof(MONITORINFO) }; GetMonitorInfo(hM, &mi);
            SetWindowPos(m_hwnd, HWND_TOP, mi.rcMonitor.left, mi.rcMonitor.top, mi.rcMonitor.right - mi.rcMonitor.left, mi.rcMonitor.bottom - mi.rcMonitor.top, SWP_NOOWNERZORDER | SWP_FRAMECHANGED);
            ShowCursor(FALSE); m_isFullscreen = TRUE;
        }
    }
    void RunMessageLoop() {
        timeBeginPeriod(1);
        MSG msg = { 0 };
        LARGE_INTEGER f, l, c;
        QueryPerformanceFrequency(&f);
        QueryPerformanceCounter(&l);
        const double SPF = 1.0 / 60.0988;
        const int CPF = (int)(NES_CPU_CLOCK / 60.0988);
        while (true) {
            if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
                if (msg.message == WM_QUIT) break;
                TranslateMessage(&msg);
                DispatchMessage(&msg);
                continue;
            }
            QueryPerformanceCounter(&c);
            double el = (double)(c.QuadPart - l.QuadPart) / f.QuadPart;
            if (el >= SPF) {
                l.QuadPart += (LONGLONG)(SPF * f.QuadPart);
                if ((double)(c.QuadPart - l.QuadPart) / f.QuadPart > SPF) {
                    l.QuadPart = c.QuadPart;
                }
                if (m_nesCore.isLoaded) {
                    m_nesCore.StepFrame();
                    m_nesCore.apu.Step(CPF);
                    if (!m_nesCore.apu.buffer.empty()) {
                        m_audio.PushSamples(m_nesCore.apu.buffer);
                        m_nesCore.apu.buffer.clear();
                    }
                }
                OnRender();
            }
            else {
                if (SPF - el > 0.002) Sleep(1);
            }
        }
        timeEndPeriod(1);
    }
    void CreateDeviceResources() {
        if (!m_pRenderTarget) {
            RECT rc; GetClientRect(m_hwnd, &rc);
            m_pDirect2dFactory->CreateHwndRenderTarget(D2D1::RenderTargetProperties(), D2D1::HwndRenderTargetProperties(m_hwnd, D2D1::SizeU(rc.right - rc.left, rc.bottom - rc.top)), &m_pRenderTarget);
            if (m_pRenderTarget) {
                D2D1_BITMAP_PROPERTIES p = { D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE), 96.0f, 96.0f };
                m_pRenderTarget->CreateBitmap(D2D1::SizeU(NES_WIDTH, NES_HEIGHT), p, &m_pBitmap);
            }
        }
    }
    void OnRender() {
        CreateDeviceResources();
        if (m_pRenderTarget && !(m_pRenderTarget->CheckWindowState() & D2D1_WINDOW_STATE_OCCLUDED)) {
            m_pRenderTarget->BeginDraw(); m_pRenderTarget->Clear(D2D1::ColorF(D2D1::ColorF::Black));
            if (m_pBitmap) {
                m_pBitmap->CopyFromMemory(NULL, m_nesCore.displayBuffer.data(), NES_WIDTH * sizeof(uint32_t));
                D2D1_SIZE_F s = m_pRenderTarget->GetSize(); float sc = (std::min)(s.width / NES_WIDTH, s.height / NES_HEIGHT), dw = NES_WIDTH * sc, dh = NES_HEIGHT * sc, ox = (s.width - dw) / 2.0f, oy = (s.height - dh) / 2.0f;
                m_pRenderTarget->DrawBitmap(m_pBitmap, D2D1::RectF(ox, oy, ox + dw, oy + dh), 1.0f, D2D1_BITMAP_INTERPOLATION_MODE_NEAREST_NEIGHBOR, NULL);
            }
            if (m_pRenderTarget->EndDraw() == D2DERR_RECREATE_TARGET) { SafeRelease(&m_pBitmap); SafeRelease(&m_pRenderTarget); }
        }
    }
    static LRESULT CALLBACK WndProc(HWND h, UINT m, WPARAM w, LPARAM l) {
        App* p = reinterpret_cast<App*>(GetWindowLongPtr(h, GWLP_USERDATA));
        switch (m) {
        case WM_CREATE: SetWindowLongPtr(h, GWLP_USERDATA, (LONG_PTR)((LPCREATESTRUCT)l)->lpCreateParams); return 0;
        case WM_ENTERMENULOOP: case WM_ENTERSIZEMOVE: if (p) p->PauseAudio(); return 0;
        case WM_EXITMENULOOP: case WM_EXITSIZEMOVE: if (p) p->ResumeAudio(); return 0;
        case WM_COMMAND:
            if (LOWORD(w) == IDM_FILE_OPEN && p) {
                p->PauseAudio(); OPENFILENAME ofn = { sizeof(ofn) }; wchar_t f[260] = { 0 }; ofn.hwndOwner = h; ofn.lpstrFile = f; ofn.nMaxFile = 260; ofn.lpstrFilter = L"NES ROMs\0*.nes\0All Files\0*.*\0"; ofn.nFilterIndex = 1; ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
                if (GetOpenFileName(&ofn)) p->OpenRomFile(f);
                p->ResumeAudio();
            }
            if (LOWORD(w) == IDM_FILE_EXIT) DestroyWindow(h);
            if (LOWORD(w) == IDM_FILE_FULLSCREEN && p) p->ToggleFullscreen();
            return 0;
        case WM_NCHITTEST: { LRESULT r = DefWindowProc(h, m, w, l); return (r == HTCLIENT && p && !p->m_isFullscreen) ? HTCAPTION : r; }
        case WM_SIZE: if (p && p->m_pRenderTarget) p->m_pRenderTarget->Resize(D2D1::SizeU(LOWORD(l), HIWORD(l))); return 0;
        case WM_KEYDOWN: case WM_KEYUP: if (p) {
            if (m == WM_KEYDOWN && w == VK_F11) { p->ToggleFullscreen(); return 0; }
            if (m == WM_KEYDOWN && w == VK_ESCAPE && p->m_isFullscreen) { p->ToggleFullscreen(); return 0; }
        int id = -1; switch (w) { case 'X': id = 0; break; case 'Z': id = 1; break; case VK_SHIFT: id = 2; break; case VK_RETURN: id = 3; break; case VK_UP: id = 4; break; case VK_DOWN: id = 5; break; case VK_LEFT: id = 6; break; case VK_RIGHT: id = 7; break; }
                                          p->m_nesCore.InputKey(id, m == WM_KEYDOWN);
        } return 0;
        case WM_DROPFILES: if (p) p->OnDropFiles((HDROP)w); return 0;
        case WM_DESTROY: PostQuitMessage(0); return 0;
        }
        return DefWindowProc(h, m, w, l);
    }
};
int WINAPI wWinMain(_In_ HINSTANCE hI, _In_opt_ HINSTANCE, _In_ LPWSTR, _In_ int nC) {
    SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
    (void)CoInitialize(NULL); App a;
    if (SUCCEEDED(a.Initialize(hI, nC))) {
        int c; LPWSTR* v = CommandLineToArgvW(GetCommandLineW(), &c);
        if (v && c > 1) a.OpenRomFile(v[1]);
        LocalFree(v); a.RunMessageLoop();
    }
    CoUninitialize(); return 0;
}