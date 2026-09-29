#include "sound.h"

#include "log.h"

#include <windows.h>
#include <mmsystem.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#pragma comment(lib, "winmm.lib")

// See sound.h. PlaySound rather than XAudio2 on purpose: these are two short
// clips that never need to overlap each other, the game's own audio is not
// being touched, and a second audio engine in the process is a great deal of
// machinery and a great deal of new ways to crash for a click and a clack.

namespace {

constexpr int kMaxSounds = 4;
constexpr DWORD kMaxBytes = 4u * 1024u * 1024u; // a reload noise, not a soundtrack
// Only the synthesised fallbacks use this. A file brings its own rate, which
// is read from its header and passed through untouched - 48 kHz, 44.1, or
// anything else Windows can resample, which is all of them. 48 because that is
// what output devices run at now, so the fallback avoids a resample too.
constexpr int kRate = 48000;

struct Clip {
    char name[32];
    BYTE* data;   // as loaded or synthesised, never altered
    DWORD bytes;
    DWORD dataAt; // where the samples start inside it, and how many bytes
    DWORD dataBytes;
    unsigned short bits;
    BYTE* play;    // the scaled copy actually handed to PlaySound
    float playVol; // and the volume it was built at
    bool looked;   // tried to load, whether or not it worked
};

Clip g_clips[kMaxSounds];
SRWLOCK g_lock = SRWLOCK_INIT;
float g_volume = 0.7f;

// Walks the chunks to find the samples. Needed because a WAV is not obliged to
// put its data 44 bytes in - anything an editor felt like writing can sit in
// between - and scaling the wrong bytes would corrupt the header rather than
// quieten the sound.
bool FindSamples(const BYTE* buf, DWORD bytes, DWORD& at, DWORD& size, unsigned short& bits)
{
    at = 0;
    size = 0;
    bits = 0;
    if (bytes < 44 || std::memcmp(buf, "RIFF", 4) != 0 || std::memcmp(buf + 8, "WAVE", 4) != 0)
        return false;
    DWORD p = 12;
    while (p + 8 <= bytes) {
        const DWORD chunk = *reinterpret_cast<const DWORD*>(buf + p + 4);
        if (chunk > bytes - p - 8)
            break;
        if (std::memcmp(buf + p, "fmt ", 4) == 0 && chunk >= 16)
            bits = *reinterpret_cast<const unsigned short*>(buf + p + 8 + 14);
        else if (std::memcmp(buf + p, "data", 4) == 0) {
            at = p + 8;
            size = chunk;
        }
        p += 8 + chunk + (chunk & 1u);
    }
    return at != 0 && size != 0 && (bits == 8 || bits == 16);
}

// The folder the game is running from, which is where a player drops files.
void GameFolder(char* out, size_t size)
{
    out[0] = 0;
    if (!GetModuleFileNameA(nullptr, out, static_cast<DWORD>(size)))
        return;
    char* slash = std::strrchr(out, '\\');
    if (slash)
        slash[1] = 0;
}

Clip* Find(const char* name)
{
    for (Clip& c : g_clips)
        if (c.name[0] && _stricmp(c.name, name) == 0)
            return &c;
    return nullptr;
}

Clip* MakeRoom(const char* name)
{
    for (Clip& c : g_clips) {
        if (!c.name[0]) {
            _snprintf_s(c.name, sizeof(c.name), _TRUNCATE, "%s", name);
            c.data = nullptr;
            c.bytes = 0;
            c.looked = false;
            return &c;
        }
    }
    return nullptr;
}

// ---- Making the noise ourselves ----------------------------------------
//
// The mod ships no audio, and a magazine click does not need to be recorded:
// it is a hard attack and a short decay, which is three damped sines and a
// puff of noise. Synthesised here, so the feature works the moment it is
// switched on and owes nobody an attribution.
//
// A real one has a body and an edge. The body is the low thump of the
// magazine hitting its stop, the edge is the metal-on-metal of the catch, and
// the noise is what makes it sound like a mechanism rather than a tone. The
// insert is heavier and lower than the eject because it ends in a seat rather
// than a release.

unsigned g_rand = 0x1234567u;
float Noise()
{
    g_rand = g_rand * 1664525u + 1013904223u;
    return static_cast<float>(static_cast<int>(g_rand >> 8) & 0xFFFF) / 32768.0f - 1.0f;
}

// NO TONES (2026-09-26, user: "that sounds like a bottle pop - we wanna go for
// realistic sounds for both").
//
// Right, and the first attempt earned it. A damped sine at 150 Hz IS a bottle
// pop; that is what the sound is. Metal does not work like that. A magazine
// click has almost no tonal body at all - it is a dense broadband transient
// with resonance on top, which means the source has to be NOISE and the pitch
// has to come from filtering it, not from adding sine waves together.
//
// So every voice below is white noise through a two-pole resonant bandpass
// with its own envelope, and there is not a single oscillator in the file. Low
// Q reads as a rasp, high Q as ringing metal, and a short decay as a strike.
//
// The other thing that was wrong is that a magazine going in is not one event.
// It slides, and then it seats, and the gap between the two is most of what
// makes it sound like a magazine rather than a click. The eject is the other
// way round: the catch goes first and the magazine drops free afterwards.
// A STRUCK OBJECT, NOT A FILTERED BURST (2026-09-26, second pass).
//
// Noise through resonators got the material right and the event wrong. Two
// things give a synthetic click away, and neither is the spectrum.
//
// The first is that it is ONE impact. A real mechanism is a handful of
// collisions a few milliseconds apart - the catch, the part it throws, the
// thing that part lands on - and the ear hears that cluster as machinery. A
// single strike, however well filtered, hears as a sample of a strike.
//
// The second is that it is perfectly dry. Nothing in the world reaches a
// microphone without arriving twice, and a click with no early reflections at
// all sounds like it happened inside your head rather than in a room. Four
// sparse taps in the first twenty milliseconds is enough to fix that and far
// too short to read as reverb.
//
// So it is modelled the way the thing behaves: one excitation with several
// hits in it, one set of resonances the whole object rings at, a gentle roll
// off because metal is not a tweeter, and a very short tail.
struct Res {
    float hz, q, gain;
    float low, band; // filter state
};
struct Hit {
    float at, level, sharp; // when, how hard, and how long the excitation lasts
};

BYTE* Synthesise(const char* name, DWORD& outBytes)
{
    float seconds = 0.0f;
    Res r[6] = {};
    Hit h[6] = {};
    int nr = 0, nh = 0;
    // How much of the raw excitation is heard without going through a
    // resonator at all. This is the broadband snap of a collision, and leaving
    // it out is most of what makes a synthesised impact sound synthesised.
    float direct = 0.0f;
    int taps = 4;

    if (_stricmp(name, "magin") == 0) {
        seconds = 0.14f;
        // How the whole assembly rings: inharmonic, because a magazine is not
        // a bell, and weighted low because it is going into something heavy.
        direct = 0.15f;
        r[nr++] = { 700.0f, 4.0f, 0.45f, 0, 0 };
        r[nr++] = { 1250.0f, 6.0f, 0.60f, 0, 0 };
        r[nr++] = { 2100.0f, 8.0f, 0.50f, 0, 0 };
        r[nr++] = { 3300.0f, 9.0f, 0.28f, 0, 0 };
        r[nr++] = { 4800.0f, 12.0f, 0.12f, 0, 0 };
        // Sliding up the well for forty milliseconds, then seated - and the
        // seat is three collisions, not one.
        h[nh++] = { 0.000f, 0.13f, 0.0380f };
        h[nh++] = { 0.045f, 1.00f, 0.0011f };
        h[nh++] = { 0.0495f, 0.40f, 0.0009f };
        h[nh++] = { 0.0545f, 0.17f, 0.0007f };
    } else if (_stricmp(name, "dryfire") == 0) {
        // A TICK, NOT A PING (2026-09-26, user: "that sounds too synthesized.
        // We just want that tick sound effect").
        //
        // The first one rang, and ringing is the giveaway. High Q resonators
        // sustain, and a sustaining pitch at four kilohertz is a blip from a
        // synthesiser - a gun frame is a lump of heavily damped steel and does
        // not do that. So the Q comes down to where the resonances barely
        // survive the strike, the frequencies come down out of the glassy
        // region, and most of what you hear is DIRECT: unresonated noise, a
        // millisecond of it, which is what a snap actually is. The resonances
        // are left only to colour it.
        //
        // And it is short. The audible part of a real dry fire is about ten
        // milliseconds. Sixty was three quarters tail.
        seconds = 0.045f;
        direct = 0.55f;
        taps = 2;
        r[nr++] = { 520.0f, 2.5f, 0.25f, 0, 0 };
        r[nr++] = { 1150.0f, 3.0f, 0.35f, 0, 0 };
        r[nr++] = { 2300.0f, 4.0f, 0.22f, 0, 0 };
        r[nr++] = { 4200.0f, 5.0f, 0.10f, 0, 0 };
        h[nh++] = { 0.00000f, 1.00f, 0.00035f };
        h[nh++] = { 0.00160f, 0.25f, 0.00030f };
    } else if (_stricmp(name, "magout") == 0) {
        seconds = 0.11f;
        // Lighter and higher: a catch letting go is a small part moving, and
        // nothing heavy is being stopped.
        direct = 0.15f;
        r[nr++] = { 900.0f, 3.0f, 0.22f, 0, 0 };
        r[nr++] = { 1650.0f, 5.0f, 0.45f, 0, 0 };
        r[nr++] = { 2700.0f, 9.0f, 0.60f, 0, 0 };
        r[nr++] = { 4100.0f, 8.0f, 0.30f, 0, 0 };
        r[nr++] = { 5900.0f, 11.0f, 0.14f, 0, 0 };
        h[nh++] = { 0.000f, 1.00f, 0.0009f };
        h[nh++] = { 0.0035f, 0.34f, 0.0008f };
        h[nh++] = { 0.0080f, 0.15f, 0.0010f };
        // The magazine leaving the well afterwards, dragging as it goes.
        h[nh++] = { 0.0180f, 0.16f, 0.0260f };
    } else {
        return nullptr;
    }

    const int frames = static_cast<int>(seconds * kRate);
    const DWORD dataBytes = static_cast<DWORD>(frames) * 2u;
    const DWORD total = 44u + dataBytes;
    BYTE* buf = static_cast<BYTE*>(std::malloc(total));
    if (!buf)
        return nullptr;

    const auto put32 = [](BYTE* at, unsigned v) {
        at[0] = static_cast<BYTE>(v);
        at[1] = static_cast<BYTE>(v >> 8);
        at[2] = static_cast<BYTE>(v >> 16);
        at[3] = static_cast<BYTE>(v >> 24);
    };
    const auto put16 = [](BYTE* at, unsigned v) {
        at[0] = static_cast<BYTE>(v);
        at[1] = static_cast<BYTE>(v >> 8);
    };
    std::memcpy(buf, "RIFF", 4);
    put32(buf + 4, total - 8);
    std::memcpy(buf + 8, "WAVEfmt ", 8);
    put32(buf + 16, 16);           // PCM header size
    put16(buf + 20, 1);            // PCM
    put16(buf + 22, 1);            // mono
    put32(buf + 24, kRate);
    put32(buf + 28, kRate * 2);    // bytes per second
    put16(buf + 32, 2);            // block align
    put16(buf + 34, 16);           // bits
    std::memcpy(buf + 36, "data", 4);
    put32(buf + 40, dataBytes);

    constexpr float kPi = 3.14159265358979f;
    // Rendered to floats first so the result can be normalised. Filtered noise
    // has no predictable peak - it depends on where the noise happens to land
    // while a resonator is ringing - so guessing a scale either clips or comes
    // out quiet, and the answer is simply to measure it.
    float* mix = static_cast<float*>(std::malloc(sizeof(float) * static_cast<size_t>(frames)));
    if (!mix) {
        std::free(buf);
        return nullptr;
    }
    // The excitation is shared: one object, struck several times, rather than
    // several sounds played together. That is the whole difference between a
    // mechanism and a chord.
    float damper = 0.0f; // one pole roll off, because metal is not a tweeter
    for (int i = 0; i < frames; ++i) {
        const float t = static_cast<float>(i) / kRate;
        float exc = 0.0f;
        for (int j = 0; j < nh; ++j)
            if (t >= h[j].at)
                exc += h[j].level * std::exp(-(t - h[j].at) / h[j].sharp);
        const float in = Noise() * exc;
        float sum = 0.0f;
        for (int j = 0; j < nr; ++j) {
            Res& s = r[j];
            // Chamberlin state variable, taking the band output. f is capped
            // well below the rate where this form stops being stable.
            float f = 2.0f * std::sin(kPi * s.hz / kRate);
            if (f > 0.7f)
                f = 0.7f;
            const float high = in - s.low - s.band / s.q;
            s.band += f * high;
            s.low += f * s.band;
            sum += s.band * s.gain;
        }
        sum += in * direct;
        // About 7 kHz, which takes the glassy edge off without dulling it.
        damper += (sum - damper) * 0.62f;
        mix[i] = damper;
    }
    // Four early reflections in the first twenty milliseconds. Not reverb -
    // far too short and far too sparse to hear as a space - just enough that
    // the click arrives more than once, the way everything real does.
    {
        const float tapMs[4] = { 4.1f, 7.3f, 11.9f, 17.5f };
        const float tapGain[4] = { 0.17f, 0.12f, 0.075f, 0.042f };
        for (int k = 0; k < taps && k < 4; ++k) {
            const int d = static_cast<int>(tapMs[k] * 0.001f * kRate);
            for (int i = frames - 1; i >= d; --i)
                mix[i] += mix[i - d] * tapGain[k];
        }
    }
    float peak = 0.0f;
    for (int i = 0; i < frames; ++i) {
        const float t = static_cast<float>(i) / kRate;
        const float left = seconds - t;
        if (left < 0.004f)
            mix[i] *= left / 0.004f;
        const float mag = mix[i] < 0.0f ? -mix[i] : mix[i];
        if (mag > peak)
            peak = mag;
    }
    const float scale = peak > 1e-6f ? 27000.0f / peak : 0.0f;
    BYTE* out = buf + 44;
    for (int i = 0; i < frames; ++i) {
        int s = static_cast<int>(mix[i] * scale);
        if (s > 32767)
            s = 32767;
        if (s < -32768)
            s = -32768;
        put16(out + i * 2, static_cast<unsigned>(static_cast<short>(s)));
    }
    std::free(mix);
    outBytes = total;
    return buf;
}

// Reads the whole file. Small, once, and only the first time a name is asked
// for - after that the answer is remembered either way. A missing file is not
// a failure: the mod makes the noise itself.
void LoadIt(Clip& c)
{
    c.looked = true;
    char path[MAX_PATH];
    GameFolder(path, sizeof(path));
    char full[MAX_PATH];
    _snprintf_s(full, sizeof(full), _TRUNCATE, "%sre5vr_%s.wav", path, c.name);

    HANDLE f = CreateFileA(full, GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (f == INVALID_HANDLE_VALUE) {
        DWORD made = 0;
        c.data = Synthesise(c.name, made);
        c.bytes = made;
        if (c.data)
            Log_Printf("Sound: no %s, so %s is the mod's own - drop a WAV there to use your own", full, c.name);
        else
            Log_Printf("Sound: no %s and nothing to make in its place - %s will be silent", full, c.name);
        return;
    }
    LARGE_INTEGER size = {};
    if (!GetFileSizeEx(f, &size) || size.QuadPart < 44 || size.QuadPart > kMaxBytes) {
        Log_Printf("Sound: %s is %lld bytes, which is not a short WAV - ignored", full,
            static_cast<long long>(size.QuadPart));
        CloseHandle(f);
        return;
    }
    BYTE* buf = static_cast<BYTE*>(std::malloc(static_cast<size_t>(size.QuadPart)));
    DWORD got = 0;
    if (!buf || !ReadFile(f, buf, static_cast<DWORD>(size.QuadPart), &got, nullptr)
        || got != static_cast<DWORD>(size.QuadPart)) {
        Log_Printf("Sound: %s could not be read", full);
        std::free(buf);
        CloseHandle(f);
        return;
    }
    CloseHandle(f);
    // A WAV and nothing else. PlaySound is handed raw memory and will happily
    // be handed something that is not a RIFF file.
    if (std::memcmp(buf, "RIFF", 4) != 0 || std::memcmp(buf + 8, "WAVE", 4) != 0) {
        Log_Printf("Sound: %s is not a WAV file - ignored", full);
        std::free(buf);
        return;
    }

    // WHAT KIND OF WAV (2026-09-26). PlaySound handles 8 and 16 bit PCM and
    // quietly does nothing with anything else, which is the worst way for this
    // to fail: the file is there, the setting is on, and there is silence with
    // no reason given. The user's own recordings arrived as 24 bit, which is
    // what any decent editor exports by default, so this is not an unusual
    // case and it is worth converting rather than refusing.
    {
        unsigned short format = 0, channels = 0, bits = 0;
        unsigned rate = 0;
        const BYTE* fmt = nullptr;
        const BYTE* data = nullptr;
        DWORD dataBytes = 0;
        DWORD at = 12;
        while (at + 8 <= got) {
            const DWORD size = *reinterpret_cast<const DWORD*>(buf + at + 4);
            if (size > got - at - 8)
                break;
            if (std::memcmp(buf + at, "fmt ", 4) == 0 && size >= 16)
                fmt = buf + at + 8;
            else if (std::memcmp(buf + at, "data", 4) == 0) {
                data = buf + at + 8;
                dataBytes = size;
            }
            at += 8 + size + (size & 1u);
        }
        if (fmt) {
            format = *reinterpret_cast<const unsigned short*>(fmt);
            channels = *reinterpret_cast<const unsigned short*>(fmt + 2);
            rate = *reinterpret_cast<const unsigned*>(fmt + 4);
            bits = *reinterpret_cast<const unsigned short*>(fmt + 14);
        }
        if (fmt && data && format == 1 && bits == 24 && channels >= 1 && channels <= 2) {
            // Three bytes little endian, signed. The top two are a 16 bit
            // sample, so the conversion is a truncation and nothing more.
            const DWORD samples = dataBytes / 3;
            const DWORD outBytes = samples * 2;
            BYTE* conv = static_cast<BYTE*>(std::malloc(44 + outBytes));
            if (conv) {
                const DWORD blockAlign = static_cast<DWORD>(channels) * 2u;
                std::memcpy(conv, "RIFF", 4);
                *reinterpret_cast<DWORD*>(conv + 4) = 36 + outBytes;
                std::memcpy(conv + 8, "WAVEfmt ", 8);
                *reinterpret_cast<DWORD*>(conv + 16) = 16;
                *reinterpret_cast<unsigned short*>(conv + 20) = 1;
                *reinterpret_cast<unsigned short*>(conv + 22) = channels;
                *reinterpret_cast<DWORD*>(conv + 24) = rate;
                *reinterpret_cast<DWORD*>(conv + 28) = rate * blockAlign;
                *reinterpret_cast<unsigned short*>(conv + 32) = static_cast<unsigned short>(blockAlign);
                *reinterpret_cast<unsigned short*>(conv + 34) = 16;
                std::memcpy(conv + 36, "data", 4);
                *reinterpret_cast<DWORD*>(conv + 40) = outBytes;
                for (DWORD s = 0; s < samples; ++s) {
                    const BYTE* src = data + s * 3;
                    conv[44 + s * 2] = src[1];
                    conv[44 + s * 2 + 1] = src[2];
                }
                std::free(buf);
                c.data = conv;
                c.bytes = 44 + outBytes;
                c.dataAt = 44;
                c.dataBytes = outBytes;
                c.bits = 16;
                Log_Printf("Sound: %s was 24 bit, %u ch at %u Hz - converted to 16 bit, %.2f s", full, channels, rate,
                    blockAlign ? static_cast<double>(outBytes) / (rate * blockAlign) : 0.0);
                return;
            }
        } else if (fmt && (format != 1 || (bits != 8 && bits != 16))) {
            Log_Printf("Sound: %s is format %u at %u bit, which Windows will not play from memory - save it as "
                       "16 bit PCM",
                full, format, bits);
            std::free(buf);
            return;
        }
    }

    c.data = buf;
    c.bytes = got;
    Log_Printf("Sound: %s loaded, %lu bytes", full, got);
}

} // namespace

namespace {

// Builds the scaled copy, or reuses it if the volume has not moved. Called
// with the lock held.
const BYTE* AtVolume(Clip& c)
{
    if (!c.data)
        return nullptr;
    if (g_volume >= 0.999f) {
        std::free(c.play);
        c.play = nullptr;
        c.playVol = 1.0f;
        return c.data;
    }
    if (c.play && std::fabs(c.playVol - g_volume) < 0.001f)
        return c.play;
    if (!c.dataBytes && !FindSamples(c.data, c.bytes, c.dataAt, c.dataBytes, c.bits))
        return c.data; // cannot find the samples, so play it as it came
    // Whatever is playing is reading the old copy.
    PlaySoundA(nullptr, nullptr, SND_PURGE);
    if (!c.play) {
        c.play = static_cast<BYTE*>(std::malloc(c.bytes));
        if (!c.play)
            return c.data;
    }
    std::memcpy(c.play, c.data, c.bytes);
    if (c.bits == 16) {
        short* s = reinterpret_cast<short*>(c.play + c.dataAt);
        const DWORD n = c.dataBytes / 2;
        for (DWORD i = 0; i < n; ++i) {
            int v = static_cast<int>(s[i] * g_volume);
            if (v > 32767)
                v = 32767;
            if (v < -32768)
                v = -32768;
            s[i] = static_cast<short>(v);
        }
    } else {
        // Eight bit PCM is unsigned and sits around 128, so it is quietened
        // toward the middle rather than toward zero.
        BYTE* s = c.play + c.dataAt;
        for (DWORD i = 0; i < c.dataBytes; ++i) {
            int v = 128 + static_cast<int>((static_cast<int>(s[i]) - 128) * g_volume);
            if (v > 255)
                v = 255;
            if (v < 0)
                v = 0;
            s[i] = static_cast<BYTE>(v);
        }
    }
    c.playVol = g_volume;
    return c.play;
}

} // namespace

void Sound_SetVolume(float zeroToOne)
{
    float v = zeroToOne;
    if (!(v > 0.0f))
        v = 0.0f;
    if (v > 1.0f)
        v = 1.0f;
    AcquireSRWLockExclusive(&g_lock);
    g_volume = v;
    ReleaseSRWLockExclusive(&g_lock);
}

void Sound_Play(const char* name)
{
    if (!name || !*name)
        return;
    const BYTE* data = nullptr;
    AcquireSRWLockExclusive(&g_lock);
    if (g_volume <= 0.001f) {
        ReleaseSRWLockExclusive(&g_lock);
        return;
    }
    Clip* c = Find(name);
    if (!c)
        c = MakeRoom(name);
    if (c) {
        if (!c->looked)
            LoadIt(*c);
        data = AtVolume(*c);
    }
    ReleaseSRWLockExclusive(&g_lock);
    if (!data)
        return;
    // Asynchronous, and never the Windows default beep if anything is wrong
    // with the buffer. The memory stays alive for the life of the process,
    // which is what SND_MEMORY|SND_ASYNC requires.
    PlaySoundA(reinterpret_cast<LPCSTR>(data), nullptr, SND_MEMORY | SND_ASYNC | SND_NODEFAULT);
}

void Sound_Forget()
{
    AcquireSRWLockExclusive(&g_lock);
    // Whatever is playing is reading one of these buffers, so it stops first.
    PlaySoundA(nullptr, nullptr, SND_PURGE);
    for (Clip& c : g_clips) {
        std::free(c.data);
        std::free(c.play);
        c = Clip{};
    }
    ReleaseSRWLockExclusive(&g_lock);
}
