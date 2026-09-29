#include "frame_times.h"

#include <windows.h>

#include <atomic>
#include <cstring>

// See frame_times.h. Everything here runs inside the frame being measured, so
// it does as little as it can get away with: no locks, no allocation, no I/O.

namespace {

double TicksToMs()
{
    static double scale = 0.0;
    if (scale == 0.0) {
        LARGE_INTEGER freq;
        QueryPerformanceFrequency(&freq);
        scale = freq.QuadPart ? 1000.0 / static_cast<double>(freq.QuadPart) : 0.0;
    }
    return scale;
}

// The second being accumulated, and the last one finished. Only the EndScene
// thread writes the accumulator; the menu reads the finished copy.
struct Accum {
    unsigned long frames;
    double frameSumMs;
    double frameWorstMs;
    int frameWorstPhase;
    double phaseSumMs[kPhaseCount];
    double phaseWorstMs[kPhaseCount];
    double thisFramePhaseMs[kPhaseCount];
};

// Present to Present, kept separately because Present and EndScene are not
// the same beat: at the menu the game runs EndScene several times for each
// frame it shows.
long long g_lastPresentTicks = 0;
double g_presentSumMs = 0.0;
double g_presentWorstMs = 0.0;
unsigned long g_presents = 0;

Accum g_live = {};
FrameTimesReport g_done = {};
std::atomic<bool> g_haveDone{ false };
ULONGLONG g_secondStartedMs = 0;

} // namespace

void FrameTimes_Note(int phase, long long ticks)
{
    if (phase < 0 || phase >= kPhaseCount)
        return;
    const double ms = static_cast<double>(ticks) * TicksToMs();
    g_live.thisFramePhaseMs[phase] = ms;
    g_live.phaseSumMs[phase] += ms;
    if (ms > g_live.phaseWorstMs[phase])
        g_live.phaseWorstMs[phase] = ms;
}

void FrameTimes_EndFrame(long long startTicks)
{
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    const double ms = static_cast<double>(now.QuadPart - startTicks) * TicksToMs();
    ++g_live.frames;
    g_live.frameSumMs += ms;
    if (ms > g_live.frameWorstMs) {
        g_live.frameWorstMs = ms;
        // Which phase dominated the worst frame, so the report names a culprit
        // rather than only a number.
        int worst = 0;
        for (int i = 1; i < kPhaseCount; ++i) {
            if (g_live.thisFramePhaseMs[i] > g_live.thisFramePhaseMs[worst])
                worst = i;
        }
        g_live.frameWorstPhase = worst;
    }
    std::memset(g_live.thisFramePhaseMs, 0, sizeof(g_live.thisFramePhaseMs));

    const ULONGLONG nowMs = GetTickCount64();
    if (!g_secondStartedMs) {
        g_secondStartedMs = nowMs;
        return;
    }
    if (nowMs - g_secondStartedMs < 1000)
        return;
    g_secondStartedMs = nowMs;

    FrameTimesReport r = {};
    r.valid = true;
    r.frames = g_live.frames;
    r.worstFrameMs = static_cast<float>(g_live.frameWorstMs);
    r.averageFrameMs = g_live.frames ? static_cast<float>(g_live.frameSumMs / g_live.frames) : 0.0f;
    r.presents = g_presents;
    r.worstPresentGapMs = static_cast<float>(g_presentWorstMs);
    r.averagePresentGapMs = g_presents ? static_cast<float>(g_presentSumMs / g_presents) : 0.0f;
    r.worstPhase = g_live.frameWorstPhase;
    for (int i = 0; i < kPhaseCount; ++i) {
        r.worstPhaseMs[i] = static_cast<float>(g_live.phaseWorstMs[i]);
        r.averagePhaseMs[i] = g_live.frames ? static_cast<float>(g_live.phaseSumMs[i] / g_live.frames) : 0.0f;
    }
    g_done = r;
    g_haveDone.store(true, std::memory_order_release);
    g_live = {};
    g_presentSumMs = 0.0;
    g_presentWorstMs = 0.0;
    g_presents = 0;
}

bool FrameTimes_Get(FrameTimesReport& out)
{
    if (!g_haveDone.load(std::memory_order_acquire))
        return false;
    out = g_done;
    return out.valid;
}

void FrameTimes_NotePresent()
{
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    if (g_lastPresentTicks) {
        const double ms = static_cast<double>(now.QuadPart - g_lastPresentTicks) * TicksToMs();
        ++g_presents;
        g_presentSumMs += ms;
        if (ms > g_presentWorstMs)
            g_presentWorstMs = ms;
    }
    g_lastPresentTicks = now.QuadPart;
}

const char* FrameTimes_PhaseName(int phase)
{
    switch (phase) {
    case kPhaseStereo:
        return "stereo";
    case kPhaseCamera:
        return "camera";
    case kPhaseGuard:
        return "arm guard";
    case kPhasePatches:
        return "patches";
    case kPhaseSubmit:
        return "submit";
    case kPhaseMenu:
        return "menu";
    case kPhasePresent:
        return "present";
    default:
        return "?";
    }
}
