#pragma once

// Where the frame time goes (2026-09-18).
//
// A tester measured a rock-solid 120 fps dropping to 100 and back on a perfect
// one-second beat, smooth in between, and the same on the menu with nothing
// happening. Running dgVoodoo2 on its own - this DLL renamed out of the way -
// is flat, so the cost is ours.
//
// Three candidates were picked off the source and removed on reasoning alone:
// a watchpoint sweep that suspended every thread, a backbuffer query polled
// once a second, and a pacing line written to disk from the submit thread.
// All three were real waste and none of them was the beat. That is a bad
// enough hit rate to stop guessing: this times each phase of our own work and
// the whole frame, keeps the worst of each second, and shows it in the menu.
//
// It has to be cheap enough not to become the thing it is measuring: two
// counter reads per phase, a few atomics, no allocation and no file I/O.

enum FramePhase {
    kPhaseStereo = 0, // StereoTest_OnEndScene
    kPhaseCamera,     // CameraRigHook_OnEndScene
    kPhaseGuard,      // AimFinder_OnEndScene + PoseGuard_Tick
    kPhasePatches,    // fade, culling, query probes
    kPhaseSubmit,     // VRBridge_OnEndScene: grab the backbuffer and submit
    kPhaseMenu,       // Menu_OnPresent, which runs whether the menu is open or not
    kPhasePresent,    // our work after the real Present returns
    kPhaseCount,
};

// One phase just finished, having taken this many performance-counter ticks.
void FrameTimes_Note(int phase, long long ticks);

// The whole of our EndScene work is done; startTicks is when it began.
void FrameTimes_EndFrame(long long startTicks);

// What the last completed second looked like. Worst values are what matter:
// an average hides a single 8 ms frame among 120 good ones, and a single bad
// frame is exactly what a beat is made of.
struct FrameTimesReport {
    bool valid;
    float worstFrameMs;            // worst whole EndScene, ours only
    float averageFrameMs;
    int worstPhase;                // which phase was slowest in that worst frame
    float worstPhaseMs[kPhaseCount]; // worst each phase reached on its own
    float averagePhaseMs[kPhaseCount];
    unsigned long frames;
    // The gap between one Present and the next, which is the stutter itself
    // rather than our share of it. If the worst gap is far above the average
    // while every phase above stays flat, the time is going somewhere this
    // does not measure - and that is worth knowing too.
    float worstPresentGapMs;
    float averagePresentGapMs;
    unsigned long presents;
};
bool FrameTimes_Get(FrameTimesReport& out);

// Called from the Present hook, once per presented frame.
void FrameTimes_NotePresent();

const char* FrameTimes_PhaseName(int phase);
