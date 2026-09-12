#include "fade_patch.h"
#include "../util/log.h"

#include <MinHook.h>
#include <windows.h>
#include <intrin.h>

#include <cstring>

// ---- Where the fade comes from (2026-09-11) ----------------------------
// RE5 fades any character the camera gets close to - in first person that is
// Chris's own body and gun, and Sheva whenever she walks into the camera.
// Tracked down with fade_probe.cpp (the faded colour pass switches to
// SRCALPHA/INVSRCALPHA blending), the F5 hardware watch on the two object
// fields that flip with it (+0x304 / +0x308), and a full-code search:
//
//   SetVisibility(view, value)  re5dx9.exe+7576C0, thiscall on the model,
//     stores value in a per-view array at model+1724h and derives the
//     fade parameters from it. Once per frame a render-state function
//     (+761F80 for characters, an identical +755900 for base models) sends
//     the model down an alpha-blended "fade" path for any value but 1.0.
//
// The camera's proximity fade is three calls to SetVisibility from camera
// code (return addresses below), e.g. at +43D4A7:
//   visibility = min(1 - [camera+30D8h] / k, current)
//
// FIRST FIX, REJECTED BEFORE TESTING: skipping the render-state function's
// fade branch (jp at +761FEE / +75596A) made every non-1.0 model opaque.
// The full-code search then showed SetVisibility has ~30 callers: ~20 set
// exactly 1.0 (restores), two set exactly 0.0 (+522D1B, +522D3D - scripted
// hides), and +757B90 sets a computed value (probably timed fades). Forcing
// the opaque branch would have made deliberately hidden models visible and
// turned timed fades into pops.
//
// THIS FIX: hook SetVisibility and, only while F4 first person is on and
// only when called from one of the five camera sites, replace the value
// with 1.0. Every other caller passes through untouched. The per-caller
// summary logged at EndScene shows which sites actually fire.

// ---- The OTHER fade, found 2026-09-12 ----------------------------------
// The tester reported NPCs and Sheva fading out when you walk right up to
// them, which the above did nothing about - and the per-caller log showed no
// fractional values while it happened. Reason: there are two setters.
//
//   SetVisibility  +7576C0  stores the raw value at model+view*4+1724h,
//     then calls the mapper below with it.
//   ApplyVisibility +755410  maps a value through a global response curve
//     ([123457Ch]+3090h..309Ch) and stores the EFFECTIVE alpha the renderer
//     reads, at model+view*4+1734h.
//
// +757710 is the per-model proximity fade: distance from the model's origin
// (+0x30/34/38) to the camera, against a near/far pair of globals
// ([113AF20h] and friends, with hysteresis at [113AF38h]), driving a
// four-state machine (0 visible, 1 fading out, 2 hidden, 3 fading in) whose
// state lives at [slot-10h]. It animates the alpha itself and writes it
// STRAIGHT to +755410, never through SetVisibility - which is exactly why
// the existing hook never saw it. Its one SetVisibility call (+757B90) is a
// separate branch that sets the distance ratio instantly; it didn't fire in
// the capture but it is the same feature, so it is blocked too.
//
// Not included: +757956, which copies a child's visibility onto its parent.
// Same reasoning as +439E21/+439E6F below - with the real fades blocked it
// copies 1.0 anyway, and forcing it would reveal scripted hides.

namespace {

constexpr uintptr_t kOffSetVisibility = 0x7576C0;
constexpr uintptr_t kOffApplyVisibility = 0x755410;

// Return addresses (module-relative) of the camera's own fade writes:
//   +43D2C3  1 - ([cam+30DCh]/k) * (1 - x)      proximity fade, character
//   +43D4A7  min(1 - [cam+30D8h]/k, current)   proximity fade, character
//   +43D4EC  the +43D2C3 value applied to the character's attachment (the gun)
// Deliberately NOT included: +439E21 / +439E6F. Those copy a character's
// CURRENT visibility onto its attachments. With the camera fades blocked they
// copy 1.0 anyway - and if a script hides the character (0.0), the gun must
// hide with it rather than float in mid-air.
//   +757B90  the proximity fade's instant branch: clamp((dist - near) /
//            (far - near), 0, 1), applied to the model itself.
constexpr uintptr_t kCameraCallers[] = { 0x43D2C8, 0x43D4AC, 0x43D4F1, 0x757B90 };

// Return addresses of the proximity fade's animated writes, which go direct
// to ApplyVisibility. +757A24 is deliberately absent: that is state 0 and it
// always pushes exactly 1.0, so overriding it would only add log noise.
//   +757A89  state 1, alpha -= dt * fadeOutRate
//   +757AC4  state 2 -> 3 transition, pushes 0.0
//   +757B1B  state 3, alpha += dt * fadeInRate
constexpr uintptr_t kFadeStateCallers[] = { 0x757A89, 0x757AC4, 0x757B1B };

typedef void(__fastcall* SetVisibility_t)(void* model, void* edxUnused, int view, float value);
SetVisibility_t g_origSetVisibility = nullptr;
SetVisibility_t g_origApplyVisibility = nullptr;

uintptr_t g_moduleBase = 0;
volatile bool g_enabled = false;

// Lock-free caller table: the hook runs on game threads, so no logging or
// allocation in there - EndScene reports it.
constexpr int kMaxCallers = 48;
struct CallerStat {
    volatile LONG caller; // module-relative return address, 0 = free
    volatile LONG count;
    float minValue;
    volatile LONG overridden;
};
CallerStat g_callers[kMaxCallers];
unsigned long long g_lastReportMs = 0;

bool IsCameraCaller(uintptr_t ret)
{
    for (uintptr_t c : kCameraCallers) {
        if (ret == c)
            return true;
    }
    for (uintptr_t c : kFadeStateCallers) {
        if (ret == c)
            return true;
    }
    return false;
}

bool IsFadeStateCaller(uintptr_t ret)
{
    for (uintptr_t c : kFadeStateCallers) {
        if (ret == c)
            return true;
    }
    return false;
}

void RecordCaller(uintptr_t ret, float value, bool overridden)
{
    const LONG key = static_cast<LONG>(ret);
    for (int i = 0; i < kMaxCallers; ++i) {
        LONG cur = g_callers[i].caller;
        if (cur == 0 && InterlockedCompareExchange(&g_callers[i].caller, key, 0) == 0) {
            g_callers[i].minValue = value;
            cur = key;
        } else {
            cur = g_callers[i].caller;
        }
        if (cur == key) {
            if (InterlockedIncrement(&g_callers[i].count) == 1 || value < g_callers[i].minValue)
                g_callers[i].minValue = value;
            if (overridden)
                InterlockedIncrement(&g_callers[i].overridden);
            return;
        }
    }
}

void __fastcall hkSetVisibility(void* model, void* edxUnused, int view, float value)
{
    const uintptr_t ret = reinterpret_cast<uintptr_t>(_ReturnAddress()) - g_moduleBase;
    const bool overridden = g_enabled && IsCameraCaller(ret) && value != 1.0f;
    RecordCaller(ret, value, overridden);
    if (overridden)
        value = 1.0f;
    g_origSetVisibility(model, edxUnused, view, value);
}

// The proximity fade animates the alpha itself and writes it here, bypassing
// SetVisibility entirely. Only its own sites are overridden - everything
// else, including SetVisibility's own internal call at +7576EB (which has
// already been corrected by the hook above), passes through.
void __fastcall hkApplyVisibility(void* model, void* edxUnused, int view, float value)
{
    const uintptr_t ret = reinterpret_cast<uintptr_t>(_ReturnAddress()) - g_moduleBase;
    const bool overridden = g_enabled && IsFadeStateCaller(ret) && value != 1.0f;
    RecordCaller(ret, value, overridden);
    if (overridden)
        value = 1.0f;
    g_origApplyVisibility(model, edxUnused, view, value);
}

} // namespace

void FadePatch_Install()
{
    g_moduleBase = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr));
    void* target = reinterpret_cast<void*>(g_moduleBase + kOffSetVisibility);

    // Expected prologue: push esi / mov esi,[esp+8] / test esi,esi.
    static const unsigned char kPrologue[] = { 0x56, 0x8B, 0x74, 0x24, 0x08, 0x85, 0xF6 };
    if (std::memcmp(target, kPrologue, sizeof(kPrologue)) != 0) {
        const unsigned char* p = static_cast<const unsigned char*>(target);
        Log_Printf("FadePatch_Install: SetVisibility prologue is %02X %02X %02X %02X %02X %02X %02X, not the "
                   "expected one - not hooking (different game build?)",
            p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
        return;
    }

    MH_STATUS st = MH_CreateHook(target, reinterpret_cast<void*>(&hkSetVisibility),
        reinterpret_cast<void**>(&g_origSetVisibility));
    if (st != MH_OK && st != MH_ERROR_ALREADY_CREATED) {
        Log_Printf("FadePatch_Install: MH_CreateHook failed -> %d", static_cast<int>(st));
        return;
    }
    st = MH_EnableHook(target);
    Log_Printf("FadePatch_Install: SetVisibility hook enabled -> %d (target=%p)", static_cast<int>(st), target);

    // ApplyVisibility. Expected prologue: mov eax,ds:[0123457Ch] - an
    // absolute address, so it pins the build as tightly as a byte pattern.
    void* applyTarget = reinterpret_cast<void*>(g_moduleBase + kOffApplyVisibility);
    static const unsigned char kApplyPrologue[] = { 0xA1, 0x7C, 0x45, 0x23, 0x01 };
    if (std::memcmp(applyTarget, kApplyPrologue, sizeof(kApplyPrologue)) != 0) {
        const unsigned char* p = static_cast<const unsigned char*>(applyTarget);
        Log_Printf("FadePatch_Install: ApplyVisibility prologue is %02X %02X %02X %02X %02X, not the expected "
                   "one - not hooking (proximity fade stays on)",
            p[0], p[1], p[2], p[3], p[4]);
        return;
    }

    st = MH_CreateHook(applyTarget, reinterpret_cast<void*>(&hkApplyVisibility),
        reinterpret_cast<void**>(&g_origApplyVisibility));
    if (st != MH_OK && st != MH_ERROR_ALREADY_CREATED) {
        Log_Printf("FadePatch_Install: MH_CreateHook(ApplyVisibility) failed -> %d", static_cast<int>(st));
        return;
    }
    st = MH_EnableHook(applyTarget);
    Log_Printf("FadePatch_Install: ApplyVisibility hook enabled -> %d (target=%p)", static_cast<int>(st), applyTarget);
}

void FadePatch_SetEnabled(bool enabled)
{
    g_enabled = enabled;
    Log_Printf("FadePatch: camera-proximity fade %s", enabled ? "SUPPRESSED (first person)" : "restored");
}

void FadePatch_OnEndScene()
{
    const unsigned long long now = GetTickCount64();
    if (now - g_lastReportMs < 3000)
        return;
    g_lastReportMs = now;

    // Only report windows where something set a model to anything but
    // fully visible - otherwise this is just the ~20 reset callers.
    bool interesting = false;
    for (const CallerStat& c : g_callers) {
        if (c.caller && c.count && c.minValue < 1.0f)
            interesting = true;
    }
    if (interesting) {
        for (CallerStat& c : g_callers) {
            if (!c.caller || !c.count)
                continue;
            Log_Printf("FadePatch: SetVisibility from re5dx9.exe+%lX%s: %ld call(s), min value %.3f, %ld overridden to 1.0",
                static_cast<unsigned long>(c.caller), IsCameraCaller(static_cast<uintptr_t>(c.caller)) ? " (camera)" : "",
                c.count, c.minValue, c.overridden);
        }
    }
    for (CallerStat& c : g_callers) {
        InterlockedExchange(&c.count, 0);
        InterlockedExchange(&c.overridden, 0);
        c.minValue = 1.0f;
    }
}
