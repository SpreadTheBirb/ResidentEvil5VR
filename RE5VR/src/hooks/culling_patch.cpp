#include "culling_patch.h"
#include "camera_rig_hook.h"
#include "../util/log.h"

#include <MinHook.h>
#include <windows.h>

#include <cstring>

// ---- Why (2026-09-11) --------------------------------------------------
// VR culling was "god awful": the game only draws what its own camera can
// see, but the headset shows a wider view and the head turns independently
// of the game camera. Widening the game FOV was tried and removed - even
// 165 deg vertical is only +-85 deg horizontal at 16:9, so anything over
// the shoulder stays outside any frustum around the game's forward.
//
// Found by walking the camera data flow with F5 watchpoints (controller
// output -> main camera -> the renderer's readers) and the code dump:
//
//   re5dx9.exe+435A90  bool Camera::IsSphereVisible(const float sphere[4])
//     thiscall on a camera, sphere = (x, y, z, radius), ret 4. Tests the
//     sphere against six planes stored in the camera at +0xE0, +0xF0,
//     +0x100, +0x110, +0x120, +0x130 (each nx, ny, nz, d): returns 0 as
//     soon as dot(n, centre) + d < -radius for any plane, else 1.
//
// The model draw-list builder calls it per object for each camera
// (+3E4845), and on a model's bounding sphere at model+0x2D0 (+3E4288).
// While VR is on this detour answers "visible" every time; flat screen is
// untouched. It also counts how often the game WOULD have culled, so the
// log shows whether this is the test that matters for what's missing.
//
// Open question at the time of writing: level geometry may be culled by a
// different routine (a box test?) - being checked separately.

namespace {

constexpr uintptr_t kOffSphereVisible = 0x435A90;

typedef bool(__fastcall* SphereVisible_t)(void* camera, void* edxUnused, const float* sphere);
SphereVisible_t g_origSphereVisible = nullptr;

volatile LONG g_calls = 0;
volatile LONG g_wouldCull = 0;
unsigned long long g_lastReportMs = 0;

bool __fastcall hkSphereVisible(void* camera, void* edxUnused, const float* sphere)
{
    const bool visible = g_origSphereVisible(camera, edxUnused, sphere);
    if (!CameraRigHook_IsVrActive())
        return visible;
    InterlockedIncrement(&g_calls);
    if (!visible)
        InterlockedIncrement(&g_wouldCull);
    return true;
}

} // namespace

void CullingPatch_Install()
{
    const uintptr_t base = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr));
    void* target = reinterpret_cast<void*>(base + kOffSphereVisible);

    // push ebp / mov ebp,esp / and esp,-16 / mov eax,[ebp+8]
    static const unsigned char kPrologue[] = { 0x55, 0x8B, 0xEC, 0x83, 0xE4, 0xF0, 0x8B, 0x45, 0x08 };
    if (std::memcmp(target, kPrologue, sizeof(kPrologue)) != 0) {
        Log_Printf("CullingPatch_Install: sphere-test prologue doesn't match - not hooking (different game build?)");
        return;
    }
    MH_STATUS st = MH_CreateHook(target, reinterpret_cast<void*>(&hkSphereVisible),
        reinterpret_cast<void**>(&g_origSphereVisible));
    if (st != MH_OK && st != MH_ERROR_ALREADY_CREATED) {
        Log_Printf("CullingPatch_Install: MH_CreateHook failed -> %d", static_cast<int>(st));
        return;
    }
    st = MH_EnableHook(target);
    Log_Printf("CullingPatch_Install: frustum sphere-test hook enabled -> %d (target=%p), active while VR is on",
        static_cast<int>(st), target);
}

void CullingPatch_OnEndScene()
{
    const unsigned long long now = GetTickCount64();
    if (now - g_lastReportMs < 5000)
        return;
    g_lastReportMs = now;
    const LONG calls = InterlockedExchange(&g_calls, 0);
    const LONG culls = InterlockedExchange(&g_wouldCull, 0);
    if (calls)
        Log_Printf("CullingPatch: last 5 s - %ld sphere test(s), %ld would have been culled, all drawn", calls, culls);
}
