#include "blind_forwards.h"

// Real Resident Evil 5 (2009, MT Framework) never calls any of these - they
// are perf/debug/internal d3d9.dll exports we don't understand or need to
// intercept. Rather than guess unknown/undocumented calling signatures
// (risky - a wrong prototype would corrupt the stack), each is a naked
// jmp-trampoline: it emits no prologue/epilogue of its own, so jumping
// straight into the real implementation leaves the caller's stack exactly
// as the real d3d9.dll expects it, regardless of the true signature.
// x86-only (naked + inline asm), which matches our Win32-only build target.

#define BLIND_FORWARD(name)                                    \
    extern "C" FARPROC g_real_##name = nullptr;                \
    extern "C" __declspec(naked) void name()                   \
    {                                                            \
        __asm { jmp dword ptr [g_real_##name] }                 \
    }

BLIND_FORWARD(D3DPERF_BeginEvent)
BLIND_FORWARD(D3DPERF_EndEvent)
BLIND_FORWARD(D3DPERF_GetStatus)
BLIND_FORWARD(D3DPERF_QueryRepeatFrame)
BLIND_FORWARD(D3DPERF_SetMarker)
BLIND_FORWARD(D3DPERF_SetOptions)
BLIND_FORWARD(D3DPERF_SetRegion)
BLIND_FORWARD(DebugSetLevel)
BLIND_FORWARD(DebugSetMute)
BLIND_FORWARD(Direct3D9EnableMaximizedWindowedModeShim)
BLIND_FORWARD(Direct3DCreate9On12)
BLIND_FORWARD(Direct3DCreate9On12Ex)
BLIND_FORWARD(Direct3DShaderValidatorCreate9)
BLIND_FORWARD(PSGPError)
BLIND_FORWARD(PSGPSampleTexture)

#undef BLIND_FORWARD

namespace {

struct ForwardEntry {
    const char* name;
    FARPROC* slot;
};

#define ENTRY(name) { #name, &g_real_##name }

const ForwardEntry kForwardTable[] = {
    ENTRY(D3DPERF_BeginEvent),
    ENTRY(D3DPERF_EndEvent),
    ENTRY(D3DPERF_GetStatus),
    ENTRY(D3DPERF_QueryRepeatFrame),
    ENTRY(D3DPERF_SetMarker),
    ENTRY(D3DPERF_SetOptions),
    ENTRY(D3DPERF_SetRegion),
    ENTRY(DebugSetLevel),
    ENTRY(DebugSetMute),
    ENTRY(Direct3D9EnableMaximizedWindowedModeShim),
    ENTRY(Direct3DCreate9On12),
    ENTRY(Direct3DCreate9On12Ex),
    ENTRY(Direct3DShaderValidatorCreate9),
    ENTRY(PSGPError),
    ENTRY(PSGPSampleTexture),
};

#undef ENTRY

} // namespace

void ResolveBlindForwards(HMODULE hRealD3D9)
{
    for (const auto& entry : kForwardTable)
        *entry.slot = GetProcAddress(hRealD3D9, entry.name);
}
