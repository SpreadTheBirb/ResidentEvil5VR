#include "menu.h"
#include "input_block.h"
#include "../hooks/camera_rig_hook.h"
#include "../hooks/filter_patch.h"
#include "../render/stereo_test.h"
#include "../render/hud_shaders.h"
#include "../vr/openxr_bridge.h"
#include "../util/build_config.h"
#include "../util/log.h"

#include <imgui.h>
#include <imgui_impl_dx9.h>
#include <imgui_impl_win32.h>

#include <windows.h>

#include <algorithm>
#include <atomic>
#include <cfloat>
#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

namespace {

constexpr const char* kTitle = "STB - TrueFP/VR Mod";

// ---- Settings that belong to the menu itself ---------------------------
struct MenuPrefs {
    float uiScale = 1.0f;
    // Convergence distance of the VR menu. NOT a player option: the user found
    // (2026-09-13) that moving it reads as the two eyes' copies spreading
    // apart, not as the menu moving away - size is what sets felt distance.
    float vrMenuDistance = 1.5f;
    bool padChord = true;        // a quick click of both sticks opens the menu
    bool startupHint = true;
    bool autoStartVr = false;
};

struct AllSettings {
    CameraRigSettings cam;
    StereoSettings stereo;
    VRBridgeSettings vr;
    bool filterRemoved = true;
    MenuPrefs menu;
};

AllSettings g_defaults;
MenuPrefs g_prefs;

AllSettings CaptureSettings()
{
    AllSettings s;
    s.cam = CameraRigHook_GetSettings();
    s.stereo = StereoTest_GetSettings();
    s.vr = VRBridge_GetSettings();
    s.filterRemoved = FilterPatch_IsFilterRemoved();
    s.menu = g_prefs;
    return s;
}

void ApplySettings(const AllSettings& in)
{
    CameraRigHook_ApplySettings(in.cam);
    StereoSettings st = in.stereo;
    // Stereo on/off belongs to VR mode, which flips it from its own thread.
    // Never let a menu copy taken a moment earlier switch it back.
    st.stereoEnabled = StereoTest_IsEnabled();
    StereoTest_ApplySettings(st);
    VRBridge_ApplySettings(in.vr);
    if (FilterPatch_IsAvailable())
        FilterPatch_SetFilterRemoved(in.filterRemoved);
    g_prefs = in.menu;
}

// One list drives load, save and "did anything change". Developer switches
// are deliberately absent: a diagnostics build must never leave, say, direct
// submit turned off in an ini that a release build then reads.
#define RE5VR_SETTINGS(X)                                            \
    X("Camera", "FirstPerson", cam.firstPerson)                      \
    X("Camera", "ShowHeadDuringActions", cam.showHeadDuringActions)  \
    X("Camera", "FlatFov", cam.flatFovDeg)                           \
    X("Camera", "FlatEyeHeight", cam.flatEyeUp)                      \
    X("Camera", "FlatEyeForward", cam.flatEyeAhead)                  \
    X("Graphics", "RemoveColourFilter", filterRemoved)               \
    X("VR", "StartInVR", menu.autoStartVr)                           \
    X("VR", "HeadTurnsCamera", cam.headFollow)                       \
    X("VR", "Stabilise", cam.vrStabilise)                            \
    X("VR", "MatchCullingToHeadset", cam.vrMatchCullFov)             \
    X("VR", "EyeHeight", cam.vrEyeUp)                                \
    X("VR", "EyeForward", cam.vrEyeAhead)                            \
    X("VR", "EyeSeparation", stereo.halfSeparation)                  \
    X("VR", "FovWiden", stereo.fovWiden)                             \
    X("VR", "MonoPostProcess", stereo.monoSmallTargets)              \
\
    X("VR", "HudScale", stereo.hudScale)                             \
    X("VR", "HeadPredictionMs", vr.headPredictMs)                    \
    X("VR", "HeadRotationGain", vr.headRotationGain)                 \
    X("Menu", "Scale", menu.uiScale)                                 \
\
    X("Menu", "OpenWithBothSticks", menu.padChord)                   \
    X("Menu", "StartupHint", menu.startupHint)

char g_iniPath[MAX_PATH] = "";

void ReadValue(const char* section, const char* key, bool& v)
{
    char buf[32];
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), g_iniPath);
    if (buf[0])
        v = std::atoi(buf) != 0;
}
void ReadValue(const char* section, const char* key, float& v)
{
    char buf[32];
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), g_iniPath);
    if (buf[0])
        v = static_cast<float>(std::atof(buf));
}
std::string FormatValue(bool v)
{
    return v ? "1" : "0";
}
std::string FormatValue(float v)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "%.3f", v);
    return buf;
}

std::string Serialize(const AllSettings& s)
{
    std::string out;
#define X(section, key, member) out += std::string(section) + "." + key + "=" + FormatValue(s.member) + "\n";
    RE5VR_SETTINGS(X)
#undef X
    return out;
}

void LoadSettings(AllSettings& s)
{
#define X(section, key, member) ReadValue(section, key, s.member);
    RE5VR_SETTINGS(X)
#undef X
}

void SaveSettings(const AllSettings& s)
{
#define X(section, key, member) \
    WritePrivateProfileStringA(section, key, FormatValue(s.member).c_str(), g_iniPath);
    RE5VR_SETTINGS(X)
#undef X
    Log_Printf("Menu: settings saved to %s", g_iniPath);
}

// ---- State --------------------------------------------------------------
IDirect3DDevice9* g_device = nullptr;
HWND g_hwnd = nullptr;
WNDPROC g_gameWndProc = nullptr;
bool g_imguiReady = false;
bool g_open = false;
std::atomic<bool> g_drawing{ false };

std::string g_savedSnapshot;
ULONGLONG g_lastChangeMs = 0;
bool g_dirty = false;

ULONGLONG g_firstPresentMs = 0;
bool g_autoVrDone = false;
float g_fontPxBuilt = 0.0f;
float g_frameMsAvg = 0.0f;
LARGE_INTEGER g_qpcFreq = {}, g_lastPresentQpc = {};

float g_cursorX = 0.0f, g_cursorY = 0.0f; // in UI units
float g_sentX = -1e9f, g_sentY = -1e9f;     // last position handed to ImGui
float g_lastRealX = -1e9f, g_lastRealY = -1e9f;
int g_selectTab = -1;                     // set by the bumpers, applied next frame
int g_currentTab = 0;

// Window messages arrive on the game's window thread; ImGui is fed on the
// render thread. Copy them across rather than touch ImGui from two threads.
struct QueuedMsg {
    UINT msg;
    WPARAM w;
    LPARAM l;
};
SRWLOCK g_msgLock = SRWLOCK_INIT;
QueuedMsg g_msgs[256];
int g_msgCount = 0;

bool IsKeyboardMsg(UINT m)
{
    return m == WM_KEYDOWN || m == WM_KEYUP || m == WM_CHAR;
}
bool IsMouseMsg(UINT m)
{
    return (m >= WM_MOUSEFIRST && m <= WM_MOUSELAST) || m == WM_MOUSEHOVER || m == WM_MOUSELEAVE;
}

void QueueMessage(UINT msg, WPARAM w, LPARAM l)
{
    if (msg == WM_MOUSEMOVE || !(IsKeyboardMsg(msg) || IsMouseMsg(msg)))
        return;
    AcquireSRWLockExclusive(&g_msgLock);
    if (g_msgCount < static_cast<int>(_countof(g_msgs)))
        g_msgs[g_msgCount++] = { msg, w, l };
    ReleaseSRWLockExclusive(&g_msgLock);
}

// Most input is diverted earlier, where the game pulls it off its queue (see
// input_block.cpp); this catches whatever is sent straight to the window.
LRESULT CALLBACK MenuWndProc(HWND hwnd, UINT msg, WPARAM w, LPARAM l)
{
    // One pointer, not two: over the game's own menus Windows shows its
    // cursor too, sitting on top of ours. Windows asks for the cursor shape
    // with WM_SETCURSOR on every mouse move, so answering "none" while the
    // menu is open hides it; closing hands the question back to the game.
    if (g_open && msg == WM_SETCURSOR && LOWORD(l) == HTCLIENT) {
        SetCursor(nullptr);
        return TRUE;
    }
    if (g_open && (IsKeyboardMsg(msg) || IsMouseMsg(msg))) {
        QueueMessage(msg, w, l);
        return 0; // the menu has them; the game does not
    }
    return CallWindowProcA(g_gameWndProc, hwnd, msg, w, l);
}

bool GameIsForeground()
{
    DWORD pid = 0;
    GetWindowThreadProcessId(GetForegroundWindow(), &pid);
    return pid == GetCurrentProcessId();
}

bool ExeIsLargeAddressAware()
{
    const auto* base = reinterpret_cast<const BYTE*>(GetModuleHandleA(nullptr));
    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(base + dos->e_lfanew);
    return (nt->FileHeader.Characteristics & IMAGE_FILE_LARGE_ADDRESS_AWARE) != 0;
}

// ---- Style and fonts ----------------------------------------------------
void ApplyStyle(float scale)
{
    ImGuiStyle& st = ImGui::GetStyle();
    st = ImGuiStyle();
    ImGui::StyleColorsDark(&st);
    st.WindowRounding = 6.0f;
    st.ChildRounding = 4.0f;
    st.FrameRounding = 4.0f;
    st.GrabRounding = 4.0f;
    st.TabRounding = 4.0f;
    st.PopupRounding = 4.0f;
    st.WindowPadding = ImVec2(12, 10);
    st.FramePadding = ImVec2(8, 4);
    st.ItemSpacing = ImVec2(8, 6);
    st.WindowTitleAlign = ImVec2(0.5f, 0.5f);

    const ImVec4 accent(0.64f, 0.16f, 0.12f, 1.00f);
    const ImVec4 accentHi(0.78f, 0.24f, 0.18f, 1.00f);
    const ImVec4 accentLo(0.40f, 0.11f, 0.09f, 1.00f);
    ImVec4* c = st.Colors;
    c[ImGuiCol_WindowBg] = ImVec4(0.06f, 0.06f, 0.07f, 0.95f);
    c[ImGuiCol_TitleBg] = ImVec4(0.10f, 0.05f, 0.05f, 1.00f);
    c[ImGuiCol_TitleBgActive] = accentLo;
    c[ImGuiCol_FrameBg] = ImVec4(0.16f, 0.15f, 0.15f, 1.00f);
    c[ImGuiCol_FrameBgHovered] = ImVec4(0.26f, 0.20f, 0.19f, 1.00f);
    c[ImGuiCol_FrameBgActive] = ImVec4(0.32f, 0.22f, 0.20f, 1.00f);
    c[ImGuiCol_CheckMark] = accentHi;
    c[ImGuiCol_SliderGrab] = accent;
    c[ImGuiCol_SliderGrabActive] = accentHi;
    c[ImGuiCol_Button] = accentLo;
    c[ImGuiCol_ButtonHovered] = accent;
    c[ImGuiCol_ButtonActive] = accentHi;
    c[ImGuiCol_Header] = accentLo;
    c[ImGuiCol_HeaderHovered] = accent;
    c[ImGuiCol_HeaderActive] = accentHi;
    c[ImGuiCol_Tab] = ImVec4(0.18f, 0.10f, 0.09f, 1.00f);
    c[ImGuiCol_TabHovered] = accent;
    c[ImGuiCol_TabSelected] = accentLo;
    c[ImGuiCol_TabSelectedOverline] = accentHi;
    c[ImGuiCol_SeparatorHovered] = accent;
    c[ImGuiCol_SeparatorActive] = accentHi;
    c[ImGuiCol_ResizeGrip] = accentLo;
    c[ImGuiCol_ResizeGripHovered] = accent;
    c[ImGuiCol_ResizeGripActive] = accentHi;
    c[ImGuiCol_NavCursor] = accentHi;
    st.ScaleAllSizes(scale);
}

// Rebuilds the font atlas at a new pixel size. A real outline font rebuilt
// at the size needed stays sharp; scaling the built-in bitmap font would blur.
void EnsureFont(float px)
{
    px = (std::max)(12.0f, (std::min)(px, 72.0f));
    // Hysteresis: a size hovering on a .5 boundary must not rebuild every frame.
    if (g_fontPxBuilt > 0.0f && std::fabs(px - g_fontPxBuilt) < 0.75f)
        return;
    px = std::round(px);
    g_fontPxBuilt = px;
    ImGuiIO& io = ImGui::GetIO();
    ImGui_ImplDX9_InvalidateDeviceObjects(); // font texture rebuilt at the next NewFrame
    io.Fonts->Clear();
    char path[MAX_PATH];
    const UINT n = GetWindowsDirectoryA(path, MAX_PATH);
    ImFont* font = nullptr;
    if (n && n < MAX_PATH - 32) {
        strcat_s(path, "\\Fonts\\segoeui.ttf");
        if (GetFileAttributesA(path) != INVALID_FILE_ATTRIBUTES)
            font = io.Fonts->AddFontFromFileTTF(path, px);
    }
    if (!font) {
        ImFontConfig cfg;
        cfg.SizePixels = px;
        io.Fonts->AddFontDefault(&cfg);
    }
    ApplyStyle(px / 18.0f);
}

// ---- Layout: flat, or once per eye --------------------------------------
struct Layout {
    bool stereo = false;
    float bbW = 0, bbH = 0;
    float unitsW = 0, unitsH = 0; // ImGui's display size
    float sx = 1.0f;              // UI unit -> backbuffer pixels, horizontally
    float ox[2] = {}, oy[2] = {}; // where the UI's origin lands, per eye
    float clipX0[2] = {}, clipX1[2] = {};
    float fontPx = 18.0f;
};

Layout ComputeLayout(UINT bbW, UINT bbH)
{
    Layout L;
    L.bbW = static_cast<float>(bbW);
    L.bbH = static_cast<float>(bbH);
    L.stereo = StereoTest_IsEnabled();
    if (!L.stereo) {
        L.unitsW = L.bbW;
        L.unitsH = L.bbH;
        L.clipX1[0] = L.bbW;
        // 20 px at 1080p, but never below 16 - a 720p window is still read
        // from a normal desk distance.
        L.fontPx = (std::max)(16.0f, 20.0f * (L.bbH / 1080.0f)) * g_prefs.uiScale;
        return L;
    }

    StereoPanelEye eyes[2];
    StereoTest_GetPanelPlacement(g_prefs.vrMenuDistance, bbW, bbH, eyes);
    // One UI unit is one vertical pixel. An eye's half is narrower in pixels
    // than the angle it covers, so horizontal units are squeezed by sx to
    // keep text and boxes square in the headset.
    const float pyt = eyes[0].pxPerTanY;
    L.sx = eyes[0].pxPerTanX / pyt;
    // About 50 x 53 degrees of view - big enough to read, small enough that
    // the edges stay out of the blurry part of the lens.
    L.unitsH = (std::min)(1.0f * pyt, L.bbH * 0.95f);
    L.unitsW = (std::min)(0.95f * pyt, eyes[0].halfWidth * 0.95f / L.sx);
    const float pw = L.unitsW * L.sx;
    for (int e = 0; e < 2; ++e) {
        const StereoPanelEye& eye = eyes[e];
        float x = eye.centreX - pw * 0.5f;
        float y = eye.centreY - L.unitsH * 0.5f;
        x = (std::max)(eye.halfX0, (std::min)(x, eye.halfX0 + eye.halfWidth - pw));
        y = (std::max)(0.0f, (std::min)(y, L.bbH - L.unitsH));
        L.ox[e] = std::round(x);
        L.oy[e] = std::round(y);
        L.clipX0[e] = eye.halfX0;
        L.clipX1[e] = eye.halfX0 + eye.halfWidth;
    }
    L.fontPx = 0.042f * pyt * g_prefs.uiScale;
    return L;
}

// ---- Input into ImGui ---------------------------------------------------
void FeedPad(ImGuiIO& io, const XINPUT_STATE* pad)
{
    const WORD b = pad ? pad->Gamepad.wButtons : 0;
    const auto key = [&](ImGuiKey k, WORD mask) { io.AddKeyEvent(k, (b & mask) != 0); };
    key(ImGuiKey_GamepadStart, XINPUT_GAMEPAD_START);
    key(ImGuiKey_GamepadBack, XINPUT_GAMEPAD_BACK);
    key(ImGuiKey_GamepadFaceLeft, XINPUT_GAMEPAD_X);
    key(ImGuiKey_GamepadFaceRight, XINPUT_GAMEPAD_B);
    key(ImGuiKey_GamepadFaceUp, XINPUT_GAMEPAD_Y);
    key(ImGuiKey_GamepadFaceDown, XINPUT_GAMEPAD_A);
    key(ImGuiKey_GamepadDpadLeft, XINPUT_GAMEPAD_DPAD_LEFT);
    key(ImGuiKey_GamepadDpadRight, XINPUT_GAMEPAD_DPAD_RIGHT);
    key(ImGuiKey_GamepadDpadUp, XINPUT_GAMEPAD_DPAD_UP);
    key(ImGuiKey_GamepadDpadDown, XINPUT_GAMEPAD_DPAD_DOWN);
    key(ImGuiKey_GamepadL1, XINPUT_GAMEPAD_LEFT_SHOULDER);
    key(ImGuiKey_GamepadR1, XINPUT_GAMEPAD_RIGHT_SHOULDER);
    key(ImGuiKey_GamepadL3, XINPUT_GAMEPAD_LEFT_THUMB);
    key(ImGuiKey_GamepadR3, XINPUT_GAMEPAD_RIGHT_THUMB);
    const float lt = pad ? pad->Gamepad.bLeftTrigger / 255.0f : 0.0f;
    const float rt = pad ? pad->Gamepad.bRightTrigger / 255.0f : 0.0f;
    io.AddKeyAnalogEvent(ImGuiKey_GamepadL2, lt > 0.3f, lt);
    io.AddKeyAnalogEvent(ImGuiKey_GamepadR2, rt > 0.3f, rt);
    const auto stick = [&](ImGuiKey k, short v, int sign) {
        constexpr float dead = 7849.0f;
        float f = (sign * static_cast<float>(v) - dead) / (32767.0f - dead);
        f = (std::max)(0.0f, (std::min)(f, 1.0f));
        io.AddKeyAnalogEvent(k, f > 0.1f, f);
    };
    const short lx = pad ? pad->Gamepad.sThumbLX : 0, ly = pad ? pad->Gamepad.sThumbLY : 0;
    stick(ImGuiKey_GamepadLStickLeft, lx, -1);
    stick(ImGuiKey_GamepadLStickRight, lx, +1);
    stick(ImGuiKey_GamepadLStickUp, ly, +1);
    stick(ImGuiKey_GamepadLStickDown, ly, -1);
}

void FeedMouseAndKeys(ImGuiIO& io, const Layout& L)
{
    InputBlockMouse m;
    InputBlock_TakeMouse(m);

    QueuedMsg msgs[_countof(g_msgs)];
    AcquireSRWLockExclusive(&g_msgLock);
    const int count = g_msgCount;
    for (int i = 0; i < count; ++i)
        msgs[i] = g_msgs[i];
    g_msgCount = 0;
    ReleaseSRWLockExclusive(&g_msgLock);
    for (int i = 0; i < count; ++i) {
        // With a DirectInput mouse the buttons come from there instead.
        if (m.fromDirectInput && IsMouseMsg(msgs[i].msg) && msgs[i].msg != WM_MOUSEWHEEL)
            continue;
        ImGui_ImplWin32_WndProcHandler(g_hwnd, msgs[i].msg, msgs[i].w, msgs[i].l);
    }
    // Modifier state from the system, not the window thread's key state.
    io.AddKeyEvent(ImGuiMod_Ctrl, (GetAsyncKeyState(VK_CONTROL) & 0x8000) != 0);
    io.AddKeyEvent(ImGuiMod_Shift, (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0);
    io.AddKeyEvent(ImGuiMod_Alt, (GetAsyncKeyState(VK_MENU) & 0x8000) != 0);

    // Pointer: both sources, whichever actually moved. DirectInput deltas
    // are the only thing that moves while the game holds the mouse
    // exclusively (gameplay); the Windows cursor is what moves when it
    // doesn't (menus, windowed). The first build picked ONE source by whether
    // DirectInput had been read recently, which left the pointer dead in
    // whichever case it guessed wrong.
    //
    // The Windows cursor wins whenever it is alive. Measured 2026-09-13 on the
    // title screen: the game reads its DirectInput mouse there too, WITHOUT
    // holding it, so both sources move at once - and adding the deltas on top
    // drifted the pointer away from where Windows had it (hovering one row
    // while the real cursor sat on another). Deltas are only used while the
    // Windows cursor has been still for a moment, which is what an exclusive
    // (gameplay) mouse looks like.
    static ULONGLONG s_realMovedMs = 0;
    const ULONGLONG nowMs = GetTickCount64();
    POINT p;
    RECT rc;
    if (GetCursorPos(&p) && ScreenToClient(g_hwnd, &p) && GetClientRect(g_hwnd, &rc) && rc.right > 0 && rc.bottom > 0) {
        const float rx = (p.x * L.bbW / rc.right - L.ox[0]) / L.sx, ry = p.y * L.bbH / rc.bottom - L.oy[0];
        const bool first = g_lastRealX <= -1e8f;
        const bool realMoved = !first && (std::fabs(rx - g_lastRealX) >= 1.0f || std::fabs(ry - g_lastRealY) >= 1.0f);
        // On opening, start where the Windows cursor already is if it is over the menu area.
        const bool startHere = first && rx >= 0.0f && ry >= 0.0f && rx < L.unitsW && ry < L.unitsH;
        g_lastRealX = rx;
        g_lastRealY = ry;
        if (realMoved || startHere) {
            s_realMovedMs = nowMs;
            g_cursorX = rx;
            g_cursorY = ry;
        }
    }
    if ((m.dx || m.dy) && nowMs - s_realMovedMs > 250) {
        const float speed = L.stereo ? 1.0f : (std::max)(1.0f, L.bbH / 1080.0f);
        g_cursorX += m.dx * speed;
        g_cursorY += m.dy * speed;
    }
    if (m.fromDirectInput) {
        for (int b = 0; b < 3; ++b)
            io.AddMouseButtonEvent(b, m.buttons[b]);
        if (m.wheel)
            io.AddMouseWheelEvent(0.0f, m.wheel / 120.0f);
    }
    g_cursorX = (std::max)(0.0f, (std::min)(g_cursorX, L.unitsW - 1.0f));
    g_cursorY = (std::max)(0.0f, (std::min)(g_cursorY, L.unitsH - 1.0f));
    // Only a real move is sent: ImGui treats ANY pointer movement as "the
    // mouse is in charge now" and hides the controller's highlight, so a
    // pointer that twitched every frame made the pad look dead.
    if (std::fabs(g_cursorX - g_sentX) >= 1.0f || std::fabs(g_cursorY - g_sentY) >= 1.0f) {
        io.AddMousePosEvent(g_cursorX, g_cursorY);
        g_sentX = g_cursorX;
        g_sentY = g_cursorY;
    }
}

// ---- Drawing --------------------------------------------------------------
void RenderToBackbuffer(IDirect3DDevice9* dev, const Layout& L)
{
    ImDrawData* dd = ImGui::GetDrawData();
    if (!dd || dd->CmdListsCount == 0 || dd->TotalVtxCount == 0)
        return;
    IDirect3DSurface9* bb = nullptr;
    if (FAILED(dev->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb)) || !bb)
        return;

    // Whatever the game left bound goes back afterwards.
    IDirect3DSurface9* oldRt[4] = {};
    for (DWORD i = 0; i < 4; ++i)
        dev->GetRenderTarget(i, &oldRt[i]);
    IDirect3DSurface9* oldDs = nullptr;
    dev->GetDepthStencilSurface(&oldDs);
    D3DVIEWPORT9 oldVp = {};
    dev->GetViewport(&oldVp);

    dev->SetRenderTarget(0, bb);
    for (DWORD i = 1; i < 4; ++i)
        if (oldRt[i])
            dev->SetRenderTarget(i, nullptr);
    dev->SetDepthStencilSurface(nullptr);

    g_drawing.store(true, std::memory_order_release);
    StereoTest_SetSuppressed(true); // one plain draw each - no per-eye duplication
    const bool began = SUCCEEDED(dev->BeginScene());

    // The backend draws into a viewport at (0,0) the size of DisplaySize, so
    // make that the whole backbuffer and move the geometry instead.
    dd->DisplayPos = ImVec2(0, 0);
    dd->DisplaySize = ImVec2(L.bbW, L.bbH);
    std::vector<ImVec4> clips;
    for (int n = 0; n < dd->CmdListsCount; ++n)
        for (const ImDrawCmd& cmd : dd->CmdLists[n]->CmdBuffer)
            clips.push_back(cmd.ClipRect);

    const int passes = L.stereo ? 2 : 1;
    for (int e = 0; e < passes; ++e) {
        size_t ci = 0;
        for (int n = 0; n < dd->CmdListsCount; ++n) {
            ImDrawList* list = dd->CmdLists[n];
            for (ImDrawVert& v : list->VtxBuffer) {
                if (e == 0) {
                    v.pos.x = v.pos.x * L.sx + L.ox[0];
                    v.pos.y = v.pos.y + L.oy[0];
                } else {
                    v.pos.x += L.ox[1] - L.ox[0];
                    v.pos.y += L.oy[1] - L.oy[0];
                }
            }
            for (ImDrawCmd& cmd : list->CmdBuffer) {
                const ImVec4& c = clips[ci++];
                const float x0 = L.stereo ? L.clipX0[e] : 0.0f, x1 = L.stereo ? L.clipX1[e] : L.bbW;
                cmd.ClipRect.x = (std::max)(x0, c.x * L.sx + L.ox[e]);
                cmd.ClipRect.z = (std::min)(x1, c.z * L.sx + L.ox[e]);
                cmd.ClipRect.y = (std::max)(0.0f, c.y + L.oy[e]);
                cmd.ClipRect.w = (std::min)(L.bbH, c.w + L.oy[e]);
            }
        }
        ImGui_ImplDX9_RenderDrawData(dd);
    }

    if (began)
        dev->EndScene();
    StereoTest_SetSuppressed(false);
    g_drawing.store(false, std::memory_order_release);

    for (DWORD i = 0; i < 4; ++i) {
        if (i == 0 || oldRt[i])
            dev->SetRenderTarget(i, oldRt[i]);
        if (oldRt[i])
            oldRt[i]->Release();
    }
    dev->SetDepthStencilSurface(oldDs);
    if (oldDs)
        oldDs->Release();
    dev->SetViewport(&oldVp);
    bb->Release();
}

// ---- The menu itself ------------------------------------------------------
void HelpMarker(const char* text)
{
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    ImGui::SetItemTooltip("%s", text);
}

bool ResetButton(const char* id)
{
    ImGui::PushID(id);
    const bool pressed = ImGui::SmallButton("Reset");
    ImGui::PopID();
    return pressed;
}

void StatusRow(const char* label, const char* fmt, ...)
{
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextDisabled("%s", label);
    ImGui::TableSetColumnIndex(1);
    va_list args;
    va_start(args, fmt);
    ImGui::TextV(fmt, args);
    va_end(args);
}

bool BeginStatusTable(const char* id)
{
    if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_RowBg))
        return false;
    // Same label width in every table, so the values line up down the page.
    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, ImGui::CalcTextSize("Image age at submit  ").x);
    ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
    return true;
}

int TabFlags(int index)
{
    return g_selectTab == index ? ImGuiTabItemFlags_SetSelected : 0;
}

void DrawCameraTab(AllSettings& s, bool& changed)
{
    ImGui::SeparatorText("First person");
    changed |= ImGui::Checkbox("First person camera", &s.cam.firstPerson);
    changed |= ImGui::Checkbox("Show head during action cameras", &s.cam.showHeadDuringActions);
    HelpMarker("When the game swings its camera out for a kick, a vault or a grab, Chris's head pops back in so "
               "you don't see a headless body. Off keeps the head hidden no matter where the camera goes.");

    ImGui::SeparatorText("Flat screen view");
    changed |= ImGui::SliderFloat("Field of view", &s.cam.flatFovDeg, 50.0f, 120.0f, "%.0f deg");
    HelpMarker("Horizontal. 75-80 tends to feel the right size for Chris; 90 reads wide.");
    changed |= ImGui::SliderFloat("Eye height##flat", &s.cam.flatEyeUp, 0.0f, 3.0f, "%.2f");
    HelpMarker("1.0 is the skeleton's eye, 0 is the head joint.");
    changed |= ImGui::SliderFloat("Eye forward##flat", &s.cam.flatEyeAhead, -1.0f, 2.0f, "%.2f");
    if (ResetButton("flatview")) {
        s.cam.flatFovDeg = g_defaults.cam.flatFovDeg;
        s.cam.flatEyeUp = g_defaults.cam.flatEyeUp;
        s.cam.flatEyeAhead = g_defaults.cam.flatEyeAhead;
        changed = true;
    }

    ImGui::SeparatorText("Picture");
    ImGui::BeginDisabled(!FilterPatch_IsAvailable());
    changed |= ImGui::Checkbox("Remove RE5's colour filter", &s.filterRemoved);
    ImGui::EndDisabled();
    HelpMarker(FilterPatch_IsAvailable() ? "The heavy yellow grade over everything. Untick for the original look."
                                         : "This game executable doesn't match the one the patch was made for.");
}

void DrawVrTab(AllSettings& s, bool& changed)
{
    VRBridgeStatus vr;
    VRBridge_GetStatus(vr);

    ImGui::SeparatorText("Headset");
    if (!vr.available) {
        ImGui::BeginDisabled();
        bool off = false;
        ImGui::Checkbox("Enable VR", &off);
        ImGui::EndDisabled();
        ImGui::TextWrapped("This is the flat-screen install: dgVoodoo2 isn't next to the game, so there's nothing "
                           "to send to a headset. Install the VR package to play in VR.");
        return;
    }
    bool enabled = vr.modeEnabled;
    if (ImGui::Checkbox("Enable VR", &enabled))
        VRBridge_RequestXrMode(enabled);
    if (vr.modeEnabled && vr.sessionRunning) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.5f, 0.85f, 0.5f, 1), "running");
    } else if (vr.modeEnabled) {
        ImGui::SameLine();
        ImGui::TextDisabled("starting...");
    } else if (vr.initFailed) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.9f, 0.6f, 0.3f, 1), "no headset / OpenXR runtime found last time");
    }
    changed |= ImGui::Checkbox("Start in VR automatically", &s.menu.autoStartVr);

    ImGui::BeginDisabled(!vr.sessionRunning);
    if (ImGui::Button("Reset view"))
        VRBridge_RequestRecenter("menu");
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextDisabled("or hold both sticks in for a second");

    ImGui::SeparatorText("View");
    changed |= ImGui::Checkbox("Head turns the game camera", &s.cam.headFollow);
    HelpMarker("While the gun is down the game's camera follows your head, so what's over your shoulder gets "
               "drawn. Raising the gun hands aim straight back to the mouse or stick.");
    changed |= ImGui::Checkbox("Stabilise camera", &s.cam.vrStabilise);
    HelpMarker("Smooths Chris's idle-animation sway out of your view.");
    changed |= ImGui::SliderFloat("Eye height##vr", &s.cam.vrEyeUp, 0.0f, 3.0f, "%.2f");
    changed |= ImGui::SliderFloat("Eye forward##vr", &s.cam.vrEyeAhead, -1.0f, 2.0f, "%.2f");
    changed |= ImGui::SliderFloat("World scale", &s.stereo.halfSeparation, 0.5f, 8.0f, "%.2f");
    HelpMarker("Eye separation in game units. Higher makes the world look smaller, lower makes it look bigger. "
               "2.75 was measured to make Sheva, guns and doors feel life-size.");
    if (ResetButton("vrview")) {
        s.cam.vrEyeUp = g_defaults.cam.vrEyeUp;
        s.cam.vrEyeAhead = g_defaults.cam.vrEyeAhead;
        s.stereo.halfSeparation = g_defaults.stereo.halfSeparation;
        changed = true;
    }

    ImGui::SeparatorText("HUD");
    // No distance slider: changing the convergence only spread the two eyes'
    // copies apart (user, 2026-09-13). Size is what makes it feel nearer or further.
    // Presented as distance because that is how it reads in the headset: the
    // HUD's size (0.3-1.0 of an eye) mapped so 0 is closest and 100 furthest.
    float hudDistance = (1.0f - s.stereo.hudScale) / 0.7f * 100.0f;
    if (ImGui::SliderFloat("HUD distance", &hudDistance, 0.0f, 100.0f, "%.0f")) {
        s.stereo.hudScale = 1.0f - (std::max)(0.0f, (std::min)(hudDistance, 100.0f)) / 100.0f * 0.7f;
        changed = true;
    }
    HelpMarker("0 is closest, 100 is furthest.");
    if (ResetButton("hud")) {
        s.stereo.hudScale = g_defaults.stereo.hudScale;
        changed = true;
    }

    if (ImGui::CollapsingHeader("Advanced")) {
        changed |= ImGui::SliderFloat("Head prediction", &s.vr.headPredictMs, 0.0f, 60.0f, "%.0f ms");
        HelpMarker("Pushes the view ahead while your head is turning, to hide pipeline delay. Does nothing once "
                   "you stop. 0 is off.");
        changed |= ImGui::SliderFloat("Head rotation gain", &s.vr.headRotationGain, 0.5f, 3.0f, "%.1fx");
        HelpMarker("1.0 is 1:1 with your neck. Anything else overshoots when you stop turning - most people "
                   "should leave this alone.");
        changed |= ImGui::SliderFloat("Extra FOV", &s.stereo.fovWiden, 0.5f, 2.0f, "%.2fx");
        changed |= ImGui::Checkbox("Match culling to the headset's FOV", &s.cam.vrMatchCullFov);
        HelpMarker("Tells the game to draw everything the headset can see. Off falls back to the game's 90 "
                   "degrees, and things at the edge of your view vanish.");
        changed |= ImGui::Checkbox("Mono post-processing (light-leak fix)", &s.stereo.monoSmallTargets);
        if (ResetButton("advanced")) {
            s.vr.headPredictMs = g_defaults.vr.headPredictMs;
            s.vr.headRotationGain = g_defaults.vr.headRotationGain;
            s.stereo.fovWiden = g_defaults.stereo.fovWiden;
            s.cam.vrMatchCullFov = g_defaults.cam.vrMatchCullFov;
            s.stereo.monoSmallTargets = g_defaults.stereo.monoSmallTargets;
            changed = true;
        }
    }
}

void DrawStatusTab()
{
    VRBridgeStatus vr;
    VRBridge_GetStatus(vr);
    CameraRigStatus cam;
    CameraRigHook_GetStatus(cam);

    ImGui::SeparatorText("Mod");
    if (BeginStatusTable("mod")) {
#if RE5VR_DIAGNOSTICS
        StatusRow("Build", "developer (diagnostics on)");
#else
        StatusRow("Build", "release");
#endif
        StatusRow("Install", vr.available ? "VR (dgVoodoo2 found)" : "flat screen (no dgVoodoo2)");
        MEMORYSTATUSEX mem = {};
        mem.dwLength = sizeof(mem);
        const bool laa = ExeIsLargeAddressAware();
        if (GlobalMemoryStatusEx(&mem)) {
            const double totalGb = mem.ullTotalVirtual / 1073741824.0;
            const double usedGb = (mem.ullTotalVirtual - mem.ullAvailVirtual) / 1073741824.0;
            StatusRow("Address space", "%.2f of %.1f GB used", usedGb, totalGb);
        }
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("4GB patch");
        ImGui::TableSetColumnIndex(1);
        if (laa)
            ImGui::TextColored(ImVec4(0.5f, 0.85f, 0.5f, 1), "applied");
        else
            ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.3f, 1), "NOT applied - expect stutters and crashes");
        ImGui::EndTable();
    }

    ImGui::SeparatorText("Rendering");
    if (BeginStatusTable("render")) {
        StatusRow("Game frame rate", "%.0f fps (%.1f ms)", g_frameMsAvg > 0 ? 1000.0f / g_frameMsAvg : 0.0f, g_frameMsAvg);
        D3DSURFACE_DESC d = {};
        IDirect3DSurface9* bb = nullptr;
        if (g_device && SUCCEEDED(g_device->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb)) && bb) {
            bb->GetDesc(&d);
            bb->Release();
        }
        StatusRow("Backbuffer", "%u x %u", d.Width, d.Height);
        StatusRow("Stereo", StereoTest_IsEnabled() ? "on - each eye gets %u x %u" : "off", d.Width / 2, d.Height);
        int hudVs = 0, hudPs = 0;
        unsigned seen = 0;
        HudShaders_GetCounts(&hudVs, &hudPs, &seen);
        StatusRow("HUD shaders", "%d of 2 vertex, %d of 1 pixel recognised (%u shaders seen)", hudVs, hudPs, seen);
        ImGui::EndTable();
    }

    ImGui::SeparatorText("VR");
    if (BeginStatusTable("vr")) {
        StatusRow("State", !vr.available ? "unavailable" : vr.sessionRunning ? "running" : vr.modeEnabled ? "starting" : "off");
        if (vr.runtimeName[0])
            StatusRow("Runtime", "%s", vr.runtimeName);
        if (vr.systemName[0])
            StatusRow("Headset", "%s", vr.systemName);
        if (vr.recommendedEyeWidth)
            StatusRow("Runtime wants", "%u x %u per eye", vr.recommendedEyeWidth, vr.recommendedEyeHeight);
        if (vr.eyeWidth)
            StatusRow("We send", "%u x %u per eye%s", vr.eyeWidth, vr.eyeHeight,
                vr.recommendedEyeWidth > vr.eyeWidth ? " (upscaled by the runtime)" : "");
        if (vr.sessionRunning) {
            StatusRow("Headset refresh", "%.0f Hz", vr.predictedDisplayPeriodMs > 0 ? 1000.0f / vr.predictedDisplayPeriodMs : 0.0f);
            StatusRow("Frames submitted", "%.0f / s", vr.submitHz);
            StatusRow("Image age at submit", "%.1f ms", vr.imageAgeMs);
        }
        ImGui::EndTable();
    }

    ImGui::SeparatorText("Camera");
    if (BeginStatusTable("cam")) {
        StatusRow("First person", CameraRigHook_IsEnabled() ? "on" : "off");
        static const char* const kPlayers[] = { "not identified yet", "Chris", "Sheva", "someone else" };
        if (cam.player)
            StatusRow("Player", "%s (%d joints)", kPlayers[cam.player], cam.playerJointCount);
        else
            StatusRow("Player", "%s", kPlayers[0]);
        if (cam.cameraHookAgeMs == ~0ull)
            StatusRow("Camera hook", "not seen yet");
        else if (cam.cameraHookAgeMs < 100)
            StatusRow("Camera hook", "live");
        else
            StatusRow("Camera hook", "quiet for %.1f s", cam.cameraHookAgeMs / 1000.0f);
        StatusRow("Head turns camera", cam.headFollowDriving ? "yes, right now" : "not right now");
        StatusRow("Action cameras", "%lu cut-away(s), %lu watchdog restore(s), %lu flicker lockout(s)",
            cam.headCutaways, cam.watchdogRestores, cam.flickerLockouts);
        ImGui::EndTable();
    }
}

void DrawMenuTab(AllSettings& s, bool& changed, bool& resetAll)
{
    ImGui::SeparatorText("Menu");
    changed |= ImGui::SliderFloat("Text size", &s.menu.uiScale, 0.75f, 2.0f, "%.2fx");
    changed |= ImGui::Checkbox("Open with a click of both sticks", &s.menu.padChord);
    changed |= ImGui::Checkbox("Show the menu hint at startup", &s.menu.startupHint);

    ImGui::SeparatorText("Controls");
    ImGui::BulletText("Insert, or click both sticks: open / close this menu");
    ImGui::BulletText("Hold both sticks in for a second: reset VR view");
    ImGui::BulletText("Pad: d-pad or left stick to move, A to select, B to close");
    ImGui::BulletText("LB / RB: switch tabs; on a slider, A then d-pad left / right");
    ImGui::BulletText("Keyboard: arrows to move, Space to select, Esc to close");
    ImGui::BulletText("On a slider: Space then arrows, or Ctrl+click to type a value");

    ImGui::SeparatorText("Settings");
    ImGui::TextWrapped("Changes save automatically to %s", g_iniPath);
    if (ImGui::Button("Reset everything to defaults"))
        ImGui::OpenPopup("confirm reset");
    if (ImGui::BeginPopupModal("confirm reset", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Put every option back to its default?");
        if (ImGui::Button("Reset")) {
            resetAll = true;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

#if RE5VR_DIAGNOSTICS
void DrawDeveloperTab(AllSettings& s, bool& changed)
{
    ImGui::TextDisabled("Developer builds only. None of these are saved.");
    bool stereo = StereoTest_IsEnabled();
    if (ImGui::Checkbox("Stereo without VR (was F8)", &stereo))
        StereoTest_SetEnabled(stereo);
    changed |= ImGui::Checkbox("Don't re-rotate eyes while head steers camera (was \\)", &s.stereo.compensateHeadFollow);
    changed |= ImGui::Checkbox("Direct submit (was Delete)", &s.vr.directSubmit);
    changed |= ImGui::Checkbox("Producer waits for consumer (was Insert)", &s.vr.waitForConsumer);
    changed |= ImGui::Checkbox("Co-op aim-walk step commit (was F6)", &s.cam.aimWalkCommit);
    HelpMarker("The gun fires several rounds per trigger pull while this is on.");
}
#endif

void DrawMenu(const Layout& L)
{
    AllSettings s = CaptureSettings();
    bool changed = false, resetAll = false;

    if (L.stereo) {
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(L.unitsW, L.unitsH));
    } else {
        const float k = L.fontPx / 18.0f;
        ImGui::SetNextWindowSize(ImVec2(560 * k, 640 * k), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowPos(ImVec2(L.unitsW * 0.5f, L.unitsH * 0.5f), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));
    }
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;
    if (L.stereo)
        flags |= ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
    bool keepOpen = true;
    if (ImGui::Begin(kTitle, &keepOpen, flags)) {
        if (ImGui::BeginTabBar("tabs")) {
            int index = 0;
            const auto tab = [&](const char* name) {
                const bool open = ImGui::BeginTabItem(name, nullptr, TabFlags(index));
                if (open)
                    g_currentTab = index;
                ++index;
                return open;
            };
            if (tab("Camera")) {
                DrawCameraTab(s, changed);
                ImGui::EndTabItem();
            }
            if (tab("VR")) {
                DrawVrTab(s, changed);
                ImGui::EndTabItem();
            }
            if (tab("Status")) {
                DrawStatusTab();
                ImGui::EndTabItem();
            }
            if (tab("Menu")) {
                DrawMenuTab(s, changed, resetAll);
                ImGui::EndTabItem();
            }
#if RE5VR_DIAGNOSTICS
            if (tab("Developer")) {
                DrawDeveloperTab(s, changed);
                ImGui::EndTabItem();
            }
#endif
            ImGui::EndTabBar();
        }
    }
    ImGui::End();
    g_selectTab = -1;

    if (resetAll) {
        s = g_defaults;
        changed = true;
        Log_Printf("Menu: everything reset to defaults");
    }
    if (changed) {
        ApplySettings(s);
        g_dirty = true;
        g_lastChangeMs = GetTickCount64();
    }
    if (!keepOpen)
        g_open = false;
}

// The menu's own mouse pointer. ImGui's built-in software cursor never showed
// in gameplay (2026-09-13: the log had the pointer moving and hovering while
// the user saw nothing - on menus they had only ever been looking at the
// game's Windows cursor, which RE5 hides once you are playing). Drawn as plain
// polygons in the foreground list, so it needs nothing from the font atlas and
// lands in both eyes in VR like the rest of the menu.
// The game's own pointer graphic, copied into a texture so the menu's
// pointer looks the same everywhere, gameplay and VR included (user request,
// 2026-09-13). Windows cursors carry their transparency as a mask or an alpha
// channel; drawing the cursor once over black and once over white recovers
// both kinds the same way: alpha = 255 - (white - black), colour = black / alpha.
IDirect3DTexture9* g_cursorTex = nullptr;
HCURSOR g_cursorTexFor = nullptr;
int g_cursorW = 0, g_cursorH = 0, g_cursorHotX = 0, g_cursorHotY = 0;

// Only a cursor the game itself set, and never one of Windows' stock cursors:
// the first version fell back to the window class cursor, which at startup is
// the plain Windows arrow (handle 00010003), and locked that in before RE5's
// own pointer ever appeared (user, 2026-09-13: "shows the windows cursor").
bool IsSystemCursor(HCURSOR cursor)
{
    static const LPCSTR kIds[] = { IDC_ARROW, IDC_IBEAM, IDC_WAIT, IDC_CROSS, IDC_UPARROW, IDC_SIZENWSE, IDC_SIZENESW,
        IDC_SIZEWE, IDC_SIZENS, IDC_SIZEALL, IDC_NO, IDC_HAND, IDC_APPSTARTING, IDC_HELP };
    for (LPCSTR id : kIds)
        if (LoadCursorA(nullptr, id) == cursor)
            return true;
    return false;
}

HCURSOR GameCursorHandle()
{
    const HCURSOR c = InputBlock_GameCursor();
    return c && !IsSystemCursor(c) ? c : nullptr;
}

bool BuildCursorTexture(IDirect3DDevice9* dev, HCURSOR cursor)
{
    ICONINFO ii = {};
    if (!GetIconInfo(cursor, &ii))
        return false;
    BITMAP bm = {};
    int w = 0, h = 0;
    if (ii.hbmColor && GetObjectA(ii.hbmColor, sizeof(bm), &bm)) {
        w = bm.bmWidth;
        h = bm.bmHeight;
    } else if (ii.hbmMask && GetObjectA(ii.hbmMask, sizeof(bm), &bm)) {
        w = bm.bmWidth;
        h = bm.bmHeight / 2; // monochrome cursors stack AND and XOR masks
    }
    const int hotX = static_cast<int>(ii.xHotspot), hotY = static_cast<int>(ii.yHotspot);
    if (ii.hbmColor)
        DeleteObject(ii.hbmColor);
    if (ii.hbmMask)
        DeleteObject(ii.hbmMask);
    if (w <= 0 || h <= 0 || w > 256 || h > 256)
        return false;

    HDC screen = GetDC(nullptr);
    HDC mem = CreateCompatibleDC(screen);
    ReleaseDC(nullptr, screen);
    BITMAPINFO bi = {};
    bi.bmiHeader.biSize = sizeof(bi.bmiHeader);
    bi.bmiHeader.biWidth = w;
    bi.bmiHeader.biHeight = -h; // top-down
    bi.bmiHeader.biPlanes = 1;
    bi.bmiHeader.biBitCount = 32;
    bi.bmiHeader.biCompression = BI_RGB;
    std::vector<BYTE> onBlack, onWhite;
    for (int pass = 0; pass < 2; ++pass) {
        void* bits = nullptr;
        HBITMAP dib = CreateDIBSection(mem, &bi, DIB_RGB_COLORS, &bits, nullptr, 0);
        if (!dib || !bits) {
            DeleteDC(mem);
            return false;
        }
        HGDIOBJ old = SelectObject(mem, dib);
        std::memset(bits, pass ? 0xFF : 0x00, static_cast<size_t>(w) * h * 4);
        DrawIconEx(mem, 0, 0, cursor, w, h, 0, nullptr, DI_NORMAL);
        (pass ? onWhite : onBlack).assign(static_cast<BYTE*>(bits), static_cast<BYTE*>(bits) + static_cast<size_t>(w) * h * 4);
        SelectObject(mem, old);
        DeleteObject(dib);
    }
    DeleteDC(mem);

    IDirect3DTexture9* tex = nullptr;
    if (FAILED(dev->CreateTexture(w, h, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &tex, nullptr)) || !tex)
        return false;
    D3DLOCKED_RECT lr;
    if (FAILED(tex->LockRect(0, &lr, nullptr, 0))) {
        tex->Release();
        return false;
    }
    int opaque = 0;
    for (int y = 0; y < h; ++y) {
        auto* row = static_cast<BYTE*>(lr.pBits) + y * lr.Pitch;
        for (int x = 0; x < w; ++x) {
            const BYTE* b = &onBlack[(y * w + x) * 4];
            const BYTE* wt = &onWhite[(y * w + x) * 4];
            int spread = 0;
            for (int c = 0; c < 3; ++c)
                spread = (std::max)(spread, wt[c] - b[c]);
            const int a = 255 - (std::min)(255, (std::max)(0, spread));
            for (int c = 0; c < 3; ++c)
                row[x * 4 + c] = a ? static_cast<BYTE>((std::min)(255, b[c] * 255 / a)) : 0;
            row[x * 4 + 3] = static_cast<BYTE>(a);
            if (a > 32)
                ++opaque;
        }
    }
    tex->UnlockRect(0);

    // In gameplay RE5 hides its pointer by setting a fully transparent cursor
    // (log, 2026-09-13: handle 082C0DCB, 32x32, and the menu pointer vanished
    // in game). A graphic with next to nothing visible is never adopted.
    if (opaque < 8) {
        tex->Release();
        Log_Printf("Menu: game cursor %p is blank (%d visible pixel(s)) - keeping the pointer we have", cursor, opaque);
        return false;
    }

    if (g_cursorTex)
        g_cursorTex->Release();
    g_cursorTex = tex;
    g_cursorTexFor = cursor;
    g_cursorW = w;
    g_cursorH = h;
    g_cursorHotX = hotX;
    g_cursorHotY = hotY;
    Log_Printf("Menu: pointer graphic taken from the game's cursor %p (%dx%d, hotspot %d,%d)", cursor, w, h, hotX, hotY);
    return true;
}

// Captures the game's menu pointer graphic ONCE, the first time it shows a
// visible one - normally on the title screen, before our menu is ever opened.
// Kept from then on: the game swaps handles (menu pointer, a brief variant, a
// blank one in gameplay) and the user wants the menu pointer look everywhere.
void CaptureGameCursorOnce()
{
    if (g_cursorTex || !g_device)
        return;
    const HCURSOR cursor = GameCursorHandle();
    static HCURSOR s_tried[8] = {};
    if (!cursor)
        return;
    for (HCURSOR t : s_tried)
        if (t == cursor)
            return;
    for (HCURSOR& t : s_tried) {
        if (!t) {
            t = cursor;
            break;
        }
    }
    BuildCursorTexture(g_device, cursor);
}

void DrawPointer(const Layout& L)
{
    const ImGuiIO& io = ImGui::GetIO();
    if (!ImGui::IsMousePosValid(&io.MousePos))
        return;

    if (g_cursorTex) {
        // Native size at 1080p, growing with the text beyond that.
        const float s = (std::max)(1.0f, L.fontPx / 20.0f);
        const ImVec2 p0(io.MousePos.x - g_cursorHotX * s, io.MousePos.y - g_cursorHotY * s);
        const ImVec2 p1(p0.x + g_cursorW * s, p0.y + g_cursorH * s);
        ImGui::GetForegroundDrawList()->AddImage(reinterpret_cast<ImTextureID>(g_cursorTex), p0, p1);
        return;
    }

    // Fallback until the game has shown a cursor: a plain drawn arrow.
    const float k = (std::max)(1.0f, L.fontPx / 16.0f);
    static const ImVec2 kArrow[] = { { 0, 0 }, { 0, 17 }, { 4, 13 }, { 7, 20 }, { 10, 19 }, { 7, 12 }, { 12, 12 } };
    ImVec2 pts[_countof(kArrow)], shadow[_countof(kArrow)];
    for (size_t i = 0; i < _countof(kArrow); ++i) {
        pts[i] = ImVec2(io.MousePos.x + kArrow[i].x * k, io.MousePos.y + kArrow[i].y * k);
        shadow[i] = ImVec2(pts[i].x + 1.5f * k, pts[i].y + 1.5f * k);
    }
    // The concave fill left ragged edges (user screenshot, 2026-09-13).
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    // The notch makes the arrow concave, so it is filled as three triangles
    // and the tail quad, with fill anti-aliasing off so the pieces don't leave
    // faint seams where they meet. The outline on top keeps the edge smooth.
    const auto fill = [&](const ImVec2* v, ImU32 col) {
        dl->AddTriangleFilled(v[0], v[1], v[2], col);
        dl->AddTriangleFilled(v[0], v[2], v[5], col);
        dl->AddTriangleFilled(v[0], v[5], v[6], col);
        const ImVec2 tail[] = { v[2], v[3], v[4], v[5] };
        dl->AddConvexPolyFilled(tail, _countof(tail), col);
    };
    const ImDrawListFlags flags = dl->Flags;
    dl->Flags &= ~ImDrawListFlags_AntiAliasedFill;
    fill(shadow, IM_COL32(0, 0, 0, 90));
    fill(pts, IM_COL32_WHITE);
    dl->Flags = flags;
    dl->AddPolyline(pts, _countof(kArrow), IM_COL32_BLACK, ImDrawFlags_Closed, (std::max)(1.0f, 1.2f * k));
}

void DrawHint(const Layout& L, float alpha, float secondsLeft)
{
    const float pad = L.fontPx;
    ImGui::SetNextWindowPos(ImVec2(L.stereo ? L.unitsW * 0.5f : pad, pad), ImGuiCond_Always,
        ImVec2(L.stereo ? 0.5f : 0.0f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.85f * alpha);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, alpha);
    ImGui::Begin("##hint", nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoInputs |
            ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoSavedSettings);
    ImGui::TextUnformatted(kTitle);
    ImGui::TextDisabled(g_prefs.padChord ? "Insert or click both sticks for the menu" : "Insert for the menu");
    ImGui::TextDisabled("This hides in %d seconds", static_cast<int>(std::ceil((std::max)(0.0f, secondsLeft))));
    ImGui::End();
    ImGui::PopStyleVar();
}

bool EnsureImGui(IDirect3DDevice9* device)
{
    if (g_imguiReady)
        return true;
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr; // our settings live in re5vr.ini; window layout isn't worth a file
    io.LogFilename = nullptr;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_NavEnableGamepad;
    io.BackendFlags |= ImGuiBackendFlags_HasGamepad;
    if (!ImGui_ImplWin32_Init(g_hwnd) || !ImGui_ImplDX9_Init(device)) {
        Log_Printf("Menu: ImGui backend init failed - no menu this session");
        ImGui::DestroyContext();
        return false;
    }
    g_imguiReady = true;
    Log_Printf("Menu: Dear ImGui %s ready", IMGUI_VERSION);
    return true;
}

void OpenMenu(bool open)
{
    if (open == g_open)
        return;
    g_open = open;
}

} // namespace

void Menu_Install(IDirect3DDevice9* device)
{
    g_device = device;
    QueryPerformanceFrequency(&g_qpcFreq);

    D3DDEVICE_CREATION_PARAMETERS cp = {};
    if (SUCCEEDED(device->GetCreationParameters(&cp)))
        g_hwnd = cp.hFocusWindow;
    if (!g_hwnd) {
        IDirect3DSwapChain9* sc = nullptr;
        if (SUCCEEDED(device->GetSwapChain(0, &sc)) && sc) {
            D3DPRESENT_PARAMETERS pp = {};
            if (SUCCEEDED(sc->GetPresentParameters(&pp)))
                g_hwnd = pp.hDeviceWindow;
            sc->Release();
        }
    }
    if (g_hwnd) {
        g_gameWndProc = reinterpret_cast<WNDPROC>(
            SetWindowLongPtrA(g_hwnd, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(&MenuWndProc)));
    }
    Log_Printf("Menu: game window %p, window procedure %s", g_hwnd, g_gameWndProc ? "hooked" : "NOT hooked");
    InputBlock_SetMessageSink(&QueueMessage);

    // re5vr.ini next to the game's exe.
    GetModuleFileNameA(nullptr, g_iniPath, MAX_PATH);
    char* slash = strrchr(g_iniPath, '\\');
    if (slash)
        strcpy_s(slash + 1, MAX_PATH - (slash + 1 - g_iniPath), "re5vr.ini");

    // Everything as the code ships it, before the file has a say - this is
    // what the Reset buttons go back to.
    g_defaults = CaptureSettings();
    AllSettings s = g_defaults;
    const bool haveFile = GetFileAttributesA(g_iniPath) != INVALID_FILE_ATTRIBUTES;
    if (haveFile) {
        LoadSettings(s);
        ApplySettings(s);
        Log_Printf("Menu: settings loaded from %s", g_iniPath);
    } else {
        Log_Printf("Menu: no %s yet - defaults, written on the first change", g_iniPath);
    }
    g_savedSnapshot = Serialize(CaptureSettings());
}

bool Menu_IsDrawing()
{
    return g_drawing.load(std::memory_order_acquire);
}

void Menu_OnBeforeReset()
{
    if (g_imguiReady)
        ImGui_ImplDX9_InvalidateDeviceObjects();
}

void Menu_OnAfterReset()
{
    // Device objects are recreated lazily by the next NewFrame.
}

void Menu_OnPresent(IDirect3DDevice9* device)
{
    if (!g_hwnd || device != g_device)
        return;
    const ULONGLONG nowMs = GetTickCount64();
    if (!g_firstPresentMs)
        g_firstPresentMs = nowMs;

    // Frame time, smoothed.
    LARGE_INTEGER qpc;
    QueryPerformanceCounter(&qpc);
    if (g_lastPresentQpc.QuadPart) {
        const float ms = static_cast<float>(qpc.QuadPart - g_lastPresentQpc.QuadPart) * 1000.0f /
            static_cast<float>(g_qpcFreq.QuadPart);
        g_frameMsAvg = g_frameMsAvg > 0 ? g_frameMsAvg + (ms - g_frameMsAvg) * 0.05f : ms;
    }
    const float deltaSec = g_lastPresentQpc.QuadPart
        ? static_cast<float>(qpc.QuadPart - g_lastPresentQpc.QuadPart) / static_cast<float>(g_qpcFreq.QuadPart)
        : 1.0f / 60.0f;
    g_lastPresentQpc = qpc;

    // Start in VR, once the game has had a few seconds to get going.
    if (!g_autoVrDone && nowMs - g_firstPresentMs > 5000) {
        g_autoVrDone = true;
        if (g_prefs.autoStartVr && VRBridge_IsAvailable()) {
            Log_Printf("Menu: starting VR automatically (StartInVR=1)");
            VRBridge_RequestXrMode(true);
        }
    }

    // ---- Open / close ----
    const bool wasOpen = g_open;
    static bool s_prevInsert = false;
    const bool insert = GameIsForeground() && (GetAsyncKeyState(VK_INSERT) & 0x8000) != 0;
    if (insert && !s_prevInsert)
        OpenMenu(!g_open);
    s_prevInsert = insert;

    XINPUT_STATE pad = {};
    const bool havePad = InputBlock_ReadPad(&pad);
    const WORD buttons = havePad ? pad.Gamepad.wButtons : 0;
    {
        // Both sticks: a quick click toggles the menu, a one-second hold resets
        // the VR view (once per hold). Decided on release, so a hold never
        // flashes the menu open first.
        constexpr WORD kBoth = XINPUT_GAMEPAD_LEFT_THUMB | XINPUT_GAMEPAD_RIGHT_THUMB;
        static ULONGLONG s_chordSinceMs = 0;
        static bool s_recentered = false;
        const bool both = (buttons & kBoth) == kBoth;
        if (both) {
            if (!s_chordSinceMs)
                s_chordSinceMs = nowMs;
            if (!s_recentered && nowMs - s_chordSinceMs >= 1000 && CameraRigHook_IsVrActive()) {
                s_recentered = true;
                VRBridge_RequestRecenter("both sticks held");
            }
        } else if (s_chordSinceMs) {
            if (!s_recentered && nowMs - s_chordSinceMs < 500 && g_prefs.padChord)
                OpenMenu(!g_open);
            s_chordSinceMs = 0;
            s_recentered = false;
        }
    }
    static WORD s_prevButtons = 0;
    const WORD pressed = buttons & ~s_prevButtons;
    s_prevButtons = buttons;

    if (g_open && wasOpen && g_imguiReady) {
        // B or Escape closes, unless it is busy cancelling something in the menu.
        const bool busy = ImGui::IsAnyItemActive() || ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId);
        const bool esc = GameIsForeground() && (GetAsyncKeyState(VK_ESCAPE) & 0x8000) != 0;
        static bool s_prevEsc = false;
        if (!busy && (((pressed & XINPUT_GAMEPAD_B) != 0) || (esc && !s_prevEsc)))
            g_open = false;
        s_prevEsc = esc;
        if (pressed & XINPUT_GAMEPAD_LEFT_SHOULDER)
            g_selectTab = (std::max)(0, g_currentTab - 1);
        if (pressed & XINPUT_GAMEPAD_RIGHT_SHOULDER)
            g_selectTab = g_currentTab + 1;
    }

    if (g_open != wasOpen) {
        InputBlock_SetBlocking(g_open);
        Log_Printf("Menu: %s", g_open ? "opened" : "closed");
        g_cursorX = g_cursorY = -1.0f; // recentred below once the layout is known
        g_sentX = g_sentY = g_lastRealX = g_lastRealY = -1e9f;
    }

    // ---- Save, a second after the last change or when the menu closes ----
    if (g_dirty && (!g_open || nowMs - g_lastChangeMs > 1000)) {
        g_dirty = false;
        const AllSettings now = CaptureSettings();
        const std::string snapshot = Serialize(now);
        if (snapshot != g_savedSnapshot) {
            SaveSettings(now);
            g_savedSnapshot = snapshot;
        }
    }

    // The hint's clock only runs while the game is in front and presenting
    // real frames: the first presents are black loading frames nobody sees,
    // and the first build burned the whole hint on those.
    static ULONGLONG s_hintStartMs = 0;
    if (!s_hintStartMs && deltaSec < 0.1f && GameIsForeground())
        s_hintStartMs = nowMs;
    const float hintAgeSec = s_hintStartMs ? (nowMs - s_hintStartMs) / 1000.0f : 0.0f;
    constexpr float kHintSec = 10.0f;
    const bool showHint = g_prefs.startupHint && hintAgeSec < kHintSec && !g_open;
    // Every frame until it has one - the title screen is where it shows first.
    CaptureGameCursorOnce();
    if (!g_open && !showHint)
        return;
    if (device->TestCooperativeLevel() != D3D_OK)
        return; // lost device: nothing can be drawn until the game resets it

    IDirect3DSurface9* bb = nullptr;
    D3DSURFACE_DESC desc = {};
    if (FAILED(device->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb)) || !bb)
        return;
    bb->GetDesc(&desc);
    bb->Release();
    if (desc.Width < 64 || desc.Height < 64)
        return;

    if (!EnsureImGui(device))
        return;
    const Layout L = ComputeLayout(desc.Width, desc.Height);
    EnsureFont(L.fontPx);

    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(L.unitsW, L.unitsH);
    io.DeltaTime = (std::max)(1.0f / 1000.0f, (std::min)(deltaSec, 0.25f));
    io.MouseDrawCursor = false; // we draw our own - see DrawPointer
    if (g_cursorX < 0.0f) {
        g_cursorX = L.unitsW * 0.5f;
        g_cursorY = L.unitsH * 0.5f;
    }
    if (g_open) {
        FeedMouseAndKeys(io, L);
        FeedPad(io, havePad ? &pad : nullptr);
    } else {
        FeedPad(io, nullptr);
        io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
    }

    const bool openBeforeDraw = g_open;
    ImGui_ImplDX9_NewFrame();
    ImGui::NewFrame();
    if (g_open) {
        DrawMenu(L);
        DrawPointer(L);
    } else {
        DrawHint(L, (std::min)(1.0f, (kHintSec - hintAgeSec) / 1.0f), kHintSec - hintAgeSec);
    }
    ImGui::Render();
    RenderToBackbuffer(device, L);

    // The close button on the window.
    if (openBeforeDraw && !g_open) {
        InputBlock_SetBlocking(false);
        Log_Printf("Menu: closed");
    }
}
