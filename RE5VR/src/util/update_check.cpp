#include "update_check.h"
#include "log.h"
#include "version.h"

#include <windows.h>
#include <winhttp.h>
#include <shellapi.h>
#include <objbase.h>

#include <atomic>
#include <cctype>
#include <cstring>
#include <mutex>

namespace {

// The mod on Nexus: nexusmods.com/residentevil5goldedition/mods/1107
constexpr const char* kNexusUrl = "https://www.nexusmods.com/residentevil5goldedition/mods/1107";
constexpr const wchar_t* kApiHost = L"api.nexusmods.com";
constexpr const wchar_t* kApiPath = L"/v2/graphql";
constexpr const char* kQuery =
    "{\"query\":\"{ legacyModsByDomain(ids: [{ gameDomain: \\\"residentevil5goldedition\\\", modId: 1107 }]) "
    "{ nodes { version } } }\"}";

std::atomic<int> g_state{static_cast<int>(UpdateState::Off)};
std::atomic<bool> g_running{false};
// Bumped by SetOff, so a check that finishes after the player turned it off
// doesn't bring its answer back.
std::atomic<unsigned> g_generation{0};
std::mutex g_latestMutex;
std::string g_latest;

// POSTs the query and returns the body, or an empty string.
std::string Fetch()
{
    std::string body;
    const std::wstring agent = L"TrueFP-VR/" RE5VR_VERSION_W;
    HINTERNET session = WinHttpOpen(agent.c_str(), WINHTTP_ACCESS_TYPE_DEFAULT_PROXY, WINHTTP_NO_PROXY_NAME,
        WINHTTP_NO_PROXY_BYPASS, 0);
    if (!session) {
        Log_Printf("UpdateCheck: WinHttpOpen failed (%lu)", GetLastError());
        return body;
    }
    WinHttpSetTimeouts(session, 5000, 5000, 5000, 10000);
    HINTERNET connect = WinHttpConnect(session, kApiHost, INTERNET_DEFAULT_HTTPS_PORT, 0);
    HINTERNET request = connect ? WinHttpOpenRequest(connect, L"POST", kApiPath, nullptr, WINHTTP_NO_REFERER,
                                      WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE)
                                : nullptr;
    const DWORD queryLen = static_cast<DWORD>(strlen(kQuery));
    bool ok = request &&
        WinHttpSendRequest(request, L"Content-Type: application/json\r\n", static_cast<DWORD>(-1),
            const_cast<LPVOID>(static_cast<const void*>(kQuery)), queryLen, queryLen, 0) &&
        WinHttpReceiveResponse(request, nullptr);
    if (!ok) {
        Log_Printf("UpdateCheck: request to Nexus failed (%lu)", GetLastError());
    } else {
        DWORD status = 0, size = sizeof(status);
        WinHttpQueryHeaders(request, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX,
            &status, &size, WINHTTP_NO_HEADER_INDEX);
        for (;;) {
            DWORD avail = 0;
            if (!WinHttpQueryDataAvailable(request, &avail) || avail == 0)
                break;
            const size_t old = body.size();
            if (old + avail > 65536) // the answer is under 100 bytes
                break;
            body.resize(old + avail);
            DWORD read = 0;
            if (!WinHttpReadData(request, &body[old], avail, &read)) {
                body.resize(old);
                break;
            }
            body.resize(old + read);
        }
        if (status != 200) {
            Log_Printf("UpdateCheck: Nexus answered HTTP %lu", status);
            body.clear();
        }
    }
    if (request)
        WinHttpCloseHandle(request);
    if (connect)
        WinHttpCloseHandle(connect);
    WinHttpCloseHandle(session);
    return body;
}

// Pulls "version":"..." out of the answer. The only string field we ask for.
bool ParseVersion(const std::string& body, std::string& out)
{
    constexpr const char* kKey = "\"version\":\"";
    size_t at = body.find(kKey);
    if (at == std::string::npos)
        return false;
    at += strlen(kKey);
    out.clear();
    for (size_t i = at; i < body.size() && out.size() < 32; ++i) {
        const char c = body[i];
        if (c == '"')
            return !out.empty();
        if (c == '\\' || static_cast<unsigned char>(c) < 0x20)
            return false;
        out.push_back(c);
    }
    return false;
}

DWORD WINAPI CheckThread(LPVOID param)
{
    const unsigned generation = static_cast<unsigned>(reinterpret_cast<UINT_PTR>(param));
    std::string latest;
    const std::string body = Fetch();
    const bool parsed = !body.empty() && ParseVersion(body, latest);

    if (g_generation.load() == generation) {
        if (!parsed) {
            Log_Printf("UpdateCheck: couldn't read a version from Nexus");
            g_state = static_cast<int>(UpdateState::Failed);
        } else {
            {
                std::lock_guard<std::mutex> lock(g_latestMutex);
                g_latest = latest;
            }
            const bool newer = UpdateCheck_CompareVersions(latest.c_str(), RE5VR_VERSION) > 0;
            Log_Printf("UpdateCheck: Nexus has %s, this is %s - %s", latest.c_str(), RE5VR_VERSION,
                newer ? "update available" : "up to date");
            g_state = static_cast<int>(newer ? UpdateState::Available : UpdateState::UpToDate);
        }
    }
    g_running = false;
    return 0;
}

DWORD WINAPI OpenPageThread(LPVOID)
{
    const HRESULT co = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
    ShellExecuteA(nullptr, "open", kNexusUrl, nullptr, nullptr, SW_SHOWNORMAL);
    if (SUCCEEDED(co))
        CoUninitialize();
    return 0;
}

// Splits "v0.4.0a" into {0,4,0} and "a".
void SplitVersion(const char* s, unsigned parts[4], std::string& suffix)
{
    parts[0] = parts[1] = parts[2] = parts[3] = 0;
    suffix.clear();
    while (*s == ' ' || *s == 'v' || *s == 'V')
        ++s;
    int n = 0;
    while (n < 4 && std::isdigit(static_cast<unsigned char>(*s))) {
        unsigned v = 0;
        while (std::isdigit(static_cast<unsigned char>(*s)))
            v = v * 10 + static_cast<unsigned>(*s++ - '0');
        parts[n++] = v;
        if (*s != '.' || !std::isdigit(static_cast<unsigned char>(s[1])))
            break;
        ++s;
    }
    for (; *s; ++s) {
        const unsigned char c = static_cast<unsigned char>(*s);
        if (!std::isspace(c) && c != '-' && c != '_')
            suffix.push_back(static_cast<char>(std::tolower(c)));
    }
}

} // namespace

void UpdateCheck_Start()
{
    bool expected = false;
    if (!g_running.compare_exchange_strong(expected, true))
        return;
    g_state = static_cast<int>(UpdateState::Checking);
    const unsigned generation = ++g_generation;
    HANDLE thread = CreateThread(nullptr, 0, &CheckThread, reinterpret_cast<LPVOID>(static_cast<UINT_PTR>(generation)), 0, nullptr);
    if (!thread) {
        g_state = static_cast<int>(UpdateState::Failed);
        g_running = false;
        return;
    }
    CloseHandle(thread);
}

void UpdateCheck_SetOff()
{
    ++g_generation;
    g_state = static_cast<int>(UpdateState::Off);
}

UpdateStatus UpdateCheck_GetStatus()
{
    UpdateStatus s;
    s.state = static_cast<UpdateState>(g_state.load());
    std::lock_guard<std::mutex> lock(g_latestMutex);
    s.latest = g_latest;
    return s;
}

const char* UpdateCheck_NexusUrl()
{
    return kNexusUrl;
}

void UpdateCheck_OpenNexusPage()
{
    // ShellExecute can take a moment to start the browser; keep it off the
    // render thread.
    HANDLE thread = CreateThread(nullptr, 0, &OpenPageThread, nullptr, 0, nullptr);
    if (thread)
        CloseHandle(thread);
}

int UpdateCheck_CompareVersions(const char* a, const char* b)
{
    unsigned pa[4], pb[4];
    std::string sa, sb;
    SplitVersion(a, pa, sa);
    SplitVersion(b, pb, sb);
    for (int i = 0; i < 4; ++i) {
        if (pa[i] != pb[i])
            return pa[i] < pb[i] ? -1 : 1;
    }
    // "0.4.0a" is newer than "0.4.0".
    if (sa == sb)
        return 0;
    return sa < sb ? -1 : 1;
}
