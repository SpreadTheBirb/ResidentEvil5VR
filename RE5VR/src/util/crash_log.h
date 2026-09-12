#pragma once

// Crash reporter (2026-09-12). A tester's game vanished the instant he
// pressed F7 and the log simply stopped - the last line before the drop was
// the step right before xrCreateSession, which told us the area but not the
// culprit. This logs the faulting address as module+offset, so the next
// report says whether it died in the OpenXR runtime, dgVoodoo, or us.
//
// Inert until something crashes; safe to ship in a release build.
void CrashLog_Install();
