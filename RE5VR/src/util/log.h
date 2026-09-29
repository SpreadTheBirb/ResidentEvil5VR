#pragma once

// Minimal file logger. Writes to re5vr.log next to this DLL.
//
// The file work happens on its own thread (2026-09-18). A caller formats its
// line into a fixed slot and returns; nothing on a game thread ever waits on a
// disk. Before that, every line was a write and an fflush on whichever thread
// logged it - fine one at a time, and not fine when several timers came due on
// the same frame, which cost a tester a steady twenty frames once a second.
// Safe to call from any thread. Keeps one file handle open for the whole
// session and fflush()es after every write, so a crash still never loses
// buffered lines - but unlike the original per-call fopen/fclose design,
// it doesn't pay a kernel handle create/destroy on every single log line.
// That per-call open/close (serialized behind one mutex, across 3 threads,
// many times per frame) turned out to be the likely cause of >1s stalls
// seen right when logging volume spiked (2026-07-29, supercomputer log:
// a stall began the instant after a Log_Printf call, with ALL threads -
// game/copy/submit - silent for the whole gap, consistent with every
// thread blocking on the same mutex waiting on one slow file open).

void Log_Init();
void Log_Printf(const char* fmt, ...);

// Gets whatever is queued onto disk, for the crash handler. Does the writing
// itself rather than waiting on the writer thread, since it is called from a
// process that is already going down.
void Log_Flush();
