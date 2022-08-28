
#pragma once

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN

#define NOWINRES
#define NOSERVICE
#define NOMCX
#define NOIME
#define NOCRYPT
#define NOMETAFILE
#define NOMINMAX
#define MMNOSOUND

#include <windows.h>

#undef PostMessage
#undef CreateEvent
#undef PlaySound
#undef SetCursor
#undef ProgressBox
#undef RegisterClass
#undef AddJob
#undef GetJob
#undef Yield
#undef SetPort
#undef CreateFont
#undef ShellExecute
#undef ShellExecuteEx
#undef GetCurrentTime
#undef GetTickCount
#undef GetCurrentDirectory
#undef GetObject
#undef LoadCursorFromFile
#undef GetCharABCWidths

#endif
