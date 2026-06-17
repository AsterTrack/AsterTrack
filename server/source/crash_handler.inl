/**
 * Crash Handler for AsterTrack
 * Author: Seneral
 * License: MIT
 *
 * Combination of:
 *
 * CrashCatch - A simple cross-platform crash handler
 * Version 1.4.0
 * Author: Keith Pottratz
 * Email: keithpotz@gmail.com
 * License: MIT
 * https://github.com/keithpotz/CrashCatch
 *
 * handlecrash.h
 * Author: Angelo Geels
 * License: MIT
 * https://github.com/Crackshell/handlecrash.h
 */

#include <string>
#include <chrono>
#include <functional>
#include <cstdlib>
#include <ctime>
#include <cinttypes>

#if defined(_WIN32)
#define CRASH_PLATFORM_WINDOWS
#include <windows.h>
#include <dbgHelp.h>
#pragma comment(lib, "dbgHelp.lib") // Auto-link debugging support library
#elif defined(__linux__)
#define CRASH_PLATFORM_LINUX
#include <signal.h>
#include <execinfo.h>
#include <unistd.h>
#include <cxxabi.h>
#include <cstring>
#include <sys/wait.h>
#include <sys/stat.h>
#include <ucontext.h>

#ifndef CRASH_ALTSTACK_SIZE
#define CRASH_ALTSTACK_SIZE MINSIGSTKSZ + 3*1000
#endif
static std::vector<uint8_t> CrashAltStack;

#endif

#ifndef CRASH_MAX_STACK_FRAMES
#define CRASH_MAX_STACK_FRAMES 64
#endif

#ifndef CRASH_MAX_TS_SIZE
#define CRASH_MAX_TS_SIZE 20
#endif

#ifndef CRASH_MAX_PATH_SIZE
#define CRASH_MAX_PATH_SIZE 1000
#endif

// Context data passed to crash callback (onCrash, onCrashUpload)
struct CrashContext
{
	std::string dumpFilePath = "";  // .dmp (Windows) or blank (Linux)
	std::string logFilePath = "";   // .txt summary log
	std::string timestamp = "";     // Crash timestamp
	int signalOrCode = 0;           // Signal or exception code
};

// Configuration structure for CrashCatch behavior
struct CrashConfig
{
	std::string dumpFolder = "./crash_reports/";	// Where to save crash files
	std::string dumpFileName = "crash";				// Base name (timestamp added optionally)
	bool enableTextLog = true;						// Output .txt human-readable crash report
	bool autoTimestamp = true;						// Auto-append timestamp to filenames
	bool showCrashDialog = false;					// (Windows only) Show MessageBox on crash
	std::function<bool(const CrashContext&)> onCrash = nullptr;			// Called on crash (log before exit)
	std::function<void(const CrashContext&)> onCrashUpload = nullptr;	// Optional hook to upload crash report
	std::string version = "unknown";				// Application version string
	std::string build =
#ifdef _DEBUG
		"Debug";
#else
		"Release";
#endif
	std::string descriptor = "";				// Optional notes in crash log
	bool includeStackTrace = true;					// Output stack trace in .txt log (Windows + Linux)
	bool includeRegMemMap = false;					// Output registers and memory map in .txt log (Linux)
	bool heapSafeFork = true;						// Fork process on linux before writing crash log to be extra sure heap is not further corrupted
};

static CrashConfig crashConfig; // Global configuration

#ifdef CRASH_PLATFORM_WINDOWS
// Set by UnhandledExceptionHandler so writeCrashLogWindows can walk the crash-site stack,
// not the handler's own stack. Only valid during crash handling.
inline CONTEXT* crashSiteContext = nullptr;
#endif

constexpr const char* GetArch()
{
	#if defined(__x86_64__) || defined(_M_X64)
	return "x86_64";
	#elif defined(i386) || defined(__i386__) || defined(__i386) || defined(_M_IX86)
	return "x86_32";
	#elif defined(__riscv32)
	return "RiscV32";
	#elif defined(__riscv__) || defined(__riscv)
	return "RiscV64";
	#elif defined(__aarch64__) || defined(_M_ARM64)
	return "ARM64";
	#else
	return "UNKNOWN";
	#endif
}

// Generate timestamp string (YYYY-MM-DD_HH-MM-SS)
inline void getTimestamp(char timestamp[CRASH_MAX_TS_SIZE])
{
	auto now = std::chrono::system_clock::now();
	auto time = std::chrono::system_clock::to_time_t(now);
	std::tm tm_result = {};
#ifdef CRASH_PLATFORM_WINDOWS
	localtime_s(&tm_result, &time);  // thread-safe on Windows
#else
	localtime_r(&time, &tm_result);  // thread-safe on POSIX
#endif
	std::strftime(timestamp, CRASH_MAX_TS_SIZE, "%Y-%m-%d_%H-%M-%S", &tm_result);
}

#define _print(fmt, ...) printf(fmt, ##__VA_ARGS__); fprintf(fpCrash, fmt, ##__VA_ARGS__);
#define _print_file(fmt, ...) fprintf(fpCrash, fmt, ##__VA_ARGS__);

// Write human-readable crash report to .txt file
inline void writeCrashLogHeader(FILE* fpCrash, const char *timestamp)
{
	_print("***\n");
	_print("*** Crash Report\n");
	_print("***\n");

	_print("*** Timestamp: %s\n", timestamp);
#if defined(CRASH_PLATFORM_WINDOWS)
	_print("*** Platform: Windows\n");
#elif defined(CRASH_PLATFORM_LINUX)
	_print("*** Platform: Linux\n");
#endif
	_print("*** Architecture: %s\n", GetArch());
	_print("***\n");

	_print("*** Program:\n");
	_print("*** Version: %s\n", crashConfig.version.c_str());
	_print("*** Build: %s\n", crashConfig.build.c_str());
	if (!crashConfig.descriptor.empty())
		_print("*** Descriptor: %s\n", crashConfig.descriptor.c_str());
	_print("***\n");
}

#ifdef CRASH_PLATFORM_WINDOWS

// Write human-readable crash report to .txt file
inline void writeCrashLogWindows(const char *logPath, const char *timestamp)
{
	FILE* fpCrash = fopen(logPath, "w");
	if (!fpCrash) return;

	writeCrashLogHeader(fpCrash, timestamp);

	if (crashConfig.includeStackTrace)
	{
		// Walk the stack using DbgHelp (already linked via pragma comment)
		// SymInitialize was called at CrashCatch::initialize() time
		HANDLE process = GetCurrentProcess();
		HANDLE thread  = GetCurrentThread();

		// Use crash-site context if available (set by exception handler),
		// otherwise fall back to capturing here (e.g. called standalone).
		CONTEXT localContext = {};
		CONTEXT& context = crashSiteContext ? *crashSiteContext : localContext;
		if (!crashSiteContext)
		{
			localContext.ContextFlags = CONTEXT_FULL;
			RtlCaptureContext(&localContext);
		}

		STACKFRAME64 frame = {};
#if defined(_M_X64)
		DWORD machineType       = IMAGE_FILE_MACHINE_AMD64;
		frame.AddrPC.Offset     = context.Rip;
		frame.AddrPC.Mode       = AddrModeFlat;
		frame.AddrFrame.Offset  = context.Rbp;
		frame.AddrFrame.Mode    = AddrModeFlat;
		frame.AddrStack.Offset  = context.Rsp;
		frame.AddrStack.Mode    = AddrModeFlat;
#elif defined(_M_IX86)
		DWORD machineType       = IMAGE_FILE_MACHINE_I386;
		frame.AddrPC.Offset     = context.Eip;
		frame.AddrPC.Mode       = AddrModeFlat;
		frame.AddrFrame.Offset  = context.Ebp;
		frame.AddrFrame.Mode    = AddrModeFlat;
		frame.AddrStack.Offset  = context.Esp;
		frame.AddrStack.Mode    = AddrModeFlat;
#else
		DWORD machineType = IMAGE_FILE_MACHINE_AMD64;
#endif
		// Symbol buffer
		const int MAX_SYM_NAME_LEN = 256;
		char symBuffer[sizeof(SYMBOL_INFO) + MAX_SYM_NAME_LEN * sizeof(char)];
		SYMBOL_INFO* symbol = reinterpret_cast<SYMBOL_INFO*>(symBuffer);
		symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
		symbol->MaxNameLen   = MAX_SYM_NAME_LEN;

		_print("*** Stack Trace:\n");
		int frameIndex = 0;

		while (StackWalk64(machineType, process, thread, &frame, &context,
							nullptr, SymFunctionTableAccess64, SymGetModuleBase64, nullptr))
		{
			if (frame.AddrPC.Offset == 0) break;
			_print("[%d]:\t0x%" PRIXPTR "\t", frameIndex++, (uintptr_t)frame.AddrPC.Offset);

			DWORD64 displacement = 0;
			if (SymFromAddr(process, frame.AddrPC.Offset, &displacement, symbol))
			{
				_print("%s", symbol->Name);

				// Try to get file/line info
				IMAGEHLP_LINE64 line = {};
				line.SizeOfStruct    = sizeof(IMAGEHLP_LINE64);
				DWORD lineDisp       = 0;
				if (SymGetLineFromAddr64(process, frame.AddrPC.Offset, &lineDisp, &line))
				{
					_print(" (%s: %lu)", line.FileName, line.LineNumber);
				}
			}
			_print("\n");

			// Avoid runaway stack walks
			if (frameIndex > CRASH_MAX_STACK_FRAMES) break;
		}
	}

	fclose(fpCrash);
}

// Windows unhandled exception handler
LONG WINAPI UnhandledExceptionHandler(EXCEPTION_POINTERS* ep)
{
	DWORD code = ep->ExceptionRecord->ExceptionCode;
	if (code == DBG_PRINTEXCEPTION_C || code == DBG_CONTROL_C)
	{
		return EXCEPTION_CONTINUE_SEARCH;
	}

	char timestamp[CRASH_MAX_TS_SIZE];
	getTimestamp(timestamp);

	char logPath[CRASH_MAX_PATH_SIZE], dmpPath[CRASH_MAX_PATH_SIZE], dialogMsg[CRASH_MAX_PATH_SIZE];
	snprintf(logPath, sizeof(logPath), "%s%s_%s.txt", crashConfig.dumpFolder.c_str(), crashConfig.dumpFileName.c_str(), timestamp);
	snprintf(dmpPath, sizeof(dmpPath), "%s%s_%s.dmp", crashConfig.dumpFolder.c_str(), crashConfig.dumpFileName.c_str(), timestamp);

	CreateDirectory(crashConfig.dumpFolder.c_str(), NULL);

	// Point the stack walker at the actual crash site, not the handler frame
	crashSiteContext = ep->ContextRecord;

	HANDLE hFile = CreateFileA(dmpPath, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (hFile != INVALID_HANDLE_VALUE)
	{
		MINIDUMP_EXCEPTION_INFORMATION dumpInfo = { GetCurrentThreadId(), ep, FALSE };
		MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), hFile, MiniDumpWithDataSegs, &dumpInfo, nullptr, nullptr);
		CloseHandle(hFile);

		if (crashConfig.enableTextLog)
		{
			writeCrashLogWindows(logPath, timestamp);
		}

		if (crashConfig.showCrashDialog)
		{
			snprintf(dialogMsg, sizeof(dialogMsg), "Crash occurred. Dump written to: %s\n", dmpPath);
			MessageBoxA(nullptr, dialogMsg, "Crash Detected", MB_OK | MB_ICONERROR);
		}
	}

	// Build context once, after files are written, so callbacks can access them
	CrashContext context{ dmpPath, logPath, timestamp, static_cast<int>(code) };

	if (crashConfig.onCrash)
	{
		bool recovered = crashConfig.onCrash(context);
		if (recovered)
			return EXCEPTION_EXECUTE_HANDLER;

	}

	if (crashConfig.onCrashUpload)
	{
		crashConfig.onCrashUpload(context);
	}

	return EXCEPTION_EXECUTE_HANDLER;
}
#endif

#ifdef CRASH_PLATFORM_LINUX

static const char* GetSignal(int signum)
{
	switch(signum)
	{
		case SIGILL:
			return "SIGILL";
		case SIGBUS:
			return "SIGBUS";
		case SIGFPE:
			return "SIGFPE";
		case SIGSEGV:
			return "SIGSEGV";
		case SIGPIPE:
			return "SIGPIPE";
		case SIGABRT:
			return "SIGABRT";
		default:
			return "UNKNOWN";
	}
}

static inline void writeCrashLogLinux(const char* logPath, const char *timestamp, int signum, siginfo_t* siginfo, void* context)
{
	FILE* fpCrash = fopen(logPath, "w");
	if (!fpCrash)
	{
		if (errno == EACCES)
			printf("Failed to write crash log! Permission Denied!\n");
		else
			printf("Failed to write crash log! Errno %d\n", errno);
		return;
	}

	writeCrashLogHeader(fpCrash, timestamp);

	_print("*** Signal: %s (num %d, errno %d, code %d)\n", GetSignal(signum), signum, siginfo->si_errno, siginfo->si_code);
	_print("*** Fault address: 0x%" PRIXPTR "\n", (uintptr_t)siginfo->si_addr);
	_print("***\n");

	if (!crashConfig.includeStackTrace)
	{
		fclose(fpCrash);
		return;
	}

	void* stackBuffer[CRASH_MAX_STACK_FRAMES];
	int size = backtrace(stackBuffer, CRASH_MAX_STACK_FRAMES);
	char** symbols = backtrace_symbols(stackBuffer, size);
	int skip = 2; // writeCrashLogLinux + linuxSignalHandler
	_print("*** Stack Trace (%d frames):\n", size-skip);
	for (int i = skip; i < size; i++)
	{
		_print("[%d]:\t0x%" PRIXPTR "\t", i-skip, (uintptr_t)stackBuffer[i]);
		// Extract mangled name: between '(' and '+'
		char *symbol = symbols[i];
		char *parenOpen = strchr(symbol, '(');
		char *plusSign = parenOpen? strchr(parenOpen, '+') : NULL;
		if (parenOpen && plusSign && plusSign > parenOpen + 1)
		{
			// Apparently this doesn't even guarantee no dynamic allocation, cool
			char demangleBuffer[100];
			size_t size = sizeof(demangleBuffer);
			int status  = 0;
			*plusSign = '\0'; // Delimit middle segment, to use it as mangled
			char* demangled = abi::__cxa_demangle(parenOpen+1, demangleBuffer, &size, &status);
			*plusSign = '+'; // Restore trailing segment, middle segment is not used anymore
			if (status == 0 && demangled)
			{
				*(parenOpen+1) = '\0'; // Delimit front segment
				_print("%s%s%s\n", symbol, demangled, plusSign);
				continue;
			}
		}
		// Fall back to mangled symbol
		_print("%s\n", symbol);
	}
	_print("***\n");

	if (!crashConfig.includeRegMemMap)
	{
		fclose(fpCrash);
		return;
	}

	_print("*** Registers:\n");

	ucontext_t* ctx = (ucontext_t*)context;
	#define RV(reg) ctx->uc_mcontext.gregs[reg]
	#define FPSTe(i) ctx->uc_mcontext.fpregs->_st[i].exponent
	#define FPSTsa(i) ctx->uc_mcontext.fpregs->_st[i].significand[3] << 16 | ctx->uc_mcontext.fpregs->_st[i].significand[2]
	#define FPSTsb(i) ctx->uc_mcontext.fpregs->_st[i].significand[1] << 16 | ctx->uc_mcontext.fpregs->_st[i].significand[0]

#ifdef __LP64__
	#if (!defined(__clang__) && __GNUC__ < 5) || (defined(__clang__) && (__clang_major__ <= 3 && __clang_minor__ < 8))
	#define HC_REGFMT16 "%016lx"
	#define HC_REGFMT8 "%08lx"
	#define HC_REGFMT4 "%04lx"
	#else
	#define HC_REGFMT16 "%016llx"
	#define HC_REGFMT8 "%08llx"
	#define HC_REGFMT4 "%04llx"
	#endif
	_print("***   RAX: " HC_REGFMT16 "     RBX: " HC_REGFMT16 "     RCX: " HC_REGFMT16 "\n", RV(REG_RAX), RV(REG_RBX), RV(REG_RCX));
	_print("***   RDX: " HC_REGFMT16 "     RSI: " HC_REGFMT16 "     RDI: " HC_REGFMT16 "\n", RV(REG_RDX), RV(REG_RSI), RV(REG_RDI));
	_print("***   RBP: " HC_REGFMT16 "      R8: " HC_REGFMT16 "      R9: " HC_REGFMT16 "\n", RV(REG_RBP), RV( REG_R8), RV( REG_R9));
	_print("***   R10: " HC_REGFMT16 "     R11: " HC_REGFMT16 "     R12: " HC_REGFMT16 "\n", RV(REG_R10), RV(REG_R11), RV(REG_R12));
	_print("***   R13: " HC_REGFMT16 "     R14: " HC_REGFMT16 "     R15: " HC_REGFMT16 "\n", RV(REG_R13), RV(REG_R14), RV(REG_R15));
	_print("***   RSP: " HC_REGFMT16 "\n", RV(REG_RSP));
	_print("***\n");
	_print("***   RIP: " HC_REGFMT16 "     EFL: " HC_REGFMT8 "\n", RV(REG_RIP), RV(REG_EFL));
	_print("***\n");
	_print("***    CS: " HC_REGFMT4 "    FS: " HC_REGFMT4 "      GS: " HC_REGFMT4 "\n", RV(REG_CSGSFS) & 0xFFFF, (RV(REG_CSGSFS) >> 16) & 0xFFFF, (RV(REG_CSGSFS) >> 32) & 0xFFFF);
	_print("***\n");
	for (int i = 0; i < 8; i += 2)
	{
		_print("***   ST(%d) %04hx %08x%08x      ST(%d) %04hx %08x%08x\n", i, FPSTe(i), FPSTsa(i), FPSTsb(i), i + 1, FPSTe(i + 1), FPSTsa(i + 1), FPSTsb(i + 1));
	}
	_print("***\n");
	#define FPXMMa(i) ctx->uc_mcontext.fpregs->_xmm[i].element[1] << 8 | ctx->uc_mcontext.fpregs->_xmm[i].element[0]
	#define FPXMMb(i) ctx->uc_mcontext.fpregs->_xmm[i].element[3] << 24 | ctx->uc_mcontext.fpregs->_xmm[i].element[2] << 16
	for (int i = 0; i < 16; i += 2)
	{
		_print("***   XMM%d%s: %016x%016x     XMM%d%s: %016x%016x\n", i, i < 10 ? " " : "", FPXMMa(i), FPXMMb(i), i + 1, i < 10 ? " " : "", FPXMMa(i + 1), FPXMMb(i + 1));
	}
#else
	_print("***   EAX: %08x   EBX: %08x   ECX: %08x   EDX: %08x\n", RV(REG_EAX), RV(REG_EBX), RV(REG_ECX), RV(REG_EDX));
	_print("***   ESI: %08x   EDI: %08x   EBP: %08x   ESP: %08x\n", RV(REG_ESI), RV(REG_EDI), RV(REG_EBP), RV(REG_ESP));
	_print("***\n");
	_print("***   EIP: %08x   EFL: %08x\n", RV(REG_EIP), RV(REG_EFL));
	_print("***\n");
	_print("***   CS: %04x   DS: %04x   ES: %04x   FS: %04x   GS: %04x   SS: %04x\n", RV(REG_CS), RV(REG_DS), RV(REG_ES), RV(REG_FS), RV(REG_GS), RV(REG_SS));
	_print("***\n");
	for (int i = 0; i < 8; i += 2) {
		_print("***   ST(%d) %04hx %08x%08x      ST(%d) %04hx %08x%08x\n", i, FPSTe(i), FPSTsa(i), FPSTsb(i), i + 1, FPSTe(i + 1), FPSTsa(i + 1), FPSTsb(i + 1));
	}
#endif

	_print("***\n");

	FILE* fpMaps = fopen("/proc/self/maps", "r");
	if (fpMaps != 0)
	{
		_print_file("*** Memory map:\n");

		// Mirror to stdout and log without using heap
		uint8_t buffer[1000];
		while (true)
		{
			size_t read = fread(buffer, 1, sizeof(buffer), fpMaps);
			if (read == 0) break;
			//fwrite(buffer, 1, read, stdout);
			fwrite(buffer, 1, read, fpCrash);
		}

		fclose(fpMaps);
	}

	fclose(fpCrash);
}

void linuxSignalHandler(int signum, siginfo_t* siginfo, void* context)
{
	char timestamp[CRASH_MAX_TS_SIZE];
	getTimestamp(timestamp);

	char logPath[CRASH_MAX_PATH_SIZE];
	snprintf(logPath, sizeof(logPath), "%s%s_%s.txt", crashConfig.dumpFolder.c_str(), crashConfig.dumpFileName.c_str(), timestamp);

	pid_t pid = crashConfig.heapSafeFork? fork() : 0;
	if (pid == 0)
	{
		if (mkdir(crashConfig.dumpFolder.c_str(), 0755) < 0 && errno != EEXIST)
			printf("Failed to mkdir %s, errno %d\n", crashConfig.dumpFolder.c_str(), errno);

		writeCrashLogLinux(logPath, timestamp, signum, siginfo, context);

		// Build context once, after file is written, so callbacks can access it
		CrashContext context{ "", logPath, timestamp, signum };

		if (crashConfig.onCrash)
		{
			bool recovered = crashConfig.onCrash(context);
			if (recovered)
			{
				if (crashConfig.heapSafeFork)
					_exit(0);
				else // Don't exit
					return;
			}
		}

		if (crashConfig.onCrashUpload)
		{
			crashConfig.onCrashUpload(context);
		}

		_exit(0);
	}
	else if (pid > 0)
	{
		// Parent: wait for child to finish writing the report, then exit
		waitpid(pid, nullptr, 0);
	}

	// Re-raise signal for default handling (e.g. core dump): 
	struct sigaction sa;
	sa.sa_handler = SIG_DFL;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;
	sigaction(signum, &sa, 0);
	raise(signum);
}

#endif

#undef _print
#undef _print_file

// Initialize CrashCatch with user configuration
static inline bool EnableCrashHandler(const CrashConfig &config = CrashConfig())
{
	crashConfig = config;
#ifdef CRASH_PLATFORM_WINDOWS
	// Load symbols now so they're ready when a crash occurs
	SymInitialize(GetCurrentProcess(), nullptr, TRUE);
	SetUnhandledExceptionFilter(UnhandledExceptionHandler);
#elif defined(CRASH_PLATFORM_LINUX)
	stack_t ss;
	CrashAltStack.resize(CRASH_ALTSTACK_SIZE);
	ss.ss_sp = CrashAltStack.data();
	ss.ss_size = CRASH_ALTSTACK_SIZE;
	ss.ss_flags = 0;
	if (sigaltstack(&ss, 0) != 0)
		printf("Failed to install sigaltstack!");

	struct sigaction action;
	action.sa_sigaction = linuxSignalHandler;
	sigemptyset(&action.sa_mask);
	action.sa_flags = SA_SIGINFO | SA_ONSTACK;

	sigaction(SIGILL, &action, 0);
	sigaction(SIGBUS, &action, 0);
	sigaction(SIGFPE, &action, 0);
	sigaction(SIGSEGV, &action, 0);
	sigaction(SIGPIPE, &action, 0);
	sigaction(SIGABRT, &action, 0);
#endif
	return true;
}

// Auto-initialize when included (optional)
#ifdef CRASHCATCH_AUTO_INIT
namespace
{
	const bool _autoInit = EnableCrashHandler();
}
#endif

