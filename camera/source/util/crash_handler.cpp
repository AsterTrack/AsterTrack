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

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cinttypes>
#include <signal.h>
#include <execinfo.h>
#include <unistd.h>
#include <cxxabi.h>
#include <cstring>
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <ucontext.h>
#include <vector>

static std::vector<char> ReportBuf;
static std::vector<uint8_t> CrashAltStack;

// External
void CrashHandler(int signal, const char *reportBuf, int reportLen);

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

void SignalHandler(int signum, siginfo_t *siginfo, void *context)
{
	int thread_id = syscall(SYS_gettid);
	char thread_name[16] = { 0 };
	prctl(PR_GET_NAME, thread_name);
	fprintf(stderr, "Thread %s crashed with signal %s (%d):", thread_name, GetSignal(signum), signum);
	int ReportLen = 0;
	ReportLen += snprintf(ReportBuf.data()+ReportLen, ReportBuf.capacity()-ReportLen,
		"Thread %s crashed with signal %s (%d):", thread_name, GetSignal(signum), signum);
	ReportLen++; // Include termination
	ReportLen += snprintf(ReportBuf.data()+ReportLen, ReportBuf.capacity()-ReportLen,
		"Fault Address: 0x%" PRIXPTR "", (uintptr_t)siginfo->si_addr);
	ReportLen++; // Include termination
	
	void* stackBuffer[32];
	int size = backtrace(stackBuffer, 32);
	char** symbols = backtrace_symbols(stackBuffer, size);
	int skip = 1; // linuxSignalHandler
	for (int i = skip; i < size; i++)
	{
		ReportLen += snprintf(ReportBuf.data()+ReportLen, ReportBuf.capacity()-ReportLen,
			"[%d]:\t0x%" PRIXPTR "\t", i-skip, (uintptr_t)stackBuffer[i]);
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
				ReportLen += snprintf(ReportBuf.data()+ReportLen, ReportBuf.capacity()-ReportLen,
					"%s%s%s", symbol, demangled, plusSign);
				ReportLen++; // Include termination
				continue;
			}
		}
		// Fall back to mangled symbol
		ReportLen += snprintf(ReportBuf.data()+ReportLen, ReportBuf.capacity()-ReportLen,
			"%s", symbol);
		ReportLen++; // Include termination
	}

	CrashHandler(signum, ReportBuf.data(), ReportLen);

	// Re-raise signal for default handling
	struct sigaction sa;
	sa.sa_handler = SIG_DFL;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;
	sigaction(signum, &sa, 0);
	raise(signum);
}

bool EnableCrashHandler()
{
	ReportBuf.resize(32*300);
	CrashAltStack.resize(MINSIGSTKSZ + 1000);
	stack_t ss;
	ss.ss_sp = CrashAltStack.data();
	ss.ss_size = CrashAltStack.size();
	ss.ss_flags = 0;
	if (sigaltstack(&ss, 0) != 0)
		printf("Failed to install sigaltstack!");

	struct sigaction action;
	action.sa_sigaction = SignalHandler;
	sigemptyset(&action.sa_mask);
	action.sa_flags = SA_SIGINFO | SA_ONSTACK;

	sigaction(SIGILL, &action, 0);
	sigaction(SIGBUS, &action, 0);
	sigaction(SIGFPE, &action, 0);
	sigaction(SIGSEGV, &action, 0);
	sigaction(SIGPIPE, &action, 0);
	sigaction(SIGABRT, &action, 0);
	return true;
}
