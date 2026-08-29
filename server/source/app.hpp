/**
AsterTrack Optical Tracking System
Copyright (C) 2026 Seneral <seneral@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, specifically version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef APP_H
#define APP_H

#include "util/log.hpp"
#include "util/blocked_vector.hpp"

#include <fstream>
#include <shared_mutex>

class AppState;
extern AppState AppInstance;
static inline AppState &GetApp() { return AppInstance; }

/**
 * Main AsterTrack application managing lifetime of server and interface
 */
class AppState
{
public:
	// Logging
	BlockedQueue<LogEntry, 16384> logEntries;
	std::string logPath;
	std::ofstream logFile;
	std::size_t lastFlushed;

	void FlushLog();
	void SignalQuitApp();
	void SignalInterfaceClosed();
};

#endif // APP_H