/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef APP_H
#define APP_H

#include "util/log.hpp"
#include "util/blocked_vector.hpp"

#include <fstream>

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
	struct LogEntry {
		std::string log;
		enum LogCategory category = LDefault;
		enum LogLevel level;
		int context = 0; // #Target, #Controller, #Camera, etc.
	};
	BlockedQueue<LogEntry, 1024*16> logEntries;
	std::ofstream logFile;
	std::size_t lastFlushed;
	std::mutex logIteratorAccess;

	void FlushLog();
	void SignalQuitApp();
	void SignalInterfaceClosed();
};

#endif // APP_H