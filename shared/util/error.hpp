/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef ERROR_H
#define ERROR_H

#include <string>
#include <optional>


/* Error Handling */

//typedef std::string ErrorMessage;

struct [[nodiscard]] ErrorMessage
{
	std::string msg;
	int code;

	inline constexpr ErrorMessage(std::string &&msg) noexcept : msg(std::move(msg)), code(-1) {}
	inline constexpr ErrorMessage(const char *msg) noexcept : msg(msg), code(-1) {}
	inline constexpr ErrorMessage(std::string &&msg, int code) noexcept : msg(std::move(msg)), code(code) {}
	inline constexpr ErrorMessage(const char *msg, int code) noexcept : msg(msg), code(code) {}

	inline constexpr const std::string& str() noexcept { return msg; }
	inline constexpr const char* c_str() noexcept { return msg.c_str(); }
};

#define HANDLE_ERROR [[nodiscard]] std::optional<ErrorMessage>

#endif // ERROR_H