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

#ifndef MEMORY_H
#define MEMORY_H

#include <memory> // shared_ptr

/**
 * Small additions to std::unique_ptr for opaque structs.
 * Requires specialisation in a compile unit that does know how to delete the struct, e.g.:
 * template<> void OpaqueDeleter<_Tp>::operator()(_Tp* ptr) const
 * { delete ptr; }
 */

template<typename _Tp>
struct OpaqueDeleter
{
	void operator()(_Tp* ptr) const;
};

template<typename _Tp>
using opaque_ptr = std::unique_ptr<_Tp, OpaqueDeleter<_Tp>>;

template<typename _Tp, typename... _Args>
constexpr inline opaque_ptr<_Tp>
make_opaque(_Args&&... __args)
{ return opaque_ptr<_Tp>(new _Tp(std::forward<_Args>(__args)...)); }

#endif // MEMORY_H