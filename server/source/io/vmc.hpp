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

#ifndef VMC_H
#define VMC_H

#include "pipeline/record.hpp"

#include "util/memory.hpp" // opaque_ptr
#include "util/eigendef.hpp" // opaque_ptr

#define vmc_DEFAULT_PERFORMER_PORT_NO 39540
#define vmc_DEFAULT_MARIONETTE_PORT_NO 39539

struct vmc_output;

enum class VMCRole { Camera, HMD, Controller, Tracker, MAX };

struct vmc_device
{
    VMCRole role;
    std::string serial;
    Eigen::Isometry3f pose;
    float fov;
};

opaque_ptr<vmc_output> vmc_init_output(const std::string &host, const std::string &port);

bool vmc_is_connected(opaque_ptr<vmc_output> &vmc);

bool vmc_is_opened(opaque_ptr<vmc_output> &vmc);

bool vmc_try_open(opaque_ptr<vmc_output> &vmc);

void vmc_send_device_packets(opaque_ptr<vmc_output> &vmc, const std::vector<vmc_device> &trackers, TimePoint_t timestamp, float deltaS);

#endif /* VMC_H */
