# AsterTrack

AsterTrack is a custom multi-camera system designed to track markers in 3D space for a variety of purposes like virtual reality and motion capture. This so-called optical tracking is commonly used in professional setups, but typically costs several thousands of euros. AsterTrack aims to implement the same concept with much less expensive hardware, and pioneer a user-friendly multi-camera experience.
It aims to be as accurate as the best consumer VR tracking systems with low latency, and very affordable. The trackers can be passive (that is, no battery) and are thus very cheap and lightweight. Users can also create and calibrate trackers themselves from 3D prints and even from everyday objects. 

## Current State

There are several components affecting the current usability of AsterTrack in real-world use: <br>
**Camera Calibration:** Fully functional and easy to use - may need improvements for multi-room setups. <br>
**Target Calibration:** New Targets can be calibrated just fine, but it is a slow, compute-intensive process that requires some manual involvement still. However, this is only needed once for each new target. <br>
**Target Tracking:** Target tracking works fine and is very much useable, though not yet at peak stability or performance. While multiple targets can track, there are many improvements still to be done here. <br>
**Integrations:** A VRPN integration exists to send tracker data to other custom applications, but no official SteamVR/Monado driver for VR exists yet, nor any file formats used for mocap.

## Software

You can build and try the AsterTrack Application right now without any hardware. <br>
You can load sample recordings to try the software, test camera- and target calibration as well as tracking, and perhaps even design your own target geometry in Simulation Mode.
There are recordings with some image frame data included which you can use to emulate blob detection and develop it further.

## Hardware

You need at least 3 AsterTrack Cameras and one AsterTrack Controller for a minimal setup. You can add more cameras and even controllers at any point, with each controller supporting up to 8 cameras. <br>
Additionally, you will need a USB PD power source to another power supply that can output between 12V to 24V. 10W should be sufficient for a 3 camera setup, each camera needs about 3W, but please calculate with 5W to be safe. <br>
Finally, you'll need USB Data and Power cable for each controller, and a CAT 5(e) cable with straight wiring for each camera. These CAT 5 cables are best sourced on your own, they can be found for cheap in a variety of lengths, colors and shapes that you can choose to fit your room. Cheap ones cost about 2€, more expensive ones might cost up to 10€ with smaller, easier to hide wiring (whether round or even flat).

### A note on DIY

AsterTrack has been designed around the custom hardware from the very beginning and it is essential for a performant and user-friendly multi-camera experience. <br>Most importantly, the images are processed onboard the AsterTrack Cameras on a Raspberry Pi Zero. The tracking-relevant data is then sent over a custom protocol through a CAT 5 cable to a AsterTrack Controller, which is a custom board based on the CH32V307 chip. It synchronises and communicates with up to 8 AsterTrack Cameras, and connects to the host PC through an USB 2.0 HS connection. <br>
While it is theoretically possible to build the hardware DIY with just a CH32V307 EVT board, a Raspberry Pi, an OV9281 camera module from innomaker and some wires, it is insufficient for real-world use. For reliable communication over the CAT 5 cable, the UART communication is wrapped in RS-422 differential signaling, and for stable operation, the power supply voltage through the cable is 12-24V - passing 5V straight through is not sufficient as the Raspberry Pi will suffer from undervoltage crashes. <br>
In addition to addressing these fundamental problems, the custom hardware also adds support for passive retroreflective markers via a IR LED ring around the camera lens, various buttons and signaling LEDs, power regulation with external power and USB PD support, external sync I/O, and other smaller amenities. <br>
It is highly recommended to make use of the custom PCBs, either the official dev kit or the DIY version.

## License
Most of the software and firmware in this repository is licensed under the LGPL v3, but parts of the codebase are licensed under the MIT license or the MPL 2.0. This can be determined on a file-by-file basis by looking at the file header.

The following merely describes the intent behind this seggregation: <br>
**MIT:**
In general, smaller, self-contained components and utilities that might be useful to other projects are licensed under the MIT license. <br>
**MPL 2.0:**
Some bigger algorithmic components are licensed under the MPL 2.0 with the intent that they can be repurposed with minor efforts for other projects. These files are not explicitly designed to be repurposed and some references to LGPL-licensed code might have to be removed. However, the author (Seneral) deemed those components potentially valuable to further open source development in other fields. <br>
**LGPL v3**:
The majority of the source code that is specific to AsterTrack or an optical tracking system just like it is licensed under the LGPL v3, with file-by-file exceptions as previously noted. <br>

If you wish to use a source code file in part or in full that currently is licensed too restrictively for you, feel free to contact me and I will consider changing the license of at least the source code over which I still hold copyright.