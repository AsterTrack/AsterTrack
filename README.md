# AsterTrack

<img alt="Banner image with AsterTrack hardware" src="https://docs.astertrack.dev/assets/Banner_AT_Rendered.png">

AsterTrack is a custom multi-camera system designed to track markers and targets in 3D space for a variety of purposes like virtual reality and motion capture.
This so-called optical tracking is commonly used in professional studios, but typically costs several thousands of euros for even the most basic setups. <br>
AsterTrack implements the same concept with much less expensive hardware, and pioneer a user-friendly multi-camera tracking experience. <br>
It aims to be as accurate as the best consumer VR tracking systems, with similarly low latency, while being very affordable for what it does. <br>
The trackers can be passive (that is, without battery) and are thus very cheap and lightweight.
Users can even create and calibrate trackers themselves from 3D prints or existing objects.

## Current State

AsterTrack is in a functional state, but usability varies depending on your exact use case. Here are the states of some of the major components:  <br>
**Camera Calibration:** Fully functional and easy to use - may need improvements for multi-room setups. <br>
**Target Calibration:** New Targets can be calibrated just fine, but it is a slow, compute-intensive process that requires some manual involvement still. However, this is only needed once for each new target. <br>
**Target Tracking:** Unassisted target tracking is complete and working almost at the peak of what's reasonably possible. IMU-assisted target tracking is very much still a WIP, with only raw gyroscope samples being automatically calibrated and used for tracking prediction (missing accelerometer and fused orientation measurements).  <br>
**Integrations:** A VRPN integration exists to send tracker data to other custom applications, but no official VR driver exists yet, nor any file formats used for mocap. A Monado driver is being worked on, and a SteamVR driver is planned.

## Documentation

For guides and instructions (many TBD), refer to the [documentation site](https://docs.astertrack.dev/). <br>
For compilation and software setup instructions, refer to each sub-projects README.

## License
Most of the software and firmware in this repository is licensed under the LGPL v3, but parts of the codebase are licensed under the MIT license or the MPL 2.0. This can be determined on a file-by-file basis by looking at the file header. <br>
Additionally, any modifications to the dependencies are licensed under the license of the respective dependency as found in the licenses folder, with express permission to incorporate these modifications upstream.

The following merely describes the intent behind this seggregation: <br>
**MIT:**
In general, smaller, self-contained components and utilities that might be useful to other projects are licensed under the MIT license. <br>
**MPL 2.0:**
Some bigger algorithmic components are licensed under the MPL 2.0 with the intent that they can be repurposed with minor efforts for other projects. These files are not explicitly designed to be repurposed and some references to LGPL-licensed code might have to be removed. However, the author (Seneral) deemed those components potentially valuable to further open source development in other fields. <br>
**LGPL v3**:
The majority of the source code that is specific to AsterTrack or an optical tracking system just like it is licensed under the LGPL v3, with file-by-file exceptions as previously noted. <br>

If you wish to use a source code file in part or in full that currently is licensed too restrictively for you, feel free to contact me and I will consider changing the license of at least the source code over which I still hold copyright.