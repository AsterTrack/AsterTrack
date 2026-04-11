# AsterTrack

<img alt="Banner image with AsterTrack hardware" src="https://docs.astertrack.dev/assets/Banner_AT_Rendered.png">

AsterTrack is a custom multi-camera system designed to track markers and targets in 3D space for a variety of purposes like virtual reality and motion capture.
This so-called marker-based optical tracking is common in professional studios, but typically costs several thousands of euros for even the most basic setup. <br>
AsterTrack implements the same concept with much less expensive hardware, and tries to pioneer a user-friendly multi-camera tracking experience.
It aims to be as accurate as the best consumer VR tracking systems, with similarly low latency, while being very affordable for what it does. <br>
Targets can be anything you can attach retroreflective markers to, even 3D prints or existing objects without a battery.
This allows them to be much lighter, cheaper, and more versatile than any other type of tracker. <br>
Notably, AsterTrack does not just rely on triangulation, but fully supports the use of flat marker targets, setting it apart even from most professional optical tracking systems.
This makes tracked objects much less intrusive as it does not require attaching big spherical markers, which would be impractical in consumer VR applications.

## Links
[Project Website & History](https://www.seneral.dev/astertrack.html) •  [Documentation](https://docs.astertrack.dev) <br>
[Example Clients Repository](https://github.com/AsterTrack/Clients) <br>
For compilation and software setup instructions, refer to each sub-projects README. <br>
<br>
Bridged Chat Server: [Discord](https://discord.gg/gKPQCW4bZ3) • [Fluxer](https://fluxer.gg/e1SAv1nZ) • [Stoat](https://stt.gg/xpNfGTQF) <br>
[Ticketing System](https://ticket.astertrack.dev) • contact via [DevKits](mailto:devkits@astertrack.dev) / [Support](mailto:support@astertrack.dev) emails

## How does it work?
There are three main components of an AsterTrack tracking system: <br>
<b>AsterTrack Cameras:</b> The tracking cameras that detect markers using onboard processing on the Raspberry Pi - this greatly lowers the burden on the host system for processing and data throughput, enabling a higher quality of tracking, and much more of it. <br>
<b>AsterTrack Controller:</b> The hub that synchronises the cameras and connects them to the host system via USB,
ensuring maximum stability for wired cameras and a good user experience. <br>
<b>AsterTrack Server:</b> The software that performs the tracking, turning the marker observations into usable 3D tracking data for use in external applications. <br>
<br>
Both active markers (IR LEDs) as well as passive markers (retroreflective tape) are supported, but the focus is very much on unintrusive flat passive markers that can be easily applied by the end-user. <br>
Unique sets of such markers can be calibrated and tracked as targets, enabling rotational tracking without an IMU by associating the markers between frames and multiple calibrated cameras.
Full support for IMUs is very much planned to aid in occlusion resistance in smaller setups, especially for VR use. <br>
If you're interested in more details, check out our documentation on the <a href="https://docs.astertrack.dev/building/hardware.html">hardware design</a> and <a href="https://docs.astertrack.dev/details/architecture.html">system architecture</a>.

## Current State
The base multi-camera experience of AsterTrack is solid and ready for use, though some usability improvements can still be made.
The rest depends heavily on the use case: <br>
As a tracking system originally designed for consumer VR use, the support of flat marker targets and 6-DOF tracking in general is well developed.
But to truly be usable for VR, the IMU integration needs to be completed, and common trackers designed for 3D printing.
IMU integration will also help track small trackers more reliably. <br>
For use cases relying on a triangulated point cloud of markers, this is still being developed - both the frame-to-frame labelling of markers as well as integrations like C3D export. <br>
The hardware is in the process of receiving its final major iteration, adding protections, more mounting points, and support for future wireless features.
Once it is ready, it will be available as a <a href="https://docs.astertrack.dev/building/hardware.html#hardware">dev kit</a>, and <a href="https://docs.astertrack.dev/building/hardware.html#diy-version">DIY versions</a> are planned as well.

## Roadmap
Out first priority is getting the dev kits ready for early adopters that already know what they need an optical tracking system for. <br>
In the leadup to this, we will work on integrations into third-party software used for MoCap and VR alike. <br>
But for true VR use, we will need to complete the IMU integration, both for existing VR hardware via monado, and custom IMU trackers. <br>
This is our next priority, and involves making retrofitting existing VR hardware as user-friendly as possible, and designing premade VR-ready designs for, among others, DIY HMDs, Controllers and FBT. <br>
Once the VR use case is fleshed out and ready for day-to-day use, we intend to move from small batches to a crowdfunding-supported consumer version.

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