# RPL-GUI-V.1


GUI prototype for the RPL line.

Allows the user to create tradition joint/cartesian paths and test/create path plans for the motions.
Additional feature is the 'custom' option, allowing the user to use external scripts (force/optimisation for example) to directly control the robot within a behaviour tree setup.

Topic to monitor describes the topics that must be alive during the process. For example, a visual quality inspection needs the specific camera topic to be broadcasting for the motion to be useful.

Scans for image topics to have a live-display of up to 3 camera topics on the dashboard (thsi can be increased or reduced based on need).

A seperate window for KPIs - up to the engineer and up to the process.

Also contains an embedding of RQT to monitor the network in real time.

[output.webm](https://github.com/NMIS-Automation/RPL-GUI-V.1/assets/52172608/8eb611a9-d0aa-4695-a303-2d1ebe04516a)

TODO:
- flexible all in one launch file to set up monitored topics in a pre-set configuration.
- config describing the cell layout, and robot joint/parameter server/name loaded at the start
- saving complex path plans for passing onto the hardware interface
- integration with custom UR drivers to enable hand-guided path planning,
- fleshing out visual marker capability and compatability.
