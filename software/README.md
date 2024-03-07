## HexLev Control Software

The HexLev Control Center is a protoype software developed in Unity game engine to aid the control of the levitator. 

The simplified UI acts an interface between the user and the levitator, abstracting Serial and SPI comms involving the Arduino and FPGAs.

The software allows for the creation, deletion and movement of mutliple particles along user-defined trajectories in 3D.

#### Operation

The current version of the software is a prototype and uses a crude implementation for a movement algortihm (see documentation for more details)

The user is at liberty to build the program and run it as a standalone application, however it may be useful to use the program within the Unity editor as this provides users with further technical insight. Pressing the 'Play' button within the Unity editor runs the program.

The UI allows for common user input such as mouse clicks, dragging and scrolling. Devices llke joysticks and controllers may be used as Unity provides basic functionality, though it is not recommended in this case.

#### Notes

The current prototype version snaps particles to positons, employing a rigid 0-100% scheme. The code will be updated with a solver for the final product.

The Arduino serial code contains test structures for serial debugging purposes, so a small levitator can be attached and controlled.
