# body-on-track

This project produces a real-time simulation of a point mass interacting with a track defined by a parametric curve. The simulation time steps are carried out by use of the RK4 integration method to solve the appropriate differential equations describing the motion of the point mass depending on its current state. Collissions between the point mass and the track are handeled using a binary search roll-back procedure and the appropriate vector arithmatic. Forces allowed to act on the point mass are gravity, friction, air drag, normal force, and spring force.

## Usage

An easy way to run this project is to add all of the files to a Visual Studio project that has been given proper access to the SFML animation library and run the project in Visual Studio.

## Animation

Two .mp4 files are included to display runs of the real-time simulation with various initial conditions and force parameters.
