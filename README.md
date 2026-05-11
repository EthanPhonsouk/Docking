# Autonomous Vehicle Docking System
**Status: In Progress** — Algorithm implemented and verified in simulation. Practical hardware testing not yet conducted

## Authors
Faris Stitou, George Hinds

Based on Morris, J.C. thesis on AUTOMATED SPACECRAFT DOCKING USING A VISION-BASED RELATIVE NAVIGATION SENSOR

## What it does
Information gathered by a PSD (or any device capable of gathering XYZ spatial cords) is passed through as a matrix which is then used to formulate a Direction Cosine Matrix and Measurement Sensitivity Matrix (Jacobian) which allows us to iteratively calculate the vector from the follower to the target.

## Installation
Requires MATLAB. Clone or download all .m and .csv files into the same directory.

## Usage
Simply go into MASTERLOOP.m and run in MATLAB. Sample test cases made by George can be used to calculate the target vector in a perfect scenario. If you want to try a different target position, go into get_points.m and change the filename to one of the other names of .csv files.

## How It Works
Using a good enough initial guess of the targets position and orientation, the measured vectors of all the beacons are used to find the DCM matrix. The DCM is then used to calculate the Jacobian with respect to the position and orientation. The Jacobian is used to calculate a correction magnitude. The correction magnitude is used to update the value of the initial guess of the orientation and the position and this process repeats until the norm of the correction magnitude reaches a preferred tolerance. Once the condition is met, the loop breaks and the iteratively updated guess vectors are output which is read as the spatial coordinates of the target with respect to the follower.

## Known Discrepancy
When looking at the output of the test cases, if the target is oriented X degrees, the position vector appears to be incorrect. The magnitude of this 'incorrect' vector is exactly equal to the magnitude of the true vector. We believe that if you were to rotate the vehicle and then draw the vector, you will see the correct position.


