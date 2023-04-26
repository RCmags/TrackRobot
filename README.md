# Autonomous tracked Robot 
This is a large arduino sketch for a stracked vehicle capable of driving itself to different waypoints. It is composed of multiple header files and a main.ino file. The code is intended for an __Arduino MEGA__. 

## How it works
The position of the vehile is estimated using a physical model of the vehicle, an accelerometer that is integrated twice, a gyroscope for heading, and the displacement measured by two optical flow sensors. All of this data is fused to obtain the net displacements relative to an initial position using rectangular coordinates. These position and orientation estimates are used to guide the vehicle to a specified location using two PID loops, one for displacements and one for heading.  

The coordinates are stored in a buffer that can be filled in real time via a bluetooth module. Coordinates can be pushed or poped off the buffer. Once the vehicle is within a given radius of a coordinate, the following coodinate is made the target destination. This process will continue indefinitely and the vehicle will follow a closed path with the coordinates as the vertices. 

## Dependencies
The sketch requires the following libraries: 

- [imuFilter v1.0.0](https://github.com/RCmags/imuFilter)
- [basicMPU6050 v0.1.0](https://github.com/RCmags/basicMPU6050)
- [ADNS3080 v1.0.1](https://github.com/RCmags/ADNS3080)
- [Vector datatypes v1.0.1](https://github.com/RCmags/vector_datatype)

## Example
Below is the small tracked robot built for this sketch:  

<p align="center">
<img src = "/images/tank1_res.jpg" width = "30%"> <img src = "/images/tank2_res.jpg" width = "30%"> <img src = "/images/tank3_res.jpg" width = "30%">  
</p>

<p align="center">
<img src = "/images/inclined_plane.gif" width = "30%"> <img src = "/images/disturbance.gif" width = "30%"> <img src = "/images/zip_zag.gif" width = "30%">
</p>

See this video: [functionality and performance of an autonomous tracked robot](https://www.youtube.com/watch?v=VYhLW5owS3A&t=3s)
