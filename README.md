# System Architecture Diagram

The main idea is that the car will follow smoothly(controlled with a PID and a low pass filter) every waypoint of the map using the drive-by-wire node(DBW), which will determine the vehicle acceleration, brake and steering. When a red traffic light is detected in front of the vehicle, the perception module will update the waypoints so that the control module can apply the suitable deceleration to make the vehicle come to a stop.

![image](ros-graph.png)