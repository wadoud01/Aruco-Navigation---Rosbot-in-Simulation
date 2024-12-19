## Project Overview
This project is designed to facilitate Aruco navigation for a ROSbot robot, wherein it autonomously follows a specific Aruco Marker while maintaining a minimum safe distance. The implementation leverages ROS and Python as the primary software stacks, with the incorporation of OpenCV for Aruco detection.

An ArUco marker, utilized in this context, is a synthetic square marker characterized by a distinct black border and an internal binary matrix that determines its unique identifier (ID). The black border expedites rapid detection in the image, while the binary codification enables identification and the application of error detection and correction techniques. The size of the marker determines the pixel size of the ArUco in the the camera frame. 

In our implementation, we have employed four markers with IDs 11, 12, 13, and 15. Each marker serves a specific purpose in the navigation sequence:

- Marker 11: Instructs the robot to rotate until Marker 12 is detected, then proceed to reach Marker 12.
- Marker 12: Directs the robot to rotate until Marker 13 is detected, then move towards reaching Marker 13.
- Marker 13: Guides the robot to rotate until Marker 15 is detected, and subsequently, reach Marker 15.
- Marker 15: Signifies the completion of the navigation task.
This structured approach ensures a systematic and autonomous navigation process based on the identification and sequence of ArUco markers.

## Flowchart

Encapsulating the intricacies of our Aruco navigation system, the flowchart provided below delves into the fundamental logic and sequential steps that govern the robot's autonomous movements.

 <img src="https://github.com/Melih199/Exp_rob_assignment_1/assets/58879182/5d27ada1-4e73-41b3-b62d-09fe41a9a5f3.type" width="1000" height="500">

**For simulation**:
```bash
roslaunch assignment_1 aruco_navigation.launch
```

<img src="https://github.com/Melih199/Exp_rob_assignment_1/blob/Rosbot_aruco_simulation/simulation_node.gif" width="600" height="300"/>

### aruco_detector node ###

In this node we used the image information provided from /camera/color/image_raw and /camera/color/camera_info topics to find the ArUco markers.

```python
self.camera_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_callback)

```

Then we used this information to calculate the marker_center and marker_size.

```python
marker_center_x = (corners[0][0][0][0] + corners[0][0][1][0] +
                                corners[0][0][2][0] + corners[0][0][3][0]) / 4
marker_center_y = (corners[0][0][0][1] + corners[0][0][1][1] +
                                corners[0][0][2][1] + corners[0][0][3][1]) / 4            

marker_center = [marker_center_x, marker_center_y]
            
top_right = [corners[0][0][0][0], corners[0][0][0][1]]
bottom_right = [corners[0][0][3][0], corners[0][0][3][1]]
x_cord = top_right[0] - bottom_right[0]
y_cord = top_right[1] - bottom_right[1]
size = int(np.sqrt(np.power(x_cord, 2) + np.power(y_cord, 2)))
```

Finally the node publishes the marker_center, marker_size, id(provided with ArUco library) camera_center(provided with /camera/color/camera_info topic).

```python
self.aruco_info_pub = rospy.Publisher('aruco_info', Aruco_info, queue_size=10)
...
def camera_callback(self, camera_msg):
        self.camera_center_x = camera_msg.width / 2
        self.camera_center_y = camera_msg.height / 2
...
info_msg = Aruco_info()
info_msg.id = int(ids[0][0])
info_msg.camera_center = camera_center
info_msg.marker_center = marker_center
info_msg.marker_size = [size]            
self.aruco_info_pub.publish(info_msg)
```

### robot_control node 

This node is a simple PID controller for our robot it is using the information proided from  aruco_info topic and published the velocity commands 
on /cmd_vel topic. To better understanding of this node check the flowchart above.


```python
self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
self.aruco_info_sub = rospy.Subscriber('aruco_info', Aruco_info, self.aruco_callback)
...
def control_loop(self):
        self.marker_turn=0
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():

            if self.marker_id_list[self.marker_turn] != self.vision_id:
                # Rotate to search for the target ArUco marker
                self.vel = Twist()          
                self.vel.angular.z = -0.4
                self.vel_pub.publish(self.vel)
        
            else:
                self.angular_error = self.camera_center[0] - self.marker_center[0]
                self.linear_error = self.target_size - self.marker_size[0]

                # Proportional control
                self.vel.linear.x = self.linear_gain * self.linear_error
                self.vel.angular.z = self.angular_gain * self.angular_error

                # Check for saturation in linear and angular velocities
                self.vel.linear.x = max(min(self.vel.linear.x, 1.0), -1.0)
                self.vel.angular.z = max(min(self.vel.angular.z, 1.0), -1.0)

                # Publish velocity command
                self.vel_pub.publish(self.vel)

                if self.marker_size[0] >= self.target_size:
            
                    if self.marker_id_list[self.marker_turn] != 15:
                        self.marker_turn = self.marker_turn+1
                        self.vel.angular.z = -0.4
                        self.vel_pub.publish(self.vel)
                    else:
                        rospy.signal_shutdown("exit")
            
            self.print_infos()
            rate.sleep()
```

**Results**

<img src="Rosbot Real World Scenario.gif" width="1000" height="400"/>

## Conclusion and Future Improvements

In concluding our Aruco Navigation project, our team has successfully developed a robust system for autonomous robot navigation using Aruco markers. The project's structure and implementation, detailed in the README, lay the foundation for future enhancements and applications.

### Achievements:

1. **Aruco Navigation:** The project achieves its primary goal of enabling a ROSbot robot to autonomously follow a predefined sequence of Aruco markers while maintaining a safe distance. The utilization of ROS, Python, and OpenCV contributes to the efficiency of the navigation system.

2. **Marker Logic:** The distinctive logic associated with each Aruco marker (11, 12, 13, and 15) provides a systematic approach, ensuring a seamless navigation sequence. The markers' roles in instructing the robot to rotate, detect subsequent markers, and reach specific destinations add a layer of intelligence to the navigation process.

3. **Flowchart:** The included flowchart offers a comprehensive and deep insight into the algorithmic underpinnings of the navigation system. This serves as a valuable resource for understanding the logical flow and procedural steps governing the robot's movements.



