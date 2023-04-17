# turtlesim_plus
"turtlesim_plus" is a tool similar to "turtlesim" with some added features such as /spawn_pizza and /eat services..   

1.) Clone the repo to the src directory of your workspace. You must unzip and put each folder in the src directory.
2.) Add "dependencies_install.bash" to your workspace [outside of the src directory]. Execute the script to install all necessary Python libraries and ROS2 Packages.
```
source [your_workspace]/dependencies_install.bash
```

3.) Download a demo bag file from the following link and put in "/aruco_pipeline/aruco_detection/bag/my_new_bag". Do not change the name of the file.

https://drive.google.com/file/d/1EmaexBtwRY7qIY5cs90rXdc4MCuafJwY/view?usp=share_link

4.) [Optional] If you want the camera_calibration and image_view, you have to download the image_pipeline. [Note that installing every at once often cause the system to crash. Reinstalling again should be fine.]

https://github.com/ros-perception/image_pipeline

5.) Launch the following file
```
ros2 launch aruco_localization aruco_localization_bag.launch.py 
```
