
**face detection + object_tracker**\
colcon build\
ros2 launch vision2 vision2.test.launch.py\
source install/setup.bash\

**rqt view**\
ros2 run\
ros2 run rqt_image_view rqt_image_view\

**expression detection**\
source install/setup.bash\
ros2 run vision2 expression_detection_node\