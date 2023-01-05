
**face detectionr**\
colcon build\
source install/setup.bash\
ros2 launch vision2 vision2.test.launch.py


**rqt view**\
source install/setup.bash\
ros2 run rqt_image_view rqt_image_view

**expression detection**\
source install/setup.bash\
ros2 run vision2 expression_detection_node

**Tracking**\
Tracking can be enabled via changing\
expression det sub and launching tracking node

**Action node**\

Under development\
source install/setup.bash\
ros2 run vision2 vision2_action_msgs_node