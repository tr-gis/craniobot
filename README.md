Steps:
--------------------------
1. roslaunch craniobot_controller craniobot_controller.launch
This launches all the actuators of the craniobot and their corresponding controllers.

2. roslaunch craniobot_ros_control craniobot_control.launch
This launches the ros control node where the read and write to driver node is modified according to the gears attached with the actuators.

3. roslaunch craniobot_5kg_moveit_config demo.launch
This launches the files required for the moveit motion planning. This opens rviz as well.

4. Choose panels --> Add new panel -- >MoveIt Cartesian Plan Plug-in -->Click ok.
This adds the cartesian path planner plugin into moveit.
Note:
------
This plugin is not available in the moveit by default. It should installed in the workspace seperately. The name of the package is fermi-indigo-devel. The ros wiki tutorials is available in the below link.
http://wiki.ros.org/moveit_cartesian_plan_plugin

5. Add interactive markers from display tab and set the topic to /moveit_cartesian_plan_plugin/update

6. For creating path and saving it refer the tutorial listed above.

Note:
-----
To make changes in the GUI of cartesian path planning, it should be done in the QT software.Currently the plugin is set to its default values.

