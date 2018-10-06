Turtlebot_arm_moveit_config and turtlebot arm_description are gotten from the package phantomx_rst

## How to Start
roslaunch pxpincher_launch pxp.launch 

Ctrl + c

__
roslaunch phantomx_pincher_robot_arm_edited_moveit_config arm.launch 


__
roslaunch skeleton_tracker tracker.launch 


__
rosrun raw_skeleton skeleton_tf_listener 


__
rosrun raw_skeleton feature_extraction.py


__
rosrun pincher_commander pinchercommander_node.py


__ = New Terminal

Paketler için (catkin_ws/src içine git clone ile alınıp, catkin_make yapılabilir)
https://github.com/rst-tu-dortmund/phantomx_rst #For kinetic, add "add_compile_options(-std=c++11)" to the cmakelists of the every package in the phantomx_rst metapackage.
https://github.com/ozzdemir/skeleton_tracker
https://github.com/ozzdemir/raw_skeleton
https://github.com/ozzdemir/pincher_commander
https://github.com/ozzdemir/phantomx_pincher_robot_arm_edited
https://github.com/ozzdemir/phantomx_pincher_robot_arm_edited_moveit_config


skeleton trackerin çalışması için https://drive.google.com/open?id=0B-SFThXVfVhRNWQzMEVRc0VHNU0 klasöründeki dökumanları dikkatle okuyup, NiTE yi ve openni2 yi yüklemeniz gerekiyor.

pxp.launchun baştan config fileları yüklemek için sonrasında kapatıp arm.launchu çalıştırmak gerekiyor.
