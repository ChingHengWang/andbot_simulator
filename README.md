# andbot_simulator

### ~/.bashrc

    export ROS_MASTER_URI='http://localhost:11311'

    source ~/noros_ws/devel/setup.bash

    export ROS_IP='172.16.70.213'
   
    source ~/noros_ws/devel/setup.bash




### How to launch to debug navfn path?
    
    rosrun andbot_teleop andbot_teleop_key

    andbot_simulator_gmapping.launch

    rosrun map_server map_saver -f name

    roslaunch andbot_simulator_amcl.launch


