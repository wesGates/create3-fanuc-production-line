When cloning for the first time, make sure you build the folder first; this step only needs to be done once. 
In your terminal, go to this folder and type:
    colcon build
and ROS will do the rest for you. After building, you will need to run:
    source install/setup.sh
before running any code in the "src" folder
