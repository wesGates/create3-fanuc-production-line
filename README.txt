!! Make sure that your Ubuntu is on Wayland !!
Steps if not (only needs to be done once):
    1. Log out
    2. Select the gear in the bottom right
    3. Select 'Ubuntu on Xorg'
        - If you have 'Ubuntu on Wayland' instead, you're already on Xorg
    4. Log back in
    

When cloning for the first time, make sure you build the folder first; this step only needs to be done once. 
In your terminal, go to this folder and type:
    colcon build
and ROS will do the rest for you. After building, you will need to run:
    source install/setup.sh
before running any code in the "src" folder
