# Assignment #1

Due: 2/17 (Wed) 02:30 PM

In this assignment, you will implement ROS nodes for controlling a simulated vehicle in CARLA.  

## 0. Preparation
1) Log in to your virtual machine
2) Check out the assignment repository into the catkin workspace. It should have been created by Github classroom. 
    ```
    cd ~/carla-ros-bridge/catkin_ws/src
    git clone https://github.com/YaleCPSC335/assignment1-[YourGitHubName].git
    ```
    where `YourGitHubName` is your GitHub username.

3) Copy the files under `ydriving` package (used in Lecture 3) from `~/carla-ros-bridge/catkin_ws/src/cpsc335_carla` into this repository.
    ```
    cd ~/carla-ros-bridge/catkin_ws/src/assignment1-[YourGitHubName]
    cp -r ~/carla-ros-bridge/catkin_ws/src/cpsc335_carla/*  .
    ```
    

2) Edit package.xml and CMakeLists.txt
    - Go to the package directory
        ```
        cd ~/carla-ros-bridge/catkin_ws/src/assignment1-[YourGitHubName]
        ```
    - Edit package.xml
        + Change the package name
            ```xml
            <name>assignment1_[YourGitHubName]</name>
            ```
        + Fill in your name and email address
            ```xml
            <maintainer email="your_email">Your name</maintainer>
            ```
    - Edit CMakeLists.txt
        + Change `project(cpsc335_carla)` to `project(assignment1_[YourGitHubName])`

3) Build the package
    ```
    cd ~/carla-ros-bridge/catkin_ws
    catkin_make
    ```


## 1. keyboard_control.py
#### Modify `keyboard_control.py` to publish `manual_throttle` and `manual_steer` topics of type `Float32`. 
+ Fill in the blocks that are labeled `YOUR CODE` in `keyboard_control.py`.
+ The topic names should be 
    + `/carla/vehicle_cpsc335_yourname/manual_throttle`
    + `/carla/vehicle_cpsc335_yourname/manual_steer`
    + where `vehicle_cpsc335_yourname` is your car's role name.
+ Do not forget to change the role_name
    + Example: 'vehicle_cpsc335_mkyoon'

+ Node name: `keyboard_control_[role name]`    
+ UP and DOWN arrow keys: throttle UP (+0.05) and DOWN (-0.05), respectively.
+ LEFT and RIGHT arrow keys: steering LEFT (-0.05) and RIGHT (+0.05), respectively.
+ SPACEBAR: reset both throttle and steer to zero    
+ If you are not sure whether the actuation commands are published, use `rostopic echo topic_name` command. 

## 2. driver.py
#### Modify `driver.py` so that it subscribes to the manual throttle and steer commands that are published by `keyboard_control.py`. 
+ Node name: `driver_[role name]`
+ Drive in the reverse direction if the throttle value is negative.   
+ Inside the periodic loop, print out the latest throttle and steer values.        
    + Use `rospy.loginfo()` to print 1) your name, 2) throttle, and 3) steering.        
        + Example: `name: Man-Ki Yoon, throttle: 0.35, steer: -0.05`
    
## 3. Test
If implemented correctly, you should be able to drive your simulated car using your keyboard. You need to launch 1) your CARLA client, 2) driver.py, and 3) keyboard_control.py.

## 4. Submission
- Make sure that the files and directories are added (using `git add` command).
- Also add a screenshot of an execution result.
- The submission version should indicate that it is the final version for grading (e.g., commit message: "FINAL - Your Name").  
- Your repository must include 
    + CMakeLists.txt
    + package.xml
    + src directory
        - keyboard_control.py
        - driver.py
    + screenshot image file
