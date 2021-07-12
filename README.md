# AWS-Enabled Waveshare Jetbot with NVidia Jetson Nano

A collection of ROS sample applications for the NVidia JetBot to run in AWS RoboMaker. This repository includes the following sample applications:

- **Line Following:** This simple navigation application parses frames from the camera feed using OpenCV and instructs the Jetbot to follow a line.
- **SLAM and GMapping:** This sample application will run move base, gmapping and SLAM - enabling you to generate maps of AWS-provided 3D worlds.
- **Simple Motion** This sample application performs basic rotate in place and forward motion commands. 

## Setup 

To run these sample applications, you can use the AWS RoboMaker IDE. For a full description on how to setup and get started with the virtual deskop feature in AWS RoboMaker, [click here](https://aws.amazon.com/blogs/robotics/aws-announces-a-new-developer-desktop-feature-within-the-aws-robomaker-ide/).

1. Open AWS RoboMaker in the AWS console. Click on *Development Environments*. 
2. Click *Create development environment*. It will take a few minutes to provision your IDE. Once done, it will open a new window. 
3. In the terminal of the IDE (bottom pane), run the following commands:

    ```bash
        cd ~/environment
        git clone https://github.com/aws-samples/aws-robomaker-jetbot-ros
        cd aws-robomaker-jetbot-ros
        chmod +x ./setup.sh
        ./setup.sh
    ```

## Develop and interact with simulations in AWS RoboMaker

The sample applications in this repository can be run with a Gazebo-based simulation.

1. Open a **Virtual Desktop** by clicking the virtual desktop button in the top navigation tool bar. This will open a pop-up window, if you are prompted, allow your browser to open the pop-up window. 
2. Move the new window adjacent to your IDE browser window. In the terminal of the IDE, set the display to the virtual desktop:

    ```bash
    export DISPLAY=:0
    ```

3. Next, we will build the ROS application to run in the simulation. *Note: every time you make a code change to the ROS packages in this sample repository, you will need to run this command.*

    ```bash
    cd ~/environment/aws-robomaker-jetbot-ros
    colcon build
    ```

4. Now that the ROS application is built, we can source the application and run the simulation. Let's start with the line following sample.

    ```bash
    source install/setup.sh
    roslaunch line_following line_following line_following_sim.launch
    ```

5. Next, try running the SLAM and gmapping Demo. There are three worlds to choose from; `small_house`, `bookstore` and `small_warehouse`. Simply update the `world:=<world>` paramater with the world you would like to use!

    ```bash
    source install/setup.sh
    roslaunch slam_demo explore_world.launch world:=bookstore
    ```

**Congratulations!!** The simulation should now be running. To run another application, look through the **aws_example_apps** folder. In each example app, there is a launch folder with launch files you can run using the command structure above. 

## Deploy and run with an NVidia Jetbot Kit

Coming Soon! 