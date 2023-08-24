# prefab.MobileTrunk
[![SoftRobots](https://img.shields.io/badge/info-on_website-purple.svg)](https://project.inria.fr/softrobot/) 
[![SOFA](https://img.shields.io/badge/SOFA-on_github-blue.svg)](https://github.com/sofa-framework)
[![Ronotnik Automation](https://img.shields.io/badge/RonotnikAutomation-on_github-green.svg)](https://github.com/RobotnikAutomation)
[![Ros](https://img.shields.io/badge/Ros-on_readthedocs-chocolate.svg)](https://docs.ros.org/en/foxy/index.html)

The *MobileTrunk's* robot prefab for SoftRobots. 

Index
-----

  * [Requirement](#requirement)
  * [Compile sofa](#compile-sofa)
  * [Install Required dependencies](#install-required-dependencies)
  * [Download and set up working space](#download-and-set-up-working-space)
  * [Install *ROS* and set up working space](#install-ros-and-set-up-working-space)
  * [Launch test](#launch-test)
  * [Connecting the digital twin to the physical robot](#connecting-the-digital-twin-to-the-physical-robot)
  * [License](#license)

Requirement
-----------
This prefab was developed as part of the SOMOROB project. The goal is to integrate a
deformable robotic trunk (softrobot) on a 4-wheeled mobile robot base equipped with 
a LIDAR and to develop its digital twin equivalent. The deformable trunk is a project
[Echelon 3](https://www.inria.fr/en/interface-inria-centre-university-lille-demonstration-space)
developed by the Defrost team at INRIA. The mobile base is developed by the company
[Robotnik automation](https://robotnik.eu/)

Software version
---------------
The softwares are use with the following version :
- Python 3

The following Sofa plugins are required. version V21.12 is needed.

- SofaPython3
- BeamAdapter
- SoftRobots

The following python module are required :

- time
- threading
- math
- numpy
- dataclasses
- sys

Compile *SOFA*
--------------

Install Required dependencies
------------------------------------
For that follow these [instructions](https://www.sofa-framework.org/download/) (We will install the linux one)
<!--(The best way to add plugin to SOFA is explained [here](https://www.sofa-framework.org/community/doc/getting-started/video-tutorials/how-to-compile-a-plugin/)) 

- STLIB

    ```console
    foo@bar:~$  git clone https://github.com/SofaDefrost/STLIB.git
    ```

- SoftRobots

    ```console
    foo@bar:~$  git clone https://github.com/SofaDefrost/SoftRobots.git
    ```

- SofaPython3

    ```console
    foo@bar:~$  git clone https://github.com/sofa-framework/SofaPython3
    ```
    
- BeamAdapter

    ```console
    foo@bar:~$  git clone https://github.com/SofaDefrost/BeamAdapter
    ``` 
-->

Download and set up working space
---------------------------------

### Download the mobile trunk's prefab

Now that you have install SOFA binairy we can clone
prefab.MobileTrunk repository.

- Create a directory where you will put it
- Move into it and clone the last working version

    ```console
    foo@bar:~$  git clone https://github.com/CRIStAL-PADR/prefab.MobileTrunk
    ```

### Set up working space
Open your bashrc and add the following lines in order to setup your working space
- Add to the PATH the path to the folder where the binary files are located

    ```console
        export PATH="/path to binary folder/bin:$PATH"
    ```

- Tell to  *SOFA_ROOT* the path where to find the path to the buid folder of SOFA

    ```console
        export SOFA_ROOT=/path to binary folder
    ```

- Make an alias in order to be able to launch Sofa easily by doing a runSofa

    ```console
        runSofa="/path to binary folder/bin/runSofa"
    ```

- Add to the PYTHONPATH the path to *SofaPython3*, *STLIB* , *SofaROS* , *Python3*
    ```console
        export PYTHONPATH=$PYTHONPATH:"/path to binary folder/plugin/SofaPython3/lib/python3/site-packages":"/path to binary folder/plugin/STLIB/lib/python3/site-packages":"/usr/local/lib/python3.8/dist-packages":"/path to binary folder/plugin/SoftRobots/share/sofa/examples/SoftRobots/sofaros"
    ```

- Now you can source your .bashrc file with the command
    ```console
        source ~/.bashrc
    ```

- And run the command:
    ```console
        runSofa
    ```
If everything went well, you should see the Sofa GUI appear.

Now, click on the *Edit* button and go to *Plugin Manager*. Then click on *Add*. A window will appear. You need to navigate to the directory where the binary files are located. Go to the *plugins* directory, then to *ArticulatedSystemPlugin*, and then to *lib*. There, you will find a file named *libArticulatedSystemPlugin.so*. Click on *Open*.

Repeat the same procedure to add the *.so* files for the *SofaPython3* plugin.

Install *ROS* and set up working space
--------------------------------------

The digital twin has a *ros2* interface while the mobile platform (summit_xl) has a *ros1* interface.
It is possible to be able to link the digital twin to the real robot using rosbridge.

### Install ros2 and ros1

Install the [noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) version for ros1 in order to be able to use rosbridge

Install the [foxy](https://docs.ros.org/en/foxy/Installation.html) version for ros2.

If you are new to ros you can follow the [tutorials](https://docs.ros.org/en/foxy/Tutorials.html) before continuing

Install rosbridge
    
```console
    foo@bar:~$  sudo apt-get install -y ros-foxy-ros1-bridge
```

### Set up working space

Open your bashrc and add the following lines in order to setup your working space for ros

- Create an alias for noetic(ros1) and foxy(ros2) in order to be able to source your terminal more easily

    ```console
        alias foxy="source /opt/ros/foxy/setup.bash"
        alias noetic="source /opt/ros/noetic/setup.bash"
    ```

- Open sofaros.py file located in plugings/SoftRobots/share/sofa/examples/SoftRobots folder.
Then replace in the callback method of RosReceiver class the line
    ```console
        self.data = data.data by self.data = data
    ``` 
Launch test
-----------
To confirm all the previous steps and verify that the prefab is working properly you can :

#### Test using sofa controller

-  Launch the scene.py SOFA scene situated in prefab.MobileTrunk/mobile_trunk_sim by doing:
 ```console
    foo@bar:~$  runSofa scene.py --argv KeyboardController
```
- If everything went well you should see the GUI Sofa with the digital twin of

    ![somorob](/docs/somorob.png)


- Then with your keyboard send velocity and orientation command to the Sofa scene in order to see the robot
move.
 

#### Test using ros api

- source your terminal using : 
    ```console
    foo@bar:~$  foxy
    ```

- Go to the directory where the prefab.MobleTrunk/mobile_trunk_sim folder is located and run 
 the following command:

    ```console
    foo@bar:~$  runSofa scene.py --argv  RobotToSim
    ```
- Open a new terminal then source it as before then execute the commands to the following command:

    ```console
        foo@bar:~$  colcon build && . install/setup.bash && foxy
    ```

-  You can now launch the keyboard controller to interact with the robot in the simulation

    ```console
        foo@bar:~$  ros2 run summitxl summitxl_teleop_key

    ```

- If everything went well you should see this appear in the terminal
    
    ![ros_teleop_key](/docs/ros_teleopkey.png/)


## Echelon

### Controller

#### Forward kinematics

- ctl + i : To select the cable i (from 1 to 9) you want to actuate
- ctl + plus : if you want to pull the cable
- ctl + minus : if you want to release the cable

### Links

You can found the Echelon3 folder with the reference simulation there : https://gitlab.inria.fr/defrost/Robots/Echelon3.git with the branch Centrale_Dynamixel. The reference simulation is in the folder **simulation**.

There is also the ros2 package : https://gitlab.inria.fr/defrost/Robots/sim_echelon3_ros.git with the branch ros2.

You can find all the details and the explanation in this two repository.

Connecting the digital twin to the physical robot
-------------------------------------------------


License
-------

Creative Commons Zero v1.0 Universal see LICENSE
