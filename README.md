# prefab.MobileTrunk

The MobileTrunk's robot prefab for SoftRobots.

## Echelon

### Controller

#### Forward kinematics

- ctl + i : To select the cable i (from 1 to 9) you want to actuate
- ctl + plus : if you want to pull the cable
- ctl + minus : if you want to release the cable

### Software version

The softwares are use with the following version :

- Sofa V21.12
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

### Links

You can found the Echelon3 folder with the reference simulation there : https://gitlab.inria.fr/defrost/Robots/Echelon3.git with the branch Centrale_Dynamixel. The reference simulation is in the folder **simulation**.

There is also the ros2 package : https://gitlab.inria.fr/defrost/Robots/sim_echelon3_ros.git with the branch ros2.

You can find all the details and the explanation in this two repository.
