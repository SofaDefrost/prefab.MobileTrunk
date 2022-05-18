from turtle import position
import Sofa
from stlib3.scene import Scene
from splib3.numerics import Quat
from math import pi
import sofaros
from sofaros import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from summit_xl import SummitXL, Floor
from summitxl_roscontroller import *
from nav_msgs.msg import Odometry

rosNode = sofaros.init("SofaNode")


def createScene(rootNode):
    scene = Scene(rootNode)
    scene.addMainHeader()
    scene.dt = 0.01
    scene.gravity = [0., -9810., 0.]

    SummitXL(scene.Modelling)
    Floor(scene.Modelling, rotation=[90,0,0], translation=[-2,-0.12,-2], scale=4)
    robot=scene.Modelling.SummitXL
    scene.Modelling.SummitXL.addObject(SummitxlROSController(name="KeyboardController", robot=scene.Modelling.SummitXL))


    scene.Modelling.SummitXL.addObject(sofaros.RosReceiver(rosNode, "/summit_xl/robotnik_base_control/cmd_vel",
                                           [robot.findData('robot_linear_vel'),robot.findData('robot_angular_vel')],
                                           Twist, vel_recv))


    scene.Modelling.SummitXL.addObject(sofaros.RosSender(rosNode, "/sofa_sim/imu/data",[robot.findData('sim_orientation'),
                                                        robot.findData('robot_angular_vel'), robot.findData('linear_acceleration'),
                                                        robot.findData('timestamp')],Imu, send))

    scene.Modelling.SummitXL.addObject(sofaros.RosSender(rosNode, "/sofa_sim/odom",[robot.findData('timestamp'),
                                                        robot.findData('sim_position'), robot.findData('sim_orientation'),
                                                        robot.findData('robot_linear_vel'), robot.findData('robot_angular_vel')],
                                                        Odometry, odom_send))

    scene.Modelling.SummitXL.addObject(sofaros.RosReceiver(rosNode, "/summit_xl/robotnik_base_control/odom",[robot.findData('timestamp'),
                                                            robot.findData('reel_position'), robot.findData('reel_orientation')],
                                                            Odometry, odom_recv))

    scene.Modelling.SummitXL.addObject(sofaros.RosSender(rosNode, "/sofa_sim/cmd_vel",
                                           [robot.findData('robot_linear_vel'),robot.findData('robot_angular_vel')],
                                           Twist, vel_send))
    scene.Simulation.addChild(scene.Modelling)

    return rootNode
