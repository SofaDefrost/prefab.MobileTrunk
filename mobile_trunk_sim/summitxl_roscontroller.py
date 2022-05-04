# coding: utf8
#!/usr/bin/env python3
import Sofa
from splib3.numerics import RigidDof, Quat
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


from math import *

def send(data):
    """This is a message to hold data from an IMU (Inertial Measurement Unit)

    Accelerations should be in m/s^2
    and rotational velocity should be in rad/sec


    Args:
        data[0]: Orientation
        data[1] : angular_vel
        data[2] : linear_acceleration

    Returns:
        msg: return  data as ros msg
    """
    msg = Imu()
    msg.orientation.x = data[0].value[0]
    msg.orientation.y = data[0].value[1]
    msg.orientation.z = data[0].value[2]
    msg.orientation.w = data[0].value[3]

    msg.angular_velocity.x = data[1].value[0]
    msg.angular_velocity.y = data[1].value[1]
    msg.angular_velocity.z = data[1].value[2]

    msg.linear_acceleration.x = data[2].value[0]
    msg.linear_acceleration.y = data[2].value[1]
    msg.linear_acceleration.z= data[2].value[2]

    msg.header.stamp.sec =  int(data[3].value[0])
    msg.header.stamp.nanosec = int(data[3].value[1])

    return msg

def vel_send(data):
    # 2 -> déplacement sur l'axe z
    msg = Twist()
    msg.linear.x = data[0].value[0]
    msg.linear.y = data[0].value[1]
    msg.linear.z = data[0].value[2]

    msg.angular.x = data[1].value[0]
    msg.angular.y = data[1].value[1]
    msg.angular.z = data[1].value[2]
    return msg

def vel_recv(data, datafield):
    datafield[0].value = [data.linear.x ,data.linear.y, data.linear.z]
    datafield[1].value = [data.angular.x ,data.angular.z, data.angular.y]


def time_recv(data, datafield):
    t = data.tolist()
    datafield.value = [t[0], t[1]]

def odom_recv(data, datafield):
    """ Function called by the RosReceiver. It
        allows to recover the odometry fields of the
        real robot

    Args:
        data (nav_msgs.msg.Odometry): Real robot odometer data
        datafield (nav_msgs.msg.Odometry): Data allowing to store the
                              odometery of the real robot.
                              Is used by the roscontroller
    sim_position = [x , y , z]   avec y = 0.0
    reel_position = [x, y, z]    avec z = 0.
    => sim_position = [reel_position.y ,reel_position.z, reel_position.x]

    sim_orientation = [ x, y, z, w] with  x = 0. and z = 0.
    reel_orientation = [x, y, z, w] with  x = 0. and y = 0.
    => sim_orientation = [reel_orientation.x, reel_orientation.z, reel_orientation.y,
                            reel_orientation.w]
    """
    datafield[0].value = [data.header.stamp.sec ,data.header.stamp.nanosec]

    datafield[1].value = [data.pose.pose.position.y,data.pose.pose.position.z,
                          data.pose.pose.position.x]

    datafield[2].value = [data.pose.pose.orientation.x,data.pose.pose.orientation.z,
                          data.pose.pose.orientation.y,data.pose.pose.orientation.w ]
    #print("position        ",datafield[1].value)
    #print("\n")
    #print("orientation     ",datafield[2].value)


def odom_send(data):
    """ Returns the odometry of
        the robot in the simulation
    Args:
        data (list): list containing the fields and values of the odometry of the
                     robot in simulation

    Returns:
            Objet de type nav_msgs.msg.Odometry
    """
    msg = Odometry()
    msg.header.stamp.sec = int(data[0].value[0])
    msg.header.stamp.nanosec = int(data[0].value[1])

    #print(data[1].value)
    #odom position x y z
    # I put a - sign so that the x position
    # of the robot in the simulation
    # corresponds to the  x position of the real robot
    #same for z orientation
    msg.pose.pose.position.x = -data[1].value[2]
    msg.pose.pose.position.z = data[1].value[1]
    msg.pose.pose.position.y = data[1].value[0]

    #odom orientation x y z w
    msg.pose.pose.orientation.x = data[2].value[0]
    msg.pose.pose.orientation.z = -data[2].value[1]
    msg.pose.pose.orientation.y = data[2].value[2]
    msg.pose.pose.orientation.w = data[2].value[3]

    #print(msg.pose.pose.position, "------>", msg.pose.pose.orientation)
    #odom linear & angular vel
    #linear vel
    msg.twist.twist.linear.x = data[3].value[0]
    msg.twist.twist.linear.y = data[3].value[1]
    msg.twist.twist.linear.z = data[3].value[2]

    #angular vel
    msg.twist.twist.angular.x = data[4].value[0]
    msg.twist.twist.angular.y = data[4].value[1]
    msg.twist.twist.angular.z = data[4].value[2]
    return msg


class SummitxlROSController(Sofa.Core.Controller):
    """A Simple keyboard controller for the SummitXL
       Key UP, DOWN, LEFT, RIGHT to move
    """
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.robot = kwargs["robot"]
        self.wheel_ray = 0.25
        self.flag = True

    def move(self, fwd, angle):
        """Move the robot using the forward speed and angular speed)"""
        robot = RigidDof(self.robot.Chassis.position)
        robot.translate(robot.forward * fwd)
        robot.rotateAround([0, 1, 0], angle)
        with self.robot.Chassis.WheelsMotors.angles.position.writeable() as angles:
            #Make the wheel turn according to forward speed
            # TODO: All the value are random, need to be really calculated
            angles += (fwd/self.wheel_ray)

            #Make the wheel turn in reverse mode according to turning speed
            # TODO: the value are random, need to be really calculated
            angles[0] += (angle)
            angles[2] += (angle)
            angles[1] -= (angle)
            angles[3] -= (angle)

    def init_pose(self):
        with self.robot.Chassis.position.position.writeable() as summit_pose:
            #position x, y z
            summit_pose[0][0] = self.robot.reel_position[0]
            summit_pose[0][1] = self.robot.reel_position[1]
            summit_pose[0][2] = self.robot.reel_position[2]

            #orientaion x y z w
            summit_pose[0][3] = self.robot.reel_orientation[0]
            summit_pose[0][4] = self.robot.reel_orientation[1]
            summit_pose[0][5] = self.robot.reel_orientation[2]
            summit_pose[0][6] = self.robot.reel_orientation[3]

    def onAnimateBeginEvent(self, event):
        """At each time step we move the robot by the given forward_speed and angular_speed)
           TODO: normalize the speed by the dt so it is a real speed
        """
        if self.flag :
            self.init_pose()
            print("init")

        self.flag = False

        dt = event['dt']
        self.robot.simrobot_linear_vel[0] = self.robot.reelrobot_linear_vel[0]  * dt
        self.robot.simrobot_angular_vel[2] = self.robot.reelrobot_angular_vel[2] * dt

        self.robot.linear_acceleration[0] = self.robot.simrobot_linear_vel[0]/(dt)

        self.robot.sim_orientation[0] = self.robot.Chassis.position.position.value[0][0+3]
        self.robot.sim_orientation[1] = self.robot.Chassis.position.position.value[0][1+3]
        self.robot.sim_orientation[2] = self.robot.Chassis.position.position.value[0][2+3]
        self.robot.sim_orientation[3] = self.robot.Chassis.position.position.value[0][3+3]


        self.robot.sim_position[0] = self.robot.Chassis.position.position.value[0][0]
        self.robot.sim_position[1] = self.robot.Chassis.position.position.value[0][1]
        self.robot.sim_position[2] = self.robot.Chassis.position.position.value[0][2]

        #print(self.robot.Chassis.position.position.value[:2])
        self.move(self.robot.simrobot_linear_vel[0], self.robot.simrobot_angular_vel[2])
