import Sofa
from splib3.numerics import RigidDof, Quat
from splib3.animation import animate
from splib3.constants import Key
from stlib3.scene import Scene
from math import *

msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    p
   j    k    l
   m    ,    .

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'p': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1)
   }

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
   }

def vels(speed, turn):
       return 'currently:\tspeed %s\tturn %s ' % (speed, turn)
class SummitxlController(Sofa.Core.Controller):
    """A Simple keyboard controller for the SummitXL
       Key UP, DOWN, LEFT, RIGHT to move
    """
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.robot = kwargs["robot"]
        self.wheel_ray = 0.0015
        self.dt = 0

        self.status = 0.
        self.speed = 0.1
        self.turn = 0.1
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.robot.simrobot_angular_vel = [0., 0., 0.]
        self.robot.simrobot_linear_vel = [0., 0., 0.]

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

    def onAnimateBeginEvent(self, event):
        """At each time step we move the robot by the given forward_speed and angular_speed)
           TODO: normalize the speed by the dt so it is a real speed
        """
        self.dt = event['dt']
        self.move(self.robot.simrobot_linear_vel[0] , self.robot.simrobot_angular_vel[2])
        print("position x = ",self.robot.Chassis.position.position.value[0][0])
        print("position y = ",self.robot.Chassis.position.position.value[0][1])
        print("position z = ",self.robot.Chassis.position.position.value[0][2])
        print("\n")
        print("orientation x = ",self.robot.Chassis.position.position.value[0][3])
        print("orientation y = ",self.robot.Chassis.position.position.value[0][4])
        print("orientation z = ",self.robot.Chassis.position.position.value[0][5])
        print("orientation w = ",self.robot.Chassis.position.position.value[0][6])






    def onKeypressedEvent(self, event):
        key = event['key']
        key = key.lower()
        if key in moveBindings.keys():
            self.x = moveBindings[key][0]
            self.th = moveBindings[key][3]
        elif  key in speedBindings.keys():
            self.speed = self.speed + speedBindings[key][0]
            self.turn = self.speed + speedBindings[key][1]

            print(vels(self.speed, self.turn))
            if (self.status == 14):
                print(msg)
            self.status = (self.status + 1) % 15
        else:
            self.x = 0.0
            self.th = 0.0
        self.robot.simrobot_linear_vel[0] =  self.x * self.speed * self.dt
        self.robot.simrobot_angular_vel[2] = self.th * self.turn * self.dt



    def onKeyreleasedEvent(self, event):
        key = event['key']
        key = key.lower()
        if key in moveBindings.keys() or key in speedBindings.keys():
            self.robot.simrobot_linear_vel[0]= 0
            self.robot.simrobot_angular_vel[2] = 0
