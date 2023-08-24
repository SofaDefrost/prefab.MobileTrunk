import Sofa
import sys
from stlib3.scene import Scene
from summitxl_roscontroller import *
from stlib3.physics.rigid import Floor
from mobile_trunk import mobileTrunk
import sofaros
from sofaros import *

#from geometry_msgs.msg import Twist
#from sensor_msgs.msg import Imu
#from nav_msgs.msg import Odometry

rosNode = sofaros.init("SofaNode")


def createScene(rootNode):

    print("PYTHON: ", sys.argv)

    #########################################
    # Plugins, data and Solvers
    ######################################### 
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    rootNode.addObject('RequiredPlugin', name='BeamAdapter')
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SofaMeshCollision')
    rootNode.addObject('RequiredPlugin', name='SofaPlugins', pluginName='SofaGeneralRigid SofaGeneralEngine SofaConstraint SofaImplicitOdeSolver SofaSparseSolver SofaDeformable SofaEngine SofaBoundaryCondition SofaRigid SofaTopologyMapping SofaOpenglVisual SofaMiscCollision')


    scene = Scene(rootNode, iterative=False)
    scene.addMainHeader()
    scene.addContact(alarmDistance=0.2*1000, contactDistance=0.005*1000, frictionCoef=1)
    scene.VisualStyle.displayFlags = 'hideBehaviorModels showForceFields showCollisionModels showInteractionForceFields'
    scene.addObject('DefaultVisualManagerLoop')
    scene.dt = 0.01
    scene.gravity = [0., -9810., 0.]
    scene.Settings.removeObject(scene.Settings.OglSceneFrame)

    scene.Simulation.addObject('GenericConstraintCorrection', solverName='LinearSolver', ODESolverName='GenericConstraintSolver')

    #scene.Simulation.addObject('GenericConstraintCorrection', linearSolver='@LinearSolver', ODESolver='@GenericConstraintSolver')
    scene.Simulation.TimeIntegrationSchema.vdamping.value = 0.1
    scene.Simulation.TimeIntegrationSchema.rayleighStiffness = 0.01
    
    #########################################
    # add robot
    #########################################
    self, trunk= mobileTrunk(scene.Modelling)
    scene.Simulation.addChild(trunk)
    scene.Simulation.addChild(self)
    floor = Floor(rootNode,
                  name="Floor",
                  translation=[-2*1000, -0.12*1000, -2*1000],
                  uniformScale=0.1*1000,
                  isAStaticObject=True)

    robot=scene.Modelling.SummitXL

    scene.Modelling.SummitXL.addObject(SummitxlROSController(name="KeyboardController", robot=robot))

    scene.Modelling.SummitXL.addObject(sofaros.RosReceiver(rosNode, "/summit_xl/cmd_vel",
                                                           [robot.findData('robot_linear_vel'),
                                                            robot.findData('robot_angular_vel')],
                                                           Twist, vel_recv))

    #scene.Modelling.SummitXL.addObject(
    #    sofaros.RosSender(rosNode, "/sofa_sim/imu/data", [robot.findData('sim_orientation'),
    #                                                      robot.findData('robot_angular_vel'),
    #                                                      robot.findData('linear_acceleration'),
    #                                                      robot.findData('timestamp')], Imu, send))

    # scene.Modelling.SummitXL.addObject(sofaros.RosSender(rosNode, "/summit_xl/robotnik_base_control/odom",
    #                                                                                 [robot.findData('timestamp'), 
    #                                                                                 robot.findData('sim_position'), 
    #                                                                                 robot.findData('sim_orientation'),
    #                                                                                 robot.findData('robot_linear_vel'), 
    #                                                                                 robot.findData('robot_angular_vel')], 
    #                                                                                 Odometry, odom_send))

    scene.Modelling.SummitXL.addObject(
       sofaros.RosReceiver(rosNode, "/summit_xl/robotnik_base_control/odom", [robot.findData('timestamp'),
                                                                              robot.findData('reel_position'),
                                                                              robot.findData('reel_orientation'),
                                                                              robot.findData('robot_linear_vel'),
                                                                              robot.fincData('robot_angular_vel')],
                           Odometry, odom_recv))

    scene.Modelling.SummitXL.addObject(sofaros.RosSender(rosNode, "/summit_xl/robotnik_base_control/cmd_vel",
                                           [robot.findData('robot_linear_vel'),robot.findData('robot_angular_vel')],
                                           Twist, vel_send))

    return rootNode
