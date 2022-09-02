import Sofa
from stlib3.scene import Scene, ContactHeader
import sofaros
from sofaros import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from summit_xl import SummitXL, Floor
from summitxl_roscontroller import *
from nav_msgs.msg import Odometry
from echelon3.parameters import *
from echelon3.createEchelon import *


rosNode = sofaros.init("SofaNode")


def createScene(rootNode):
    ContactHeader(rootNode, alarmDistance=0.5*1000, contactDistance=0.07*1000)

    #########################################
    # Plugins, data and Solvers
    ######################################### 

    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight");
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    rootNode.addObject('RequiredPlugin', name='BeamAdapter')
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SofaMeshCollision')
    rootNode.addObject('RequiredPlugin', name='SofaPlugins', pluginName='SofaGeneralRigid SofaGeneralEngine SofaConstraint SofaImplicitOdeSolver SofaSparseSolver SofaDeformable SofaEngine SofaBoundaryCondition SofaRigid SofaTopologyMapping SofaOpenglVisual SofaMiscCollision')

    scene = Scene(rootNode)
    scene.addMainHeader()
    scene.VisualStyle.displayFlags = 'showCollisionModels showForceFields'
    scene.addObject('DefaultVisualManagerLoop')
    scene.dt = 0.001
    scene.gravity = [0., -9810., 0.]

    scene.Modelling.addObject('EulerImplicitSolver')
    solver = scene.Modelling.addObject('SparseLDLSolver',name = 'SparseLDLSolver',template="CompressedRowSparseMatrixMat3x3d")
    scene.Modelling.addObject('GenericConstraintCorrection' , solverName='SparseLDLSolver')
    
    #########################################
    # create summit
    #########################################

    SummitXL(scene.Modelling)
    floor = Floor(rootNode,
                  name="Floor",
                  translation=[-2*1000, -0.12*1000, -2*1000],
                  uniformScale=0.3*1000,
                  isAStaticObject=True)

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

    ########################################
    # createEchelon
    ######################################## 

    arm = rootNode.Modelling.SummitXL.Chassis.addChild('Arm')
    connection = rootNode.Modelling.SummitXL.Chassis.Sensors.position
    createEchelon(arm,connection,3,[0., 0.26*1000, 0.19*1000],[-90,-90,0])

    scene.Simulation.addChild(scene.Modelling)

    return rootNode
