import sys
import Sofa
from stlib3.physics.rigid import Floor
from  mobile_trunk import mobileTrunk
from summitxl_controller import *



def createScene(rootNode):


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
    scene.addContact(alarmDistance=0.1*1000, contactDistance=0.05*1000, frictionCoef=8)
    scene.VisualStyle.displayFlags = 'hideBehaviorModels showForceFields showCollisionModels showInteractionForceFields'
    scene.addObject('DefaultVisualManagerLoop')
    scene.dt = 0.01
    scene.gravity = [0., -9810., 0.]
    scene.Settings.removeObject(scene.Settings.OglSceneFrame)

    scene.Simulation.addObject('GenericConstraintCorrection', solverName='LinearSolver', ODESolverName='GenericConstraintSolver')

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
                  translation=[-2*1000, -0.1*1000, 2*1000],
                  uniformScale=0.1*1000,
                  isAStaticObject=True)
 
    print(sys.argv[1])
    arg = sys.argv[1]
    arg = arg.split()
    print(arg)


    robot_angular_speed = float(arg[0])
    robot_linear_speed = float(arg[1])
    sim_duration = float(arg[2])
    scene.Modelling.SummitXL.addObject(SummitxlController(name="KeyboardController", robot=scene.Modelling.SummitXL,
                                                          angular_speed = robot_angular_speed,
                                                          linear_speed = robot_linear_speed,
                                                          duration = sim_duration, test= True)
                                                        )

        #print(sys.argv)
        #def myAnimation(target, body, factor):
        #    body.position += [[0.0,0.0,0.001,0.0,0,0,1]]
        #    target.position = [[factor* 3.14 * 2]]*len(target.position.value)

        #animate(myAnimation, {
        #        "body" : scene.Modelling.SummitXL.Chassis.position,
        #        "target": scene.Modelling.SummitXL.Chassis.WheelsMotors.angles}, duration=2, mode="loop")


    return rootNode