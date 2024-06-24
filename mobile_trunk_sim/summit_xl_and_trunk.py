import Sofa
from stlib3.scene import Scene
from stlib3.physics.rigid import Floor
from summitxl_controller import *
from summit_xl import SummitXL
from echelon3.parameters import *
from echelon3.createEchelon import *


def createScene(rootNode):

    #########################################
    # Plugins, data and Solvers
    ######################################### 

    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight");
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    rootNode.addObject('RequiredPlugin', name='BeamAdapter')
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SofaMeshCollision')
    rootNode.addObject('RequiredPlugin', name='SofaPlugins', pluginName='SofaGeneralRigid SofaGeneralEngine SofaConstraint SofaImplicitOdeSolver SofaSparseSolver SofaDeformable SofaEngine SofaBoundaryCondition SofaRigid SofaTopologyMapping SofaOpenglVisual SofaMiscCollision')

    scene = Scene(rootNode, iterative=False)
    scene.addMainHeader()
    scene.addContact(alarmDistance=0.2*1000, contactDistance=0.005*1000)
    scene.VisualStyle.displayFlags = 'hideBehaviorModels showForceFields showCollisionModels showInteractionForceFields'
    scene.addObject('DefaultVisualManagerLoop')
    scene.dt = 0.001
    scene.gravity = [0., -9810., 0.]


    #scene.Modelling.addObject('EulerImplicitSolver',rayleighStiffness=0.01, rayleighMass=0, vdamping=0.1)
    #solver = scene.Modelling.addObject('SparseLDLSolver',name = 'SparseLDLSolver',template="CompressedRowSparseMatrixMat3x3d")

    scene.Simulation.TimeIntegrationSchema.vdamping.value = 0.1
    scene.Simulation.TimeIntegrationSchema.rayleighStiffness = 0.01
    scene.Simulation.addObject('GenericConstraintCorrection' , solverName='LinearSolver', ODESolverName='GenericConstraintSolver')

    #########################################
    # create summit
    #########################################
    #scene.Simulation.addChild(scene.Modelling)
    scale = 1000
    SummitXL(scene.Modelling, scale)
    floor = Floor(rootNode,
                  name="Floor",
                  translation=[-2*1000, -0.12*1000, -2*1000],
                  uniformScale=0.1*1000,
                  isAStaticObject=True)

    #def myAnimation(target, body, factor):
    #    body.position += [[0.0,0.0,0.001,0.0,0,0,1]]
    #    target.position = [[factor* 3.14 * 2]]*len(target.position.value)

    #animate(myAnimation, {
    #        "body" : scene.Modelling.SummitXL.Chassis.position,
    #        "target": scene.Modelling.SummitXL.Chassis.WheelsMotors.angles}, duration=2, mode="loop")

    scene.Modelling.SummitXL.addObject(SummitxlController(name="KeyboardController", robot=scene.Modelling.SummitXL, scale=scale))

    ########################################
    # createEchelon
    ######################################## 

    trunk = scene.Modelling.SummitXL.Chassis.addChild("Trunk")
    trunk.addObject("MechanicalObject", name = "position", template="Rigid3d",
                    position=[0., 0.26*1000, 0.32*1000,-0.5, -0.5, -0.5 , 0.5 ],
                     showObject=True,showObjectScale = 30)    
    trunk.addObject('RigidRigidMapping',name='mapping', input=scene.Modelling.SummitXL.Chassis.position.getLinkPath(),
                                                index=0)

    scene.Modelling.SummitXL.Chassis.addChild('Arm')

    arm = scene.Simulation.addChild(scene.Modelling.SummitXL.Chassis.Arm)
    connection = rootNode.Modelling.SummitXL.Chassis.Trunk.position
    parameters, cables = createEchelon(arm,connection,0,[0., 0.26*1000, 0.32*1000],[-90,-90,0])

    if typeControl == 'displacement':
        arm.addObject(CableController(cables, name = 'Cablecontroller'))
    elif typeControl == 'force' :
        arm.addObject(ForceController(cables,dt,name = 'ForceController'))
    return rootNode