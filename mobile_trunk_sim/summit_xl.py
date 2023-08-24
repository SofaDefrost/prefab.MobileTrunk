import Sofa

from stlib3.scene import Scene
from stlib3.scene import ContactHeader
from stlib3.physics.rigid import Floor

from summitxl_controller import *

from echelon3.parameters import *
from echelon3.createEchelon import *

def Chassis():

    #########################################
    # parameters
    #########################################
    totalMass = 1.0
    volume = 1.0*1e9
    inertiaMatrix = [1.0*1e6, 0.0, 0.0, 0.0, 1.0*1e6, 0.0, 0.0, 0.0, 1.0*1e6]

    wheelPositions = [[0.229*1000, 0,0.235*1000],
                      [-0.229*1000, 0,0.235*1000],
                      [0.229*1000, 0,-0.235*1000],
                      [-0.229*1000, 0,-0.235*1000]]

    sensorPositions = [[0., 0.28*1000 ,0.],          # 2d lazer
                       [0,0.275*1000,-0.22*1000],         # gps
                       ]

    sensorName=["lazer", "gps"]

    trunkPosition = [0., 0.26*1000, 0.32*1000,-0.5, -0.5, -0.5 , 0.5 ]

    #########################################
    #Add chassis mechanical Object
    #########################################

    self = Sofa.Core.Node("Chassis")

    base = self.addChild('Base')
    base.addObject("MechanicalObject", name="position", template="Rigid3d", position=[[0,0,0,0,0,0,1]])
    base.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])
    base.addObject('UncoupledConstraintCorrection')

    #########################################
    #creation of the articulated chain of wheels, sensors
    #########################################

    #wheels
    chain = self.addChild("WheelsMotors")
    chain.addObject('MechanicalObject', name="angles", template="Vec1d", position=[0,0,0,0,0])
    chain.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])
    chain.addObject('RestShapeSpringsForceField', stiffness=1e12)

    #capteur
    sensor = self.addChild("FixedSensor")
    sensor.addObject('MechanicalObject', name="angles", template="Vec1d", position=[0,0,0])
    sensor.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    #########################################
    #description of the articulated chain from the chassis'position to the wheels and the sensors
    #########################################

    chain.addObject('ArticulatedHierarchyContainer')

    sensor.addObject('ArticulatedHierarchyContainer')

    for i in range(2):
        sc = sensor.addChild(sensorName[i])
        sc.addObject('ArticulationCenter', parentIndex=0, childIndex=1+i, posOnParent=sensorPositions[i])
        s = sc.addChild("Articulation")
        s.addObject('Articulation', translation=False, rotation=True, rotationAxis=[1, 0, 0], articulationIndex=i)

    for i in range(4):
        ac = chain.addChild("MotorToWheel{0}".format(i))
        ac.addObject('ArticulationCenter', parentIndex=0, childIndex=1+i, posOnParent=wheelPositions[i])
        a = ac.addChild("Articulation")
        a.addObject('Articulation', translation=False, rotation=True, rotationAxis=[1, 0, 0], articulationIndex=i)

    ##############################
    #There is one extra position in this mechanical object because there
    #the articulated chain Needs a root one (in addition to the four wheels)
    ##############################

    wheels = base.addChild("Wheels")
    chain.addChild(wheels)

    wheels.addObject("MechanicalObject", name="position", template="Rigid3d",
                          position=[[0,0,0,0,0,0,1], [0,0,0,0,0,0,1], [0,0,0,0,0,0,1], [0,0,0,0,0,0,1], [0,0,0,0,0,0,1]],
                          showObject=True)
    wheels.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    wheels.addObject('ArticulatedSystemMapping',
                          input1=chain.angles.getLinkPath(),
                          input2=base.position.getLinkPath(),
                          output=wheels.position.getLinkPath())

    #########################################
    #  add sensors
    #########################################

    sensors = base.addChild("Sensors")
    sensor.addChild(sensors)

    sensors.addObject("MechanicalObject", name = "position", template="Rigid3d",
                    position=[[0,0,0,0,0,0,1], [0,0,0,0,0,0,1], [0,0,0,0,0,0,1]],
                     showObject=True)
    sensors.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    sensors.addObject('ArticulatedSystemMapping',
                        input1=sensor.angles.getLinkPath(),
                        input2=base.position.getLinkPath(),
                        output=sensors.position.getLinkPath())

    #########################################
    # visual models
    #########################################

    ## Chassis's body
    visual = self.addChild("VisualModel")
    parts = {
        "Chassis" : ('meshes/summit_xl_chassis.stl', [0.1,0.1,0.1,1.0]) ,
        "ChassisCover" : ('meshes/summit_xl_covers.stl', [0.8,0.8,0.8,1.0]),
        "chassisSimple" : ('meshes/summit_xl_chassis_simple.stl', [0.5,0.5,0.5,1.0])
    }
    for name, (filepath, color) in parts.items():
        part = visual.addChild(name)
        part.addObject('MeshSTLLoader', name='loader', filename=filepath, rotation=[-90,-90,0],scale3d = [1000,1000,1000])
        part.addObject('MeshTopology', src='@loader')
        part.addObject('OglModel', name="renderer", src='@loader', color=color)
        part.addObject('RigidMapping', input=wheels.position.getLinkPath(), index=0)

    ## Wheels
    visual = wheels.addChild("VisualModel")
    visual.addObject('MeshSTLLoader', name='loader', filename='meshes/wheel.stl', rotation=[0,0,90],scale3d = [1000,1000,1000])
    visual.addObject('MeshTopology', name='geometry', src='@loader')
    for i in range(4):
        wheel = visual.addChild("Wheel{0}".format(i))
        wheel.addObject("OglModel", src=visual.geometry.getLinkPath(), color=[0.2,0.2,0.2,1.0])
        wheel.addObject("RigidMapping", input=wheels.position.getLinkPath(), index=i+1)

    ## Sensors
    visual = sensors.addChild("VisualModel")
    sensorfilepath = {
        "lazer" : ('meshes/hokuyo_urg_04lx.stl', 1) ,
        "gps" : ('meshes/antenna_3GO16.stl', 2)
    }


    for name, (filepath, index) in sensorfilepath.items():
        visual_body = visual.addChild(name)
        visual_body.addObject('MeshSTLLoader', name=name+'_loader', filename=filepath, rotation=[0,90,90],scale3d = [1000,1000,1000])
        visual_body.addObject('MeshTopology', src='@'+name+'_loader')
        visual_body.addObject('OglModel', name=name+"_renderer", src='@'+name+'_loader', color=[0.2,0.2,0.2,1.0])
        visual_body.addObject('RigidMapping', input=sensors.position.getLinkPath(),index=index)

    #########################################
    # collision models
    #########################################

    collison_model = wheels.addChild("CollisionModel")
    for i in range(4):
        wheel_collision = collison_model.addChild("WheelCollision{0}".format(i))
        wheel_collision.addObject('MeshSTLLoader', name='loader', filename='meshes/collision_wheel.stl', rotation=[0, 90, 0],scale3d = [1000,1000,1000])
        wheel_collision.addObject('MeshTopology', src='@loader')
        wheel_collision.addObject('MechanicalObject')
        wheel_collision.addObject('TriangleCollisionModel', group=0)
        wheel_collision.addObject('LineCollisionModel',group=0)
        wheel_collision.addObject('PointCollisionModel', group=0)
        wheel_collision.addObject('RigidMapping', input=wheels.position.getLinkPath(), index=i+1)

    #########################################
    # add Trunk
    #########################################

    # trunk = base.addChild("Trunk")
    # trunk.addObject("MechanicalObject", name = "position", template="Rigid3d",
    #                 position=trunkPosition,
    #                  showObject=True,showObjectScale = 30)
    # trunk.addObject('RigidRigidMapping',name='mapping', input=base.position.getLinkPath(), index=0)

    return self


def SummitXL(parentNode, name="SummitXL"):
    self = parentNode.addChild(name)
    self.addData(name="robot_linear_vel", value=[0.0, 0.0, 0.0],
                 type="Vec3d", help="Summit_xl velocity", group="Summitxl_cmd_vel")

    self.addData(name="robot_angular_vel", value=[0.0, 0.0, 0.0],
                 type="Vec3d", help="Summit_xl velocity", group="Summitxl_cmd_vel")

    self.addData(name="sim_orientation", value=[0., 0., 0., 0.],
                 type="Vec4d", help="Summit_xl imu", group="Summitxl_cmd_vel")

    self.addData(name="reel_orientation", value=[0., 0., 0., 0.],
                 type="Vec4d", help="Summit_xl imu", group="Summitxl_cmd_vel")

    self.addData(name="linear_acceleration", value=[0.0, 0.0, 0.0],
                 type="Vec3d", help="Summit_xl imu", group="Summitxl_cmd_vel")

    self.addData(name="timestamp",value=[0, 0], type="vector<int>", help="Summit_xl imu",
                 group="Summit_xl_cmd_vel")

    self.addData(name="sim_position",  value=[0.0, 0.0, 0.0],type="Vec3d",
                 help="Summit_xl odom", group="Summitxl_cmd_vel")

    self.addData(name="reel_position",  value=[0.0, 0.0, 0.0],type="Vec3d",
                 help="Summit_xl odom", group="Summitxl_cmd_vel")
    
    self.addData(name="digital_twin_joints_states_vel",value=[0, 0, 0, 0], type="Vec4d", help="Summit_xl joint_states",
                 group="Summit_xl_cmd_vel")
    
    self.addData(name="summit_xl_joints_states_vel",value=[0, 0, 0, 0], type="Vec4d", help="Summit_xl joint_states",
                 group="Summit_xl_cmd_vel")
    
    self.addData(name="effector_cable_data", type="float", help="trunk displacement", group="Echelon3")
    
    self.addData(name="end_effector_pos", value = [0,0,0,0,0,0,0],type="vector<float>", help="trunk displacement", group="Echelon3")
    
    
    self.addChild(Chassis())
    return self

def createScene(rootNode):

    #ContactHeader(rootNode, alarmDistance=0.2*1000, contactDistance=0.005*1000)

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
    scene.addContact(alarmDistance=0.2*1000, contactDistance=0.005*1000, frictionCoef=0.8)
    scene.VisualStyle.displayFlags = 'hideBehaviorModels showForceFields showCollisionModels showInteractionForceFields'
    scene.addObject('DefaultVisualManagerLoop')
    scene.dt = 0.01
    scene.gravity = [0., -9810., 0.]

    scene.Simulation.addObject('GenericConstraintCorrection', solverName='LinearSolver', ODESolverName='GenericConstraintSolver')

    scene.Simulation.TimeIntegrationSchema.vdamping.value = 0.1
    scene.Simulation.TimeIntegrationSchema.rayleighStiffness = 0.01
    
    #########################################
    # create summit
    #########################################
    scene.Simulation.addChild(scene.Modelling)
    SummitXL(scene.Modelling)
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

    scene.Modelling.SummitXL.addObject(SummitxlController(name="KeyboardController", robot=scene.Modelling.SummitXL, test=False))

    ########################################
    # createEchelon
    ######################################## 

    # arm = rootNode.Modelling.SummitXL.Chassis.addChild('Arm')
    # connection = rootNode.Modelling.SummitXL.Chassis.Base.Trunk.position
    # parameters, cables = createEchelon(arm,connection,0,[0., 0.26*1000, 0.32*1000],[-90,-90,0])

    # if typeControl == 'displacement':
    #     arm.addObject(CableController(cables, name= 'Cablecontroller'))
    # elif typeControl == 'force':
    #     arm.addObject(ForceController(cables, dt, name = 'ForceController'))
    return rootNode