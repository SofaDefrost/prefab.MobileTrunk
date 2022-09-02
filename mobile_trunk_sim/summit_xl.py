import Sofa

from stlib3.scene import Scene
from stlib3.scene import ContactHeader
from stlib3.physics.rigid import Floor

from summitxl_controller import *

from echelon3.parameters import *
from echelon3.createEchelon import *

def Chassis(scale):

    #########################################
    # parameters
    #########################################
    # totalMass = 1.0
    # volume = 1.0*1e9
    # inertiaMatrix = [1.0*1e6, 0.0, 0.0, 0.0, 1.0*1e6, 0.0, 0.0, 0.0, 1.0*1e6]
    wheelPositions = [[0.229*scale, 0,0.235*scale],
                      [-0.229*scale, 0,0.235*scale],
                      [0.229*scale, 0,-0.235*scale],
                      [-0.229*scale, 0,-0.235*scale]]

    sensorPositions = [[0., 0.28*scale ,0.],          # 2d lazer
                       [0,0.275*scale,-0.22*scale],         # gps
                       ]

    sensorName=["lazer", "gps"]

    trunkPosition = [0., 0.26*scale, 0.32*scale,-0.5, -0.5, -0.5 , 0.5 ]

    #########################################
    #Add chassis mechanical Object
    #########################################

    self = Sofa.Core.Node("Chassis")
    self.addObject("MechanicalObject", name="position", template="Rigid3d", position=[[0,0,0,0,0,0,1]])
    # self.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    #self.addObject('UncoupledConstraintCorrection')

    #########################################
    #creation of the articulated chain of wheels, sensors
    #########################################

    #wheels
    chain = self.addChild("WheelsMotors")
    chain.addObject('MechanicalObject', name="angles", template="Vec1d", position=[0,0,0,0,0])
    # chain.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    #capteur
    sensor =  self.addChild("FixedSensor")
    sensor.addObject('MechanicalObject', name="angles", template="Vec1d", position=[0,0,0])
    # sensor.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

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

    wheels = self.addChild("Wheels")

    wheels.addObject("MechanicalObject", name="position", template="Rigid3d",
                          position=[[0,0,0,0,0,0,1], [0,0,0,0,0,0,1], [0,0,0,0,0,0,1], [0,0,0,0,0,0,1], [0,0,0,0,0,0,1]],
                          showObject=True)
    # wheels.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    wheels.addObject('ArticulatedSystemMapping',
                          input1=chain.angles.getLinkPath(),
                          input2=self.position.getLinkPath(),
                          output=wheels.position.getLinkPath())

    #########################################
    #  add sensors
    #########################################

    sensors = self.addChild("Sensors")
    sensors.addObject("MechanicalObject", name = "position", template="Rigid3d",
                    position=[[0,0,0,0,0,0,1], [0,0,0,0,0,0,1], [0,0,0,0,0,0,1]],
                     showObject=True)
    # sensors.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    sensors.addObject('ArticulatedSystemMapping',
                        input1=sensor.angles.getLinkPath(),
                        input2=self.position.getLinkPath(),
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
        part.addObject('MeshSTLLoader', name='loader', filename=filepath, rotation=[-90,-90,0],scale3d = [scale,scale,scale])
        part.addObject('MeshTopology', src='@loader')
        part.addObject('OglModel', name="renderer", src='@loader', color=color)
        part.addObject('RigidMapping', input=self.Wheels.position.getLinkPath(), index=0)

    ## Wheels
    visual = wheels.addChild("VisualModel")
    visual.addObject('MeshSTLLoader', name='loader', filename='meshes/wheel.stl', rotation=[0,0,90],scale3d = [scale,scale,scale])
    visual.addObject('MeshTopology', name='geometry', src='@loader')
    for i in range(4):
        wheel = visual.addChild("Wheel{0}".format(i))
        wheel.addObject("OglModel", src=visual.geometry.getLinkPath(), color=[0.2,0.2,0.2,1.0])
        wheel.addObject("RigidMapping", input=self.Wheels.position.getLinkPath(), index=i+1)

    ## Sensors
    visual = sensors.addChild("VisualModel")
    sensorfilepath = {
        "lazer" : ('meshes/hokuyo_urg_04lx.stl', 1) ,
        "gps" : ('meshes/antenna_3GO16.stl', 2)
    }


    for name, (filepath, index) in sensorfilepath.items():
        visual_body = visual.addChild(name)
        visual_body.addObject('MeshSTLLoader', name=name+'_loader', filename=filepath, rotation=[0,90,90],scale3d = [scale,scale,scale])
        visual_body.addObject('MeshTopology', src='@'+name+'_loader')
        visual_body.addObject('OglModel', name=name+"_renderer", src='@'+name+'_loader', color=[0.2,0.2,0.2,1.0])
        visual_body.addObject('RigidMapping', input=self.Sensors.position.getLinkPath(),index=index)

    #########################################
    # collision models
    #########################################

    # collison_model = wheels.addChild("CollisionModel")
    # for i in range(4):
    #     wheel_collision = collison_model.addChild("WheelCollision{0}".format(i))
    #     wheel_collision.addObject('MeshSTLLoader', name='loader', filename='meshes/collision_wheel.stl', rotation=[0, 90, 0],scale3d = [1000,1000,1000])
    #     wheel_collision.addObject('MeshTopology', src='@loader')
    #     wheel_collision.addObject('MechanicalObject')
    #     wheel_collision.addObject('TriangleCollisionModel', group=0)
    #     wheel_collision.addObject('LineCollisionModel',group=0)
    #     wheel_collision.addObject('PointCollisionModel', group=0)
    #     wheel_collision.addObject('RigidMapping', input=self.Wheels.position.getLinkPath(), index=i+1)

    return self


def SummitXL(parentNode, scale=1, name="SummitXL"):
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
                 group="Summitxl_cmd_vel")

    self.addData(name="robot_scale",value=0, type="int", help="Summit_xl",
                group="Summitxl")

    self.addData(name="sim_position",  value=[0.0, 0.0, 0.0],type="Vec3d",
                 help="Summit_xl odom", group="Summitxl_cmd_vel")

    self.addData(name="reel_position",  value=[0.0, 0.0, 0.0],type="Vec3d",
                 help="Summit_xl odom", group="Summitxl_cmd_vel")

    self.addChild(Chassis(scale))
    return self


def createScene(rootNode):

    #########################################
    # Plugins, data and Solvers
    ######################################### 

    scene = Scene(rootNode)
    scene.addMainHeader()
    #scene.addContact(alarmDistance=0.2*1000, contactDistance=0.005*1000)
    scene.VisualStyle.displayFlags = 'hideBehaviorModels showForceFields showCollisionModels showInteractionForceFields'
    scene.addObject('DefaultVisualManagerLoop')
    scene.dt = 0.001
    scene.gravity = [0., -9810, 0.]

    #########################################
    # create summit
    #########################################

    SummitXL(scene.Modelling)
    floor = Floor(rootNode,
                  name="Floor",
                  translation=[-2, -0.12, -2],
                  uniformScale=0.1,
                  isAStaticObject=True)

    #def myAnimation(target, body, factor):
    #    body.position += [[0.0,0.0,0.001,0.0,0,0,1]]
    #    target.position = [[factor* 3.14 * 2]]*len(target.position.value)

    #animate(myAnimation, {
    #        "body" : scene.Modelling.SummitXL.Chassis.position,
    #        "target": scene.Modelling.SummitXL.Chassis.WheelsMotors.angles}, duration=2, mode="loop")

    scene.Modelling.SummitXL.addObject(SummitxlController(name="KeyboardController", robot=scene.Modelling.SummitXL, scale=1))
    scene.Simulation.addChild(scene.Modelling)

    return rootNode