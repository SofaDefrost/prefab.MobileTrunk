from .parameters import *
from .classEchelon import *

if not inverse :
    from .CableController import *
    from .ForceController import *


def createEchelon(echelon,base,index,translation,rotation):
    """
    Creates topology and cables for echelon3 arm
    Parameters :
        echelon : Sofa child on which we add everything
        base : mechanical object on which we attach the arm
        index : index of the mechanical point we attach the arm
        translation : from 0,0,0 where we creat the arm
        rotation : from 0,0,0 where we creat the arm
    """
    
    parameters = Echelon3([6,6,8],[12,11,7.375],[100,50.0*1.33, 35.0,35],[75,50.0,35, 35.0])

    position =  echelon.addObject('TransformEngine',name="transform" ,template="Vec3d" ,translation=translation ,rotation=rotation ,input_position=parameters.position)
    positionRigid =  echelon.addObject('TransformEngine',name="transformRigid" ,template="Rigid3d" ,translation=translation ,rotation=rotation ,input_position=parameters.positionRigid)
    # position =  echelon.addObject('TransformEngine',name="transform" ,template="Vec3d" ,translation=[0,0,0] ,rotation=[45,0,0] ,input_position=position.output_position)
    # positionRigid =  echelon.addObject('TransformEngine',name="transformRigid" ,template="Rigid3d" ,translation=[0,0,0] ,rotation=[45,0,0] ,input_position=positionRigid.output_position)
    position.init()
    positionRigid.init()

    #########################################
    # adding Points and topology
    ######################################### 

    framesNode = echelon.addChild('framesNode')

    framesNode.addObject('PointSetTopologyContainer', position=position.output_position);
    frames = framesNode.addObject('MechanicalObject', name='frames', template='Rigid3d',  showObject=1, showObjectScale=2, position=positionRigid.output_position)

    topoTubeNode=framesNode.addChild('topoTube')
    topoTubeNode.addObject('EdgeSetTopologyContainer', edges=parameters.backboneEdges, name='edgeContainer')
    topoTubeNode.addObject('EdgeSetTopologyModifier', name='edgeModif')
    topoTubeNode.addObject('BeamInterpolation', name='TubeInterpolation', radius=parameters.sections[0].backbone.external_radius, innerRadius=parameters.sections[0].backbone.inner_radius, straight=1, defaultYoungModulus=parameters.sections[0].backbone.youngModulus, defaultPoissonRatio = 0.3)
    topoTubeNode.addObject('AdaptiveBeamForceFieldAndMass', massDensity=parameters.sections[0].backbone.volume_mass)
   
    topoRibNode=framesNode.addChild('topoRibs')
    topoRibNode.addObject('EdgeSetTopologyContainer',  edges=parameters.ribsEdges,name='edgecont')
    topoRibNode.addObject('BeamInterpolation',name = 'RibsInterpolation', crossSectionShape='rectangular', lengthY=parameters.sections[0].ribs[0].width, lengthZ=parameters.sections[0].ribs[0].thickness, straight=0, defaultYoungModulus=parameters.sections[0].ribs[0].youngModulus, DOF0TransformNode0= parameters.dof0TransformeNode0, DOF1TransformNode1 = parameters.dof1TransformeNode1, dofsAndBeamsAligned=0)
    topoRibNode.addObject('AdaptiveBeamForceFieldAndMass', massDensity=parameters.sections[0].ribs[0].volumMass)

    #########################################
    # Spring to connect the arm to a point
    ######################################### 
    
    framesNode.addObject('RestShapeSpringsForceField', points=parameters.backboneEdges[0][0], external_points = index,external_rest_shape =base.getLinkPath() ,stiffness=1e15 , angularStiffness=1e20)

    #########################################
    # Cables
    ######################################### 
    
    # creation of the cables
    
    constraintPoints  = framesNode.addChild('constraintPoints')
    constraintPoints.addObject('MechanicalObject', position=parameters.positionConstraint)
    test= constraintPoints.addObject('RigidMapping', rigidIndexPerPoint = parameters.rigidMapIndices, globalToLocalCoords=0) 
    test.init()

    cables = []
    for c in range(len(parameters.constraintIndices)):
        cable = constraintPoints.addObject('CableConstraint', name ='Section'+str(c//3+1)+'Cable'+str(c%3+1), indices=parameters.constraintIndices[c], hasPullPoint=0, valueType=typeControl, value=0)        
        cables.append(cable)
    
    ###########################################
    # #  Mechanical model for colision
    # ##########################################
    trunk = framesNode.addChild("Trunk")

    trunk.addObject("MechanicalObject", name="position", template="Vec3d",
                          position=position.output_position, showObject=True)
    trunk.addObject('UniformMass', totalMass=0.032)

    # trunk.addObject('TetrahedronHyperelasticityFEMForceField',
    #                       name="linearElasticBehavior",
    #                       youngModulus=250,
    #                       poissonRatio=0.45)

    ###########################################
    # # Visual model for colision
    # ##########################################
    # visual = trunk.addChild("VisualModel")
    # visual.addObject('MeshSTLLoader', name='loader', filename='meshes/model_colision.stl',scale3d = [1000,1000,1000], rotation=[90,90,180],
    #                                  translation=[0., 0.26*1000, 0.65*1000])
                        
    # visual.addObject('MeshTopology', src='@loader')
    # visual.addObject('OglModel', name="renderer", src='@loader', color=[0.2,0.2,0.2,1.0])
    # visual.addObject('SkinningMapping' , template="Rigid3d,Vec3d", input=frames.getLinkPath())

    # ##########################################
    # # Colision model
    # ##########################################

    # collison_model = trunk.addChild("CollisionModel")
    # collison_model.addObject('MeshSTLLoader', name='loader1', filename='meshes/model_colision.stl', scale3d=[1000, 1000, 1000],rotation=[90,90,180],
    #                                          translation=[0., 0.26*1000, 0.65*1000])
    # collison_model.addObject('MeshTopology', src='@loader1')
    # collison_model.addObject('MechanicalObject', src = '@loader1')
    # collison_model.addObject('TriangleCollisionModel', group=0)
    # collison_model.addObject('LineCollisionModel',group=0)
    # collison_model.addObject('PointCollisionModel', group=0)
    # collison_model.addObject('SkinningMapping' , template="Rigid3d,Vec3d", input=frames.getLinkPath())

    return parameters,cables