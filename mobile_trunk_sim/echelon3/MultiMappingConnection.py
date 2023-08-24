
from parameters import *
from classEchelon import *

def createEchelon(echelon,target,parameters):


    #########################################
    # adding Points and topology
    ######################################### 

    framesNode = echelon
    framesNode.addObject('PointSetTopologyContainer', position=parameters.position);
    frames = framesNode.addObject('MechanicalObject', name='frames', template='Rigid3d',  showObject=1, showObjectScale=2, position=parameters.positionRigid)

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
    # Cables
    ######################################### 
    
    # creation of the cables
    
    constraintPoints  = framesNode.addChild('constraintPoints')
    constraintPoints.addObject('MechanicalObject', position=parameters.positionConstraint)
    test= constraintPoints.addObject('RigidMapping', rigidIndexPerPoint = parameters.rigidMapIndices, globalToLocalCoords=0) 
    test.init()

    if all_sections:
        a = len(parameters.constraintIndices)
    else :
        a = parameters.numCables[0]

    for c in range(a):
        if (inverse):
            cable = constraintPoints.addObject('CableActuator', name='Section'+str(c//3+1)+'Cable'+str(c%3+1), indices=parameters.constraintIndices[c],  hasPullPoint=0, maxForce=parameters.cables.forceMax, minForce=0,maxPositiveDisp=parameters.cables.stroke/2-parameters.cables.neutralPosition[c],maxNegativeDisp=parameters.cables.stroke/2+parameters.cables.neutralPosition[c])
        else:
            cable = constraintPoints.addObject('CableConstraint', name ='Section'+str(c//3+1)+'Cable'+str(c%3+1), indices=parameters.constraintIndices[c], hasPullPoint=0, valueType=typeControl, value=0)        

    ########################################
    # Effector constraint
    ######################################### 
        
    rotationParam=200; #150;
    direction=[ 1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,rotationParam,0,0,  0,0,0,0,rotationParam,0,  0,0,0,0,0, rotationParam]
    indice = parameters.numRibsTot+2*parameters.numRibs[0]

    if (inverse) :
        if all_sections :
            framesNode.addObject('PositionEffector', template='Rigid3', indices=parameters.backboneEdges[-1][-1], effectorGoal=target.position.linkpath, useDirections=[ 1, 1, 1, 0, 1, 1], directions=direction)
        else :
            framesNode.addObject('PositionEffector', template='Rigid3', indices=indice, effectorGoal=target.position.linkpath, useDirections=[ 1, 1, 1, 0, 1, 1], directions=direction)
      

    return echelon

    

def createScene(rootNode):

    #########################################
    # Plugins, data and Solvers
    ######################################### 

    rootNode.addObject('VisualStyle', displayFlags='hideBehaviorModels showForceFields showCollisionModels showInteractionForceFields');

    rootNode.findData('dt').value= dt;
    rootNode.findData('gravity').value= [0, 0, -9810];

    rootNode.addObject('BackgroundSetting', color=[ 0, 0.16, 0.21])

    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight");
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    rootNode.addObject('RequiredPlugin', name='BeamAdapter')
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    rootNode.addObject('RequiredPlugin', name='EigenLinearSolvers')
    rootNode.addObject('RequiredPlugin', name='SofaMeshCollision')
    rootNode.addObject('RequiredPlugin', name='SofaPlugins', pluginName='SofaGeneralEngine SofaConstraint SofaImplicitOdeSolver SofaSparseSolver SofaDeformable SofaEngine SofaBoundaryCondition SofaRigid SofaTopologyMapping SofaOpenglVisual SofaGeneralAnimationLoop SofaMiscMapping')
       
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')

    if (inverse):
        rootNode.addObject('QPInverseProblemSolver', name='QPSolver', printLog=0, epsilon=0.1, maxIterations=100, tolerance=1e-15)
    else:
        rootNode.addObject('GenericConstraintSolver', name='GSSolver', maxIterations=1000, tolerance=1e-15)
    
    
    #########################################
    # grab points
    # BLUE = consigne
    # RED = polhemus racking (optionnal)
    # GREEN = direct QP target
    #########################################    
    
    blue_offset = 60
    blue_ray = 10
    green_offset = 40
    green_ray = 10

    # GREEN = QP target
    qpTargetNode= rootNode.addChild('QPTarget')
    qpTarget = qpTargetNode.addObject('MechanicalObject', name='QPTarget', template='Rigid3d', position=sensor.start_pos, showObject=1)
        # grabpoints
    grabPointsTarget=qpTargetNode.addChild('grabPoints')
    grabPointsTarget.addObject('MechanicalObject', name='grabPoints', template='Vec3', position=[[ green_offset, 0, 0], [ 0, green_offset, 0], [ 0, 0, green_offset]])
    grabPointsTarget.addObject('SphereCollisionModel', radius=green_ray, group=3)
    grabPointsTarget.addObject('RigidMapping')

    # BLUE = consigne
    consigneNode= rootNode.addChild('Consigne')
    consigneNode.addObject('EulerImplicitSolver', firstOrder=1, rayleighStiffness=0.1)
    consigneNode.addObject('CGLinearSolver', name='solver', iterations=100, tolerance = 1e-05, threshold = 1e-05)
    consigne = consigneNode.addObject('MechanicalObject', name='Consigne', template='Rigid3d', position=sensor.start_pos, showObject=1)
    consigneNode.addObject('UncoupledConstraintCorrection')
        # grappoints
    grabPointsCons=consigneNode.addChild('grab points')
    polhemusVisual = grabPointsCons.addObject('MechanicalObject', name='PIGrabPoints', template='Vec3', position=[ [ 0, 0, 0], [ blue_offset, 0, 0], [ 0, blue_offset, 0], [ 0, 0, blue_offset]])
    grabPointsCons.addObject('SphereCollisionModel', radius=blue_ray, group=3)
    grabPointsCons.addObject('RigidMapping') 
    
    
    ########################################
    # Create parameters
    ########################################
    #     
    parameters = Echelon3([6,6,8],[12,11,7.375],[100,50.0*1.33, 35.0,35],[75,50.0,35, 35.0])
    # parameters = echelon3 = Echelon3([2],[5],[50.0*1.33, 35.0],[50.0, 35.0])
    # parameters = Echelon3([2,2],[8,8],[100.0,50.0*1.33, 35.0],[75.0,50.0, 35.0])

    ########################################
    # Create Solvers
    ########################################

    simulation = rootNode.addChild('Simulation')
    simulation.addObject('EulerImplicitSolver', rayleighStiffness=parameters.numerical.rayleighStiffness, rayleighMass=parameters.numerical.rayleighMass, vdamping=parameters.numerical.vdamping) 
    # simulation.addObject('EigenSimplicialLDLT',  name='ldl', template='CompressedRowSparseMatrixMat3x3d');
    simulation.addObject('SparseLDLSolver',  name='ldl', template='CompressedRowSparseMatrixMat3x3d');
    simulation.addObject('GenericConstraintCorrection' , solverName='ldl')
    
    ########################################
    # create Echelon
    ########################################

    echelonPoints = simulation.addChild('EchelonPoints')
    positionRigid = parameters.positionRigid[0:parameters.backboneEdges[0][0]]+parameters.positionRigid[parameters.backboneEdges[0][1]:]
    echelonPoints.addObject('MechanicalObject', name='frames', template='Rigid3d',  showObject=1, showObjectScale=2, position=positionRigid)

    #########################################
    # Base
    ######################################### 

    BaseNode=simulation.addChild('Base')
    BaseNode.addObject('MechanicalObject',name ='base', position=[0,0,0,0,0,0,1], template='Rigid3d')
    BaseNode.addObject('RestShapeSpringsForceField', points=0, stiffness=1e10 , angularStiffness=1e15)

    ########################################
    # Arm
    ########################################

    echelon = BaseNode.addChild('Echelon')
    echelonPoints.addChild(echelon)
    createEchelon(echelon,qpTarget,parameters)
    echelon.addObject('SubsetMultiMapping',template = 'Rigid3,Rigid3', input = [BaseNode.base.getLinkPath(),echelonPoints.frames.getLinkPath()], output =echelon.frames.getLinkPath() ,indexPairs = parameters.indexPairs)
    
    simulation.addObject('MechanicalMatrixMapper', template='Rigid3,Rigid3',object1 = BaseNode.base.getLinkPath(), object2 = echelonPoints.frames.getLinkPath(),nodeToParse = echelon.getLinkPath())


    return rootNode