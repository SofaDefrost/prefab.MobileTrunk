from createEchelon import *

from parameters import *

from ForceController import *


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
    rootNode.addObject('RequiredPlugin', name='SofaPlugins', pluginName='SofaGeneralEngine SofaConstraint SofaImplicitOdeSolver SofaSparseSolver SofaDeformable SofaEngine SofaBoundaryCondition SofaRigid SofaTopologyMapping SofaOpenglVisual')
       
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')

    rootNode.addObject('GenericConstraintSolver', name='GSSolver', maxIterations=1000, tolerance=1e-15)
    
    #########################################
    # Base
    ######################################### 

    BaseNode=rootNode.addChild('Base')
    base = BaseNode.addObject('MechanicalObject',name ='base', position=[-100,0,0,0,0,0,1], template='Rigid3d')
    BaseNode.addObject('RestShapeSpringsForceField', points=0, stiffness=1e10 , angularStiffness=1e15)

    ########################################
    # create Echelon
    ########################################

    echelon = BaseNode.addChild('Echelon')
    parameters = createEchelon(echelon,base,[-100,0,0],[0,0,0])

    #########################################
    # Controllers
    #########################################
    
    if typeControl == 'force':
        echelon.addObject(ForceController(echelon.framesNode.constraintPoints,dt, name = 'Cablecontroller'))
    else :
        echelon.addObject(CableController(echelon.framesNode.constraintPoints, name = 'Cablecontroller'))

    return rootNode