from splib3.numerics import Quat

from dataclasses import dataclass, field
from math import sin,cos,pi
import numpy as np

@dataclass
class Numerical:
    """
    Numerical parameters of the scene
    """

    # Euler Implicit Solver parameters
    rayleighStiffness : float = 0.1 
    rayleighMass : float = 1
    vdamping : float = 0

    # num beam between ribs on the tube 
    num_beam_between_half_ribs : float = 1

    # num beam for each rib
    num_beam_on_rib : float = 1

    # Rate Limiter Parameters
    rise : list = field(default_factory=lambda :[100., 100., 100.])
    fall : list = field(default_factory=lambda :[-100., -100., -100.])

@dataclass
class Cables:
    """
    Mechanical parameters for the cables
    """

    stroke : float = 145
    forceMax : float = 100*1e6  #10 N (1N = 1kg*m/s^2 = 1e6 g*mm/s^2)
    # ribs parameters
    neutralPosition : list = field(default_factory=lambda :[-40,0,0,0, 0,0,0, 20,-10,-10])
    dist : list = field(default_factory=lambda :[5,3,2])

@dataclass
class Base:
    """
    Mechanical parameters for the Base, 
    The base (node 0) can move thanks to a sliding constraint
    """
    maxSpeed : int = 20 #(20mm/s  ?)
    maxDisplacement : int = 140 # 14 cm in positive and negative direction = 24 cm in total
    marginDisplacement : int = 10 # distance btwn end switch and minimal displacement

@dataclass
class Rib:

    x_list : list # mm position in x of the first and last points of the ribs
    height :  list # mm heigth corresponds to the length (in the radial direction) of the ribs 
    angle : float # rad

    width : float = 14.         # mm
    thickness : float = 1.5     # mm

    volumMass : float = 0.001    # 0.001g /mm^3
    youngModulus : float = 1e9 #Pa (1Pa = 1N/m^2 = 1kg/(m*s^2)  = 1g/(mm*s^2)

    nodes : list = field(default_factory=list)
    
    def __post_init__(self):
        dx = (self.x_list[1]-self.x_list[0])/2

        # describe with 3 points (two at the bottom and one on the top)
        self.nodes = [[self.x_list[0],self.height[1],self.angle],[self.x_list[0]+dx,self.height[0],self.angle],[self.x_list[1],self.height[1],self.angle]]
        
        self.position = [ [n[0], n[1]*sin(n[2]), n[1]*cos(n[2])] for n in self.nodes ]
        self.positionRigid =[ [ n[0], n[1]*sin(n[2]), n[1]*cos(n[2]), -sin(n[2]/2), 0, 0, cos(n[2]/2) ] for n in self.nodes ]

        self.createDof0TransformNode0([self.positionRigid[0][0],0,0,0,0,0,1],self.positionRigid[0])
        self.createDof1TransformNode1([self.positionRigid[-1][0],0,0,0,0,0,1],self.positionRigid[-1])

    def createDof0TransformNode0(self,backbonePose,ribPose):
               
        # dof from the backbone to the rib
        dof0_H_node0 =  [0,0,0,0,-sin(pi/4),0,cos(pi/4)]
        dof0_H_R0= self.inverseTransform(backbonePose)
        dof0_H_dof0 = self.composeTransform(dof0_H_R0, ribPose)
        dof0_H_node0 = self.composeTransform(dof0_H_dof0, dof0_H_node0)
        self.dof0TransformeNode0 = [dof0_H_node0,[0.0, 0.0, 0.0, -0.0, 0, 0, 1.0]]
        return dof0_H_node0 
        
    def createDof1TransformNode1(self,backbonePose,ribPose):

        dof1_H_node1 =  [0,0,0,0,sin(pi/4),0,cos(pi/4)]
        dof1_H_R0=self.inverseTransform(backbonePose)
        dof1_H_dof1 = self.composeTransform(dof1_H_R0, ribPose)
        dof1_H_node1 = self.composeTransform(dof1_H_dof1, dof1_H_node1)
        self.dof1TransformeNode1 = [[0.0, 0.0, 0.0, -0.0, 0, 0, 1.0],dof1_H_node1]
        return dof1_H_node1

    def inverseTransform(self,a_H_b):
        b_H_a = []
        a_H_b = list(np.float_(a_H_b))
        q = Quat([ a_H_b[3], a_H_b[4], a_H_b[5], a_H_b[6] ])
        b_H_a = b_H_a + list(q.rotate([ -a_H_b[0], -a_H_b[1],-a_H_b[2] ]))
        q = q.getConjugate()
        b_H_a = b_H_a + [q.take(0),q.take(1), q.take(2), q.take(3)]

        return b_H_a

    def composeTransform(self,a_H_b, b_H_c):
        a_H_c = []
        a_H_b, b_H_c = list(np.float_(a_H_b)), list(np.float_(b_H_c))
        q1, q2 = Quat([a_H_b[3], a_H_b[4], a_H_b[5],a_H_b[6]]), Quat([b_H_c[3], b_H_c[4], b_H_c[5],b_H_c[6]])
        b_c_in_a = q1.getConjugate().rotate([ b_H_c[0], b_H_c[1], b_H_c[2]])
        a_H_c = a_H_c + [ a_H_b[0] + b_c_in_a[0],  a_H_b[1] + b_c_in_a[1], a_H_b[2] + b_c_in_a[2] ]     
        q = Quat.product(q1,q2)
        a_H_c = a_H_c + [q.take(0),q.take(1),q.take(2),q.take(3)]

        return a_H_c

@dataclass
class Backbone:
    """
    
    """ 
    x_list : list # mm x position of the first and last points
    numStep : int # number of points

    external_radius : int = 15   # mm
    inner_radius : int = 14      # mm
    thickness_support : int = 3  # mm
    volume_mass : float  = 0.0005  # 0.001g /mm^3
    youngModulus : float = 1e7   #Pa (1Pa = 1N/m^2 = 1kg/(m*s^2)  = 1g/(mm*s^2)

    # create backbone points
    def __post_init__(self):

        dx = (self.x_list[1]-self.x_list[0])/self.numStep
        self.position = [[self.x_list[0]+i*dx,0,0] for i in range(self.numStep+1)]
        self.positionRigid = [[self.x_list[0]+i*dx,0,0,0,0,0,1] for i in range(self.numStep+1)]
        

@dataclass
class Section :
    """

    """
    x_list : list #mm      # first and last position on x axe
    heigth : list #mm      # heigth corresponds to the length (in the radial direction) of the first and last rib on the top part of the robot value linearly interpolated for each rib
    side : list   #mm      # side corresponds to the length (in the radial direction) of the first and last rib on the other part of the robot value linearly interpolated for each rib
    numRibs : int          # number of ribs in this sections 
    numCables : int        # number of cables in this sections 

    def __post_init__(self):

        # deduce parameter
        self.length = self.x_list[1]-self.x_list[0] #mm
        self.numRibsTot = self.numRibs*self.numCables
        self.angles = [ i*(2*pi/self.numCables) for i in range(self.numCables)] # in radian for each cable position

        # create backbone
        self.backbone = Backbone(self.x_list,self.numRibs*2)
        
        # creates ribs
        dh = (self.heigth[0]-self.heigth[1])/self.numRibs
        ds = (self.side[0]-self.side[1])/self.numRibs
        self.dx = (self.x_list[1] - self.x_list[0])/self.numRibs

        self.ribs = []
        for i in range(self.numRibs):
            self.ribs = self.ribs + [ Rib([self.x_list[0]+i*self.dx,self.x_list[0]+(i+1)*self.dx],[self.heigth[0]-i*dh,self.backbone.external_radius],self.angles[0])]
            for j in range(1,self.numCables):
                self.ribs = self.ribs + [ Rib([self.x_list[0]+i*self.dx,self.x_list[0]+(i+1)*self.dx],[self.side[0]-i*ds,self.backbone.external_radius],self.angles[j])]


        # get position
        self.positionRibs = [self.ribs[i].position[1] for i in range(self.numRibsTot)]
        self.positionRigidRibs = [self.ribs[i].positionRigid[1] for i in range(self.numRibsTot)]

        # get doftransform
        self.dof0TransformeNode0 = [self.ribs[i].dof0TransformeNode0 for i in range(self.numRibsTot)]
        self.dof1TransformeNode1 = [self.ribs[i].dof1TransformeNode1 for i in range(self.numRibsTot)]



@dataclass
class Echelon3:
    """
    Contains all the parameters for echelon3 and create the use points and transformation for the robot
    """
    numRibs :  list #  defines the number of ribs per section 
    steps : list # lentht of ONE step of the backbone. ribs MUST align to them

    ribsHeight : list = field(default_factory=lambda :[100.0*1.33, 50.0, 35.0,35]) # in mm #height corresponds to the length (in the radial direction) of the ribs placed on the upper part of the robot
    ribsSide : list = field(default_factory=lambda :[75.0, 50.0, 35.0, 35.0])  # in mm ribs side corresponds to the length (in the radial direction) of the ribs placed on the upper part of the robot

    step : float = 3.85 #  mm : lentht of ONE step of the backbone. ribs MUST align to them
    numCables : list = field(default_factory=lambda :[3,3,3]) # num of cables in each section

    cables : Cables = Cables()
    base : Base = Base()
    numerical : Numerical = Numerical()

    def __post_init__(self):
        
        # deduce parameters
        self.numSections = len(self.numRibs)
        self.x_list = [0] # beginning and end of each section
        self.numRibsTot = sum([self.numRibs[i]*self.numCables[i] for i in range(self.numSections)])
        
        for i in range(self.numSections) :
            self.x_list = self.x_list + [self.x_list[i] + self.numRibs[i]*self.step*self.steps[i] ]

        # create sections
        self.sections = [Section([self.x_list[i],self.x_list[i+1]],[self.ribsHeight[i],self.ribsHeight[i+1]],[self.ribsSide[i],self.ribsSide[i+1]],self.numRibs[i],self.numCables[i]) for i in range(self.numSections)]

        # get positions
        positionRibs,positionRigidRibs,positionBackbone,positionRigidBackbone = [],[],[],[]
        for i in range(self.numSections):
            positionRibs.extend(self.sections[i].positionRibs)
            positionRigidRibs.extend(self.sections[i].positionRigidRibs)
            positionBackbone.extend(self.sections[i].backbone.position[int(i>0):])
            positionRigidBackbone.extend(self.sections[i].backbone.positionRigid[int(i>0):])
        
        self.position = positionRibs + positionBackbone 
        self.positionRigid = positionRigidRibs + positionRigidBackbone
        
        # get dof trasnform
        self.dof0TransformeNode0 = [self.sections[i].dof0TransformeNode0 for i in range(self.numSections)]
        self.dof1TransformeNode1 = [self.sections[i].dof1TransformeNode1 for i in range(self.numSections)]

        # create Edges
        self.backboneEdges = [[self.numRibsTot+i,self.numRibsTot+i+1] for i in range(2*sum(self.numRibs))]
        self.ribsEdges = []
        rib,ribpass = 0,0
        for s in range(self.numSections):
            for r in range(self.numRibs[s]):
                for c in range(self.numCables[s]):
                    self.ribsEdges.extend([2*ribpass+self.numRibsTot,rib,rib,2*(ribpass+1)+self.numRibsTot])
                    rib+=1
                ribpass +=1


        # add actuator 
        # rigidMapIndices: for each constraint pos, provides the indice of the input NEW node 
        self.rigidMapIndices = [self.backboneEdges[0][0] for i in range(self.numCables[0]+1)]
        for i in range(self.numRibsTot):
            self.rigidMapIndices.append(i)

        #posConstraint: position of the constraint point
        self.positionConstraint = [[0,0,0],[0, 0,  self.ribsHeight[0]+ self.cables.dist[0] ] ]
        
        # position of the constraint were we place Cable Attachement on the First Rib 
        for i in range(1,self.numCables[0]):
            self.positionConstraint.append([ 0, (self.ribsSide[0]+self.cables.dist[0])*sin(i*2*pi/self.numCables[0]), (self.ribsSide[0]+self.cables.dist[0])*cos(i*2*pi/self.numCables[0])])
        # these constraint are attached to node 0 (as the 3 nodes attached to the ground were fixed, see below)

        it_ribs=0;
        it_point=self.numCables[0]

        self.constraintIndices = [ ]
        for s in range(self.numSections):
            # n cables created for each section, we need to store the indices of the constraint point
            listOfIndices= [ [it_point-j] for j in range(self.numCables[s]-1,-1,-1)]

            for r in range(self.numRibs[s]):    #iterates on num_ribs
                it_ribs = it_ribs +1;
                it_point = it_point+self.numCables[s]
                for i in range(self.numCables[s]):
                    self.positionConstraint = self.positionConstraint + [[0, 0, self.cables.dist[s]]]
                    listOfIndices[i] = listOfIndices[i] + [it_point-(self.numCables[s]-i-1)]

            self.constraintIndices = self.constraintIndices + listOfIndices

        # create indexPairs
        self.indexPairs = []
        for i in range(self.numRibsTot):
            self.indexPairs.extend([1,i])
        self.indexPairs.extend([0,0])
        for edges in self.backboneEdges[1:]:
            self.indexPairs.extend([1,edges[0]-1])
        self.indexPairs.extend([1,self.backboneEdges[-1][-1]-1])
        


        

# echelon3 = Echelon3([6,6,8,7],[25,11,7,6],[200,100,50.0*1.33, 35.0,35],[100,75,50.0,35, 35.0])

# echelon3 = Echelon3([6,6,8],[12,11,7],[100,50.0*1.33, 35.0,35],[75,50.0,35, 35.0])
# echelon3 = Echelon3([2,2],[8,8],[100.0,50.0*1.33, 35.0],[75.0,50.0, 35.0])
# echelon3 = Echelon3([2],[5],[50.0*1.33, 35.0],[50.0, 35.0])

 # echelon3.position[numSection][0 : ribs, 1 : backbone][numRibs][point rib]
 # echelon3.position[].ribs[numribs][0 : first point (for first rib or middle point for other), 1: middle of last point]
