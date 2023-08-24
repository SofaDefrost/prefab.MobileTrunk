from summit_xl import SummitXL
from echelon3.parameters import *
from echelon3.createEchelon import *
from addCamera import addCamera

def mobileTrunk(modellingNode):

    #########################################
    # create summit
    #########################################
    self = SummitXL(modellingNode)

    ########################################
    # createEchelon
    ######################################## 

    AttachedArm = self.Chassis.addChild("AttachedArm")

    AttachedArm.addObject("MechanicalObject", name = "position", template="Rigid3d",
                    position=[0., 0.26*1000, 0.32*1000,-0.5, -0.5, -0.5 , 0.5 ],
                     showObject=True,showObjectScale = 30)    
    AttachedArm.addObject('RigidRigidMapping',name='mapping', input=self.Chassis.Base.position.getLinkPath(),
                                                index=0)

    trunk = AttachedArm.addChild('Trunk')
    base_position = AttachedArm.position
    parameters, cables = createEchelon(trunk,base_position,0,[0., 0.26*1000, 0.32*1000],[-90,-90,0])

    ###############
    # TO DO : Réecrire correctement le keybord controller de la trompe
    #         en mettant le code commenté si dessous dans le fichier 
    #         approprié :  CableController
    ##############
    # if typeControl == 'displacement':
    #     trunk.addObject(CableController(cables, name = 'Cablecontroller'))
    # elif typeControl == 'force' :
    #     trunk.addObject(ForceController(cables,dt,name = 'ForceController'))
    
    ##########################################
    # add Camera
    ##########################################
    addCamera(AttachedArm)
    return trunk, self, cables


