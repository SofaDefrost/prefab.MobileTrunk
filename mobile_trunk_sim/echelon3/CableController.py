import Sofa
import math

class CableController(Sofa.Core.Controller):   
    """
    Controller to control the cables in forward kinematics
    Parameters :
        cables : list of the cables (constraint object)
        name : the name of the component 
    """

    def __init__(self, cables : list, name : str = "CableController"):
        Sofa.Core.Controller.__init__(self, name = name)
                
        self.cables = cables     
        self.index =0
    # action at beginning frame
    def onAnimateBeginEvent(self, eventType):
            pass
  
    #action when key press
    def onKeypressedEvent(self, c):
        key = c['key']    
        displacement = [math.inf] 

        ##### cables number ######
        if (key == "1"):
            self.index = 0
        elif (key == "2"):
            self.index = 1
        elif (key == "3"):
            self.index = 2
        elif (key == "4"): 
            self.index = 3
        elif (key == "5"):
            self.index = 4
        elif (key == "6"):
            self.index = 5
        elif (key == "7"): 
            self.index = 6
        elif (key == "8"):
            self.index = 7
        elif (key == "9"):
            self.index = 8

        ##### add or substract ######
        if key == '+':
            displacement = self.cables[self.index].value.value+1
            self.cables[self.index].value.value = [displacement]
            print(f'Cables {self.index} displacement = {displacement[0]}')

        elif key == '-':
            displacement = self.cables[self.index].value.value-1
            self.cables[self.index].value.value = [displacement]
            print(f'Cables {self.index} displacement = {displacement[0]}')
