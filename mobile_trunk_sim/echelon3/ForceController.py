import Sofa


class ForceController(Sofa.Core.Controller):   
    """
    Controller to control the cables in forward kinematics
    Parameters :
        mechanicalObject : the mechanicalObject to control
        dt : step time as in Sofa Force(Sofa)  = Force(N) *dt 
        name : the name of the component 
    """

    def __init__(self, cables : list, dt : float, name : str = "CableController"):
        Sofa.Core.Controller.__init__(self, name = name)
                
        self.cables = cables     
        self.forceMax = 1e20 # max 6kg
        self.forceMin = 0
        self.dt = dt

    # action at beginning frame
    def onAnimateBeginEvent(self, eventType):
            pass
  
    #action when key press
    def onKeypressedEvent(self, c):
        key = c['key']        

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
            force = self.cables[self.index].value.value+981000*self.dt # F = g.mm.s-2 ==> 100g : 100*9810 =  g.mm.s-2
            if force[0] <= self.forceMax :
                self.cables[self.index].value.value = [force]
            print(f'Cables {self.index} force = {force[0]*1e-6}')

        elif key == '-':
            force = self.cables[self.index].value.value-981000*self.dt # F = g.mm.s-2 ==> 100g : 100*9810 =  g.mm.s-2
            if force[0] >= -self.forceMin :
                self.cables[self.index].value.value = [force]
            print(f'Cables {self.index} force = {force[0]*1e-6}')
    