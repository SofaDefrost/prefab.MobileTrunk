from .classEchelon import *

all_sections = True

# time step
dt = 0.01

#####################
### type of simulation
#####################

inverse = False			# inverse model
base_translation = True
rate_limiter = True     # limit the movement of the target 
typeControl = 'displacement'

ros = False

########### Sensor #############

@dataclass
class Sensor:
    
    position : float
    offset : float
    axe : int = 0

    def __post_init__(self):
        self.start_pos = [0,0,0, 0,0,0,1]
        self.start_pos[self.axe] = self.position-self.offset

# create sensor
if all_sections :
    sensor = Sensor(760,50,0)
else : 
    sensor = Sensor(260,10,0)