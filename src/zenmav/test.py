
from core import Zenmav
from zenpoint import wp
import time
drone = Zenmav()



local_point = wp(10,20,-30)

drone.orbit(center = local_point, radius = 5, speed = 2)





