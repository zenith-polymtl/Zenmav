from core import Zenmav
from zenpoint import wp

drone = Zenmav('tcp:127.0.0.1:5762')

drone.guided_arm_takeoff()

import time
start_time = time.time()
while time.time()-start_time<20:
    drone.speed_target((1,0,-1), yaw_rate=4)
drone.RTL()
