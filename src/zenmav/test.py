
from core import Zenmav
import time
drone = Zenmav(gps_thresh=0.5)
drone.guided_arm_takeoff(100)



drone.set_mode('ACRO')
finisheed = False
start_time = time.time()
if time.time()- start_time < 3:
    drone.rc_override({'ch3': 1600})
elif not finisheed:
    drone.set_mode('GUIDED')
    finished = True
