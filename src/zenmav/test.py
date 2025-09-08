
from core import Zenmav
import time
drone = Zenmav()

drone.set_mode('GUIDED')
input('WAIT')
drone.takeoff(10, 5)

drone.local_target((10,20,-30), wait_to_reach=False)

print('GOING TO POINT AND SHUTTING DOWN')


