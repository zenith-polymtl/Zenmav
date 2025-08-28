from core import Zenmav

drone = Zenmav()
drone.set_mode('GUIDED')
drone.arm()
drone.takeoff(5)

drone.local_target((20,0,-5), heading = 90)
drone.RTL()