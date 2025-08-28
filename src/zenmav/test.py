from core import Zenmav

drone = Zenmav()
drone.set_mode('GUIDED')



drone.yaw_target(-45)
input('END')

drone.yaw_target(45, relative=True)