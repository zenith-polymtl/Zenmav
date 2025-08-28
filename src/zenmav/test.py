from core import Zenmav
import time

drone = Zenmav()
drone.set_mode('GUIDED')
drone.download_all_params()
input('FINISEHD')


