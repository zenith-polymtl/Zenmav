from core import Zenmav

drone = Zenmav('tcp:127.0.0.1:5762', GCS = True)
drone.set_mode('GUIDED')
input('PRESS ENTER TO ')
drone.set_mode('LOITER')
while True:
    print('RUNNING')
    print(drone.get_global_pos())