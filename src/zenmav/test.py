from core import Zenmav
import time
while True:
    drone = Zenmav(GCS=True)
    drone.set_mode('GUIDED')
    time.sleep(0.5)
    drone.set_mode('RTL')
    drone.connection.close()
    time.sleep(2)