from zenfence import Fence
from core import Zenmav
import time
drone = Zenmav()


fence = Fence(drone, "fence_config.toml")

drone.set_mode("GUIDED")  # Set mode to GUIDED for fence checks
drone.arm()# check if inside fence

drone.takeoff(10)  # Take off to 10 meters
drone.local_target([90,0,-10])
input('Press Enter to continue')
drone.local_target([110,0,-10])
