from core import Zenmav

drone = Zenmav(boundary_path="global_fence.toml")
drone.set_mode("GUIDED")  # Set mode to GUIDED for fence checks
drone.arm()
drone.takeoff(20)  # Take off to 20 meters
drone.local_target([4000,0,-20])  # Move to a point outside the fence
