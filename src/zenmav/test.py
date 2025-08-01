from zenmav.core import Zenmav

nav = Zenmav()  # Example threshold of 5 meters
nav.guided_arm_takeoff(height=20)  # Example height of 20 meters

target = nav.convert_to_global(local_pos=(10, 10))  # Example local position
nav.global_target(target.append(10))