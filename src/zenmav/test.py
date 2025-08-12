from core import Zenmav
from zenpoint import wp

drone = Zenmav('tcp:127.0.0.1:5762')
#Use 'udp:127.0.0.1:14550' for SITL in gazebo instead of tcp
#It works with tuples I tried it out just now but lists are more robust in the long haul because tuples are immutable
waypoints = [
        wp(10, 0, -10), 
        wp(10, 10, -10),  
        wp(0, 10, -10),   
        wp(0, 0, -10)    
    ]

for waypoint in waypoints:
    
    waypoint.frame = "local"
    drone.global_target(waypoint)
print("Mission complete")