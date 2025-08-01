"""
Gimbal Control Example Script

This script demonstrates how to use the gimbal control functionality
in the Zenmav library.

Example usage:
    python gimbal_example.py
"""

from zenmav.core import Zenmav
import time

def main():
    """Main function demonstrating gimbal control"""
    
    # Initialize drone connection
    print("Initializing drone connection...")
    drone = Zenmav()
    
    try:
        # Arm the drone
        print("Arming motors...")
        drone.arm()
        
        # Set mode to GUIDED
        print("Setting mode to GUIDED...")
        drone.set_mode("GUIDED")
        
        # Takeoff to 10 meters
        print("Taking off to 10 meters...")
        drone.takeoff(altitude=10)
        
        # Wait a moment for stabilization
        time.sleep(2)
        
        # Point gimbal down
        print("Pointing gimbal down...")
        drone.gimbal_point_down()
        time.sleep(2)
        
        # Point gimbal forward
        print("Pointing gimbal forward...")
        drone.gimbal_point_forward()
        time.sleep(2)
        
        # Set specific angles
        print("Setting custom gimbal angles...")
        drone.gimbal_set_angle(pitch=45, yaw=90)
        time.sleep(2)
        
        # Point at specific location (example coordinates)
        print("Pointing gimbal at specific location...")
        # Note: Replace with actual coordinates for your location
        drone.gimbal_point_at_location(lat=37.7749, lon=-122.4194, alt=100)
        time.sleep(2)
        
        # Retract gimbal
        print("Retracting gimbal...")
        drone.gimbal_retract()
        
        # Return to launch
        print("Returning to launch...")
        drone.RTL()
        
    except Exception as e:
        print(f"Error occurred: {e}")
        # Try to return to launch in case of error
        try:
            drone.RTL()
        except:
            pass

if __name__ == "__main__":
    main()