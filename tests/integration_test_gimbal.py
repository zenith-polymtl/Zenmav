"""
Integration test script for Zenmav gimbal functionality

This script demonstrates and tests the complete gimbal control functionality
in the Zenmav library with a simulated drone connection.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from zenmav.core import Zenmav
from zenmav.gimbal import GimbalController, GimbalError, GimbalParameterError
import time

def mock_connection():
    """Create a mock connection for testing"""
    class MockConnection:
        def __init__(self):
            self.target_system = 1
            self.target_component = 1
            self.mav = MockMAV()
    
    class MockMAV:
        def command_long_send(self, *args, **kwargs):
            print(f"Mock command_long_send called with args: {args}")
            return True
            
        def command_int_send(self, *args, **kwargs):
            print(f"Mock command_int_send called with args: {args}")
            return True
    
    return MockConnection()

def test_gimbal_integration():
    """Test complete gimbal integration"""
    print("Testing Zenmav Gimbal Integration")
    print("=" * 40)
    
    # Create mock connection
    mock_conn = mock_connection()
    
    # Create Zenmav instance with mock connection
    print("Creating Zenmav instance...")
    drone = Zenmav.__new__(Zenmav)
    drone.connection = mock_conn
    
    # Initialize gimbal controller
    print("Initializing gimbal controller...")
    drone.gimbal = GimbalController(drone)
    
    try:
        # Test gimbal mode configuration
        print("\n1. Testing gimbal mode configuration...")
        drone.gimbal.retract()
        drone.gimbal.neutral()
        drone.gimbal.mavlink_targeting()
        print("   ✓ Gimbal mode configuration successful")
        
        # Test angle setting
        print("\n2. Testing gimbal angle setting...")
        drone.gimbal.set_angle(pitch=45, yaw=90)
        drone.gimbal.point_down()
        drone.gimbal.point_forward()
        print("   ✓ Gimbal angle setting successful")
        
        # Test ROI pointing
        print("\n3. Testing ROI pointing...")
        drone.gimbal.point_at_location(lat=37.7749, lon=-122.4194, alt=100)
        print("   ✓ ROI pointing successful")
        
        # Test convenience methods
        print("\n4. Testing convenience methods...")
        drone.gimbal_point_down()
        drone.gimbal_point_forward()
        drone.gimbal_retract()
        drone.gimbal_neutral()
        drone.gimbal_mavlink_targeting()
        drone.gimbal_point_at_location(lat=37.7749, lon=-122.4194, alt=100)
        print("   ✓ Convenience methods successful")
        
        # Test parameter methods
        print("\n5. Testing parameter methods...")
        # Mock the get_param and set_param methods
        drone.get_param = lambda x: 45.0
        drone.set_param = lambda x, y: True
        drone.gimbal.get_gimbal_param("PITCH_MIN")
        drone.gimbal.set_gimbal_param("PITCH_MIN", 30.0)
        print("   ✓ Parameter methods successful")
        
        # Test status retrieval
        print("\n6. Testing status retrieval...")
        status = drone.gimbal.get_status()
        print(f"   Status: {status}")
        print("   ✓ Status retrieval successful")
        
        print("\n" + "=" * 40)
        print("All integration tests passed!")
        print("=" * 40)
        
    except Exception as e:
        print(f"\n❌ Integration test failed: {e}")
        return False
    
    return True

def test_error_handling():
    """Test error handling functionality"""
    print("\nTesting Error Handling")
    print("=" * 40)
    
    # Create mock connection
    mock_conn = mock_connection()
    
    # Create Zenmav instance with mock connection
    drone = Zenmav.__new__(Zenmav)
    drone.connection = mock_conn
    drone.gimbal = GimbalController(drone)
    
    try:
        # Test parameter validation errors
        print("1. Testing parameter validation errors...")
        try:
            drone.gimbal.set_angle(pitch=190)  # Invalid pitch
            print("   ❌ Should have raised GimbalParameterError")
            return False
        except GimbalParameterError:
            print("   ✓ Parameter validation error handled correctly")
        
        try:
            drone.gimbal.point_at_location(lat=91, lon=0, alt=0)  # Invalid latitude
            print("   ❌ Should have raised GimbalParameterError")
            return False
        except GimbalParameterError:
            print("   ✓ Coordinate validation error handled correctly")
        
        # Test parameter name validation
        print("\n2. Testing parameter name validation...")
        try:
            drone.gimbal.get_gimbal_param("")  # Empty parameter name
            print("   ❌ Should have raised GimbalParameterError")
            return False
        except GimbalParameterError:
            print("   ✓ Parameter name validation error handled correctly")
        
        try:
            drone.gimbal.set_gimbal_param(None, 30.0)  # None parameter name
            print("   ❌ Should have raised GimbalParameterError")
            return False
        except GimbalParameterError:
            print("   ✓ Parameter name validation error handled correctly")
        
        print("\n" + "=" * 40)
        print("All error handling tests passed!")
        print("=" * 40)
        
    except Exception as e:
        print(f"\n❌ Error handling test failed: {e}")
        return False
    
    return True

def test_complete_workflow():
    """Test a complete workflow similar to real usage"""
    print("\nTesting Complete Workflow")
    print("=" * 40)
    
    try:
        # This would normally create a real connection, but we'll mock it
        print("Creating Zenmav instance...")
        drone = Zenmav.__new__(Zenmav)
        drone.connection = mock_connection()
        drone.gimbal = GimbalController(drone)
        
        # Mock required methods
        drone.get_param = lambda x: 45.0
        drone.set_param = lambda x, y: True
        drone.arm = lambda: print("   Drone armed")
        drone.set_mode = lambda x: print(f"   Mode set to {x}")
        drone.takeoff = lambda altitude: print(f"   Taking off to {altitude}m")
        drone.RTL = lambda: print("   Returning to launch")
        
        print("\nWorkflow Steps:")
        print("1. Initializing drone...")
        print("   ✓ Drone initialized")
        
        print("\n2. Arming drone...")
        drone.arm()
        
        print("\n3. Setting mode to GUIDED...")
        drone.set_mode("GUIDED")
        
        print("\n4. Taking off...")
        drone.takeoff(altitude=20)
        
        print("\n5. Pointing gimbal down for inspection...")
        drone.gimbal_point_down()
        time.sleep(0.1)  # Simulate delay
        
        print("\n6. Setting custom gimbal angles...")
        drone.gimbal_set_angle(pitch=45, yaw=90)
        time.sleep(0.1)  # Simulate delay
        
        print("\n7. Pointing at specific location...")
        drone.gimbal_point_at_location(lat=37.7749, lon=-122.4194, alt=100)
        time.sleep(0.1)  # Simulate delay
        
        print("\n8. Retracting gimbal...")
        drone.gimbal_retract()
        
        print("\n9. Returning to launch...")
        drone.RTL()
        
        print("\n" + "=" * 40)
        print("Complete workflow test passed!")
        print("=" * 40)
        
    except Exception as e:
        print(f"\n❌ Complete workflow test failed: {e}")
        return False
    
    return True

def main():
    """Run all integration tests"""
    print("Zenmav Gimbal Integration Tests")
    print("=" * 50)
    
    # Run tests
    tests = [
        ("Gimbal Integration", test_gimbal_integration),
        ("Error Handling", test_error_handling),
        ("Complete Workflow", test_complete_workflow)
    ]
    
    results = []
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"Test {test_name} failed with exception: {e}")
            results.append((test_name, False))
    
    # Print summary
    print("\n" + "=" * 50)
    print("TEST SUMMARY")
    print("=" * 50)
    
    passed = 0
    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        print(f"{test_name}: {status}")
        if result:
            passed += 1
    
    print(f"\nPassed: {passed}/{len(results)} tests")
    
    if passed == len(results):
        print("🎉 All tests passed!")
        return True
    else:
        print("❌ Some tests failed!")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)