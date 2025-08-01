"""
Test script for Zenmav gimbal functionality

This script tests the gimbal control functionality in the Zenmav library.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from zenmav.core import Zenmav
from zenmav.gimbal import GimbalController, GimbalError, GimbalParameterError
import pytest
from unittest.mock import Mock, patch

def test_gimbal_controller_initialization():
    """Test gimbal controller initialization"""
    # Mock drone object
    mock_drone = Mock()
    mock_drone.connection = Mock()
    
    # Create gimbal controller
    gimbal = GimbalController(mock_drone)
    
    # Verify initialization
    assert gimbal.drone == mock_drone
    assert gimbal.current_mode is None
    assert gimbal.current_angles == {"pitch": 0, "yaw": 0, "roll": 0}

def test_gimbal_mode_constants():
    """Test gimbal mode constants"""
    assert GimbalController.MODE_RETRACT == 0
    assert GimbalController.MODE_NEUTRAL == 1
    assert GimbalController.MODE_MAVLINK_TARGETING == 2
    assert GimbalController.MODE_RC_TARGETING == 3
    assert GimbalController.MODE_GPS_POINT == 4
    assert GimbalController.MODE_SYSID_TARGET == 5
    assert GimbalController.MODE_HOME_LOCATION == 6

def test_gimbal_yaw_flags():
    """Test gimbal yaw flags"""
    assert GimbalController.YAW_FOLLOW == 0
    assert GimbalController.YAW_LOCK == 16

def test_gimbal_parameter_validation():
    """Test gimbal parameter validation"""
    mock_drone = Mock()
    mock_drone.connection = Mock()
    gimbal = GimbalController(mock_drone)
    
    # Test valid pitch values
    try:
        gimbal.set_angle(pitch=90)  # Should be valid
        gimbal.set_angle(pitch=-90)  # Should be valid
    except GimbalParameterError:
        pytest.fail("Valid pitch values should not raise GimbalParameterError")
    
    # Test invalid pitch values
    with pytest.raises(GimbalParameterError):
        gimbal.set_angle(pitch=190)  # Should be invalid
    
    with pytest.raises(GimbalParameterError):
        gimbal.set_angle(pitch=-190)  # Should be invalid

def test_gimbal_convenience_methods():
    """Test gimbal convenience methods"""
    # Mock drone object
    mock_drone = Mock()
    mock_drone.connection = Mock()
    
    # Create Zenmav instance with mock gimbal controller
    drone = Zenmav.__new__(Zenmav)
    drone.gimbal = GimbalController.__new__(GimbalController)
    drone.gimbal.retract = Mock()
    drone.gimbal.neutral = Mock()
    drone.gimbal.point_down = Mock()
    drone.gimbal.point_forward = Mock()
    
    # Test convenience methods
    drone.gimbal_retract()
    drone.gimbal.retract.assert_called_once()
    
    drone.gimbal_neutral()
    drone.gimbal.neutral.assert_called_once()
    
    drone.gimbal_point_down()
    drone.gimbal.point_down.assert_called_once()
    
    drone.gimbal_point_forward()
    drone.gimbal.point_forward.assert_called_once()

def test_gimbal_angle_setting():
    """Test gimbal angle setting"""
    mock_drone = Mock()
    mock_drone.connection = Mock()
    mock_drone.connection.mav = Mock()
    mock_drone.connection.target_system = 1
    mock_drone.connection.target_component = 1
    
    gimbal = GimbalController(mock_drone)
    
    # Test setting angles
    result = gimbal.set_angle(pitch=45, yaw=90)
    assert result is True
    mock_drone.connection.mav.command_long_send.assert_called_once()

def test_gimbal_point_down():
    """Test gimbal point down method"""
    mock_drone = Mock()
    mock_drone.connection = Mock()
    mock_drone.connection.mav = Mock()
    mock_drone.connection.target_system = 1
    mock_drone.connection.target_component = 1
    
    gimbal = GimbalController(mock_drone)
    
    # Test point down
    result = gimbal.point_down()
    assert result is True

def test_gimbal_point_forward():
    """Test gimbal point forward method"""
    mock_drone = Mock()
    mock_drone.connection = Mock()
    mock_drone.connection.mav = Mock()
    mock_drone.connection.target_system = 1
    mock_drone.connection.target_component = 1
    
    gimbal = GimbalController(mock_drone)
    
    # Test point forward
    result = gimbal.point_forward()
    assert result is True

def test_gimbal_retract():
    """Test gimbal retract method"""
    mock_drone = Mock()
    mock_drone.connection = Mock()
    mock_drone.connection.mav = Mock()
    mock_drone.connection.target_system = 1
    mock_drone.connection.target_component = 1
    
    gimbal = GimbalController(mock_drone)
    
    # Test retract
    result = gimbal.retract()
    assert result is True

def test_gimbal_neutral():
    """Test gimbal neutral method"""
    mock_drone = Mock()
    mock_drone.connection = Mock()
    mock_drone.connection.mav = Mock()
    mock_drone.connection.target_system = 1
    mock_drone.connection.target_component = 1
    
    gimbal = GimbalController(mock_drone)
    
    # Test neutral
    result = gimbal.neutral()
    assert result is True

def test_gimbal_mavlink_targeting():
    """Test gimbal mavlink targeting method"""
    mock_drone = Mock()
    mock_drone.connection = Mock()
    mock_drone.connection.mav = Mock()
    mock_drone.connection.target_system = 1
    mock_drone.connection.target_component = 1
    
    gimbal = GimbalController(mock_drone)
    
    # Test mavlink targeting
    result = gimbal.mavlink_targeting()
    assert result is True

def test_gimbal_point_at_location():
    """Test gimbal point at location method"""
    mock_drone = Mock()
    mock_drone.connection = Mock()
    mock_drone.connection.mav = Mock()
    mock_drone.connection.target_system = 1
    mock_drone.connection.target_component = 1
    
    gimbal = GimbalController(mock_drone)
    
    # Test point at location with valid coordinates
    result = gimbal.point_at_location(lat=37.7749, lon=-122.4194, alt=100)
    assert result is True

def test_gimbal_point_at_location_invalid_coordinates():
    """Test gimbal point at location with invalid coordinates"""
    mock_drone = Mock()
    mock_drone.connection = Mock()
    mock_drone.connection.mav = Mock()
    mock_drone.connection.target_system = 1
    mock_drone.connection.target_component = 1
    
    gimbal = GimbalController(mock_drone)
    
    # Test with invalid latitude
    with pytest.raises(GimbalParameterError):
        gimbal.point_at_location(lat=91, lon=-122.4194, alt=100)
    
    # Test with invalid longitude
    with pytest.raises(GimbalParameterError):
        gimbal.point_at_location(lat=37.7749, lon=181, alt=100)

def test_gimbal_get_status():
    """Test gimbal get status method"""
    mock_drone = Mock()
    mock_drone.connection = Mock()
    mock_drone.connection.mav = Mock()
    mock_drone.connection.target_system = 1
    mock_drone.connection.target_component = 1
    
    gimbal = GimbalController(mock_drone)
    
    # Test get status
    status = gimbal.get_status()
    assert isinstance(status, dict)
    assert "mode" in status
    assert "angles" in status
    assert "health" in status

def test_gimbal_parameter_methods():
    """Test gimbal parameter methods"""
    mock_drone = Mock()
    mock_drone.connection = Mock()
    mock_drone.connection.mav = Mock()
    mock_drone.connection.target_system = 1
    mock_drone.connection.target_component = 1
    
    # Mock the get_param and set_param methods
    mock_drone.get_param = Mock(return_value=45.0)
    mock_drone.set_param = Mock(return_value=True)
    
    gimbal = GimbalController(mock_drone)
    
    # Test get gimbal param
    result = gimbal.get_gimbal_param("PITCH_MIN")
    assert result == 45.0
    
    # Test set gimbal param
    result = gimbal.set_gimbal_param("PITCH_MIN", 30.0)
    assert result is True

def test_gimbal_parameter_validation_errors():
    """Test gimbal parameter validation errors"""
    mock_drone = Mock()
    mock_drone.connection = Mock()
    mock_drone.connection.mav = Mock()
    mock_drone.connection.target_system = 1
    mock_drone.connection.target_component = 1
    
    # Mock the get_param and set_param methods
    mock_drone.get_param = Mock(return_value=45.0)
    mock_drone.set_param = Mock(return_value=True)
    
    gimbal = GimbalController(mock_drone)
    
    # Test invalid parameter name
    with pytest.raises(GimbalParameterError):
        gimbal.get_gimbal_param("")
    
    with pytest.raises(GimbalParameterError):
        gimbal.get_gimbal_param(None)
    
    with pytest.raises(GimbalParameterError):
        gimbal.set_gimbal_param("", 30.0)
    
    with pytest.raises(GimbalParameterError):
        gimbal.set_gimbal_param(None, 30.0)

if __name__ == "__main__":
    # Run tests
    pytest.main([__file__])