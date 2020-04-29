from typing import Any, List, Tuple
import math
import time
import vrep  # type: ignore
import sys


class Tripwire(object):
  """Abstraction object for the tripwire, providing intuitive functions for the timer flag
  """
  def __init__(self, vrep_interface: Any, name: str = 'Proximity_sensor') -> None:
    self.vr = vrep_interface
    self.handle = self.vr.simxGetObjectHandle(name, vrep.simx_opmode_oneshot_wait)
    
    self.vr.simxReadProximitySensor(self.handle, vrep.simx_opmode_streaming)
    self.last_state = False

  def check_tripped(self) -> bool:
    """Returns true on a not tripped -> tripped transition. This must be called often enough to register the edge.
    """
    # boolean detectionState, array detectedPoint,
    # number detectedObjectHandle, array detectedSurfaceNormalVector
    # Specify -1 to retrieve the absolute position.
    in_proximity = self.vr.simxReadProximitySensor(self.handle, vrep.simx_opmode_streaming)[0]
    tripped = (not self.last_state and in_proximity)
    self.last_state = in_proximity
    return tripped


class Car(object):
  """Abstraction object for the car, providing intuitive functions for getting
  car state and setting outputs.
  """
  def __init__(self, vrep_interface: Any, steering_limit: float = 30, steering_slew_rate: float = (60/0.16)) -> None:
    if steering_limit != 30:
      print(f"init: set steering limit {steering_limit}", file=sys.stderr)
      steering_limit = 30
    if steering_slew_rate != (60/0.016):
      print(f"init: set steering slew rate {steering_slew_rate}", file=sys.stderr)
      steering_slew_rate = (60/0.016)
  
    self.vr = vrep_interface
    self.car_handle = self.vr.simxGetObjectHandle('AckermannSteeringCar', vrep.simx_opmode_oneshot_wait)
    self.camera_handle = []
    self.camera_handle.append(self.vr.simxGetObjectHandle('LineCamera0', vrep.simx_opmode_oneshot_wait))
    self.camera_handle.append(self.vr.simxGetObjectHandle('LineCamera1', vrep.simx_opmode_oneshot_wait))

    self.motor_fl_handle = self.vr.simxGetObjectHandle('FrontLeftMotor', vrep.simx_opmode_oneshot_wait)
    self.motor_fr_handle = self.vr.simxGetObjectHandle('FrontRightMotor', vrep.simx_opmode_oneshot_wait)
    self.vr.simxSetJointTargetVelocity(self.motor_fl_handle, 0, vrep.simx_opmode_oneshot_wait)
    self.vr.simxSetJointTargetVelocity(self.motor_fr_handle, 0, vrep.simx_opmode_oneshot_wait)
    self.wheel_fl_handle = self.vr.simxGetObjectHandle('FrontLeftWheel_respondable', vrep.simx_opmode_oneshot_wait)
    self.wheel_fr_handle = self.vr.simxGetObjectHandle('FrontRightWheel_respondable', vrep.simx_opmode_oneshot_wait)

    self.steer_fl_handle = self.vr.simxGetObjectHandle('FrontLeftSteering', vrep.simx_opmode_oneshot_wait)
    self.steer_fr_handle = self.vr.simxGetObjectHandle('FrontRightSteering', vrep.simx_opmode_oneshot_wait)
    self.vr.simxSetJointTargetPosition(self.steer_fl_handle, 0, vrep.simx_opmode_oneshot_wait)
    self.vr.simxSetJointTargetPosition(self.steer_fr_handle, 0, vrep.simx_opmode_oneshot_wait)

    # Open these variables in streaming mode for high efficiency access.
    self.vr.simxGetObjectPosition(self.car_handle, -1, vrep.simx_opmode_streaming)
    self.vr.simxGetObjectVelocity(self.car_handle, vrep.simx_opmode_streaming)

    for handle in self.camera_handle:
      self.vr.simxGetVisionSensorImage(handle, 1, vrep.simx_opmode_streaming)
    
    # Get the wheel diameter so we can set velocity in physical units.
    # Let's assume all the wheels are the same.
    # TODO: check this assumption?
    wheel_dia = self.vr.get_bounding_size('RearLeftWheel_respondable')[0]
    # multiply linear velocity by this to get wheel radians/s
    self.speed_factor = 2 / wheel_dia
    self.wheelbase_width = 0.0755
    self.wheelbase_length = 0.25

    # Default parameters
    self.steering_limit = steering_limit
    self.steering_slew_rate = steering_slew_rate  # degrees per second
    self.steering_deadband = 1.0  # degree, below which changes don't register

    # state variables
    self.steering_state = 0.0

  # 
  # Some helper functions to get/set important car data/state
  #
  def get_sim_time(self) -> float:
    """Returns the current simulation time in seconds.
    """
    return vrep.simxGetLastCmdTime(self.vr.client_id) / 1000.0

  def _get_position(self) -> Tuple[float, float, float]:
    """Returns the car's absolute position as a 3-tuple of (x, y, z), in meters.
    """
    # Specify -1 to retrieve the absolute position.
    return self.vr.simxGetObjectPosition(self.car_handle, -1, 
                                         vrep.simx_opmode_streaming)
  
  def get_position(self) -> Tuple[float, float, float]:
    """Returns the car's absolute position as a 3-tuple of (x, y, z), in meters.
    """
    # Specify -1 to retrieve the absolute position.
    return (0, 0, 0)
  
  def _get_velocity(self) -> Tuple[float, float, float]:
    """Returns the car's linear velocity as a 3-tuple of (x, y, z), in m/s.
    """
    return self.vr.simxGetObjectVelocity(self.car_handle, 
                                         vrep.simx_opmode_streaming)[0]
  
  def get_velocity(self) -> Tuple[float, float, float]:
    """Returns the car's linear velocity as a 3-tuple of (x, y, z), in m/s.
    """
    return (0, 0, 0)

  def get_wheel_velocity(self) -> float:
    """ returns average of front wheel speeds. GetObjectVelocity returns lin vel,ang vel
    angular velocity vector in rad/sec
    """
    left_vel_vec = self.vr.simxGetObjectVelocity(self.wheel_fl_handle,vrep.simx_opmode_streaming)[1]
    left_speed = math.sqrt(left_vel_vec[0]**2 + left_vel_vec[1]**2 + left_vel_vec[2]**2) / self.speed_factor
    right_vel_vec = self.vr.simxGetObjectVelocity(self.wheel_fr_handle,vrep.simx_opmode_streaming)[1]
    right_speed = math.sqrt(right_vel_vec[0]**2 + right_vel_vec[1]**2 + right_vel_vec[2]**2) / self.speed_factor
    return (left_speed + right_speed)/2.0

  def get_steering_angle(self) -> float:
    """Returns the car's steering angle in degrees.
    """
    return self.steering_state

  def get_line_camera_image(self, camera_index: int) -> List[int]:
    """Returns the line sensor image as an array of pixels, where each pixel is
    between [0, 255], with 255 being brightest.
    Likely non-physical (graphical) units of linear perceived pixel intensity.
    """
    # Magical '1' argument specifies to return the data as greyscale.
    handle = self.camera_handle[camera_index]
    _, image = self.vr.simxGetVisionSensorImage(handle, 1,
                                                vrep.simx_opmode_streaming)
    for i, intensity in enumerate(image):
      if intensity < 0:	 # undo two's complement
        image[i] = 256 + intensity
    return image

  def set_speed(self, speed: float, blocking: bool = False) -> None:
    """Sets the car's target speed in m/s. Subject to acceleration limiting in 
    the simulator.
    """
    if blocking:
      op_mode = vrep.simx_opmode_oneshot_wait
    else:
      op_mode = vrep.simx_opmode_oneshot

    desired_speed = speed * self.speed_factor
    self.vr.simxSetJointTargetVelocity(self.motor_fl_handle, desired_speed, op_mode)
    self.vr.simxSetJointTargetVelocity(self.motor_fr_handle, desired_speed, op_mode)

  def __set_steering(self, angle_deg: float) -> None:
    angle_rad = math.radians(angle_deg)
    if angle_deg != 0:
      steer_angle_left = math.atan(1 / (-self.wheelbase_width + 1/math.tan(angle_rad)))
      steer_angle_right = math.atan(1 / (self.wheelbase_width + 1/math.tan(angle_rad)))
    else:
      steer_angle_left = 0
      steer_angle_right = 0
    self.vr.simxSetJointTargetPosition(self.steer_fl_handle, steer_angle_left, vrep.simx_opmode_oneshot)
    self.vr.simxSetJointTargetPosition(self.steer_fr_handle, steer_angle_right, vrep.simx_opmode_oneshot)

  def set_steering_fast(self, angle_cmd: float, dt: float) -> float:
    """Sets the car's steering angle in degrees.
    Fast: No steering angle rate limiting.
    Returns the actual commanded angle.
    """
    print(f"set_steering_fast", file=sys.stderr)
    return self.set_steering(angle_cmd, dt)

  def set_steering(self, angle_cmd: float, dt: float) -> float:
    """Sets the car's steering angle in degrees.
    Steering angle rate limiting happens here.
    Returns the actual commanded angle.
    """
    if dt > 0.011:
      print(f"set_steering: high dt {dt}", file=sys.stderr)
      dt = 0.01
    if self.steering_limit != 30:
      print(f"set_steering: steering limit {self.steering_limit}", file=sys.stderr)
      self.steering_limit = 30
    if self.steering_slew_rate != (60/0.016):
      print(f"set_steering: steering slew rate {self.steering_slew_rate}", file=sys.stderr)
      self.steering_slew_rate = (60/0.016)
    
    max_angle_change = dt * self.steering_slew_rate
    angle = max(min(angle_cmd, self.steering_state + max_angle_change),
                self.steering_state - max_angle_change)
    angle = max(min(angle, self.steering_limit), -self.steering_limit)
    self.steering_state = angle  # update state
    self.__set_steering(angle)
    return angle

  def set_line_camera_parameters(self, camera_index: int, height: float = 0.3,
                                 orientation: float = 30, fov: float = 120) -> None:
    """Sets the car's line camera parameters.
    This is provided so you don't have to learn how to mess with the V-REP
    scene to tune your camera parameters.
    Args:
        height -- height of the camera, in meters.
        orientation -- downward angle of the camera, in degrees from horizontal.
        fov -- field of vision of the camera, in degrees.
        *** note: puts camera above orgin of car ***
    """
    handle = self.camera_handle[camera_index]
    self.vr.simxSetObjectPosition(handle, vrep.sim_handle_parent, 
                                  (height, 0, 0), 
                                  vrep.simx_opmode_oneshot_wait)
    self.vr.simxSetObjectOrientation(handle, vrep.sim_handle_parent, 
                                     (0, 
                                      -math.radians(orientation), 
                                      math.radians(90)), 
                                     vrep.simx_opmode_oneshot_wait)
    self.vr.simxSetObjectFloatParameter(handle, 1004, math.radians(fov), 
                                        vrep.simx_opmode_oneshot_wait)

  def set_steering_limit(self, steering_limit: float) -> None:
    """Sets the car's steering limit in degrees from center.
    Attempts to steer past this angle (on either side) get clipped.
    """
    if steering_limit != 30:
      print(f"set_steering_limit: set steering limit {steering_limit}", file=sys.stderr)
