from typing import Any, List, Tuple
import math
import time
import vrep  # type: ignore


class Tripwire(object):
  """Abstraction object for the tripwire, providing intuitive functions for the timer flag
  """
  def __init__(self, vrep_interface: Any, name: str='Proximity_sensor') -> None:
    self.vr = vrep_interface
    self.handle = self.vr.simxGetObjectHandle(name, 
                                              vrep.simx_opmode_oneshot_wait)
    
    self.vr.simxReadProximitySensor(self.handle, vrep.simx_opmode_streaming)

  def get_tripped(self) -> bool:
    """Returns the distance that the sensor sees
    """
    # boolean detectionState, array detectedPoint,
    # number detectedObjectHandle, array detectedSurfaceNormalVector
    # Specify -1 to retrieve the absolute position.
    return self.vr.simxReadProximitySensor(self.handle, vrep.simx_opmode_buffer)[0]


class Car(object):
  """Abstraction object for the car, providing intuitive functions for getting
  car state and setting outputs.
  """
  def __init__(self, vrep_interface: Any, name: str='AckermannSteeringCar') -> None:
    self.vr = vrep_interface
    self.car_handle = self.vr.simxGetObjectHandle(name, 
                                                  vrep.simx_opmode_oneshot_wait)
    self.boom_handle = self.vr.simxGetObjectHandle('BoomSensor', 
                                                   vrep.simx_opmode_oneshot_wait)
    self.camera_handle = []
    self.camera_handle.append(self.vr.simxGetObjectHandle('LineCamera0', 
                                                          vrep.simx_opmode_oneshot_wait))
    self.camera_handle.append(self.vr.simxGetObjectHandle('LineCamera1', 
                                                          vrep.simx_opmode_oneshot_wait))

    # Open these variables in streaming mode for high efficiency access.
    self.vr.simxGetObjectPosition(self.car_handle, -1, vrep.simx_opmode_streaming)
    self.vr.simxGetObjectVelocity(self.car_handle, vrep.simx_opmode_streaming)
    
    self.vr.simxGetFloatSignal('yDist', vrep.simx_opmode_streaming)
    self.vr.simxGetFloatSignal('steerAngle', vrep.simx_opmode_streaming)
    
    for handle in self.camera_handle:
      self.vr.simxGetVisionSensorImage(handle, 1, vrep.simx_opmode_streaming)
    
    # Get the wheel diameter so we can set velocity in physical units.
    # Let's assume all the wheels are the same.
    # TODO: check this assumption?
    wheel_dia = self.vr.get_bounding_size('RearLeftWheel_respondable')[0]
    # multiply linear velocity by this to get wheel radians/s
    self.speed_factor = 2 / wheel_dia 

    # Default parameters
    self.steering_limit = 30.0  # i believe that it's all 45 degrees...
    self.steering_slew_rate = 600/0.16 # depends on servo 600 degrees in 160 ms
    self.steering_slew_rate = 1200.0 # should be degrees per second
    #self.steering_slew_rate = 60/0.16 # depends on servo 60 degrees in 160 ms
    self.steering_time_ms = self.get_time()
    
    # state variables
    self.steering_state = 0.0
    self.old_lat_err = 0.0
    self.int_err = 0.0  # integral error

  # 
  # Some helper functions to get/set important car data/state
  #
  def get_position(self) -> Tuple[float, float, float]:
    """Returns the car's absolute position as a 3-tuple of (x, y, z), in meters.
    """
    # Specify -1 to retrieve the absolute position.
    return self.vr.simxGetObjectPosition(self.car_handle, -1, 
                                         vrep.simx_opmode_buffer)
  
  def get_velocity(self) -> Tuple[float, float, float]:
    """Returns the car's linear velocity as a 3-tuple of (x, y, z), in m/s.
    """
    return self.vr.simxGetObjectVelocity(self.car_handle, 
                                         vrep.simx_opmode_buffer)[0]

  def get_steering_angle(self) -> float:
    """Returns the car's steering angle in degrees.
    """
    return math.degrees(self.vr.simxGetFloatSignal('steerAngle', 
                                                   vrep.simx_opmode_buffer))

  def get_lateral_error(self) -> float:
    """Returns the lateral error (distance from sensor to line) in meters.
    """
    return self.vr.simxGetFloatSignal('yDist', vrep.simx_opmode_buffer)
  
  def get_line_camera_image(self, camera_index: int) -> List[int]:
    """Returns the line sensor image as an array of pixels, where each pixel is
    between [0, 255], with 255 being brightest.
    Likely non-physical (graphical) units of linear perceived pixel intensity.
    """
    # Magical '1' argument specifies to return the data as greyscale.
    handle = self.camera_handle[camera_index]
    _, image = self.vr.simxGetVisionSensorImage(handle, 1,
                                                vrep.simx_opmode_buffer)
    for i, intensity in enumerate(image):
      if intensity < 0:	# undo two's complement
        image[i] = 256 + intensity
    return image

  def set_speed(self, speed: float, blocking: bool=False) -> None:
    """Sets the car's target speed in m/s. Subject to acceleration limiting in 
    the simulator.
    """
    if blocking:
      op_mode = vrep.simx_opmode_oneshot_wait
    else:
      op_mode = vrep.simx_opmode_oneshot
    self.vr.simxSetFloatSignal('xSpeed', speed*self.speed_factor, op_mode)

  def get_time(self) -> int:  # gets time in Ms since the epoch.
    return int(round(time.time() * 1000))  # time in milli seconds.

  def set_steering_fast(self, angle_cmd: float, dt: float) -> float:
    """Sets the car's steering angle in degrees.
    Fast: No steering angle rate limiting.
    Returns the actual commanded angle.
    """
    angle = min(angle_cmd, self.steering_limit)
    angle = max(angle, -self.steering_limit)
    self.steering_state = angle  # update state
    self.vr.simxSetFloatSignal('steerAngle', angle*(math.pi/180.0), 
                               vrep.simx_opmode_oneshot)
    return(angle)

  def set_steering(self, angle_cmd: float, dt: float) -> float:
    """Sets the car's steering angle in degrees.
    Steering angle rate limiting happens here.
    Returns the actual commanded angle.
    """
    angle = self.steering_state  # get present angle
    angle_err = angle_cmd - angle
    
    # check if can reach angle in single time step?
    # if so, then don't need slew rate limit
    if (abs(angle_err) < dt*self.steering_slew_rate):
        angle = min(angle_cmd, self.steering_limit)  # check saturation
        angle = max(angle, -self.steering_limit)
        self.steering_state = angle # update state
        self.vr.simxSetFloatSignal('steerAngle', angle*(math.pi/180.0), 
                               vrep.simx_opmode_oneshot)
        return angle # can reach angle in single time step
    
    if (angle_cmd > self.steering_state +1):    # add some dead band
        angle = self.steering_state + dt * self.steering_slew_rate
    if (angle_cmd < self.steering_state -1):
        angle = self.steering_state - dt * self.steering_slew_rate
    angle = min(angle, self.steering_limit)
    angle = max(angle, -self.steering_limit)
    self.steering_state = angle  # update state
    self.vr.simxSetFloatSignal('steerAngle', angle*(math.pi/180.0), 
                               vrep.simx_opmode_oneshot)
    return angle

  def set_boom_sensor_offset(self, boom_length: float) -> None:
    """Sets the car's boom sensor's offset (approximate distance from front of
    car, in meters).
    This is provided so you don't have to learn how to mess with the V-REP
    scene to tune your boom sensor parameters.
    NOTE: this doesn't update the graphical boom stick.
    """
    self.vr.simxSetObjectPosition(self.boom_handle, vrep.sim_handle_parent, 
                                  (0, 0, -(boom_length-0.35)),
                                  vrep.simx_opmode_oneshot_wait)

  def set_line_camera_parameters(self, camera_index: int, height: float=0.3,
                                 orientation: float=30, fov: float=120) -> None:
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
    self.steering_limit = steering_limit
