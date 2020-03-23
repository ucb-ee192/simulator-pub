# modifications by Doug from Spring 2016 using single loop
# modified Spring 2018 to handle angle and rate limits for steering servo
from typing import Any, List, Optional

import math
import time
import os.path
import numpy as np  # type: ignore
import vrep  # type: ignore
import vrepInterface  # type: ignore
import sys
from carInterface import Car, Tripwire
from simpleCsvDict import SimpleCsvDictWriter


class SimulationAssignment():
  """Everything you need to implement for the assignment is here.
  You may """
  def __init__(self, vr: Any, car: Car, target_speed: float) -> None:
    # You may initialize additional state variables here
    self.last_sim_time = vr.simxGetFloatSignal('simTime', vrep.simx_opmode_oneshot_wait)

    self.target_speed = target_speed

    self.old_lat_err = 0.0
    self.int_err = 0.0  # integral error

  def setup_car(self, vr: Any, car: Car) -> None:
    """Sets up the car's physical parameters.
    """
    #
    # ASSIGNMENT: You may want to tune these paraneters.
    #
    car.set_boom_sensor_offset(0.1)
    # In the scene, we provide two cameras.
    # Camera 0 is used by default in the control loop and is the "near" one.
    # Some teams have also used a "far" camera in the past (Camera 1).
    # You can use additional cameras, but will have to add then in the V-REP scene
    # and bind the handle in Car.__init__. The V-REP remote API doesn't provide a
    # way to instantiate additional vision sensors.
    car.set_line_camera_parameters(0, height=0.3, orientation=60, fov=90)
    car.set_line_camera_parameters(1, height=0.4, orientation=15, fov=60)
    # You should measure the steering servo limit and set it here.
    # A more accurate approach would be to implement servo slew limiting.
    car.set_steering_limit(30)

  def get_line_camera_error(self, image: List[int]) -> float:
    """Returns the distance from the line, as seen by the line camera, in 
    pixels. The actual physical distance (in meters) can be derived with some
    trig given the camera parameters.
    """
    #
    # ASSIGNMENT: You should implement your line tracking algorithm here.
    # The default code implements a very simple, very not-robust line detector.
    #
    # NOTE: unlike the actual camera, get_line_camera_image() returns pixel
    # intensity data in the range of [0, 255] and aren't physically based (i.e.
    # intensity in display intensity units, no integration time).
    #
    INTENSITY_THRESHOLD = 192

    weighted_sum = 0
    element_sum = 0
    for i, intensity in enumerate(image):
      if intensity > INTENSITY_THRESHOLD:
        weighted_sum += i
        element_sum += 1
    if element_sum == 0:
      return 0
    return weighted_sum / element_sum - 63
  
  def control_loop(self, vr: Any, car: Car, csvfile: Optional[SimpleCsvDictWriter]) -> None:
    """Control iteration. This is called on a regular basis.
    Args:
        vr -- VRepInterface object, which is an abstraction on top of the VREP
              Python API. See the class docstring near the top of this file.
        car -- Car object, providing abstractions for the car (like steering and
               velocity control).
        csvfile -- Optional to log data. None to disable.
    """
    sim_time = car.get_sim_time()
    dt = sim_time - self.last_sim_time
    self.last_sim_time = sim_time

    #
    # ASSIGNMENT: Tune / implement a better controller loop here.
    #
    
    # Here are two different sensors you can play with.
    # One provides an ideal shortest-distance-to-path (in meters), but isn't
    # path-following and may jump at crossings.
    # The other uses a more realistic line sensor (in pixels), which you can
    # plug your track detection algorithm into.
    line_camera_image0 = car.get_line_camera_image(0)
    line_camera_image1 = car.get_line_camera_image(1)
    line0_err = self.get_line_camera_error(line_camera_image0)
    line1_err = self.get_line_camera_error(line_camera_image1)

    # line camera has 0.7 m field of view
    lat_err = -(np.float(line0_err)/128)*0.7  # pixel to meter conversion
    # lat_err = car.get_lateral_error()  # actual distance rather than camera estimate, might be broken
    
    if dt > 0.0:
      lat_vel = (lat_err - self.old_lat_err)/dt
    else:
      lat_vel = 0.0
    self.old_lat_err = lat_err
    
    # calculate integral error
    self.int_err = self.int_err + dt*lat_err

    # Proportional gain in steering control (degrees) / lateral error (meters)
    kp = 200
    kd = 0 # deg per m/s
    ki = 0 # deg per m-s
    steer_angle = -kp * lat_err - kd * lat_vel - ki * self.int_err

    steer_angle = car.set_steering(steer_angle, dt)  # use set_steering to include servo slew rate limit
    # steer_angle = car.set_steering_fast(steer_angle,dt)  # use set_steering_fast for no delay
    
    # Constant speed for now. You can tune this and/or implement advanced controllers.
    car.set_speed(self.target_speed)

    # Print out and record debugging info
    pos = car.get_position()
    vel_vector = car.get_velocity()
    vel = math.sqrt(vel_vector[0]**2 + vel_vector[1]**2 + vel_vector[2]**2)
    print('t=%6.3f (x=%5.2f, y=%5.2f, sp=%5.2f): lat_err=%5.2f, int_err=%5.2f, line0_err=%3i, steer_angle=%3.1f'
          % (sim_time, pos[0], pos[1], vel,
             lat_err, self.int_err, (line0_err or 0), steer_angle))
  
    if csvfile is not None:
      csvfile.writerow({'t': sim_time,
                        'x': pos[0], 'y': pos[1],
                        'linescan': line_camera_image0,
                        'line_pos': line0_err + 63,  # needs to be in camera pixels so overlaid plots work
                        'linescan_far': line_camera_image1,
                        'line_pos_far': line1_err + 63,
                        'speed': vel,
                        'lat_err': lat_err,
                        'steer_angle': steer_angle,
                        })


if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(description='ee192 Python V-REP Car controller.')
  parser.add_argument('--synchronous', metavar='s', type=bool, default=True,
                      help="""enable synchronous mode, forcing the simulator to 
                      operate in lockstep with the simulator - potentially 
                      increases accuracy / repeatability at the cost of 
                      performance""")
  parser.add_argument('--restart', metavar='r', type=bool, default=False,
                      help="""whether to restart the simulation if a simulation
                      is currently running""")
  parser.add_argument('--csvfile', metavar='c', default='car_data.csv',
                      help='csv filename to log to')
  parser.add_argument('--csvfile_overwrite', metavar='csvfile_overwrite', type=bool, default=True,
                      help='overwrite the specified csvfile without warning')
  parser.add_argument('--laps', metavar='l', type=int, default=1,
                      help="""Number of laps to run, default of 1. 0 means infinite.""")
  parser.add_argument('--velocity', metavar='v', type=float, default=2.5,
                     help="""Set the Velocity, in m/s.""")
  args = parser.parse_args()

  # Check that we won't overwrite an existing csvfile before mucking with the
  # simulator.
  if (args.csvfile is not None and os.path.exists(args.csvfile) 
      and not args.csvfile_overwrite):
    print("csvfile '%s' already exists: aborting." % args.csvfile)
    sys.exit()

  # Stop the existing simulation if requested. Not using a separate
  # VRepInterface (and hence VREP API client id) seems to cause crashes.
  if args.restart:
    with vrepInterface.VRepInterface.open() as vr:
      vr.simxStopSimulation(vrep.simx_opmode_oneshot_wait)

  # This needs not be in the main body, or the API throws an exception a split second after simulation start.
  with vrepInterface.VRepInterface.open() as vr:
    ret = vr.simxStartSimulation(vrep.simx_opmode_oneshot_wait)

  # Open a V-REP API connection and get the car.
  with vrepInterface.VRepInterface.open() as vr:
    if args.synchronous:
      vr.simxSynchronous(1)

    car = Car(vr)
    wire = Tripwire(vr)

    success = False
    while not success:
      try:
        vr.simxGetFloatSignal('simTime', vrep.simx_opmode_oneshot_wait)
        success = True
      except vrepInterface.VRepAPIError:
        print("waiting for simulation start")
        time.sleep(1)

    assignment = SimulationAssignment(vr, car, args.velocity)
    assignment.setup_car(vr, car)
    
    csvfile = None
    if args.csvfile:
      csvfile = SimpleCsvDictWriter(args.csvfile)

    try:
      done = False
      completed_laps = -1
      while not done:
        assignment.control_loop(vr, car, csvfile)
        finish_tripped = wire.check_tripped()
        if finish_tripped:
          completed_laps += 1
          if completed_laps > 0:  # discard the first finish line crossing, which happens at the start
            print("finished lap " + str(completed_laps))
          if completed_laps >= args.laps and args.laps != 0:
            done = True

        if args.synchronous:
          vr.simxSynchronousTrigger()
    except KeyboardInterrupt:
      # Allow a keyboard interrupt to break out of the loop while still shutting
      # down gracefully.
      print("caught keyboard interrupt, terminating control loop")
      pass
    finally:
      print("ending simulation")
      vr.simxStopSimulation(vrep.simx_opmode_oneshot_wait)

      if csvfile is not None:
        csvfile.close()   # close file and writer
        print("csv files closed")
