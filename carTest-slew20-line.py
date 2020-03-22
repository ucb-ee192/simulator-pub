# modifications by Doug from Spring 2016 using single loop
# modified Spring 2018 to handle angle and rate limits for steering servo
from typing import Any, List, Optional, Union, TextIO, BinaryIO

import csv
import math
import os.path
import sys
import time
import numpy as np  # type: ignore
import vrep  # type: ignore
import vrepInterface  # type: ignore
from carInterface import Car, Tripwire


class SimulationAssignment():
  """Everything you need to implement for the assignment is here.
  You may """
  def __init__(self, vr: Any, car: Car, wire: Tripwire) -> None:
    # You may initialize additional state variables here
    self.last_sim_time = vr.simxGetFloatSignal('simTime', 
                                               vrep.simx_opmode_oneshot_wait)
  
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
      #return None
      return 0
    return weighted_sum / element_sum - 63
  
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
  
  def control_loop(self, vr: Any, car: Car, wire: Tripwire, csvfile: Optional[Any]=None, linecsv: Optional[Any]=None) -> bool:
    """Control iteration. This is called on a regular basis.
    Args:
        vr -- VRepInterface object, which is an abstraction on top of the VREP
              Python API. See the class docstring near the top of this file.
        car -- Car object, providing abstractions for the car (like steering and
               velocity control).
        csvfile -- Optional csv.DictWriter for logging data. None to disable.
        linecsv -- Optional csv.DictWriter for line camera. None to disable.
    """
    # Waiting mode is used here to 
    time = vr.simxGetFloatSignal('simTime', vrep.simx_opmode_oneshot_wait)
    dt = time - self.last_sim_time
    self.last_sim_time = time
    crossed = wire.get_tripped()

    #
    # ASSIGNMENT: Tune / implement a better controller loop here.
    #
    
    # Here are two different sensors you can play with.
    # One provides an ideal shortest-distance-to-path (in meters), but isn't
    # path-following and may jump at crossings.
    # The other uses a more realistic line sensor (in pixels), which you can
    # plug your track detection algorithm into.
    #lat_err = car.get_lateral_error()
    line_camera_image0 = car.get_line_camera_image(0)
    # line0_err = self.get_line_camera_error(car.get_line_camera_image(0))
    line0_err = self.get_line_camera_error(line_camera_image0)
    line1_err = self.get_line_camera_error(car.get_line_camera_image(1))
    
    # line camera has 0.7 m field of view
    lat_err = -(np.float(line0_err)/128)*0.7 # pixel to meter conversion
    
    #lat_err = car.get_lateral_error() # actual distance rather than camera estimate
    
    if (dt > 0.0):
        lat_vel = (lat_err - car.old_lat_err)/dt
    else:
            lat_vel = 0.0
    car.old_lat_err = lat_err
    
    #calculate integral error    
    car.int_err = car.int_err + dt*lat_err
    
    
    
    # Proportional gain in steering control (degrees) / lateral error (meters)
    kp = 200
    kd = 0 # deg per m/s
    ki = 0 # deg per m-s
    steer_angle = -kp * lat_err - kd*lat_vel - ki* car.int_err
    
    # use set_steering to include servo slew rate limit
    steer_angle = car.set_steering(steer_angle,dt)
    #steer_angle = car.set_steering_fast(steer_angle,dt)
    #steer_angle = car.set_steering_fast(16,dt)  # check steering radius
    
    # use this function for no delay
    # steer_angle = car.set_steering_fast(steer_angle,dt) 
    
    # Constant speed for now. You can tune this and/or implement advanced
    # controllers.
    car.set_speed(ve)
    #car.set_speed(2.0)
    
    # Print out debugging info
    # lat_err = car.get_lateral_error() # actual distance rather than camera estimate
    # lateral error seems broken compared to camera
    pos = car.get_position()
    vel_vector = car.get_velocity()
    vel = math.sqrt(vel_vector[0]**2 + vel_vector[1]**2 + vel_vector[2]**2)
    print('t=%6.3f (x=%5.2f, y=%5.2f, sp=%5.2f): lat_err=%5.2f, int_err=%5.2f, line0_err=%3i, steer_angle=%3.1f'
          % (time, pos[0], pos[1], vel, 
             lat_err, car.int_err, (line0_err or 0), steer_angle))
  
    if csvfile is not None:
      csvfile.writerow({'t': time, 
                        'x': pos[0], 'y': pos[1], 'speed': vel,
                        'lat_err': lat_err, 
                        'line0_err': (line0_err or ""), 
                        'line1_err': (line1_err or ""), 
                        'steer_angle': steer_angle
                        })
    if linecsv is not None:
      linecsv.writerow({'time_ms':time,
                      'linescan_near':line_camera_image0, 
                      'velocity(m/s)':vel})
    return crossed  # tells me if i've crossed the line or not!  Do NOT delete this line.
  
if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(description='ee192 Python V-REP Car controller.')
  parser.add_argument('--iterations', metavar='iteration', type=int, default=300,
                      help='number of control iterations to run')
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
  parser.add_argument('--linefile', metavar='linefile', default='line_data.csv',
                      help='line data filename to log to')
  parser.add_argument('--csvfile_overwrite', metavar='csvfile_overwrite', type=bool, default=True,
                      help='overwrite the specified csvfile without warning')
  parser.add_argument('--oneLoop', metavar='l',default = True,
                      help="""Run the car through only one round of the track, based on the tripwire.  Defaults to False.""")
  parser.add_argument('--constants', metavar='k', nargs="+", default=None, type=float,
                       help="""A list (of airbitrary length) of contants.  Useful for PID tuning, for example""")
  parser.add_argument('--velocity', metavar='v', type=float, default=2.5,
                     help="""Set the Velocity, in m/s.""")
  args = parser.parse_args()
  

  ve = args.velocity


  # Check that we won't overwrite an existing csvfile before mucking with the
  # simulator.
  if (args.csvfile is not None and os.path.exists(args.csvfile) 
      and not args.csvfile_overwrite):
    print("csvfile '%s' already exists: aborting." % args.csvfile)
    sys.exit()
  
  # Terminate any existing sessions, just in case.
  vrep.simxFinish(-1)
  
  # Stop the existing simulation if requested. Not using a separate
  # VRepInterface (and hence VREP API client id) seems to cause crashes.
  if args.restart:
    with vrepInterface.VRepInterface.open() as vr:
      vr.simxStopSimulation(vrep.simx_opmode_oneshot_wait)
  
  # Open a V-REP API connection and get the car.
  with vrepInterface.VRepInterface.open() as vr:
    if args.synchronous:
      vr.simxSynchronous(1)
      
    vr.simxStartSimulation(vrep.simx_opmode_oneshot_wait)
    
    car = Car(vr)
    wire = Tripwire(vr)
    assignment = SimulationAssignment(vr, car, wire) #give it the tripwire, too
    assignment.setup_car(vr, car)
    
    csvfile = None
    if args.csvfile:
      # Dirty hack to get this (potentially) working in Python 2 and 3
      if sys.version_info.major < 3:
        outfile: Union[TextIO, BinaryIO] = open(args.csvfile, 'wb')
      else:
        outfile = open(args.csvfile, 'w', newline='')
        
      fieldnames = ['t', 'x', 'y', 'speed', 'lat_err', 'line0_err', 'line1_err', 
                    'steer_angle']
      fielddict = {}
      for fieldname in fieldnames:
        fielddict[fieldname] = fieldname
      csvfile = csv.DictWriter(outfile, fieldnames=fieldnames)  # change so don't over write orig file object!
      csvfile.writerow(fielddict)
      
      # setup csv file for line data
      if sys.version_info.major < 3:
        linefile: Union[TextIO, BinaryIO] = open(args.linefile,'wb')
      else:
        linefile = open(args.linefile,'w', newline='')
        
    
    
    fieldnames = ['time_ms', 'linescan_near', 'velocity(m/s)']
    linecsv = csv.DictWriter(linefile, fieldnames=fieldnames)
    linecsv.writeheader()
 
    
    if not args.oneLoop:
      try:
        for i in range(0, args.iterations):
    
          assignment.control_loop(vr, car, wire, csvfile, linecsv)
          
          # Advance to the next frame
          if args.synchronous:
            vr.simxSynchronousTrigger()
            
        print("Finished %i control iterations: pausing simulation." 
              % args.iterations)
        vr.simxPauseSimulation(vrep.simx_opmode_oneshot_wait)
        # need to have clean file close
        outfile.close()   # close file and writer
        linefile.close()  # close line writing
        print("files closed")
      except KeyboardInterrupt:
      # Allow a keyboard interrupt to break out of the loop while still shutting
      # down gracefully. 
        print("KeyboardInterrupt: pausing simulation.")
        vr.simxPauseSimulation(vrep.simx_opmode_oneshot_wait)
    else:
      try:
        keepGoing = True
        count = 0
        oldCrossed = False;
        while keepGoing:
          # crossed = assignment.control_loop(vr, car, wire, csvfile, linecsv)
          crossed = assignment.control_loop(vr, car, wire, csvfile, linecsv)
          if (oldCrossed == False) and (crossed == True): #transition 0-> 1!
              oldCrossed = True # on top of sensor
              count = count + 1 # increase lap count on starting edge
          if (oldCrossed == True) and (crossed == False): # transition 1 -> 0
              oldCrossed = False # arm for next lap trigger

          # Advance to the next frame
          if args.synchronous:
             vr.simxSynchronousTrigger()
          if count == 2: #two lowToHigh Transitions.  First one is crossing the tripwire the first time (get a run-up on the speed).  Second is to stop.
             keepGoing = False
            
        print("Finished one loop: ending simulation." )
        vr.simxStopSimulation(vrep.simx_opmode_oneshot_wait)
        # need to have clean file close
        outfile.close()   # close file and writer
        linefile.close()  # close line writing
        print("files closed")
        
      except KeyboardInterrupt:
        # Allow a keyboard interrupt to break out of the loop while still shutting
        # down gracefully. 
        print("KeyboardInterrupt: pausing simulation.")
        vr.simxPauseSimulation(vrep.simx_opmode_oneshot_wait)
