# modifications by Doug from Spring 2016 using single loop
# modified Spring 2018 to handle angle and rate limits for steering servo
from typing import Any, List, Optional

import math
import time
import os.path
import numpy as np  # type: ignore
import vrep  # type: ignore
import vrepInterface  # type: ignore
from carInterface import Car, Tripwire
from simpleCsvDict import SimpleCsvDictWriter
import importlib


if __name__ == "__main__":
  import argparse
  import sys
  
  parser = argparse.ArgumentParser(description='ee192 Python V-REP Car controller.')
  parser.add_argument('--synchronous', metavar='s', type=bool, default=True,
                      help="""enable synchronous mode, forcing the simulator to 
                      operate in lockstep with the simulator - potentially 
                      increases accuracy / repeatability at the cost of 
                      performance""")
  parser.add_argument('--restart', metavar='r', type=bool, default=False,
                      help="""whether to restart the simulation if a simulation
                      is currently running""")
  parser.add_argument('--csvfile', metavar='c', default='car_data',
                      help='csv filename to log to')
  parser.add_argument('--csvfile_overwrite', metavar='csvfile_overwrite', type=bool, default=True,
                      help='overwrite the specified csvfile without warning')
  parser.add_argument('--laps', metavar='l', type=int, default=1,
                      help="""Number of laps to run, default of 1. 0 means infinite.""")
  parser.add_argument('--velocity', metavar='v', type=float, default=2.0,
                     help="""Set the Velocity, in m/s.""")
  parser.add_argument('--module', metavar='m',
                     help="""SimulationAssignment module to import.""")
  args = parser.parse_args()

  SimulationAssignment = importlib.import_module(args.module).SimulationAssignment

  # Check that we won't overwrite an existing csvfile before mucking with the
  # simulator.
  assert args.csvfile_overwrite or (args.csvfile is not None and not os.path.exists(args.csvfile))


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
    finish_wire = Tripwire(vr, 'Proximity_sensor_StartLine')
    stopping_wire = Tripwire(vr, 'Proximity_sensor')

    success = False
    while not success:
      try:
        sim_start_time = car.get_sim_time()
        success = True
      except vrepInterface.VRepAPIError:
        print("waiting for simulation start")
        print("waiting for simulation start", file=sys.stderr)
        time.sleep(1)

    assignment = SimulationAssignment(vr, car, args.velocity)
    assignment.setup_car(vr, car)
    
    csvfile = None
    if args.csvfile:
      csvfile = SimpleCsvDictWriter(args.csvfile + '_lap0.csv')

    try:
      done = False
      completed_laps = -1
      lap_start_time = -100.0
      lap_start_odometer = 0
      odometer = 0
      last_pos = car._get_position()
      last_sim_time = car.get_sim_time()
      
      while not done:
        assignment.control_loop(vr, car, csvfile)
        sim_time = car.get_sim_time()
        pos = car._get_position()
        odometer += math.sqrt((pos[0] - last_pos[0])**2 + (pos[1] - last_pos[1])**2)
        last_pos = pos
        vel_vec = car._get_velocity()
        vel = math.sqrt(vel_vec[0]**2 + vel_vec[1]**2 + vel_vec[2]**2)
        
        if sim_time == lap_start_time:
          avg_vel = 0
        else:
          avg_vel = (odometer - lap_start_odometer) / (sim_time - lap_start_time)
        
        print('\rlap %i: t=%6.2f (od=%5.2f, sp=%5.2f, avsp=%5.2f): steer_angle=%6.1f'
          % (completed_laps + 1, sim_time, odometer, vel, avg_vel, car.get_steering_angle()),
          file=sys.stderr, end='')
        
        if finish_wire.check_tripped():
          completed_laps += 1
          if completed_laps > 0:  # discard the first finish line crossing, which happens at the start
            print("\nfinished lap %i, lap time: %.2f, total elapsed time: %.2f" %
                (completed_laps, car.get_sim_time() - lap_start_time, car.get_sim_time() - sim_start_time))
            print("\nfinished lap %i, lap time: %.2f, total elapsed time: %.2f" %
                (completed_laps, car.get_sim_time() - lap_start_time, car.get_sim_time() - sim_start_time),
                file=sys.stderr)
          else:
            print("\nstarted lap %i, total elapsed time: %.2f" %
                (completed_laps + 1, car.get_sim_time() - sim_start_time))
            print("\nstarted lap %i, total elapsed time: %.2f" %
                (completed_laps + 1, car.get_sim_time() - sim_start_time), file=sys.stderr)
          lap_start_time = sim_time
          lap_start_odometer = odometer
          
          if args.csvfile and csvfile is not None and not done:
            csvfile.close()
            csvfile = SimpleCsvDictWriter(args.csvfile + '_lap' + str(completed_laps + 1) + '.csv')

        if stopping_wire.check_tripped() and completed_laps > 0:
          print("\ncrossed stopping line, lap+%.2f, total elapsed time: %.2f" %
              (car.get_sim_time() - lap_start_time, car.get_sim_time() - sim_start_time))
          print("\ncrossed stopping line, lap+%.2f, total elapsed time: %.2f" %
              (car.get_sim_time() - lap_start_time, car.get_sim_time() - sim_start_time), file=sys.stderr)
          if completed_laps >= args.laps and args.laps != 0:
            done = True

        if args.synchronous:
          vr.simxSynchronousTrigger()
    except KeyboardInterrupt:
      # Allow a keyboard interrupt to break out of the loop while still shutting
      # down gracefully.
      print("caught keyboard interrupt, terminating control loop")
      print("caught keyboard interrupt, terminating control loop", file=sys.stderr)
    finally:
      print("ending simulation")
      print("ending simulation", file=sys.stderr)
      vr.simxStopSimulation(vrep.simx_opmode_oneshot_wait)

      if csvfile is not None:
        csvfile.close()   # close file and writer
        print("csv files closed")
        print("csv files closed", file=sys.stderr)
