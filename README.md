# simulator-pub
V-REP simulator


## Assignment
The control algorithm is defined in [controller.py](controller.py).
A simple (unreliable) camera algorithm and barely tuned PID (also unreliable) controller is provided for you in the skeleton code in the `SimulationAssignment` class, which you should modify.

Feel free to change constants or add additional state variables as needed.
You may also add additional items to `csvfile.writerow` to dump more useful data, as long as the keys are kept constant every iteration.

### Running
1. You will need to [download and install V-REP 3.6.2](https://www.coppeliarobotics.com/previousVersions).
1. With a [track file](tracks/) open in V-REP, start `controller.py`. cory-track-fastcar.ttt and ee192_Round2 work.
   This can be in a separate terminal. 
   - If `controller.py` refuses to start because it cannot find the shared library, you will have to copy the relevant shared library file from your local V-REP installation directory into the folder where `controller.py` is.
1. A few warnings will pop up in the simulator after starting - click through these, and the simulation should resume.
1. By default, the simulation will run one lap and exit.
   You can change this by passing in `--laps` into `controller.py`, with `0` meaning infinite. You can stop the simulation using the square stop button in V-rep.
1. Data will be dumped by default in `car_data_lapX.csv`, which you can visualize (plot) using [log-visualizer.py in the telemetry repository](https://github.com/ucb-ee192/telemetry/blob/master/client-py/log-visualizer.py).
   This invocation is a starting point assuming stock CSV output: `python log-visualizer.py --merge linescan,line_pos --merge linescan_far,line_pos_far --merge x,y car_data.csv`
1. You can also plot the X-Y (car track) data using `xyplot.py`.
   This invocation is a starting point assuming stock CSV output: `python xyplot.py car_data_lapX.csv`
   This script an optional `-z` column name argument to color-code (using absolute value of the data) the plot, defaulting to `steer_angle`.


## Developer Resources

### Other Files
Here is a short description of other files provided, that you **do not need to modify**.
- [vrep.py](vrep.py), [vrepConst.py](vrepConst.py), [vrepInterface.py](vrepInterface.py) are API files provided by V-REP.
  - [vrep.py](vrep.py) has been modified to try to auto-detect your V-REP installation location to automatically determine the API shared library location.
    This only works on some platforms right now.
- [simpleCsvDict.py](simpleCsvDict.py) provides a modified csv.DictWriter-like class that encapsulates both Python2 and Python3 styles for opening CSV files, and does not require fieldnames to be provided separately.
- [carInterface.py](carInterface.py) provides a higher-level API for controlling the simulated car.

### Static checking
The majority of this code has mypy static type annotations.
If you have mypy installed (can be done via `pip`) you can typecheck the code using:   
```
dmypy run -- --follow-imports=error --disallow-untyped-defs --disallow-incomplete-defs --check-untyped-defs -p controller -p carInterface -p simpleCsvDict
```
