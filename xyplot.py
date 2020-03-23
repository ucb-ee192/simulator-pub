from abc import abstractmethod
from typing import List, Any, Dict, Tuple, FrozenSet  # need to not alias OrderedDict
from collections import OrderedDict

import csv
import numpy as np  # type: ignore
import matplotlib.pyplot as plt  # type: ignore


if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser(description='CSV Telemetry / Logger Visualizer')

  parser.add_argument('filename',
                      help='filename of CSV to open')
  parser.add_argument('--skip_data_rows', type=int, default=0,
                      help='data rows (after the first header row) to skip')
  parser.add_argument('--x', '-x', type=str, default='x',
                      help='column name of x axis')
  parser.add_argument('--y', '-y', type=str, default='y',
                      help='column name of y axis')
  parser.add_argument('--z', '-z', type=str, default='steer_angle',
                      help='column name of z (color) axis, displayed in greyscale and absolute value')
  args = parser.parse_args()

  #
  # Parse the input CSV
  #
  with open(args.filename, newline='') as csvfile:
    reader = csv.reader(csvfile)
    names = next(reader)

    x_ind = names.index(args.x)
    y_ind = names.index(args.y)
    z_ind = names.index(args.z)
    xs: List[float] = []
    ys: List[float] = []
    zs: List[float] = []

    try:
      for i in range(args.skip_data_rows):  # skip skipped rows
        next(reader)

      data_row_idx = 0
      print(f"working: parsed {data_row_idx} rows", end='\r')
      while True:
        data_row = next(reader)
        xs.append(float(data_row[x_ind]))
        ys.append(float(data_row[y_ind]))
        zs.append(float(data_row[z_ind]))

        data_row_idx += 1
        if data_row_idx % 1000 == 0:
          print(f"working: parsed {data_row_idx} rows", end='\r')
    except StopIteration:
      print(f"finished: parsed {data_row_idx} rows")

  #
  # Postprocess z-axis (color) to absolute value
  #
  zs = [abs(z) for z in zs]

  #
  # Render graphs
  #
  print(f"working: rendering", end='\r')
  sc = plt.scatter(xs, ys, c=zs, s=1)
  plt.colorbar(sc)
  print(f"finished: rendered ")

  plt.subplots_adjust(bottom=0.001, left=0.001, top=0.999, right=0.999)  # remove extraneous whitespace around plot
  plt.show()
