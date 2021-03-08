from abc import abstractmethod
from typing import List, Any, Dict, Tuple, FrozenSet  # need to not alias OrderedDict
from collections import OrderedDict

import csv
import numpy as np  # type: ignore
import matplotlib.pyplot as plt  # type: ignore


class BasePlot:
  @abstractmethod
  def add_cell(self, indep_val: float, data: str) -> None:
    raise NotImplementedError()

  @abstractmethod
  def render(self, subplot: Any) -> None:
    raise NotImplementedError()


class LinePlot(BasePlot):
  def __init__(self) -> None:
    self.x_values: List[float] = []
    self.y_values: List[float] = []

  def add_cell(self, indep_val: float, data: str) -> None:
    self.x_values.append(indep_val)
    self.y_values.append(float(data))

  def render(self, subplot: Any) -> None:
    subplot.plot(self.x_values, self.y_values)


class WaterfallPlot(BasePlot):
  def __init__(self) -> None:
    self.x_values: List[float] = []
    self.y_values: List[List[float]] = []

  def add_cell(self, indep_val: float, data: str) -> None:
    arr_data = [float(arr_elt) for arr_elt in data[1:-1].split(',')]
    if self.y_values:
      assert len(arr_data) == len(self.y_values[0])

    self.x_values.append(indep_val)
    self.y_values.append(arr_data)

  def render(self, subplot: Any) -> None:
    # note, mesh is the fencepost surrounding the data - so these must be 1 larger in both dimensions than the values
    arr_len = len(self.y_values[0])
    val_len = len(self.y_values)
    x_mesh: List[List[float]] = []
    y_mesh: List[List[float]] = [[i - 0.5 for i in range(arr_len + 1)]] * (val_len + 1)

    # generate the x_mesh from x_values
    if val_len == 0:
      return
    elif val_len == 1:  # fencepost with arbitrary size of unit 1
      x_point = self.x_values[0]
      x_mesh.append([x_point - 0.5] * (arr_len + 1))
      x_mesh.append([x_point + 0.5] * (arr_len + 1))
    else:
      lower_x_size = self.x_values[1] - self.x_values[0]
      x_mesh.append([self.x_values[0] - lower_x_size / 2] * (arr_len + 1))
      for i in range(val_len - 1):
        x_mesh.append([(self.x_values[i+1] + self.x_values[i]) / 2] * (arr_len + 1))
      upper_x_size = self.x_values[-1] - self.x_values[-2]
      x_mesh.append([self.x_values[-1] + upper_x_size / 2] * (arr_len + 1))

    subplot.pcolorfast(np.array(x_mesh), np.array(y_mesh), np.array(self.y_values),
                       cmap='gray', interpolation='None')


def str_is_float(input: str) -> bool:
  if not input:  # TODO: this is a bit hacky, we default empty cell as float
    return True
  try:
    float(input)
    return True
  except ValueError:
    return False


def str_is_array(input: str) -> bool:
  return len(input) > 1 and input[0] == '[' and input[-1] == ']'


if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser(description='CSV Telemetry / Logger Visualizer')

  parser.add_argument('filename',
                      help='filename of CSV to open, the first column is treated as the independent axis')
  parser.add_argument('--merge', '-m', action='append', default=[],
                      help='column names to merge for each plot, comma-separated without spaces, '
                           'can be specified multiple times, eg "-m camera,line -m kp,kd"')
  parser.add_argument('--skip_data_rows', type=int, default=0,
                      help='data columns to skip')
  args = parser.parse_args()

  #
  # Parse the input CSV
  #
  first_x = 0.0
  last_x = 0.0
  with open(args.filename, newline='') as csvfile:
    reader = csv.reader(csvfile)
    names = next(reader)

    try:
      for i in range(args.skip_data_rows):  # skip skipped rows
        next(reader)

      data_row = next(reader)  # infer data type from first row
      plots: List[BasePlot] = []
      for col_name, data_cell in zip(names[1:], data_row[1:]):  # discard first col
        if str_is_float(data_cell):
          print(f"detected numeric / line plot for '{col_name}'")
          plot: BasePlot = LinePlot()
        elif str_is_array(data_cell):
          print(f"detected array / waterfall plot for '{col_name}'")
          plot = WaterfallPlot()
        else:
          raise ValueError(f"Unable to infer data type for '{col_name}' from data contents '{data_cell}'")
        plots.append(plot)

      data_row_idx = 0
      first_x = float(data_row[0])
      print(f"working: parsed {data_row_idx} rows", end='\r')
      while True:
        indep_value = float(data_row[0])
        last_x = indep_value
        for data_col_idx, data_cell in enumerate(data_row[1:]):  # discard first col
          if data_cell:
            plots[data_col_idx].add_cell(indep_value, data_cell)

        data_row_idx += 1
        if data_row_idx % 1000 == 0:
          print(f"working: parsed {data_row_idx} rows", end='\r')

        data_row = next(reader)

    except StopIteration:
      print(f"finished: parsed {data_row_idx} rows")

  #
  # Build plots
  #
  merge_sets = [frozenset(arg.split(',')) for arg in args.merge]  # use frozenset since it's hashable
  merge_dict: Dict[str, FrozenSet[str]] = {}
  for merge_set in merge_sets:
    for merge_item in merge_set:
      merge_dict[merge_item] = merge_set

  merged_plots: 'OrderedDict[FrozenSet[str], List[Tuple[str, BasePlot]]]' = OrderedDict()
  for col_name, plot in zip(names[1:], plots):
    simple_col_name = col_name.split(' ')[0]
    if col_name in merge_dict:
      key = merge_dict[col_name]
    elif simple_col_name in merge_dict:  # allow taking the short name
      key = merge_dict[simple_col_name]
    else:
      key = frozenset([col_name])  # non-merged, use name as key

    merged_plots.setdefault(key, []).append((col_name, plot))

  #
  # Render graphs
  #
  print(f"working: rendering", end='\r')
  figure, axs = plt.subplots(len(merged_plots), 1, sharex='all', frameon=False)
  if len(merged_plots) == 1:  # unify special case of single Axes to list
    axs = [axs]

  figure.tight_layout()
  figure.subplots_adjust(wspace=0, hspace=0)
  for ax in axs:
    ax.tick_params(axis='y', direction='in', pad=-25)
    ax.tick_params(axis='x', direction='in', pad=-15)

  for plot_idx, (key, name_plots) in enumerate(merged_plots.items()):
    ax = axs[plot_idx]
    ax.set_xlim([first_x, last_x])
    ax.text(0.5, 1.0, ", ".join([name for (name, plot) in name_plots]),
            horizontalalignment='center', verticalalignment='top', transform=ax.transAxes)
    for name, plot in name_plots:
      print(f"working: rendering {name}{' '*(30 - len(name))}", end='\r')  # TODO arbitrary 30-char name "limit"
      plot.render(ax)

  print(f"finished: rendered {len(merged_plots)} plots{' '*30}")

  plt.subplots_adjust(bottom=0.001, left=0.001, top=0.999, right=0.999)  # remove extraneous whitespace around plot
  plt.show()
