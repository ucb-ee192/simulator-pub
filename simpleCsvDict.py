from typing import Any, Set, Dict, Optional, Union, TextIO, BinaryIO
import sys
import csv


class SimpleCsvDictWriter():
  def __init__(self, filename: str):
    if sys.version_info.major < 3:  # hack to allow Python 2 and 3 compatibility
      self.file: Optional[Union[TextIO, BinaryIO]] = open(filename, 'wb')
    else:
      self.file = open(filename, 'w', newline='')
    self.csv: Optional[csv.DictWriter] = None
    self.fieldnames: Optional[Set[str]] = None

  def writerow(self, row: Dict[str, Any]) -> None:
    assert self.file is not None, "can't continue writing after close()"

    if self.csv is not None:
      assert set(row.keys()) == self.fieldnames, "field names must not change"
    else:
      self.fieldnames = set(row.keys())
      self.csv = csv.DictWriter(self.file, fieldnames=row.keys())
      self.csv.writeheader()

    self.csv.writerow(row)

  def close(self) -> None:
    assert self.file is not None, "can't close after close()"

    self.file.close()
    self.file = None
    self.csv = None
