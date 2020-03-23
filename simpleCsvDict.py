from typing import Any, List, Dict, Optional, Union, TextIO, BinaryIO
import sys
import csv


class BufferedCsvDictWriter():
  def __init__(self, filename: str):
    if sys.version_info.major < 3:  # hack to allow Python 2 and 3 compatibility
      self.file: Optional[Union[TextIO, BinaryIO]] = open(filename, 'wb')
    else:
      self.file = open(filename, 'w', newline='')
    self.rows: List[Dict[str, Any]] = []
    self.fieldnames: List[str] = []

  def writerow(self, row: Dict[str, Any]) -> None:
    assert self.file is not None, "can't continue writing after close()"
    self.rows.append(row)
    for field in row.keys():
      if field not in self.fieldnames:
        self.fieldnames.append(field)

  def close(self) -> None:
    assert self.file is not None, "can't close after close()"

    dictwriter = csv.DictWriter(self.file, fieldnames=self.fieldnames)
    dictwriter.writeheader()
    for row in self.rows:
      dictwriter.writerow(row)

    self.file.close()
    self.file = None
