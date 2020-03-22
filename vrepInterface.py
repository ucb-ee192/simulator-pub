import os.path
import sys

import vrep

class VRepAPIError(Exception):
  pass

class VRepInterfaceError(Exception):
  pass

class VRepInterfaceFactory(object):
  def __init__(self, connection_addr, connection_port, timeout, thread_period):
    self.connection_addr = connection_addr
    self.connection_port = connection_port
    self.timeout = timeout
    self.thread_period = thread_period
    
  def __enter__(self):
    """Opens a V-REP API connection or dies trying.
    """
    client_id = vrep.simxStart(self.connection_addr, self.connection_port, 
                               True, True, self.timeout, self.thread_period)
    if client_id == -1:
      raise VRepAPIError("Connection failed")
    
    self.vrep_interface = VRepInterface(client_id)
    return self.vrep_interface
    
  def __exit__(self, type, value, traceback):
    self.vrep_interface.close()

class VRepInterface(object):
  """A more Pythonic V-REP interface, storing state data (like client ID) in an
  object and using exceptions to avoid checking returns everywhere.
  """
  def __init__(self, client_id):
    assert client_id != -1
    self.client_id = client_id
    pass

  @staticmethod
  def open(connection_addr='127.0.0.1', connection_port=19997, 
           timeout=500, thread_period=5):
    return VRepInterfaceFactory(connection_addr, connection_port, timeout,
                                thread_period)

  def __getattr__(self, name):
    """Pass-through for V-Rep API functions, removing the need to pass in a
    clientID everywhere and automatically checking the return code. Should work
    for the vast majority of V-Rep API functions...
    """
    if not hasattr(vrep, name):
      raise VRepInterfaceError("V-Rep API has no function '%s'" % name)
    
    vrep_fn = getattr(vrep, name)
    
    def inner(*args, **kwargs):
      args = (self.client_id, ) + args
      result_packed = vrep_fn(*args, **kwargs)
      
      if isinstance(result_packed, tuple):
        result_code = result_packed[0]
        result = result_packed[1:]
        if len(result) == 1:
          result = result[0]  
      elif isinstance(result_packed, int):
        result_code = result_packed
        result = None
      else:
        raise VRepInterfaceError("Unexpected result format from API call '%s' (args=%s, kwargs=%s), got %s"
                                 % (name, args, kwargs, result_packed))

      if (result_code != vrep.simx_return_ok 
          and result_code != vrep.simx_return_novalue_flag):
        # TODO: what circumstances are the second not an error in?!
        raise VRepAPIError("API call '%s' (args=%s, kwargs=%s), failed (got %i)"
                           % (name, args, kwargs, result_code))
      return result
    
    return inner
  
  def close(self):
    """Cleans up the V-Rep API connection. Call this when done.
    """
    if self.client_id != -1:
      vrep.simxFinish(self.client_id)

  #
  # Helper functions to abstract away lower-level functionality
  #
  def get_bounding_box(self, name):
    handle = self.simxGetObjectHandle(name, vrep.simx_opmode_oneshot_wait)
    xmin = self.simxGetObjectFloatParameter(handle, 15, 
                                            vrep.simx_opmode_oneshot_wait)
    ymin = self.simxGetObjectFloatParameter(handle, 16, 
                                            vrep.simx_opmode_oneshot_wait)
    zmin = self.simxGetObjectFloatParameter(handle, 17, 
                                            vrep.simx_opmode_oneshot_wait)
    xmax = self.simxGetObjectFloatParameter(handle, 18, 
                                            vrep.simx_opmode_oneshot_wait)
    ymax = self.simxGetObjectFloatParameter(handle, 19, 
                                            vrep.simx_opmode_oneshot_wait)
    zmax = self.simxGetObjectFloatParameter(handle, 20, 
                                            vrep.simx_opmode_oneshot_wait)
    return ((xmin, xmax), (ymin, ymax), (zmin, zmax))

  def get_bounding_size(self, name):
    bbox = self.get_bounding_box(name)
    return (bbox[0][1]-bbox[0][0], bbox[1][1]-bbox[1][0], bbox[2][1]-bbox[2][0])
