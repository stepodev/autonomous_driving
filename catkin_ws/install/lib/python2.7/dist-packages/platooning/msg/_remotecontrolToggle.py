# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from platooning/remotecontrolToggle.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class remotecontrolToggle(genpy.Message):
  _md5sum = "8f56e4c7d94d4f505d773a5b64c0ffed"
  _type = "platooning/remotecontrolToggle"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint32 vehicle_id
bool enable_remotecontrol"""
  __slots__ = ['vehicle_id','enable_remotecontrol']
  _slot_types = ['uint32','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       vehicle_id,enable_remotecontrol

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(remotecontrolToggle, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.vehicle_id is None:
        self.vehicle_id = 0
      if self.enable_remotecontrol is None:
        self.enable_remotecontrol = False
    else:
      self.vehicle_id = 0
      self.enable_remotecontrol = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_IB().pack(_x.vehicle_id, _x.enable_remotecontrol))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 5
      (_x.vehicle_id, _x.enable_remotecontrol,) = _get_struct_IB().unpack(str[start:end])
      self.enable_remotecontrol = bool(self.enable_remotecontrol)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_IB().pack(_x.vehicle_id, _x.enable_remotecontrol))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 5
      (_x.vehicle_id, _x.enable_remotecontrol,) = _get_struct_IB().unpack(str[start:end])
      self.enable_remotecontrol = bool(self.enable_remotecontrol)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_IB = None
def _get_struct_IB():
    global _struct_IB
    if _struct_IB is None:
        _struct_IB = struct.Struct("<IB")
    return _struct_IB
