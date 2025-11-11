import ctypes
import enum

class Header(ctypes.BigEndianStructure):
    _fields_ = [
        ("message_id", ctypes.c_uint32),
        ("payload_length", ctypes.c_uint32),
        ("timestamp_sec", ctypes.c_uint32),
        ("timestamp_nsec", ctypes.c_uint32),
        ("reserved_0", ctypes.c_uint32),
        ("reserved_1", ctypes.c_uint32),
        ("reserved_2", ctypes.c_uint32),
        ("reserved_3", ctypes.c_uint32),
    ]

class MessageIDs(enum.IntEnum):
    iPhoneName = 0
    CameraFrame = 1
    LidarFrame = 2
    GPSMessage = 3
