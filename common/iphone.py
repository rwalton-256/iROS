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

class IMU(ctypes.BigEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("roll",  ctypes.c_double),
        ("pitch", ctypes.c_double),
        ("yaw",   ctypes.c_double),
        ("rotx",   ctypes.c_double),
        ("roty",   ctypes.c_double),
        ("rotz",   ctypes.c_double),
        ("gx",   ctypes.c_double),
        ("gy",   ctypes.c_double),
        ("gz",   ctypes.c_double),
        ("accx",   ctypes.c_double),
        ("accy",   ctypes.c_double),
        ("accz",   ctypes.c_double),
        ("magx",   ctypes.c_double),
        ("magy",   ctypes.c_double),
        ("magz",   ctypes.c_double),
        ("magacc",   ctypes.c_uint32),
    ]


class MessageIDs(enum.IntEnum):
    iPhoneName = 0
    CameraFrame = 1
    LidarFrame = 2
    IMUMessage = 3
    GPSMessage = 4
