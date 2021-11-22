#!/usr/bin/env python

import sys
import ctypes
import struct
import asyncio
import websockets

class TelemetryData(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_ushort),
        ("dummy", ctypes.c_byte),
    ]

class TelemetryDataf(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_ushort),
        ("dummy", ctypes.c_short),
        ("timestamp", ctypes.c_uint),
        ("v", ctypes.c_float),
    ]

class TelemetryData3Df(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_ushort),
        ("dummy", ctypes.c_short),
        ("timestamp", ctypes.c_uint),
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
    ]

class TelemetryData4Df(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_ushort),
        ("dummy", ctypes.c_short),
        ("timestamp", ctypes.c_uint),
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("t", ctypes.c_float),
    ]

class TelemetryDataRc(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_ushort),
        ("dummy", ctypes.c_short),
        ("timestamp", ctypes.c_uint),
        ("throttle", ctypes.c_short),
        ("roll", ctypes.c_short),
        ("pitch", ctypes.c_short),
        ("yaw", ctypes.c_short),
        ("aux1", ctypes.c_short),
        ("aux2", ctypes.c_short),
    ]

class TelemetryDataZ(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_ushort),
        ("dummy", ctypes.c_short),
        ("timestamp", ctypes.c_uint),
        ("rc_t",  ctypes.c_short),
        ("dummy1", ctypes.c_short),
        ("x_z",   ctypes.c_float),
        ("u_z",   ctypes.c_float),
        ("w_z",   ctypes.c_float),
        ("y_z",   ctypes.c_float),
        ("v_z",   ctypes.c_float),
    ]

def z(x, resp):
  fmt = "<hhIhhfffff"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryDataZ();
  k.type, k.dummy, k.timestamp, k.rc_t, k.dummy1, k.x_z,k.u_z,k.w_z,k.y_z,k.v_z= struct.unpack(fmt, resp[:fmt_size])
  print("z..: [{:d} {:d} {:f} {:f} {:f} {:f} {:f}]".format(k.timestamp, k.rc_t, k.x_z, k.u_z, k.w_z, k.y_z, k.v_z))
def rc(x, resp):
  fmt = "<hhIhhhhhh"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryDataRc();
  k.type, k.dummy, k.timestamp, k.throttle, k.roll,k.pitch,k.yaw,k.aux1,k.aux2= struct.unpack(fmt, resp[:fmt_size])
  print("rc.: [{:d} {:d} {:d} {:d} {:d} {:d} {:d}]".format(k.timestamp, k.throttle, k.roll, k.pitch, k.yaw, k.aux1, k.aux2))
def rpy(x, resp):
  fmt = "<hhIfff"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryData3Df();
  k.type, k.dummy, k.timestamp, k.x, k.y,k.z = struct.unpack(fmt, resp[:fmt_size])
  print("rpy: [{:d} {:f} {:f} {:f}]".format(k.timestamp, k.x, k.y, k.z))
def acc(x, resp):
  fmt = "<hhIfff"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryData3Df();
  k.type, k.dummy, k.timestamp, k.x, k.y,k.z = struct.unpack(fmt, resp[:fmt_size])
  print("acc: [{:d} {:f} {:f} {:f}]".format(k.timestamp, k.x, k.y, k.z))
def x(data, resp):
  fmt = "<hhIffff"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryData4Df();
  k.type, k.dummy, k.timestamp, k.x, k.y,k.z,k.t = struct.unpack(fmt, resp[:fmt_size])
  print("x..: [{:d} {:f} {:f} {:f} {:f}]".format(k.timestamp, k.x, k.y, k.z, k.t))
def u(x, resp):
  fmt = "<hhIffff"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryData4Df();
  k.type, k.dummy, k.timestamp, k.x, k.y,k.z,k.t = struct.unpack(fmt, resp[:fmt_size])
  print("u..: [{:d} {:f} {:f} {:f} {:f}]".format(k.timestamp, k.x, k.y, k.z, k.t))
def w(x, resp):
  fmt = "<hhIffff"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryData4Df();
  k.type, k.dummy, k.timestamp, k.x, k.y,k.z,k.t = struct.unpack(fmt, resp[:fmt_size])
  print("w..: [{:d} {:f} {:f} {:f} {:f}]".format(k.timestamp, k.x, k.y, k.z, k.t))
def y(x, resp):
  fmt = "<hhIffff"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryData4Df();
  k.type, k.dummy, k.timestamp, k.x, k.y,k.z,k.t = struct.unpack(fmt, resp[:fmt_size])
  print("y..: [{:d} {:f} {:f} {:f} {:f}]".format(k.timestamp, k.x, k.y, k.z, k.t))
def axis(x, resp):
  fmt = "<hhIffff"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryData4Df();
  k.type, k.dummy, k.timestamp, k.x, k.y,k.z,k.t = struct.unpack(fmt, resp[:fmt_size])
  print("ax.: [{:d} {:f} {:f} {:f} {:f}]".format(k.timestamp, k.x, k.y, k.z, k.t))
def gravity(x, resp):
  fmt = "<hhIfff"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryData3Df();
  k.type, k.dummy, k.timestamp, k.x, k.y,k.z = struct.unpack(fmt, resp[:fmt_size])
  print("gr.: [{:d} {:f} {:f} {:f}]".format(k.timestamp, k.x, k.y, k.z))

def vertical_v(x, resp):
  fmt = "<hhIf"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryDataf();
  k.type, k.dummy, k.timestamp, k.v = struct.unpack(fmt, resp[:fmt_size])
  print("vv.: [{:d} {:f}]".format(k.timestamp, k.v))

async def sdrone():
#  try:
    async with websockets.connect("ws://192.168.4.1:80/ws", ping_interval=300, ping_timeout=300) as websocket:
#    async with websockets.connect("ws://192.168.4.1:80/ws") as websocket:
        await websocket.send("Hello world!")
        while True:
          resp = await websocket.recv()
          data = TelemetryData()
          options = {
            1: rc,
            2: rpy,
            3: acc,
            4: w,
            5: axis,
            6: gravity,
            7: vertical_v,
            8: x,
            9: u,
            10: y,
            11: z,
          }
          fmt = "<h18s"
          fmt_size = struct.calcsize(fmt)
          temp = "";
          data.type, temp = struct.unpack(fmt, resp[:fmt_size])
          options[data.type](data, resp);
        await websocket.close()
#  except:
#     print("Exception received");

#for x in range(100000):
asyncio.run(sdrone())

