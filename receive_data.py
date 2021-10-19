#!/usr/bin/env python

import sys
import ctypes
import struct
import asyncio
import websockets

class TelemetryData(ctypes.Structure):
    _fields_ = [
        ("rc_throttle", ctypes.c_short),
        ("rc_roll", ctypes.c_short),
        ("rc_pitch", ctypes.c_short),
        ("rc_yaw", ctypes.c_short),
        ("rc_aux1", ctypes.c_short),
        ("rc_aux2", ctypes.c_short),
        ("roll", ctypes.c_float),
        ("pitch", ctypes.c_float),
        ("yaw", ctypes.c_float),
        ("acc_x", ctypes.c_float),
        ("acc_y", ctypes.c_float),
        ("acc_z", ctypes.c_float),
        ("w_x", ctypes.c_float),
        ("w_y", ctypes.c_float),
        ("w_z", ctypes.c_float),
        ("w_t", ctypes.c_float),
        ("ax_x", ctypes.c_float),
        ("ax_y", ctypes.c_float),
        ("ax_z", ctypes.c_float),
        ("ax_t", ctypes.c_float),
    ]


async def sdrone():
  try:
    async with websockets.connect("ws://192.168.4.1:80/ws", ping_interval=100, ping_timeout=100) as websocket:
#    async with websockets.connect("ws://192.168.4.1:80/ws") as websocket:
        await websocket.send("Hello world!")
        while True:
          resp = await websocket.recv()
          x = TelemetryData()
          fmt = "<hhhhhhffffffffffffff"
          fmt_size = struct.calcsize(fmt)
          x.rc_throttle, x.rc_roll, x.rc_pitch, x.rc_yaw, x.rc_aux1, x.rc_aux2, x.roll, x.pitch, x.yaw, x.acc_x, x.acc_y, x.acc_z, x.u_x, x.u_y, x.u_z, x.u_t, x.ax_x, x.ax_y, x.ax_z, x.ax_t = struct.unpack(fmt, resp[:fmt_size])
          print("rc.: [{:d} {:d} {:d} {:d} {:d} {:d}]".format(x.rc_throttle, x.rc_roll, x.rc_pitch, x.rc_yaw, x.rc_aux1, x.rc_aux2))
          print("rpy: [{:f} {:f} {:f}]".format(x.roll, x.pitch, x.yaw))
          print("acc: [{:f} {:f} {:f}]".format(x.acc_x, x.acc_y, x.acc_z))
          print("w..: [{:f} {:f} {:f} {:f}]".format(x.u_x, x.u_y, x.u_z, x.u_t))
          print("ax.: [{:f} {:f} {:f} {:f}]".format(x.ax_x, x.ax_y, x.ax_z, x.ax_t))
        await websocket.close()
  except:
     print("Exception received");

#for x in range(100000):
asyncio.run(sdrone())

