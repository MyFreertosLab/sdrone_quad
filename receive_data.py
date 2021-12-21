#!/usr/bin/env python

import sys
import ctypes
import struct
import asyncio
import websockets
import traceback

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

def urot(x, resp):
  fmt = "<hhIfff"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryData3Df();
  k.type, k.dummy, k.timestamp, k.x, k.y,k.z = struct.unpack(fmt, resp[:fmt_size])
  print("urot: [{:d} {:f} {:f} {:f}]".format(k.timestamp, k.x, k.y, k.z))
def rpy(x, resp):
  fmt = "<hhIfff"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryData3Df();
  k.type, k.dummy, k.timestamp, k.x, k.y,k.z = struct.unpack(fmt, resp[:fmt_size])
  print("rpy: [{:d} {:f} {:f} {:f}]".format(k.timestamp, k.x, k.y, k.z))
def alfa(x, resp):
  fmt = "<hhIfff"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryData3Df();
  k.type, k.dummy, k.timestamp, k.x, k.y,k.z = struct.unpack(fmt, resp[:fmt_size])
  print("alfa: [{:d} {:f} {:f} {:f}]".format(k.timestamp, k.x, k.y, k.z))
def ealfa(x, resp):
  fmt = "<hhIfff"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryData3Df();
  k.type, k.dummy, k.timestamp, k.x, k.y,k.z = struct.unpack(fmt, resp[:fmt_size])
  print("ealfa: [{:d} {:f} {:f} {:f}]".format(k.timestamp, k.x, k.y, k.z))
def uacc(x, resp):
  fmt = "<hhIfff"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryData3Df();
  k.type, k.dummy, k.timestamp, k.x, k.y,k.z = struct.unpack(fmt, resp[:fmt_size])
  print("uacc: [{:d} {:f} {:f} {:f}]".format(k.timestamp, k.x, k.y, k.z))
def acc(data, resp):
  fmt = "<hhIfff"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryData3Df();
  k.type, k.dummy, k.timestamp, k.x, k.y,k.z = struct.unpack(fmt, resp[:fmt_size])
  print("acc: [{:d} {:f} {:f} {:f}]".format(k.timestamp, k.x, k.y, k.z))
def speed(data, resp):
  fmt = "<hhIfff"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryData3Df();
  k.type, k.dummy, k.timestamp, k.x, k.y,k.z = struct.unpack(fmt, resp[:fmt_size])
  print("speed: [{:d} {:f} {:f} {:f}]".format(k.timestamp, k.x, k.y, k.z))
def eacc(data, resp):
  fmt = "<hhIfff"
  fmt_size = struct.calcsize(fmt)
  k = TelemetryData3Df();
  k.type, k.dummy, k.timestamp, k.x, k.y,k.z = struct.unpack(fmt, resp[:fmt_size])
  print("eacc: [{:d} {:f} {:f} {:f}]".format(k.timestamp, k.x, k.y, k.z))

async def sdrone():
  try:
    async with websockets.connect("ws://192.168.4.1:80/telemetry", ping_interval=400, ping_timeout=500) as websocket:
#    async with websockets.connect("ws://192.168.4.1:80/telemetry") as websocket:
        await websocket.send("Hello world!")
        while True:
          resp = await websocket.recv()
          data = TelemetryData()
          options = {
            1: urot,
            2: rpy,
            3: alfa,
            4: ealfa,
            5: uacc,
            6: speed,
            7: acc,
            8: eacc,
          }
          fmt = "<h18s"
          fmt_size = struct.calcsize(fmt)
          temp = "";
          data.type, temp = struct.unpack(fmt, resp[:fmt_size])
          options[data.type](data, resp);
        await websocket.close()
  except Exception:
     traceback.print_exc()
#  except Exception as e:
#     print(e);
#     print("Exception received");

#for x in range(100000):
asyncio.run(sdrone())

