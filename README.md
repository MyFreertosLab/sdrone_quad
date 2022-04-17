ESP32 ESP-IDF Drone
===================

This is a Drone Test Implementation with esp-idf and esp32.
This software is under construction

```
  git clone --recursive https://github.com/MyFreertosLab/sdrone_quad.git
  idf.py build
  idf.py flash
```
<h1>The first step</h1>
<p align="left">
  <img src="https://github.com/MyFreertosLab/sdrone_test/blob/master/docs/images/Step1.jpg" width="300" title="The hardware">
  <p>Step 1: Minimal Hardware List</p>
</p>

1) The frame
2) RC TX/RX (I use FlySky I-BUS)
3) PPM Encoder (if you not use FlySky I-Bus)
4) IMU (mpu9250) with at least accelerometer and gyroscope
5) Battery
6) Esc (30Ah)
7) Motors
8) Propellers
9) The Board (a simple ESP32 board)
10) Cup of coffee
