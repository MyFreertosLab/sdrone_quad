ESP32 ESP-IDF Drone
===================

This is a Drone Implementation with esp-idf and esp32.
This software is under construction, but it works.

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
2) RC TX/RX (I use FlySky I-BUS, but works well with PPM also)
3) PPM Encoder (if you not use FlySky I-Bus)
4) IMU (mpu9250) with at least accelerometer and gyroscope
5) Battery
6) Esc (30Ah)
7) Motors
8) Propellers
9) The Board (a simple ESP32 board)
10) Cup of coffee

<h1>The first test</h1>

[![Watch the video](http://img.youtube.com/vi/N0eg7YZjhg0/0.jpg)](https://youtube.com/shorts/mvMZztF52lA?feature=share)
