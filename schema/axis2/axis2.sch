EESchema Schematic File Version 4
LIBS:axis2-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L axis2-rescue:Conn_02x10_Odd_Even-Connector_Generic J1
U 1 1 6072B308
P 1900 4400
F 0 "J1" H 1950 5017 50  0000 C CNN
F 1 "JA" H 1950 4926 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x10_P2.54mm_Vertical" H 1900 4400 50  0001 C CNN
F 3 "~" H 1900 4400 50  0001 C CNN
	1    1900 4400
	1    0    0    -1  
$EndComp
$Comp
L axis2-rescue:Conn_02x10_Odd_Even-Connector_Generic J2
U 1 1 6072B4E0
P 5400 4400
F 0 "J2" H 5450 5017 50  0000 C CNN
F 1 "JB" H 5450 4926 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x10_P2.54mm_Vertical" H 5400 4400 50  0001 C CNN
F 3 "~" H 5400 4400 50  0001 C CNN
	1    5400 4400
	1    0    0    -1  
$EndComp
$Comp
L gy-9250:gy-9250 U1
U 1 1 6073C1B2
P 3800 4500
F 0 "U1" H 3800 5400 50  0000 L CNN
F 1 "gy-9250" V 4100 4450 50  0000 L CNN
F 2 "gy-9250_footprint:gy-9250" H 3800 4750 50  0001 C CNN
F 3 "" H 3800 4750 50  0001 C CNN
	1    3800 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 4600 1600 4600
Text GLabel 3250 4700 0    50   Output ~ 0
MISO
Text GLabel 1550 4600 0    50   Input ~ 0
MISO
Wire Wire Line
	1550 4700 1600 4700
Text GLabel 3250 4250 0    50   Input ~ 0
MOSI
Text GLabel 2700 4600 2    50   Output ~ 0
SCL
Text GLabel 1550 4700 0    50   Output ~ 0
MOSI
Text GLabel 1500 4000 0    50   Output ~ 0
+3V3
Wire Wire Line
	1500 4000 1550 4000
Text GLabel 3250 3800 0    50   Input ~ 0
+3V3
Text GLabel 5700 4700 2    50   Output ~ 0
NCS
Text GLabel 3250 5000 0    50   Input ~ 0
NCS
Text GLabel 5200 4700 0    50   Output ~ 0
INT
Text GLabel 3250 4850 0    50   Input ~ 0
INT
Text GLabel 3250 4100 0    50   Input ~ 0
SCL
NoConn ~ 3250 5150
NoConn ~ 3250 4400
NoConn ~ 3250 4550
$Comp
L axis2-rescue:CP1-Device C2
U 1 1 60758924
P 2950 1600
F 0 "C2" H 3065 1646 50  0000 L CNN
F 1 "100uF" H 3065 1555 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 2950 1600 50  0001 C CNN
F 3 "~" H 2950 1600 50  0001 C CNN
	1    2950 1600
	1    0    0    -1  
$EndComp
$Comp
L axis2-rescue:Conn_01x02_Female-Connector J3
U 1 1 60759B12
P 3350 1200
F 0 "J3" V 3290 1012 50  0000 R CNN
F 1 "5VPower" V 3400 1250 50  0000 R CNN
F 2 "Connector_JST:JST_XH_B2B-XH-AM_1x02_P2.50mm_Vertical" H 3350 1200 50  0001 C CNN
F 3 "~" H 3350 1200 50  0001 C CNN
	1    3350 1200
	0    -1   -1   0   
$EndComp
Text Label 3150 3950 2    50   ~ 0
GND
Wire Wire Line
	3250 3950 3150 3950
Text Label 5950 4500 2    50   ~ 0
GND
Wire Wire Line
	5700 4500 5750 4500
Text GLabel 3450 1750 3    50   Input ~ 0
+5V
Text Label 3350 1750 3    50   ~ 0
GND
Text Label 2950 2050 1    50   ~ 0
GND
Wire Wire Line
	2950 1750 2950 2050
Text GLabel 2950 1350 1    50   Input ~ 0
+5V
Wire Wire Line
	2950 1450 2950 1350
Text Notes 4450 2200 0    50   ~ 0
Power 5V
Text Notes 3400 5900 0    50   ~ 0
Controller Board
$Comp
L Connector_Generic:Conn_01x03 J7
U 1 1 6075F9C4
P 5900 1250
F 0 "J7" H 5980 1292 50  0000 L CNN
F 1 "M3" H 5980 1201 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 5900 1250 50  0001 C CNN
F 3 "~" H 5900 1250 50  0001 C CNN
	1    5900 1250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J8
U 1 1 6075FA63
P 6950 1250
F 0 "J8" H 7030 1292 50  0000 L CNN
F 1 "M2" H 7030 1201 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 6950 1250 50  0001 C CNN
F 3 "~" H 6950 1250 50  0001 C CNN
	1    6950 1250
	1    0    0    -1  
$EndComp
Text Notes 6550 2750 0    50   ~ 0
Motors Control
Wire Wire Line
	5550 1250 5700 1250
Wire Wire Line
	6550 1250 6750 1250
Text GLabel 5800 4300 2    50   Output ~ 0
M3S
Wire Wire Line
	5700 4300 5750 4300
Text GLabel 5550 1350 0    50   Input ~ 0
M3S
Wire Wire Line
	5550 1350 5700 1350
Text GLabel 6550 1350 0    50   Input ~ 0
M2S
Wire Wire Line
	6550 1350 6750 1350
Text GLabel 5100 4400 0    50   Output ~ 0
M2S
Wire Wire Line
	5200 4400 5150 4400
Wire Wire Line
	3450 1800 3500 1800
Text GLabel 5800 4000 2    50   Input ~ 0
RCS
Wire Wire Line
	5800 4000 5750 4000
Text GLabel 1250 1200 0    50   Input ~ 0
RCS
Wire Notes Line
	800  800  800  1350
Wire Notes Line
	800  1350 2450 1350
Wire Notes Line
	2450 1350 2450 800 
Wire Notes Line
	2450 800  800  800 
Text Notes 2200 1250 0    50   ~ 0
RC
$Comp
L Connector_Generic:Conn_01x03 J9
U 1 1 60762012
P 1650 1100
F 0 "J9" H 1730 1142 50  0000 L CNN
F 1 "RC-PPM" H 1730 1051 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 1650 1100 50  0001 C CNN
F 3 "~" H 1650 1100 50  0001 C CNN
	1    1650 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 1200 1450 1200
Text Label 1200 1000 2    50   ~ 0
GND
Wire Wire Line
	1200 1000 1450 1000
Text GLabel 1250 1100 0    50   Input ~ 0
+5V
Wire Wire Line
	1250 1100 1450 1100
Text Label 5450 1150 2    50   ~ 0
GND
Wire Wire Line
	5450 1150 5700 1150
Text Label 6500 1150 2    50   ~ 0
GND
Wire Wire Line
	6500 1150 6750 1150
$Comp
L Connector_Generic:Conn_01x03 J12
U 1 1 60FB4339
P 5900 1800
F 0 "J12" H 5980 1842 50  0000 L CNN
F 1 "M4" H 5980 1751 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 5900 1800 50  0001 C CNN
F 3 "~" H 5900 1800 50  0001 C CNN
	1    5900 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 1800 5700 1800
Text GLabel 5550 1900 0    50   Input ~ 0
M4S
Wire Wire Line
	5550 1900 5700 1900
Text Label 5450 1700 2    50   ~ 0
GND
Wire Wire Line
	5450 1700 5700 1700
$Comp
L Connector_Generic:Conn_01x03 J14
U 1 1 60FB4D31
P 6950 1800
F 0 "J14" H 7030 1842 50  0000 L CNN
F 1 "M1" H 7030 1751 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 6950 1800 50  0001 C CNN
F 3 "~" H 6950 1800 50  0001 C CNN
	1    6950 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 1800 6750 1800
Text GLabel 6600 1900 0    50   Input ~ 0
M1S
Wire Wire Line
	6600 1900 6750 1900
Text Label 6500 1700 2    50   ~ 0
GND
Wire Wire Line
	6500 1700 6750 1700
$Comp
L Connector_Generic:Conn_01x03 J13
U 1 1 60FB5923
P 5900 2350
F 0 "J13" H 5980 2392 50  0000 L CNN
F 1 "M5" H 5980 2301 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 5900 2350 50  0001 C CNN
F 3 "~" H 5900 2350 50  0001 C CNN
	1    5900 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 2350 5700 2350
Text GLabel 5550 2450 0    50   Input ~ 0
M5S
Wire Wire Line
	5550 2450 5700 2450
Text Label 5450 2250 2    50   ~ 0
GND
Wire Wire Line
	5450 2250 5700 2250
$Comp
L Connector_Generic:Conn_01x03 J15
U 1 1 60FB592F
P 6950 2350
F 0 "J15" H 7030 2392 50  0000 L CNN
F 1 "M6" H 7030 2301 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 6950 2350 50  0001 C CNN
F 3 "~" H 6950 2350 50  0001 C CNN
	1    6950 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 2350 6750 2350
Text GLabel 6600 2450 0    50   Input ~ 0
M6S
Wire Wire Line
	6600 2450 6750 2450
Text Label 6500 2250 2    50   ~ 0
GND
Wire Wire Line
	6500 2250 6750 2250
Wire Notes Line
	5250 800  5250 2800
Wire Notes Line
	5250 2800 7150 2800
Wire Notes Line
	7150 2800 7150 800 
Wire Notes Line
	7150 800  5250 800 
Text GLabel 5700 4900 2    50   Input ~ 0
+5V
Text GLabel 5800 4200 2    50   Output ~ 0
M4S
Wire Wire Line
	5700 4200 5750 4200
Text GLabel 5100 4100 0    50   Output ~ 0
M1S
Text GLabel 5800 4400 2    50   Output ~ 0
M6S
Wire Wire Line
	5700 4400 5750 4400
Text GLabel 5100 4600 0    50   Output ~ 0
M5S
Wire Wire Line
	5200 4600 5150 4600
Wire Wire Line
	5200 4100 5150 4100
$Comp
L axis4:axis4-symbol U3
U 1 1 60FC7096
P 1500 2050
F 0 "U3" H 1450 2150 50  0000 L CNN
F 1 "axis4-symbol" H 1250 2050 50  0000 L CNN
F 2 "axis4:axis4-frame" H 1500 2050 50  0001 C CNN
F 3 "" H 1500 2050 50  0001 C CNN
	1    1500 2050
	1    0    0    -1  
$EndComp
$Comp
L axis2-rescue:Conn_02x10_Odd_Even-Connector_Generic J16
U 1 1 60FD35B9
P 1900 5850
F 0 "J16" H 1950 6467 50  0000 C CNN
F 1 "JA1" H 1950 6376 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x10_P2.54mm_Vertical" H 1900 5850 50  0001 C CNN
F 3 "" H 1900 5850 50  0001 C CNN
	1    1900 5850
	1    0    0    -1  
$EndComp
$Comp
L axis2-rescue:Conn_02x10_Odd_Even-Connector_Generic J17
U 1 1 60FD39E2
P 5400 5850
F 0 "J17" H 5450 6467 50  0000 C CNN
F 1 "JB1" H 5450 6376 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x10_P2.54mm_Vertical" H 5400 5850 50  0001 C CNN
F 3 "" H 5400 5850 50  0001 C CNN
	1    5400 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 4000 1550 3850
Wire Wire Line
	1550 3850 1100 3850
Wire Wire Line
	1100 3850 1100 5450
Wire Wire Line
	1100 5450 1700 5450
Connection ~ 1550 4000
Wire Wire Line
	1550 4000 1700 4000
Wire Wire Line
	1700 4100 1150 4100
Wire Wire Line
	1150 4100 1150 5550
Wire Wire Line
	1150 5550 1700 5550
Wire Wire Line
	1700 4200 1200 4200
Wire Wire Line
	1200 4200 1200 5650
Wire Wire Line
	1200 5650 1700 5650
Wire Wire Line
	1700 4300 1250 4300
Wire Wire Line
	1250 4300 1250 5750
Wire Wire Line
	1250 5750 1700 5750
Wire Wire Line
	1700 4400 1300 4400
Wire Wire Line
	1300 4400 1300 5850
Wire Wire Line
	1300 5850 1700 5850
Wire Wire Line
	1700 4500 1350 4500
Wire Wire Line
	1350 4500 1350 5950
Wire Wire Line
	1350 5950 1700 5950
Wire Wire Line
	1600 4600 1600 4550
Wire Wire Line
	1600 4550 1400 4550
Wire Wire Line
	1400 4550 1400 6050
Wire Wire Line
	1400 6050 1700 6050
Connection ~ 1600 4600
Wire Wire Line
	1600 4600 1700 4600
Wire Wire Line
	1600 4700 1600 4650
Wire Wire Line
	1600 4650 1450 4650
Wire Wire Line
	1450 4650 1450 6150
Wire Wire Line
	1450 6150 1700 6150
Connection ~ 1600 4700
Wire Wire Line
	1600 4700 1700 4700
Wire Wire Line
	1700 4800 1500 4800
Wire Wire Line
	1500 4800 1500 6250
Wire Wire Line
	1500 6250 1700 6250
Wire Wire Line
	1700 4900 1550 4900
Wire Wire Line
	1550 4900 1550 6350
Wire Wire Line
	1550 6350 1700 6350
Wire Wire Line
	2200 4000 2250 4000
Wire Wire Line
	2250 4000 2250 5450
Wire Wire Line
	2250 5450 2200 5450
Wire Wire Line
	2200 4100 2300 4100
Wire Wire Line
	2300 4100 2300 5550
Wire Wire Line
	2300 5550 2200 5550
Wire Wire Line
	2200 4200 2350 4200
Wire Wire Line
	2350 4200 2350 5650
Wire Wire Line
	2350 5650 2200 5650
Wire Wire Line
	2200 4300 2400 4300
Wire Wire Line
	2400 5750 2200 5750
Wire Wire Line
	2200 4400 2450 4400
Wire Wire Line
	2450 4400 2450 5850
Wire Wire Line
	2450 5850 2200 5850
Wire Wire Line
	2200 4500 2500 4500
Wire Wire Line
	2500 4500 2500 5950
Wire Wire Line
	2500 5950 2200 5950
Wire Wire Line
	2200 4600 2550 4600
Wire Wire Line
	2400 4300 2400 5750
Wire Wire Line
	2550 4600 2550 6050
Wire Wire Line
	2550 6050 2200 6050
Connection ~ 2550 4600
Wire Wire Line
	2550 4600 2700 4600
Wire Wire Line
	2200 4700 2600 4700
Wire Wire Line
	2600 6150 2200 6150
Wire Wire Line
	2200 4800 2650 4800
Wire Wire Line
	2650 4800 2650 6250
Wire Wire Line
	2650 6250 2200 6250
Wire Wire Line
	2200 4900 2700 4900
Wire Wire Line
	2700 4900 2700 6350
Wire Wire Line
	2700 6350 2200 6350
Wire Wire Line
	4450 4000 4450 5450
Wire Wire Line
	4450 5450 5200 5450
Wire Wire Line
	5150 4100 5150 4050
Wire Wire Line
	5150 4050 4500 4050
Wire Wire Line
	4500 4050 4500 5550
Wire Wire Line
	4500 5550 5200 5550
Connection ~ 5150 4100
Wire Wire Line
	5150 4100 5100 4100
Wire Wire Line
	5200 4200 4550 4200
Wire Wire Line
	4550 4200 4550 5650
Wire Wire Line
	4550 5650 5200 5650
Wire Wire Line
	5200 4300 4600 4300
Wire Wire Line
	4600 4300 4600 5750
Wire Wire Line
	4600 5750 5200 5750
Wire Wire Line
	5150 4400 5150 4350
Wire Wire Line
	5150 4350 4650 4350
Wire Wire Line
	4650 4350 4650 5850
Wire Wire Line
	4650 5850 5200 5850
Connection ~ 5150 4400
Wire Wire Line
	5150 4400 5100 4400
Wire Wire Line
	5200 4500 4700 4500
Wire Wire Line
	4700 4500 4700 5950
Wire Wire Line
	4700 5950 5200 5950
Wire Wire Line
	5150 4600 5150 4550
Wire Wire Line
	5150 4550 4750 4550
Wire Wire Line
	4750 4550 4750 6050
Wire Wire Line
	4750 6050 5200 6050
Connection ~ 5150 4600
Wire Wire Line
	5150 4600 5100 4600
Wire Wire Line
	4800 4650 4800 6150
Wire Wire Line
	4800 6150 5200 6150
Wire Wire Line
	5200 4800 4850 4800
Wire Wire Line
	4850 4800 4850 6250
Wire Wire Line
	4850 6250 5200 6250
Wire Wire Line
	5200 4900 4900 4900
Wire Wire Line
	4900 4900 4900 6350
Wire Wire Line
	4900 6350 5200 6350
Wire Wire Line
	5750 4000 5750 3900
Wire Wire Line
	5750 3900 6400 3900
Wire Wire Line
	6400 3900 6400 5450
Connection ~ 5750 4000
Wire Wire Line
	5750 4000 5700 4000
Wire Wire Line
	5700 4100 6350 4100
Wire Wire Line
	6350 4100 6350 5550
Wire Wire Line
	6350 5550 5700 5550
Wire Wire Line
	5750 4200 5750 4150
Wire Wire Line
	5750 4150 6300 4150
Wire Wire Line
	6300 4150 6300 5650
Wire Wire Line
	6300 5650 5700 5650
Connection ~ 5750 4200
Wire Wire Line
	5750 4200 5800 4200
Wire Wire Line
	5750 4300 5750 4250
Wire Wire Line
	5750 4250 6250 4250
Wire Wire Line
	6250 4250 6250 5750
Wire Wire Line
	6250 5750 5700 5750
Connection ~ 5750 4300
Wire Wire Line
	5750 4300 5800 4300
Wire Wire Line
	5750 4400 5750 4350
Wire Wire Line
	5750 4350 6200 4350
Wire Wire Line
	6200 4350 6200 5450
Wire Wire Line
	6200 5850 5700 5850
Connection ~ 5750 4400
Wire Wire Line
	5750 4400 5800 4400
Wire Wire Line
	6200 5450 6200 5850
Wire Wire Line
	5750 4500 5750 4450
Wire Wire Line
	5750 4450 6150 4450
Wire Wire Line
	6150 4450 6150 5950
Wire Wire Line
	6150 5950 5700 5950
Connection ~ 5750 4500
Wire Wire Line
	5750 4500 5950 4500
Wire Wire Line
	5700 4600 6100 4600
Wire Wire Line
	6100 4600 6100 6050
Wire Wire Line
	6100 6050 5700 6050
Wire Wire Line
	5700 4700 5700 4650
Wire Wire Line
	5700 4650 6050 4650
Wire Wire Line
	6050 4650 6050 6150
Wire Wire Line
	6050 6150 5700 6150
Wire Wire Line
	5700 4800 6000 4800
Wire Wire Line
	6000 4800 6000 6250
Wire Wire Line
	6000 6250 5700 6250
Wire Wire Line
	5700 4900 5700 4850
Wire Wire Line
	5700 4850 5950 4850
Wire Wire Line
	5950 4850 5950 6350
Wire Wire Line
	5950 6350 5700 6350
Wire Wire Line
	5700 5450 6400 5450
Text GLabel 5550 1250 0    50   Output ~ 0
+5V
Text GLabel 5550 1800 0    50   Output ~ 0
+5V
Text GLabel 5550 2350 0    50   Output ~ 0
+5V
Text GLabel 6550 1250 0    50   Output ~ 0
+5V
Text GLabel 6600 1800 0    50   Output ~ 0
+5V
Text GLabel 6600 2350 0    50   Output ~ 0
+5V
Text Label 4300 2000 1    50   ~ 0
GND
Text GLabel 4300 1250 1    50   Input ~ 0
+5V
$Comp
L Diode:1N4007 D1
U 1 1 610E581C
P 4300 1550
F 0 "D1" V 4254 1630 50  0000 L CNN
F 1 "1N4007" V 4345 1630 50  0000 L CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 4300 1375 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88503/1n4001.pdf" H 4300 1550 50  0001 C CNN
	1    4300 1550
	0    1    1    0   
$EndComp
Wire Wire Line
	4300 2000 4300 1700
Wire Wire Line
	4300 1400 4300 1250
Wire Wire Line
	5200 4700 5200 4650
Wire Wire Line
	5200 4650 4800 4650
Wire Notes Line
	6500 3400 6500 6500
Wire Notes Line
	6500 6500 850  6500
Wire Notes Line
	850  6500 850  3400
Wire Notes Line
	850  3400 6500 3400
Text Label 2500 4000 2    50   ~ 0
GND
Wire Wire Line
	2250 4000 2500 4000
Connection ~ 2250 4000
Text Label 4350 4000 2    50   ~ 0
GND
Wire Wire Line
	4350 4000 4450 4000
Connection ~ 4450 4000
Wire Wire Line
	4450 4000 5200 4000
Text Label 2850 4750 2    50   ~ 0
GND
Wire Wire Line
	2850 4750 2600 4750
Wire Wire Line
	2600 4700 2600 4750
Connection ~ 2600 4750
Wire Wire Line
	2600 4750 2600 6150
Wire Wire Line
	4350 4000 4350 4300
Wire Wire Line
	4350 4300 4600 4300
Connection ~ 4600 4300
Wire Wire Line
	4350 4300 4350 4500
Wire Wire Line
	4350 4500 4700 4500
Connection ~ 4350 4300
Connection ~ 4700 4500
Wire Wire Line
	3350 1750 3350 1400
Wire Wire Line
	3450 1750 3450 1400
Wire Notes Line
	2800 800  2800 2300
Wire Notes Line
	2800 2300 5000 2300
Wire Notes Line
	5000 2300 5000 800 
Wire Notes Line
	5000 800  2800 800 
$EndSCHEMATC
