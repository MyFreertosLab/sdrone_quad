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
NoConn ~ 1700 4100
NoConn ~ 1700 4200
NoConn ~ 1700 4400
NoConn ~ 1700 4500
NoConn ~ 1700 4800
NoConn ~ 1700 4900
NoConn ~ 2200 4900
NoConn ~ 2200 4800
NoConn ~ 2200 4700
NoConn ~ 2200 4500
NoConn ~ 2200 4400
NoConn ~ 2200 4300
NoConn ~ 2200 4200
NoConn ~ 2200 4100
NoConn ~ 2200 4000
NoConn ~ 5200 4000
NoConn ~ 5200 4100
NoConn ~ 5200 4200
NoConn ~ 5200 4300
NoConn ~ 5200 4500
NoConn ~ 5200 4600
NoConn ~ 5200 4800
NoConn ~ 5200 4900
NoConn ~ 5700 4900
NoConn ~ 5700 4800
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
	1550 4600 1700 4600
Text GLabel 3250 4700 0    50   Output ~ 0
MISO
Text GLabel 1550 4600 0    50   Input ~ 0
MISO
Wire Wire Line
	1550 4700 1700 4700
Text GLabel 3250 4250 0    50   Input ~ 0
MOSI
Text GLabel 2400 4600 2    50   Output ~ 0
SCL
Wire Wire Line
	2200 4600 2400 4600
Text GLabel 1550 4700 0    50   Output ~ 0
MOSI
Text GLabel 1500 4000 0    50   Output ~ 0
+3V3
Wire Wire Line
	1500 4000 1700 4000
Text GLabel 3250 3800 0    50   Input ~ 0
+3V3
Text GLabel 5700 4700 2    50   Output ~ 0
NCS
Text GLabel 3250 5000 0    50   Input ~ 0
NCS
NoConn ~ 5700 4600
NoConn ~ 5700 4400
NoConn ~ 5700 4200
NoConn ~ 5700 4100
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
L axis2-rescue:L7805-Regulator_Linear U2
U 1 1 60754FE6
P 5400 1500
F 0 "U2" H 5400 1742 50  0000 C CNN
F 1 "L7805" H 5400 1651 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 5425 1350 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 5400 1450 50  0001 C CNN
	1    5400 1500
	1    0    0    -1  
$EndComp
Text GLabel 2700 1100 1    50   Input ~ 0
+12V
Wire Wire Line
	2700 1350 2700 1100
Wire Wire Line
	2700 1900 2700 1650
Text GLabel 4800 1500 0    50   Input ~ 0
+12V
$Comp
L axis2-rescue:GND-power #PWR0101
U 1 1 60756A5D
P 1550 1100
F 0 "#PWR0101" H 1550 850 50  0001 C CNN
F 1 "GND" H 1555 927 50  0000 C CNN
F 2 "" H 1550 1100 50  0001 C CNN
F 3 "" H 1550 1100 50  0001 C CNN
	1    1550 1100
	-1   0    0    1   
$EndComp
$Comp
L axis2-rescue:+12V-power #PWR0102
U 1 1 60756DB1
P 1050 1100
F 0 "#PWR0102" H 1050 950 50  0001 C CNN
F 1 "+12V" H 1065 1273 50  0000 C CNN
F 2 "" H 1050 1100 50  0001 C CNN
F 3 "" H 1050 1100 50  0001 C CNN
	1    1050 1100
	1    0    0    -1  
$EndComp
$Comp
L axis2-rescue:PWR_FLAG-power #FLG02
U 1 1 607576CC
P 1550 1400
F 0 "#FLG02" H 1550 1475 50  0001 C CNN
F 1 "PWR_FLAG" H 1550 1573 50  0000 C CNN
F 2 "" H 1550 1400 50  0001 C CNN
F 3 "~" H 1550 1400 50  0001 C CNN
	1    1550 1400
	-1   0    0    1   
$EndComp
$Comp
L axis2-rescue:PWR_FLAG-power #FLG01
U 1 1 60757862
P 1050 1400
F 0 "#FLG01" H 1050 1475 50  0001 C CNN
F 1 "PWR_FLAG" H 1050 1573 50  0000 C CNN
F 2 "" H 1050 1400 50  0001 C CNN
F 3 "~" H 1050 1400 50  0001 C CNN
	1    1050 1400
	-1   0    0    1   
$EndComp
Wire Wire Line
	1050 1400 1050 1100
Wire Wire Line
	1550 1400 1550 1100
$Comp
L axis2-rescue:CP1-Device C1
U 1 1 607588D9
P 2700 1500
F 0 "C1" H 2815 1546 50  0000 L CNN
F 1 "100uF" H 2815 1455 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 2700 1500 50  0001 C CNN
F 3 "~" H 2700 1500 50  0001 C CNN
	1    2700 1500
	1    0    0    -1  
$EndComp
$Comp
L axis2-rescue:CP1-Device C2
U 1 1 60758924
P 4450 2350
F 0 "C2" H 4565 2396 50  0000 L CNN
F 1 "100uF" H 4565 2305 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 4450 2350 50  0001 C CNN
F 3 "~" H 4450 2350 50  0001 C CNN
	1    4450 2350
	1    0    0    -1  
$EndComp
$Comp
L axis2-rescue:Conn_01x02_Female-Connector J3
U 1 1 60759B12
P 4850 1950
F 0 "J3" V 4790 1762 50  0000 R CNN
F 1 "5VPower" V 4900 2000 50  0000 R CNN
F 2 "Connector_JST:JST_XH_B2B-XH-AM_1x02_P2.50mm_Vertical" H 4850 1950 50  0001 C CNN
F 3 "~" H 4850 1950 50  0001 C CNN
	1    4850 1950
	0    -1   -1   0   
$EndComp
$Comp
L axis2-rescue:Conn_01x02_Female-Connector J4
U 1 1 6075A23A
P 3250 1000
F 0 "J4" V 3190 812 50  0000 R CNN
F 1 "12VPower" V 3099 812 50  0000 R CNN
F 2 "Connector_BarrelJack:BarrelJack_Wuerth_6941xx301002" H 3250 1000 50  0001 C CNN
F 3 "~" H 3250 1000 50  0001 C CNN
	1    3250 1000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4800 1500 5100 1500
Text Label 5400 2050 1    50   ~ 0
GND
Wire Wire Line
	5400 1800 5400 2050
Text Label 3150 3950 2    50   ~ 0
GND
Wire Wire Line
	3250 3950 3150 3950
Text Label 5950 4500 2    50   ~ 0
GND
Wire Wire Line
	5700 4500 5950 4500
Text Label 3400 1450 3    50   ~ 0
GND
Text GLabel 3250 1350 3    50   Input ~ 0
+12V
Text GLabel 4850 2500 3    50   Input ~ 0
+5V
Text Label 5000 2550 3    50   ~ 0
GND
Text Label 4450 2800 1    50   ~ 0
GND
Wire Wire Line
	4450 2500 4450 2800
Text GLabel 6050 1500 2    50   Output ~ 0
+5V
Wire Wire Line
	5700 1500 6050 1500
Text GLabel 4450 2100 1    50   Input ~ 0
+5V
Wire Wire Line
	4450 2200 4450 2100
Text Label 2700 1900 1    50   ~ 0
GND
$Comp
L axis2-rescue:Heatsink_Pad_2Pin-Mechanical HS1
U 1 1 60765A1F
P 5400 1000
F 0 "HS1" H 5587 1041 50  0000 L CNN
F 1 "Heatsink_Pad_2Pin" H 5587 950 50  0000 L CNN
F 2 "Heatsink:Heatsink_Stonecold_HS-132_32x14mm_2xFixation1.5mm" H 5412 950 50  0001 C CNN
F 3 "~" H 5412 950 50  0001 C CNN
	1    5400 1000
	1    0    0    -1  
$EndComp
NoConn ~ 5300 1100
NoConn ~ 5500 1100
$Comp
L dk_Rectangular-Connectors-Headers-Male-Pins:22-23-2021 J5
U 1 1 607713B7
P 7000 1000
F 0 "J5" H 6912 960 50  0000 R CNN
F 1 "Power Motor 1" H 6912 1051 50  0000 R CNN
F 2 "Connector:Deans" H 7200 1200 60  0001 L CNN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/Molex%20PDFs/A-6373-N_Series_Dwg_2010-12-03.pdf" H 7200 1300 60  0001 L CNN
F 4 "WM4200-ND" H 7200 1400 60  0001 L CNN "Digi-Key_PN"
F 5 "22-23-2021" H 7200 1500 60  0001 L CNN "MPN"
F 6 "Connectors, Interconnects" H 7200 1600 60  0001 L CNN "Category"
F 7 "Rectangular Connectors - Headers, Male Pins" H 7200 1700 60  0001 L CNN "Family"
F 8 "https://media.digikey.com/pdf/Data%20Sheets/Molex%20PDFs/A-6373-N_Series_Dwg_2010-12-03.pdf" H 7200 1800 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/molex/22-23-2021/WM4200-ND/26667" H 7200 1900 60  0001 L CNN "DK_Detail_Page"
F 10 "CONN HEADER VERT 2POS 2.54MM" H 7200 2000 60  0001 L CNN "Description"
F 11 "Molex" H 7200 2100 60  0001 L CNN "Manufacturer"
F 12 "Active" H 7200 2200 60  0001 L CNN "Status"
	1    7000 1000
	-1   0    0    1   
$EndComp
$Comp
L dk_Rectangular-Connectors-Headers-Male-Pins:22-23-2021 J6
U 1 1 6075CB28
P 7950 1000
F 0 "J6" H 7862 960 50  0000 R CNN
F 1 "Power Motor 2" H 7862 1051 50  0000 R CNN
F 2 "Connector:Deans" H 8150 1200 60  0001 L CNN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/Molex%20PDFs/A-6373-N_Series_Dwg_2010-12-03.pdf" H 8150 1300 60  0001 L CNN
F 4 "WM4200-ND" H 8150 1400 60  0001 L CNN "Digi-Key_PN"
F 5 "22-23-2021" H 8150 1500 60  0001 L CNN "MPN"
F 6 "Connectors, Interconnects" H 8150 1600 60  0001 L CNN "Category"
F 7 "Rectangular Connectors - Headers, Male Pins" H 8150 1700 60  0001 L CNN "Family"
F 8 "https://media.digikey.com/pdf/Data%20Sheets/Molex%20PDFs/A-6373-N_Series_Dwg_2010-12-03.pdf" H 8150 1800 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/molex/22-23-2021/WM4200-ND/26667" H 8150 1900 60  0001 L CNN "DK_Detail_Page"
F 10 "CONN HEADER VERT 2POS 2.54MM" H 8150 2000 60  0001 L CNN "Description"
F 11 "Molex" H 8150 2100 60  0001 L CNN "Manufacturer"
F 12 "Active" H 8150 2200 60  0001 L CNN "Status"
	1    7950 1000
	-1   0    0    1   
$EndComp
Text GLabel 7000 1350 3    50   Input ~ 0
+12VM
Text GLabel 6900 1350 3    50   Input ~ 0
GND
Wire Wire Line
	7000 1350 7000 1100
Wire Wire Line
	6900 1350 6900 1100
Text GLabel 7950 1350 3    50   Input ~ 0
+12VM
Wire Wire Line
	7950 1350 7950 1100
Text GLabel 7850 1350 3    50   Input ~ 0
GND
Wire Wire Line
	7850 1350 7850 1100
Text Notes 8600 3400 0    50   ~ 0
Power\nMotors
Wire Notes Line
	2200 700  2200 2100
Wire Notes Line
	2200 2100 3900 2100
Wire Notes Line
	3900 2100 3900 700 
Wire Notes Line
	3900 700  2200 700 
Text Notes 3350 1950 0    50   ~ 0
Power 12V
Wire Notes Line
	4100 700  6450 700 
Wire Notes Line
	6450 700  6450 2900
Wire Notes Line
	6450 2900 4100 2900
Wire Notes Line
	4100 2900 4100 700 
Text Notes 5950 2700 0    50   ~ 0
Power 5V
Wire Notes Line
	1050 5450 6200 5450
Wire Notes Line
	6200 5450 6200 3250
Wire Notes Line
	6200 3250 1050 3250
Wire Notes Line
	1050 3250 1050 5450
Text Notes 5350 5300 0    50   ~ 0
Controller Board
$Comp
L Connector_Generic:Conn_01x03 J7
U 1 1 6075F9C4
P 9750 1150
F 0 "J7" H 9830 1192 50  0000 L CNN
F 1 "M1" H 9830 1101 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 9750 1150 50  0001 C CNN
F 3 "~" H 9750 1150 50  0001 C CNN
	1    9750 1150
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J8
U 1 1 6075FA63
P 10800 1150
F 0 "J8" H 10880 1192 50  0000 L CNN
F 1 "M2" H 10880 1101 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 10800 1150 50  0001 C CNN
F 3 "~" H 10800 1150 50  0001 C CNN
	1    10800 1150
	1    0    0    -1  
$EndComp
Wire Notes Line
	9100 850  9100 1500
Wire Notes Line
	9100 1500 11150 1500
Wire Notes Line
	11150 1500 11150 850 
Wire Notes Line
	11150 850  9100 850 
Text Notes 10450 1450 0    50   ~ 0
Motors Control
Wire Wire Line
	9400 1150 9550 1150
Wire Wire Line
	10400 1150 10600 1150
Text GLabel 5800 4300 2    50   Output ~ 0
M1S
Wire Wire Line
	5700 4300 5800 4300
Text GLabel 9400 1250 0    50   Input ~ 0
M1S
Wire Wire Line
	9400 1250 9550 1250
Text GLabel 10400 1250 0    50   Input ~ 0
M2S
Wire Wire Line
	10400 1250 10600 1250
Text GLabel 5100 4400 0    50   Output ~ 0
M2S
Wire Wire Line
	5200 4400 5100 4400
Wire Wire Line
	4850 2150 4850 2500
Wire Wire Line
	4950 2550 5000 2550
Wire Wire Line
	4950 2150 4950 2550
Text GLabel 5800 4000 2    50   Input ~ 0
RCS
Wire Wire Line
	5800 4000 5700 4000
Text GLabel 2650 2650 0    50   Input ~ 0
RCS
Wire Notes Line
	2200 2250 2200 2800
Wire Notes Line
	2200 2800 3850 2800
Wire Notes Line
	3850 2800 3850 2250
Wire Notes Line
	3850 2250 2200 2250
Text Notes 3600 2700 0    50   ~ 0
RC
$Comp
L Connector_Generic:Conn_01x03 J9
U 1 1 60762012
P 3050 2550
F 0 "J9" H 3130 2592 50  0000 L CNN
F 1 "RC-PPM" H 3130 2501 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 3050 2550 50  0001 C CNN
F 3 "~" H 3050 2550 50  0001 C CNN
	1    3050 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 2650 2850 2650
Text Label 2600 2450 2    50   ~ 0
GND
Wire Wire Line
	2600 2450 2850 2450
Text GLabel 2650 2550 0    50   Input ~ 0
+5V
Wire Wire Line
	2650 2550 2850 2550
$Comp
L axis2-symbols:axis2-frame U3
U 1 1 6076B53D
P 10000 1950
F 0 "U3" H 10178 2096 50  0000 L CNN
F 1 "axis2-frame" H 10178 2005 50  0000 L CNN
F 2 "axis2:axis2-footprint" H 10000 1950 50  0001 C CNN
F 3 "" H 10000 1950 50  0001 C CNN
	1    10000 1950
	1    0    0    -1  
$EndComp
NoConn ~ 9400 1150
NoConn ~ 10400 1150
Wire Wire Line
	3350 1200 3350 1400
Wire Wire Line
	3250 1200 3250 1350
Wire Wire Line
	3350 1400 3400 1400
Wire Wire Line
	3400 1400 3400 1450
Text Label 9300 1050 2    50   ~ 0
GND
Wire Wire Line
	9300 1050 9550 1050
Text Label 10350 1050 2    50   ~ 0
GND
Wire Wire Line
	10350 1050 10600 1050
$Comp
L dk_Power-Relays-Over-2-Amps:G5LE-14_DC5 RLY1
U 1 1 608096DF
P 8150 2300
F 0 "RLY1" H 8478 2346 50  0000 L CNN
F 1 "G5LE-14_DC5" H 8478 2255 50  0000 L CNN
F 2 "Relay_THT:Relay_SPDT_Omron-G5LE-1" H 8350 2500 50  0001 L CNN
F 3 "https://omronfs.omron.com/en_US/ecb/products/pdf/en-g5le.pdf" H 8350 2600 60  0001 L CNN
F 4 "Z1011-ND" H 8350 2700 60  0001 L CNN "Digi-Key_PN"
F 5 "G5LE-14 DC5" H 8350 2800 60  0001 L CNN "MPN"
F 6 "Relays" H 8350 2900 60  0001 L CNN "Category"
F 7 "Power Relays, Over 2 Amps" H 8350 3000 60  0001 L CNN "Family"
F 8 "https://omronfs.omron.com/en_US/ecb/products/pdf/en-g5le.pdf" H 8350 3100 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/omron-electronics-inc-emc-div/G5LE-14-DC5/Z1011-ND/280371" H 8350 3200 60  0001 L CNN "DK_Detail_Page"
F 10 "RELAY GEN PURPOSE SPDT 10A 5V" H 8350 3300 60  0001 L CNN "Description"
F 11 "Omron Electronics Inc-EMC Div" H 8350 3400 60  0001 L CNN "Manufacturer"
F 12 "Active" H 8350 3500 60  0001 L CNN "Status"
	1    8150 2300
	1    0    0    -1  
$EndComp
Text GLabel 7200 2900 0    50   Input ~ 0
GPIO32
Text GLabel 8250 2800 3    50   Input ~ 0
+12V
Wire Wire Line
	8250 2800 8250 2600
Text GLabel 8150 1800 1    50   Output ~ 0
+12VM
Wire Wire Line
	8350 2000 8350 1800
NoConn ~ 8350 1800
Text GLabel 1550 4300 0    50   Output ~ 0
GPIO32
Wire Wire Line
	1700 4300 1550 4300
$Comp
L Device:D_ALT D1
U 1 1 60836FCC
P 7550 2350
F 0 "D1" V 7504 2430 50  0000 L CNN
F 1 "D_ALT" V 7595 2430 50  0000 L CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 7550 2350 50  0001 C CNN
F 3 "~" H 7550 2350 50  0001 C CNN
	1    7550 2350
	0    1    1    0   
$EndComp
Wire Wire Line
	7550 2200 7550 2000
Connection ~ 7550 2000
Wire Wire Line
	7550 2000 7950 2000
Wire Wire Line
	7550 2500 7550 2600
Connection ~ 7550 2600
Wire Wire Line
	7550 2600 7950 2600
Text GLabel 7200 2000 0    50   Input ~ 0
+3V3
Wire Wire Line
	7200 2000 7550 2000
Wire Notes Line
	6600 3600 9050 3600
Wire Notes Line
	9050 3600 9050 700 
Wire Notes Line
	9050 700  6600 700 
Wire Notes Line
	6600 700  6600 3600
$Comp
L Transistor_BJT:BC547 Q1
U 1 1 60841F0D
P 7400 2900
F 0 "Q1" H 7591 2946 50  0000 L CNN
F 1 "BC547" H 7591 2855 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92" H 7600 2825 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/BC550-D.pdf" H 7400 2900 50  0001 L CNN
	1    7400 2900
	1    0    0    -1  
$EndComp
Text Label 6950 3100 0    50   ~ 0
GND
Wire Wire Line
	7500 2700 7500 2600
Wire Wire Line
	7500 2600 7550 2600
Wire Wire Line
	7500 3100 6950 3100
Wire Wire Line
	8150 2000 8150 1800
$EndSCHEMATC
