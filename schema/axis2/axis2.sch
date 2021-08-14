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
Text GLabel 2700 1100 1    50   Input ~ 0
+12V
Wire Wire Line
	2700 1350 2700 1100
Wire Wire Line
	2700 1900 2700 1650
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
Text Label 3150 3950 2    50   ~ 0
GND
Wire Wire Line
	3250 3950 3150 3950
Text Label 5950 4500 2    50   ~ 0
GND
Wire Wire Line
	5700 4500 5750 4500
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
Text GLabel 4450 2100 1    50   Input ~ 0
+5V
Wire Wire Line
	4450 2200 4450 2100
Text Label 2700 1900 1    50   ~ 0
GND
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
+12V
Text GLabel 6900 1350 3    50   Input ~ 0
GND
Wire Wire Line
	7000 1350 7000 1100
Wire Wire Line
	6900 1350 6900 1100
Text GLabel 7950 1350 3    50   Input ~ 0
+12V
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
Text Notes 3400 5900 0    50   ~ 0
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
Text Notes 10400 2650 0    50   ~ 0
Motors Control
Wire Wire Line
	9400 1150 9550 1150
Wire Wire Line
	10400 1150 10600 1150
Text GLabel 5800 4300 2    50   Output ~ 0
M1S
Wire Wire Line
	5700 4300 5750 4300
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
	5200 4400 5150 4400
Wire Wire Line
	4850 2150 4850 2500
Wire Wire Line
	4950 2550 5000 2550
Wire Wire Line
	4950 2150 4950 2550
Text GLabel 5800 4000 2    50   Input ~ 0
RCS
Wire Wire Line
	5800 4000 5750 4000
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
Wire Notes Line
	6600 3600 9050 3600
Wire Notes Line
	9050 3600 9050 700 
Wire Notes Line
	9050 700  6600 700 
Wire Notes Line
	6600 700  6600 3600
$Comp
L dk_Rectangular-Connectors-Headers-Male-Pins:22-23-2021 J10
U 1 1 60FAFD34
P 7000 2250
F 0 "J10" H 6912 2210 50  0000 R CNN
F 1 "Power Motor 3" H 6912 2301 50  0000 R CNN
F 2 "Connector:Deans" H 7200 2450 60  0001 L CNN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/Molex%20PDFs/A-6373-N_Series_Dwg_2010-12-03.pdf" H 7200 2550 60  0001 L CNN
F 4 "WM4200-ND" H 7200 2650 60  0001 L CNN "Digi-Key_PN"
F 5 "22-23-2021" H 7200 2750 60  0001 L CNN "MPN"
F 6 "Connectors, Interconnects" H 7200 2850 60  0001 L CNN "Category"
F 7 "Rectangular Connectors - Headers, Male Pins" H 7200 2950 60  0001 L CNN "Family"
F 8 "https://media.digikey.com/pdf/Data%20Sheets/Molex%20PDFs/A-6373-N_Series_Dwg_2010-12-03.pdf" H 7200 3050 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/molex/22-23-2021/WM4200-ND/26667" H 7200 3150 60  0001 L CNN "DK_Detail_Page"
F 10 "CONN HEADER VERT 2POS 2.54MM" H 7200 3250 60  0001 L CNN "Description"
F 11 "Molex" H 7200 3350 60  0001 L CNN "Manufacturer"
F 12 "Active" H 7200 3450 60  0001 L CNN "Status"
	1    7000 2250
	-1   0    0    1   
$EndComp
$Comp
L dk_Rectangular-Connectors-Headers-Male-Pins:22-23-2021 J11
U 1 1 60FAFDEC
P 7950 2250
F 0 "J11" H 7862 2210 50  0000 R CNN
F 1 "Power Motor 4" H 7862 2301 50  0000 R CNN
F 2 "Connector:Deans" H 8150 2450 60  0001 L CNN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/Molex%20PDFs/A-6373-N_Series_Dwg_2010-12-03.pdf" H 8150 2550 60  0001 L CNN
F 4 "WM4200-ND" H 8150 2650 60  0001 L CNN "Digi-Key_PN"
F 5 "22-23-2021" H 8150 2750 60  0001 L CNN "MPN"
F 6 "Connectors, Interconnects" H 8150 2850 60  0001 L CNN "Category"
F 7 "Rectangular Connectors - Headers, Male Pins" H 8150 2950 60  0001 L CNN "Family"
F 8 "https://media.digikey.com/pdf/Data%20Sheets/Molex%20PDFs/A-6373-N_Series_Dwg_2010-12-03.pdf" H 8150 3050 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/molex/22-23-2021/WM4200-ND/26667" H 8150 3150 60  0001 L CNN "DK_Detail_Page"
F 10 "CONN HEADER VERT 2POS 2.54MM" H 8150 3250 60  0001 L CNN "Description"
F 11 "Molex" H 8150 3350 60  0001 L CNN "Manufacturer"
F 12 "Active" H 8150 3450 60  0001 L CNN "Status"
	1    7950 2250
	-1   0    0    1   
$EndComp
Text GLabel 7000 2600 3    50   Input ~ 0
+12V
Text GLabel 6900 2600 3    50   Input ~ 0
GND
Wire Wire Line
	6900 2600 6900 2350
Wire Wire Line
	7000 2600 7000 2350
Text GLabel 7950 2600 3    50   Input ~ 0
+12V
Text GLabel 7850 2600 3    50   Input ~ 0
GND
Wire Wire Line
	7850 2600 7850 2350
Wire Wire Line
	7950 2600 7950 2350
$Comp
L Connector_Generic:Conn_01x03 J12
U 1 1 60FB4339
P 9750 1700
F 0 "J12" H 9830 1742 50  0000 L CNN
F 1 "M3" H 9830 1651 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 9750 1700 50  0001 C CNN
F 3 "~" H 9750 1700 50  0001 C CNN
	1    9750 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 1700 9550 1700
Text GLabel 9400 1800 0    50   Input ~ 0
M3S
Wire Wire Line
	9400 1800 9550 1800
Text Label 9300 1600 2    50   ~ 0
GND
Wire Wire Line
	9300 1600 9550 1600
$Comp
L Connector_Generic:Conn_01x03 J14
U 1 1 60FB4D31
P 10800 1700
F 0 "J14" H 10880 1742 50  0000 L CNN
F 1 "M4" H 10880 1651 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 10800 1700 50  0001 C CNN
F 3 "~" H 10800 1700 50  0001 C CNN
	1    10800 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	10450 1700 10600 1700
Text GLabel 10450 1800 0    50   Input ~ 0
M4S
Wire Wire Line
	10450 1800 10600 1800
Text Label 10350 1600 2    50   ~ 0
GND
Wire Wire Line
	10350 1600 10600 1600
$Comp
L Connector_Generic:Conn_01x03 J13
U 1 1 60FB5923
P 9750 2250
F 0 "J13" H 9830 2292 50  0000 L CNN
F 1 "M5" H 9830 2201 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 9750 2250 50  0001 C CNN
F 3 "~" H 9750 2250 50  0001 C CNN
	1    9750 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 2250 9550 2250
Text GLabel 9400 2350 0    50   Input ~ 0
M5S
Wire Wire Line
	9400 2350 9550 2350
Text Label 9300 2150 2    50   ~ 0
GND
Wire Wire Line
	9300 2150 9550 2150
$Comp
L Connector_Generic:Conn_01x03 J15
U 1 1 60FB592F
P 10800 2250
F 0 "J15" H 10880 2292 50  0000 L CNN
F 1 "M6" H 10880 2201 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 10800 2250 50  0001 C CNN
F 3 "~" H 10800 2250 50  0001 C CNN
	1    10800 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	10450 2250 10600 2250
Text GLabel 10450 2350 0    50   Input ~ 0
M6S
Wire Wire Line
	10450 2350 10600 2350
Text Label 10350 2150 2    50   ~ 0
GND
Wire Wire Line
	10350 2150 10600 2150
Wire Notes Line
	9100 700  9100 2700
Wire Notes Line
	9100 2700 11000 2700
Wire Notes Line
	11000 2700 11000 700 
Wire Notes Line
	11000 700  9100 700 
Text GLabel 5700 4900 2    50   Input ~ 0
+5V
Text GLabel 5800 4200 2    50   Output ~ 0
M3S
Wire Wire Line
	5700 4200 5750 4200
Text GLabel 5100 4100 0    50   Output ~ 0
M4S
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
P 10050 3400
F 0 "U3" H 10000 3500 50  0000 L CNN
F 1 "axis4-symbol" H 9800 3400 50  0000 L CNN
F 2 "axis4:axis4-frame" H 10050 3400 50  0001 C CNN
F 3 "" H 10050 3400 50  0001 C CNN
	1    10050 3400
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
Text GLabel 9400 1150 0    50   Output ~ 0
+5V
Text GLabel 9400 1700 0    50   Output ~ 0
+5V
Text GLabel 9400 2250 0    50   Output ~ 0
+5V
Text GLabel 10400 1150 0    50   Output ~ 0
+5V
Text GLabel 10450 1700 0    50   Output ~ 0
+5V
Text GLabel 10450 2250 0    50   Output ~ 0
+5V
Text Label 5700 1900 1    50   ~ 0
GND
Text GLabel 5700 1150 1    50   Input ~ 0
+5V
$Comp
L Diode:1N4007 D1
U 1 1 610E581C
P 5700 1450
F 0 "D1" V 5654 1530 50  0000 L CNN
F 1 "1N4007" V 5745 1530 50  0000 L CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 5700 1275 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88503/1n4001.pdf" H 5700 1450 50  0001 C CNN
	1    5700 1450
	0    1    1    0   
$EndComp
Wire Wire Line
	5700 1900 5700 1600
Wire Wire Line
	5700 1300 5700 1150
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
$EndSCHEMATC
