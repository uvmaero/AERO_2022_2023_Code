EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 5250 7250 975  325 
U 61EB08F6
F0 "TSAL" 50
F1 "TSAL.sch" 50
F2 "TSAL_GND" O R 6225 7400 50 
F3 "HV+" I L 5250 7350 50 
F4 "HV-" I L 5250 7450 50 
$EndSheet
$Comp
L Connector_Generic:Conn_01x02 J?
U 1 1 61EC9EFD
P 3050 7025
F 0 "J?" H 2968 6700 50  0000 C CNN
F 1 "Conn_01x02" H 2968 6791 50  0000 C CNN
F 2 "" H 3050 7025 50  0001 C CNN
F 3 "~" H 3050 7025 50  0001 C CNN
	1    3050 7025
	-1   0    0    1   
$EndComp
Wire Wire Line
	6725 7400 6225 7400
$Comp
L power:+12V #PWR?
U 1 1 61ECA494
P 3575 6925
F 0 "#PWR?" H 3575 6775 50  0001 C CNN
F 1 "+12V" H 3590 7098 50  0000 C CNN
F 2 "" H 3575 6925 50  0001 C CNN
F 3 "" H 3575 6925 50  0001 C CNN
	1    3575 6925
	-1   0    0    -1  
$EndComp
Text Label 2450 1450 2    50   ~ 0
HV+
Text Label 2450 1700 2    50   ~ 0
HV-
Text Label 1400 5450 2    50   ~ 0
FANOUT1
Wire Wire Line
	1050 5450 1400 5450
Wire Wire Line
	1050 6250 1400 6250
Wire Wire Line
	1050 5550 1400 5550
Text Label 1400 6250 2    50   ~ 0
FANOUT2
Text Label 1400 5550 2    50   ~ 0
FANOUT3
Text Label 1400 7250 2    50   ~ 0
HECS_AN1
Text Label 1400 7150 2    50   ~ 0
HECS_AN2
Text Label 1400 7050 2    50   ~ 0
HECS_AN3
$Comp
L power:GND #PWR?
U 1 1 5C83048B
P 1300 5850
AR Path="/5C83048B" Ref="#PWR?"  Part="1" 
AR Path="/5C83278E/5C83048B" Ref="#PWR0105"  Part="1" 
F 0 "#PWR?" H 1300 5600 50  0001 C CNN
F 1 "GND" V 1305 5722 50  0000 R CNN
F 2 "" H 1300 5850 50  0001 C CNN
F 3 "" H 1300 5850 50  0001 C CNN
	1    1300 5850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1050 6650 1400 6650
Wire Wire Line
	1050 6550 1400 6550
Text Label 1400 6650 2    50   ~ 0
CAN+
Text Label 1400 6550 2    50   ~ 0
CAN-
$Comp
L Connector_Generic:Conn_01x01 J?
U 1 1 5C830497
P 1300 1450
AR Path="/5C830497" Ref="J?"  Part="1" 
AR Path="/5C83278E/5C830497" Ref="J6"  Part="1" 
F 0 "J?" H 1300 1350 50  0000 C CNN
F 1 "HV+" H 1450 1450 50  0000 C CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_Pad_Via" H 1300 1450 50  0001 C CNN
F 3 "~" H 1300 1450 50  0001 C CNN
	1    1300 1450
	-1   0    0    1   
$EndComp
Text Notes 6350 1400 0    50   ~ 0
Global nets connected to this sheet: \nGND, 5V,  SDA, SCL
$Sheet
S 6400 3150 1600 850 
U 5C8304BF
F0 "vicor" 50
F1 "vicor.sch" 50
F2 "TRIM_EN" I L 6400 3600 50 
F3 "ENABLE" I L 6400 3500 50 
F4 "FAULT" O L 6400 3700 50 
F5 "HV+" I L 6400 3250 50 
F6 "HV-" I L 6400 3350 50 
F7 "12V_OUT" O R 8000 3250 50 
$EndSheet
Text Notes 6400 3050 0    50   ~ 0
Global nets connected to this sheet: \nGND
Text Label 5650 3250 0    50   ~ 0
HV+
Text Label 5650 3350 0    50   ~ 0
HV-
Text Label 5750 1850 0    50   ~ 0
DC_DC_TRIM_EN
Wire Wire Line
	5750 1850 6350 1850
Wire Wire Line
	5750 1950 6350 1950
Text Label 5750 1950 0    50   ~ 0
DC_DC_EN
Wire Wire Line
	5750 2050 6350 2050
Text Label 5750 2050 0    50   ~ 0
DC_DC_FAULT
Text Label 5800 3600 0    50   ~ 0
DC_DC_TRIM_EN
Wire Wire Line
	5800 3600 6400 3600
Wire Wire Line
	5800 3500 6400 3500
Text Label 5800 3500 0    50   ~ 0
DC_DC_EN
Wire Wire Line
	5800 3700 6400 3700
Text Label 5800 3700 0    50   ~ 0
DC_DC_FAULT
Text Label 8700 3250 2    50   ~ 0
DCDC_OUT
$Comp
L Connector_Generic:Conn_01x05 J?
U 1 1 5C830507
P 950 3250
AR Path="/5C830507" Ref="J?"  Part="1" 
AR Path="/5C83278E/5C830507" Ref="J1"  Part="1" 
F 0 "J?" H 950 2950 50  0000 C CNN
F 1 "HECS" V 1100 3250 50  0000 C CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-05A_1x05_P2.54mm_Vertical" H 950 3250 50  0001 C CNN
F 3 "~" H 950 3250 50  0001 C CNN
	1    950  3250
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5C83050E
P 1200 3350
AR Path="/5C83050E" Ref="#PWR?"  Part="1" 
AR Path="/5C83278E/5C83050E" Ref="#PWR0111"  Part="1" 
F 0 "#PWR?" H 1200 3200 50  0001 C CNN
F 1 "+5V" V 1215 3478 50  0000 L CNN
F 2 "" H 1200 3350 50  0001 C CNN
F 3 "" H 1200 3350 50  0001 C CNN
	1    1200 3350
	0    1    1    0   
$EndComp
Wire Wire Line
	1200 3350 1150 3350
$Comp
L power:GND #PWR?
U 1 1 5C830515
P 1200 3500
AR Path="/5C830515" Ref="#PWR?"  Part="1" 
AR Path="/5C83278E/5C830515" Ref="#PWR0112"  Part="1" 
F 0 "#PWR?" H 1200 3250 50  0001 C CNN
F 1 "GND" H 1205 3327 50  0000 C CNN
F 2 "" H 1200 3500 50  0001 C CNN
F 3 "" H 1200 3500 50  0001 C CNN
	1    1200 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 3500 1200 3450
Wire Wire Line
	1200 3450 1150 3450
Wire Wire Line
	1150 3250 1500 3250
Wire Wire Line
	1150 3150 1500 3150
Wire Wire Line
	1150 3050 1500 3050
Text Label 1500 3050 2    50   ~ 0
HECS_AN1
Text Label 1500 3150 2    50   ~ 0
HECS_AN2
Text Label 1500 3250 2    50   ~ 0
HECS_AN3
Wire Wire Line
	6350 1600 6050 1600
Wire Wire Line
	6350 1700 6050 1700
Text Label 6050 1600 0    50   ~ 0
CAN+
Text Label 6050 1700 0    50   ~ 0
CAN-
Text Label 8250 2150 2    50   ~ 0
TEMP_1
Text Label 8250 2250 2    50   ~ 0
TEMP_2
Wire Wire Line
	7950 2150 8250 2150
Wire Wire Line
	7950 2250 8250 2250
$Comp
L power:+5V #PWR?
U 1 1 5C83055F
P 2400 4250
AR Path="/5C83055F" Ref="#PWR?"  Part="1" 
AR Path="/5C83278E/5C83055F" Ref="#PWR0116"  Part="1" 
F 0 "#PWR?" H 2400 4100 50  0001 C CNN
F 1 "+5V" H 2415 4423 50  0000 C CNN
F 2 "" H 2400 4250 50  0001 C CNN
F 3 "" H 2400 4250 50  0001 C CNN
	1    2400 4250
	1    0    0    -1  
$EndComp
Text Label 2600 4500 2    50   ~ 0
TEMP_1
Wire Wire Line
	2300 4500 2600 4500
Wire Wire Line
	2300 4600 2600 4600
Text Label 2600 4600 2    50   ~ 0
TEMP_2
Wire Notes Line width 39 rgb(253, 0, 0)
	650  850  2900 850 
Wire Notes Line width 39 rgb(255, 0, 0)
	2950 900  2950 2150
Wire Notes Line width 39 rgb(255, 0, 0)
	2900 2150 650  2150
Wire Notes Line width 39 rgb(255, 0, 0)
	650  2100 650  850 
Text Notes 650  800  0    157  ~ 31
HV Danger!
$Comp
L Connector_Generic:Conn_01x23 J?
U 1 1 5C830583
P 850 6550
AR Path="/5C830583" Ref="J?"  Part="1" 
AR Path="/5C83278E/5C830583" Ref="J16"  Part="1" 
F 0 "J?" H 770 7867 50  0000 C CNN
F 1 "Conn_01x23" H 770 7776 50  0000 C CNN
F 2 "1-776087-1:TE_1-776087-1" H 850 6550 50  0001 C CNN
F 3 "~" H 850 6550 50  0001 C CNN
	1    850  6550
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1050 6150 1800 6150
Wire Wire Line
	1050 7250 1400 7250
Wire Wire Line
	1050 7050 1400 7050
Wire Wire Line
	1050 6050 1800 6050
Wire Wire Line
	1150 5750 1150 5850
Connection ~ 1150 5850
Wire Wire Line
	1150 5850 1150 5950
Wire Wire Line
	1150 5850 1300 5850
$Comp
L power:GND #PWR?
U 1 1 5C83059B
P 1150 6450
AR Path="/5C83059B" Ref="#PWR?"  Part="1" 
AR Path="/5C83278E/5C83059B" Ref="#PWR0119"  Part="1" 
F 0 "#PWR?" H 1150 6200 50  0001 C CNN
F 1 "GND" V 1155 6322 50  0000 R CNN
F 2 "" H 1150 6450 50  0001 C CNN
F 3 "" H 1150 6450 50  0001 C CNN
	1    1150 6450
	0    -1   -1   0   
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 5C8305A2
P 1250 5650
AR Path="/5C8305A2" Ref="#PWR?"  Part="1" 
AR Path="/5C83278E/5C8305A2" Ref="#PWR0120"  Part="1" 
F 0 "#PWR?" H 1250 5500 50  0001 C CNN
F 1 "+12V" V 1265 5778 50  0000 L CNN
F 2 "" H 1250 5650 50  0001 C CNN
F 3 "" H 1250 5650 50  0001 C CNN
	1    1250 5650
	0    1    1    0   
$EndComp
Wire Wire Line
	1050 5950 1150 5950
$Comp
L power:+12V #PWR?
U 1 1 5C8305A9
P 1050 6350
AR Path="/5C8305A9" Ref="#PWR?"  Part="1" 
AR Path="/5C83278E/5C8305A9" Ref="#PWR0121"  Part="1" 
F 0 "#PWR?" H 1050 6200 50  0001 C CNN
F 1 "+12V" V 1065 6478 50  0000 L CNN
F 2 "" H 1050 6350 50  0001 C CNN
F 3 "" H 1050 6350 50  0001 C CNN
	1    1050 6350
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x05 J?
U 1 1 5C8305B0
P 3500 5750
AR Path="/5C8305B0" Ref="J?"  Part="1" 
AR Path="/5C83278E/5C8305B0" Ref="J4"  Part="1" 
F 0 "J?" H 3500 5450 50  0000 C CNN
F 1 "Bender-LV" V 3600 5700 50  0000 C CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-05A_1x05_P2.54mm_Vertical" H 3500 5750 50  0001 C CNN
F 3 "~" H 3500 5750 50  0001 C CNN
	1    3500 5750
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C8305B7
P 3750 5850
AR Path="/5C8305B7" Ref="#PWR?"  Part="1" 
AR Path="/5C83278E/5C8305B7" Ref="#PWR0122"  Part="1" 
F 0 "#PWR?" H 3750 5600 50  0001 C CNN
F 1 "GND" V 3755 5722 50  0000 R CNN
F 2 "" H 3750 5850 50  0001 C CNN
F 3 "" H 3750 5850 50  0001 C CNN
	1    3750 5850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3700 5750 3700 5850
Connection ~ 3700 5850
Wire Wire Line
	3700 5850 3700 5950
Wire Wire Line
	3700 5850 3750 5850
Wire Wire Line
	3700 5650 4000 5650
Text Label 4000 5650 2    50   ~ 0
IMD_OK
$Comp
L power:+12V #PWR?
U 1 1 5C8305C3
P 3750 5500
AR Path="/5C8305C3" Ref="#PWR?"  Part="1" 
AR Path="/5C83278E/5C8305C3" Ref="#PWR0123"  Part="1" 
F 0 "#PWR?" H 3750 5350 50  0001 C CNN
F 1 "+12V" H 3765 5673 50  0000 C CNN
F 2 "" H 3750 5500 50  0001 C CNN
F 3 "" H 3750 5500 50  0001 C CNN
	1    3750 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 5500 3750 5550
Wire Wire Line
	3750 5550 3700 5550
Wire Wire Line
	1050 7350 1400 7350
Text Label 1400 7350 2    50   ~ 0
IMD_OK
$Comp
L Device:Fuse F?
U 1 1 5C8305CD
P 6200 3250
AR Path="/5C8305CD" Ref="F?"  Part="1" 
AR Path="/5C83278E/5C8305CD" Ref="F1"  Part="1" 
F 0 "F?" V 6003 3250 50  0000 C CNN
F 1 "2A" V 6094 3250 50  0000 C CNN
F 2 "CustomFootprints:Fuseholder_Cylinder-5x20mm_Schurter_0031_8201_Horizontal_Open_3d" V 6130 3250 50  0001 C CNN
F 3 "~" H 6200 3250 50  0001 C CNN
	1    6200 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	6400 3250 6350 3250
Wire Wire Line
	5650 3250 6050 3250
Wire Wire Line
	5650 3350 6400 3350
Text Notes 1900 6050 0    50   ~ 0
Return from safety loop
$Comp
L Connector_Generic:Conn_01x03 J?
U 1 1 5C8305F3
P 1800 1250
AR Path="/5C8305F3" Ref="J?"  Part="1" 
AR Path="/5C83278E/5C8305F3" Ref="J8"  Part="1" 
F 0 "J?" V 1766 1062 50  0000 R CNN
F 1 "HV+" V 1675 1062 50  0000 R CNN
F 2 "Connector_TE-Connectivity:TE_MATE-N-LOK_1-770870-x_1x03_P4.14mm_Vertical" H 1800 1250 50  0001 C CNN
F 3 "~" H 1800 1250 50  0001 C CNN
	1    1800 1250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1500 1450 1700 1450
Wire Wire Line
	1900 1450 2450 1450
Wire Wire Line
	1700 1450 1800 1450
Connection ~ 1700 1450
Connection ~ 1900 1450
Connection ~ 1800 1450
Wire Wire Line
	1800 1450 1900 1450
$Comp
L Connector_Generic:Conn_01x03 J?
U 1 1 5C830602
P 1800 1900
AR Path="/5C830602" Ref="J?"  Part="1" 
AR Path="/5C83278E/5C830602" Ref="J7"  Part="1" 
F 0 "J?" V 1800 2125 50  0000 C CNN
F 1 "HV-" V 1700 2150 50  0000 C CNN
F 2 "Connector_TE-Connectivity:TE_MATE-N-LOK_1-770870-x_1x03_P4.14mm_Vertical" H 1800 1900 50  0001 C CNN
F 3 "~" H 1800 1900 50  0001 C CNN
	1    1800 1900
	0    1    1    0   
$EndComp
Wire Wire Line
	1500 1600 1500 1700
Connection ~ 1500 1700
Wire Wire Line
	1500 1700 1500 1800
Text Notes 650  5200 0    50   ~ 0
SHDN_LOOP_OUT is connected to 12V through shutdown loop
Text Notes 900  3850 0    50   ~ 0
Local fan controls and thermocoupler inputs
Text Notes 900  2800 0    50   ~ 0
Current monitor for HV and UART debug port
$Comp
L Regulator_Switching:R-78E5.0-0.5 U?
U 1 1 5C830611
P 9575 5625
AR Path="/5C830611" Ref="U?"  Part="1" 
AR Path="/5C667FAF/5C830611" Ref="U?"  Part="1" 
AR Path="/5C83278E/5C830611" Ref="U7"  Part="1" 
F 0 "U?" H 9575 5867 50  0000 C CNN
F 1 "R-78E5.0-0.5" H 9575 5776 50  0000 C CNN
F 2 "Converter_DCDC:Converter_DCDC_RECOM_R-78E-0.5_THT" H 9625 5375 50  0001 L CIN
F 3 "https://www.recom-power.com/pdf/Innoline/R-78Exx-0.5.pdf" H 9575 5625 50  0001 C CNN
	1    9575 5625
	1    0    0    -1  
$EndComp
Wire Wire Line
	9125 5625 9275 5625
Wire Wire Line
	10025 5625 9875 5625
$Comp
L power:GND #PWR?
U 1 1 5C83061A
P 9575 5975
AR Path="/5C83061A" Ref="#PWR?"  Part="1" 
AR Path="/5C667FAF/5C83061A" Ref="#PWR?"  Part="1" 
AR Path="/5C83278E/5C83061A" Ref="#PWR0124"  Part="1" 
F 0 "#PWR?" H 9575 5725 50  0001 C CNN
F 1 "GND" H 9580 5802 50  0000 C CNN
F 2 "" H 9575 5975 50  0001 C CNN
F 3 "" H 9575 5975 50  0001 C CNN
	1    9575 5975
	1    0    0    -1  
$EndComp
Wire Wire Line
	9575 5975 9575 5925
Text Notes 8675 5125 0    100  ~ 0
12V->5V
$Comp
L Device:C C?
U 1 1 5C830622
P 10025 5775
AR Path="/5C830622" Ref="C?"  Part="1" 
AR Path="/5C667FAF/5C830622" Ref="C?"  Part="1" 
AR Path="/5C83278E/5C830622" Ref="C6"  Part="1" 
F 0 "C?" H 10140 5821 50  0000 L CNN
F 1 "0.1uF" H 9775 5675 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10063 5625 50  0001 C CNN
F 3 "~" H 10025 5775 50  0001 C CNN
	1    10025 5775
	1    0    0    -1  
$EndComp
Connection ~ 10025 5625
$Comp
L power:GND #PWR?
U 1 1 5C83062A
P 10175 5975
AR Path="/5C83062A" Ref="#PWR?"  Part="1" 
AR Path="/5C667FAF/5C83062A" Ref="#PWR?"  Part="1" 
AR Path="/5C83278E/5C83062A" Ref="#PWR0125"  Part="1" 
F 0 "#PWR?" H 10175 5725 50  0001 C CNN
F 1 "GND" H 10180 5802 50  0000 C CNN
F 2 "" H 10175 5975 50  0001 C CNN
F 3 "" H 10175 5975 50  0001 C CNN
	1    10175 5975
	1    0    0    -1  
$EndComp
Wire Wire Line
	10025 5925 10175 5925
Wire Wire Line
	10175 5925 10175 5975
Connection ~ 10175 5925
Wire Wire Line
	10175 5925 10325 5925
$Comp
L Device:C C?
U 1 1 5C830634
P 9125 5775
AR Path="/5C830634" Ref="C?"  Part="1" 
AR Path="/5C667FAF/5C830634" Ref="C?"  Part="1" 
AR Path="/5C83278E/5C830634" Ref="C5"  Part="1" 
F 0 "C?" H 9240 5821 50  0000 L CNN
F 1 "10uF" H 9240 5730 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9163 5625 50  0001 C CNN
F 3 "~" H 9125 5775 50  0001 C CNN
	1    9125 5775
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C83063B
P 9125 5925
AR Path="/5C83063B" Ref="#PWR?"  Part="1" 
AR Path="/5C667FAF/5C83063B" Ref="#PWR?"  Part="1" 
AR Path="/5C83278E/5C83063B" Ref="#PWR0126"  Part="1" 
F 0 "#PWR?" H 9125 5675 50  0001 C CNN
F 1 "GND" H 9130 5752 50  0000 C CNN
F 2 "" H 9125 5925 50  0001 C CNN
F 3 "" H 9125 5925 50  0001 C CNN
	1    9125 5925
	1    0    0    -1  
$EndComp
Wire Notes Line
	10725 4975 10725 6275
Wire Wire Line
	10325 5625 10025 5625
$Comp
L Device:C C?
U 1 1 5C830643
P 10325 5775
AR Path="/5C830643" Ref="C?"  Part="1" 
AR Path="/5C667FAF/5C830643" Ref="C?"  Part="1" 
AR Path="/5C83278E/5C830643" Ref="C15"  Part="1" 
F 0 "C?" H 10440 5821 50  0000 L CNN
F 1 "10uF" H 10440 5730 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10363 5625 50  0001 C CNN
F 3 "~" H 10325 5775 50  0001 C CNN
	1    10325 5775
	1    0    0    -1  
$EndComp
Connection ~ 10325 5625
Wire Notes Line
	8625 6275 10725 6275
Wire Notes Line
	8625 4975 8625 6275
Wire Notes Line
	8625 4975 10725 4975
$Comp
L power:+12V #PWR?
U 1 1 5C83064E
P 9125 5575
AR Path="/5C83064E" Ref="#PWR?"  Part="1" 
AR Path="/5C83278E/5C83064E" Ref="#PWR0127"  Part="1" 
F 0 "#PWR?" H 9125 5425 50  0001 C CNN
F 1 "+12V" H 9140 5748 50  0000 C CNN
F 2 "" H 9125 5575 50  0001 C CNN
F 3 "" H 9125 5575 50  0001 C CNN
	1    9125 5575
	1    0    0    -1  
$EndComp
Wire Wire Line
	9125 5575 9125 5625
Connection ~ 9125 5625
$Comp
L power:+5V #PWR?
U 1 1 5C830656
P 10325 5575
AR Path="/5C830656" Ref="#PWR?"  Part="1" 
AR Path="/5C83278E/5C830656" Ref="#PWR0128"  Part="1" 
F 0 "#PWR?" H 10325 5425 50  0001 C CNN
F 1 "+5V" H 10340 5748 50  0000 C CNN
F 2 "" H 10325 5575 50  0001 C CNN
F 3 "" H 10325 5575 50  0001 C CNN
	1    10325 5575
	1    0    0    -1  
$EndComp
Wire Wire Line
	10325 5575 10325 5625
$Comp
L Connector_Generic:Conn_01x04 J3
U 1 1 5CA5874A
P 2100 4500
F 0 "J3" H 2020 4075 50  0000 C CNN
F 1 "NTCs" V 2200 4450 50  0000 C CNN
F 2 "Connector_JST:JST_PH_B4B-PH-K_1x04_P2.00mm_Vertical" H 2100 4500 50  0001 C CNN
F 3 "~" H 2100 4500 50  0001 C CNN
	1    2100 4500
	-1   0    0    1   
$EndComp
Wire Wire Line
	2300 4300 2400 4300
Wire Wire Line
	2400 4400 2300 4400
Wire Wire Line
	2400 4250 2400 4300
Connection ~ 2400 4300
Wire Wire Line
	2400 4300 2400 4400
$Comp
L power:+5V #PWR?
U 1 1 5CAA3E99
P 1150 7550
AR Path="/5CAA3E99" Ref="#PWR?"  Part="1" 
AR Path="/5C83278E/5CAA3E99" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 1150 7400 50  0001 C CNN
F 1 "+5V" H 1165 7723 50  0000 C CNN
F 2 "" H 1150 7550 50  0001 C CNN
F 3 "" H 1150 7550 50  0001 C CNN
	1    1150 7550
	0    1    1    0   
$EndComp
Wire Wire Line
	1150 7550 1050 7550
Text Label 1400 7450 2    50   ~ 0
TEMP_1
Wire Wire Line
	1050 5850 1150 5850
Wire Wire Line
	1050 5750 1150 5750
Wire Wire Line
	1250 5650 1050 5650
Wire Wire Line
	1050 7150 1400 7150
Wire Wire Line
	1050 7450 1400 7450
Wire Wire Line
	1150 6450 1050 6450
Text Label 4475 7350 0    50   ~ 0
HV+
Wire Wire Line
	5000 7350 5250 7350
Text Label 4975 7450 0    50   ~ 0
HV-
Wire Wire Line
	4975 7450 5250 7450
Wire Wire Line
	8000 3250 8700 3250
$Comp
L Device:Fuse F?
U 1 1 61F33221
P 4850 7350
AR Path="/61F33221" Ref="F?"  Part="1" 
AR Path="/5C83278E/61F33221" Ref="F?"  Part="1" 
F 0 "F?" V 4653 7350 50  0000 C CNN
F 1 "2A" V 4744 7350 50  0000 C CNN
F 2 "CustomFootprints:Fuseholder_Cylinder-5x20mm_Schurter_0031_8201_Horizontal_Open_3d" V 4780 7350 50  0001 C CNN
F 3 "~" H 4850 7350 50  0001 C CNN
	1    4850 7350
	0    1    1    0   
$EndComp
Wire Wire Line
	4475 7350 4700 7350
Text Notes 5250 6975 0    50   ~ 0
Global nets connected to this sheet: \n12V, GND
Wire Wire Line
	3250 6925 3575 6925
Text Label 6725 7400 2    50   ~ 0
TSAL_GND
Text Label 3625 7025 2    50   ~ 0
TSAL_GND
Wire Wire Line
	3625 7025 3250 7025
Text Notes 3100 6575 0    50   ~ 0
TSAL Output
Wire Wire Line
	1900 1700 2450 1700
Wire Wire Line
	1500 1700 1700 1700
Connection ~ 1900 1700
Connection ~ 1700 1700
Wire Wire Line
	1700 1700 1800 1700
Connection ~ 1800 1700
Wire Wire Line
	1800 1700 1900 1700
$Comp
L Connector_Generic:Conn_01x01 J?
U 1 1 6218DB42
P 1300 1700
AR Path="/6218DB42" Ref="J?"  Part="1" 
AR Path="/5C83278E/6218DB42" Ref="J?"  Part="1" 
F 0 "J?" H 1300 1600 50  0000 C CNN
F 1 "HV-" H 1450 1700 50  0000 C CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_Pad_Via" H 1300 1700 50  0001 C CNN
F 3 "~" H 1300 1700 50  0001 C CNN
	1    1300 1700
	-1   0    0    1   
$EndComp
Text Notes 8375 3125 0    50   ~ 0
Goes to RCB to charge \nbattery / run 12V systems\n
$Sheet
S 6350 1500 1600 900 
U 5C8304A7
F0 "controller" 50
F1 "controller.sch" 50
F2 "CAN+" B L 6350 1600 50 
F3 "CAN-" B L 6350 1700 50 
F4 "DC_DC_EN" O L 6350 1950 50 
F5 "DC_DC_FAULT" I L 6350 2050 50 
F6 "DC_DC_TRIM_EN" O L 6350 1850 50 
F7 "NTC1" I R 7950 2150 50 
F8 "NTC2" I R 7950 2250 50 
$EndSheet
Text Label 1800 6050 2    50   ~ 0
DCDC_OUT
Text Label 1800 6150 2    50   ~ 0
DCDC_OUT
Text Label 1800 6750 2    50   ~ 0
DCDC_OUT
Wire Wire Line
	1050 6750 1800 6750
Wire Wire Line
	1050 6950 1400 6950
Wire Wire Line
	1050 6850 1400 6850
Text Label 1400 6950 2    50   ~ 0
CAN+
Text Label 1400 6850 2    50   ~ 0
CAN-
$EndSCHEMATC
