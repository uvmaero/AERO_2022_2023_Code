EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 5
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
L Transistor_FET:STB15N80K5 Q3
U 1 1 61F4E7F1
P 2200 2700
F 0 "Q3" H 2400 2600 50  0000 L CNN
F 1 "STB15N80K5" H 2350 2450 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TO-263-2" H 2400 2625 50  0001 L CIN
F 3 "https://www.st.com/resource/en/datasheet/stb15n80k5.pdf" H 2200 2700 50  0001 L CNN
	1    2200 2700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 61F4EF4B
P 3450 3200
F 0 "#PWR0121" H 3450 2950 50  0001 C CNN
F 1 "GND" H 3455 3027 50  0000 C CNN
F 2 "" H 3450 3200 50  0001 C CNN
F 3 "" H 3450 3200 50  0001 C CNN
	1    3450 3200
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:STB15N80K5 Q4
U 1 1 61F4F19F
P 3350 2750
F 0 "Q4" H 3550 2650 50  0000 L CNN
F 1 "STB15N80K5" H 3550 2500 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TO-263-2" H 3550 2675 50  0001 L CIN
F 3 "https://www.st.com/resource/en/datasheet/stb15n80k5.pdf" H 3350 2750 50  0001 L CNN
	1    3350 2750
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:STB15N80K5 Q5
U 1 1 61F4FA3C
P 4500 2750
F 0 "Q5" H 4750 2650 50  0000 L CNN
F 1 "STB15N80K5" H 4700 2500 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TO-263-2" H 4700 2675 50  0001 L CIN
F 3 "https://www.st.com/resource/en/datasheet/stb15n80k5.pdf" H 4500 2750 50  0001 L CNN
	1    4500 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 3200 3450 2950
Wire Wire Line
	2300 3200 3450 3200
Wire Wire Line
	2300 2900 2300 3200
Connection ~ 3450 3200
Wire Wire Line
	4600 2950 4600 3200
Wire Wire Line
	4600 3200 3450 3200
Text HLabel 2350 2350 2    50   Input ~ 0
FAN_OUT
Wire Wire Line
	2350 2350 2300 2350
Wire Wire Line
	2300 2350 2300 2500
Text HLabel 1800 2700 0    50   Input ~ 0
FAN_CTRL
Wire Wire Line
	1800 2700 2000 2700
Text HLabel 3500 2350 2    50   Input ~ 0
PUMP_OUT
Wire Wire Line
	3500 2350 3450 2350
Wire Wire Line
	3450 2350 3450 2550
Text HLabel 3150 2750 0    50   Input ~ 0
PUMP_CTRL
Text HLabel 4650 2350 2    50   Input ~ 0
BRAKE_OUT
Wire Wire Line
	4650 2350 4600 2350
Wire Wire Line
	4600 2350 4600 2550
Text HLabel 4300 2750 0    50   Input ~ 0
BRAKE_CTRL
$EndSCHEMATC
