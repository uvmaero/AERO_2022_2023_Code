EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 5
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
L Relay:G5V-2 K3
U 1 1 62056119
P 5700 2250
F 0 "K3" V 6467 2250 50  0000 C CNN
F 1 "G5V-2" V 6376 2250 50  0000 C CNN
F 2 "Relay_THT:Relay_DPDT_Omron_G5V-2" H 6350 2200 50  0001 L CNN
F 3 "http://omronfs.omron.com/en_US/ecb/products/pdf/en-g5v_2.pdf" H 5700 2250 50  0001 C CNN
	1    5700 2250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4100 1850 4650 1850
Wire Wire Line
	4650 1850 4650 1750
Wire Wire Line
	4650 1750 5400 1750
$Comp
L Connector_Generic:Conn_01x02 J5
U 1 1 620582A9
P 5000 2450
F 0 "J5" V 5200 2500 50  0000 R CNN
F 1 "Conn_01x02" V 5100 2700 50  0000 R CNN
F 2 "Connector_JST:JST_XH_B2B-XH-AM_1x02_P2.50mm_Vertical" H 5000 2450 50  0001 C CNN
F 3 "~" H 5000 2450 50  0001 C CNN
	1    5000 2450
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J4
U 1 1 62058AE0
P 3550 3625
F 0 "J4" V 3750 3725 50  0000 R CNN
F 1 "Conn_01x02" V 3650 3875 50  0000 R CNN
F 2 "Connector_JST:JST_XH_B2B-XH-AM_1x02_P2.50mm_Vertical" H 3550 3625 50  0001 C CNN
F 3 "~" H 3550 3625 50  0001 C CNN
	1    3550 3625
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 2650 4350 2650
Wire Wire Line
	5100 2650 5325 2650
Text HLabel 3125 1750 0    50   Input ~ 0
LOOP_IN
Wire Wire Line
	3125 1750 3500 1750
Text HLabel 6500 1850 2    50   Output ~ 0
LOOP_OUT
Wire Wire Line
	6500 1850 6000 1850
Text HLabel 6150 2250 2    50   Input ~ 0
IMD_FAULT_SIG
Wire Wire Line
	6150 2250 6000 2250
Text HLabel 4250 2250 2    50   Input ~ 0
BMS_FAULT_SIG
Wire Wire Line
	4250 2250 4100 2250
$Comp
L power:VDD #PWR0117
U 1 1 62064CA0
P 4350 2550
F 0 "#PWR0117" H 4350 2400 50  0001 C CNN
F 1 "VDD" H 4365 2723 50  0000 C CNN
F 2 "" H 4350 2550 50  0001 C CNN
F 3 "" H 4350 2550 50  0001 C CNN
	1    4350 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 2550 4350 2650
Connection ~ 4350 2650
Wire Wire Line
	4350 2650 4200 2650
$Comp
L Relay:G5V-2 K2
U 1 1 62055384
P 3800 2250
F 0 "K2" V 4567 2250 50  0000 C CNN
F 1 "G5V-2" V 4476 2250 50  0000 C CNN
F 2 "Relay_THT:Relay_DPDT_Omron_G5V-2" H 4450 2200 50  0001 L CNN
F 3 "http://omronfs.omron.com/en_US/ecb/products/pdf/en-g5v_2.pdf" H 3800 2250 50  0001 C CNN
	1    3800 2250
	0    -1   -1   0   
$EndComp
$Comp
L Transistor_FET:2N7002 Q2
U 1 1 6207167B
P 5025 3325
F 0 "Q2" H 5230 3371 50  0000 L CNN
F 1 "2N7002" H 5230 3280 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5225 3250 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 5025 3325 50  0001 L CNN
	1    5025 3325
	-1   0    0    -1  
$EndComp
$Comp
L Diode:1N4148 D5
U 1 1 62074109
P 5700 3325
F 0 "D5" H 5700 3542 50  0000 C CNN
F 1 "1N4148" H 5700 3451 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 5700 3150 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 5700 3325 50  0001 C CNN
	1    5700 3325
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N4148 D4
U 1 1 620751EB
P 3800 3150
F 0 "D4" H 3800 2933 50  0000 C CNN
F 1 "1N4148" H 3800 3024 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 3800 2975 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 3800 3150 50  0001 C CNN
	1    3800 3150
	-1   0    0    1   
$EndComp
$Comp
L Transistor_FET:BSS84 Q1
U 1 1 620790CC
P 2925 3150
F 0 "Q1" H 3130 3104 50  0000 L CNN
F 1 "BSS84" H 3130 3195 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3125 3075 50  0001 L CIN
F 3 "http://assets.nexperia.com/documents/data-sheet/BSS84.pdf" H 2925 3150 50  0001 L CNN
	1    2925 3150
	-1   0    0    1   
$EndComp
Wire Wire Line
	3125 3150 3350 3150
Wire Wire Line
	3350 3150 3350 2650
Wire Wire Line
	3350 2650 3500 2650
Wire Wire Line
	3650 3150 3350 3150
Connection ~ 3350 3150
Wire Wire Line
	3350 3150 3350 3625
Wire Wire Line
	5550 3325 5325 3325
Wire Wire Line
	5325 3325 5325 2650
Connection ~ 5325 2650
Wire Wire Line
	5325 2650 5400 2650
Wire Wire Line
	5325 3325 5225 3325
Connection ~ 5325 3325
Wire Wire Line
	5850 3325 6125 3325
Wire Wire Line
	6125 3325 6125 2650
Wire Wire Line
	6125 2650 6000 2650
Wire Wire Line
	6125 3325 6125 3725
Wire Wire Line
	6125 3725 4925 3725
Wire Wire Line
	4925 3725 4925 3525
Connection ~ 6125 3325
Wire Wire Line
	3500 2150 3350 2150
Wire Wire Line
	3350 2150 3350 2650
Connection ~ 3350 2650
Wire Wire Line
	5400 2150 5325 2150
Wire Wire Line
	5325 2150 5325 2650
Wire Wire Line
	3950 3150 4200 3150
Wire Wire Line
	4200 3150 4200 2650
Connection ~ 4200 2650
Wire Wire Line
	4200 2650 4100 2650
$Comp
L power:GND #PWR0118
U 1 1 620849A4
P 3350 3925
F 0 "#PWR0118" H 3350 3675 50  0001 C CNN
F 1 "GND" H 3355 3752 50  0000 C CNN
F 2 "" H 3350 3925 50  0001 C CNN
F 3 "" H 3350 3925 50  0001 C CNN
	1    3350 3925
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 3925 3350 3725
$Comp
L power:GND #PWR0119
U 1 1 620856C2
P 4925 3725
F 0 "#PWR0119" H 4925 3475 50  0001 C CNN
F 1 "GND" H 4930 3552 50  0000 C CNN
F 2 "" H 4925 3725 50  0001 C CNN
F 3 "" H 4925 3725 50  0001 C CNN
	1    4925 3725
	1    0    0    -1  
$EndComp
Connection ~ 4925 3725
$Comp
L power:+3.3V #PWR0120
U 1 1 62088B9F
P 2825 2775
F 0 "#PWR0120" H 2825 2625 50  0001 C CNN
F 1 "+3.3V" H 2840 2948 50  0000 C CNN
F 2 "" H 2825 2775 50  0001 C CNN
F 3 "" H 2825 2775 50  0001 C CNN
	1    2825 2775
	1    0    0    -1  
$EndComp
Wire Wire Line
	2825 2775 2825 2950
Text HLabel 2825 3500 0    50   Output ~ 0
BMS_IND
Wire Wire Line
	2825 3500 2825 3350
Text HLabel 4925 2925 0    50   Output ~ 0
IMD_IND
Wire Wire Line
	4925 2925 4925 3125
Text Notes 3875 3875 1    50   ~ 0
BMS SWITCH
Text Notes 5175 2750 2    50   ~ 0
IMD SWITCH
Text Notes 3375 1325 0    50   ~ 0
BMS\nLOW Clear, HIGH Fault\nBMS_IND is HIGH when clear, \npulled low when fault
Text Notes 5375 1325 0    50   ~ 0
IMD\nHIGH Clear, LOW Fault\nIMD_IND is LOW when clear, \npulled HIGH when fault
Text Label 4350 2650 0    50   ~ 0
12V
Text Label 3175 1750 0    50   ~ 0
Loop_IN
Text Label 6375 1850 2    50   ~ 0
Loop_Out
$EndSCHEMATC
