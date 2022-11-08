EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 5
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 3900 3250 0    50   Input ~ 0
DCDC_IN
Text HLabel 3925 3925 0    50   Input ~ 0
BATTERY_IN
$Comp
L Device:Fuse F11
U 1 1 622D3615
P 4225 3250
F 0 "F11" V 4028 3250 50  0000 C CNN
F 1 "20A" V 4119 3250 50  0000 C CNN
F 2 "AERO_Footprints:Fuseholder_Blade_Mini_Keystone_3568" V 4155 3250 50  0001 C CNN
F 3 "~" H 4225 3250 50  0001 C CNN
	1    4225 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	3900 3250 4075 3250
Wire Wire Line
	4375 3250 4675 3250
Wire Wire Line
	4675 3250 4675 3400
Text Notes 3150 2800 0    50   ~ 0
Not really a charge controller, just a fuse and diode\n\nCrude, but functional implementation
Wire Wire Line
	4625 3850 4625 3925
Wire Wire Line
	4625 3925 3925 3925
$Comp
L Device:D_KAK D3
U 1 1 6256B1F2
P 4675 3600
F 0 "D3" V 4696 3512 50  0000 R CNN
F 1 "D_KAK" V 4605 3512 50  0000 R CNN
F 2 "Package_TO_SOT_SMD:TO-252-2" H 4675 3600 50  0001 C CNN
F 3 "~" H 4675 3600 50  0001 C CNN
	1    4675 3600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4575 3850 4625 3850
Wire Wire Line
	4675 3850 4625 3850
Connection ~ 4625 3850
$EndSCHEMATC
