EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 3
Title "Lawrence Button Main Board"
Date "2021-01-16"
Rev "A2"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:DIN-5_180degree J2
U 1 1 5F60782C
P 2650 6900
F 0 "J2" H 2650 6625 50  0000 C CNN
F 1 "MAB5SH" H 2650 6534 50  0000 C CNN
F 2 "" H 2650 6900 50  0001 C CNN
F 3 "http://www.mouser.com/ds/2/18/40_c091_abd_e-75918.pdf" H 2650 6900 50  0001 C CNN
	1    2650 6900
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R14
U 1 1 5F6105A6
P 3900 6850
F 0 "R14" H 3959 6896 50  0000 L CNN
F 1 "220R" H 3959 6805 50  0000 L CNN
F 2 "" H 3900 6850 50  0001 C CNN
F 3 "~" H 3900 6850 50  0001 C CNN
	1    3900 6850
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R13
U 1 1 5F61A34B
P 1550 7000
F 0 "R13" V 1354 7000 50  0000 C CNN
F 1 "220R" V 1445 7000 50  0000 C CNN
F 2 "" H 1550 7000 50  0001 C CNN
F 3 "~" H 1550 7000 50  0001 C CNN
	1    1550 7000
	0    1    1    0   
$EndComp
Wire Wire Line
	3900 7000 3900 6950
$Comp
L power:+5V #PWR030
U 1 1 5F61B375
P 3900 6700
F 0 "#PWR030" H 3900 6550 50  0001 C CNN
F 1 "+5V" H 3915 6873 50  0000 C CNN
F 2 "" H 3900 6700 50  0001 C CNN
F 3 "" H 3900 6700 50  0001 C CNN
	1    3900 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 6700 3900 6750
Text GLabel 1300 7000 0    50   Input ~ 0
MIDI_OUT_5V
Wire Wire Line
	1300 7000 1450 7000
$Comp
L power:GND #PWR029
U 1 1 5F61F9F6
P 2650 7250
F 0 "#PWR029" H 2650 7000 50  0001 C CNN
F 1 "GND" H 2655 7077 50  0000 C CNN
F 2 "" H 2650 7250 50  0001 C CNN
F 3 "" H 2650 7250 50  0001 C CNN
	1    2650 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 7200 2650 7250
NoConn ~ 2950 6900
NoConn ~ 2350 6900
$Comp
L Device:Ferrite_Bead_Small FB4
U 1 1 5F63DC5A
P 3350 7000
F 0 "FB4" V 3113 7000 50  0000 C CNN
F 1 "100 @ 100MHz" V 3204 7000 50  0000 C CNN
F 2 "" V 3280 7000 50  0001 C CNN
F 3 "~" H 3350 7000 50  0001 C CNN
	1    3350 7000
	0    1    1    0   
$EndComp
$Comp
L Device:Ferrite_Bead_Small FB3
U 1 1 5F63E7B4
P 2000 7000
F 0 "FB3" V 1763 7000 50  0000 C CNN
F 1 "100 @ 100MHz" V 1854 7000 50  0000 C CNN
F 2 "" V 1930 7000 50  0001 C CNN
F 3 "~" H 2000 7000 50  0001 C CNN
	1    2000 7000
	0    1    1    0   
$EndComp
Wire Wire Line
	1650 7000 1900 7000
Wire Wire Line
	2100 7000 2350 7000
Wire Wire Line
	2950 7000 3250 7000
Wire Wire Line
	3450 7000 3900 7000
NoConn ~ 22850 4200
Wire Wire Line
	9350 2950 9350 3000
$Comp
L power:GND #PWR058
U 1 1 5FFB68FE
P 9050 2700
F 0 "#PWR058" H 9050 2450 50  0001 C CNN
F 1 "GND" H 9055 2527 50  0000 C CNN
F 2 "" H 9050 2700 50  0001 C CNN
F 3 "" H 9050 2700 50  0001 C CNN
	1    9050 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 1800 9350 1750
Wire Wire Line
	9800 1450 9750 1450
Text GLabel 9750 2650 2    50   Input ~ 0
OUTR_P
Text GLabel 9450 2950 3    50   Input ~ 0
OUTR_N
Text GLabel 9450 1750 3    50   Input ~ 0
OUTL_N
Text GLabel 9800 1450 2    50   Input ~ 0
OUTL_P
Wire Notes Line
	4200 7650 4200 6200
Wire Notes Line
	700  6200 700  7650
$Comp
L Audio:PCM5122PW U4
U 1 1 60051B55
P 2400 3050
F 0 "U4" H 2750 4100 50  0000 C CNN
F 1 "PCM5122PW" H 2850 4000 50  0000 C CNN
F 2 "Package_SO:TSSOP-28_4.4x9.7mm_P0.65mm" H 2400 3050 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/pcm5122.pdf" H 2400 4100 50  0001 C CNN
	1    2400 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C23
U 1 1 600544C2
P 3450 3400
F 0 "C23" H 3542 3446 50  0000 L CNN
F 1 "2.2u" H 3542 3355 50  0000 L CNN
F 2 "" H 3450 3400 50  0001 C CNN
F 3 "~" H 3450 3400 50  0001 C CNN
	1    3450 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 3350 3250 3350
Wire Wire Line
	3250 3350 3250 3300
Wire Wire Line
	3250 3300 3450 3300
Wire Wire Line
	3450 3500 3250 3500
Wire Wire Line
	3250 3500 3250 3450
Wire Wire Line
	3250 3450 3100 3450
Text GLabel 3100 2350 2    50   Input ~ 0
DAC_OUTL
Text GLabel 3100 2450 2    50   Input ~ 0
DAC_OUTR
$Comp
L Device:C_Small C24
U 1 1 60063536
P 3650 3800
F 0 "C24" H 3742 3846 50  0000 L CNN
F 1 "0.1u" H 3742 3755 50  0000 L CNN
F 2 "" H 3650 3800 50  0001 C CNN
F 3 "~" H 3650 3800 50  0001 C CNN
	1    3650 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 3650 3650 3650
Wire Wire Line
	3650 3650 3650 3700
$Comp
L power:GND #PWR042
U 1 1 6006A50A
P 3650 3950
F 0 "#PWR042" H 3650 3700 50  0001 C CNN
F 1 "GND" H 3655 3777 50  0000 C CNN
F 2 "" H 3650 3950 50  0001 C CNN
F 3 "" H 3650 3950 50  0001 C CNN
	1    3650 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 3900 3650 3950
$Comp
L Device:C_Small C22
U 1 1 6006CFB7
P 3150 3900
F 0 "C22" H 3242 3946 50  0000 L CNN
F 1 "2.2u" H 3242 3855 50  0000 L CNN
F 2 "" H 3150 3900 50  0001 C CNN
F 3 "~" H 3150 3900 50  0001 C CNN
	1    3150 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 3750 3150 3750
Wire Wire Line
	3150 3750 3150 3800
$Comp
L power:GND #PWR040
U 1 1 600720CA
P 3150 4050
F 0 "#PWR040" H 3150 3800 50  0001 C CNN
F 1 "GND" H 3155 3877 50  0000 C CNN
F 2 "" H 3150 4050 50  0001 C CNN
F 3 "" H 3150 4050 50  0001 C CNN
	1    3150 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 4000 3150 4050
$Comp
L power:GND #PWR041
U 1 1 60074BF6
P 3200 3150
F 0 "#PWR041" H 3200 2900 50  0001 C CNN
F 1 "GND" H 3205 2977 50  0000 C CNN
F 2 "" H 3200 3150 50  0001 C CNN
F 3 "" H 3200 3150 50  0001 C CNN
	1    3200 3150
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR036
U 1 1 60082080
P 2600 4050
F 0 "#PWR036" H 2600 3800 50  0001 C CNN
F 1 "GND" H 2605 3877 50  0000 C CNN
F 2 "" H 2600 4050 50  0001 C CNN
F 3 "" H 2600 4050 50  0001 C CNN
	1    2600 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 4050 2600 4000
Wire Wire Line
	2600 4000 2500 4000
Wire Wire Line
	2300 4000 2300 3950
Wire Wire Line
	2400 3950 2400 4000
Connection ~ 2400 4000
Wire Wire Line
	2400 4000 2300 4000
Wire Wire Line
	2500 3950 2500 4000
Connection ~ 2500 4000
Wire Wire Line
	2500 4000 2400 4000
$Comp
L Connector:XLR3_Ground J4
U 1 1 5FFB179E
P 9450 2650
F 0 "J4" H 9500 3000 50  0000 L CNN
F 1 "RIGHT" H 9500 2900 50  0000 L CNN
F 2 "" H 9450 2650 50  0001 C CNN
F 3 " ~" H 9450 2650 50  0001 C CNN
	1    9450 2650
	1    0    0    -1  
$EndComp
$Comp
L Connector:XLR3_Ground J3
U 1 1 5FFAFBAB
P 9450 1450
F 0 "J3" H 9500 1800 50  0000 L CNN
F 1 "LEFT" H 9500 1700 50  0000 L CNN
F 2 "" H 9450 1450 50  0001 C CNN
F 3 " ~" H 9450 1450 50  0001 C CNN
	1    9450 1450
	1    0    0    -1  
$EndComp
NoConn ~ 9350 1800
NoConn ~ 9350 3000
$Comp
L power:GND #PWR057
U 1 1 60017FE3
P 9050 1500
F 0 "#PWR057" H 9050 1250 50  0001 C CNN
F 1 "GND" H 9055 1327 50  0000 C CNN
F 2 "" H 9050 1500 50  0001 C CNN
F 3 "" H 9050 1500 50  0001 C CNN
	1    9050 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 1500 9050 1450
Wire Wire Line
	9050 1450 9150 1450
Wire Wire Line
	9150 2650 9050 2650
Wire Wire Line
	9050 2650 9050 2700
$Comp
L maxlibrary:DRV135 U5
U 1 1 600199C3
P 5300 2250
F 0 "U5" H 5750 900 50  0000 R CNN
F 1 "DRV135" H 5850 800 50  0000 R CNN
F 2 "" H 5300 2250 50  0001 C CNN
F 3 "" H 5300 2250 50  0001 C CNN
	1    5300 2250
	1    0    0    -1  
$EndComp
Text GLabel 4900 3050 0    50   Input ~ 0
DAC_OUTL
$Comp
L power:-5V #PWR047
U 1 1 6001AC91
P 5150 2400
F 0 "#PWR047" H 5150 2500 50  0001 C CNN
F 1 "-5V" H 5165 2573 50  0000 C CNN
F 2 "" H 5150 2400 50  0001 C CNN
F 3 "" H 5150 2400 50  0001 C CNN
	1    5150 2400
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR049
U 1 1 6001B9A4
P 5350 2400
F 0 "#PWR049" H 5350 2250 50  0001 C CNN
F 1 "+5V" H 5365 2573 50  0000 C CNN
F 2 "" H 5350 2400 50  0001 C CNN
F 3 "" H 5350 2400 50  0001 C CNN
	1    5350 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C31
U 1 1 60026CE7
P 6350 2800
F 0 "C31" H 6442 2846 50  0000 L CNN
F 1 "10u" H 6442 2755 50  0000 L CNN
F 2 "" H 6350 2800 50  0001 C CNN
F 3 "~" H 6350 2800 50  0001 C CNN
	1    6350 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C32
U 1 1 60027C4C
P 6350 3250
F 0 "C32" H 6442 3296 50  0000 L CNN
F 1 "10u" H 6442 3205 50  0000 L CNN
F 2 "" H 6350 3250 50  0001 C CNN
F 3 "~" H 6350 3250 50  0001 C CNN
	1    6350 3250
	1    0    0    -1  
$EndComp
Text GLabel 8250 2700 2    50   Input ~ 0
OUTL_P
Text GLabel 8250 3350 2    50   Input ~ 0
OUTL_N
$Comp
L Device:D_Schottky D6
U 1 1 6002BE59
P 6750 2500
F 0 "D6" V 6704 2580 50  0000 L CNN
F 1 "SS210" V 6795 2580 50  0000 L CNN
F 2 "" H 6750 2500 50  0001 C CNN
F 3 "~" H 6750 2500 50  0001 C CNN
	1    6750 2500
	0    1    1    0   
$EndComp
$Comp
L Device:D_Schottky D8
U 1 1 6002DA8B
P 7250 3150
F 0 "D8" V 7204 3230 50  0000 L CNN
F 1 "SS210" V 7295 3230 50  0000 L CNN
F 2 "" H 7250 3150 50  0001 C CNN
F 3 "~" H 7250 3150 50  0001 C CNN
	1    7250 3150
	0    1    1    0   
$EndComp
$Comp
L Device:D_Schottky D7
U 1 1 6002ED6A
P 6750 3550
F 0 "D7" V 6704 3630 50  0000 L CNN
F 1 "SS210" V 6795 3630 50  0000 L CNN
F 2 "" H 6750 3550 50  0001 C CNN
F 3 "~" H 6750 3550 50  0001 C CNN
	1    6750 3550
	0    1    1    0   
$EndComp
$Comp
L Device:D_Schottky D9
U 1 1 6002F928
P 7800 2900
F 0 "D9" V 7754 2980 50  0000 L CNN
F 1 "SS210" V 7845 2980 50  0000 L CNN
F 2 "" H 7800 2900 50  0001 C CNN
F 3 "~" H 7800 2900 50  0001 C CNN
	1    7800 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	5950 3350 6350 3350
Connection ~ 6350 3350
Wire Wire Line
	6350 3350 6750 3350
Wire Wire Line
	8250 2700 7800 2700
Connection ~ 6350 2700
Wire Wire Line
	6350 2700 5950 2700
Wire Wire Line
	5950 2900 6350 2900
Wire Wire Line
	6350 3150 5950 3150
$Comp
L power:+5V #PWR050
U 1 1 600344AB
P 6750 2250
F 0 "#PWR050" H 6750 2100 50  0001 C CNN
F 1 "+5V" H 6765 2423 50  0000 C CNN
F 2 "" H 6750 2250 50  0001 C CNN
F 3 "" H 6750 2250 50  0001 C CNN
	1    6750 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 2250 6750 2300
Wire Wire Line
	6750 2650 6750 2700
Connection ~ 6750 2700
Wire Wire Line
	6750 2700 6350 2700
Wire Wire Line
	6750 2300 7250 2300
Wire Wire Line
	7250 2300 7250 3000
Connection ~ 6750 2300
Wire Wire Line
	6750 2300 6750 2350
Wire Wire Line
	7250 3300 7250 3350
Connection ~ 7250 3350
Wire Wire Line
	7250 3350 8250 3350
$Comp
L power:-5V #PWR051
U 1 1 60038925
P 6750 3800
F 0 "#PWR051" H 6750 3900 50  0001 C CNN
F 1 "-5V" H 6765 3973 50  0000 C CNN
F 2 "" H 6750 3800 50  0001 C CNN
F 3 "" H 6750 3800 50  0001 C CNN
	1    6750 3800
	-1   0    0    1   
$EndComp
Wire Wire Line
	6750 3800 6750 3750
Wire Wire Line
	6750 3400 6750 3350
Connection ~ 6750 3350
Wire Wire Line
	6750 3350 7250 3350
Wire Wire Line
	7800 2750 7800 2700
Connection ~ 7800 2700
Wire Wire Line
	7800 2700 6750 2700
Wire Wire Line
	7800 3050 7800 3750
Wire Wire Line
	7800 3750 6750 3750
Connection ~ 6750 3750
Wire Wire Line
	6750 3750 6750 3700
$Comp
L power:GND #PWR048
U 1 1 6003D8F6
P 5300 3650
F 0 "#PWR048" H 5300 3400 50  0001 C CNN
F 1 "GND" H 5305 3477 50  0000 C CNN
F 2 "" H 5300 3650 50  0001 C CNN
F 3 "" H 5300 3650 50  0001 C CNN
	1    5300 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 3650 5300 3600
NoConn ~ 3100 2650
NoConn ~ 3100 2750
NoConn ~ 3100 2850
NoConn ~ 3100 2950
Text GLabel 1700 2450 0    50   Input ~ 0
I2S_CK
Text GLabel 1700 2550 0    50   Input ~ 0
I2S_WS
Text GLabel 1700 2650 0    50   Input ~ 0
I2S_SD
Text GLabel 1700 2850 0    50   Input ~ 0
I2C1_SCL
Text GLabel 1700 2950 0    50   Input ~ 0
I2C1_SDA
Wire Wire Line
	3200 3150 3100 3150
$Comp
L power:+3.3V #PWR034
U 1 1 6005ABAD
P 2150 2050
F 0 "#PWR034" H 2150 1900 50  0001 C CNN
F 1 "+3.3V" H 2165 2223 50  0000 C CNN
F 2 "" H 2150 2050 50  0001 C CNN
F 3 "" H 2150 2050 50  0001 C CNN
	1    2150 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 2050 2150 2100
Wire Wire Line
	2150 2100 2300 2100
Wire Wire Line
	2500 2100 2500 2150
Wire Wire Line
	2400 2150 2400 2100
Connection ~ 2400 2100
Wire Wire Line
	2400 2100 2500 2100
Wire Wire Line
	2300 2150 2300 2100
Connection ~ 2300 2100
Wire Wire Line
	2300 2100 2400 2100
Wire Notes Line
	8850 900  8850 3400
Wire Notes Line
	8850 3400 10200 3400
Wire Notes Line
	10200 3400 10200 900 
Wire Notes Line
	10200 900  8850 900 
$Comp
L maxlibrary:DRV135 U6
U 1 1 6008199B
P 5300 4450
F 0 "U6" H 5750 3100 50  0000 R CNN
F 1 "DRV135" H 5850 3000 50  0000 R CNN
F 2 "" H 5300 4450 50  0001 C CNN
F 3 "" H 5300 4450 50  0001 C CNN
	1    5300 4450
	1    0    0    -1  
$EndComp
Text GLabel 4900 5250 0    50   Input ~ 0
DAC_OUTR
$Comp
L power:-5V #PWR052
U 1 1 600819A6
P 5150 4600
F 0 "#PWR052" H 5150 4700 50  0001 C CNN
F 1 "-5V" H 5165 4773 50  0000 C CNN
F 2 "" H 5150 4600 50  0001 C CNN
F 3 "" H 5150 4600 50  0001 C CNN
	1    5150 4600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR054
U 1 1 600819B0
P 5350 4600
F 0 "#PWR054" H 5350 4450 50  0001 C CNN
F 1 "+5V" H 5365 4773 50  0000 C CNN
F 2 "" H 5350 4600 50  0001 C CNN
F 3 "" H 5350 4600 50  0001 C CNN
	1    5350 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C33
U 1 1 600819EE
P 6350 5000
F 0 "C33" H 6442 5046 50  0000 L CNN
F 1 "10u" H 6442 4955 50  0000 L CNN
F 2 "" H 6350 5000 50  0001 C CNN
F 3 "~" H 6350 5000 50  0001 C CNN
	1    6350 5000
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C34
U 1 1 600819F8
P 6350 5450
F 0 "C34" H 6442 5496 50  0000 L CNN
F 1 "10u" H 6442 5405 50  0000 L CNN
F 2 "" H 6350 5450 50  0001 C CNN
F 3 "~" H 6350 5450 50  0001 C CNN
	1    6350 5450
	1    0    0    -1  
$EndComp
Text GLabel 8250 4900 2    50   Input ~ 0
OUTR_P
Text GLabel 8250 5550 2    50   Input ~ 0
OUTR_N
$Comp
L Device:D_Schottky D10
U 1 1 60081A04
P 6750 4700
F 0 "D10" V 6704 4780 50  0000 L CNN
F 1 "SS210" V 6795 4780 50  0000 L CNN
F 2 "" H 6750 4700 50  0001 C CNN
F 3 "~" H 6750 4700 50  0001 C CNN
	1    6750 4700
	0    1    1    0   
$EndComp
$Comp
L Device:D_Schottky D12
U 1 1 60081A0E
P 7250 5350
F 0 "D12" V 7204 5430 50  0000 L CNN
F 1 "SS210" V 7295 5430 50  0000 L CNN
F 2 "" H 7250 5350 50  0001 C CNN
F 3 "~" H 7250 5350 50  0001 C CNN
	1    7250 5350
	0    1    1    0   
$EndComp
$Comp
L Device:D_Schottky D11
U 1 1 60081A18
P 6750 5750
F 0 "D11" V 6704 5830 50  0000 L CNN
F 1 "SS210" V 6795 5830 50  0000 L CNN
F 2 "" H 6750 5750 50  0001 C CNN
F 3 "~" H 6750 5750 50  0001 C CNN
	1    6750 5750
	0    1    1    0   
$EndComp
$Comp
L Device:D_Schottky D13
U 1 1 60081A22
P 7800 5100
F 0 "D13" V 7754 5180 50  0000 L CNN
F 1 "SS210" V 7845 5180 50  0000 L CNN
F 2 "" H 7800 5100 50  0001 C CNN
F 3 "~" H 7800 5100 50  0001 C CNN
	1    7800 5100
	0    1    1    0   
$EndComp
Wire Wire Line
	5950 5550 6350 5550
Connection ~ 6350 5550
Wire Wire Line
	6350 5550 6750 5550
Wire Wire Line
	8250 4900 7800 4900
Connection ~ 6350 4900
Wire Wire Line
	6350 4900 5950 4900
Wire Wire Line
	5950 5100 6350 5100
Wire Wire Line
	6350 5350 5950 5350
$Comp
L power:+5V #PWR055
U 1 1 60081A34
P 6750 4450
F 0 "#PWR055" H 6750 4300 50  0001 C CNN
F 1 "+5V" H 6765 4623 50  0000 C CNN
F 2 "" H 6750 4450 50  0001 C CNN
F 3 "" H 6750 4450 50  0001 C CNN
	1    6750 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 4450 6750 4500
Wire Wire Line
	6750 4850 6750 4900
Connection ~ 6750 4900
Wire Wire Line
	6750 4900 6350 4900
Wire Wire Line
	6750 4500 7250 4500
Wire Wire Line
	7250 4500 7250 5200
Connection ~ 6750 4500
Wire Wire Line
	6750 4500 6750 4550
Wire Wire Line
	7250 5500 7250 5550
Connection ~ 7250 5550
Wire Wire Line
	7250 5550 8250 5550
$Comp
L power:-5V #PWR056
U 1 1 60081A49
P 6750 6000
F 0 "#PWR056" H 6750 6100 50  0001 C CNN
F 1 "-5V" H 6765 6173 50  0000 C CNN
F 2 "" H 6750 6000 50  0001 C CNN
F 3 "" H 6750 6000 50  0001 C CNN
	1    6750 6000
	-1   0    0    1   
$EndComp
Wire Wire Line
	6750 6000 6750 5950
Wire Wire Line
	6750 5600 6750 5550
Connection ~ 6750 5550
Wire Wire Line
	6750 5550 7250 5550
Wire Wire Line
	7800 4950 7800 4900
Connection ~ 7800 4900
Wire Wire Line
	7800 4900 6750 4900
Wire Wire Line
	7800 5250 7800 5950
Wire Wire Line
	7800 5950 6750 5950
Connection ~ 6750 5950
Wire Wire Line
	6750 5950 6750 5900
$Comp
L power:GND #PWR053
U 1 1 60081A5E
P 5300 5850
F 0 "#PWR053" H 5300 5600 50  0001 C CNN
F 1 "GND" H 5305 5677 50  0000 C CNN
F 2 "" H 5300 5850 50  0001 C CNN
F 3 "" H 5300 5850 50  0001 C CNN
	1    5300 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 5850 5300 5800
Wire Notes Line
	4400 900  4400 6350
Wire Notes Line
	4400 6350 8700 6350
Wire Notes Line
	8700 6350 8700 900 
Wire Notes Line
	8700 900  4400 900 
Text Notes 8850 850  0    118  ~ 0
XLR OUT
Text Notes 4400 850  0    118  ~ 0
LINE DRIVERS
Text Notes 850  850  0    118  ~ 0
DAC
Text Notes 700  6150 0    118  ~ 0
MIDI OUT
$Comp
L power:+3.3V #PWR031
U 1 1 60132F0F
P 1800 1200
F 0 "#PWR031" H 1800 1050 50  0001 C CNN
F 1 "+3.3V" H 1815 1373 50  0000 C CNN
F 2 "" H 1800 1200 50  0001 C CNN
F 3 "" H 1800 1200 50  0001 C CNN
	1    1800 1200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR038
U 1 1 6013362E
P 2950 1600
F 0 "#PWR038" H 2950 1350 50  0001 C CNN
F 1 "GND" H 2955 1427 50  0000 C CNN
F 2 "" H 2950 1600 50  0001 C CNN
F 3 "" H 2950 1600 50  0001 C CNN
	1    2950 1600
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C19
U 1 1 60133B0D
P 2150 1400
F 0 "C19" H 2242 1446 50  0000 L CNN
F 1 "2.2u" H 2242 1355 50  0000 L CNN
F 2 "" H 2150 1400 50  0001 C CNN
F 3 "~" H 2150 1400 50  0001 C CNN
	1    2150 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C20
U 1 1 60134243
P 2550 1400
F 0 "C20" H 2642 1446 50  0000 L CNN
F 1 "0.1u" H 2642 1355 50  0000 L CNN
F 2 "" H 2550 1400 50  0001 C CNN
F 3 "~" H 2550 1400 50  0001 C CNN
	1    2550 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C21
U 1 1 601349EC
P 2950 1400
F 0 "C21" H 3042 1446 50  0000 L CNN
F 1 "0.1u" H 3042 1355 50  0000 L CNN
F 2 "" H 2950 1400 50  0001 C CNN
F 3 "~" H 2950 1400 50  0001 C CNN
	1    2950 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C18
U 1 1 60145D2E
P 1800 1400
F 0 "C18" H 1892 1446 50  0000 L CNN
F 1 "10u" H 1892 1355 50  0000 L CNN
F 2 "" H 1800 1400 50  0001 C CNN
F 3 "~" H 1800 1400 50  0001 C CNN
	1    1800 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 1250 2950 1300
Wire Wire Line
	2550 1300 2550 1250
Connection ~ 2550 1250
Wire Wire Line
	2550 1250 2950 1250
Wire Wire Line
	2150 1300 2150 1250
Connection ~ 2150 1250
Wire Wire Line
	2150 1250 2550 1250
Wire Wire Line
	1800 1300 1800 1250
Wire Wire Line
	1800 1250 2150 1250
Wire Wire Line
	1800 1500 1800 1550
Wire Wire Line
	2950 1550 2950 1500
Wire Wire Line
	2150 1550 2150 1500
Wire Wire Line
	1800 1550 2150 1550
Connection ~ 2150 1550
Wire Wire Line
	2150 1550 2550 1550
Wire Wire Line
	2550 1500 2550 1550
Connection ~ 2550 1550
Wire Wire Line
	2550 1550 2950 1550
Wire Wire Line
	1800 1200 1800 1250
Connection ~ 1800 1250
Wire Wire Line
	2950 1600 2950 1550
Connection ~ 2950 1550
Wire Notes Line
	4150 900  850  900 
Wire Wire Line
	5350 4600 5350 4650
Wire Wire Line
	5150 4600 5150 4650
Wire Wire Line
	5350 2400 5350 2450
Wire Wire Line
	5150 2400 5150 2450
$Comp
L power:GND #PWR044
U 1 1 60207B95
P 6050 1700
F 0 "#PWR044" H 6050 1450 50  0001 C CNN
F 1 "GND" H 6055 1527 50  0000 C CNN
F 2 "" H 6050 1700 50  0001 C CNN
F 3 "" H 6050 1700 50  0001 C CNN
	1    6050 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C26
U 1 1 60207BA9
P 5250 1500
F 0 "C26" H 5342 1546 50  0000 L CNN
F 1 "1u" H 5342 1455 50  0000 L CNN
F 2 "" H 5250 1500 50  0001 C CNN
F 3 "~" H 5250 1500 50  0001 C CNN
	1    5250 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C27
U 1 1 60207BB3
P 5650 1500
F 0 "C27" H 5742 1546 50  0000 L CNN
F 1 "1u" H 5742 1455 50  0000 L CNN
F 2 "" H 5650 1500 50  0001 C CNN
F 3 "~" H 5650 1500 50  0001 C CNN
	1    5650 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C25
U 1 1 60207BBD
P 4850 1500
F 0 "C25" H 4942 1546 50  0000 L CNN
F 1 "10u" H 4942 1455 50  0000 L CNN
F 2 "" H 4850 1500 50  0001 C CNN
F 3 "~" H 4850 1500 50  0001 C CNN
	1    4850 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 1350 5650 1400
Wire Wire Line
	5250 1400 5250 1350
Connection ~ 5250 1350
Wire Wire Line
	5250 1350 5650 1350
Wire Wire Line
	4850 1400 4850 1350
Wire Wire Line
	4850 1600 4850 1650
Wire Wire Line
	5650 1650 5650 1600
Wire Wire Line
	5250 1600 5250 1650
Connection ~ 5250 1650
Wire Wire Line
	5250 1650 5650 1650
Wire Wire Line
	4850 1300 4850 1350
Connection ~ 4850 1350
Connection ~ 5650 1650
$Comp
L power:+5V #PWR043
U 1 1 6021B1CF
P 4850 1300
F 0 "#PWR043" H 4850 1150 50  0001 C CNN
F 1 "+5V" H 4865 1473 50  0000 C CNN
F 2 "" H 4850 1300 50  0001 C CNN
F 3 "" H 4850 1300 50  0001 C CNN
	1    4850 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 1650 5250 1650
Wire Wire Line
	4850 1350 5250 1350
$Comp
L Device:C_Small C29
U 1 1 60254F84
P 7150 1500
F 0 "C29" H 7242 1546 50  0000 L CNN
F 1 "1u" H 7242 1455 50  0000 L CNN
F 2 "" H 7150 1500 50  0001 C CNN
F 3 "~" H 7150 1500 50  0001 C CNN
	1    7150 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C30
U 1 1 60254F8E
P 7550 1500
F 0 "C30" H 7642 1546 50  0000 L CNN
F 1 "1u" H 7642 1455 50  0000 L CNN
F 2 "" H 7550 1500 50  0001 C CNN
F 3 "~" H 7550 1500 50  0001 C CNN
	1    7550 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C28
U 1 1 60254F98
P 6750 1500
F 0 "C28" H 6842 1546 50  0000 L CNN
F 1 "10u" H 6842 1455 50  0000 L CNN
F 2 "" H 6750 1500 50  0001 C CNN
F 3 "~" H 6750 1500 50  0001 C CNN
	1    6750 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 1350 7550 1400
Wire Wire Line
	7150 1400 7150 1350
Connection ~ 7150 1350
Wire Wire Line
	7150 1350 7550 1350
Wire Wire Line
	6750 1400 6750 1350
Wire Wire Line
	6750 1600 6750 1650
Wire Wire Line
	7550 1650 7550 1600
Wire Wire Line
	7150 1600 7150 1650
Connection ~ 7150 1650
Wire Wire Line
	7150 1650 7550 1650
Wire Wire Line
	6750 1650 7150 1650
Wire Wire Line
	6750 1350 7150 1350
$Comp
L power:GND #PWR046
U 1 1 6025D72B
P 7950 1300
F 0 "#PWR046" H 7950 1050 50  0001 C CNN
F 1 "GND" H 7955 1127 50  0000 C CNN
F 2 "" H 7950 1300 50  0001 C CNN
F 3 "" H 7950 1300 50  0001 C CNN
	1    7950 1300
	-1   0    0    1   
$EndComp
$Comp
L power:-5V #PWR045
U 1 1 60267E61
P 6750 1700
F 0 "#PWR045" H 6750 1800 50  0001 C CNN
F 1 "-5V" H 6765 1873 50  0000 C CNN
F 2 "" H 6750 1700 50  0001 C CNN
F 3 "" H 6750 1700 50  0001 C CNN
	1    6750 1700
	-1   0    0    1   
$EndComp
Wire Wire Line
	6750 1700 6750 1650
Connection ~ 6750 1650
Connection ~ 7550 1350
Wire Notes Line
	4200 6200 700  6200
Wire Notes Line
	700  7650 4200 7650
$Comp
L Diode:Z2SMBxxx D4
U 1 1 603AB800
P 6050 1500
F 0 "D4" V 6004 1580 50  0000 L CNN
F 1 "P6SMB6.8CA" V 6095 1580 50  0000 L CNN
F 2 "Diode_SMD:D_SMB" H 6050 1325 50  0001 C CNN
F 3 "https://diotec.com/tl_files/diotec/files/pdf/datasheets/z2smb1.pdf" H 6050 1500 50  0001 C CNN
	1    6050 1500
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 1350 6050 1350
Connection ~ 5650 1350
Wire Wire Line
	6050 1650 6050 1700
Wire Wire Line
	6050 1650 5650 1650
Connection ~ 6050 1650
$Comp
L Diode:Z2SMBxxx D5
U 1 1 603D268F
P 7950 1500
F 0 "D5" V 7904 1580 50  0000 L CNN
F 1 "P6SMB6.8CA" V 7995 1580 50  0000 L CNN
F 2 "Diode_SMD:D_SMB" H 7950 1325 50  0001 C CNN
F 3 "https://diotec.com/tl_files/diotec/files/pdf/datasheets/z2smb1.pdf" H 7950 1500 50  0001 C CNN
	1    7950 1500
	0    1    1    0   
$EndComp
Wire Wire Line
	7950 1300 7950 1350
Wire Wire Line
	7550 1350 7950 1350
Connection ~ 7950 1350
Wire Wire Line
	7950 1650 7550 1650
Connection ~ 7550 1650
Text Notes 3450 850  0    59   ~ 0
I2C addr: 0x98
$Comp
L Device:C_Small C35
U 1 1 604C30BC
P 9050 4150
F 0 "C35" H 9142 4196 50  0000 L CNN
F 1 "10u" H 9142 4105 50  0000 L CNN
F 2 "" H 9050 4150 50  0001 C CNN
F 3 "~" H 9050 4150 50  0001 C CNN
	1    9050 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C36
U 1 1 604C3751
P 9400 4150
F 0 "C36" H 9492 4196 50  0000 L CNN
F 1 "10u" H 9492 4105 50  0000 L CNN
F 2 "" H 9400 4150 50  0001 C CNN
F 3 "~" H 9400 4150 50  0001 C CNN
	1    9400 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR063
U 1 1 604C3FEC
P 9400 4000
F 0 "#PWR063" H 9400 3750 50  0001 C CNN
F 1 "GND" H 9405 3827 50  0000 C CNN
F 2 "" H 9400 4000 50  0001 C CNN
F 3 "" H 9400 4000 50  0001 C CNN
	1    9400 4000
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR060
U 1 1 604C4553
P 9050 4300
F 0 "#PWR060" H 9050 4050 50  0001 C CNN
F 1 "GND" H 9055 4127 50  0000 C CNN
F 2 "" H 9050 4300 50  0001 C CNN
F 3 "" H 9050 4300 50  0001 C CNN
	1    9050 4300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR059
U 1 1 604C4DB5
P 9050 4000
F 0 "#PWR059" H 9050 3850 50  0001 C CNN
F 1 "+5V" H 9065 4173 50  0000 C CNN
F 2 "" H 9050 4000 50  0001 C CNN
F 3 "" H 9050 4000 50  0001 C CNN
	1    9050 4000
	1    0    0    -1  
$EndComp
$Comp
L power:-5V #PWR064
U 1 1 604C5253
P 9400 4300
F 0 "#PWR064" H 9400 4400 50  0001 C CNN
F 1 "-5V" H 9415 4473 50  0000 C CNN
F 2 "" H 9400 4300 50  0001 C CNN
F 3 "" H 9400 4300 50  0001 C CNN
	1    9400 4300
	-1   0    0    1   
$EndComp
Wire Wire Line
	9400 4300 9400 4250
Wire Wire Line
	9050 4300 9050 4250
Wire Wire Line
	9050 4050 9050 4000
Wire Wire Line
	9400 4050 9400 4000
Wire Notes Line
	8850 6350 10600 6350
Text Notes 8850 3650 0    118  ~ 0
-5V CHARGE PUMP
Wire Notes Line
	10600 3700 10600 6350
Wire Notes Line
	8850 6350 8850 3700
Wire Notes Line
	8850 3700 10600 3700
$Comp
L maxlibrary:LTC3261 U7
U 1 1 60592C6E
P 9700 4150
F 0 "U7" H 9900 2350 50  0000 C CNN
F 1 "LTC3261" H 10000 2250 50  0000 C CNN
F 2 "" H 9900 2400 50  0001 C CNN
F 3 "" H 9900 2400 50  0001 C CNN
	1    9700 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR065
U 1 1 605A57CA
P 9700 6000
F 0 "#PWR065" H 9700 5750 50  0001 C CNN
F 1 "GND" H 9705 5827 50  0000 C CNN
F 2 "" H 9700 6000 50  0001 C CNN
F 3 "" H 9700 6000 50  0001 C CNN
	1    9700 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 6000 9700 5950
$Comp
L power:GND #PWR067
U 1 1 605ACF59
P 10150 5600
F 0 "#PWR067" H 10150 5350 50  0001 C CNN
F 1 "GND" H 10155 5427 50  0000 C CNN
F 2 "" H 10150 5600 50  0001 C CNN
F 3 "" H 10150 5600 50  0001 C CNN
	1    10150 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 5550 10150 5550
Wire Wire Line
	10150 5550 10150 5600
$Comp
L power:GND #PWR062
U 1 1 605B4575
P 9250 5600
F 0 "#PWR062" H 9250 5350 50  0001 C CNN
F 1 "GND" H 9255 5427 50  0000 C CNN
F 2 "" H 9250 5600 50  0001 C CNN
F 3 "" H 9250 5600 50  0001 C CNN
	1    9250 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 5550 9250 5550
Wire Wire Line
	9250 5550 9250 5600
$Comp
L power:+5V #PWR061
U 1 1 605BBDE5
P 9250 5050
F 0 "#PWR061" H 9250 4900 50  0001 C CNN
F 1 "+5V" H 9265 5223 50  0000 C CNN
F 2 "" H 9250 5050 50  0001 C CNN
F 3 "" H 9250 5050 50  0001 C CNN
	1    9250 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 5050 9250 5100
Wire Wire Line
	9250 5100 9350 5100
Wire Wire Line
	9350 5350 9250 5350
Wire Wire Line
	9250 5350 9250 5100
Connection ~ 9250 5100
$Comp
L power:-5V #PWR066
U 1 1 605CB3B1
P 10150 5150
F 0 "#PWR066" H 10150 5250 50  0001 C CNN
F 1 "-5V" H 10165 5323 50  0000 C CNN
F 2 "" H 10150 5150 50  0001 C CNN
F 3 "" H 10150 5150 50  0001 C CNN
	1    10150 5150
	-1   0    0    1   
$EndComp
Wire Wire Line
	10050 5100 10150 5100
Wire Wire Line
	10150 5100 10150 5150
$Comp
L Device:C_Small C37
U 1 1 605D382E
P 9700 4650
F 0 "C37" H 9792 4696 50  0000 L CNN
F 1 "1u" H 9792 4605 50  0000 L CNN
F 2 "" H 9700 4650 50  0001 C CNN
F 3 "~" H 9700 4650 50  0001 C CNN
	1    9700 4650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9600 4650 9550 4650
Wire Wire Line
	9550 4650 9550 4750
Wire Wire Line
	9800 4650 9850 4650
Wire Wire Line
	9850 4650 9850 4750
Text GLabel 1700 2350 0    50   Input ~ 0
I2S_MCK
Text GLabel 5000 7050 0    50   Input ~ 0
I2S_CK
Text GLabel 5000 7300 0    50   Input ~ 0
I2S_WS
Text GLabel 5000 7550 0    50   Input ~ 0
I2S_SD
Text GLabel 5000 6800 0    50   Input ~ 0
I2S_MCK
Text GLabel 6150 7050 0    50   Input ~ 0
DAC_OUTL
Text GLabel 6150 6800 0    50   Input ~ 0
DAC_OUTR
Text GLabel 6150 7300 0    50   Input ~ 0
I2C1_SCL
Text GLabel 6150 7550 0    50   Input ~ 0
I2C1_SDA
$Comp
L power:GND #PWR?
U 1 1 615E225A
P 2400 5150
AR Path="/615E225A" Ref="#PWR?"  Part="1" 
AR Path="/5F606192/615E225A" Ref="#PWR035"  Part="1" 
F 0 "#PWR035" H 2400 4900 50  0001 C CNN
F 1 "GND" H 2405 4977 50  0000 C CNN
F 2 "" H 2400 5150 50  0001 C CNN
F 3 "" H 2400 5150 50  0001 C CNN
	1    2400 5150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 615E2260
P 2100 5150
AR Path="/615E2260" Ref="#PWR?"  Part="1" 
AR Path="/5F606192/615E2260" Ref="#PWR033"  Part="1" 
F 0 "#PWR033" H 2100 4900 50  0001 C CNN
F 1 "GND" H 2105 4977 50  0000 C CNN
F 2 "" H 2100 5150 50  0001 C CNN
F 3 "" H 2100 5150 50  0001 C CNN
	1    2100 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 615E2266
P 2700 5000
AR Path="/615E2266" Ref="R?"  Part="1" 
AR Path="/5F606192/615E2266" Ref="R18"  Part="1" 
F 0 "R18" H 2759 5046 50  0000 L CNN
F 1 "10k" H 2759 4955 50  0000 L CNN
F 2 "" H 2700 5000 50  0001 C CNN
F 3 "~" H 2700 5000 50  0001 C CNN
	1    2700 5000
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 615E226C
P 2400 5000
AR Path="/615E226C" Ref="R?"  Part="1" 
AR Path="/5F606192/615E226C" Ref="R17"  Part="1" 
F 0 "R17" H 2459 5046 50  0000 L CNN
F 1 "10k" H 2459 4955 50  0000 L CNN
F 2 "" H 2400 5000 50  0001 C CNN
F 3 "~" H 2400 5000 50  0001 C CNN
	1    2400 5000
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR039
U 1 1 6161528B
P 3000 4850
F 0 "#PWR039" H 3000 4700 50  0001 C CNN
F 1 "+3.3V" H 3015 5023 50  0000 C CNN
F 2 "" H 3000 4850 50  0001 C CNN
F 3 "" H 3000 4850 50  0001 C CNN
	1    3000 4850
	1    0    0    -1  
$EndComp
Text GLabel 2400 4850 1    50   Input ~ 0
DAC_MODE1
Wire Wire Line
	2400 4850 2400 4900
Text GLabel 2700 5150 3    50   Input ~ 0
DAC_MODE2
Text GLabel 3000 5150 3    50   Input ~ 0
DAC_XSMT
Text GLabel 1800 4850 1    50   Input ~ 0
DAC_ADR1
Text GLabel 2100 4850 1    50   Input ~ 0
DAC_ADR2
$Comp
L Device:R_Small R?
U 1 1 61625075
P 1800 5000
AR Path="/61625075" Ref="R?"  Part="1" 
AR Path="/5F606192/61625075" Ref="R15"  Part="1" 
F 0 "R15" H 1859 5046 50  0000 L CNN
F 1 "10k" H 1859 4955 50  0000 L CNN
F 2 "" H 1800 5000 50  0001 C CNN
F 3 "~" H 1800 5000 50  0001 C CNN
	1    1800 5000
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 616254E9
P 2100 5000
AR Path="/616254E9" Ref="R?"  Part="1" 
AR Path="/5F606192/616254E9" Ref="R16"  Part="1" 
F 0 "R16" H 2159 5046 50  0000 L CNN
F 1 "10k" H 2159 4955 50  0000 L CNN
F 2 "" H 2100 5000 50  0001 C CNN
F 3 "~" H 2100 5000 50  0001 C CNN
	1    2100 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61626170
P 1800 5150
AR Path="/61626170" Ref="#PWR?"  Part="1" 
AR Path="/5F606192/61626170" Ref="#PWR032"  Part="1" 
F 0 "#PWR032" H 1800 4900 50  0001 C CNN
F 1 "GND" H 1805 4977 50  0000 C CNN
F 2 "" H 1800 5150 50  0001 C CNN
F 3 "" H 1800 5150 50  0001 C CNN
	1    1800 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 6162DB6E
P 3000 5000
AR Path="/6162DB6E" Ref="R?"  Part="1" 
AR Path="/5F606192/6162DB6E" Ref="R19"  Part="1" 
F 0 "R19" H 3059 5046 50  0000 L CNN
F 1 "10k" H 3059 4955 50  0000 L CNN
F 2 "" H 3000 5000 50  0001 C CNN
F 3 "~" H 3000 5000 50  0001 C CNN
	1    3000 5000
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR037
U 1 1 61634E1B
P 2700 4850
F 0 "#PWR037" H 2700 4700 50  0001 C CNN
F 1 "+3.3V" H 2715 5023 50  0000 C CNN
F 2 "" H 2700 4850 50  0001 C CNN
F 3 "" H 2700 4850 50  0001 C CNN
	1    2700 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 4850 2700 4900
Wire Wire Line
	2400 5100 2400 5150
Wire Wire Line
	1800 5100 1800 5150
Wire Wire Line
	2700 5100 2700 5150
Wire Wire Line
	2100 5100 2100 5150
Wire Wire Line
	3000 5100 3000 5150
Wire Wire Line
	3000 4900 3000 4850
Wire Wire Line
	2100 4900 2100 4850
Wire Wire Line
	1800 4900 1800 4850
Text GLabel 1700 3150 0    50   Input ~ 0
DAC_ADR1
Text GLabel 1700 3250 0    50   Input ~ 0
DAC_ADR2
Text GLabel 1700 3450 0    50   Input ~ 0
DAC_MODE1
Text GLabel 1700 3550 0    50   Input ~ 0
DAC_MODE2
Text GLabel 1700 3750 0    50   Input ~ 0
DAC_XSMT
Wire Notes Line
	4150 5800 850  5800
Wire Notes Line
	850  900  850  5800
Wire Notes Line
	4150 900  4150 5800
$Comp
L Connector:TestPoint_Small TP1
U 1 1 616E87F3
P 5100 6800
F 0 "TP1" H 5148 6846 50  0000 L CNN
F 1 "SCK" H 5148 6755 50  0000 L CNN
F 2 "" H 5300 6800 50  0001 C CNN
F 3 "~" H 5300 6800 50  0001 C CNN
	1    5100 6800
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Small TP2
U 1 1 6170D9F0
P 5100 7050
F 0 "TP2" H 5148 7096 50  0000 L CNN
F 1 "BCK" H 5148 7005 50  0000 L CNN
F 2 "" H 5300 7050 50  0001 C CNN
F 3 "~" H 5300 7050 50  0001 C CNN
	1    5100 7050
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Small TP3
U 1 1 6170DD27
P 5100 7300
F 0 "TP3" H 5148 7346 50  0000 L CNN
F 1 "LRCK" H 5148 7255 50  0000 L CNN
F 2 "" H 5300 7300 50  0001 C CNN
F 3 "~" H 5300 7300 50  0001 C CNN
	1    5100 7300
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Small TP4
U 1 1 6170EA7C
P 5100 7550
F 0 "TP4" H 5148 7596 50  0000 L CNN
F 1 "DIN" H 5148 7505 50  0000 L CNN
F 2 "" H 5300 7550 50  0001 C CNN
F 3 "~" H 5300 7550 50  0001 C CNN
	1    5100 7550
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Small TP8
U 1 1 6170ED54
P 6250 7550
F 0 "TP8" H 6298 7596 50  0000 L CNN
F 1 "SDA" H 6298 7505 50  0000 L CNN
F 2 "" H 6450 7550 50  0001 C CNN
F 3 "~" H 6450 7550 50  0001 C CNN
	1    6250 7550
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Small TP7
U 1 1 6170F28C
P 6250 7300
F 0 "TP7" H 6298 7346 50  0000 L CNN
F 1 "SCL" H 6298 7255 50  0000 L CNN
F 2 "" H 6450 7300 50  0001 C CNN
F 3 "~" H 6450 7300 50  0001 C CNN
	1    6250 7300
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Small TP6
U 1 1 6170F58A
P 6250 7050
F 0 "TP6" H 6298 7096 50  0000 L CNN
F 1 "OUTR" H 6298 7005 50  0000 L CNN
F 2 "" H 6450 7050 50  0001 C CNN
F 3 "~" H 6450 7050 50  0001 C CNN
	1    6250 7050
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Small TP5
U 1 1 6170F745
P 6250 6800
F 0 "TP5" H 6298 6846 50  0000 L CNN
F 1 "OUTL" H 6298 6755 50  0000 L CNN
F 2 "" H 6450 6800 50  0001 C CNN
F 3 "~" H 6450 6800 50  0001 C CNN
	1    6250 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 6800 5000 6800
Wire Wire Line
	5100 7050 5000 7050
Wire Wire Line
	5000 7300 5100 7300
Wire Wire Line
	5000 7550 5100 7550
Wire Wire Line
	6250 6800 6150 6800
Wire Wire Line
	6250 7050 6150 7050
Wire Wire Line
	6250 7300 6150 7300
Wire Wire Line
	6250 7550 6150 7550
Wire Notes Line
	4450 6650 4450 7700
Wire Notes Line
	4450 7700 6700 7700
Wire Notes Line
	6700 7700 6700 6650
Wire Notes Line
	4450 6650 6700 6650
Text Notes 4450 6600 0    118  ~ 0
TEST POINTS
$EndSCHEMATC
