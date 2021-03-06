EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 3
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
L Connector_Generic:Conn_02x05_Odd_Even J?
U 1 1 6120CAED
P 3250 3750
AR Path="/6120CAED" Ref="J?"  Part="1" 
AR Path="/611FFD37/6120CAED" Ref="J6"  Part="1" 
F 0 "J6" H 3300 4167 50  0000 C CNN
F 1 "SWD" H 3300 4076 50  0000 C CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_2x05_P1.27mm_Vertical_SMD" H 3250 3750 50  0001 C CNN
F 3 "~" H 3250 3750 50  0001 C CNN
	1    3250 3750
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 6120CAF3
P 3050 3550
AR Path="/6120CAF3" Ref="#PWR?"  Part="1" 
AR Path="/611FFD37/6120CAF3" Ref="#PWR071"  Part="1" 
F 0 "#PWR071" H 3050 3400 50  0001 C CNN
F 1 "+3.3V" V 3050 3800 50  0000 C CNN
F 2 "" H 3050 3550 50  0001 C CNN
F 3 "" H 3050 3550 50  0001 C CNN
	1    3050 3550
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6120CAF9
P 2950 4050
AR Path="/6120CAF9" Ref="#PWR?"  Part="1" 
AR Path="/611FFD37/6120CAF9" Ref="#PWR070"  Part="1" 
F 0 "#PWR070" H 2950 3800 50  0001 C CNN
F 1 "GND" H 2955 3877 50  0000 C CNN
F 2 "" H 2950 4050 50  0001 C CNN
F 3 "" H 2950 4050 50  0001 C CNN
	1    2950 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 3650 2950 3650
Wire Wire Line
	2950 3650 2950 3750
Wire Wire Line
	3050 3750 2950 3750
Connection ~ 2950 3750
Wire Wire Line
	2950 3750 2950 3950
Wire Wire Line
	3050 3950 2950 3950
Connection ~ 2950 3950
Wire Wire Line
	2950 3950 2950 4050
Text GLabel 3550 3550 2    50   Input ~ 0
SWDIO
Text GLabel 3550 3750 2    50   Input ~ 0
SWO
Text GLabel 3550 3650 2    50   Input ~ 0
SWCLK
Text GLabel 3550 3950 2    50   Input ~ 0
NRST
NoConn ~ 3550 3850
NoConn ~ 3050 3850
$Comp
L power:+5V #PWR?
U 1 1 6120CB8C
P 7800 3550
AR Path="/6120CB8C" Ref="#PWR?"  Part="1" 
AR Path="/611FFD37/6120CB8C" Ref="#PWR076"  Part="1" 
F 0 "#PWR076" H 7800 3400 50  0001 C CNN
F 1 "+5V" H 7815 3723 50  0000 C CNN
F 2 "" H 7800 3550 50  0001 C CNN
F 3 "" H 7800 3550 50  0001 C CNN
	1    7800 3550
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 6120CB9C
P 7600 3700
AR Path="/6120CB9C" Ref="R?"  Part="1" 
AR Path="/611FFD37/6120CB9C" Ref="R20"  Part="1" 
F 0 "R20" H 7659 3746 50  0000 L CNN
F 1 "1k" H 7659 3655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 7600 3700 50  0001 C CNN
F 3 "~" H 7600 3700 50  0001 C CNN
F 4 "C21190" H 7600 3700 50  0001 C CNN "LCSC"
	1    7600 3700
	0    1    1    0   
$EndComp
Text GLabel 7400 3700 0    50   Input ~ 0
PHOTOSENSOR
Wire Wire Line
	7400 3700 7500 3700
$Comp
L power:GND #PWR?
U 1 1 6120CB94
P 7800 3950
AR Path="/6120CB94" Ref="#PWR?"  Part="1" 
AR Path="/611FFD37/6120CB94" Ref="#PWR077"  Part="1" 
F 0 "#PWR077" H 7800 3700 50  0001 C CNN
F 1 "GND" H 7805 3777 50  0000 C CNN
F 2 "" H 7800 3950 50  0001 C CNN
F 3 "" H 7800 3950 50  0001 C CNN
	1    7800 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 3600 7800 3600
Wire Wire Line
	7800 3600 7800 3550
Wire Wire Line
	7700 3700 7900 3700
Text GLabel 9600 3900 0    50   Input ~ 0
GPIO7
Text GLabel 10100 3800 2    50   Input ~ 0
GPIO6
Text GLabel 10100 3900 2    50   Input ~ 0
GPIO8
Text GLabel 9600 3800 0    50   Input ~ 0
GPIO5
Text GLabel 9600 3600 0    50   Input ~ 0
GPIO1
Text GLabel 10100 3700 2    50   Input ~ 0
GPIO4
Text GLabel 10100 3600 2    50   Input ~ 0
GPIO2
Text GLabel 9600 3700 0    50   Input ~ 0
GPIO3
Text GLabel 6000 3750 0    50   Input ~ 0
WS2812_5V
Wire Wire Line
	5900 3650 5900 3550
Wire Wire Line
	6000 3650 5900 3650
Wire Wire Line
	5900 3850 6000 3850
Wire Wire Line
	5900 3900 5900 3850
$Comp
L power:GND #PWR?
U 1 1 6120CB2A
P 5900 3900
AR Path="/6120CB2A" Ref="#PWR?"  Part="1" 
AR Path="/611FFD37/6120CB2A" Ref="#PWR075"  Part="1" 
F 0 "#PWR075" H 5900 3650 50  0001 C CNN
F 1 "GND" H 5905 3727 50  0000 C CNN
F 2 "" H 5900 3900 50  0001 C CNN
F 3 "" H 5900 3900 50  0001 C CNN
	1    5900 3900
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J?
U 1 1 6120CB24
P 6200 3750
AR Path="/6120CB24" Ref="J?"  Part="1" 
AR Path="/611FFD37/6120CB24" Ref="J8"  Part="1" 
F 0 "J8" H 6280 3792 50  0000 L CNN
F 1 "LEDs" H 6280 3701 50  0000 L CNN
F 2 "Connector_Molex:Molex_PicoBlade_53048-0310_1x03_P1.25mm_Horizontal" H 6200 3750 50  0001 C CNN
F 3 "~" H 6200 3750 50  0001 C CNN
	1    6200 3750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x15 J7
U 1 1 61C1D0E6
P 4850 3800
F 0 "J7" H 4930 3842 50  0000 L CNN
F 1 "DB" H 4930 3751 50  0000 L CNN
F 2 "Connector_Molex:Molex_PicoBlade_53048-1510_1x15_P1.25mm_Horizontal" H 4850 3800 50  0001 C CNN
F 3 "~" H 4850 3800 50  0001 C CNN
	1    4850 3800
	1    0    0    -1  
$EndComp
Text GLabel 4650 3100 0    50   Input ~ 0
LCD_A
Text GLabel 4650 3300 0    50   Input ~ 0
LCD_C
Text GLabel 4650 3500 0    50   Input ~ 0
LCD_E
Text GLabel 4650 3700 0    50   Input ~ 0
LCD_G
Text GLabel 4650 3900 0    50   Input ~ 0
LCD_DIG1CC
Text GLabel 4650 4100 0    50   Input ~ 0
B_PREV
Text GLabel 4650 4300 0    50   Input ~ 0
B_STOP
Text GLabel 4650 3200 0    50   Input ~ 0
LCD_B
Text GLabel 4650 3400 0    50   Input ~ 0
LCD_D
Text GLabel 4650 3600 0    50   Input ~ 0
LCD_F
Text GLabel 4650 3800 0    50   Input ~ 0
LCD_DP
Text GLabel 4650 4000 0    50   Input ~ 0
LCD_DIG2CC
Text GLabel 4650 4200 0    50   Input ~ 0
B_NEXT
Text GLabel 4650 4400 0    50   Input ~ 0
NRST
$Comp
L power:GND #PWR?
U 1 1 61C210A1
P 4500 4550
AR Path="/61C210A1" Ref="#PWR?"  Part="1" 
AR Path="/611FFD37/61C210A1" Ref="#PWR072"  Part="1" 
F 0 "#PWR072" H 4500 4300 50  0001 C CNN
F 1 "GND" H 4505 4377 50  0000 C CNN
F 2 "" H 4500 4550 50  0001 C CNN
F 3 "" H 4500 4550 50  0001 C CNN
	1    4500 4550
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x06_Odd_Even J12
U 1 1 61C40614
P 9800 3800
F 0 "J12" H 9850 4217 50  0000 C CNN
F 1 "GPIO" H 9850 4126 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x06_P2.54mm_Vertical" H 9800 3800 50  0001 C CNN
F 3 "~" H 9800 3800 50  0001 C CNN
	1    9800 3800
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 61C43C50
P 10100 4000
AR Path="/61C43C50" Ref="#PWR?"  Part="1" 
AR Path="/611FFD37/61C43C50" Ref="#PWR0101"  Part="1" 
F 0 "#PWR0101" H 10100 3850 50  0001 C CNN
F 1 "+3.3V" V 10100 4250 50  0000 C CNN
F 2 "" H 10100 4000 50  0001 C CNN
F 3 "" H 10100 4000 50  0001 C CNN
	1    10100 4000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61C443E2
P 9500 4150
AR Path="/61C443E2" Ref="#PWR?"  Part="1" 
AR Path="/611FFD37/61C443E2" Ref="#PWR0102"  Part="1" 
F 0 "#PWR0102" H 9500 3900 50  0001 C CNN
F 1 "GND" H 9505 3977 50  0000 C CNN
F 2 "" H 9500 4150 50  0001 C CNN
F 3 "" H 9500 4150 50  0001 C CNN
	1    9500 4150
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 61DFBE86
P 9600 4000
AR Path="/61DFBE86" Ref="#PWR?"  Part="1" 
AR Path="/611FFD37/61DFBE86" Ref="#PWR0103"  Part="1" 
F 0 "#PWR0103" H 9600 3850 50  0001 C CNN
F 1 "+5V" V 9615 4128 50  0000 L CNN
F 2 "" H 9600 4000 50  0001 C CNN
F 3 "" H 9600 4000 50  0001 C CNN
	1    9600 4000
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 61DFC804
P 5900 3550
AR Path="/61DFC804" Ref="#PWR?"  Part="1" 
AR Path="/611FFD37/61DFC804" Ref="#PWR0104"  Part="1" 
F 0 "#PWR0104" H 5900 3400 50  0001 C CNN
F 1 "+5V" H 5915 3723 50  0000 C CNN
F 2 "" H 5900 3550 50  0001 C CNN
F 3 "" H 5900 3550 50  0001 C CNN
	1    5900 3550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61F64F75
P 10200 4150
AR Path="/61F64F75" Ref="#PWR?"  Part="1" 
AR Path="/611FFD37/61F64F75" Ref="#PWR0105"  Part="1" 
F 0 "#PWR0105" H 10200 3900 50  0001 C CNN
F 1 "GND" H 10205 3977 50  0000 C CNN
F 2 "" H 10200 4150 50  0001 C CNN
F 3 "" H 10200 4150 50  0001 C CNN
	1    10200 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 4150 10200 4100
Wire Wire Line
	10200 4100 10100 4100
Wire Wire Line
	9600 4100 9500 4100
Wire Wire Line
	9500 4100 9500 4150
$Comp
L Connector_Generic:Conn_01x04 J9
U 1 1 61FCE5F5
P 8100 3700
F 0 "J9" H 8180 3692 50  0000 L CNN
F 1 "PHOTOSENSOR" H 8180 3601 50  0000 L CNN
F 2 "Connector_Molex:Molex_PicoBlade_53048-0410_1x04_P1.25mm_Horizontal" H 8100 3700 50  0001 C CNN
F 3 "~" H 8100 3700 50  0001 C CNN
	1    8100 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 3900 7800 3900
Wire Wire Line
	7800 3900 7800 3950
NoConn ~ 7900 3800
Wire Wire Line
	4650 4500 4500 4500
Wire Wire Line
	4500 4500 4500 4550
$EndSCHEMATC
