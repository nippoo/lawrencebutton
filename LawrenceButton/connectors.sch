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
F 2 "" H 3250 3750 50  0001 C CNN
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
L Connector_Generic:Conn_02x10_Odd_Even J?
U 1 1 6120CB49
P 4700 3700
AR Path="/6120CB49" Ref="J?"  Part="1" 
AR Path="/611FFD37/6120CB49" Ref="J7"  Part="1" 
F 0 "J7" H 4750 4317 50  0000 C CNN
F 1 "Conn_02x10_Odd_Even" H 4750 4226 50  0000 C CNN
F 2 "" H 4700 3700 50  0001 C CNN
F 3 "~" H 4700 3700 50  0001 C CNN
	1    4700 3700
	1    0    0    -1  
$EndComp
Text GLabel 4500 3400 0    50   Input ~ 0
LCD_A
Text GLabel 4500 3500 0    50   Input ~ 0
LCD_C
Text GLabel 4500 3600 0    50   Input ~ 0
LCD_E
Text GLabel 4500 3700 0    50   Input ~ 0
LCD_G
Text GLabel 5000 3400 2    50   Input ~ 0
LCD_B
Text GLabel 5000 3500 2    50   Input ~ 0
LCD_D
Text GLabel 5000 3600 2    50   Input ~ 0
LCD_F
Text GLabel 5000 3700 2    50   Input ~ 0
LCD_DP
Text GLabel 4500 3800 0    50   Input ~ 0
LCD_DIG1CC
Text GLabel 5000 3800 2    50   Input ~ 0
LCD_DIG2CC
$Comp
L power:+3.3V #PWR?
U 1 1 6120CB59
P 5000 3300
AR Path="/6120CB59" Ref="#PWR?"  Part="1" 
AR Path="/611FFD37/6120CB59" Ref="#PWR073"  Part="1" 
F 0 "#PWR073" H 5000 3150 50  0001 C CNN
F 1 "+3.3V" V 5000 3550 50  0000 C CNN
F 2 "" H 5000 3300 50  0001 C CNN
F 3 "" H 5000 3300 50  0001 C CNN
	1    5000 3300
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6120CB5F
P 4500 3300
AR Path="/6120CB5F" Ref="#PWR?"  Part="1" 
AR Path="/611FFD37/6120CB5F" Ref="#PWR072"  Part="1" 
F 0 "#PWR072" H 4500 3050 50  0001 C CNN
F 1 "GND" V 4500 3100 50  0000 C CNN
F 2 "" H 4500 3300 50  0001 C CNN
F 3 "" H 4500 3300 50  0001 C CNN
	1    4500 3300
	0    1    1    0   
$EndComp
Text GLabel 4500 3900 0    50   Input ~ 0
B_PREV
Text GLabel 4500 4000 0    50   Input ~ 0
B_STOP
Text GLabel 5000 3900 2    50   Input ~ 0
B_NEXT
Text GLabel 5000 4200 2    50   Input ~ 0
NRST
Text GLabel 5000 4100 2    50   Input ~ 0
GPIO11
Text GLabel 5000 4000 2    50   Input ~ 0
GPIO9
Text GLabel 4500 4100 0    50   Input ~ 0
GPIO10
Text GLabel 4500 4200 0    50   Input ~ 0
GPIO12
$Comp
L Connector_Generic:Conn_01x02 J?
U 1 1 612918C1
P 2250 3750
AR Path="/612918C1" Ref="J?"  Part="1" 
AR Path="/611FFD37/612918C1" Ref="J5"  Part="1" 
F 0 "J5" H 2330 3742 50  0000 L CNN
F 1 "POWER" H 2330 3651 50  0000 L CNN
F 2 "" H 2250 3750 50  0001 C CNN
F 3 "~" H 2250 3750 50  0001 C CNN
	1    2250 3750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 612918CD
P 1950 3900
AR Path="/612918CD" Ref="#PWR?"  Part="1" 
AR Path="/611FFD37/612918CD" Ref="#PWR069"  Part="1" 
F 0 "#PWR069" H 1950 3650 50  0001 C CNN
F 1 "GND" H 1955 3727 50  0000 C CNN
F 2 "" H 1950 3900 50  0001 C CNN
F 3 "" H 1950 3900 50  0001 C CNN
	1    1950 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 3850 1950 3850
Wire Wire Line
	1950 3850 1950 3900
Wire Wire Line
	2050 3750 1950 3750
Wire Wire Line
	1950 3750 1950 3700
$Comp
L power:VDD #PWR068
U 1 1 614A0ECD
P 1950 3700
F 0 "#PWR068" H 1950 3550 50  0001 C CNN
F 1 "VDD" H 1965 3873 50  0000 C CNN
F 2 "" H 1950 3700 50  0001 C CNN
F 3 "" H 1950 3700 50  0001 C CNN
	1    1950 3700
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 6120CB8C
P 8000 3650
AR Path="/6120CB8C" Ref="#PWR?"  Part="1" 
AR Path="/611FFD37/6120CB8C" Ref="#PWR076"  Part="1" 
F 0 "#PWR076" H 8000 3500 50  0001 C CNN
F 1 "+5V" H 8015 3823 50  0000 C CNN
F 2 "" H 8000 3650 50  0001 C CNN
F 3 "" H 8000 3650 50  0001 C CNN
	1    8000 3650
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J?
U 1 1 6120CBA2
P 8300 3800
AR Path="/6120CBA2" Ref="J?"  Part="1" 
AR Path="/611FFD37/6120CBA2" Ref="J9"  Part="1" 
F 0 "J9" H 8380 3842 50  0000 L CNN
F 1 "PHOTOSENSOR" H 8380 3751 50  0000 L CNN
F 2 "" H 8300 3800 50  0001 C CNN
F 3 "~" H 8300 3800 50  0001 C CNN
	1    8300 3800
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 6120CB9C
P 7800 3800
AR Path="/6120CB9C" Ref="R?"  Part="1" 
AR Path="/611FFD37/6120CB9C" Ref="R20"  Part="1" 
F 0 "R20" H 7859 3846 50  0000 L CNN
F 1 "1k" H 7859 3755 50  0000 L CNN
F 2 "" H 7800 3800 50  0001 C CNN
F 3 "~" H 7800 3800 50  0001 C CNN
	1    7800 3800
	0    1    1    0   
$EndComp
Text GLabel 7600 3800 0    50   Input ~ 0
PHOTOSENSOR
Wire Wire Line
	7600 3800 7700 3800
$Comp
L power:GND #PWR?
U 1 1 6120CB94
P 8000 3950
AR Path="/6120CB94" Ref="#PWR?"  Part="1" 
AR Path="/611FFD37/6120CB94" Ref="#PWR077"  Part="1" 
F 0 "#PWR077" H 8000 3700 50  0001 C CNN
F 1 "GND" H 8005 3777 50  0000 C CNN
F 2 "" H 8000 3950 50  0001 C CNN
F 3 "" H 8000 3950 50  0001 C CNN
	1    8000 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 3950 8000 3900
Wire Wire Line
	8000 3900 8100 3900
Wire Wire Line
	8100 3700 8000 3700
Wire Wire Line
	8000 3700 8000 3650
Wire Wire Line
	7900 3800 8100 3800
$Comp
L Connector_Generic:Conn_01x08 J?
U 1 1 6120CB83
P 9750 3700
AR Path="/6120CB83" Ref="J?"  Part="1" 
AR Path="/611FFD37/6120CB83" Ref="J10"  Part="1" 
F 0 "J10" H 9830 3692 50  0000 L CNN
F 1 "GPIO" H 9830 3601 50  0000 L CNN
F 2 "" H 9750 3700 50  0001 C CNN
F 3 "~" H 9750 3700 50  0001 C CNN
	1    9750 3700
	1    0    0    -1  
$EndComp
Text GLabel 9550 4000 0    50   Input ~ 0
GPIO7
Text GLabel 9550 3900 0    50   Input ~ 0
GPIO6
Text GLabel 9550 4100 0    50   Input ~ 0
GPIO8
Text GLabel 9550 3800 0    50   Input ~ 0
GPIO5
Text GLabel 9550 3400 0    50   Input ~ 0
GPIO1
Text GLabel 9550 3700 0    50   Input ~ 0
GPIO4
Text GLabel 9550 3500 0    50   Input ~ 0
GPIO2
Text GLabel 9550 3600 0    50   Input ~ 0
GPIO3
Text GLabel 6300 3800 0    50   Input ~ 0
WS2812_5V
$Comp
L power:VDD #PWR074
U 1 1 614A1594
P 6200 3600
F 0 "#PWR074" H 6200 3450 50  0001 C CNN
F 1 "VDD" H 6215 3773 50  0000 C CNN
F 2 "" H 6200 3600 50  0001 C CNN
F 3 "" H 6200 3600 50  0001 C CNN
	1    6200 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 3700 6200 3600
Wire Wire Line
	6300 3700 6200 3700
Wire Wire Line
	6200 3900 6300 3900
Wire Wire Line
	6200 3950 6200 3900
$Comp
L power:GND #PWR?
U 1 1 6120CB2A
P 6200 3950
AR Path="/6120CB2A" Ref="#PWR?"  Part="1" 
AR Path="/611FFD37/6120CB2A" Ref="#PWR075"  Part="1" 
F 0 "#PWR075" H 6200 3700 50  0001 C CNN
F 1 "GND" H 6205 3777 50  0000 C CNN
F 2 "" H 6200 3950 50  0001 C CNN
F 3 "" H 6200 3950 50  0001 C CNN
	1    6200 3950
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J?
U 1 1 6120CB24
P 6500 3800
AR Path="/6120CB24" Ref="J?"  Part="1" 
AR Path="/611FFD37/6120CB24" Ref="J8"  Part="1" 
F 0 "J8" H 6580 3842 50  0000 L CNN
F 1 "LEDs" H 6580 3751 50  0000 L CNN
F 2 "" H 6500 3800 50  0001 C CNN
F 3 "~" H 6500 3800 50  0001 C CNN
	1    6500 3800
	1    0    0    -1  
$EndComp
$EndSCHEMATC
