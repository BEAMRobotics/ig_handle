EESchema Schematic File Version 4
EELAYER 30 0
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
L teensy:Teensy3.6 U?
U 1 1 5FFCE603
P 4000 3300
F 0 "U?" H 4000 5737 60  0000 C CNN
F 1 "Teensy3.6" H 4000 5631 60  0000 C CNN
F 2 "" H 4000 3350 60  0000 C CNN
F 3 "" H 4000 3350 60  0000 C CNN
	1    4000 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 1750 2350 1750
Text GLabel 2350 1750 0    50   Output ~ 0
PPS
Text Notes 800  1950 0    50   ~ 0
FrequencyTimer2 uses pin 5\nand can output a very low jitter \nsignal based on the PWM Timer
Text GLabel 2350 1350 0    50   Output ~ 0
$GPRMC
Wire Wire Line
	2350 1350 2850 1350
Text GLabel 2350 1550 0    50   Output ~ 0
Cam_Trigger
Wire Wire Line
	2350 1550 2850 1550
Text Notes 650  1650 0    50   ~ 0
Pins 3 and 4 are on PWM\nTimer 1 (FTM1) so changes\nto their timer affect both pins
$EndSCHEMATC
