# Elektronik-Hardware: vollständige logische Bauteil-Netzliste

Diese gezielt durchsuchbare Momentaufnahme ergänzt [Einstieg](hardware-overview.md) und [technische Referenz](hardware-reference.md). Sie wurde aus den `PAD_NET`-Einträgen der beiden `.epcb` im [EasyEDA-Projekt](../hardware/pcb/ProPrj_NavBot-ES02.epro) abgeleitet. Angegeben sind **Bauteil-Designator, Bauteilwert/Typ, Padnummer und Netz**; wo vorhanden steht der Schaltplansymbol-Pinname in eckigen Klammern. Suche mit `rg "U6|\$1N13944|VIN" agent_notes/hardware-netlist.md`.

`$1N…` bezeichnet ein unbenanntes, aber elektrisch identisches Netz innerhalb **derselben** Platine. `∅` bezeichnet ein im PCB-Netzlistenexport unverbundenes nummeriertes Pad. Mechanische Board-Geometrie ist ausgelassen; bei mehrfachen Gehäusepads auf einem Netz wird jede eindeutige Padnummer einmal genannt. Die Netzliste beschreibt die **geplante logische Verbindung**, nicht Leiterbahnführung, Schicht, Massefläche, Bauteilplatzierung, Bestückung oder gemessene Funktion. Bei Layout-, SI-/EMV- und Reparaturfragen bleibt der PCB-Editor bzw. die reale Platine maßgeblich.

## MAIN (116 Bauteile)

| Ref | Wert / Typ | Pads → Netze |
| --- | --- | --- |
| `C1` | 100nF | `1=GND, 2=ADC` |
| `C2` | 22uF | `1=GND, 2=VIN` |
| `C3` | 22uF | `1=GND, 2=VIN` |
| `C4` | 100nF | `1=GND, 2=VIN` |
| `C5` | 100nF | `1=$1N2389, 2=$1N2414` |
| `C6` | 22uF | `1=GND, 2=+5V` |
| `C7` | 22uF | `1=GND, 2=+5V` |
| `C8` | 100nF | `1=GND, 2=+5V` |
| `C9` | 10uF | `1=GND, 2=+5V` |
| `C10` | 100nF | `1=GND, 2=+5V` |
| `C11` | 10uF | `1=+3.3V, 2=GND` |
| `C12` | 100nF | `1=+3.3V, 2=GND` |
| `C13` | 1uF | `1=GND, 2=+3.3V` |
| `C14` | 1uF | `1=GND, 2=$1N3674` |
| `C15` | 22uF | `1=+3.3V, 2=GND` |
| `C16` | 100nF | `1=GND, 2=$1N4673` |
| `C17` | 100nF | `1=GND, 2=+3.3V` |
| `C18` | 100nF | `1=$1N10925, 2=U_A` |
| `C19` | 100nF | `1=$1N11051, 2=V_A` |
| `C20` | 100nF | `1=W_A, 2=$1N11092` |
| `C21` | 10uF | `1=$1N11138, 2=GND` |
| `C22` | 10uF | `1=$1N11195, 2=GND` |
| `C23` | 100nF | `1=GND, 2=VIN` |
| `C24` | 100uF | `1=GND, 2=VIN` |
| `C25` | 100nF | `1=$1N14887, 2=U_B` |
| `C26` | 100nF | `1=$1N14892, 2=V_B` |
| `C27` | 100nF | `1=W_B, 2=$1N14893` |
| `C28` | 10uF | `1=$1N14895, 2=GND` |
| `C29` | 10uF | `1=$1N14897, 2=GND` |
| `C30` | 100nF | `1=GND, 2=VIN` |
| `C31` | 100uF | `1=GND, 2=VIN` |
| `C32` | 10uF | `1=GND, 2=+3.3V` |
| `C33` | 100nF | `1=GND, 2=+3.3V` |
| `C34` | 10nF | `1=B-, 2=$1N46033` |
| `C35` | 10nF | `1=B-, 2=$1N46092` |
| `C36` | 100nF | `1=VSTA, 2=GND` |
| `C37` | 100nF | `1=B-, 2=GND` |
| `C38` | 22uF | `1=GND, 2=VIN` |
| `C39` | 22uF | `1=GND, 2=VIN` |
| `CN2` | HX25003-3WAP | `1=B2, 2=B1, 3=B-` |
| `CN3` | WAFER-SH1.0-5PLB | `1=SERVO1, 2=SERVO2, 3=SERVO3, 4=SERVO4, 5=GND, 6=GND, 7=GND` |
| `CN4` | WAFER-SH1.0-5PLB | `1=GND, 2=+5V, 3=GND, 4=RX1, 5=TX1, 6=GND, 7=GND` |
| `D2` | SD103AWS_C181207 | `1[C]=$1N13984, 2[A]=$1N14018` |
| `D3` | SD103AWS_C181207 | `1[C]=$1N14914, 2[A]=$1N14916` |
| `D4` | SS54_C7420369 | `1[A]=GND, 2[K]=$1N2414` |
| `D5` | SS54_C7420369 | `1[A]=VBUS, 2[K]=$1N30381` |
| `H1` | WAFER-SH1.0-4PLB | `1=+5V, 2=GND, 3=RX0, 4=TX0, 5=GND, 6=GND` |
| `H2` | WAFER-SH1.0-4PLB | `1=+5V, 2=GND, 3=RX1, 4=TX1, 5=GND, 6=GND` |
| `H3` | WAFER-SH1.0-4PLB | `1=+5V, 2=GND, 3=RX2, 4=TX2, 5=GND, 6=GND` |
| `H4` | PZ254R-11-03P | `1=GND, 2=+5V, 3=$1N8581` |
| `H5` | 1.0-4PWB | `1=SDA2_A, 2=SCL2_A, 3=GND, 4=+3.3V, 5=GND, 6=GND` |
| `H6` | 1.0-4PWB | `1=SDA2_B, 2=SCL2_B, 3=GND, 4=+3.3V, 5=GND, 6=GND` |
| `H7` | PZ254R-11-03P | `1=U_A, 2=V_A, 3=W_A` |
| `H8` | PZ254R-11-03P | `1=U_B, 2=V_B, 3=W_B` |
| `H9` | PH2.54-07-03PWS | `1=SERVO1, 2=SERVO2, 3=$1N10550, 4=$1N10550, 5=GND, 6=GND` |
| `H10` | PH2.54-07-03PWS | `1=SERVO3, 2=SERVO4, 3=$1N10550, 4=$1N10550, 5=GND, 6=GND` |
| `H11` | PZ254R-11-03P | `1=+5V, 2=$1N10550, 3=VIN` |
| `H12` | HDR-F_2.54_1x8P | `1=+5V, 2=GND, 3=SCLK, 4=MOSI, 5=MISO, 6=CS1, 8=GND, 7=∅` |
| `J1` | 0805短接点 | `1=+5V, 2=$1N30381` |
| `L1` | 10uH | `1=$1N2414, 2=+5V` |
| `LED1` | KT-0603R | `1[K]=GND, 2[A]=$1N3996` |
| `LED2` | LED_0603-B | `1[C]=$1N5206, 2[A]=+3.3V` |
| `LED3` | LED_0603-B | `1[C]=$1N18308, 2[A]=+3.3V` |
| `M1` | 2S_RECHARGE | `1=VBUS, 2=GND, 3=VSTA, 4=GND, 5=B1, 6=B2` |
| `Q1` | SS8050(RANGE:200-350)_C2150 | `1[B]=$1N8578, 2[E]=GND, 3[C]=RX2` |
| `Q2` | WSD3056DN33 | `1[S1]=GND, 2[G1]=$1N42523, 3[S2]=B-, 4[G2]=$1N42515, 5[D2]=$1N31293, 6[D2]=$1N31293, 7[D1]=$1N31293, 8[D1]=$1N31293, 9[D1]=$1N31293, 10[D2]=$1N31293` |
| `R1` | 10kΩ | `1=ADC, 2=VIN` |
| `R2` | 1kΩ | `1=GND, 2=ADC` |
| `R3` | 10kΩ | `1=$1N2260, 2=+5V` |
| `R5` | 4.7kΩ | `1=GND, 2=$1N3025` |
| `R6` | 4.7kΩ | `1=GND, 2=$1N3041` |
| `R7` | 1kΩ | `1=TX0, 2=$1N3733` |
| `R8` | 1kΩ | `1=RX0, 2=$1N3813` |
| `R9` | 3.3kΩ | `1=LED2, 2=$1N5206` |
| `R11` | 3.3kΩ | `1=$1N3996, 2=+3.3V` |
| `R12` | 10kΩ | `1=$1N8581, 2=$1N8578` |
| `R13` | 10kΩ | `1=RX2, 2=+3.3V` |
| `R14` | 3.3kΩ | `1=SDA2_A, 2=+3.3V` |
| `R15` | 3.3kΩ | `1=SCL2_A, 2=+3.3V` |
| `R16` | 3.3kΩ | `1=SDA2_B, 2=+3.3V` |
| `R17` | 3.3kΩ | `1=SCL2_B, 2=+3.3V` |
| `R18` | 10kΩ | `1=BOOT, 2=+3.3V` |
| `R19` | 1kΩ | `1=$1N4673, 2=RST` |
| `R20` | 51kΩ | `1=$1N4673, 2=+3.3V` |
| `R21` | 100kΩ | `1=+3.3V, 2=$1N13948` |
| `R22` | 100kΩ | `1=+3.3V, 2=$1N13944` |
| `R23` | 1kΩ | `1=EN_A, 2=$1N14018` |
| `R24` | 100kΩ | `1=+3.3V, 2=$1N14908` |
| `R25` | 100kΩ | `1=+3.3V, 2=$1N14906` |
| `R26` | 1kΩ | `1=EN_B, 2=$1N14916` |
| `R28` | 10kΩ | `1=GND, 2=INT2` |
| `R29` | 4.7kΩ | `1=CS1, 2=+3.3V` |
| `R30` | 4.7kΩ | `1=SCLK, 2=+3.3V` |
| `R31` | 4.7kΩ | `1=MOSI, 2=+3.3V` |
| `R32` | 3.3kΩ | `1=LED3, 2=$1N18308` |
| `R33` | 200Ω | `1=GND, 2=$1N24341` |
| `R34` | 3kΩ | `1=$1N24341, 2=$1N2260` |
| `R35` | 330Ω | `1=B2, 2=$1N46033` |
| `R36` | 330Ω | `1=B1, 2=$1N46092` |
| `R37` | 2kΩ | `1=GND, 2=CS` |
| `R39` | 100kΩ | `1=MISO, 2=+3.3V` |
| `R40` | 180kΩ | `1=+3.3V, 2=VERSION_ADC` |
| `R42` | 100kΩ | `1=VERSION_ADC, 2=GND` |
| `SW1` | SS-12D11G5R | `1=VIN, 2=B2, 4=GND, 5=GND, 3=∅` |
| `SW2` | TS-1088-AR02016 | `1=RST, 2=GND` |
| `SW3` | TS-1088-AR02016 | `1=BOOT, 2=GND` |
| `SW4` | TS24CA | `1=CS, 2=B-, 3=GND, 4=GND` |
| `U1` | RT8289GSP | `1[BOOT]=$1N2389, 4[FB]=$1N2260, 6=GND, 7[VIN]=VIN, 8[SW]=$1N2414, 9[PAD]=GND, 2=∅, 3=∅, 5=∅` |
| `U2` | XC6210B332MR | `1[VIN]=+5V, 2=GND, 3[CE]=+5V, 5[OUT]=+3.3V, 4=∅` |
| `U3` | CH340X | `1[UD+]=D+, 2[UD-]=D-, 3=GND, 4[RTS#]=RST, 5[CTS#]=BOOT, 6[TNOW/DTR#]=BOOT, 7[VCC]=+3.3V, 8[TXD]=$1N3813, 9[RXD]=$1N3733, 10[V3]=$1N3674` |
| `U4` | ESP32-S3-WROOM-1-N8R8 | `1=GND, 2[3V3]=+3.3V, 3[EN]=$1N4673, 4[IO4]=SDA2_A, 5[IO5]=SCL2_A, 6[IO6]=PWM3_A, 7[IO7]=PWM2_A, 8[IO15]=PWM1_A, 9[IO16]=EN_A, 10[IO17]=ADC, 11[IO18]=VSTA, 12[IO8]=VERSION_ADC, 13[IO19]=RX1, 14[IO20]=TX1, 15[IO3]=SCLK, 17[IO9]=MOSI, 18[IO10]=MISO, 19[IO11]=SERVO1, 20[IO12]=SERVO2, 21[IO13]=CS1, 22[IO14]=SERVO3, 23[IO21]=SERVO4, 27[IO0]=BOOT, 28[IO35]=LED2, 29[IO36]=LED3, 30[IO37]=EN_B, 31[IO38]=PWM3_B, 32[IO39]=PWM2_B, 33[IO40]=PWM1_B, 34[IO41]=SDA2_B, 35[IO42]=SCL2_B, 36[RXD0]=RX0, 37[TXD0]=TX0, 38[IO2]=TX2, 39[IO1]=RX2, 40=GND, 41=GND, 16=∅, 24=∅, 25=∅, 26=∅` |
| `U5` | ICM-42688P-HXY | `1[SDO/SA0]=MISO, 4[INT1]=INT1, 5[VDDIO]=+3.3V, 6[GNDIO]=GND, 7=GND, 8[VDD]=+3.3V, 9[INT2]=INT2, 12[CSB]=CS1, 13[SCX]=SCLK, 14[SDX]=MOSI, 2=∅, 3=∅, 10=∅, 11=∅` |
| `U6` | MP6536DU-LF-Z | `1[SW2]=U_A, 2[SW2]=U_A, 3[VSP]=VIN, 4[VSP]=VIN, 5[VSP]=VIN, 6[SW1]=V_A, 7[SW1]=V_A, 8[LS1]=GND, 9[LS1]=GND, 10[BST1]=$1N11051, 12[PWM1]=PWM1_A, 13[PWM2]=PWM2_A, 14[FAULTB]=$1N13984, 15[SHDNB]=+3.3V, 16[PWM3]=PWM3_A, 17[STBYB]=$1N14018, 18[AGND]=GND, 19[FLT3B]=$1N13948, 20[FLT2B]=$1N13944, 22[PGND]=GND, 23[PGND]=GND, 26[VSP]=VIN, 27[VSP]=VIN, 28[VSP]=VIN, 29[SW3]=W_A, 30[SW3]=W_A, 31[LS3]=GND, 32[LS3]=GND, 33[BST3]=$1N11092, 34[VDR2]=$1N11138, 35[AGND]=GND, 36[AGND]=GND, 37[VDR1]=$1N11195, 38[BST2]=$1N10925, 39[LS2]=GND, 40[LS2]=GND, 41[EP]=GND, 11=∅, 21=∅, 24=∅, 25=∅` |
| `U7` | MP6536DU-LF-Z | `1[SW2]=U_B, 2[SW2]=U_B, 3[VSP]=VIN, 4[VSP]=VIN, 5[VSP]=VIN, 6[SW1]=V_B, 7[SW1]=V_B, 8[LS1]=GND, 9[LS1]=GND, 10[BST1]=$1N14892, 12[PWM1]=PWM1_B, 13[PWM2]=PWM2_B, 14[FAULTB]=$1N14914, 15[SHDNB]=+3.3V, 16[PWM3]=PWM3_B, 17[STBYB]=$1N14916, 18[AGND]=GND, 19[FLT3B]=$1N14908, 20[FLT2B]=$1N14906, 22[PGND]=GND, 23[PGND]=GND, 26[VSP]=VIN, 27[VSP]=VIN, 28[VSP]=VIN, 29[SW3]=W_B, 30[SW3]=W_B, 31[LS3]=GND, 32[LS3]=GND, 33[BST3]=$1N14893, 34[VDR2]=$1N14895, 35[AGND]=GND, 36[AGND]=GND, 37[VDR1]=$1N14897, 38[BST2]=$1N14887, 39[LS2]=GND, 40[LS2]=GND, 41[EP]=GND, 11=∅, 21=∅, 24=∅, 25=∅` |
| `U8` | HY2120-LB | `1[OD]=$1N42515, 2[OC]=$1N42523, 3[CS]=CS, 4[VC]=$1N46092, 5[VDD]=$1N46033, 6[VSS]=B-` |
| `USB1` | TYPE-C 16PIN 2MD(073) | `1=GND, 2[VBUS]=VBUS, 4[CC1]=$1N3041, 5[DN2]=D-, 6[DP1]=D+, 7[DN1]=D-, 8[DP2]=D+, 10[CC2]=$1N3025, 11[VBUS]=VBUS, 12=GND, 13[SHELL]=GND, 14[SHELL]=GND, 3=∅, 9=∅` |

## CODER (8 Bauteile)

| Ref | Wert / Typ | Pads → Netze |
| --- | --- | --- |
| `C1` | 10uF | `1=GND, 2=3.3V` |
| `C2` | 100nF | `1=GND, 2=3.3V` |
| `H1` | 1.0-4PWB | `1=SDA, 2=SCL, 3=GND, 4=3.3V, 5=GND, 6=GND` |
| `R1` | 100kΩ | `1=$1N478, 2=3.3V` |
| `R2` | 1kΩ | `1=GND, 2=$1N587` |
| `R3` | 10kΩ | `1=SCL, 2=3.3V` |
| `R4` | 10kΩ | `1=SDA, 2=3.3V` |
| `U1` | AS5600-ASOM | `1[VDD5V]=3.3V, 2[VDD3V3]=3.3V, 4=GND, 5[PGO]=$1N587, 6[SDA]=SDA, 7[SCL]=SCL, 8[DIR]=$1N478, 3=∅` |

