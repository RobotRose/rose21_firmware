'' Mobile robot power board controller
'' Opteq June/July 2014 HJ Kiela
'' V0
'' ADC 16 channel, buzzer, 2 batteries 6 ower outputs
'' Complete test 23-7-2014 iteration 2 of PCB
''
'' ********************************************************************
'' WARNING!! Do not connect or disconnect any battery and or power consuming device to an active connection!!!!!
'' Otherwise the FET-switches will break down
'' ********************************************************************
''
'' Commands:
'' $90 Debug monitor on
'' $91 Debug monitor off  (default)
''
'' $9: Reset min/max for current and voltage measurements
''
'' $10: Switch all batteries off
'' $11: Battery 1 active
'' $12: Battery 2 active
''
'' $21: Controller output on
'' $22: PC1 on
'' $23: PC2 on
'' $24: DR1 on
'' $25: DR2 on
'' $26: AUX on

'' $30: All outputs off
'' $31: Controller off
'' $32: PC1 off
'' $33: PC2 off
'' $34: DR1 off
'' $35: DR2 off
'' $36: AUX off

'' Summary of operation
''                            [1]     [2]     [3]     [4]     [5]     [6]
''      ┌────────┐   /        CONTR   PC1     PC2     DR1     DR2     AUX
''      │ Bat1   │──/  ───┐    │       │       │       │       │       │
''      └────────┘ $11    │    │       │       │       │       │       │
''                        │     /       /       /       /       /       /
''                        ┣────/────── /────── /────── /────── / ───── /
''      ┌────────┐   /    │ on $21    $22    $23      $24     $25     $26
''      │ Bat2   │──/  ───┘off $31    $32    $33      $34     $35     $36
''      └────────┘ $12     All outputs off $30
'' 
''        $10 all batteries off
'' 
'' 
'' Data for PC:
'' NTC    NC Vcont  IAux  IDr2  IDr1  IPC2  IPC1 ICntr IBat2 IBat1  V24b VBat1 VBat2   3V3    5V
'' Data available in: Raw ADC bits
''                    Voltage (mV)
''                    Engineering units (mW, mOhm, mA)
'' 

DAT Product BYTE "900087 Robot Power Board",0
    Version BYTE "1",0
    SubVersion BYTE "2",0
    
    
    NOTE_B0  WORD 31
    NOTE_C1  WORD 33
    NOTE_CS1 WORD 35
    NOTE_D1  WORD 37
    NOTE_DS1 WORD 39
    NOTE_E1  WORD 41
    NOTE_F1  WORD 44
    NOTE_FS1 WORD 46
    NOTE_G1  WORD 49
    NOTE_GS1 WORD 52
    NOTE_A1  WORD 55
    NOTE_AS1 WORD 58
    NOTE_B1  WORD 62
    NOTE_C2  WORD 65
    NOTE_CS2 WORD 69
    NOTE_D2  WORD 73
    NOTE_DS2 WORD 78
    NOTE_E2  WORD 82
    NOTE_F2  WORD 87
    NOTE_FS2 WORD 93
    NOTE_G2  WORD 98
    NOTE_GS2 WORD 104
    NOTE_A2  WORD 110
    NOTE_AS2 WORD 117
    NOTE_B2  WORD 123
    NOTE_C3  WORD 131
    NOTE_CS3 WORD 139
    NOTE_D3  WORD 147
    NOTE_DS3 WORD 156
    NOTE_E3  WORD 165
    NOTE_F3  WORD 175
    NOTE_FS3 WORD 185
    NOTE_G3  WORD 196
    NOTE_GS3 WORD 208
    NOTE_A3  WORD 220
    NOTE_AS3 WORD 233
    NOTE_B3  WORD 247
    NOTE_C4  WORD 262
    NOTE_CS4 WORD 277
    NOTE_D4  WORD 294
    NOTE_DS4 WORD 311
    NOTE_E4  WORD 330
    NOTE_F4  WORD 349
    NOTE_FS4 WORD 370
    NOTE_G4  WORD 392
    NOTE_GS4 WORD 415
    NOTE_A4  WORD 440
    NOTE_AS4 WORD 466
    NOTE_B4  WORD 494
    NOTE_C5  WORD 523
    NOTE_CS5 WORD 554
    NOTE_D5  WORD 587
    NOTE_DS5 WORD 622
    NOTE_E5  WORD 659
    NOTE_F5  WORD 698
    NOTE_FS5 WORD 740
    NOTE_G5  WORD 784
    NOTE_GS5 WORD 831
    NOTE_A5  WORD 880
    NOTE_AS5 WORD 932
    NOTE_B5  WORD 988
    NOTE_C6  WORD 1047
    NOTE_CS6 WORD 1109
    NOTE_D6  WORD 1175
    NOTE_DS6 WORD 1245
    NOTE_E6  WORD 1319
    NOTE_F6  WORD 1397
    NOTE_FS6 WORD 1480
    NOTE_G6  WORD 1568
    NOTE_GS6 WORD 1661
    NOTE_A6  WORD 1760
    NOTE_AS6 WORD 1865
    NOTE_B6  WORD 1976
    NOTE_C7  WORD 2093
    NOTE_CS7 WORD 2217
    NOTE_D7  WORD 2349
    NOTE_DS7 WORD 2489
    NOTE_E7  WORD 2637
    NOTE_F7  WORD 2794
    NOTE_FS7 WORD 2960
    NOTE_G7  WORD 3136
    NOTE_GS7 WORD 3322
    NOTE_A7  WORD 3520
    NOTE_AS7 WORD 3729
    NOTE_B7  WORD 3951
    NOTE_C8  WORD 4186
    NOTE_CS8 WORD 4435
    NOTE_D8  WORD 4699
    NOTE_DS8 WORD 4978

CON

'Set 80Mhz
   _clkmode = xtal1+pll16x
   _xinfreq = 5000000

   
'' Serial port 
   CR = 13
   LF = 10
   CS = 0
   CE = 11                 'CE: Clear to End of line
   TXD = 30 '26 '30
   RXD = 31 '27 '31
   Baud = 115200
   
   NumWidth = 5            ' Number of positions for debug array display

'' Strings
  lMaxStr = 257        'Stringlength is 256 + 1 for 0 termination
  StringSize = lMaxStr-1
  bytebuffersize = 2048
  Cmdlen = 10
  LineLen = bytebuffersize ' Buffer size for incoming line   
     
'ADC
   dpin1 = 4 '12
   cpin1 = 7 '13
   spin1 = 6

   dpin2 = 4
   cpin2 = 7
   spin2 = 5

   dpin3 = 3
   cpin3 = 7
   spin3 = 3

   chip1 = 1
   chip2 = 2
   chip3 = 3

   Mode1 = 1

'LED
   LED = 25

'Buzzer
   BUZZ         = 9

'Power channels
   PWRBAT1      = 16
   PWRBAT2      = 17
   PWRCONTR     = 18
   PWRPC1       = 19
   PWRPC2       = 20
   PWRDR2       = 22
   PWRDR1       = 21 
   PWRAUX       = 23

'ADC channels
   NCh = 16              'Number of measuring channels

  'ADC1   
   cNTC       = 0
   NC1        = 1  'not used
   V24VContr  = 2
   IAUX       = 3
   IDR2       = 4
   IDR1       = 5
   IPC2       = 6
   IPC1       = 7

   'ADC2
   cIControl  = 8
   cIBAT2     = 9
   cIBAT1     = 10
   cV24VBus   = 11
   cVBAT1     = 12
   cVBAT2     = 13
   cV3V3      = 14
   cV5V       = 15

 ' Conversion factors for Engineering value of measured ADC values
   cADCbits2mV = 1.220703
 
   mVBat     = 11.0 ' Real voltage in mV on battery resitor network 10k 1k
   mI        = 10.0 ' 1:10000 conversion from mA current to mV
   
'Safety I/O
  EMERin = 0   'Safety input
  OK     = 1   'Safety output relais
  IN0    = 2   'Extra input optocoupler
  OUT0   = 3   'Extra relais output

' Safety related values
  c5V = 4800                 ' Minimal 5V supply
  c3V3 = 3200                ' Minimal 3V3 supply
  cVin = 22000               ' Minimal Vin supply
   
OBJ
  ADC           : "MCP3208_fast_multi"                  ' ADC
  ser           : "full_duplex_serial_005"              ' Full duplex serial communication 
  t             : "Timing"
  num           : "simple_numbers"                      ' Number to string conversion
  STRs          : "STRINGS2hk"
  f             : "FloatMath1_1"                        ' Floating point library
  
VAR
  LONG ADC1Cog, ADC2Cog, SerCog, ADCMCog, ADCMStack[50], ADCConvStack[250], ADCConvCog
  LONG PLCCog, PLCStack[50], LinMotCog
  Long ADCRaw[NCh], ADCRawMin[NCh], ADCRawMax[NCh], ADCRawAvg[NCh]  ' Raw bit values
  Long ADCch[NCh], ADCchAVG[NCh], ADCchMAX[NCh], ADCchMIN[NCh]      ' Volt
  Long engADCch[NCh], engADCchAVG[NCh], engADCchMAX[NCh], engADCchMIN[NCh] ' Scaled values 
  LONG Switchstate, MainCnt, MaxCh, ADCTime, ADCCnt, DoMotorCnt, ScalingCnt, ScalingTime

  ' Safety related vars
  Long AllOK, s5VOK, s3V3OK, sVInOK, OutofRange, PlcCnt

  Long Scale[Nch]   '' array with scaling factors
  
  'Battery management
  Long AutoBatterySwitch  '1: enables local auto battery switch
  Long BatteryActive      '0: all batteries off, 1: Battery 1 on 2: battery 2 on

  'Vars for command handling and processing
  Long ConnectCnt, ConnectTime    
  byte data[bytebuffersize]     'Receive buffer
  long DataLen                  'Buffer len
  Byte StrMiscReport[lMaxStr]   'Report string various other data
  Long CommandTime, ConnectionTime
  Long Sender, LastPar1[CmdLen]
  
 'Input string handling
  Byte StrBuf[lMaxStr]  'String buffer for receiving chars from serial port
  Long StrSP, StrP, StrLen        'Instring semaphore, counter, Stringpointer, String length
  Byte Ch

' Debug
  LONG Debug
       
PUB Main
  Init

  Beep 

  ' Main loop
  repeat
    MainCnt++

    handleCommunication      

    If Debug
      'ser.position(0,2)
      DoDisplay

    t.Pause1ms(50)
    !OUTA[Led]

PRI handleCommunication | NewCh, T1
    ' Next section handles command char input
    NewCh:=Ser.RxCheck
    if NewCh>0
      If Debug
        Ser.Str(string("New Cmd received"))
        T1:=cnt
        Ser.Str(num.decf(ConnectCnt,4))
        Ser.Str(string(" ",Ser#NL, Ser#LF))

      'Initialize the buffers and bring the data over
      bytefill(@data, 0, bytebuffersize)
      ser.StrIn(@Data)
      bytemove(@data+1,@data,bytebuffersize-1)
      data[0]:=NewCh      
      DataLen:=strsize(@data)        'store string length for use by parsers
      
      If debug
        Ser.Str(string(CR, "Packet from client: "))
        Ser.Str(@Data)
        Ser.char(CR)

      DoCommand   'Execute command in received string

'Do switches
PRI DOSwitches
'0 = off 1 = on

    case Switchstate // 5
      0: OUTA[PWRBAT1]:=0   '
         OUTA[PWRBAT2]:=1
         OUTA[PWRCONTR]:=1
         OUTA[PWRPC1]:=1
         OUTA[PWRPC2]:=1  
         OUTA[PWRDR1]:=1
         OUTA[PWRDR2]:=1
         OUTA[PWRAUX]:=1
         
      1: OUTA[PWRBAT1]:=0   '
         OUTA[PWRBAT2]:=1
         OUTA[PWRCONTR]:=0
         OUTA[PWRPC1]:=0
         OUTA[PWRPC2]:=0
         OUTA[PWRDR1]:=0
         OUTA[PWRDR2]:=0
         OUTA[PWRAUX]:=0

      2: OUTA[PWRBAT1]:=0   '
         OUTA[PWRBAT2]:=1
         OUTA[PWRCONTR]:=0
         OUTA[PWRPC1]:=0
         OUTA[PWRPC2]:=0  
         OUTA[PWRDR1]:=0
         OUTA[PWRDR2]:=0
         OUTA[PWRAUX]:=0

      3: OUTA[PWRBAT1]:=1   '
         OUTA[PWRBAT2]:=0
         OUTA[PWRCONTR]:=1
         OUTA[PWRPC1]:=1
         OUTA[PWRPC2]:=1  
         OUTA[PWRDR1]:=1
         OUTA[PWRDR2]:=1
         OUTA[PWRAUX]:=1

      4: OUTA[PWRBAT1]:=0   '
         OUTA[PWRBAT2]:=0
         OUTA[PWRCONTR]:=0
         OUTA[PWRPC1]:=0
         OUTA[PWRPC2]:=0  
         OUTA[PWRDR1]:=0
         OUTA[PWRDR2]:=0
         OUTA[PWRAUX]:=0
  
'Switch battery 1 or 2. Disable outputs to cut all output current off from darlington
'Otherwise the Fet won't switch off properly
PRI SwitchBat(N)
  case N
    0: OUTA[PWRBAT1]:=0   '
       OUTA[PWRBAT2]:=0
       DIRA[PWRBAT1]:=0
       DIRA[PWRBAT2]:=0
       
    1: DIRA[PWRBAT1]:=1
       OUTA[PWRBAT1]:=1 '
       DIRA[PWRBAT2]:=0
       OUTA[PWRBAT2]:=0

    2: DIRA[PWRBAT1]:=0
       OUTA[PWRBAT1]:=0 '
       DIRA[PWRBAT2]:=1
       OUTA[PWRBAT2]:=1

' Switch output 1 to 6. 0= all off
PRI SwitchAllOff
    DIRA[PWRCONTR]:=0
    DIRA[PWRPC1]:=0
    DIRA[PWRPC2]:=0  
    DIRA[PWRDR1]:=0
    DIRA[PWRDR2]:=0
    DIRA[PWRAUX]:=0     
    OUTA[PWRCONTR]:=0
    OUTA[PWRPC1]:=0
    OUTA[PWRPC2]:=0  
    OUTA[PWRDR1]:=0
    OUTA[PWRDR2]:=0
    OUTA[PWRAUX]:=0

PRI SwitchOn(N)
  Case N
    1: DIRA[PWRCONTR]:=1
       OUTA[PWRCONTR]:=1
    2: DIRA[PWRPC1]:=1
       OUTA[PWRPC1]:=1
    3: DIRA[PWRPC2]:=1
       OUTA[PWRPC2]:=1
    4: DIRA[PWRDR1]:=1
       OUTA[PWRDR1]:=1
    5: DIRA[PWRDR2]:=1
       OUTA[PWRDR2]:=1
    6: DIRA[PWRAUX]:=1
       OUTA[PWRAUX]:=1

PRI SwitchOff(N)
  Case N
    1: DIRA[PWRCONTR]:=0
       OUTA[PWRCONTR]:=0
    2: DIRA[PWRPC1]:=0
       OUTA[PWRPC1]:=0
    3: DIRA[PWRPC2]:=0
       OUTA[PWRPC2]:=0
    4: DIRA[PWRDR1]:=0
       OUTA[PWRDR1]:=0
    5: DIRA[PWRDR2]:=0
       OUTA[PWRDR2]:=0
    6: DIRA[PWRAUX]:=0
       OUTA[PWRAUX]:=0

' ------------------------ Send AVG parameters to PC
PRI ReportAVG | i
    i:=0
    ser.str(string("eADCvAVG,"))   ' Parameter name
    ser.dec(NCh)                   ' Number of values
    repeat NCh
      ser.tx(",")  
      ser.str(num.decf(engADCchAVG[i],NumWidth)) 'Engineering avg
      i++

    ser.tx(",")  
    ser.tx(CR)  

PRI ReportMAX | i
    i:=0
    ser.str(string("eADCvMAX,"))   ' Parameter name
    ser.dec(NCh)                   ' Number of values
    repeat NCh
      ser.tx(",")  
      ser.str(num.decf(engADCchMAX[i],NumWidth)) 'Engineering avg
      i++

    ser.tx(",")  
    ser.tx(CR)  

PRI ReportMIN | i
    i:=0
    ser.str(string("eADCvMIN,"))   ' Parameter name
    ser.dec(NCh)                   ' Number of values
    repeat NCh
      ser.tx(",")  
      ser.str(num.decf(engADCchMIN[i],NumWidth)) 'Engineering avg
      i++

    ser.tx(",")  
    ser.tx(CR)  

PRI ReportLabels
    ser.str(string("Labels,"))   ' Parameter name
    ser.dec(NCh)                   ' Number of values
    ser.tx(",")  
    ser.str(@s_NTC)
    ser.tx(",")  
    ser.str(@s_NC)
    ser.tx(",")  
    ser.str(@s_Vcontrol)
    ser.tx(",")  
    ser.str(@s_iAux)
    ser.tx(",")  
    ser.str(@s_iDrive2)
    ser.tx(",")  
    ser.str(@s_iDrive1)
    ser.tx(",")  
    ser.str(@s_ipc2)
    ser.tx(",")  
    ser.str(@s_ipc1)
    ser.tx(",")  
    ser.str(@s_iControl)
    ser.tx(",")  
    ser.str(@s_ibat2)
    ser.tx(",")  
    ser.str(@s_ibat1)
    ser.tx(",")  
    ser.str(@s_V24b)     
    ser.tx(",")  
    ser.str(@s_vbat1)
    ser.tx(",")  
    ser.str(@s_vbat2)
    ser.tx(",")  
    ser.str(@s_3V3)             
    ser.tx(",")  
    ser.str(@s_5V)
    ser.tx(",")  
    ser.tx(CR)  

' ------------------------ Show debug output -----------------------------
PRI DoDisplay  | i
'     ser.tx(CE)
    ser.position(0,0)

    ser.str(@Product)
    ser.str(@Version)
    ser.str(@SubVersion)
    ser.str(string(" ADCmeetCog: "))
    ser.dec(ADCMCog)
    ser.str(string("  ADCConvCog: "))
    ser.dec(ADCConvCog)
    ser.str(string(" PLCCog: "))
    ser.dec(LinMotCog)
     
    ser.tx(CR)
    ser.tx(CR)
    ser.str(string("       "))
    ser.str(@s_NTC)
    ser.str(@s_NC)
    ser.str(@s_Vcontrol)
    ser.str(@s_iAux)
    ser.str(@s_iDrive2)
    ser.str(@s_iDrive1)
    ser.str(@s_ipc2)
    ser.str(@s_ipc1)
    ser.str(@s_iControl)
    ser.str(@s_ibat2)
    ser.str(@s_ibat1)
    ser.str(@s_V24b)     
    ser.str(@s_vbat1)
    ser.str(@s_vbat2)
    ser.str(@s_3V3)             
    ser.str(@s_5V)             

    ser.tx(CR)
    i:=0
    ser.str(string("      #: "))
    repeat NCh               'Actual adc
      ser.str(num.decf(i,NumWidth))
      i++
      ser.tx(" ")

    ser.tx(CE)
    ser.tx(CR)

    i:=0
    ser.str(string("    ADCr:"))
    repeat NCh               'Actual adc
      ser.str(num.decf(ADCRaw[i],NumWidth))
      i++
      ser.tx(" ")

    ser.tx(CE)
    ser.tx(CR)

    i:=0
    ser.str(string("    Avgr:"))
    repeat NCh
      ser.str(num.decf(ADCRawAVG[i],NumWidth)) 'AVG adc value
      i++
      ser.tx(" ")      

    ser.tx(CE)

    ser.tx(CR)
    i:=0
    ser.str(string("    Minr:"))
    repeat NCh
      ser.str(num.decf(ADCRawMIN[i],NumWidth)) 'Min adc value
      i++
      ser.tx(" ")

    ser.tx(CE)

    ser.tx(CR)
    i:=0
    ser.str(string("    Maxr:"))
    repeat NCh
      ser.str(num.decf(ADCRawMAX[i],NumWidth)) 'Max adc value
      i++
      ser.tx(" ")
    ser.tx(CE)

    ser.tx(CR)
    ser.tx(CR)
    i:=0
    ser.str(string("    ADCv:"))
    repeat NCh
      ser.str(num.decf(ADCch[i],NumWidth))     'Converted
      i++
      ser.tx(" ")
    ser.tx(CE)

    ser.tx(CR)
    i:=0
    ser.str(string(" ADCvAvg:"))
    repeat NCh
      ser.str(num.decf(ADCchAVG[i],NumWidth))  'Converted avg
      i++
      ser.tx(" ")
    ser.tx(CE)

    ser.tx(CR)
    i:=0
    ser.str(string(" ADCvMin:"))
    repeat NCh
      ser.str(num.decf(ADCchMIN[i],NumWidth))  'Converted min
      i++
      ser.tx(" ")
    ser.tx(CE)

    ser.tx(CR)
    i:=0
    ser.str(string(" ADCvMax:"))
    repeat NCh
      ser.str(num.decf(ADCchMAX[i],NumWidth))  'Converted max
      i++
      ser.tx(" ")
    ser.tx(CE)

    ser.tx(CR)
    ser.tx(CR)
    i:=0
    ser.str(string("eADCvAVG:"))
    repeat NCh
      ser.str(num.decf(engADCchAVG[i],NumWidth)) 'Engineering avg
      i++
      ser.tx(" ")
    ser.tx(CE)

    ser.tx(CR)
    i:=0
    ser.str(string("eADCvMax:"))
    repeat NCh
      ser.str(num.decf(engADCchMax[i],NumWidth)) 'Engineering Max
      i++
      ser.tx(" ")
    ser.tx(CE)

    ser.tx(CR)
    i:=0
    ser.str(string("eADCvMin:"))
    repeat NCh
      ser.str(num.decf(engADCchMin[i],NumWidth)) 'Engineering Min
      i++
      ser.tx(" ")
    ser.tx(CE)

    ser.tx(CR)
    ser.tx(CR)
    ser.str(string("ADC time: "))
    ser.dec(ADCTime)  
    ser.tx(" ")
    ser.str(string(" ADC cnt: "))
    ser.dec(ADCCnt)
    ser.str(string(" Scaling time: "))
    ser.dec(ScalingTime)
    ser.tx(" ")
    ser.str(string(" Scaling cnt: "))
    ser.dec(ScalingCnt)
    ser.tx(" ")
    ser.str(string(" PLC cnt: "))
    ser.dec(PLCCnt)
    ser.tx(" ")
    ser.str(string("MainCnt: "))
    ser.dec(MainCnt)   

    ser.tx(cr)
    ser.tx(ce)

    ser.str(string("EMER: "))
    ser.dec(GetEMERin)
    ser.str(string(" OK: "))
    ser.dec(GetOK)
    ser.str(string(" IN0: "))
    ser.dec(GetIN0)
    ser.str(string(" OUT0: "))
    ser.dec(GetOUT0)
    ser.tx(cr)
    ser.tx(ce)
    ser.str(string("sV5OK: "))
    ser.dec(s5VOK)
    ser.str(string(" sV3V3OK: "))
    ser.dec(s3V3OK)
    ser.str(string(" sVinOK: "))
    ser.dec(sVinOK)
    ser.str(string(" AllOK: "))
    ser.dec(AllOK)
    ser.tx(cr)
    
    DisplayIO


' Display IO status on screen
PRI DisplayIO
  ser.tx(cr)
  ser.str(string("    "))
  ser.str(@s_CON)
  ser.str(string(" "))
  ser.str(@s_PC1)
  ser.str(string(" "))
  ser.str(@s_PC2)
  ser.str(string(" "))
  ser.str(@s_DR1)
  ser.str(string(" "))
  ser.str(@s_DR2)
  ser.str(string(" "))
  ser.str(@s_AUX)
  ser.tx(cr)

  ser.str(string("   "))
  ser.str(num.decf(GetContr,3)) 'Engineering avg
  ser.str(string(" "))
  ser.str(num.decf(GetPC1,3)) '
  ser.str(string(" "))
  ser.str(num.decf(GetPC2,3)) '
  ser.str(string(" "))
  ser.str(num.decf(GetDR1,3)) '
  ser.str(string(" "))
  ser.str(num.decf(GetDR2,3)) '
  ser.str(string(" "))
  ser.str(num.decf(GetAUX,3)) '
  ser.str(string(" "))
  
  ser.tx(cr)
  ser.tx(ce)

' --------------------------------- Do logic tasks and safety tasks
PRI DoPLC
  Repeat
  ' Check safetay related values

    if engADCchAVG[cV5V] < c5V
      s5VOK :=0

    if engADCchAVG[cV3V3] < c3V3                                                               
      s3V3OK :=0

    if engADCchAVG[cV24VBus] < cVin
      sVinOK :=0

   ' Battery management and automatic switch from Bat1 to Bat 2
   ' WARNING!  Can only work if battery properties of both batteries are properly set!!
 '   if engADCchAVG[cVBAT1] < cVin
 '     sVinOK :=0

    if sVinOK==1 and s3V3OK==1 and  s5VOK==1  '' Check power supplies 
      SetOUT0(1)
    else
      SetOUT0(0)
      
    If GetEMERin==1 ' and sVinOK==1 and AllOK==1 ' Process emergency alarms to OK output
      SetOK(1)
    else
      SetOK(0)
      AllOK:=0
 
    PlcCnt++
    
' ----------------------- Reset program ------------------------
PRI DoReset
  sVinOK:=1
  s3V3OK :=1
  s5VOK :=1
  AllOK:=1
    
'Measure ADC channels. 
{PRI DoADC | i, j, T1
   repeat
     T1:=cnt

     i:=0
     j:=0
     repeat 8 ' NCh
       ADCRaw[i]:= ADC1.in(i)      
'      ADCRaw[i]:= ADC.in(j,chip1) 
       ADCRawAVG[i]:= (ADCRawAVG[i] *9 + ADCRaw[i] )/10
'      ADCRawAVG[i]:= (ADCRawAVG[i] *9 + ADC.in(i) )/10
       ADCRawMIN[i] <#= ADCRawAVG[i]         'save min value
       ADCRawMAX[i] #>= ADCRawAVG[i]         'save max value
       i++
       j++

 {   j:=0
     repeat 8 ' NCh
       ADCRaw[i]:= ADC2.in(i)      
'      ADCRaw[i]:= ADC.in(j,chip2) 
       ADCRawAVG[i]:= (ADCRawAVG[i] *9 + ADCRaw[i] )/10
'      ADCRawAVG[i]:= (ADCRawAVG[i] *9 + ADC.in(i) )/10
       ADCRawMIN[i] <#= ADCRawAVG[i]         'save min value
       ADCRawMAX[i] #>= ADCRawAVG[i]         'save max value
       i++
       j++
    }
     ADCCnt++
        
     ADCTime:=(CNT-T1)/80   }
  
'Measure ADC channels.
PRI DoADC | i, T1
   repeat
     T1:=cnt

{    ADCRaw[0]:= ADC.in(0,chip1)
     ADCRaw[1]:= ADC.in(1,chip1)
     ADCRaw[2]:= ADC.in(2,chip1)
     ADCRaw[3]:= ADC.in(3,chip1)
     ADCRaw[4]:= ADC.in(4,chip1)
     ADCRaw[5]:= ADC.in(5,chip1)
     ADCRaw[6]:= ADC.in(6,chip1)
     ADCRaw[7]:= ADC.in(7,chip1)

     ADCRaw[8]:= ADC.in(0,chip2)
     ADCRaw[9]:= ADC.in(1,chip2)
     ADCRaw[10]:= ADC.in(2,chip2)
     ADCRaw[11]:= ADC.in(3,chip2)
     ADCRaw[12]:= ADC.in(4,chip2)
     ADCRaw[13]:= ADC.in(5,chip2)
     ADCRaw[14]:= ADC.in(6,chip2)
     ADCRaw[15]:= ADC.in(7,chip2)    }

     i:=0
     repeat NCh
'      ADCRaw[i]:= ADC.in(i)       
       if i<8
         ADCRaw[i]:= ADC.in(i,chip1)       ' First 8 channels from ADC1 
'        ADCRaw[i]:= ADC1.in(i)            ' First 8 channels from ADC1
       else
         ADCRaw[i]:= ADC.in(i-8,chip2)     ' And next 8 from ADC2  
'        ADCRaw[i]:= ADC2.in(i-8)          ' And next 8 from ADC2

       
       ADCRawAVG[i]:= (ADCRawAVG[i] *9 +  ADCRaw[i] )/10
'      ADCRawAVG[i]:= (ADCRawAVG[i] *9 + ADC.in(i) )/10
       ADCRawMIN[i] <#= ADCRaw[i]            'save min value
       ADCRawMAX[i] #>= ADCRaw[i]            'save max value
       i++
 
 

     ADCCnt++
        
     ADCTime:=(CNT-T1)/80

' Do scaling from counts to engineering values * 1000
  

PRI DoADCScaling | T1, i
  ' Fill calibration table
  Scale[0]:= 1.0                ' NTC
  Scale[2]:= 1.0                ' nc
  Scale[2]:= mVBat              ' Vcontroller
  Scale[3]:= mI                 ' Iaux
  Scale[4]:= mI                 ' IDr2
  Scale[5]:= mI                 ' IDr1
  Scale[6]:= mI                 ' IPC2
  Scale[7]:= mI                 ' IPC1
  Scale[8]:= mI                 ' IContr
  Scale[9]:= mI                 ' IBat2
  Scale[10]:= mI                ' IBat1
  Scale[11]:= mVBat             ' Vbat1
  Scale[12]:= mVBat             ' Vbat2
  Scale[13]:= mVBat             ' V24b
  Scale[14]:= 1.0               ' V3V3
  Scale[15]:= 2.05              ' V5V
  
  Repeat
    T1:=cnt

    i:=0
    Repeat NCh
      ADCch[i]:= f.fround(f.fmul(f.Ffloat(ADCRaw[i]), cADCbits2mV)) 'Calculate mV
      ADCchAVG[i]:= f.fround(f.fmul(f.Ffloat(ADCRawAVG[i]), cADCbits2mV)) 'Calculate mV
      ADCchMax[i]:= f.fround(f.fmul(f.Ffloat(ADCRawMax[i]), cADCbits2mV)) 'Calculate mV
      ADCchMin[i]:= f.fround(f.fmul(f.Ffloat(ADCRawMin[i]), cADCbits2mV)) 'Calculate mV

'     case Nch   ' Scaling to integer engineering units (mV and mA). 
'       0: engADCchAVG[i]:=
'       1: engADCchAVG[i]:=99
'       2: engADCchAVG[i]:= f.fround(f.fmul(f.Ffloat(ADCchAVG[i]), Scale[i])) 'Calculate mV
        engADCchAVG[i]:= f.fround(f.fmul(f.Ffloat(ADCchAVG[i]), Scale[i])) 'Calculate AVG mV
        engADCchMAX[i]:= f.fround(f.fmul(f.Ffloat(ADCchMAX[i]), Scale[i])) 'Calculate MAX mV
        engADCchMIN[i]:= f.fround(f.fmul(f.Ffloat(ADCchMIN[i]), Scale[i])) 'Calculate MIN mV
      i++
      
    ScalingCnt++
    ScalingTime:=(CNT-T1)/80

PRI Init

'  SerCog:=ser.start(Baud)                 'Start serial port start(rxpin, txpin, mode, baudrate)
  SerCog:=ser.start(rxd, txd, 0, baud)     'serial port on prop plug
  ser.Clear
  t.Pause1ms(200)

  Debug:=false 

  ADC1Cog:= ADC.Start(dpin1, cpin1, spin1, dpin2, cpin2, spin2, dpin3, cpin3, spin3)
'  ADC1Cog:= ADC1.Start(dpin1, cpin1, spin1, Mode1) ' Start ADC low level
'  ADC2Cog:= ADC2.Start(dpin2, cpin2, spin2, Mode1) ' ADC2
  MaxCh:= NCh-1

'  OUTA[Switch]:=1
'  DIRA[Switch] ~~  'set as output

  OUTA[Led]:=1
  DirA[Led] ~~

  DirA[BUZZ] ~~    'Set Buzzer as output

  ' Battery and output init
'   SwitchBat(0)
 '  SwitchAllOff 

  SwitchState:=0

  ADCMCog:=cognew(DoADC,@ADCMStack)
  ADCConvCog:=cognew(DoADCScaling,@ADCConvStack)
  PlcCog:=cognew(DoPLC,@PLCStack)            'Start PLC cog

' ---------------- Set OK output on or off (1 or 0) ---------------------------------------
PUB SetOK(Value)
  if Value==0
    DirA[OK]:=0
    OUTA[OK]:=0
  else
    DirA[OK]:=1
    OUTA[OK]:=1
' ---------------- Set OUT0 output on or off (1 or 0) ---------------------------------------
PUB SetOUT0(Value)
  if Value==0
    DirA[OUT0]:=0  
    OUTA[OUT0]:=0
  else  
    DirA[OUT0]:=1  
    OUTA[OUT0]:=1

' ---------------- Get status OK output (on=1 or off=0) ---------------------------------------
PUB GetOK
  if InA[OK]==1
    Return 1
  else
    Return 0  

' ---------------- Get status OK output (on=1 or off=0) ---------------------------------------
PUB GetOUT0
  if InA[OUT0]==1
    Return 1
  else
    Return 0   

' ---------------- Get EMERin input (on=1 or off=0) ---------------------------------------
PUB GetEMERin
Return InA[EMERin]  

' ---------------- Get IN0 input (on=1 or off=0) ---------------------------------------
PUB GetIN0
Return InA[IN0]
  
' ---------------- Get status CONTR output (on=1 or off=0) ---------------------------------------
PUB GetCONTR
  if InA[PWRCONTR]==1
    Return 1
  else
    Return 0   

' ---------------- Get status PC1 output (on=1 or off=0) ---------------------------------------
PUB GetPC1
  if InA[PWRPC1]==1
    Return 1
  else
    Return 0                 

' ---------------- Get status PC2 output (on=1 or off=0) ---------------------------------------
PUB GetPC2
  if InA[PWRPC2]==1
    Return 1
  else
    Return 0                 

' ---------------- Get status DR1 output (on=1 or off=0) ---------------------------------------
PUB GetDR1
  if InA[PWRDR1]==1
    Return 1
  else
    Return 0                 

' ---------------- Get status DR2 output (on=1 or off=0) ---------------------------------------
PUB GetDR2
  if InA[PWRDR2]==1
    Return 1
  else
    Return 0                 

  
' ---------------- Get status AUX output (on=1 or off=0) ---------------------------------------
PUB GetAUX
  if InA[PWRAUX]==1
    Return 1
  else
    Return 0   

PRI Beep
  repeat 1000
    !OUTA[BUZZ]
    t.Pause10us(100)
    
PRI BeepHz(hz, time) | times, sleep_time   '-- time in us
    if hz==0
        t.Pause10us(time/10)
    else
      sleep_time := 1000000/((hz)/2)  ' [us]
      times      := (time/sleep_time)*2 ' [#]                               
      repeat times
        !OUTA[BUZZ]
        t.Pause10us(sleep_time/10)
        
PRI MarioUnderworldTune
  repeat 2
    BeepHz(NOTE_C4, 1000000/12)
    BeepHz(NOTE_C5, 1000000/12)
    BeepHz(NOTE_A3, 1000000/12)
    BeepHz(NOTE_A4, 1000000/12)
    BeepHz(NOTE_AS3, 1000000/12)
    BeepHz(NOTE_AS4, 1000000/12)
    BeepHz(0, 1000000/6)
    BeepHz(0, 1000000/3)  
  
  BeepHz(NOTE_F3, 1000000/12)
  BeepHz(NOTE_F4, 1000000/12)
  BeepHz(NOTE_D3, 1000000/12)
  BeepHz(NOTE_D4, 1000000/12)
  BeepHz(NOTE_DS3, 1000000/12)
  BeepHz(NOTE_DS4, 1000000/12)
  BeepHz(0, 1000000/6)
  BeepHz(0, 1000000/3)
  
  BeepHz(NOTE_F3, 1000000/12)
  BeepHz(NOTE_F4, 1000000/12)
  BeepHz(NOTE_D3, 1000000/12)
  BeepHz(NOTE_D4, 1000000/12)
  BeepHz(NOTE_DS3, 1000000/12)
  BeepHz(NOTE_DS4, 1000000/12)
  BeepHz(0, 1000000/6)
  BeepHz(0, 1000000/6)
  BeepHz(NOTE_DS4, 1000000/18)
  BeepHz(NOTE_CS4, 1000000/18)
  BeepHz(NOTE_D4, 1000000/18)

  BeepHz(NOTE_CS4, 1000000/6)
  BeepHz(NOTE_DS4, 1000000/6)
  BeepHz(NOTE_DS4, 1000000/6)
  BeepHz(NOTE_GS3, 1000000/6)
  BeepHz(NOTE_G3, 1000000/6)
  BeepHz(NOTE_CS4, 1000000/6)
 
  BeepHz(NOTE_C4, 1000000/18)
  BeepHz(NOTE_FS4, 1000000/18)
  BeepHz(NOTE_F4, 1000000/18)
  BeepHz(NOTE_E3, 1000000/18)
  BeepHz(NOTE_AS4, 1000000/18)
  BeepHz(NOTE_A4, 1000000/18)

  BeepHz(NOTE_GS4, 1000000/10)
  BeepHz(NOTE_DS4, 1000000/10)
  BeepHz(NOTE_B3, 1000000/10)
  
  BeepHz(NOTE_AS3, 1000000/10)
  BeepHz(NOTE_A3, 1000000/10)
  BeepHz(NOTE_GS3, 1000000/10)
  
  BeepHz(0, 1000000/3)
  BeepHz(0, 1000000/3)
  BeepHz(0, 1000000/3)        
        

' ------------------------------------ Reset Min Max values ADC conversion
PRI ResetMinMax  | ii
  ii:=0
  repeat NCh
    ADCRawMIN[ii]:=999999
    ADCRawMAX[ii]:=0
    ADCRawAVG[ii]:=0
    ii++
    
' ADCch[NCh], ADCchAVG[NCh], ADCchMAX[NCh], ADCchMIN[NCh]

' -----------------Command handling section ---------------------------------------------
' ---------------- Get next parameter from string ---------------------------------------
PRI sGetPar | j, ii, lPar
  j:=0
  Bytefill(@LastPar1,0,CmdLen)   'Clear buffer

'  ser.str(string(" In GetPar : " ))
'  ser.tx(Ch)
  repeat until Ch => "0" and Ch =< "9"  or Ch == "-" or Ch == 0    
'    ser.tx("{")
    Getch
'    ser.tx(Ch)

  if Ch == 0
'    ser.tx(">")
'    ser.str(string(" 1: Unexpected end of str! " ))
    Return -99  'error unexpected end of string
    
  If debug
    ser.str(string(" GetPar : " ))
  repeat while  Ch => "0" and Ch =< "9"  or Ch == "-"
    if Ch == 0
      If debug
        ser.tx(">")
        ser.str(string(" 2: Unexpected end of str! " ))
      Return -98  'error unexpected end of string
    if ch<>"," and j<CmdLen
'      ser.tx("|")
'      ser.dec(j)
'      ser.tx(">")
'      ser.tx(Ch)
      byte[@LastPar1][j]:=ch
      j++
'       ser.str(@LastPar1)
     Getch           'skip next
 
  ser.str(@LastPar1)
  LPar:=ser.strtodec(@LastPar1)
  ser.tx("=")
  ser.dec(lPar)
  ser.tx(" ")
Return Lpar

' ---------------- Get next character from string ---------------------------------------
Pri GetCh  'Get next character from commandstring
   If Ch > 0
     Ch:=Byte[@Data][StrP++]
'   ser.tx("\")          
'   ser.tx(ch)
'***************************************  Handle command string received from client
PRI DoCommand | CmdDone, lCmd, ParNr, Servonr, Value 'Process command string
  If Debug
    Ser.Str(string("Do command: "))

  CMDDone:=false
    Ch:=" "
 '   Ser.char(">")
 '   Ser.char(Data[StrP])
    StrP:=0
    
    repeat until Ch == "$" or StrP => DataLen          'Skip leading chars until $
 '    StrP++
      GetCh
      
    if Ch=="$"
 '    Ser.Str(string(" Cmd Begin "))
      lCmd:=sGetPar
      Ser.char(CR)
      case lCmd 'ch
        8:   DoReset       ' Reset errors 
        9:   ResetMinMax
        
        10:  ' All Batteries off
             SwitchBat(0)
             AutoBatterySwitch:=0
        11:  ' Switch Bat 1
             SwitchBat(1)
             AutoBatterySwitch:=0
        12:  ' Switch Bat 2
             SwitchBat(2)
             AutoBatterySwitch:=0
'       19:  AutoBatterySwitch:=1     ' Enable auto battery switching

        21: SwitchOn(1)
        22: SwitchOn(2)
        23: SwitchOn(3)
        24: SwitchOn(4)
        25: SwitchOn(5)
        26: SwitchOn(6)

        30: SwitchAllOff
        31: SwitchOff(1)
        32: SwitchOff(2)
        33: SwitchOff(3)
        34: SwitchOff(4)
        35: SwitchOff(5)
        36: SwitchOff(6)

        40: ReportAVG
            Ser.str(string("$41,"))
        41: ReportMAX
            Ser.str(string("$42,"))
        42: ReportMIN
            Ser.str(string("$43,"))  
        49: ReportLabels
            Ser.str(string("$49,"))
        
        90: Debug:=true
        91: Debug:=False
           
        other:
          if Debug
            Ser.Str(string(" No valid command "))
          else
            Ser.STR(string("$-99,"))
            Ser.dec(lCMD)
            Ser.tx(",")
            Ser.TX(CR)
    else
      if debug
        Ser.Str(string(" No command "))

      
DAT
   s_NTC      Byte "   NTC",0      '0  
   s_NC       Byte "    NC",0      '1
   s_Vcontrol Byte " Vcont",0      '2
   s_iAux     Byte "  IAux",0      '3              
   s_iDrive2  Byte "  IDr2",0      '4
   s_iDrive1  Byte "  IDr1",0      '5
   s_ipc2     Byte "  IPC2",0      '6              
   s_ipc1     Byte "  IPC1",0      '7
   s_iControl Byte " ICntr",0      '8              
   s_ibat1    Byte " IBat1",0      '9
   s_ibat2    Byte " IBat2",0      '10
   s_V24b     Byte "  V24b",0      '11
   s_vbat1    Byte " VBat1",0      '12                
   s_vbat2    Byte " VBat2",0      '13
   s_3V3      Byte "   3V3",0      '14  
   s_5V       Byte "    5V",0      '15

   s_AUX      Byte "AUX",0
   s_DR2      Byte "DR2",0
   s_DR1      Byte "DR1",0
   s_PC2      Byte "PC2",0
   s_PC1      Byte "PC1",0
   s_CON      Byte "CON",0
    