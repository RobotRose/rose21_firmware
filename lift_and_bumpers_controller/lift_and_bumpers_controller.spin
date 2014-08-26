''*****************************************************
'' Test multi MCP3208 for LinMot actuator
'' Opteq June 2014 HJ Kiela
'' V0: ADC, motor driver
'' Commands:
''   $0              Disable movement
''   $1, Pos         Move to position
''   $2:             Potm position control
''   $8:             Do Reset
''   $9:             Reset MinMax
''   $10:            Move to predefined retract position
''   $11             Move to predefined position 1
''   $12             Move to predefined position 2
''   $13             Move to predefined position 3
''
''   Button: to Retract position
''
'' SEE accompanied TXT file for communication specification
''
''   Outputs:
''   MoveMode
''   InPos  (-1 = in pos)
''   AllOK, s5VOK, s3V3OK, sVInOK, OutofRange   (1 = OK)
''
''   EMERIn signal enables movement
''*****************************************************


DAT 
    
CON
   ' Version
   major_version    = 3
   minor_version    = 0 
   CONTROLLER_ID    = 2

   'Set 80Mhz
   _clkmode = xtal1+pll16x
   _xinfreq = 5000000      

   ' Characters
   CR = 13
   LF = 10
   CS = 0
   CE = 11    

   '' Serial port 
   txd  = 30 '26 ' 30
   rxd  = 31 '27 ' 31
   baud = 115200

   NumWidth = 5             ' Number of positions for debug array display
      
' ADC SPI connections
   dpin1 = 4
   cpin1 = 7
   spin1 = 6
   Mode1 = 1

   NCh = 8                 'Number of measuring channels

  'ADC channels
   V5V = 0
   NTC = 1
   V3V3 = 2
   Current140 = 3
   Vin = 4
   FBPM = 5
   POTM1 = 6
   POTM2 = 7

 ' Conversion factors for Engineering value of measured ADC values
   V5VFactor = 2.0          'Divider 5V supply 1k 1k
   cADCbits2mV = 1.220703
   sCur140     =  71429      '140mV/A scale factor VNH5019 for mA
   sVin      = 6.6           'Vin scale factir
   srNTC     = 10000.0       'R NTC
   srRntc    = 10000.0       'R divider NTC
   Vref      = 5000.0        'Reference voltage ADC = full scale

'NTC 10k Farnell 1672384
AlphaNTC = -4.7
BetaNTC = 4220

' Safety related values
  c5V = 4800                 ' Minimal 5V supply
  c3V3 = 3200                ' Minimal 3V3 supply
  cVin = 9500                ' Minimal Vin supply
    
'Button Retract position
  retract_position = 400              ' Park position linear motor

' MotorDrive 1451 pin's
  sINA = 16
  sINB = 17
  sPWM = 16

' DC Motor PWM
  Freq          = 5000       ' PWM freq in Hz
  cDefHyst      = 50         ' Hysteresis for position control

  EndPosLow     = 150        ' Smallest position of lin mot
  EndPosHigh    = 3450       ' Highest position of lin mot
  
  default_speed = 150       ' Standard speed for moves

  InPosWindow = 100          ' If position error < than this value, InPos value is -1 else 0

'Button
  pButton1 = 22

'Safety I/O
  EMERin = 0   'Safety input
  OK     = 1   'Safety output relais
  IN0    = 2   'Extra input optocoupler
  OUT0   = 3   'Extra relais output

'Bumper inputs on 8 consecutive inputs
  Bumper0 = 8
  

  ' Led  
  LED = 25                   ' Propeller LED PIN

  main_led_interval = 250    ' [ms]       
  
  averaging_samples = 40     ' Number of samples to average the ADC values with    


OBJ
  ADC           : "MCP3208_fast"                       ' ADC
  ser           : "full_duplex_serial_005"             ' Full duplex serial communication 
  t             : "Timing"
  PWM           : "PWMx8"                              ' PWM module motor drive
  STRs          : "STRINGS2hk"
  num           : "simple_numbers"                     ' Number to string conversion
  f             : "FloatMath1_1"                       ' Floating point library
  rose_comm     : "rose_communication"                 ' Rose communication framework
  
VAR
  long ADCCog, PWMCog, ADCMCog, ADCStack[50], LinMotCog, LinMotStack[50], PLCCog, PLCStack[50], DebugCog, DebugStack[50], SafetyCog, SafetyStack[50], serial_cog
  long MaxCh, ADCTime, ADCCnt, DoMotorCnt, PlcCnt

  long ADCRaw[NCh], ADCRawMin[NCh], ADCRawAVG[NCh], ADCRawMax[NCh]         ' Bits
  long ADCch[NCh], ADCchAVG[NCh], ADCchMAX[NCh], ADCchMIN[NCh]             ' Volt
  long engADCch[NCh], engADCchAVG[NCh], engADCchMAX[NCh], engADCchMIN[NCh] ' Scaled values 
  long ADCConvCog
  long Scale[Nch], ScalingCnt, ScalingTime '' array with scaling factors
  
  long MotorPWM, MotorPos, Speed, MoveDir
  long PosError, Ibuf, Period
  long InPos, EndPos1, EndPos2
  
  ' Debug mode on/off
  long debug

  ' State
  long MoveMode                       '0= disable, 1: speed and pos, 2: potm 3: retract_position on button to fixed position
  byte OutofRange
  
  ' Button state
  long Button1
  
  ' Timers
  long led_interval_timer
  
  ' Safety
  byte AllOK, s5VOK, s3V3OK, sVInOK
  byte no_alarm
  long last_alarm
  
  ' Watchdog
  long received_wd
  long expected_wd 
  long wd
  long wd_cnt
  long wd_cnt_threshold

            
PUB Main | T1, NewCh
  
  Init
  debug := true
  repeat
    ' Read data from serial
    ser.StrInMaxTime(rose_comm.getStringBuffer, rose_comm.getMaxStringLength, rose_comm.getMaxWaitTime)  
      
    if(rose_comm.checkForCommand)
      if debug
        ser.str(string("Received command: "))
        displayCommand
        ser.char(CR)
      DoCommand
    elseif debug
      ser.str(string("No command received, previous command: "))
      displayCommand
      ser.char(CR)
      
    ' Indicate that the main loop is running   
    if led_interval_timer => main_led_interval
      !OUTA[Led]
      led_interval_timer := 0

PRI displayCommand | i
  ser.str(string("["))
  ser.str(rose_comm.getStringBuffer)
  ser.str(string("], "))
  ser.dec(rose_comm.getNrOfParameters)
  ser.str(string(" parameters: ["))
  i := 1
  repeat rose_comm.getNrOfParameters        
    ser.dec(rose_comm.getParameter(i++))
    if i =< rose_comm.getNrOfParameters
      ser.str(string(", "))
    
  ser.str(string("]"))
  
PRI Init
  DirA[Led] ~~                              ' Set led pin as output
  OUTA[Led] := 1                            ' Turn on the led
 
  'Reset all min/max values
  resetAllADCVals
  
  initialize_serial
  
  ' Wait for rose communication to startup
  rose_comm.initialize
  repeat while not rose_comm.isInitialized
  
  if ADCCog > 0
    cogstop(ADCCog~ - 1)
  ADCCog := ADC.Start(dpin1, cpin1, spin1,Mode1) ' Start ADC low level
  MaxCh  := NCh-1
  
  if ADCCog
    ser.str(string("Started ADCCog("))
    ser.dec(ADCCog)
    ser.str(string(")", CR))
  else
    ser.str(string("Unable to start ADCCog", CR))
    
  if DebugCog > 0
    cogstop(DebugCog~ - 1)  
  DebugCog := CogNew(DoDisplay, @DebugStack) + 1
  
  if DebugCog
    ser.str(string("Started DebugCog("))
    ser.dec(DebugCog)
    ser.str(string(")", CR))
  else
    ser.str(string("Unable to start DebugCog", CR))
   
  if ADCMCog > 0
    cogstop(ADCMCog~ - 1)
  ADCMCog:=cognew(DoADC, @ADCStack)                 ' Start Measurement cog
  
  if ADCMCog
    ser.str(string("Started ADCMCog("))
    ser.dec(ADCMCog)
    ser.str(string(")", CR))
  else
    ser.str(string("Unable to start ADCMCog", CR))
    
  if SafetyCog > 0
    cogstop(SafetyCog~ - 1)  
  SafetyCog := CogNew(DoSafety, @SafetyStack) + 1
  
  if SafetyCog
    ser.str(string("Started SafetyCog("))
    ser.dec(SafetyCog)
    ser.str(string(")", CR))
  else
    ser.str(string("Unable to start SafetyCog", CR))
    
  if PwmCog > 0
    cogstop(PwmCog~ - 1)  
  PwmCog := pwm.start(sPWM, %00000100, Freq)    ' Init pwm object
  pwm.duty(18, 0)
  
  if PwmCog
    ser.str(string("Started PwmCog("))
    ser.dec(PwmCog)
    ser.str(string(")", CR))
  else
    ser.str(string("Unable to start PwmCog", CR))

  if LinMotCog > 0
    cogstop(LinMotCog~ - 1) 
  LinMotCog := cognew(Do_Motor, @LinMotStack)       ' Start Lin motor controller
                                                   
  if LinMotCog
    ser.str(string("Started LinMotCog("))
    ser.dec(LinMotCog)
    ser.str(string(")", CR))
  else
    ser.str(string("Unable to start LinMotCog", CR))
    
  ' Set default values 
  debug     := false
  MotorPos  := ADCRawAVG[FBPM]                      ' Make set position equal to actual position
  sVinOK    := 1
  s3V3OK    := 1
  s5VOK     := 1
  AllOK     := 1

'=== Init Watchdog ===
PRI InitWatchDog
  ' Error tresholds (timing 1 count is 1*ms) default values
  wd_cnt_threshold  := 1000 
  received_wd       := 0 
  expected_wd       := 0                     
  wd                := 0
  wd_cnt            := 0

PRI handleWatchdogError | i
  ' Do some handling man!
  ' 
' === Init serial communication ===
PUB initialize_serial

  serial_cog := ser.start(rxd, txd, 0, baud)     'serial port on prop plug
  
  if serial_cog
    ser.str(string("Started serial communication, cog: "))
    ser.dec(serial_cog)
    ser.str(string(".", CR))
  else
    ser.str(string("Unable to start serial communication.", CR))

  return serial_cog  
  
' --------------------------------- Do logic tasks and safety tasks
PRI DoPLC | t1
  t1 := cnt
  Repeat
   ' Forward emergency   
    If GetEMERin==1 and sVinOK==1 and AllOK==1 ' Process emergency alarms to OK output
      SetOK(1)
    else
      SetOK(0)
      AllOK:=0      
  
    ' Read the button input state
    Button1 := GetButton
    if Button1 == 1
      AllOK     := 1              ' Try to reset drive
      sVinOK    := 1
      s3V3OK    := 1
      s5VOK     := 1
      Speed     := default_speed  ' Set speed
      MoveMode  := 3              ' To rectractposition
      MotorPos  := retract_position


    case MoveMode
      2: MotorPos := (ADCRawAVG[Potm1])               'Move mode 2: Position controlled by Potm 1

    ' Timers
    if ||(cnt - t1) => (clkfreq/1000)
      led_interval_timer++
      t1 := cnt
    PlcCnt++

PRI resetSafety 
  no_alarm                 := true                'Reset global alarm var
  last_alarm               := 0                   'Reset last alarm message

' ---------------- Check safety of platform and put in safe condition when needed ---------
PRI DoSafety | i, ConnectionError, bitvalue
  resetSafety

  ' Main safety loop    
  repeat
    '-- Watchdog -- 
    wd_cnt++

    if wd_cnt > wd_cnt_threshold    
      last_alarm := 1
      no_alarm   := false
      handleWatchdogError
      
    ' Voltages   
    if engADCchAVG[V5V] < c5V
      s5VOK :=0

    if engADCchAVG[V3V3] < c3V3
      s3V3OK :=0

    if engADCchAVG[Vin] < cVin
      sVinOK :=0
      
    t.Pause1ms(1)

' === Reset AllMin Max values of specific ADC channel === 
PRI resetAllADCVals | ii
    ii:=0
    repeat NCh 
        resetADCVals(ii++)
    
' === Reset Min Max values of specific ADC channel === 
PRI resetADCVals(i)
    if i => 0 and i =< NCh
        ADCRawMIN[i]:=999999
        ADCRawMAX[i]:=0
        ADCRawAVG[i]:=0
        return i
    else 
        return -1 
               
'Measuring cog. Runs continuously
PRI DoADC  | i, T1
   repeat
     T1:=cnt

     i:=0
     repeat NCh
       ADCRaw[i]:= ADC.in(i)' * 5000 / 4095'  + 55 'Calculate mV
'      ADCRawAVG[i]:= ADC.average(i,1,100)  * 5000 / 4096 'Calculate mV
       ADCRawAVG[i]:= ADCRawAVG[i] *(averaging_samples - 1) + ADCRaw[i] / (averaging_samples)
       ADCRawMax[i] #>= ADCRaw[i]               'save max value
       ADCRawMIN[i] <#= ADCRaw[i]            'save min value
       i++
       
     DoADCScaling

     ADCCnt++
        
     ADCTime:=(CNT-T1)/80

PRI DoADCScaling | T1, i
  ' Fill calibration table
  Scale[0]:= V5VFactor          ' 5V
  Scale[1]:= 1.0                ' NTC
  Scale[2]:= 1.0                ' 3V3
  Scale[3]:= sCur140            ' Cur140
  Scale[4]:= sVin               ' Vin
  Scale[5]:= 1.0                ' FBPM
  Scale[6]:= 1.0                ' POTM1
  Scale[7]:= 1.0                ' POTM2
  
  Repeat
    T1:=cnt

    i:=0
    Repeat NCh
      ADCch[i]:= f.fround(f.fmul(f.Ffloat(ADCRaw[i]), cADCbits2mV))       'Calculate mV

      ADCchAVG[i]:= f.fround(f.fmul(f.Ffloat(ADCRawAVG[i]), cADCbits2mV)) 'Calculate mV
      ADCchMax[i]:= f.fround(f.fmul(f.Ffloat(ADCRawMax[i]), cADCbits2mV)) 'Calculate mV
      ADCchMin[i]:= f.fround(f.fmul(f.Ffloat(ADCRawMin[i]), cADCbits2mV)) 'Calculate mV

      case i     ' Scaling to integer engineering units (mV and mA). 
        1: 'NTC
          engADCchAVG[i]:= f.fround(f.fmul(Calc_R_NTC(f.ffloat(ADCchAVG[i])), Scale[i])) 'Calculate AVG mV
          engADCchMAX[i]:= f.fround(f.fmul(Calc_R_NTC(f.ffloat(ADCchMAX[i])), Scale[i])) 'Calculate MAX mV
          engADCchMIN[i]:= f.fround(f.fmul(Calc_R_NTC(f.ffloat(ADCchMIN[i])), Scale[i])) 'Calculate MIN mV

        Other:
          engADCchAVG[i]:= f.fround(f.fmul(f.Ffloat(ADCchAVG[i]), Scale[i]))  'Calculate AVG mV
          engADCchMAX[i]:= f.fround(f.fmul(f.Ffloat(ADCchMAX[i]), Scale[i]))  'Calculate MAX mV
          engADCchMIN[i]:= f.fround(f.fmul(f.Ffloat(ADCchMIN[i]), Scale[i]))  'Calculate MIN mV  
      i++
      
    ScalingCnt++
    ScalingTime:=(CNT-T1)/80

' === Handle command string received from client ===
PRI DoCommand | command

    command := rose_comm.getCommand

    Case command
        '== Default 100 range communication ===
        '--- Communicate controller id ---
        100 : ser.str(string("$100,"))
              ser.dec(CONTROLLER_ID)
              ser.tx(",")  
              ser.tx(CR) 
        '--- Communicate software version ---
        101 : ser.str(string("$101,"))
              ser.dec(major_version)
              ser.tx(",")  
              ser.dec(minor_version)
              ser.tx(",")  
              ser.tx(CR) 
        '--- WATCHDOG ---
        111: received_wd := rose_comm.getParameter(1)
             ' Check value
             if received_wd <> expected_wd
                handleWatchdogError
                ser.tx("$")
                ser.dec(111)
                ser.tx(",")
                ser.dec(-1)
                ser.tx(",")  
                ser.dec(wd_cnt)
                ser.tx(",")            
                ser.dec(received_wd)
                ser.tx(",")   
                ser.dec(expected_wd)
                ser.tx(",")   
                ser.tx(CR)  
             else    
                ser.tx("$")
                ser.dec(111)
                ser.tx(",")
                ser.dec(wd)
                ser.tx(",")  
                ser.dec(wd_cnt)
                ser.tx(",")                
                ser.dec(received_wd)
                ser.tx(",")   
                ser.dec(expected_wd)
                ser.tx(",")   
                ser.tx(CR)  

                if expected_wd == 1
                   expected_wd := 0             
                else
                   expected_wd := 1

                if wd == 1
                   wd := 0             
                else
                   wd := 1                                 
 
                'Reset the watchdog counter
                wd_cnt := 0 
                
       {{ 200: ser.str(rose_comm.getCommandStr(command))
             ser.str(rose_comm.getBoolStr(rose_comm.nrOfParametersCorrect(2)))
             
             if rose_comm.nrOfParametersCorrect(2)  
              ser.str(rose_comm.getDecStr(rose_comm.getParameter(1)))  
              ser.str(rose_comm.getDecStr(rose_comm.getParameter(2)))
            ser.str(rose_comm.getEOLStr)
}}
                
'***************************************  Handle command string received from client
{{PRI DoCommand | CmdDone, lCmd, ParNr, Servonr, Value 'Process command string
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
        0:  ' Ser.Str(string(" Disable "))        'Disable
             MoveMode:=0
             DoReset                              'And Reset unit

        1:   Ser.Str(string(" Speed, Pos "))      'Set speed and position
             Speed:=sgetPar                       'Speed
'             Speed:= default_speed
             MotorPos:=sgetPar                    'Dir
             MoveMode:=1
             ser.dec(Speed)                       'Set new Speed
             ser.tx(" ")
             ser.dec(MotorPos)
             
        2: MoveMode:=2                            'Set move mode 2. Position controlled by Potm1
           Speed:=default_speed                             'At speed
           
        8: Doreset
           
        9: ResetMinMax
         
        10: Speed := default_speed ' Set speed
            MotorPos:=retract_position      ' Move to retract_position
            MoveMode:=10
        11: Speed := default_speed ' Set speed
            MotorPos:=PrePos1      ' Move to Pos 1
            MoveMode:=11
        12: Speed := default_speed ' Set speed
            MotorPos:=PrePos2      ' Move to Pos 2
            MoveMode:=12
        13: Speed := default_speed ' Set speed
            MotorPos:=PrePos3      ' Move to Pos 3
            MoveMode:=13
           
        other:
          Ser.Str(string(" No valid command "))

    else
      Ser.Str(string(" No command "))
}}
' ------------------------ Show debug output -----------------------------
PRI DoDisplay | i
  if debug
    ser.position(0,0)

 '===========================================================
 '  s_V5V Byte "5V",0                    'ch0
 '  s_NTC Byte "NTC",0                   'ch1
 '  s_V3V3 Byte "3V3",0                  'ch2
 '  s_Current140 Byte "Current140",0     'ch3
 '  s_Vin Byte "Vin",0                   'ch4
 '  s_FBPM Byte "FBPM",0                 'ch5
 '  s_POTM1 Byte "POTM1",0               'ch6
 '  s_POTM2 Byte "POTM2",0               'ch7

    ser.tx(CR)
    ser.str(string("        "))
    ser.str(@s_V5V)
    ser.str(@s_NTC)
    ser.str(@s_V3V3)
    ser.str(@s_Current140)
    ser.str(@s_Vin)
    ser.str(@s_FBPM)
    ser.str(@s_Potm1)
    ser.str(@s_Potm2)
    
    ser.tx(CR)
    ser.str(string("  ADCRaw:"))
    i:=0
    repeat NCh               'Actual adc
      ser.str(num.decf(ADCRaw[i],NumWidth))
      i++
      ser.tx(" ")

    ser.tx(CE)
    ser.tx(CR)

    ser.str(string(" ADCrAVG:"))
    i:=0
    repeat NCh                             'AVG adc value 
      ser.str(num.decf(ADCRawAVG[i],NumWidth))
      i++
      ser.tx(" ")

    ser.tx(CE)


    ser.tx(CR)
    ser.str(string(" ADCrMin:"))
    i:=0
    repeat NCh                           'Max adc value
      ser.str(num.decf(ADCRawMin[i],NumWidth))
      i++
      ser.tx(" ")

    ser.tx(CR)
          
    ser.str(string(" ADCrMax:"))
    i:=0
    repeat NCh
      ser.str(num.decf(ADCRawMax[i],NumWidth))  'Max adc value 
      i++
      ser.tx(" ")


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
    ser.str(string(" DoMotor cnt: "))
    ser.dec(DoMotorCnt)   
    ser.str(string(" PLC cnt: "))
    ser.dec(PLCCnt)   
    ser.str(string(" Scaling time: "))
    ser.dec(ScalingTime)
    ser.tx(" ")
    ser.str(string(" Scaling cnt: "))
    ser.dec(ScalingCnt)
    ser.tx(cr)
    ser.tx(ce)

    ser.str(string("MotorPos: "))
    ser.dec(MotorPos)   
    ser.str(string(" PosError: "))
    ser.dec(PosError)   
    ser.str(string(" Speed: "))
    ser.dec(Speed)      
    ser.str(string(" InPos: "))
    ser.dec(InPos)      
    ser.str(string(" MotorPWM: "))
    ser.dec(MotorPWM)   
    ser.str(string(" MoveDir: "))
    ser.dec(MoveDir)
    ser.str(string(" MoveMode: "))
    ser.dec(MoveMode)   
    ser.str(string(" OutOfRange: "))
    ser.dec(OutOfRange) 
    ser.str(string(" Button1: "))
    ser.dec(Button1)
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
    ser.tx(ce)

    i:=1
    ser.str(string("Bumper : "))
    repeat NCh               'Bumpers 1 to 8
      ser.dec(GetBumper(i))
      i++
      ser.tx(" ")
    ser.tx(cr)
    ser.tx(ce)

' Motor drive controller
Con
  _1ms  = 1_000_000 / 1_000          'Divisor for 1 ms
                                     '
PRI Do_Motor | T1, ClkCycles, Hyst, AbsMotorPWM, ActDuty, lMotorPos
  OUTA[sINA]:=0      ' Set direction pins as output for PWM
  DirA[sINA] ~~
  OUTA[sINB]:=0
  DirA[sINB] ~~

  Period:=10  ' ms looptime
  ClkCycles := ((clkfreq / _1ms * Period) - 4296) #> 381    'Calculate 1 ms time unit
  Hyst:=cDefHyst
  repeat
    T1:=cnt
    
    if MotorPos < EndPosLow or MotorPos > EndPosHigh        ' Signal that commanded position is outside range
      OutofRange:=1
    else
      OutOfRange:=0
      
'    lMotorPos:= EndPosLow #> MotorPos <# EndPosHigh         ' Clip motor setpoint at min and max position
    lMotorPos:= MotorPos
    PosError:= lMotorPos-ADCRawAVG[FBPM]                    'Calculate position error based on potmeter
    InPos := || PosError < InPosWindow                      ' Check is motor is in position

    if (PosError > -Hyst-1) and (PosError < Hyst+1)
       MotorPWM:=0
       MoveDir:=0
       OUTA[sINA]:=0   
       OUTA[sINB]:=0
              
    if PosError < -Hyst
        MoveDir:=-1
        OUTA[sINA]:=0   
        OUTA[sINB]:=1
        MotorPWM:=(Speed)

    if PosError > Hyst
        MoveDir:=1
        OUTA[sINA]:=1
        OUTA[sINB]:=0   
        MotorPWM:=(Speed)
  
'   Pwm.Duty(sPWM,MotorPWM)
    MotorPWM := -Speed #> MotorPWM <# Speed

    AbsMotorPWM := ||MotorPWM         'Ramp generator
    if ActDuty < AbsMotorPWM
      ActDuty++
    
    if ActDuty > AbsMotorPWM
      ActDuty--

    if AllOK==1                      ' Move only when all is OK
      pwm.duty(18, ||ActDuty)
    else
      pwm.duty(18, 0)
      
    DoMotorCnt++


    waitcnt(ClkCycles + T1)          'Wait for designated time
  



  
   
' ------------------------------------ Calculate NTC resistance
' V is floating
PRI Calc_R_NTC(V) | Intc, Vntc, Rntc
 'V:=2500.0
 Intc := f.fDiv(V,srRntc)
 Vntc:= f.fSub(Vref,V)
 Rntc:= f.fDiv(Vntc,Intc)

'  srNTC     = 10000.0       'R NTC
'  srRntc    = 10000.0       'R divider NTC
Return Rntc'V 'Intc

' ------------------------------------ Calculate NTC Temperature based on resistance
'R = resistance as float
'NTC 10k Farnell 1672384

'AlphaNTC = -4.7
'BetaNTC = 4220

PRI Calc_T_NTC(R) | lT
  return lT

' ---------------- Get Bumper input (on=1 or off=0) n in range 1-8 ------------------------------------
PUB GetBumper(n)
  if n > 0 and n < 9
    if InA[Bumper0+n-1]==1
      Return 0
    else
      Return 1   
  else
    return -1  'Wrong bumper index
    
' ---------------- Get Button input (on=1 or off=0) ---------------------------------------
PUB GetButton
  if InA[pButton1]==1
    Return 0
  else
    Return 1   
' ---------------- Set OK output on or off (1 or 0) ---------------------------------------
PUB SetOK(Value)
  if Value==0
    DirA[OK]:=0
    OUTA[OK]:=1
  else
    DirA[OK]:=1
    OUTA[OK]:=0
' ---------------- Set OUT0 output on or off (1 or 0) ---------------------------------------
PUB SetOUT0(Value)
  if Value==0
    DirA[OUT0]:=0  
    OUTA[OUT0]:=1
  else  
    DirA[OUT0]:=1  
    OUTA[OUT0]:=0

' ---------------- Get status OK output (on=1 or off=0) ---------------------------------------
PUB GetOK
  if InA[OK]==1
    Return 0
  else
    Return 1   

' ---------------- Get status OK output (on=1 or off=0) ---------------------------------------
PUB GetOUT0
  if InA[OUT0]==1
    Return 0
  else
    Return 1   

' ---------------- Get EMERin input (on=1 or off=0) ---------------------------------------
PUB GetEMERin
Return InA[EMERin]  

' ---------------- Get IN0 input (on=1 or off=0) ---------------------------------------
PUB GetIN0
Return InA[IN0]

DAT
   s_V5V        Byte "    5V",0         'ch0
   s_NTC        Byte "   NTC",0         'ch1
   s_V3V3       Byte "   3V3",0         'ch2
   s_Current140 Byte "Cur140",0         'ch3
   s_Vin        Byte "   Vin",0         'ch4
   s_FBPM       Byte "  FBPM",0         'ch5
   s_POTM1      Byte "  POTM1",0        'ch6
   s_POTM2      Byte "  POTM2",0        'ch7
  