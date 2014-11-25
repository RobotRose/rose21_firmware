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
''   InPos  (-1 = in pos)
''   AllVOK, s5VOK, s3V3OK, sVInOK, OutofRange   (1 = OK)
''
''   EMERIn signal enables movement
''
''  alarm_state 0 -> All OK
''  alarm_state 1 -> Watchdog error
''  alarm_state 2 -> ForceStop command received
''*****************************************************

DAT 
    
CON
  ' Version
  major_version    = 3
  minor_version    = 1 
  CONTROLLER_ID    = 2

  ' Set 80Mhz
  _clkmode = xtal1+pll16x
  _xinfreq = 5000000      

  ' Characters
  CR = 13
  LF = 10
  CS = 0
  CE = 11    

  ' Serial port 
  txd  = 30 '26 ' 30
  rxd  = 31 '27 ' 31
  baud = 115200

  NumWidth = 5              ' Number of positions for debug array display
      
  ' ADC SPI connections
  dpin1 = 4
  cpin1 = 7
  spin1 = 6
  Mode1 = 1

  NCh = 8                    ' Number of ADC measuring channels

  ' ADC channels
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
    
  ' Button Retract position
  retract_position = 2600              ' Park position linear motor

  ' MotorDrive 1451 pin's
  sINA = 16
  sINB = 17
  sPWM = 18
  PWM_BLOCK_BASE = 16

  ' DC Motor PWM
  Freq          = 5000       ' PWM freq in Hz
  cDefHyst      = 20         ' Hysteresis for position control
  
  default_speed = 128        ' Standard speed for moves

  strict_min_motor_position  = 200                      ' Smallest position of lin mot
  strict_max_motor_position  = 3400                     ' Highest position of lin mot
  strict_min_motor_speed     = 0                        ' Smallest speed of lin mot
  strict_max_motor_speed     = 255                      ' Highest speed of lin mot

  InPosWindow = 15          ' If position error < than this value, InPos value is -1 else 0

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
  
  averaging_samples = 40       ' Number of samples to average the ADC values with    
  averaging_samples_motor = 20  ' Number of samples to average the ADC values with   

  ' Timers
  nr_timers       = 2
  LED_TIMER       = 0
  WATCHDOG_TIMER  = 1
  
OBJ
  ADC           : "MCP3208_fast"                       ' ADC
  ser           : "full_duplex_serial_005"             ' Full duplex serial communication 
  t             : "Timing"
  PWM           : "PWMx8"                              ' PWM module motor drive
  STRs          : "STRINGS2hk"
  num           : "simple_numbers"                     ' Number to string conversion
  f             : "FloatMath1_1"                       ' Floating point library
  rose_comm     : "rose_communication"                 ' Rose communication framework
  timer         : "timer"
  
VAR
  ' Timers
  long timer_current[nr_timers]
  long timer_set_value[nr_timers]
  byte timer_running[nr_timers]
  
  long timer_cog, ADCCog, PWMCog, ADCStack[50], LinMotCog, LinMotStack[50], PLCCog, PLCStack[50], DebugCog, DebugStack[50], serial_cog
  long MaxCh, DoMotorCnt

  long ADCRaw[NCh], ADCRawMin[NCh], ADCRawAVG[NCh], ADCRawMax[NCh]         ' Bits
  long ADCch[NCh], ADCchAVG[NCh], ADCchMAX[NCh], ADCchMIN[NCh]             ' Volt
  long engADCch[NCh], engADCchAVG[NCh], engADCchMAX[NCh], engADCchMIN[NCh] ' Scaled values 
  long ADCConvCog
  long Scale[Nch] ' array with scaling factors
  
  ' Motor control
  long lift_motor_setpoint, MoveDir
  long motor_speed
  long min_motor_speed
  long max_motor_speed
  long min_motor_position                            ' Smallest position of lin mot
  long max_motor_position                           ' Highest position of lin mot
  long PosError, Ibuf, Period
  long InPos, EndPos1, EndPos2
  long current_duty_cycle

  ' Debug mode on/off
  long debug

  ' State
  byte OutofRange
  
  ' Button state
  long Button1
  
  ' Safety
  byte AllVOK, s5VOK, s3V3OK, sVInOK
  byte no_alarm
  long last_alarm
  
  ' Watchdog
  long received_wd, expected_wd, wd
  
  ' Controller state
  byte controller_enabled
  byte button_enable_override 

            
PUB Main | T1, NewCh
  init

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
    if timer.checkAndResetTimer(LED_TIMER)
      !OUTA[Led]

PRI displayCommand | i
  ser.str(string("["))
  ser.str(rose_comm.getStringBuffer)
  ser.str(string("], "))
  ser.dec(rose_comm.getNrOfParameters)
  ser.str(string(" parameters: ["))
  i := 1
  repeat rose_comm.getNrOfParameters        
    ser.dec(rose_comm.getParam(i++))
    if i =< rose_comm.getNrOfParameters
      ser.str(string(", "))
    
  ser.str(string("]"))

PRI setupTimers
  timer.setTimer(LED_TIMER, main_led_interval)
  timer.startTimer(LED_TIMER)
  
  return true
    
PRI Init
  DirA[Led] ~~                              ' Set led pin as output
  OUTA[Led] := 1                            ' Turn on the led

  ' Default values
  min_motor_position  := strict_min_motor_position                   ' Smallest position of lin mot
  max_motor_position  := strict_max_motor_position                   ' Highest position of lin mot
  min_motor_speed     := strict_min_motor_speed                      ' Smallest speed of lin mot
  max_motor_speed     := strict_max_motor_speed                      ' Highest speed of lin mot                                                        ' 
  debug               := false
  controller_enabled  := false
  button_enable_override := false
  
  'Reset all min/max values
  resetAllADCVals
  
  initialize_serial
  
  ' Wait for rose communication to startup
  rose_comm.initialize
  repeat while not rose_comm.isInitialized
  
  if DebugCog > 0
    cogstop(DebugCog~ - 1)  
  DebugCog := CogNew(DoDisplay, @DebugStack) + 1
  
  if DebugCog
    ser.str(string("Started DebugCog("))
    ser.dec(DebugCog)
    ser.str(string(")", CR))
  else
    ser.str(string("Unable to start DebugCog", CR))
    
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
    
 
  ' Start Timer cog
  if timer_cog > 0
    timer.stop 
  timer_cog := timer.start(@timer_current, @timer_set_value, @timer_running, nr_timers)
  
  ' Wait for the timer cog to have initialized the memory
  repeat while not timer.isReady
  
  ' Setup the timers
  setupTimers
  
  if PLCcog > 0
    cogstop(PLCcog~ - 1)  
  PLCcog := CogNew(DoPLC, @PLCstack) + 1
  
  if PLCcog
    ser.str(string("Started PLCcog("))
    ser.dec(PLCcog)
    ser.str(string(")", CR))
  else
    ser.str(string("Unable to start PLCcog", CR))
    
  if PwmCog > 0
    cogstop(PwmCog~ - 1)  
  PwmCog := pwm.start(PWM_BLOCK_BASE, %00000100, Freq)    ' Init pwm object
  pwm.duty(sPWM, 0)
  
  if PwmCog
    ser.str(string("Started PwmCog("))
    ser.dec(PwmCog)
    ser.str(string(")", CR))
  else
    ser.str(string("Unable to start PwmCog", CR))

  if LinMotCog > 0
    cogstop(LinMotCog~ - 1) 
  LinMotCog := cognew(Do_Motor, @LinMotStack) + 1       ' Start Lin motor controller
                                                   
  if LinMotCog
    ser.str(string("Started LinMotCog("))
    ser.dec(LinMotCog)
    ser.str(string(")", CR))
  else
    ser.str(string("Unable to start LinMotCog", CR))
    
  
PRI toggleControllerState(enable)
  if enable
    controller_enabled := true
  else
    controller_enabled := false
  return controller_enabled

'=== Init Watchdog ===
PRI InitWatchDog
  timer.setTimer(WATCHDOG_TIMER, 4000)
  timer.startTimer(WATCHDOG_TIMER)
  received_wd       := 0   
  expected_wd       := 0                     
  wd                := 0

PRI handleWatchdogError | i
  if last_alarm == 0
    setAlarm(1)          ' if alarm changed
    forceStopMotor
  
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

PRI resetSafety 
  no_alarm            := true                'Reset global alarm var
  last_alarm          := 0                   'Reset last alarm message
   
PRI resetVoltageSafety
  AllVOK              := true               
  sVinOK              := true
  s3V3OK              := true
  s5VOK               := true
  
PRI resetCommunication
  InitWatchDog
  resetSafety

PRI setAlarm(new_alarm_number) | changed_alarm
  changed_alarm          := new_alarm_number <> last_alarm
  no_alarm               := false
  last_alarm             := new_alarm_number  
  return changed_alarm
      
' === Do logic tasks and safety tasks ===
PRI DoPLC
  DIRA[pButton1] := 0     ' Set as input  
  DIRA[EMERin] := 0       ' Set as input  
  DIRA[IN0] := 0          ' Set as input  


  resetSafety
  resetVoltageSafety
  InitWatchDog
  
  ' Fill calibration table
  Scale[0]:= V5VFactor          ' 5V
  Scale[1]:= 1.0                ' NTC
  Scale[2]:= 1.0                ' 3V3
  Scale[3]:= sCur140            ' Cur140
  Scale[4]:= sVin               ' Vin
  Scale[5]:= 1.0                ' FBPM
  Scale[6]:= 1.0                ' POTM1
  Scale[7]:= 1.0                ' POTM2  
  
  repeat
    DoADC
    DoSafety
    
    ' Forward emergency   
    If GetEMERin AND AllVOK ' Process emergency alarms to OK output
      SetOK(true)
    else
      SetOK(false)
  
    ' Read the button input state
    Button1 := GetButton
    if Button1
      button_enable_override := true
      resetVoltageSafety                      ' Try really hard
      setMotorSpeedPercentage(50)            ' Set speed
      setMotorSetpoint(retract_position)

'Measuring cog. Rs continuously
PRI DoADC  | i
  i := 0
  repeat NCh
    case i
      5:          ' Motor pos ADC uses other moving average filter
        ADCRaw[i]    := ADC.in(i)
        ADCRawAVG[i] := (ADCRawAVG[i] *(averaging_samples_motor - 1) + ADCRaw[i]) / (averaging_samples_motor)
        ADCRawMax[i] #>= ADCRaw[i]               'save max value
        ADCRawMIN[i] <#= ADCRaw[i]               'save min value
      other:     ' Other ADC's
        ADCRaw[i]    := ADC.in(i)
        ADCRawAVG[i] := (ADCRawAVG[i] *(averaging_samples - 1) + ADCRaw[i]) / (averaging_samples)
        ADCRawMax[i] #>= ADCRaw[i]               'save max value
        ADCRawMIN[i] <#= ADCRaw[i]               'save min value
     
    ADCch[i]   := f.fround(f.fmul(f.Ffloat(ADCRaw[i]), cADCbits2mV))       'Calculate mV
    ADCchAVG[i]:= f.fround(f.fmul(f.Ffloat(ADCRawAVG[i]), cADCbits2mV)) 'Calculate mV
    ADCchMax[i]:= f.fround(f.fmul(f.Ffloat(ADCRawMax[i]), cADCbits2mV)) 'Calculate mV
    ADCchMin[i]:= f.fround(f.fmul(f.Ffloat(ADCRawMin[i]), cADCbits2mV)) 'Calculate mV

    case i     ' Scaling to integer engineering units (mV and mA). 
      1: 'NTC
        engADCchAVG[i]:= f.fround(f.fmul(Calc_R_NTC(f.ffloat(ADCch[i])), Scale[i]))    'Calculate mV
        engADCchAVG[i]:= f.fround(f.fmul(Calc_R_NTC(f.ffloat(ADCchAVG[i])), Scale[i])) 'Calculate AVG mV
        engADCchMAX[i]:= f.fround(f.fmul(Calc_R_NTC(f.ffloat(ADCchMAX[i])), Scale[i])) 'Calculate MAX mV
        engADCchMIN[i]:= f.fround(f.fmul(Calc_R_NTC(f.ffloat(ADCchMIN[i])), Scale[i])) 'Calculate MIN mV

      Other:
        engADCch[i]:= f.fround(f.fmul(f.Ffloat(ADCch[i]), Scale[i]))        'Calculate mV
        engADCchAVG[i]:= f.fround(f.fmul(f.Ffloat(ADCchAVG[i]), Scale[i]))  'Calculate AVG mV
        engADCchMAX[i]:= f.fround(f.fmul(f.Ffloat(ADCchMAX[i]), Scale[i]))  'Calculate MAX mV
        engADCchMIN[i]:= f.fround(f.fmul(f.Ffloat(ADCchMIN[i]), Scale[i]))  'Calculate MIN mV  
    i++


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

' ---------------- Check safety of platform and put in safe condition when needed ---------
PRI DoSafety | i, ConnectionError, bitvalue, prev_oneMScounter

  '-- Watchdog --
  if timer.checkTimer(WATCHDOG_TIMER)
    handleWatchdogError
    
  ' Voltages   
  if engADCchAVG[V5V] < c5V
    s5VOK  := false

  if engADCchAVG[V3V3] < c3V3
    s3V3OK := false

  if engADCchAVG[Vin] < cVin
    sVinOK := false
    
  AllVOK := s5VOK AND s3V3OK AND sVinOK
      

' === Handle command string received from client ===
PRI DoCommand | i, command

    command := rose_comm.getCommand  

    Case command
        '== Default 100 range communication ===
        ' === Default 100 range communication ===
        ' === Communicate controller id ===
        100: ser.str(rose_comm.getCommandStr(command))
             ser.str(rose_comm.getDecStr(CONTROLLER_ID))
             ser.str(rose_comm.getEOLStr) 
        
        ' === Communicate software version ===
        101: ser.str(rose_comm.getCommandStr(command))
             ser.str(rose_comm.getDecStr(major_version))
             ser.str(rose_comm.getDecStr(minor_version))
             ser.str(rose_comm.getEOLStr) 
        
        ' === WATCHDOG ===
        111: received_wd := rose_comm.getParam(1)
             ser.str(rose_comm.getCommandStr(command))
             ' Check value
             if received_wd <> expected_wd
                handleWatchdogError
                ser.str(rose_comm.getDecStr(-1))
                ser.str(rose_comm.getDecStr(timer.getTimer(WATCHDOG_TIMER)))
                ser.str(rose_comm.getDecStr(received_wd))
                ser.str(rose_comm.getDecStr(expected_wd))                
             else    
                ser.str(rose_comm.getDecStr(wd))
                ser.str(rose_comm.getDecStr(timer.getTimer(WATCHDOG_TIMER)))
                ser.str(rose_comm.getDecStr(received_wd))
                ser.str(rose_comm.getDecStr(expected_wd))
                if expected_wd == 1
                   expected_wd := 0             
                else
                   expected_wd := 1
                   
                if wd == 1
                   wd := 0             
                else
                   wd := 1                                 
             
                'Reset the watchdog timer
                timer.resetTimer(WATCHDOG_TIMER)
             ser.str(rose_comm.getEOLStr)    
             
        ' === Set watchdog timer ==         
        112: ser.str(rose_comm.getCommandStr(command))
             if rose_comm.nrOfParametersCheck(1)
               timer.setTimer(WATCHDOG_TIMER, rose_comm.getParam(1))
               ser.str(rose_comm.getDecStr( timer.getTimerSetValue(WATCHDOG_TIMER) ))
             ser.str(rose_comm.getEOLStr) 
        
        ' === Get watchdog timer ==        
        113: ser.str(rose_comm.getCommandStr(command))
             ser.str(rose_comm.getDecStr( timer.getTimerSetValue(WATCHDOG_TIMER) ))
             ser.str(rose_comm.getEOLStr) 
               
        ' Get bumper states        
        200: ser.str(rose_comm.getCommandStr(command))
             i := 1
             repeat bumper0
               ser.str(rose_comm.getBoolStr(GetBumper(i++)))
             ser.str(rose_comm.getEOLStr)
        ' Get lift motor position percentage   
        201: ser.str(rose_comm.getCommandStr(command))
             ser.str(rose_comm.getDecStr(getMotorPosPercentage)) 
             ser.str(rose_comm.getEOLStr)
        'Get safety input
        202: ser.str(rose_comm.getCommandStr(command))
             ser.str(rose_comm.getBoolStr(GetEMERin)) 
             ser.str(rose_comm.getEOLStr)
        'Get IN0 input
        203: ser.str(rose_comm.getCommandStr(command))
             ser.str(rose_comm.getDecStr(GetIN0)) 
             ser.str(rose_comm.getEOLStr)
        'Get safety state input
        204: ser.str(rose_comm.getCommandStr(command))
             ser.str(rose_comm.getBoolStr(GetEMERin)) 
             ser.str(rose_comm.getBoolStr(s5VOK)) 
             ser.str(rose_comm.getBoolStr(s3V3OK)) 
             ser.str(rose_comm.getBoolStr(sVinOK)) 
             ser.str(rose_comm.getBoolStr(button_enable_override)) 
             ser.str(rose_comm.getDecStr(last_alarm)) 
             ser.str(rose_comm.getEOLStr)
        ' Get ADC raw values
        205: ser.str(rose_comm.getCommandStr(command))
             i := 0
             repeat NCh
               ser.str(rose_comm.getDecStr(ADCRaw[i]))
               ser.str(rose_comm.getDecStr(ADCRawAVG[i]))
               ser.str(rose_comm.getDecStr(ADCRawMIN[i]))
               ser.str(rose_comm.getDecStr(ADCRawMAX[i]))
               i++
             ser.str(rose_comm.getEOLStr)
        ' Get ADC uncalibrated engineering values     
        206: ser.str(rose_comm.getCommandStr(command))
             i := 0
             repeat NCh
               ser.str(rose_comm.getDecStr(ADCch[i]))
               ser.str(rose_comm.getDecStr(ADCchAVG[i]))
               ser.str(rose_comm.getDecStr(ADCchMIN[i]))
               ser.str(rose_comm.getDecStr(ADCchMAX[i]))
               i++
             ser.str(rose_comm.getEOLStr)
        ' Get ADC engineering values     
        207: ser.str(rose_comm.getCommandStr(command))
             i := 0
             repeat NCh
               ser.str(rose_comm.getDecStr(engADCch[i]))
               ser.str(rose_comm.getDecStr(engADCchAVG[i]))
               ser.str(rose_comm.getDecStr(engADCchMIN[i]))
               ser.str(rose_comm.getDecStr(engADCchMAX[i]))
               i++
             ser.str(rose_comm.getEOLStr)
        ' Get integrated current TODO NOT IMPLEMENTED 
        208: ser.str(rose_comm.getCommandStr(command))
             repeat NCh
               ser.str(rose_comm.getDecStr(-1))
               ser.str(rose_comm.getDecStr(-1))  
             ser.str(rose_comm.getEOLStr)
        ' Get current lift motor position error
        209: ser.str(rose_comm.getCommandStr(command))
             ser.str(rose_comm.getDecStr(PosError))
             ser.str(rose_comm.getEOLStr)
        ' Get lift motor position    
        210: ser.str(rose_comm.getCommandStr(command))
             ser.str(rose_comm.getDecStr(getMotorPos)) 
             ser.str(rose_comm.getEOLStr)
        ' Is lift in position
        212: ser.str(rose_comm.getCommandStr(command))
             ser.str(rose_comm.getBoolStr(InPos))
             ser.str(rose_comm.getEOLStr)
        ' Is lift moving
        213: ser.str(rose_comm.getCommandStr(command))
             ser.str(rose_comm.getBoolStr(isMotorMoving))
             ser.str(rose_comm.getEOLStr)
        ' Is serial debug mode enabled or disabled
        214: ser.str(rose_comm.getCommandStr(command))
             ser.str(rose_comm.getBoolStr(debug))
             ser.str(rose_comm.getEOLStr)
        ' Get set_point
        215: ser.str(rose_comm.getCommandStr(command))
             ser.str(rose_comm.getDecStr(getMotorSetpoint))
             ser.str(rose_comm.getEOLStr)
        ' Get set_point percentage
        216: ser.str(rose_comm.getCommandStr(command))
             ser.str(rose_comm.getDecStr(getMotorSetpointPercentage))
             ser.str(rose_comm.getEOLStr)
        ' Get set_speed
        217: ser.str(rose_comm.getCommandStr(command))
             ser.str(rose_comm.getDecStr(getMotorSpeed))
             ser.str(rose_comm.getEOLStr)
        ' Get set_speed percentage
        218: ser.str(rose_comm.getCommandStr(command))
             ser.str(rose_comm.getDecStr(getMotorSpeedPercentage))
             ser.str(rose_comm.getEOLStr)
        
        ' === SETTERS ===
        ' Set OK output state
        300: ser.str(rose_comm.getCommandStr(command)) 
             if rose_comm.nrOfParametersCheck(1)
               setOK(rose_comm.getBoolParam(1))
               ser.str(rose_comm.getBoolStr(rose_comm.getBoolParam(1))) 
             ser.str(rose_comm.getEOLStr)
        ' Set OUT0 output state
        301: ser.str(rose_comm.getCommandStr(command)) 
             if rose_comm.nrOfParametersCheck(1)
               setOK(rose_comm.getBoolParam(1))
               ser.str(rose_comm.getBoolStr(rose_comm.getBoolParam(1))) 
             ser.str(rose_comm.getEOLStr)
        
        ' Set min/max motor position
        302: ser.str(rose_comm.getCommandStr(command)) 
             if rose_comm.nrOfParametersCheck(2)
               min_motor_position := rose_comm.getParam(1)
               max_motor_position := rose_comm.getParam(2)
               ser.str(rose_comm.getDecStr(min_motor_position)) 
               ser.str(rose_comm.getDecStr(max_motor_position + 2)) 
             ser.str(rose_comm.getEOLStr)
        ' Set min/max motor speed
        303: ser.str(rose_comm.getCommandStr(command)) 
             if rose_comm.nrOfParametersCheck(2)
               min_motor_speed := rose_comm.getParam(1)
               max_motor_speed := rose_comm.getParam(2)
               ser.str(rose_comm.getDecStr(min_motor_speed)) 
               ser.str(rose_comm.getDecStr(max_motor_speed)) 
             ser.str(rose_comm.getEOLStr)        
             
        ' === Other commands ===
        ' Enable or disable the lift controller
        400: ser.str(rose_comm.getCommandStr(command)) 
             if rose_comm.nrOfParametersCheck(1)               
               ser.str( rose_comm.getBoolStr(toggleControllerState(rose_comm.getBoolParam(1))) ) 
             ser.str(rose_comm.getEOLStr)
        ' Force motor to stop
        401: ser.str(rose_comm.getCommandStr(command)) 
             forceStopMotor
             ser.str(rose_comm.getEOLStr)     
        ' Enable/disable serial debug mode
        402: ser.str(rose_comm.getCommandStr(command)) 
             if rose_comm.nrOfParametersCheck(1)
               debug := rose_comm.getBoolParam(1)
               ser.str(rose_comm.getBoolStr(debug)) 
             ser.str(rose_comm.getEOLStr)  
        ' Move to position percentage with a certain speed percentage
        403: ser.str(rose_comm.getCommandStr(command)) 
             if rose_comm.nrOfParametersCheck(2)
               ser.str( rose_comm.getDecStr(setMotorSpeedPercentage(rose_comm.getParam(1))) )
               ser.str( rose_comm.getDecStr(setMotorSetpointPercentage(rose_comm.getParam(2))) )
             ser.str(rose_comm.getEOLStr)     
        ' Reset specific ADC its min/max values
        404: ser.str(rose_comm.getCommandStr(command)) 
             if rose_comm.nrOfParametersCheck(1)
               ser.str( rose_comm.getDecStr(resetADCVals(rose_comm.getParam(1))) )
             ser.str(rose_comm.getEOLStr)   
        ' Reset ALL ADC min/max values
        405: ser.str(rose_comm.getCommandStr(command)) 
             resetAllADCVals
             ser.str(rose_comm.getEOLStr)   
        ' Reset safety values
        406: ser.str(rose_comm.getCommandStr(command)) 
             resetSafety
             ser.str(rose_comm.getEOLStr)  
        ' Reset safety values
        407: ser.str(rose_comm.getCommandStr(command)) 
             resetCommunication
             ser.str(rose_comm.getEOLStr)        
        other:
             ser.str(rose_comm.getUnkownCommandStr)
             ser.str(rose_comm.getDecStr(command))
             ser.str(rose_comm.getEOLStr)

' === Motor drive controller ===
CON
  _1ms  = 1_000_000 / 1_000          'Divisor for 1 ms
                                     '
' Stop the motor
PUB stopMotor
  lift_motor_setpoint    := getMotorPos

' Force stop the motor
PUB forceStopMotor   
  setAlarm(2)
  button_enable_override := false
  OUTA[sINA]             := 0   
  OUTA[sINB]             := 0
  pwm.duty(sPWM, 0) 
  current_duty_cycle     := 0
  stopMotor
  
' Set setpoint by an integer in the range 0 - 100, limited to min and max values. Returns the actually set value
PUB setMotorSetpointPercentage(percentage)
   setMotorSetpoint(calcRangeValue(percentage, min_motor_position, max_motor_position))
   return getMotorSetpointPercentage

PUB getMotorSetpoint
  return lift_motor_setpoint

PUB getMotorSetpointPercentage
   return calcRangePercentage(lift_motor_setpoint, min_motor_position, max_motor_position)

' Set setpoint, limited to min and max values. Returns the actually set value
PUB setMotorSetpoint(setpoint)
  lift_motor_setpoint := min_motor_position #> setpoint <# max_motor_position
  return lift_motor_setpoint

PUB setMotorSpeedPercentage(percentage)
  setMotorSpeed( calcRangeValue(percentage, min_motor_speed, max_motor_speed) )
  return getMotorSpeedPercentage

PUB setMotorSpeed(speed)
  motor_speed := min_motor_speed #> speed <# max_motor_speed
  return getMotorSpeed 
  
PUB getMotorSpeedPercentage
  return calcRangePercentage(getMotorSpeed, min_motor_speed, max_motor_speed)
  
PUB getMotorSpeed
  return motor_speed
  
PUB getMotorPos
  return engADCchAVG[FBPM]
  
PUB getMotorPosPercentage
  return calcRangePercentage(getMotorPos, min_motor_position, max_motor_position)
  
PUB calcRangePercentage(value, minimal, maximal)
  return 0 #> ( f.fround( f.fmul( f.fdiv( f.ffloat(value - minimal), f.ffloat(maximal - minimal) ), 100.0) ) ) <# 100

PUB calcRangeValue(percentage, minimal, maximal)
  percentage := 0 #> percentage <# 100
  return f.fround( f.fdiv( f.fmul( f.ffloat(percentage), f.ffloat(maximal - minimal)), 100.0) ) + minimal

PUB isMotorMoving   
  return (current_duty_cycle > 0)
  
PRI Do_Motor | T1, T2, ClkCycles, Hyst, wanted_motor_speed, max_speed, ramp_period, ramp_period_cycles
  OUTA[sINA] := 0      ' Set direction pins as output for PWM
  DirA[sINA]~~
  OUTA[sINB] := 0
  DirA[sINB]~~
  
  stopMotor

  Period    := 10                                                          ' [ms]
  ClkCycles := ((clkfreq / _1ms * Period) - 4296) #> 381                   ' Calculate number of clockcycles in period
  ramp_period         := 20                                               ' [ms] 
  ramp_period_cycles  := ((clkfreq / _1ms * ramp_period) - 4296) #> 381    ' Calculate number of clockcycles in ramp_period
  Hyst      := cDefHyst
  T2 := cnt
  repeat
    T1 := cnt    
    
    PosError        := lift_motor_setpoint - getMotorPos               ' Calculate position error based on potmeter
    InPos           := || (PosError) < InPosWindow                     ' Check is moto  r is in position
  
    ' Is error within hysteresis
    if (PosError > -Hyst) AND (PosError < Hyst) AND MoveDir <> 0 AND current_duty_cycle < 10
      MoveDir                := 0
      current_duty_cycle     := 0       ' Always reset duty cycle when changing direction
      OUTA[sINA]             := 0   
      OUTA[sINB]             := 0
         
    ' Is error smaller than hysteresis     
    if PosError =< -Hyst AND MoveDir <> -1
      MoveDir               := -1
      current_duty_cycle    := 0        ' Always reset duty cycle when changing direction
      OUTA[sINA]            := 0   
      OUTA[sINB]            := 1
    
    ' Is error larger than hysteresis
    if PosError => Hyst AND MoveDir <> 1
      MoveDir               := 1
      current_duty_cycle    := 0        ' Always reset duty cycle when changing direction
      OUTA[sINA]            := 1
      OUTA[sINB]            := 0   
  
    max_speed := 5 + f.fround(f.fmul(f.fdiv(f.ffloat( 0 #> ((||PosError) -InPosWindow) ), f.ffloat(100)), f.ffloat(max_motor_speed)) ) 
      
    wanted_motor_speed := -max_motor_speed #> (-max_speed #> MoveDir * motor_speed <# max_speed) <# max_motor_speed
    
    
    if (no_alarm AND controller_enabled) OR ( button_enable_override AND (last_alarm == 1 OR last_alarm == 2) )   ' Controller enabled or home button pressed and not force stopped or watchdogged (alarmstate 2 or alarmstate 1)
      ' Ramp generator
      ' Runs at ramp period
      if (cnt - T2) > ramp_period_cycles
        T2 := cnt
        if current_duty_cycle < || wanted_motor_speed
          current_duty_cycle := current_duty_cycle + 5
        
        if current_duty_cycle > || wanted_motor_speed
          current_duty_cycle := current_duty_cycle - 5 ' := || wanted_motor_speed
        
      ' Set duty cycle
      'if button_enable_override   ' Move only when all is OK or button override
      pwm.duty(sPWM, || current_duty_cycle)
      'else
     '   pwm.duty(sPWM, 0)                     ' Stop motor
     '   current_duty_cycle := 0
      
    ' Check if finished with moving to button position
    if button_enable_override AND InPos AND  lift_motor_setpoint == retract_position  
        button_enable_override := false
   
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

PRI Calc_T_NTC(R) | lT        ' TODO
  return lT

' ---------------- Get Bumper input (on=true or off=false) i in range 1-8 ------------------------------------
PUB GetBumper(i)
  if i > 0 and i < 9
    if InA[Bumper0 + i - 1] == 1
      return false
    else
      return true   
  else
    return -1  'Wrong bumper index
    
' ---------------- Get Button input (on=true or off=false) ---------------------------------------
PUB GetButton
  if INA[pButton1] == 1
    return false
  else
    return true   
' ---------------- Set OK output on or off (true or false) ---------------------------------------
PUB SetOK(enable)
  if enable
    DIRA[OK]    := 0
    OUTA[OK]    := 1
  else
    DIRA[OK]    := 1
    OUTA[OK]    := 0
' ---------------- Set OUT0 output on or off (true or false) ---------------------------------------
PUB SetOUT0(enable)
  if enable
    DIRA[OUT0]  := 0  
    OUTA[OUT0]  := 1
  else  
    DIRA[OUT0]  := 1  
    OUTA[OUT0]  := 0

' ---------------- Get status OK output (on=true or off=false) ---------------------------------------
PUB GetOK
  if INA[OK] == 1
    return true
  else
    return false   

' ---------------- Get status OK output (on=true or off=false) ---------------------------------------
PUB GetOUT0
  if INA[OUT0] == 1
    return true
  else
    return false   

' ---------------- Get EMERin input (on=1 or off=0) ---------------------------------------
PUB GetEMERin
 if INA[EMERin] == 1
   return true
 else
   return false 
' ---------------- Get IN0 input (on=1 or off=0) ---------------------------------------
PUB GetIN0
 return INA[IN0]
 if INA[EMERin] == 1
   return true
 else
   return false 

' ------------------------ Show debug output -----------------------------
PRI DoDisplay |   i
  repeat 
    if debug
      t.Pause1ms(250)
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

      ser.char(CR)
      ser.str(string("WD timer: "))
      ser.dec(timer.getTimer(WATCHDOG_TIMER))
      ser.char(CR)
      
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
      ser.str(string("eADCv:"))
      repeat NCh
        ser.str(num.decf(engADCch[i],NumWidth)) 'Engineering
        i++
        ser.tx(" ")
      ser.tx(CE)
      
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
      ser.tx(" ") 
      ser.str(string(" DoMotor cnt: "))
      ser.dec(DoMotorCnt)    
      ser.tx(cr)
      ser.tx(ce)
  
      ser.str(string("lift_motor_setpoint: "))
      ser.dec(lift_motor_setpoint)   
      ser.str(string(" PosError: "))
      ser.dec(PosError)   
      ser.str(string(" Speed: "))
      ser.dec(motor_speed)      
      ser.str(string(" InPos: "))
      ser.dec(InPos)      
      ser.str(string(" motor_speed: "))
      ser.dec(motor_speed)   
      ser.str(string(" MoveDir: "))
      ser.dec(MoveDir)
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
      ser.str(string(" AllVOK: "))
      ser.dec(AllVOK)
      ser.str(string(" last_alarm: "))
      ser.dec(last_alarm)
      ser.tx(cr)
      ser.tx(ce)
  
      i:=1
      ser.str(string("Bumper : "))
      repeat NCh               'Bumpers 1 to 8
        ser.dec(GetBumper(i))
        i++
        ser.tx(" ")
      ser.tx(cr)
      
      ser.str(string("PWM duty cycle: "))
      ser.dec(current_duty_cycle)
      ser.char(CR)
      ser.tx(ce)
      
DAT
   s_V5V        Byte "    5V",0         'ch0
   s_NTC        Byte "   NTC",0         'ch1
   s_V3V3       Byte "   3V3",0         'ch2
   s_Current140 Byte "Cur140",0         'ch3
   s_Vin        Byte "   Vin",0         'ch4
   s_FBPM       Byte "  FBPM",0         'ch5
   s_POTM1      Byte "  POTM1",0        'ch6
   s_POTM2      Byte "  POTM2",0        'ch7
  