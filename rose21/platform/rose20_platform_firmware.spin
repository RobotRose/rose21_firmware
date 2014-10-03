'' Rose B.V. 2014
'' Author: Okke Hendriks
'' Please see TXT for communication documentation

CON
   ' Version
   CONTROLLER_ID    = 1
   major_version    = 2
   minor_version    = 1 
   

   ' Set 80Mhz
   _clkmode = xtal1+pll16x
   _xinfreq = 5000000      'MRS1

   ' Led
   Led = 27
  
   ' Serial port 
   ' DebugUSB = true    'DebugUSB true: Debug via pin 30 and 31 and Commands via Xbee on 22 and 23. False: reversed  
   cTXD     = 30 '20 '30
   cRXD     = 31 '21 '31
   cBaud    = 115200 '230400 
   CR       = 13
   LF       = 10
   CS       = 0
   CE       = 11                 ' CE: Clear to End of line
   EOT      = 4                  ' End of trainsmission
  
  ' PID constants
  nPIDLoops     = PID#PIDCnt  '8
  MotorCnt      = nPIDLoops
  nWheels       = 4
  MotorIndex    = MotorCnt - 1
  PIDCTime      = 20            ' PID Cycle time ms

  ' Safety constants
   _1ms  = 1_000_000 / 1_000    ' Divisor for 1 ms

  ' MAE3 PWM absolute encoder
  MAE0Pin   = 16                ' First pin of MAE encoder
  MAECnt    = 4                 ' Number of encoders 
  MAEOffset = 2048              ' MAE range/2 (4096/2)

  ' Eeprom
  cCheck      = 12345        ' Check value for correct restore of values.
  EptromStart = $7000        ' Free range for saving
   
  ' Led
  main_led_interval = 250

  ' Debugging
  debug = false

  ' Platform status bits
  Serialbit     = 0              ' 0= Serial Debug of 1= Serial pdebug port on
  USAlarm       = 1              ' US alarm bit: 1= object detected in range
  PingBit       = 2              ' 0= Ping off 1= Ping on
  EnableBit     = 3              ' 0= Motion DisablePfd 1= Motion enabled
  CommCntrBit   = 6              ' Xbee Timout error
  MotionBit     = 7              ' Platform moving
  FeBit         = 8              ' FE error on any axis
  CurrBit       = 9              ' Current error on any axis
  MAEBit        = 10             ' MAE encoder alarm
  NoAlarmBit    = 15             ' No alarm present   

  ' TIMERS
  nr_timers             = 6
  LED_TIMER             = 0
  WATCHDOG_TIMER        = 1 
  FOLLOWING_ERROR_TIMER = 2
  CURRENT_ERROR_TIMER   = 3
  CONN_ERROR_TIMER      = 4
  MAE_ERROR_TIMER       = 5
  
OBJ
  ser           : "full_duplex_serial_005"              ' Full duplex serial communication 
  t             : "timing"                              ' Timing functions
  MAE           : "MAE3"                                ' MAE absolute encoder object
  PID           : "PID_4A"                              ' PID contr. 8 loops. Sep. Pos and Vel feedb + I/O.
  n             : "simple_numbers"                      ' Number to string conversion
  eprom         : "eeprom"                              ' Routines for saving and reloading settings
  rose_comm     : "rose_communication"                  ' Rose communication
  timer         : "timer"
  f             : "FloatMath1_1"                        ' Floating point library
  
Var 
    ' Timers
    long timer_current[nr_timers]
    long timer_set_value[nr_timers]
    byte timer_running[nr_timers]
    long timer_cog

    ' Motors
    long PIDCog
    long Setp[MotorCnt] 'Actual position and velocity, Setpoint
    byte ActPID, SerCog
    
    ' PID Connect vars                    
    byte PIDCCog, MAECog
    long ActCurr[MotorCnt]
    long PIDConnTime
    word ConnCntr
    
    ' MAE encoder
    long MAEPos[MAECnt], MAEState, MainTime
    long MAEOffs[MAECnt]  'Offset position for 0
    word MAECntr, pMAECntr
    long MAEStack[100]

    ' Serial input
    long serial_cntr  
    long CMDi, myID, serial_time, Enabled, Lp, XbeeCog

    ' Received command variables
    long PfStatus, connection_error_byte
    long LastAlarm             
    long Ki           
    long K
    long Kp
    long Kd
    long Ilimit
    long PosScale
    long VelScale
    long VelMax
    long FeMax
    long MaxCurr 
    long FR_a_err, FL_a_err, BR_a_err, BL_a_err

    ' Platform vars
    long wSpeed[nWheels], wAngle[nWheels]
    word MainCntr
    long drive_pid_vals_set, steer_pid_vals_set        
    long global_brake_state, set_brake_state

    ' Parameters for saving and loading a config
    long StartVar, sMAEOffs[MAECnt], sK[MotorCnt], sKp[MotorCnt], sKI[MotorCnt], sILim[MotorCnt]
    long sPosScale[MotorCnt], PlatFormID, Check, EndVar

    ' Parameters for saving logdata
    long StartlVar, MaxCurrent[MotorCnt], ActErrNr, EndLVar

    ' Movement/turning hysteresis
    long start_a_err, stop_a_err, stopstart_a_err  
 
    ' Safety
    long SafetyCog, SafetyStack[50], SafetyCntr, NoAlarm, CurrError
    
    ' Watchdog
    long received_wd, expected_wd, wd
    
    
' ---------------- Main program CONTROLLER_ID---------------------------------------
PUB main | T1, lch 
  InitMain

  repeat
    T1:=cnt
    
    ' Read data from serial
    ser.StrInMaxTime(rose_comm.getStringBuffer, rose_comm.getMaxStringLength, rose_comm.getMaxWaitTime)  
      
    if(rose_comm.checkForCommand)
      DoCommand

    if Enabled                             'Move! if enabled
      Move           
       
    ''if DEBUG                            
    '  if (MainCntr//8)==0   'Dump debug info only once per 8 cycles
    '    ShowScreen

    ' Indicate that the main loop is running   
    if timer.checkAndResetTimer(LED_TIMER)
      !OUTA[Led]    
      
    MainTime := (cnt-T1)/80000
    MainCntr++

' ----------------  Init main program ---------------------------------------
PRI InitMain
  ' Initialize rose communication
  rose_comm.initialize
  repeat while not rose_comm.isInitialized
  
  DisableWheelUnits

  dira[Led]~~                            'Set I/O pin for LED to output…
  !outa[Led]                             'Toggle I/O Pin for debug

  ' Load movement schmitt start stop default values
  start_a_err       := 20
  stop_a_err        := 200
  stopstart_a_err   := start_a_err
  
  if SerCog > 0
    cogstop(SerCog~ - 1)  
  SerCog := ser.start(cRXD, cTXD, 0, cBaud)
  
  if SerCog AND debug
    ser.str(string("Started SerCog("))
    ser.dec(SerCog)
    ser.str(string(")", CR))
  elseif debug
    ser.str(string("Unable to start SerCog", CR))
      
  ResetPfStatus

'=== Init Watchdog Stuff ===
PRI InitWatchDog
  timer.setTimer(WATCHDOG_TIMER, 1000)
  timer.startTimer(WATCHDOG_TIMER)
  received_wd       := 0   
  expected_wd       := 0                     
  wd                := 0
  
PRI handleWatchdogError
  ResetBit(@PfStatus,NoAlarm)
  SetBit(@PfStatus,CurrBit)
  LastAlarm := 3
  NoAlarm   := false
  connection_error_byte := 0
  DisableWheelUnits
  
PRI resetCommunication
  InitWatchDog
  
PRI resetSafety | i
  PID.ResetCurrentStatus  
  PfStatus                 := 0
  NoAlarm                  := true                'Reset global alarm var
  LastAlarm                := 0                   'Reset last alarm message
  pMAECntr                 := MAECntr

  ResetBit(@PfStatus, USAlarm)         'Reset error bits in PfStatus
  ResetBit(@PfStatus, CommCntrBit)
  SetBit(@PfStatus, NoAlarmBit)
  repeat i from 0 to MotorCnt-1
    ResetBit(@connection_error_byte, i)
    
  timer.resetTimer(FOLLOWING_ERROR_TIMER)
  timer.resetTimer(CURRENT_ERROR_TIMER)
  timer.resetTimer(CONN_ERROR_TIMER)
  timer.resetTimer(MAE_ERROR_TIMER)
  
' ---------------- Check safety of platform and put in safe condition when needed ---------
PRI DoSafety | i, ConnectionError, bitvalue
  resetSafety

  ' Wait for PID cog to be started
  repeat while PIDCog == 0
    t.Pause1ms(10)

  ' Main safety loop    
  repeat
    ConnectionError := false
    SafetyCntr++

    if PID.GetFEAnyTrip == true
      'Indicate for which motor had the error
      repeat i from 0 to MotorCnt-1
        if PID.GetFETrip(i)
          SetBit(@connection_error_byte, i)
        else
          ResetBit(@connection_error_byte, i + 1)
      pid.ResetAllFETrip
    else
      timer.resetTimer(FOLLOWING_ERROR_TIMER)

    if timer.checkTimer(FOLLOWING_ERROR_TIMER)             
      ResetBit(@PfStatus, NoAlarmBit)
      SetBit(@PfStatus, FeBit)
      LastAlarm := 1
      NoAlarm   := false
      DisableWheelUnits

    if PID.GetAnyCurrError == true
      PID.ClearAnyCurrError
    else
      timer.resetTimer(CURRENT_ERROR_TIMER)

    if timer.checkTimer(CURRENT_ERROR_TIMER)  
      ResetBit(@PfStatus, NoAlarmBit)
      SetBit(@PfStatus, CurrBit)
      LastAlarm := 2
      CurrError := 1
      NoAlarm   := false
      'Indicate for which motor had the error
      repeat i from 0 to MotorCnt-1
        if PID.GetCurrError(i)
          SetBit(@connection_error_byte, i)
        else
          ResetBit(@connection_error_byte, i + 1)   
      DisableWheelUnits

    '-- Watchdog --
    if timer.checkTimer(WATCHDOG_TIMER)
      handleWatchdogError


    'Check for connection errors
    repeat i from 0 to MotorCnt-1
      if PID.GetConnectionError(i)
        ConnectionError := 1
        SetBit(@connection_error_byte, i)
      else
        ResetBit(@connection_error_byte, i + 1)
 
    if ConnectionError == 1
      PID.ResetConnectionErrors 'reset errors because we counted this one
    else
      timer.resetTimer(CONN_ERROR_TIMER)
         
    if timer.checkTimer(CONN_ERROR_TIMER)
      'Disable
      DisableWheelUnits     ' Probably does nothing due to connection error
      ResetBit(@PfStatus, NoAlarm)
      LastAlarm := 4
      NoAlarm   := false
    else
      ' Reset error if again able to communicate
      if LastAlarm == 4
        LastAlarm := 0
        NoAlarm   := true
        SetBit(@PfStatus, NoAlarm)

    ' Check for change of MAECntr 
    if (pMAECntr <> MAECntr)
      timer.resetTimer(MAE_ERROR_TIMER)
      pMAECntr := MAECntr
    
    ' If no change was detected for the duration of the MAE_ERROR_TIMER, an MAE error occured.
    elseif timer.checkTimer(MAE_ERROR_TIMER)
      ResetBit(@PfStatus,NoAlarmBit)
      SetBit(@PfStatus,MAEBit)
      LastAlarm := 5
      connection_error_byte := ||(pMAECntr - MAECntr )
      NoAlarm   := false
      DisableWheelUnits

' ---------------- Enable or disable the controller ---------------------------------------
PRI setControllerState(enable)
  if enable AND not( drive_pid_vals_set AND steer_pid_vals_set )
    return false
  else
    if enable AND not Enabled         ' Enable Disable platform 1 = enabled 0 = disabled
       EnableWheelUnits
    elseif not enable AND Enabled  
       DisableWheelUnits
  return true     

'------------------ Move all wheels individually --------------------------------
PRI Move | speed_margin 
  FR_a_err := wAngle[0] - pid.GetActPos(1)
  FL_a_err := wAngle[1] - pid.GetActPos(3)
  BR_a_err := wAngle[2] - pid.GetActPos(5)
  BL_a_err := wAngle[3] - pid.GetActPos(7)

  'Set speed to zero if a wheelunit is still turning
  if (FR_a_err > stopstart_a_err or FR_a_err < -stopstart_a_err) or (FL_a_err > stopstart_a_err or FL_a_err < -stopstart_a_err) or (BR_a_err > stopstart_a_err or BR_a_err < -stopstart_a_err) or (BL_a_err > stopstart_a_err or BL_a_err < -stopstart_a_err)
    Setp[0] := 0    
    Setp[2] := 0   
    Setp[4] := 0   
    Setp[6] := 0   
    'Wait for totally rotated
    stopstart_a_err := start_a_err
  else
    Setp[0] := wSpeed[0]    'Front right is 0
    Setp[2] := -wSpeed[1]   'Front left is  2
    Setp[4] := wSpeed[2]    'Back right is  4
    Setp[6] := -wSpeed[3]   'Back left is   6
    'Set rotation error stop driving value
    stopstart_a_err := stop_a_err

  'Set turn speed to zero if still some speed and speed set point is zero 
  speed_margin := 15
  if (||pid.getActVel(0) > speed_margin or ||pid.getActVel(2) > speed_margin or ||pid.getActVel(4) > speed_margin or ||pid.getActVel(6) > speed_margin) and (Setp[0] == 0 and Setp[2] == 0 and Setp[4] == 0 and Setp[6] == 0)
    Setp[1] := pid.GetActPos(1)
    Setp[3] := pid.GetActPos(3)
    Setp[5] := pid.GetActPos(5)
    Setp[7] := pid.GetActPos(7)  
  else  
    Setp[1] := wAngle[0]
    Setp[3] := wAngle[1]
    Setp[5] := wAngle[2]
    Setp[7] := wAngle[3]
{{
  'DEBUG  
  Setp[0] := wSpeed[0]    'Front right is 0
  Setp[2] := -wSpeed[1]   'Front left is  2
  Setp[4] := wSpeed[2]    'Back right is  4
  Setp[6] := -wSpeed[3]   'Back left is   6  
  Setp[1] := wAngle[0]
  Setp[3] := wAngle[1]
  Setp[5] := wAngle[2]
  Setp[7] := wAngle[3] 
 }} 
  setBrakeState(global_brake_state)  'Set active or passive brake mode depending on settable variable
   
' -------------- Enable or disable wheels depending on the requested brake_state
PRI setBrakeState(brake_state)
    ' Check if valid brake state requested
    if brake_state <> (0 #> brake_state <# 3)
      return -1
      
    if set_brake_state <> brake_state
      case brake_state
        0: ' Passive brake drive mode
          if Enabled
            EnableWheelsPassiveBrake 
            set_brake_state := brake_state         
        1: ' Active brake drive mode
          if Enabled          
            EnableWheelsActiveBrake
            set_brake_state := brake_state
        2: ' No Drive, No brake mode
          DisableWheels
          pid.BrakeWheels(0)                ' No braking
          set_brake_state := brake_state    
        3: ' No Drive, full brake mode
          DisableWheels
          pid.BrakeWheels(127)              ' Full braking
          set_brake_state := brake_state         

    return set_brake_state

' -------------- DoCommand: Get command parameters from rose communication and handle them -------------
PRI DoCommand | t1, i, command
  t1 := cnt
  command := rose_comm.getCommand  

  case command   
    ' === Default 100 range communication ===
    ' === Communicate controller id ===
    100 : ser.str(rose_comm.getCommandStr(command))
          ser.str(rose_comm.getDecStr(CONTROLLER_ID))
          ser.str(rose_comm.getEOLStr) 
    ' === Communicate software version ===
    101 : ser.str(rose_comm.getCommandStr(command))
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
    
    ' === GETTERS ===
    
    ' === Get velocity of motor with ID
    200:  ser.str(rose_comm.getCommandStr(command)) 
          if rose_comm.nrOfParametersCheck(1) AND ( rose_comm.getParam(1) > -1 AND rose_comm.getParam(1) < 8)
            ser.str(rose_comm.getDecStr(pid.GetActVel(rose_comm.getParam(1))))
          ser.str(rose_comm.getEOLStr)
          
    ' === Get position of motor with ID
    201:  ser.str(rose_comm.getCommandStr(command)) 
          if rose_comm.nrOfParametersCheck(1) AND ( rose_comm.getParam(1) > -1 AND rose_comm.getParam(1) < 8)
            if rose_comm.getParam(1) == 0 OR rose_comm.getParam(1) == 2 OR rose_comm.getParam(1) == 4 OR rose_comm.getParam(1) == 6                                                                                                                            '
              ser.str(rose_comm.getDecStr(pid.GetActEncPos(rose_comm.getParam(1))))  ' Drive motor, normal encoders         
            else
              ser.str(rose_comm.getDecStr(pid.GetActEncPos(rose_comm.getParam(1))))  ' Steer motors, absolute encoders
          ser.str(rose_comm.getEOLStr)

    ' === Get current of motor with ID
    202:  ser.str(rose_comm.getCommandStr(command)) 
          if rose_comm.nrOfParametersCheck(1) AND ( rose_comm.getParam(1) => 0 AND rose_comm.getParam(1) =< 7)
            ser.str(rose_comm.getDecStr(pid.GetActCurrent(rose_comm.getParam(1))))
          ser.str(rose_comm.getEOLStr)

    ' === Get maximally reached current of motor with ID
    203:  ser.str(rose_comm.getCommandStr(command)) 
          if rose_comm.nrOfParametersCheck(1) AND ( rose_comm.getParam(1) => 0 AND rose_comm.getParam(1) =< 7)
            ser.str(rose_comm.getDecStr(pid.GetMaxCurrent(rose_comm.getParam(1))))
          ser.str(rose_comm.getEOLStr)

    ' === Get position error of motor with ID
    204:  ser.str(rose_comm.getCommandStr(command)) 
          if rose_comm.nrOfParametersCheck(1) AND ( rose_comm.getParam(1) => 0 AND rose_comm.getParam(1) =< 7)
            ser.str(rose_comm.getDecStr( pid.GetSetp(rose_comm.getParam(1)) - pid.GetActPos(rose_comm.getParam(1)) ))
          ser.str(rose_comm.getEOLStr)

    ' === Get velocity error of motor with ID
    205:  ser.str(rose_comm.getCommandStr(command)) 
          if rose_comm.nrOfParametersCheck(1) AND ( rose_comm.getParam(1) => 0 AND rose_comm.getParam(1) =< 7)
            ser.str(rose_comm.getDecStr(pid.GetDeltaVel(rose_comm.getParam(1))))
          ser.str(rose_comm.getEOLStr)

    '=== Get PID out of motor with ID
    206:  ser.str(rose_comm.getCommandStr(command)) 
          if rose_comm.nrOfParametersCheck(1) AND ( rose_comm.getParam(1) => 0 AND rose_comm.getParam(1) =< 7)
            ser.str(rose_comm.getDecStr(pid.GetPIDOut(rose_comm.getParam(1))))
          ser.str(rose_comm.getEOLStr) 

    ' === Get P out of motor with ID
    207:  ser.str(rose_comm.getCommandStr(command)) 
          if rose_comm.nrOfParametersCheck(1) AND ( rose_comm.getParam(1) => 0 AND rose_comm.getParam(1) =< 7)
            ser.str(rose_comm.getDecStr( pid.GetP(rose_comm.getParam(1))))
          ser.str(rose_comm.getEOLStr) 

    ' === Get I out of motor with ID
    208:  ser.str(rose_comm.getCommandStr(command)) 
          if rose_comm.nrOfParametersCheck(1) AND ( rose_comm.getParam(1) => 0 AND rose_comm.getParam(1) =< 7)
            ser.str(rose_comm.getDecStr(pid.GetI(rose_comm.getParam(1))))
          ser.str(rose_comm.getEOLStr) 

    ' === Get D of motor with ID
    209:  ser.str(rose_comm.getCommandStr(command)) 
          if rose_comm.nrOfParametersCheck(1) AND ( rose_comm.getParam(1) => 0 AND rose_comm.getParam(1) =< 7)
            ser.str(rose_comm.getDecStr(pid.GetD(rose_comm.getParam(1))))
          ser.str(rose_comm.getEOLStr) 

    ' === Get MAE of motor with ID
    210:  ser.str(rose_comm.getCommandStr(command)) 
          if rose_comm.nrOfParametersCheck(1) AND ( rose_comm.getParam(1) => 0 AND rose_comm.getParam(1) =< 7)
            ser.str(rose_comm.getDecStr(pid.GetMAEpos(rose_comm.getParam(1))))
          ser.str(rose_comm.getEOLStr) 

    ' === Get FE of motor with ID
    211:  ser.str(rose_comm.getCommandStr(command)) 
          if rose_comm.nrOfParametersCheck(1) AND ( rose_comm.getParam(1) => 0 AND rose_comm.getParam(1) =< 7)
            ser.str(rose_comm.getDecStr(pid.GetFE(rose_comm.getParam(1))))
          ser.str(rose_comm.getEOLStr)       

    ' === Get AlarmState and LastAlarm number
    212:  ser.str(rose_comm.getCommandStr(command)) 
          ser.str(rose_comm.getBoolStr(!NoAlarm))
          ser.str(rose_comm.getDecStr(LastAlarm))
          ser.str(rose_comm.getDecStr(connection_error_byte))
          ser.str(rose_comm.getEOLStr) 

    ' === Get All drive motor positions number
    213:  repeat while not pid.getEncSem and (Cnt-t1)/80000 < 1000           ' Timeout in [ms] 
          ser.str(rose_comm.getCommandStr(command))
          if pid.getEncSem
            ser.str(rose_comm.getDecStr(pid.GetActEncPosDiff(0)))
            ser.str(rose_comm.getDecStr(pid.GetActEncPosDiff(2)))
            ser.str(rose_comm.getDecStr(pid.GetActEncPosDiff(4)))
            ser.str(rose_comm.getDecStr(pid.GetActEncPosDiff(6)))  
            ser.str(rose_comm.getDecStr(pid.getEncClkDiff))          
          else
            ser.str(rose_comm.getDecStr(0))
            ser.str(rose_comm.getDecStr(0))
            ser.str(rose_comm.getDecStr(0))
            ser.str(rose_comm.getDecStr(0))
            ser.str(rose_comm.getDecStr(0))
          pid.resetActEncPosSem 
          ser.str(rose_comm.getEOLStr) 

    ' === Get All status info
    ' First all drive motor positions and clk difference
    214:  repeat while not pid.getEncSem and (Cnt-t1)/80000 < 1000           ' Timeout in [ms]
          ser.str(rose_comm.getCommandStr(command))                                                                   ' 
          if pid.getEncSem
            ser.str(rose_comm.getDecStr(pid.GetActEncPosDiff(0)))
            ser.str(rose_comm.getDecStr(pid.GetActEncPosDiff(2)))
            ser.str(rose_comm.getDecStr(pid.GetActEncPosDiff(4)))
            ser.str(rose_comm.getDecStr(pid.GetActEncPosDiff(6)))
            ser.str(rose_comm.getDecStr(pid.getEncClkDiff))
          else
            ser.str(rose_comm.getDecStr(0))
            ser.str(rose_comm.getDecStr(0))
            ser.str(rose_comm.getDecStr(0))
            ser.str(rose_comm.getDecStr(0))
            ser.str(rose_comm.getDecStr(0))
          pid.resetActEncPosSem  

          'Steer motors positions, absolute encoders
          ser.str(rose_comm.getDecStr( pid.GetActPos(1)) )
          ser.str(rose_comm.getDecStr( pid.GetActPos(3)) )
          ser.str(rose_comm.getDecStr( pid.GetActPos(5)) )
          ser.str(rose_comm.getDecStr( pid.GetActPos(7)) )

          '=== Get AlarmState and LastAlarm number
          ser.str(rose_comm.getBoolStr(!NoAlarm))
          ser.str(rose_comm.getDecStr(LastAlarm))
          ser.str(rose_comm.getDecStr(connection_error_byte))
          ser.str(rose_comm.getEOLStr)
    
    ' === Get All debug status info per wheelunit
    215:  ser.str(rose_comm.getCommandStr(command))     
          repeat i from 0 to nPIDLoops - 1 step 2
            ser.str(rose_comm.getDecStr(pid.GetDeltaVel(i)))
            ser.str(rose_comm.getDecStr(pid.GetActEncPos(i+1)))
            ser.str(rose_comm.getDecStr(pid.GetPIDOut(i)))
            ser.str(rose_comm.getDecStr(pid.GetPIDOut(i+1)))
            ser.str(rose_comm.getDecStr(pid.GetP(i)))
            ser.str(rose_comm.getDecStr(pid.GetP(i+1)))
            ser.str(rose_comm.getDecStr(pid.GetI(i)))
            ser.str(rose_comm.getDecStr(pid.GetI(i+1)))
            ser.str(rose_comm.getDecStr(pid.GetD(i)))
            ser.str(rose_comm.getDecStr(pid.GetD(i+1)))
            ser.str(rose_comm.getDecStr(pid.GetFE(i)))
            ser.str(rose_comm.getDecStr(pid.GetActCurrent(i)))
            ser.str(rose_comm.getDecStr(pid.GetActCurrent(i+1)))
            ser.str(rose_comm.getDecStr(pid.GetMaxCurrent(i)))
            ser.str(rose_comm.getDecStr(pid.GetMaxCurrent(i+1)))      
          ser.str(rose_comm.getEOLStr)    
      
    ' === SETTERS ===
        
    ' === Set drive PID parameters: Ki, K, Kp, Kd, Ilimit, PosScale, VelMax, VelScale, FeMax, MaxCurr
    300: ser.str(rose_comm.getCommandStr(command)) 
         if rose_comm.nrOfParametersCheck(10)
           Ki       := rose_comm.getParam(1)
           K        := rose_comm.getParam(2)
           Kp       := rose_comm.getParam(3)
           Kd       := rose_comm.getParam(4)
           Ilimit   := rose_comm.getParam(5)
           PosScale := rose_comm.getParam(6)
           VelScale := rose_comm.getParam(7)
           VelMax   := rose_comm.getParam(8)
           FeMax    := rose_comm.getParam(9)
           MaxCurr  := rose_comm.getParam(10)
           
           SetDrivePIDPars(Ki, K, Kp, Kd, Ilimit, PosScale, VelScale, VelMax, FeMax, MaxCurr)
           
           ser.str(rose_comm.getDecStr(Ki)) 
           ser.str(rose_comm.getDecStr(K)) 
           ser.str(rose_comm.getDecStr(Kp)) 
           ser.str(rose_comm.getDecStr(Kd)) 
           ser.str(rose_comm.getDecStr(Ilimit)) 
           ser.str(rose_comm.getDecStr(PosScale)) 
           ser.str(rose_comm.getDecStr(VelScale)) 
           ser.str(rose_comm.getDecStr(VelMax)) 
           ser.str(rose_comm.getDecStr(FeMax)) 
           ser.str(rose_comm.getDecStr(MaxCurr)) 
         ser.str(rose_comm.getEOLStr)
           
         
    ' === Set steering PID parameters: Ki, K, Kp, Kd, Ilimit, PosScale, VelScale, VelMax, FeMax, MaxCurr
    301: ser.str(rose_comm.getCommandStr(command)) 
         if rose_comm.nrOfParametersCheck(10)
           Ki       := rose_comm.getParam(1)
           K        := rose_comm.getParam(2)
           Kp       := rose_comm.getParam(3)
           Kd       := rose_comm.getParam(4)
           Ilimit   := rose_comm.getParam(5)
           PosScale := rose_comm.getParam(6)
           VelScale := rose_comm.getParam(7)
           VelMax   := rose_comm.getParam(8)
           FeMax    := rose_comm.getParam(9)
           MaxCurr  := rose_comm.getParam(10)
           
           SetSteerPIDPars(Ki, K, Kp, Kd, Ilimit, PosScale, VelScale, VelMax, FeMax, MaxCurr)
           
           ser.str(rose_comm.getDecStr(Ki)) 
           ser.str(rose_comm.getDecStr(K)) 
           ser.str(rose_comm.getDecStr(Kp)) 
           ser.str(rose_comm.getDecStr(Kd)) 
           ser.str(rose_comm.getDecStr(Ilimit)) 
           ser.str(rose_comm.getDecStr(PosScale)) 
           ser.str(rose_comm.getDecStr(VelScale)) 
           ser.str(rose_comm.getDecStr(VelMax)) 
           ser.str(rose_comm.getDecStr(FeMax)) 
           ser.str(rose_comm.getDecStr(MaxCurr)) 
         ser.str(rose_comm.getEOLStr)
  ' === Set start/stop movement angle error values
    302: ser.str(rose_comm.getCommandStr(command)) 
         if rose_comm.nrOfParametersCheck(2) 
           start_a_err  := rose_comm.getParam(1)
           stop_a_err   := rose_comm.getParam(2)          
           ser.str(rose_comm.getDecStr(start_a_err)) 
           ser.str(rose_comm.getDecStr(stop_a_err)) 
         ser.str(rose_comm.getEOLStr)  
    ' === Set error tresholds
    303: ser.str(rose_comm.getCommandStr(command)) 
         if rose_comm.nrOfParametersCheck(4)
           timer.setTimer(FOLLOWING_ERROR_TIMER,  0 #> rose_comm.getParam(1))
           timer.setTimer(CURRENT_ERROR_TIMER,    0 #> rose_comm.getParam(2))
           timer.setTimer(CONN_ERROR_TIMER,       0 #> rose_comm.getParam(3))       
           timer.setTimer(MAE_ERROR_TIMER,        0 #> rose_comm.getParam(4))  
           ser.str(rose_comm.getDecStr(timer.getTimerSetValue(FOLLOWING_ERROR_TIMER))) 
           ser.str(rose_comm.getDecStr(timer.getTimerSetValue(CURRENT_ERROR_TIMER))) 
           ser.str(rose_comm.getDecStr(timer.getTimerSetValue(CONN_ERROR_TIMER)))
           ser.str(rose_comm.getDecStr(timer.getTimerSetValue(MAE_ERROR_TIMER)))
         ser.str(rose_comm.getEOLStr) 
 
    ' === ACTIONS ===
    
    ' === Enable command from PC 
    400: ser.str(rose_comm.getCommandStr(command)) 
         if rose_comm.nrOfParametersCheck(1) 
           ' If enabling but the drive or steer PID values are not yet set, send error response
           if setControllerState(rose_comm.getBoolParam(1))
             ser.str(rose_comm.getBoolStr(rose_comm.getBoolParam(1)))            
           else
             ' Error response meaning not all pid vals were set before enabling
             ser.str(rose_comm.getDecStr(3))  
         ser.str(rose_comm.getEOLStr)

    ' === Set the movement speeds and directions ===
    'Front right is 0
    'Front left is  2
    'Back right is  4
    'Back left is   6
    401: ser.str(rose_comm.getCommandStr(command)) 
         if rose_comm.nrOfParametersCheck(8) 
           
           wSpeed[0] := -PID.GetMaxVel(0)     #> rose_comm.getParam(1) <# PID.GetMaxVel(0)        
           wAngle[0] := PID.GetSetpMaxMin(1)  #> rose_comm.getParam(2) <# PID.GetSetpMaxPlus(1)
           wSpeed[1] := -PID.GetMaxVel(2)     #> rose_comm.getParam(3) <# PID.GetMaxVel(2)         
           wAngle[1] := PID.GetSetpMaxMin(3)  #> rose_comm.getParam(4) <# PID.GetSetpMaxPlus(3)     
           wSpeed[2] := -PID.GetMaxVel(4)     #> rose_comm.getParam(5) <# PID.GetMaxVel(4)          
           wAngle[2] := PID.GetSetpMaxMin(5)  #> rose_comm.getParam(6) <# PID.GetSetpMaxPlus(5)     
           wSpeed[3] := -PID.GetMaxVel(6)     #> rose_comm.getParam(7) <# PID.GetMaxVel(6)
           wAngle[3] := PID.GetSetpMaxMin(7)  #> rose_comm.getParam(8) <# PID.GetSetpMaxPlus(7)                        
         
           ser.str(rose_comm.getDecStr(wSpeed[0])) 
           ser.str(rose_comm.getDecStr(wAngle[0])) 
           ser.str(rose_comm.getDecStr(wSpeed[1])) 
           ser.str(rose_comm.getDecStr(wAngle[1]))
           ser.str(rose_comm.getDecStr(wSpeed[2])) 
           ser.str(rose_comm.getDecStr(wAngle[2])) 
           ser.str(rose_comm.getDecStr(wSpeed[3])) 
           ser.str(rose_comm.getDecStr(wAngle[3]))
         ser.str(rose_comm.getEOLStr)

  

    ' === Set active or no brake mode, 2 is default
    402: ser.str(rose_comm.getCommandStr(command)) 
         if rose_comm.nrOfParametersCheck(1) 
           global_brake_state  := rose_comm.getParam(1)         
           ser.str(rose_comm.getDecStr(setBrakeState(global_brake_state)))
         ser.str(rose_comm.getEOLStr) 

    ' === Reset comm ===
    403: initWatchdog
         ser.str(rose_comm.getCommandStr(command)) 
         ser.str(rose_comm.getEOLStr)
         
    ' === Reset alarm state ===
    404: resetSafety        
         ser.str(rose_comm.getCommandStr(command)) 
         ser.str(rose_comm.getEOLStr)
         
    ' === Reset platform ===
    405: ser.str(rose_comm.getCommandStr(command)) 
         ser.str(rose_comm.getEOLStr)
         ResetPfStatus
    ' === Unkown command    
    other:
         ser.str(rose_comm.getUnkownCommandStr)
         ser.str(rose_comm.getDecStr(command))
         ser.str(rose_comm.getEOLStr)
         
  serial_time := cnt-t1
  serial_cntr++    
  return true
  
' === Setup timers ===
PRI setupTimers
  timer.setTimer(LED_TIMER, main_led_interval)
  timer.startTimer(LED_TIMER)
  
  ' Error timers  default values
  timer.setTimer(FOLLOWING_ERROR_TIMER, 3000)
  timer.startTimer(FOLLOWING_ERROR_TIMER)
  
  timer.setTimer(CURRENT_ERROR_TIMER, 800)
  timer.startTimer(CURRENT_ERROR_TIMER)
  
  timer.setTimer(CONN_ERROR_TIMER, 4000)
  timer.startTimer(CONN_ERROR_TIMER)     
  
  timer.setTimer(MAE_ERROR_TIMER, 100)
  timer.startTimer(MAE_ERROR_TIMER)     
   
  return true

' ---------------- Reset platform -------------------------------
PRI ResetPfStatus | i
  ' Start Timer cog
  if timer_cog > 0
    timer.stop 
  timer_cog := timer.start(@timer_current, @timer_set_value, @timer_running, nr_timers)
  
  ' Wait for the timer cog to have initialized the memory
  repeat while not timer.isReady
  
  ' Setup the timers
  setupTimers

  if debug
    if timer_cog
      ser.str(string("Started TimerCog("))
      ser.dec(timer_cog)
      ser.str(string(")", CR))
    else
      ser.str(string("Unable to start TimerCog", CR))
      
  if MAECog > 0
    cogstop(MAECog~ - 1)
    t.Pause1ms(1)
  MAECog := CogNew(MAESense, @MAEStack) + 1                  'Start MAE sensing

  if debug
    if MAECog
      ser.str(string("Started MAECog("))
      ser.dec(MAECog)
      ser.str(string(")", CR))
    else
      ser.str(string("Unable to start MAECog", CR))
      
  if PIDcog > 0
    pid.ResetCurrentStatus
    pid.ClearErrors
    PID.Stop
    t.Pause1ms(1)
  PIDCog  := PID.Start(PIDCTime, @Setp, @MAEPos, @MAEOffs, nPIDLoops) 
  
  ' Wait while PID initializes
  repeat while PID.GetPIDStatus <> 3                    

  if debug
    if PIDCog
      ser.str(string("Started PIDCog("))
      ser.dec(PIDCog)
      ser.str(string(")", CR))
    else
      ser.str(string("Unable to start PIDCog", CR))



  if SafetyCog > 0
    cogstop(SafetyCog~ - 1)  
    t.Pause1ms(1)
  SafetyCog := CogNew(DoSafety, @SafetyStack) + 1
  
  if debug
    if SafetyCog
      ser.str(string("Started SafetyCog("))
      ser.dec(SafetyCog)
      ser.str(string(")", CR))
    else
      ser.str(string("Unable to start SafetyCog", CR))

  repeat i from 0 to MotorCnt-1
    Setp[i] := 0

  wSpeed[0] := 0
  wSpeed[1] := 0
  wSpeed[2] := 0
  wSpeed[3] := 0
  wAngle[0] := pid.GetActPos(1)
  wAngle[1] := pid.GetActPos(3)
  wAngle[2] := pid.GetActPos(5)
  wAngle[3] := pid.GetActPos(7)

  drive_pid_vals_set := false
  steer_pid_vals_set := false  

  InitWatchDog  

  global_brake_state := 2               ' Default to no drive, no brake mode
  setBrakeState(global_brake_state)  
  
  resetSafety
                                         
' ----------------  Clear FE trip ---------------------------------------
PRI ResetFE | i 
  PID.ResetAllFETrip
  ResetBit(@PfStatus,FEbit)
  
' ----------------  Enable wheeluntis  ---------------------------------------
PRI EnableWheelUnits
    EnableSteerActiveBrake    
    setBrakeState(0)                     ' Drive, passive braking, mode      
    SetBit(@PfStatus,EnableBit)
    Enabled:=true

' ----------------  Disable all wheels and steering ---------------------------------------
PRI DisableWheelUnits   
  global_brake_state := 2
  setBrakeState(global_brake_state)      ' Set to no drive, no brake mode
  DisableSteer
  ResetBit(@PfStatus,EnableBit)
  wSpeed[0] := 0
  wSpeed[1] := 0
  wSpeed[2] := 0
  wSpeed[3] := 0
  Move
  Enabled:=false

' ----------------  Enable steer  ---------------------------------------
PRI EnableSteer
  PID.SetPIDMode(1,2)                     'Pos loop steering delayed direction change mode
  PID.SetPIDMode(3,2)                     'Pos loop steering delayed direction change mode
  PID.SetPIDMode(5,2)                     'Pos loop steering delayed direction change mode
  PID.SetPIDMode(7,2)                     'Pos loop steering delayed direction change mode
                                          '
' ----------------  Enable steer Active brake ---------------------------------------
PRI EnableSteerActiveBrake
  PID.SetPIDMode(1,3)                     'Pos loop steering
  PID.SetPIDMode(3,3)                     'Pos loop steering
  PID.SetPIDMode(5,3)                     'Pos loop steering
  PID.SetPIDMode(7,3)                     'Pos loop steering

' ----------------  Disable steer  ---------------------------------------
PRI DisableSteer
  PID.SetPIDMode(1,0)                     'Disable, open loop
  PID.SetPIDMode(3,0)                     'Disable, open loop
  PID.SetPIDMode(5,0)                     'Disable, open loop
  PID.SetPIDMode(7,0)                     'Disable, open loop

' ----------------  Enable passive brake mode ---------------------------------------
PRI EnableWheelsPassiveBrake
  pid.BrakeWheels(0)
  PID.SetPIDMode(0,2)                     'Enable vel wheel passive brake mode
  PID.SetPIDMode(2,2)                     'Enable vel wheel passive brake mode
  PID.SetPIDMode(4,2)                     'Enable vel wheel passive brake mode
  PID.SetPIDMode(6,2)                     'Enable vel wheel passive brake mode

' ----------------  Enable wheels active brake mode ---------------------------------------
PRI EnableWheelsActiveBrake
  pid.BrakeWheels(0)
  PID.SetPIDMode(0,1)                     'Enable vel wheel active brake mode
  PID.SetPIDMode(2,1)                     'Enable vel wheel active brake mode
  PID.SetPIDMode(4,1)                     'Enable vel wheel active brake mode
  PID.SetPIDMode(6,1)                     'Enable vel wheel active brake mode

' ----------------  Disable wheels  ---------------------------------------
PRI DisableWheels
  'PID.SetPIDMode(0,0)                     'Disable, open loop
  PID.SetPIDMode(2,0)                     'Disable, open loop
  PID.SetPIDMode(4,0)                     'Disable, open loop
  PID.SetPIDMode(6,0)                     'Disable, open loop

' === Set drive motor control parameters ===
PRI SetDrivePIDPars(lKi, lK, lKp, lKd, lIlimit, lPosScale, lVelScale, lVelMax, lFeMax, lMaxCurr) | i
    repeat i from 0 to MotorCnt-1 step 2         
      PID.SetKi(i,lKi)
      PID.SetK(i,lK)
      PID.SetKp(i,lKp)
      PID.SetKd(i,Kd)
      PID.SetIlimit(i,lIlimit)
      PID.SetPosScale(i,lPosScale)
      PID.SetVelScale(i,lVelScale)
      PID.SetMaxVel(i,lVelMax)
      PID.SetFeMax(i,lFeMax)
      PID.SetMaxCurr(i,lMaxCurr)       
      drive_pid_vals_set:=true

' === Set steer motor control parameters ===
PRI SetSteerPIDPars(lKi, lK, lKp, lKd, lIlimit, lPosScale, lVelScale, lVelMax, lFeMax, lMaxCurr) | i
    repeat i from 1 to MotorCnt-1 step 2         
      PID.SetKi(i,lKi)
      PID.SetK(i,lK)
      PID.SetKp(i,lKp)
      PID.SetKd(i,Kd)
      PID.SetIlimit(i,lIlimit)
      PID.SetPosScale(i,lPosScale)
      PID.SetVelScale(i,lVelScale)
      PID.SetMaxVel(i,lVelMax)
      PID.SetFeMax(i,lFeMax)
      PID.SetMaxCurr(i,lMaxCurr)  

      setRotationLimits
      MAEOffs[0] := MAEOffset
      MAEOffs[1] := MAEOffset
      MAEOffs[2] := MAEOffset
      MAEOffs[3] := MAEOffset  
      steer_pid_vals_set := true

'---------------- 'Set rotation limits to steer motors to about +-(3/4)*pi --------
PRI setRotationLimits | i
  repeat i from 1 to MotorCnt-1 step 2                  
    PID.SetSetpMaxMin(i, -MAEOffset/2)
    PID.SetSetpMaxPlus(i, MAEOffset/2)                                  

' ----------------  MAE absolute encoder sense ---------------------------------------
PRI MAESense | t1 , lMAE0Pin, lMAE1Pin, lMAE2Pin, lMAE3Pin
  lMAE0Pin:= MAE0Pin
  lMAE1Pin:= MAE0Pin + 1
  lMAE2Pin:= MAE0Pin + 2
  lMAE3Pin:= MAE0Pin + 3
  
  Repeat
    MAEPos[0]:= MAE.Pos(lMAE0Pin)
    MAEPos[1]:= MAE.Pos(lMAE1Pin)
    MAEPos[2]:= MAE.Pos(lMAE2Pin)
    MAEPos[3]:= MAE.Pos(lMAE3Pin)
    
    MAECntr++                      'Update alive counter
    
' --------------------- Return PID Time in us -----------------------------
PUB GetPIDCTime
  return PIDConnTime/80

' ----------------  Show P    ID parameters ---------------------------------------
PRI ShowParameters | i
   if SerCog > -1
      ser.Position(0,0)
      ser.str(string("Rose 2.0 Drive Firmware - Rose B.V. - 2014"))
      ser.dec(major_version)
      ser.Str(string("."))
      ser.tx(minor_version)
      ser.str(string(" Max Cog : "))
      ser.dec(pid.GetQIKCog)
    
      ser.tx(CR)
      ser.str(string(" PID pars: PID Cycle Time (ms): "))
      ser.dec(PIDCTime)
      
      ser.str(string(CR,"Serial Cog "))
      ser.dec(SerCog)
      ser.str(string(" Encoder Cog "))
      ser.dec(pid.GetEncCog)
      ser.str(string(" Xbee Cog "))
      ser.dec(XbeeCog)
      ser.str(string(" PID Cog "))
      ser.dec(PIDCog)
      ser.str(string(" MAE Cog "))
      ser.dec(MAECog)
      ser.str(string(" PID Con Cog "))
      ser.dec(PIDCCog)
      ser.str(string(" QiK Cog "))
      ser.dec(pid.GetQIKCog)
      ser.str(string( "  # PID loops "))
      ser.dec(MotorCnt)
    
      ser.str(string(CR, "Control parameters: "))
      ser.str(string( "Actual PID loop : "))
      ser.dec(ActPID)
    
      ser.str(string(CR,"      # : "))
      repeat i from 0 to MotorIndex
        ser.str(n.decf(i,6))
        ser.tx("|")
      ser.str(string(CR,"      KI: "))
      repeat i from 0 to MotorIndex
        ser.str(n.decf(PID.getKI(i),6))
        ser.tx("|")
      ser.str(string(CR,"       K: "))
      repeat i from 0 to MotorIndex
        ser.str(n.decf(PID.getK(i),6))
        ser.tx("|")
      ser.str(string(CR,"      Kp: "))
      repeat i from 0 to MotorIndex
        ser.str(n.decf(PID.getKp(i),6))
        ser.tx("|")
      ser.str(string(CR,"PosScale: "))
      repeat i from 0 to MotorIndex
        ser.str(n.decf(pid.GetPosScale(i),6))
        ser.tx("|")
      ser.str(string(CR,"VelScale: "))
      repeat i from 0 to MotorIndex
        ser.str(n.decf(pid.GetVelScale(i),6))
        ser.tx("|")
      ser.str(string(CR,"  Ilimit: "))
      repeat i from 0 to MotorIndex
        ser.str(n.decf(pid.GetIlimit(i),6))
        ser.tx("|")
    
      ser.str(string(CR,"PID Mode: "))
      repeat i from 0 to MotorIndex
        ser.str(n.decf(PID.GetPIDMode(i),6))
        ser.tx("|")
    
      ser.str(string(CR,"MAE Offs: "))
      repeat i from 0 to MAECnt-1
        ser.str(n.decf(MAEOffs[i],6))
        ser.tx("|")
    
      ser.str(string(CR,"FE limit: "))
      repeat i from 0 to MotorIndex
        ser.str(n.decf(PID.GetFEMax(i),6))
        ser.tx("|")
    
      ser.str(string(CR,"Curr lim: "))
      repeat i from 0 to MotorIndex
        ser.str(n.decf(pid.GetMaxCurrent(i),6))
        ser.tx("|")
      ser.str(string(CR,"======================================================================="))  

' ----------------  Show actual value screen ---------------------------------------
PRI ShowScreen | i
  if SerCog > -1

    ser.str(string(CR,"    Setp: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(Setp[i],8))
      ser.tx("|")

    ser.str(string(CR,"     MAE: |"))
    repeat i from 0 to MAECnt-1
      ser.str(n.decf(pid.GetMAEpos(i),8))
      ser.tx("|")
                                              
    ser.str(string(CR,"ActEncPs: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(pid.GetActEncPos(i),8))
      ser.tx("|")
    ser.str(string(CR,"  ActPos: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(pid.GetActPos(i),8))
      ser.tx("|")
    ser.str(string(CR," PidSetP: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(Pid.GetSetP(i),8))
      ser.tx("|")
      
    ser.str(string(CR,"     Vel: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetActVel(i),8))
      ser.tx("|")
    ser.str(string(CR,"DeltaVel: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetDeltaVel(i),8))
      ser.tx("|")
    ser.str(string(CR," MSetVel: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetSetVel(i),8))
      ser.tx("|")
    ser.str(string(CR,"    Ibuf: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetI(i),8))
      ser.tx("|")
    ser.str(string(CR," PID Out: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetPIDOut(i),8))
      ser.tx("|")
    ser.str(string(CR," SetpM.+: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetSetpMaxPlus(i),8))
      ser.tx("|")
    ser.str(string(CR," SetpM.-: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetSetpMaxMin(i),8))
      ser.tx("|")    
    ser.str(string(CR," Current: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(pid.GetActCurrent(i),8))
      ser.tx("|")
    ser.str(string(CR,"DriveErr: | "))
    repeat i from 0 to MotorIndex
      ser.hex(pid.GetError(i),2)
      ser.str(string("   | "))
    ser.str(string(CR,"MaxStCurr:|"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(pid.GetMaxSetCurrent(i),8))
      ser.str(string("|"))
    ser.str(string(CR,"MaxCurr : |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(pid.GetMaxCurrent(i),8))
      ser.str(string("|"))
    ser.str(string(CR,"Fe      : |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetFE(i),8))
      ser.str(string("|"))

    ser.str(string(CR,"FeTrip  : |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetFETrip(i),8))
      ser.str(string("|"))
    ser.str(n.decf(PID.GetFEAnyTrip,8))
    ser.str(string(CR,"Cur Trip: |"))                  
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetCurrError(i),8))
      ser.str(string("|"))
    ser.str(n.decf(pid.getAnyCurrError,8))
      
    ser.str(string(CR,CR,"PIDTime (ms): "))
    ser.dec(PID.GetPIDTime)
    ser.str(string("  PIDLeadTime (ms): "))
    ser.dec(PID.GetPIDLeadTime)
    ser.str(string("  PID WaitTime (ms): "))
    ser.dec(PID.GetPIDWaitTime)
    ser.str(string("  PID LoopTime (ms): "))
    ser.dec(PIDCTime)
    ser.str(string("  clkfreq: "))
    ser.dec(clkfreq)
    ser.str(string(CR,"  Main loop Time (ms): "))
    ser.dec(MainTime)
    ser.str(string("  Enc cntr: "))
    ser.dec(pid.GetEncCntr)
    ser.str(string("  SafetyCntr: "))
    ser.dec(SafetyCntr)
    ser.tx(ser#CE)

    ser.str(string(" Conn Cntr: "))
    ser.dec(ConnCntr)
    ser.str(string(" PID Cntr: "))
    ser.dec(PID.GetCntr)
    ser.str(string(" MAE Cntr: "))
    ser.dec(MAECntr)
    ser.str(string(CR," PID Mode: "))
    ser.dec(PID.GetPIDMode(0))
    ser.str(string(", "))
    ser.dec(PID.GetPIDMode(1))
    ser.str(string(" LastErr: "))
    ser.hex(PID.GetError(0), 2)
    ser.str(string(", "))
    ser.hex(PID.GetError(1), 2)
    ser.str(string(", "))
    ser.hex(PID.GetError(2), 2)
    ser.str(string(", "))
    ser.hex(PID.GetError(3), 2)
    ser.str(string(" ConnErr: "))
    ser.dec(PID.GetConnectionError(0))
    ser.str(string(", "))
    ser.dec(PID.GetConnectionError(1))
    ser.str(string(", "))
    ser.dec(PID.GetConnectionError(2))
    ser.str(string(", "))
    ser.dec(PID.GetConnectionError(3))
    ser.str(string(", "))
    ser.dec(PID.GetConnectionError(4))
    ser.str(string(", "))
    ser.dec(PID.GetConnectionError(5))
    ser.str(string(", "))
    ser.dec(PID.GetConnectionError(6))
    ser.str(string(", "))
    ser.dec(PID.GetConnectionError(7))
    ser.str(string(" serial Time ms: "))
    ser.dec(serial_time/80000)
    ser.str(string(" serial_cntr: "))
    ser.dec(serial_cntr)
    ser.str(string(" Last Alarm: "))
    ser.dec(LastAlarm)
    ser.str(string(" Watchdog cnt: "))
    ser.dec(timer.getTimer(WATCHDOG_TIMER))
    
    ser.str(string(CE,CR, " Enabled: "))
    ser.str(n.dec(Enabled))
    ser.str(string(" PfStatus: "))
    ser.str(n.ibin(PfStatus,16))

' ---------------- 'Set bit in 32 bit Long var -------------------------------
PRI SetBit(VarAddr,Bit) | lBit, lMask
  lBit:= 0 #> Bit <# 31    'Limit range
  lMask:= |< lBit           'Set Bit mask
  Long[VarAddr] |= lMask   'Set Bit
    

' ---------------- 'Reset bit in 32 bit Long var -------------------------------
PRI ResetBit(VarAddr,Bit) | lBit, lMask
  lBit:= 0 #> Bit <# 31    'Limit range
  lMask:= |< lBit           'Set Bit mask
  
  Long[VarAddr] &= !lMask  'Reset bit
    


{{
┌──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                                   TERMS OF USE: MIT License                                                  │                                                            
├──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation    │ 
│files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,    │
│modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software│
│is furnished to do so, subject to the following conditions:                                                                   │
│                                                                                                                              │
│The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.│
│                                                                                                                              │
│THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE          │
│WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR         │
│COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,   │
│ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                         │
└──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
}}

