 de{=============================================================================
 Qic PID object real version for 8 motors dec/febr 2010 HJK
 Uses quadrature encoder to measure motor velocity
 Velocity/position control in 8 fold PID loop velocity and position at approx. 500 us per loop
 Set and read parameters of QiK controller
 Read PWM encoder US Digital
 V1.2 Dec 2010:
   LCD screen 2x16
   Xbee command interface
   Checks if Remote connection Joystick is alive.230400
   If no contact after 0.1 sec, platform shuts down
   Jan 2011 Board 900085a
 V2a Febr 2011:
   Table for steering and speedcommand. Stadard steering,
   cross steering and rotation around center.
   PC communication extended with movemode for steering modes
   LCD serial cong removed. Cog needed for other tasks
 Test V2b PID met FE error  Validated 1 April 2011
 V3: June 2011 String handling for commands from joystick and pc
 V31: Juli 2011 Combined PID and I/O loop in PID V4. Saves a cog for communication and safety.
 V32: Command for commanding individual motors. 
 V33: Release version okt 2011
 V33a/b: Nov 2011 Minor mod's: debug screen, Setpoints and braking
 V35: Serial communication with PC via Serial port and Debug Via standard USB port
 V36: Okke: Major overhaul, removed unneccesary code, changed communication protocol, PID loop timing and velocity calc changed
            Added watchdog
 To do: MAE time out error, xbee comm time out error
 
=============================================================================
}
{ Platform control commands:
 Uses string handling for xbee command line handling

 Xbee remote control
   XB$500,33251,-369,-88,0,0,0,0,   Id ($500),Cntr,JoyX,JoyY,Btn1,btn2,btn3,btn4

  PC Interface:
  Enable motion             :  $902,Enable   Enable : 0=no motion 1= motion allowed
  individual Wheel mode     :  $905,cnt,speed1, speed2, speed3, speed4, angle1, angle2. All angles and speeds independent
   
  Clear Errors     :  $908  (No parameters) ResetPfStatus: Reset platform status, ResetMaxCurrent, ResetCurError   
  Query wheel pos  :  $911  Returns 4 wheel positions, 4 steer positions, 4 wheel speeds,  Right Front, Left Front, RightR ear, Left Rear 
  Query Status     :  $912  Query status of various par's, like errors, platform status, life counters Xbee comm time
  Query currents   :  $914  Query actual and max currents: Return $914,actual current 0..7, max current [0..7]  
  Query PID pars   :  $916  Report PID parameters  
        
}
CON
' Version
   Version = 36
   SubVersion = "a" 
   CONTROLLER_ID = 1

   'Set 80Mhz
   _clkmode = xtal1+pll16x
   _xinfreq = 5000000      'MRS1

'  Led
   Led = 27
  
'  Serial port 
'  DebugUSB = true    'DebugUSB true: Debug via pin 30 and 31 and Commands via Xbee on 22 and 23. False: reversed  
   cTXD = 30 '20 '30
   cRXD = 31 '21 '31
   cBaud = 115200 '230400 
   CR = 13
   LF = 10
   CS = 0
   CE = 11                 'CE: Clear to End of line
   EOT = 4                 'End of trainsmission

'' LCD
  LcdPin = 24
  LCDBaud = 19200
  LCDLines = 2
  
'PID constamts
  nPIDLoops = PID#PIDCnt  '8
  MotorCnt = nPIDLoops
  nWheels = 4
  MotorIndex = MotorCnt - 1
  PIDCTime = 40            ' PID Cycle time ms

'Safety constants
   _1ms  = 1_000_000 / 1_000          'Divisor for 1 ms

'MAE3 PWM absolute encoder
  MAE0Pin  = 16            'First pin of MAE encoder
  MAECnt   = 4             'Number of encoders 

'Xbee
   cxTXD = 20 '30 '26 '23
   cxRXD = 21 '31 '25 '22
   cxBaud = 115200         ' 115200 seems reliable max
   XBID  = 400             ' Xbee Id of this Robot platform         


'Command interface and control 
   LineLen = 100           ' Buffer size for incoming line
   SenderLen= 10
   Cmdlen = 10     
   AliveTime = 100         ' Time in ms before shutdown 
   
'String buffer
  MaxStr = 257             ' Stringlength is 256 + 1 for 0 termination

'Potmeter
  Pot0  = 9
  Pcenter = 500

'Terminal screen positions
  pControlPars = 1
  pActualPars = pControlPars + 13
  pMenu       = pActualPars + 26
  pInput      = pMenu + 12

'Eeprom
 cCheck = 12345             'Check value for correct restore of values.
 EptromStart = $7000        'Free range for saving

'Error logging
  ErrorCnt = 100

'Debugging
  DEBUG = TRUE

'Platform status bits
   Serialbit     = 0              '0= Serial Debug of 1= Serial pdebug port on
   USAlarm       = 1              'US alarm bit: 1= object detected in range
   PingBit       = 2              '0= Ping off 1= Ping on
   EnableBit     = 3              '0= Motion DisablePfd 1= Motion enabled
   CommCntrBit   = 6              'Xbee Timout error
   MotionBit     = 7              'Platform moving
   FeBit         = 8              'FE error on any axis
   CurrBit       = 9              'Current error on any axis
   MAEBit        = 10             'MAE encoder alarm
   NoAlarmBit    = 15             'No alarm present   



OBJ
  ser           : "Parallax Serial Terminal"            ' Serial communication object
  xBee          : "FullDuplexSerial_rr005" '            "FullDuplexSerialPlus"       ' Xbee serial
  t             : "Timing"
  MAE           : "MAE3"                                ' MAE absolute encoder object
  PID           : "PID Connect V4a"                      ' PID contr. 8 loops. Sep. Pos and Vel feedb + I/O.
  n             : "simple_numbers"                      ' Number to string conversion
  eprom         : "Eeprom"                              ' Routines for saving and reloading settings
  
Var Long PotmValue0, SpeedCom, DoShowParameters
    Long s, ms, us
    'Motors
    Long PIDCog, PIDMode
    Long Setp[MotorCnt] 'Actual position and velocity, Setpoint
    Long EngU[MotorCnt] 'The user units for readout and commanding a servo motor
    Byte ActPID, SerCog
    Long FeError
    
    'PID Connect vars                    
    Byte PIDCCog, MAECog
    Long ActCurr[MotorCnt]
    Long PIDConnTime
    Word ConnCntr
    
    'MAE encoder
    Long MAEPos[MAECnt], MAEState, MAETime, MainTime
    Long MAEOffs[MAECnt]  'Offset position for 0
    Word MAECntr, pMAECntr
    Long MAEStack[100]

    'Xbee input
    Long XbeeCmdCntr  
    Long Sender, CMDi, myID, XbeeTime, Enabled, XbeeStat, Lp, XbeeCog
    Byte Cmd[LineLen] ,LastPar1[CmdLen]
    Byte XbeeTimeout

    'Input string handling
    Byte StrBuf[MaxStr], cStrBuf[MaxStr]      'String buffer for receiving chars from serial port
    Long StrSP, StrCnt, StrP, StrLen  'Instring semaphore, counter, Stringpointer
    Long SerEnabled, oSel0, CmdDone
    Long MaxWaitTime  'Wait time for new Xbee string

    'Safety
    Long SafetyCog, SafetyStack[50], SafetyCntr, NoAlarm, CurrError, expected_wd, wd, wd_cnt

    'Received command variables
    Byte pcCommand, LastAlarm, PfStatus
    Long pcSpeed, pcDirection, pcCntr
    Long do_enable                  
    Long Ki           
    Long K
    Long Kp
    Long Ilimit
    Long PosScale
    Long VelScale
    Long VelMax
    Long FeMax
    Long MaxCurr 
    long FR_a_err, FL_a_err, BR_a_err, BL_a_err
    'Platform vars
    Long MoveSpeed, MoveDir, lMoveSpeed, lMoveDir, MoveMode, A1, A2, Rv
    Long wSpeed[nWheels], wAngle[nWheels]
    Word MainCntr
    Long drive_pid_vals_set, steer_pid_vals_set        
    'Parameters for saving and loading a config
    Long StartVar, sMAEOffs[MAECnt], sK[MotorCnt], sKp[MotorCnt], sKI[MotorCnt], sILim[MotorCnt]
    Long sPosScale[MotorCnt], PlatFormID, Check, EndVar

    'Parameters for saving logdata
    Long StartlVar, MaxCurrent[MotorCnt], ErrorLog[ErrorCnt], ActErrNr, EndLVar

    'Movement 
    Long start_a_err, stop_a_err, stopstart_a_err     
' ---------------- Main program CONTROLLER_ID---------------------------------------
PUB main | Up, T1, lch 
  InitMain
  !outa[Led]                             ' Toggle I/O Pin for debug
  Up:=1
  PIDMode:=0
  Disable

  repeat
    T1:=cnt
    if DEBUG
      lch:= ser.RxCheck                     ' Check serial port debug port
      if lch>0                                            
        DoCommand(lch)
    lch:=0
    DoXbeeCmd                              'Linux pc roboto controller runtime com
    
    if Enabled                             'Move! if enabled
        Move           
       
    if DEBUG                            
      if (MainCntr//8)==0   'Dump debug info only once per 8 cycles
        ShowScreen

    !outa[Led]                           'Toggle I/O Pin for debug
    MainTime:=(cnt-T1)/80000
    MainCntr++

' ----------------  Init main program ---------------------------------------
PRI InitMain
  dira[Led]~~                            'Set I/O pin for LED to output…
  !outa[Led]                             'Toggle I/O Pin for debug

  ' Load movement schmitt start stop default values
  start_a_err := 10
  stop_a_err := 200
  stopstart_a_err := start_a_err

  if DEBUG
    if SerCog > 0
      ser.Stop

    SerCog:=ser.StartRxTx(cRXD, cTXD, 0, cBaud)                       'Start debug port via standard usb
    t.Pause1ms(200)
    ser.Clear
    ser.Str(string("Serial Debug Interface Started", CR))
  else
    SerCog:=0
  
  InitXbeeCmd

  !outa[Led]                               'Toggle I/O Pin for debug

  ResetPfStatus

  !outa[Led]                                            'Toggle I/O Pin for debug

  t.Pause1ms(2000)
  !outa[Led]                        'Toggle I/O Pin for debug



'================================ Init Xbee comm ==========================
PRI InitXbeeCmd
  MaxWaitTime := 100                    'ms wait time for incoming string  
  StrSp:=0
  
  ByteFill(@StrBuf,0,MaxStr)
  ByteFill(@cStrBuf,0,MaxStr)
  XbeeCog:=xBee.start(cxRXD, cxTXD, 0, cxBaud)     'Start xbee:  start(pin, baud, lines)
  'Xbee.str(string("Serial started", 13))


PRI InitWatchDog
  expected_wd:=0                     'Watchdog Stuff
  wd:=0
  wd_cnt:=0
'================================ Do Xbee command input and execution ==========================
PRI DoXbeeCmd

    StrCnt++
    Xbee.StrInMaxTime(@StrBuf,MaxStr,MaxWaitTime)   'Non blocking max wait time
    Xbee.rxflush
    if Strsize(@StrBuf)>3                           'Received string must be larger than 3 char's skip rest
        XBeeStat:=DoXCommand                        'Check input string for new commands
        ProcessCommand                              'Execute new commands


' ---------------- Check safety of platform and put in safe condition when needed ---------
PRI DoSafety | i, ConnectionError
  PID.ResetCurrError
  repeat
    ConnectionError := false
    SafetyCntr++
    if PID.GetFEAnyTrip
      Disable
      ResetBit(@PfStatus,NoAlarmBit)
      SetBit(@PfStatus,FeBit)
      LastAlarm:=1
      FEError:=1
      NoAlarm := false

    if PID.GetAnyCurrError
      Disable
      ResetBit(@PfStatus,NoAlarmBit)
      SetBit(@PfStatus,CurrBit)
      LastAlarm:=2
      CurrError:=1
      NoAlarm := false

    '-- watchdog --
    if Enabled 
        wd_cnt++
    else
        wd_cnt:=0

    if wd_cnt > 20               ' 20 will result in shutdown if no WD communication has taken place for approximatly 0.5 sec
      Disable
      ResetBit(@PfStatus,NoAlarm)
      SetBit(@PfStatus,CurrBit)
      LastAlarm:=3
      NoAlarm := false

    'Check for connection errors
    repeat i from 0 to MotorCnt-1
        ConnectionError := ConnectionError or PID.GetConnectionError(i)
        
    if ConnectionError
      Disable
      ResetBit(@PfStatus,NoAlarm)
      LastAlarm:=4
      NoAlarm := false

    If pMAECntr == MAECntr
      Disable
      ResetBit(@PfStatus,NoAlarmBit)
      SetBit(@PfStatus,MAEBit)
      LastAlarm:=5
    pMAECntr := MAECntr  


    t.Pause1ms(200)
    
' ---------------- Add error to log ---------------------------------------
PRI AddError2Log(ErCode)
' ---------------- Process Xbee commands into motion commands---------------------------------------
PRI ProcessCommand   
    if do_enable == 1 and not Enabled    'Enable Disable platform 1 = enabled 0 = disabled
       EnableWheelUnits
    elseif do_enable == 0 and Enabled 
       Disable

'------------------ Move all wheels individually --------------------------------
PRI Move
  
  Setp[1]:=wAngle[0]
  Setp[3]:=wAngle[1]
  Setp[5]:=wAngle[2]
  Setp[7]:=wAngle[3]  
 
  FR_a_err := Setp[1] - pid.GetActPos(1)
  FL_a_err := Setp[3] - pid.GetActPos(3)
  BR_a_err := Setp[5] - pid.GetActPos(5)
  BL_a_err := Setp[7] - pid.GetActPos(7)

  'Set speed to zero if a wheelunit is still turning
  if (FR_a_err > stopstart_a_err or FR_a_err < -stopstart_a_err) or (FL_a_err > stopstart_a_err or FL_a_err < -stopstart_a_err) or (BR_a_err > stopstart_a_err or BR_a_err < -stopstart_a_err) or (BL_a_err > stopstart_a_err or BL_a_err < -stopstart_a_err)
      Setp[0]:=0    
      Setp[2]:=0   
      Setp[4]:=0   
      Setp[6]:=0   
      'Wait for totally rotated
      stopstart_a_err := start_a_err
  else
      Setp[0]:=wSpeed[0]    'Front right is 0
      Setp[2]:=-wSpeed[1]   'Front left is  2
      Setp[4]:=wSpeed[2]    'Back right is  4
      Setp[6]:=-wSpeed[3]   'Back left is   6
      'Set rotation error stop driving value
      stopstart_a_err := stop_a_err

  'Disable wheels if no speed command given to avoid lock up of wheels and battery drainage
  if (wSpeed[0]==0 and wSpeed[1] ==0 and wSpeed[2] == 0 and wSpeed[3] == 0)
      DisableWheels    'Disable wheels to save battery
  else
      if Enabled
         EnableWheels
  
' -------------- DoXCommand: Get command parameters from Xbee input string -robot controller rose-------------
PRI DoXCommand | OK, i, j, Par1, Par2, lCh, t1, c1, req_id, received_wd
  t1:=cnt
  OK:=1

  StrP:=0  'Reset line pointer
  Sender:=0
  StrLen:=strsize(@StrBuf)  

  if StrLen > (MaxStr-1)        'Check max len
    OK:=-1                      'Error: String too long

  if StrLen == 0                'Check zero length
    OK:=-2                      'Error: Null string

  if OK==1                      'Parse string
    lCh:=sGetch
    repeat while (lch<>"$") and (OK == 1)       'Find start char
      lCh:=sGetch
      if StrP == StrLen
        OK:=-3                  'Error: No Command Start char found
        Quit                    'Exit loop

    if OK == 1
      Sender:=sGetPar

      Case Sender
        '--- Communicate controller id ---
        100 : Xbee.str(string("$100,"))
              Xbee.dec(CONTROLLER_ID)
              Xbee.tx(",")  
              Xbee.tx(CR) 
        '--- WATCHDOG ---
        111:      
             received_wd := sGetPar
             ' Check value
             if received_wd <> expected_wd
                Disable
                Xbee.tx("$")
                Xbee.dec(111)
                Xbee.tx(",")
                Xbee.dec(-1)
                Xbee.tx(",")  
                Xbee.dec(wd_cnt)
                Xbee.tx(",")            
                Xbee.dec(received_wd)
                Xbee.tx(",")   
                Xbee.dec(expected_wd)
                Xbee.tx(",")   
                Xbee.tx(CR)  
             else    
                Xbee.tx("$")
                Xbee.dec(111)
                Xbee.tx(",")
                Xbee.dec(wd)
                Xbee.tx(",")  
                Xbee.dec(wd_cnt)
                Xbee.tx(",")                
                Xbee.dec(received_wd)
                Xbee.tx(",")   
                Xbee.dec(expected_wd)
                Xbee.tx(",")   
                Xbee.tx(CR)  

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

        '=== Set drive PID parameters: Ki, K, Kp, Ilimit, PosScale, VelScale, FeMax, MaxCurr
        900: Ki:= sGetPar
             K:= sGetPar
             Kp:= sGetPar
             Ilimit:= sGetPar
             PosScale:= sGetPar
             VelScale:= sGetPar
             VelMax:= sGetPar
             FeMax:= sGetPar
             MaxCurr:= sGetPar
             'Send a reply (mirroring the received command)
             Xbee.tx("$")
             Xbee.dec(900)
             Xbee.tx(",")
             Xbee.dec(Ki)
             Xbee.tx(",")
             Xbee.dec(K)
             Xbee.tx(",")
             Xbee.dec(Kp)
             Xbee.tx(",")
             Xbee.dec(Ilimit)
             Xbee.tx(",")
             Xbee.dec(PosScale)
             Xbee.tx(",")
             Xbee.dec(VelScale)
             Xbee.tx(",")
             Xbee.dec(VelMax)
             Xbee.tx(",")
             Xbee.dec(FeMax)
             Xbee.tx(",")
             Xbee.dec(MaxCurr)
             Xbee.tx(",")
             Xbee.tx(CR)         
             SetDrivePIDPars(Ki, K, Kp, Ilimit, PosScale, VelScale, VelMax, FeMax, MaxCurr)
             '=== Set steering PID parameters: Ki, K, Kp, Ilimit, PosScale, VelScale, VelMax, FeMax, MaxCurr
        901: Ki:= sGetPar             
             K:= sGetPar
             Kp:= sGetPar
             Ilimit:= sGetPar
             PosScale:= sGetPar
             VelScale:= sGetPar
             VelMax:= sGetPar
             FeMax:= sGetPar
             MaxCurr:= sGetPar
             'Send a reply (mirroring the received command)
             Xbee.tx("$")
             Xbee.dec(901)
             Xbee.tx(",")
             Xbee.dec(Ki)
             Xbee.tx(",")
             Xbee.dec(K)
             Xbee.tx(",")
             Xbee.dec(Kp)
             Xbee.tx(",")
             Xbee.dec(Ilimit)
             Xbee.tx(",")
             Xbee.dec(PosScale)
             Xbee.tx(",")
             Xbee.dec(VelScale)
             Xbee.tx(",")
             Xbee.dec(VelMax)
             Xbee.tx(",")
             Xbee.dec(FeMax)
             Xbee.tx(",")
             Xbee.dec(MaxCurr)
             Xbee.tx(",")
             Xbee.tx(CR)         
             SetSteerPIDPars(Ki, K, Kp, Ilimit, PosScale, VelScale, VelMax, FeMax, MaxCurr)
    
       '=== Enable command from PC 
        902: do_enable:=sGetpar
             'Send a reply (mirroring the received command)
             Xbee.tx("$")
             Xbee.dec(902)
             Xbee.tx(",")
             ' if enabling but the drive or steer PID values are not yet set, send error response
             if do_enable==1 and drive_pid_vals_set and steer_pid_vals_set
                Xbee.dec(do_enable)        
             elseif do_enable==1
                Xbee.dec(3)     'Error response meaning not all pid vals were set before enablingZ
                do_enable:=0    'Force do_enable to zero
             'If disabling return a zero
             if do_enable==0
                Xbee.dec(0) 
             Xbee.tx(",")
             Xbee.tx(CR)         

        '=== Set the movement speeds and directions, 
        'Front right is 0
        'Front left is  2
        'Back right is  4
        'Back left is   6
        903: wSpeed[0]:=-PID.GetMaxVel(0) #> sGetPar  <# PID.GetMaxVel(0)        
             wAngle[0]:=PID.GetSetpMaxMin(1) #> -sGetPar <# PID.GetSetpMaxPlus(1)
             wSpeed[1]:=-PID.GetMaxVel(2) #> sGetPar  <# PID.GetMaxVel(2)         
             wAngle[1]:=PID.GetSetpMaxMin(3) #> -sGetPar <# PID.GetSetpMaxPlus(3)     
             wSpeed[2]:=-PID.GetMaxVel(4) #> sGetPar  <# PID.GetMaxVel(4)          
             wAngle[2]:=PID.GetSetpMaxMin(5) #> -sGetPar <# PID.GetSetpMaxPlus(5)     
             wSpeed[3]:=-PID.GetMaxVel(6) #> sGetPar  <# PID.GetMaxVel(6)
             wAngle[3]:=PID.GetSetpMaxMin(7) #> -sGetPar <# PID.GetSetpMaxPlus(7)                        
             
             'Send a reply (mirroring the received command)
             Xbee.tx("$")
             Xbee.dec(903)
             Xbee.tx(",")
             Xbee.dec(wSpeed[0])
             Xbee.tx(",")
             Xbee.dec(-wAngle[0])
             Xbee.tx(",")             
             Xbee.dec(wSpeed[1])
             Xbee.tx(",")
             Xbee.dec(-wAngle[1])
             Xbee.tx(",")
             Xbee.dec(wSpeed[2])
             Xbee.tx(",")
             Xbee.dec(-wAngle[2])
             Xbee.tx(",")
             Xbee.dec(wSpeed[3])
             Xbee.tx(",")
             Xbee.dec(-wAngle[3])
             Xbee.tx(",")
             Xbee.tx(CR)  

            '=== Set start/stop movement angle error values
        904: start_a_err:= sGetPar
             stop_a_err:= sGetPar

             'Send a reply (mirroring the received command)
             Xbee.tx("$")
             Xbee.dec(904)
             Xbee.tx(",")
             Xbee.dec(start_a_err)
             Xbee.tx(",")
             Xbee.dec(stop_a_err)
             Xbee.tx(",")
             Xbee.tx(CR)         

        999: ResetPfStatus        'Reset platform status
            'Send a reply (mirroring the received command)
             Xbee.tx("$")
             Xbee.dec(999)
             Xbee.tx(CR)
           
        '=== Status commands
        912: DoStatus2PC     'Status and errors
        913: DoPos2Pc        'Position to PC
        914: DoCurrents2PC   'Report currents
        916: DoPIDSettings   'Send PID parameters to PC
 
        '=== Get velocity of motor with ID
        1000: req_id:=sgetPar         
              if req_id > -1 and req_id < 8
                Xbee.tx("$")
                Xbee.dec(1000)
                xBee.tx(",")
                if req_id == 2 or req_id == 6
                    xBee.dec(-pid.GetActVel(req_id))  
                else
                    xBee.dec(pid.GetActVel(req_id))  
                xBee.tx(",") 
                Xbee.tx(CR)

        '=== Get position of motor with ID
        1001: req_id:=sgetPar         
              if req_id > -1 and req_id < 8
                Xbee.tx("$")
                Xbee.dec(1001)
                xBee.tx(",")
                if req_id == 0 or req_id == 2 or req_id == 4 or req_id == 6         'Drive motor, normal encoders
                    if req_id == 2 or req_id == 6
                        xBee.dec(-pid.GetActEncPos(req_id))  
                    else
                        xBee.dec(pid.GetActEncPos(req_id))   
                else                                                                'Steer motors, absolute encoders
                    xBee.dec(pid.GetActPos(req_id))  
                
                xBee.tx(",") 
                Xbee.tx(CR)

        '=== Get current of motor with ID
        1002: req_id:=sgetPar         
              if req_id > -1 and req_id < 8
                Xbee.tx("$")
                Xbee.dec(1002)
                xBee.tx(",")
                xBee.dec(pid.GetActCurrent(req_id))  
                xBee.tx(",") 
                Xbee.tx(CR)

        '=== Get maximally reached current of motor with ID
        1003: req_id:=sgetPar         
              if req_id > -1 and req_id < 8
                Xbee.tx("$")
                Xbee.dec(1003)
                xBee.tx(",")
                xBee.dec(pid.GetMaxCurrent(req_id))  
                xBee.tx(",") 
                Xbee.tx(CR)

        '=== Get position error of motor with ID
        1004: req_id:=sgetPar         
              if req_id > -1 and req_id < 8
                Xbee.tx("$")
                Xbee.dec(1004)
                xBee.tx(",")
                xBee.dec(Setp[req_id] - pid.GetActPos(req_id))  
                xBee.tx(",") 
                Xbee.tx(CR)

        '=== Get velocity error of motor with ID
        1005: req_id:=sgetPar         
              if req_id > -1 and req_id < 8
                Xbee.tx("$")
                Xbee.dec(1005)
                xBee.tx(",")
                xBee.dec(pid.GetDeltaVel(req_id))  
                xBee.tx(",") 
                Xbee.tx(CR)

        '=== Get PI out of motor with ID
        1006: req_id:=sgetPar         
              if req_id > -1 and req_id < 8
                Xbee.tx("$")
                Xbee.dec(1006)
                xBee.tx(",")
                xBee.dec(pid.GetPIDOut(req_id))  
                xBee.tx(",") 
                Xbee.tx(CR)

        '=== Get P out of motor with ID
        1007: req_id:=sgetPar         
              if req_id > -1 and req_id < 8
                Xbee.tx("$")
                Xbee.dec(1007)
                xBee.tx(",")
                xBee.dec(pid.GetPIDOut(req_id) - pid.GetIbuf(req_id))  
                xBee.tx(",") 
                Xbee.tx(CR)

        '=== Get I out of motor with ID
        1008: req_id:=sgetPar         
              if req_id > -1 and req_id < 8
                Xbee.tx("$")
                Xbee.dec(1008)
                xBee.tx(",")
                xBee.dec(pid.GetIbuf(req_id))  
                xBee.tx(",") 
                Xbee.tx(CR)

        '=== Get MAE of motor with ID
        1009: req_id:=sgetPar         
              if req_id > -1 and req_id < 8
                Xbee.tx("$")
                Xbee.dec(1009)
                xBee.tx(",")
                xBee.dec(pid.GetMAEpos(req_id))  
                xBee.tx(",") 
                Xbee.tx(CR)

        '=== Get FE of motor with ID
        1010: req_id:=sgetPar         
              if req_id > -1 and req_id < 8
                Xbee.tx("$")
                Xbee.dec(1010)
                xBee.tx(",")
                xBee.dec(pid.GetFE(req_id))  
                xBee.tx(",") 
                Xbee.tx(CR)
        '=== Get AlarmState and LastAlarm number
        1011: Xbee.tx("$")
              Xbee.dec(1011)
              xBee.tx(",")
              if not NoAlarm
                 xBee.dec(1)  
              else
                 xBee.dec(0)  
              xBee.tx(",") 
              xBee.dec(LastAlarm)  
              xBee.tx(",")
              Xbee.tx(CR)

        '=== Get All drive motor positions number
        1012: if pid.getEncSem
                Xbee.tx("$")
                Xbee.dec(1012)
                xBee.tx(",")
                xBee.dec(pid.GetActEncPosDiff(0)) 
                xBee.tx(",")
                xBee.dec(-pid.GetActEncPosDiff(2))  
                xBee.tx(",")
                xBee.dec(pid.GetActEncPosDiff(4)) 
                xBee.tx(",")
                xBee.dec(-pid.GetActEncPosDiff(6)) 
                xBee.tx(",")          
                xBee.dec(pid.getEncClkDiff) 
                xBee.tx(",") 
                Xbee.tx(CR)

                pid.resetActEncPosSem              
              else
                Xbee.tx("$")
                Xbee.dec(1012)
                xBee.tx(",")
                xBee.dec(0) 
                xBee.tx(",")
                xBee.dec(0)  
                xBee.tx(",")
                xBee.dec(0) 
                xBee.tx(",")
                xBee.dec(0) 
                xBee.tx(",")          
                xBee.dec(0) 
                xBee.tx(",") 
                Xbee.tx(CR)

        other:Xbee.tx("$")
              Xbee.dec(sender)
              xBee.tx(",")
              xBee.str(string("Unkown command", 13)) 

  XbeeTime:=cnt-t1
  XbeeCmdCntr++    
Return OK

' ---------------- Send actual and max currents to PC -------------------------------
PRI DoCurrents2PC | i
  Xbee.tx("$")
  Xbee.dec(Sender)        'Last Sender

  repeat i from 0 to MotorCnt-1       'Send actual currents
    xBee.tx(",")
    xBee.dec(ActCurr[i])

  repeat i from 0 to MotorCnt-1       'Send max actual currents
    xBee.tx(",")
    xBee.dec(MaxCurrent[i])
  
  Xbee.tx(",")
  Xbee.tx(CR)

' ---------------- Reset platform -------------------------------
PRI ResetPfStatus | i
  if DEBUG
    ser.Str(string("Pre MAECog    : "))
    ser.dec(MAECog)
    ser.Str(string(CR))

  if MAECog > 0
    cogstop(MAECog~ - 1)
    t.Pause1ms(1)
  MAECog:=CogNew(MAESense, @MAEStack) + 1                  'Start MAE sensing

  if DEBUG
    ser.Str(string("After MAECog  : "))
    ser.dec(MAECog)
    ser.Str(string(CR))

  if DEBUG
    ser.Str(string("Pre PIDcog    : "))
    ser.dec(PIDcog)
    ser.Str(string(CR))

  if PIDcog > 0
    pid.ResetCurrError 
    pid.ClearErrors
    PID.Stop
    t.Pause1ms(1)
  PIDCog  := PID.Start(PIDCTime, @Setp, @MAEPos, @MAEOffs, nPIDLoops) 
  PIDMode := PID.GetPIDMode(0)                            'Set open loop mode
  repeat while PID.GetPIDStatus <> 3                     'Wait while PID initializes

  if DEBUG
    ser.Str(string("After PIDcog  : "))
    ser.dec(PIDcog)
    ser.Str(string(CR))

  if DEBUG
    ser.Str(string("Pre SafetyCog : "))
    ser.dec(SafetyCog)
    ser.Str(string(CR)) 
 
  if SafetyCog > 0
    cogstop(SafetyCog~ - 1)  
    t.Pause1ms(1)
  SafetyCog:= CogNew(DoSafety, @SafetyStack) + 1

  if DEBUG
    ser.Str(string("After SafetyCog: "))
    ser.dec(SafetyCog)
    ser.Str(string(CR))


  PfStatus:=0
  ResetBit(@PfStatus,USAlarm)          'Reset error bits in PfStatus
  ResetBit(@PfStatus,CommCntrBit)
  SetBit(@PfStatus,NoAlarmBit)
  NoAlarm:=true                        'Reset global alarm var
  LastAlarm:=0                         'Reset last alarm message
  PcSpeed:=0                           'Reset setpoints
  MoveSpeed:=0
  MoveDir:=0

  repeat i from 0 to MotorCnt-1
    Setp[i] := 0

  wSpeed[0]:=0
  wSpeed[1]:=0
  wSpeed[2]:=0
  wSpeed[3]:=0
  wAngle[0]:=0
  wAngle[1]:=0
  wAngle[2]:=0
  wAngle[3]:=0

  drive_pid_vals_set:=false
  steer_pid_vals_set:=false  

  InitWatchDog  

' ---------------- Get next parameter from string ---------------------------------------
PRI sGetPar | j, jj, lPar, lch
  j:=0
  Bytefill(@LastPar1,0,CmdLen)   'Clear buffer
  lch:=sGetch                    'Get comma and point to first numeric char
  jj:=0
  repeat until lch=="," 'Get parameter until comma
    if lch<>"," and j<CmdLen
      LastPar1[j++]:=lch
    lch:=sGetch           'skip next
 
  LPar:=Xbee.strtodec(@LastPar1)
Return Lpar

' ---------------- Get next character from string ---------------------------------------
Pri sGetCh | lch 'Get next character from commandstring
   lch:=Byte[@StrBuf][StrP++]
Return lch

' ---------------- Print program status to PC ---------------------------------------
PRI DoStatus2PC
  Xbee.tx("$")
  Xbee.dec(Sender)        'Last Sender
  Xbee.tx(",")
  Xbee.dec(MoveMode)      'Mode mode 0 = manual 1= US sensor control
  Xbee.tx(",")
  Xbee.dec(LastAlarm)     'Last error
  Xbee.tx(",")
  Xbee.dec(XbeeTime/80000)      'Time of Xbee comm in ms
  Xbee.tx(",")
  Xbee.dec(pid.getCntr)   'HB52 counter to check life
  Xbee.tx(",")
  Xbee.dec(Enabled)       'Platform Enabled
  Xbee.tx(",")
  Xbee.dec(PfStatus)     'Platform status
  Xbee.tx(",")
  Xbee.dec(MainCntr)     'Main loop counter
  Xbee.tx(",")
  Xbee.dec(SafetyCntr)   'Safety loop counter
  Xbee.tx(",")
  Xbee.dec(Version)      'Software version
  Xbee.tx(CR)

  
' ---------------- Send Wheel Positions and speeds to PC ---------------------------------------
PRI DoPos2PC | i
  i:=0
  Xbee.tx("$")
  Xbee.dec(Sender)
  Xbee.tx(",")

  i:=0
  'Send actual positions of wheels
  xBee.tx(",")
  xBee.dec(pid.GetActPos(0))  
  xBee.tx(",")
  xBee.dec(pid.GetActPos(2))  
  xBee.tx(",")
  xBee.dec(pid.GetActPos(4))  
  xBee.tx(",")
  xBee.dec(pid.GetActPos(6))  
  xBee.tx(",")
  'Send actual positions of steer 
  xBee.dec(pid.GetActPos(1))  
  xBee.tx(",")
  xBee.dec(pid.GetActPos(3))  
  xBee.tx(",")
  xBee.dec(pid.GetActPos(5))  
  xBee.tx(",")
  xBee.dec(pid.GetActPos(7))
    
  'Send actual wheel velocities
  xBee.tx(",")
  xBee.dec(pid.GetActVel(0))  
  xBee.tx(",")
  xBee.dec(pid.GetActVel(2))  
  xBee.tx(",")
  xBee.dec(pid.GetActVel(4))  
  xBee.tx(",")
  xBee.dec(pid.GetActVel(6))  

  Xbee.tx(",")
    
  Xbee.tx(CR)


' ---------------- Command loop main program ---------------------------------------
PRI DoCommand(lCmd) | lPIDMode, lSetp
  case lCmd    
    "d","D": Disable
    "p","P": DoPIDSettings
    "c","C": pid.ClearErrors
             ResetPfStatus
    "s"    : ShowParameters                                                   
    "f","F": ResetFE                                                                                                         
                                                                
' ----------------  Clear FE trip ---------------------------------------
PRI ResetFE | i 
  PID.ResetAllFETrip
  ResetBit(@PfStatus,FEbit)

' ----------------  Move Toggle ---------------------------------------
PRI MoveToggle | i 
  MoveMode++
  If MoveMode>2
    MoveMode:=0

' ----------------  Toggle reporting ---------------------------------------
PRI ToggleReport  
  !DoShowParameters


' ----------------  Enable wheeluntis  ---------------------------------------
PRI EnableWheelUnits
    Enabled:=true
    EnableSteer    
    EnableWheels      
    SetBit(@PfStatus,EnableBit)

' ----------------  Enable steer  ---------------------------------------
PRI EnableSteer
  PID.SetPIDMode(1,3)                     'Pos loop steering
  PID.SetPIDMode(3,3)                     'Pos loop steering
  PID.SetPIDMode(5,3)                     'Pos loop steering
  PID.SetPIDMode(7,3)                     'Pos loop steering
' ----------------  Enable wheels  ---------------------------------------
PRI EnableWheels
  PID.SetPIDMode(0,1)                     'Enable vel wheel and
  PID.SetPIDMode(2,1)                     'Enable vel wheel and
  PID.SetPIDMode(4,1)                     'Enable vel wheel and
  PID.SetPIDMode(6,1)                     'Enable vel wheel and

' ----------------  Disable wheels  ---------------------------------------
PRI DisableWheels
  PID.SetPIDMode(0,0)                     'Enable vel wheel and
  PID.SetPIDMode(2,0)                     'Enable vel wheel and
  PID.SetPIDMode(4,0)                     'Enable vel wheel and
  PID.SetPIDMode(6,0)                     'Enable vel wheel and 
  
' ----------------  Disable all wheels and steerin ---------------------------------------
PRI Disable 
  do_enable:=0
  PID.KillAll
  Enabled:=false
  ResetBit(@PfStatus,EnableBit)
  'ShowParameters  
  
' Set drive motor control parameters
PRI SetDrivePIDPars(lKi, lK, lKp, lIlimit, lPosScale, lVelScale, lVelMax, lFeMax, lMaxCurr) | i
    repeat i from 0 to MotorCnt-1 step 2         
      PID.SetKi(i,lKi)
      PID.SetK(i,lK)
      PID.SetKp(i,lKp)
      PID.SetIlimit(i,lIlimit)
      PID.SetPosScale(i,lPosScale)
      PID.SetVelScale(i,lVelScale)
      PID.SetMaxVel(i,lVelMax)
      PID.SetFeMax(i,lFeMax)
      PID.SetMaxCurr(i,lMaxCurr)      
      drive_pid_vals_set:=true

' Set steer motor control parameters
PRI SetSteerPIDPars(lKi, lK, lKp, lIlimit, lPosScale, lVelScale, lVelMax, lFeMax, lMaxCurr) | i
    repeat i from 1 to MotorCnt-1 step 2         
      PID.SetKi(i,lKi)
      PID.SetK(i,lK)
      PID.SetKp(i,lKp)
      PID.SetIlimit(i,lIlimit)
      PID.SetPosScale(i,lPosScale)
      PID.SetVelScale(i,lVelScale)
      PID.SetMaxVel(i,lVelMax)
      PID.SetFeMax(i,lFeMax)
      PID.SetMaxCurr(i,lMaxCurr)  

      setRotationLimits
      MAEOffs[0]:=2000
      MAEOffs[1]:=2000
      MAEOffs[2]:=2000
      MAEOffs[3]:=2000  
      steer_pid_vals_set:=true

'=============================================================================


'---------------- 'Set rotation limits to steer motors to about +-(3/4)*pi --------
PRI setRotationLimits | i
  repeat i from 1 to MotorCnt-1 step 2                  
    PID.SetSetpMaxMin(i,-1000)
    PID.SetSetpMaxPlus(i,1000)                         
         

' ----------------  MAE absolute encoder sense ---------------------------------------
PRI MAESense | t1 , lMAE0Pin, lMAE1Pin, lMAE2Pin, lMAE3Pin
  lMAE0Pin:= MAE0Pin
  lMAE1Pin:= MAE0Pin + 1
  lMAE2Pin:= MAE0Pin + 2
  lMAE3Pin:= MAE0Pin + 3
  
  Repeat
    t1:=cnt
    MAEPos[0]:= MAE.Pos(lMAE0Pin)
    MAEPos[1]:= MAE.Pos(lMAE1Pin)
    MAEPos[2]:= MAE.Pos(lMAE2Pin)
    MAEPos[3]:= MAE.Pos(lMAE3Pin)

    MAETime:=(cnt - t1)/80000      'Cycle time in us
    MAECntr++                   'Update alive counter
    
' --------------------- Return PID Time in us -----------------------------
PUB GetPIDCTime
Return PIDConnTime/80

' ----------------  Show P    ID parameters ---------------------------------------
PRI ShowParameters | i
   if SerCog > -1
      ser.Position(0,0)
      ser.str(string("Rose 2.0 Drive Firmware - Rose B.V. - 2014"))
      ser.dec(Version)
      ser.tx(SubVersion)
      ser.str(string(" Max Cog : "))
      ser.dec(pid.GetQIKCog)
    
      ser.tx(CR)
      ser.Position(0,pControlPars)
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

' ---------------- Dump PID settigns to Xbee ---------------------------------------
PRI DoPIDSettings | i
  Xbee.tx("$")
  Xbee.dec(Sender)        'Last Sender
  Xbee.tx(",")
  xBee.str(string(CR,"MAEOffs: $0"))
  repeat i from 0 to MAECnt-1   'Copy working values to buffer
    xBee.tx(",")
    xBee.dec(MAEOffs[i])
  
  xBee.str(string(CR,"K: $1"))
  repeat i from 0 to MotorCnt-1       'Copy control parameters to storage
    xBee.dec(PID.GetK(i))

  xBee.str(string(CR,"Kp: $2"))
  repeat i from 0 to MotorCnt-1       'Copy control parameters to storage
    xBee.tx(",")
    xBee.dec(PID.GetKp(i))

  xBee.str(string(CR,"Ki: $3"))
  repeat i from 0 to MotorCnt-1       'Copy control parameters to storage
    xBee.tx(",")
    xBee.dec(PID.GetKi(i))

  xBee.str(string(CR,"Ilim: $4"))
  repeat i from 0 to MotorCnt-1       'Copy control parameters to storage
    xBee.tx(",")
    xBee.dec(PID.GetIlimit(i))

  xBee.str(string(CR,"PosScale: $5"))
  repeat i from 0 to MotorCnt-1       'Copy control parameters to storage
    xBee.tx(",")
    xBee.dec(PID.GetPosScale(i))
    xBee.tx("#")
        
' ----------------  Show actual value screen ---------------------------------------
PRI ShowScreen | i
  if SerCog > -1
    ser.Position(0,pActualPars)                      'Show actual values PID
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
      ser.str(n.decf(PID.GetIbuf(i),8))
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
    ser.str(string("  MAE LoopTime (ms): "))
    ser.dec(MAETime)
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
    ser.str(string(" Xbee Time ms: "))
    ser.dec(XbeeTime/80000)
    ser.str(string(" Xbee Stat: "))
    ser.dec(XbeeStat)
    ser.str(string(" XbeeCmdCntr: "))
    ser.dec(XbeeCmdCntr)
    ser.str(string(" Last Alarm: "))
    ser.dec(LastAlarm)
    ser.str(string(" Watchdog cnt: "))
    ser.dec(wd_cnt)
    ser.tx(" ")
    ser.str(@cStrBuf)
    
    ser.str(string(CE,CR, " Enabled: "))
    ser.str(n.dec(Enabled))
    ser.str(string(" PfStatus: "))
    ser.str(n.ibin(PfStatus,16))
       
    ser.str(string(CE,CR," Sender: "))
    ser.str(n.decf(Sender,4))

' ---------------- 'Set bit in 32 bit Long var -------------------------------
PRI SetBit(VarAddr,Bit) | lBit, lMask
  lBit:= 0 #> Bit <# 31    'Limit range
  lMask:= |< Bit           'Set Bit mask
  Long[VarAddr] |= lMask   'Set Bit
    

' ---------------- 'Reset bit in 32 bit Long var -------------------------------
PRI ResetBit(VarAddr,Bit) | lBit, lMask
  lBit:= 0 #> Bit <# 31    'Limit range
  lMask:= |< Bit           'Set Bit mask
  
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
