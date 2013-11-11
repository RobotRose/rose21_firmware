{=============================================================================
 Qic PID object real version for 8 motors dec/febr 2010 HJK
 Uses quadrature encoder to measure motor velocity
 Velocity/position control in 8 fold PID loop velocity and position at approx. 500 us per loop
 Set and read parameters of QiK controller
 Read PWM encoder US Digital
 V1.2 Dec 2010:
   LCD screen 2x16
   Xbee command interface
   Checks if Remote connection Joystick is alive.
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
      
 To do: MAE time out error, xbee comm time out error
 
=============================================================================
}
{ Platform control commands:
 Uses string handling for xbee command line handling

 Xbee remote control
   XB$500,33251,-369,-88,0,0,0,0,#   Id ($500),Cntr,JoyX,JoyY,Btn1,btn2,btn3,btn4

  PC Interface:
  Move command     :  $900,cntr,speed,dir  : where speed and dir = [-128..128]
  Enable motion    :  $901,Enable   Enable : 0=no motion 1= motion allowed
  Enable PC control:  $902,PcEnable PcEnable=0 or 1
  Wheel mode       :  $905,cnt,speed1, speed2, speed3, speed4, angle1, angle2. All angles and speeds independent
   
  Clear Errors     :  $908  (No parameters) ResetPfStatus: Reset platform status, ResetMaxCurrent, ResetCurError   
  Query wheel pos  :  $911  Returns 4 wheel positions, 4 steer positions, 4 wheel speeds,  Right Front, Left Front, RightR ear, Left Rear 
  Query Status     :  $912  Query status of various par's, like errors, platform status, life counters Xbee comm time
  Query currents   :  $914  Query actual and max currents: Return $914,actual current 0..7, max current [0..7]  
  Query PID pars   :  $916  Report PID parameters  
        
}
CON
' Version
  Version = 35
  SubVersion = "b" 

'Set 80Mhz
   _clkmode=xtal1+pll16x
' _clkmode = xtal1 + pll8x  'Spinstamp  
  _xinfreq = 5000000      'MRS1
'  _xinfreq = 10_000_000  'spin stamp

'' Led
  Led = 27
  
'' Serial port 
'   DebugUSB = true    'DebugUSB true: Debug via pin 30 and 31 and Commands via Xbee on 22 and 23. False: reversed  
   cTXD = 30 '20 '30
   cRXD = 31 '21 '31
   cBaud = 230400 ' 115200
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

  PIDCTime = 20           ' PID Cycle time ms

'Safety constants
   _1ms  = 1_000_000 / 1_000          'Divisor for 1 ms

  
'MAE3 PWM absolute encoder
  MAE0Pin  = 16            'First pin of MAE encoder
  MAECnt   = 4             'Number of encoders 

'Xbee
   cxTXD = 20 '30 '26 '23
   cxRXD = 21 '31 '25 '22
   cxBaud = 115200 '230400 '115200 '57600  115200 seems reliable max
   XBID  = 400            '' Xbee Id of this Robot platform                                                       yyyyy

'SerProp
   csTXD = 26              'Serial command port as alternative for Xbee
   csRXD = 25
   csBaud = 230400

'Command interface and control 
   LineLen = 100          ' Buffer size for incoming line
   SenderLen= 10
   Cmdlen = 10
   ButtonCnt = 4
   JoyXHyst = 250           'Joystick hysteresis
   JoyYHyst = 150           
   AliveTime = 100         ' Time in ms before shutdown 
   
'String buffer
  MaxStr = 257        'Stringlength is 256 + 1 for 0 termination

'Potmeter
  Pot0  = 9
  Pcenter = 500

'Terminal screen positions
  pControlPars = 1
  pActualPars = pControlPars + 13
  pMenu       = pActualPars + 26
  pInput      = pMenu + 12

'Eeprom
 cCheck = 12345       'Check value for correct restore of values.
 EptromStart = $7000  'Free range for saving

'Error logging
  ErrorCnt = 100


'Platform status bits
   Serialbit     = 0              '0= Serial Debug of 1= Serial pdebug port on
   USAlarm       = 1              'US alarm bit: 1= object detected in range
   PingBit       = 2              '0= Ping off 1= Ping on
   EnableBit     = 3              '0= Motion DisablePfd 1= Motion enabled
   PCEnableBit   = 4              'PC Enable -> Pf Enable
   PCControlBit  = 5              'PC In control
   CommCntrBit   = 6              'Xbee Timout error
   MotionBit     = 7              'Platform moving
   FeBit         = 8              'FE error on any axis
   CurrBit       = 9              'Current error on any axis
   MAEBit        = 10             'MAE encoder alarm
   NoAlarmBit    = 15             'No alarm present   
 
OBJ
  ser           : "Parallax Serial Terminal"            ' Serial communication object
  xBee          : "FullDuplexSerial_rr005" ' "FullDuplexSerialPlus"       ' Xbee serial
  t             : "Timing"
  MAE           : "MAE3"                                ' MAE absolute encoder object
  PID           : "PID Connect V4a"                      ' PID contr. 8 loops. Sep. Pos and Vel feedb + I/O.
'  Pot           : "Potmeter"                            ' Potmeter
'  QiK           : "QiKCommands"                         ' Standard serial for drives
'  LCD           : "Debug_LCD"                           ' Serial for LCD
  n             : "simple_numbers"                      ' Number to string conversion
  eprom         : "Eeprom"                              ' Routines for saving and reloading settings
  
Var Long PotmValue0, SpeedCom, DoShowParameters
    Long s, ms, us
    'Motors
    Long PIDCog, PIDMode
    Long Setp[MotorCnt] 'Actual position and velocity, Setpoint
    Long EngU[MotorCnt] 'The user units for readout and commanding a servo motor
    Byte ActPID, SerCog
    Long FeError', CurrError[MotorCnt], AnyCurrError 
    
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

    'Xbee and joystick input
    Long JoyCntr, JoyX, JoyY, Button[ButtonCnt], XbeeCmdCntr  
    Long Sender, CMDi, myID, Debug, XbeeTime, Enabled, XbeeStat, Lp, XbeeCog
    Byte Cmd[LineLen] ,LastPar1[CmdLen]
    Byte XbeeTimeout

    'Input string handling
    Byte StrBuf[MaxStr], cStrBuf[MaxStr]      'String buffer for receiving chars from serial port
    Long StrSP, StrCnt, StrP, StrLen  'Instring semaphore, counter, Stringpointer
    Long pcButton[ButtonCnt], SerEnabled, oSel0, CmdDone
    Long JoyComActive, PcComActive            'State var for comm monotoring
    Long MaxWaitTime  'Wait time for new Xbee string

    'Safety
    Long SafetyCog, SafetyStack[50], SafetyCntr, NoAlarm, CurrError

    'Command program variables
    Byte pcEnable, pcCommand, LastAlarm, PfStatus, PcControl  
    Long pcSpeed, pcDirection, pcCntr, pcMoveMode                    
    
    'Platform vars
    Long MoveSpeed, MoveDir, lMoveSpeed, lMoveDir, MoveMode, A1, A2, Rv 'A1, A2 wheel angle, Rv is speed ratio
    Long wSpeed[nWheels], wAngle[nWheels]
    Long DirRamp, SpeedRamp
    Word MainCntr
        
    'Parameters for saving and loading a config
    Long StartVar, sMAEOffs[MAECnt], sK[MotorCnt], sKp[MotorCnt], sKI[MotorCnt], sILim[MotorCnt]
    Long sPosScale[MotorCnt], PlatFormID, Check, EndVar

    'Parameters for saving logdata
    Long StartlVar, MaxCurrent[MotorCnt], ErrorLog[ErrorCnt], ActErrNr, EndLVar
       
' ---------------- Main program ---------------------------------------
PUB main | Up, T1, lch 'lSpeed , Offset, , ii
  InitMain
  !outa[Led]                             ' Toggle I/O Pin for debug
  LoadSettings(True)
  !outa[Led]                             ' Toggle I/O Pin for debug
  Up:=1
  PIDMode:=0
  Disable
  'Enable                                 ' Enable axis
'  lcd.cls
  repeat
    T1:=cnt
    lch:= ser.RxCheck                     ' Check serial port
    if lch>0                                            
'      ser.position(40,0)
'      ser.dec(ii)
      DoCommand(lch)
      lch:=0

    DoXbeeCmd
                                           
    if DoShowParameters and (MainCntr//8)==0   'Dump debug info only once per 8 cycles
      ShowScreen

    !outa[Led]                           'Toggle I/O Pin for debug
    MainTime:=(cnt-T1)/80000
    MainCntr++
'    t.Pause1ms(50)
    

'================================ Init Xbee comm ==========================
PRI InitXbeeCmd
  MaxWaitTime := 50                    'ms wait time for incoming string  
  StrSp:=0
  
  JoyComActive:=0                      'Reset communication state var's
  PcComActive:=0
  
  ByteFill(@StrBuf,0,MaxStr)
  ByteFill(@cStrBuf,0,MaxStr)
'  if DebugUSB 
    XbeeCog:=xBee.start(cxRXD, cxTXD, 0, cxBaud)     'Start xbee:  start(pin, baud, lines)
'  else
'    XbeeCog:=xBee.start(cRXD, cTXD, 0, cBaud)     'Start xbee:  start(pin, baud, lines)

'================================ Do Xbee command input and execution ==========================
PRI DoXbeeCmd

'  repeat
'    repeat until StrSP == 0  'Wait for release of string buf

    StrCnt++
 '   StrSp:=1
 '   StrInMaxTime(stringptr, maxcount,ms)
    Xbee.StrInMaxTime(@StrBuf,MaxStr,MaxWaitTime)   'Non blocking max wait time
      Xbee.rxflush
    if Strsize(@StrBuf)>3                           'Received string must be larger than 3 char's skip rest
       ByteMove(@cStrBuf,@StrBuf,MaxStr)            'Copy received string in display buffer for debug
 '   StrSp:=0
      XBeeStat:=DoXCommand                          'Check input string for new commands

      ProcessCommand                                  'Execute new commands


'==================================== EnableSerial ==========================
PRI EnablePfSerial     'Enable serial port
'  if DebugUSB
    SerCog:=ser.StartRxTx(cRXD, cTXD, 0, cBaud)                       'Start debug port
'  else    
'    SerCog:=ser.StartRxTx(cxRXD, cxTXD, 0, cxBaud)                    'Start debug port

  t.Pause1ms(500)
  ser.tx(CS)

'  ser.str(string("OmniBot control HJK V1.1", CR))
'  ser.str(string("Max Cog : "))
'  ser.dec(Sercog)
'  ser.tx(CR)
'  ser.tx(CR)
  
  SerEnabled:=true
  ShowCommands
  ShowParameters

  SetBit(@PfStatus,Serialbit)
  
'==================================== DisablePfSerial ==========================
PRI DisablePfSerial      'DisablePf serial port
  ser.tx(CS)
  ser.str(string("Pf Debug Comm stopped", CR))
  t.Pause1ms(100)

  Ser.stop
  SerEnabled:=false

  ResetBit(@PfStatus,Serialbit)

' ---------------- Check safety of platform and put in safe condition when needed ---------
PRI DoSafety | t1, ClkCycles, Period, OldCnt
  Period:= 100
  ClkCycles := ((clkfreq / _1ms * Period) - 4296) #> 381   'Calculate 1 ms time unit

  repeat
    t1:=cnt
    SafetyCntr++
    if PID.GetFEAnyTrip
      Disable
      ResetBit(@PfStatus,NoAlarm)
      SetBit(@PfStatus,FeBit)
      LastAlarm:=1
'      AddError2Log(111)
      FEError:=1
      
    if PID.GetAnyCurrError
      Disable
      ResetBit(@PfStatus,NoAlarm)
      SetBit(@PfStatus,CurrBit)
      LastAlarm:=2
      
'      AddError2Log(222)
      CurrError:=1

{    If JoyCntr==OldCnt              'Shutdown platform when alive counter not updated
      Disable
      ResetBit(@PfStatus,NoAlarm)
      SetBit(@PfStatus,CommCntrBit)
      LastAlarm:=3

    OldCnt:=JoyCntr

    WatchProcesses   }
      
    waitcnt(ClkCycles + T1)       'Wait for designated time
    
' ---------------- Add error to log ---------------------------------------
PRI AddError2Log(ErCode)

' ---------------- Watch various critical processes ---------------------------------------
PRI WatchProcesses
  If pMAECntr == MAECntr
    Disable
    ResetBit(@PfStatus,NoAlarm)
    SetBit(@PfStatus,MAEBit)
    LastAlarm:=4

  pMAECntr := MAECntr

' ---------------- Process Xbee commands into motion commands---------------------------------------
PRI ProcessCommand
'   PotmValue0:=Pot.Pos(Pot0)
'   If PotmValue0> 1995
'     PotmValue0:=0                       'Block faulty values
      
'   SpeedCom:=PotmValue0 - Pcenter  
        
  if pcEnable ==0      'Xbee Joy stick mode
    if JoyY > JoyYHyst    'Make correction for hysteresis
      MoveSpeed:=(JoyY-JoyYHyst) /12
    else
      if JoyY < -JoyYHyst
        MoveSpeed:=(JoyY+JoyYHyst) /12
      else
        MoveSpeed:=0
      
    if JoyX > JoyXHyst     'Dir
      MoveDir:=(JoyX-JoyXHyst) /4
    else
      if  JoyX < -JoyXHyst
        MoveDir:=(JoyX+JoyXHyst) /4
      else
        MoveDir:=0

    Move(MoveSpeed, MoveDir, MoveMode)
      
    if Button[0]==1 and Enabled
       Disable
    else  
      if Button[0]==1 and not Enabled 
         PID.ResetAllFETrip       'Reset all following errors
         FeError:=0
         pid.ResetCurrError       'Reset all current errors
         SetBit(@PfStatus,NoAlarmBit)
         EnableSteer              'Enable platform

    if Button[1]==1
      MoveMode:=0
    if Button[2]==1
      MoveMode:=1
    if Button[3]==1
      MoveMode:=2
      
  else 'PC control enabled   
      MoveSpeed:=pcSpeed  / 2
      MoveDir:=pcDirection ' * 4000/1800
      MoveMode:=pcMoveMode
      Move(MoveSpeed, MoveDir, MoveMode)

      if pcEnable==1 and  !Enabled            ' Update platform status
        EnableSteer
      if pcEnable==0 and Enabled  
        Disable

    
' ---------------- 'Move mode control platform -------------------------------
PRI Move(Speed, Dir, Mode)
  Case Mode
    -1: 'Nothing
    0: Move0(Speed/2, Dir)   'Normal forward backward
    1: Move1(Speed/3, Dir)   'Cross movement
    2: Move2(Speed/8, Dir)   'Rotate
    3: Move3                 'individual wheels

'------------------ Move all wheels individually --------------------------------
PRI Move3
  Setp[0]:=wSpeed[0] / 4
  Setp[2]:=-wSpeed[1] / 4
  Setp[4]:=wSpeed[2] / 4
  Setp[6]:=-wSpeed[3] / 4
  Setp[1]:=wAngle[0] * 2
  Setp[3]:=wAngle[1] * 2
  Setp[5]:=wAngle[2] * 2
  Setp[7]:=wAngle[3] * 2
  
 'Disable wheels if no speed command given to avoid lock up of wheels and battery drainage
 if wSpeed[0]==0 and wSpeed[1] ==0 and wSpeed[2] == 0 and wSpeed[3] == 0   'Disable wheels to save battery
     DisableWheels
 else
   if Enabled
     EnableWheels
' ---------------- 'Rotate platform around center -------------------------------
PRI Move2(Speed, Dir) | sA1, sA2 ', lSpeed
      sA1:=sA2:=0
      Setp[1]:=-sA2-500 'RF
      Setp[3]:=-sA1+500 'LF
      Setp[5]:= sA2+500 'RR
      Setp[7]:= sA1-500 'RL
      Setp[6]:=Setp[4]:=lMoveSpeed  'RF RR
      Setp[2]:=Setp[0]:=lMoveSpeed  'LF LR

 'Ramp control speed
 if Speed==0    'Disable wheels to save battery
     DisableWheels
 else
   if Enabled
     EnableWheels

 Speed:=Speed*Rv/100  ' Reduce speed when turning

 if Speed>lMoveSpeed
   lMoveSpeed:=lMoveSpeed + SpeedRamp <# Speed

 if Speed<lMoveSpeed
   lMoveSpeed:=Speed #> lMoveSpeed - SpeedRamp
   
' ---------------- 'Move platform with speed and direction Cross direction -------------------------------
PRI Move1(Speed, Dir) | sA1, sA2   
'     lA1:=lA2:=Dir
'     Setp[1]:=-lA2-1000 'RF
'     Setp[3]:=-lA1+1000 'LF
'     Setp[5]:= lA2+1000 'RR
'     Setp[7]:= lA1-1000 'RL
 'Steering pairs:  1)RF RR
 '                 2)LF LR
 if lMoveDir<0       'Turn left
   GetSteerAng(-lMoveDir/2,@A1, @A2, @Rv)
     sA1:=A1/20
     sA2:=A2/20
     Setp[1]:= -sA1-1000 'RF
     Setp[3]:= sA1+1000 'LF
     Setp[5]:=- sA2+1000 'RR
     Setp[7]:= sA2-1000 'RL
     Setp[6]:=Setp[4]:=lMoveSpeed        'RF RR
     Setp[2]:=Setp[0]:=-lMoveSpeed*Rv/100 'LF LR
 else  'Dir>0        turn right
   GetSteerAng(lMoveDir/2,@A1, @A2, @Rv)
     sA1:=A1/20
     sA2:=A2/20
     Setp[1]:=sA2-1000 'RF
     Setp[3]:=-sA2+1000 'LF
     Setp[5]:=sA1+1000 'RR
     Setp[7]:=-sA1-1000 'RL
     Setp[6]:=Setp[4]:=lMoveSpeed*Rv/100 'RF RR
     Setp[2]:=Setp[0]:=-lMoveSpeed      'LF LR

 if Dir>lMoveDir   'Direction rampgenerator
   lMoveDir:=lMoveDir + DirRamp <# Dir

 if Dir<lMoveDir
   lMoveDir:=Dir #> lMoveDir - DirRamp
   
 'Ramp control speed
 if Speed==0    'Disable wheels to save battery
     DisableWheels
 else
   if Enabled
     EnableWheels
     
' Speed:=Speed*Rv/100
 Speed:=Speed
 if Speed>lMoveSpeed
   lMoveSpeed:=lMoveSpeed + SpeedRamp <# Speed

 if Speed<lMoveSpeed
   lMoveSpeed:=Speed #> lMoveSpeed - SpeedRamp
   
' ---------------- 'Move platform with speed and direction -------------------------------
PRI Move0(Speed, Dir) | i, sA1, sA2, pSpeed, pDir 
 'Dir >0 = turn right Dir<0 = turn left while moving forward
 'Steering pairs:  1)RF RR
 '                 2)LF LR
 if lMoveDir<0       'Turn left
   GetSteerAng(-lMoveDir/2,@A1, @A2, @Rv)
     sA1:=A1/8
     sA2:=A2/8
     Setp[1]:=-sA2 'RF
     Setp[3]:=-sA1 'LF
     Setp[5]:= sA2 'RR
     Setp[7]:= sA1 'RL
     Setp[0]:=Setp[4]:=lMoveSpeed        'RF RR
'     Setp[2]:=Setp[6]:=-lMoveSpeed/100 'LF LR
     Setp[2]:=Setp[6]:=-lMoveSpeed*Rv/100 'LF LR
 else  'Dir>0        turn right
   GetSteerAng(lMoveDir/2,@A1, @A2, @Rv)
     sA1:=A1/8
     sA2:=A2/8
     Setp[1]:=sA1 'RF
     Setp[3]:=sA2 'LF
     Setp[5]:=- sA1 'RR
     Setp[7]:=-sA2 'RL
     Setp[0]:=Setp[4]:=lMoveSpeed*Rv/100 'RF RR
'     Setp[0]:=Setp[4]:=lMoveSpeed/100 'RF RR
     Setp[2]:=Setp[6]:=-lMoveSpeed      'LF LR
   
 'Ramp control speed
 if Speed==0    'Disable wheels to save battery
     DisableWheels
 else
   if Enabled
     EnableWheels
     
 Speed:=Speed  ' *Rv/100
 if Speed>lMoveSpeed
   lMoveSpeed:=lMoveSpeed + SpeedRamp <# Speed

 if Speed<lMoveSpeed
   lMoveSpeed:=Speed #> lMoveSpeed - SpeedRamp

 if Dir>lMoveDir
   lMoveDir:=lMoveDir + DirRamp <# Dir

 if Dir<lMoveDir
   lMoveDir:=Dir #> lMoveDir - DirRamp

' ---------------- Show steering angles from table ---------------------------------------
PUB ShowSteerAng(Index,lA1, lA2, lR)
    ser.dec(Index)
    ser.tx(" ")
    ser.dec(lA1)
    ser.tx(" ")
    ser.dec(lA2)
    ser.tx(" ")
    ser.dec(lR)
    ser.tx(CR)

' ---------------- Get steering angles from table ---------------------------------------
PUB GetSteerAng(Index, aA1, aA2, aR) | i
    Index:= 0 #> Index <# 255
    i:=Index*6
    long[aA1]:=word[@SB+i]
    long[aA2]:=word[@SB+i +2]
    long[aR]:= word[@SB+i +4]
Return Index

' ---------------- Report platform parameters to PC ---------------------------------------
PRI DoReportPFPars | i
  xBee.str(string("$902,"))
  xBee.dec(PlatformID)
  repeat i from 0 to MotorCnt-1   'Actual positions
    xBee.tx(",")
    xBee.dec(pid.GetActPos(i))
  
  repeat i from 0 to MotorCnt-1   'Actual currents
    xBee.tx(",")
    xBee.dec(pid.GetActCurrent(i))

 { repeat i from 0 to MotorCnt-1   'Actual currents
    xBee.tx(",")
    xBee.dec(MaxCurrent[i])     } 

  xBee.tx(",")
  xBee.tx("#")  
  xBee.tx(CR)

  
' -------------- DoXCommand: Get command parameters from Xbee input string --------------
PRI DoXCommand | OK, i, j, Par1, Par2, lCh, t1, c1     
'  ser.position(0,24)
'  ser.position(0,10)
'  ser.str(string("Debug XB "))
  t1:=cnt
  OK:=1

  StrP:=0  'Reset line pointer
  Sender:=0
  StrLen:=strsize(@StrBuf)  
'  ser.dec(StrLen)
'  ser.tx(" ")
'  ser.str(@StrBuf)

  if StrLen > (MaxStr-1)       'Check max len
'    ser.dec(MaxStr-1)
'    ser.tx(" ")
    OK:=-1                      'Error: String too long
    
  if StrLen == 0                'Check zero length
    OK:=-2                      'Error: Null string
    
  if OK==1                      'Parse string
    lCh:=sGetch
'    ser.Tx("1")
    repeat while (lch<>"$") and (OK == 1)       'Find start char
'      ser.Tx(">")
'        Return -5  'timeout
      lCh:=sGetch
      if StrP == StrLen
        OK:=-3                  'Error: No Command Start char found
        Quit                    'Exit loop

'    ser.str(string(" Sender : " ))
    if OK == 1
      Sender:=sGetPar
'    ser.dec(Sender)
'    ser.Tx(" ")
'    ser.Tx("3")
'    lch:=sGetch   'Get comma
'     ser.tx(CR)
      Case Sender
        '== Move command from Joy stick
        500: JoyComActive:=1     'Get Joystick values
             PcComActive:=0
'          ser.Tx("4")
'          ser.Tx(" ")
             JoyCntr := sGetPar
             JoyX := sGetPar
             JoyY := sGetPar
             Button[0]:=sGetpar
             Button[1]:=sGetpar
             Button[2]:=sGetpar
             Button[3]:=sGetpar
{          ser.str(string(" JCntr : " ))
          ser.dec(Jcntr)
          ser.str(string(" Button[0] : " ))
          ser.dec(Button[0])
          ser.Tx(" ")  }

        '=== Move commands from PC
        900: JoyComActive:=0
             PcComActive:=1       'Get PC command for speed and direction from PC move commands
             PcCntr := sGetPar
             PcSpeed := sGetPar
             PcDirection := sGetPar
             pcMoveMode := sGetPar

        901: PcEnable:=sGetpar    'Enable DisablePf platform in PC control mode 1= enabled 0= disable

        902: PcControl:=sGetpar   'Enable DisablePf PC control 1 = pc 0 is joystick

        905: PcComActive:= 1
             JoyComActive:=0
             PcCntr := sGetPar
             wSpeed[0]:=sGetPar   ' wSpeed[nWheels], wAngle[nWheels]               
             wSpeed[1]:=sGetPar               
             wSpeed[2]:=sGetPar               
             wSpeed[3]:=sGetPar
             wAngle[0]:=sgetPar              
             wAngle[1]:=sgetPar              
             wAngle[2]:=sgetPar              
             wAngle[3]:=sgetPar
             pcMoveMode:=3  'individual wheel control              
          
 '       906: 'PcComActive:=1      'Autonomous mode

        908: ResetPfStatus        'Reset platform status
'             ResetMaxCurrent
             pid.ResetCurrError
             ResetPfStatus   
{       909: PingEnable := sGetPar
             DoUSsensors(PingEnable)
}            
        '=== Status commands
'        911: DoSensors2PC   'US sensors
        912: DoStatus2PC     'Status and errors
        913: DoPos2Pc        'Position to PC
        914: DoCurrents2PC   'Report currents
        916: DoPIDSettings   'Send PID parameters to PC
        
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
  Xbee.tx("#")
  Xbee.tx(EOT)   'End of transmission

' ---------------- Reset platform -------------------------------
PRI ResetPfStatus 

  pid.ClearErrors
  PfStatus:=0
  ResetBit(@PfStatus,USAlarm)          'Reset error bits in PfStatus
  ResetBit(@PfStatus,CommCntrBit)
  SetBit(@PfStatus,NoAlarmBit)
  NoAlarm:=true                        'Reset global alarm var
  LastAlarm:=0                         'Reset last alarm message
  
  PcSpeed:=0                           'Reset setpoints
  MoveSpeed:=0
  MoveDir:=0
  

' ---------------- Get next parameter from string ---------------------------------------
PRI sGetPar | j, jj, ii, lPar, lch
  j:=0
  Bytefill(@LastPar1,0,CmdLen)   'Clear buffer
  lch:=sGetch                    'Get comma and point to first numeric char
  jj:=0
  repeat until lch=="," 'Get parameter until comma
'    if jj++ >100
'      return -1
    if lch<>"," and j<CmdLen
      LastPar1[j++]:=lch
    lch:=sGetch           'skip next
 
  LPar:=ser.strtodec(@LastPar1)
'  ser.str(string(" GetPar : " ))
'  ser.dec(lPar)
Return Lpar

' ---------------- Get next character from string ---------------------------------------
Pri sGetCh | lch 'Get next character from commandstring
   lch:=Byte[@StrBuf][StrP++]
'   ser.tx("\")          
'   ser.tx(lch)
 '  Cmd[Lp++]:=lch
Return lch

' ---------------- Print program status to PC ---------------------------------------
PRI DoStatus2PC
  Xbee.tx("$")
'  Xbee.tx("%")
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
  Xbee.dec(PcEnable)     'Pc has control Enabled
  Xbee.tx(",")
  Xbee.dec(PfStatus)     'Platform status
  Xbee.tx(",")
  Xbee.dec(MainCntr)     'Main loop counter
  Xbee.tx(",")
  Xbee.dec(SafetyCntr)   'Safety loop counter
  Xbee.tx(",")
  Xbee.dec(Version)      'Software version
  Xbee.tx(CR)
  Xbee.tx("#")
  Xbee.tx(EOT)   'End of transmission

  
' ---------------- Send Wheel Positions and speeds to PC ---------------------------------------
PRI DoPos2PC | i
  i:=0
  Xbee.tx("$")
'  Xbee.tx("%")
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
  Xbee.tx("#")
  Xbee.tx(EOT)   'End of transmission

' ---------------- Command loop main program ---------------------------------------
PRI DoCommand(lCmd) | lPIDMode, lSetp
  case lCmd
    "0": PIDMode:=0   'Open loop
         PID.SetAllPIDMode(PIDMode)
         MoveMode:=-1
         ShowParameters
    "1": PIDMode:=1   'Vel loop
         Setp[ActPID]:=0
'         PID.SetAllPIDMode(PIDMode)
         MoveMode:=-1
         PID.SetPIDMode(ActPID,PIDMode)
         ShowParameters
    "2": PIDMode:=3   'Pos loop
         PID.SetPIDMode(ActPID,PIDMode)
'         PID.SetAllPIDMode(PIDMode)
         ShowParameters
         MoveMode:=-1
    "3": SetPID       'select PID loop
         MoveMode:=-1
    "4": SetParameter
         MoveMode:=-1
    "O": SetSteerOffset
         MoveMode:=-1
    "o": lPIDMode:=PID.GetPIDMode(ActPID)
         if lPIDMode == 0 or lPIDMode == -2  'Check if loop is open loop
           lSetp:=NewSetpoint(-30,30)
           ser.str(string(" Open loop New setp: "))
           ser.dec(lSetp)
           PID.SetPIDMode(ActPID,-2)         'Set Open loop mode with command to output
           Setp[ActPID]:=lSetp
         else
               ser.Position(0,pInput)
           ser.str(string(" Not allowed! Current PID mode: "))
           ser.dec(lPIDMode)  
         
    "e","E": EnableSteer
    "d","D": Disable
    "s","S": SaveSettings
    "p","P": DoPIDSettings
    "r","R": LoadSettings(False)
    "i","I": InitEpromSettings
    "c","C": pid.ClearErrors
             ResetPfStatus
    " "    : ShowParameters
             ShowCommands
    "9"    : ToggleReport                                                     
    "t","T": pid.ResetCurrError                                                     
    "f","F": ResetFE                                                     
    "b","B": lPIDMode:=PID.GetPIDMode(ActPID)    'Setpoint mode
             if lPIDMode == 0 or lPIDMode == -1 'Check if loop is open loop or brake mode
               lSetp:=NewSetpoint(-30,30)
               ser.str(string(" Brake New setp: "))
               ser.dec(lSetp)
               PID.SetPIDMode(ActPID,-1)         'Set Open loop mode with brake
               pid.BrakeWheels(lSetp)                                                     
             else
               ser.Position(0,pInput)
               ser.str(string(" Not allowed! Current PID mode: "))
               ser.dec(lPIDMode)
    "m","M": MoveToggle                                                     
    "x","X": lPIDMode:=PID.GetPIDMode(ActPID)    'Setpoint mode
             if lPIDMode == 0 or lPIDMode == 1 or lPIDMode == 2 or lPIDMode == 3 'Check if loop is active in 1,2,3
               lSetp:=NewSetpoint(-300,300)
               ser.str(string(" Setpoint New value: "))
               ser.dec(lSetp)
               Setp[ActPID]:=lSetp
             else
               ser.Position(0,pInput)
               ser.str(string(" Not allowed! Current PID mode: "))
               ser.dec(lPIDMode)  
                                                                 

' ----------------  New Setpoint between value Min and Max---------------------------------------
PRI NewSetpoint(SpMin, SpMax) | InDec
  ClearInputArea
  ser.Position(0,pInput)
  ser.str(string(CR,"New Setpoint: ("))
  ser.dec(SpMin)
  ser.tx("-")
  ser.dec(SpMax)
  ser.str(string("  -999 = abort) "))
  InDec:= ser.DecIn
  if InDec == -999
    Return 0
  else
    Return SpMin #> InDec <# SpMax
  ClearInputArea
 
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

' ----------------  Enable steer  ---------------------------------------
PRI EnableSteer
  PID.SetPIDMode(1,3)                     'Pos loop steering
  PID.SetPIDMode(3,3)                     'Pos loop steering
  PID.SetPIDMode(5,3)                     'Pos loop steering
  PID.SetPIDMode(7,3)                     'Pos loop steering
  Enabled:=true
  SetBit(@PfStatus,EnableBit)
'  ShowParameters  
' ----------------  Enable wheels  ---------------------------------------
PRI EnableWheels
  PID.SetPIDMode(0,1)                     'Enable vel wheel and
  PID.SetPIDMode(2,1)                     'Enable vel wheel and
  PID.SetPIDMode(4,1)                     'Enable vel wheel and
  PID.SetPIDMode(6,1)                     'Enable vel wheel and
'  ShowParameters  

' ----------------  Disable wheels  ---------------------------------------
PRI DisableWheels
  PID.SetPIDMode(0,0)                     'Enable vel wheel and
  PID.SetPIDMode(2,0)                     'Enable vel wheel and
  PID.SetPIDMode(4,0)                     'Enable vel wheel and
  PID.SetPIDMode(6,0)                     'Enable vel wheel and 
'  ShowParameters
  
' ----------------  Disable all wheels and steerin ---------------------------------------
PRI Disable 
  PID.KillAll
  Enabled:=false
  ResetBit(@PfStatus,EnableBit)
  ShowParameters  

' ----------------  Set actual PID loop for parameter change ---------------------------------------
PRI SetPID 
  ser.Position(0,pInput) 
  ser.str(string("Select actual PID Loop : "))
  ActPID:= 0 #> ser.DecIn <# PID#PIDCnt-1
  ser.dec(ActPID)
  ShowParameters  

' ----------------  Set Steer offsets ---------------------------------------
PRI SetSteerOffset | i, lPar, lValue
  ClearInputArea
  ser.Position(0,pInput) 
  ser.str(string(CR,"Select Steer Unit (-1=abort): "))
  lPar:=-1 #> ser.DecIn <# MAECnt-1
  if lPar == -1
    Return
    
  i:=lPar  'Save Steer unit index
  ser.str(string(CR,"Selected Steer Unit : "))
  ser.dec(lPar)
  ser.str(string(CR,"Actual Value : "))
  ser.dec(MAEOffs[i])
  ser.str(string(CR,"New Value : 0..4100 (-1= abort): "))
  lPar:=-1 #> ser.DecIn <# 4100
  if lPar == -1
    Return
    
  MAEOffs[i]:=LPar
  ShowParameters

  ' ----------------  Set PID parameters  ---------------------------------------
PRI SetParameter | lPar, lValue, Choice
  ClearInputArea
  ser.Position(0,pInput) 
  ser.str(string(CR,"Selected PID Loop : $"))
  ser.dec(ActPID)
  ser.str(string(CR,"Select Parameter 0=Kp 1=K 2=Ki 3=Ilimit 4=PosScale 5=VelScale x=exit: "))
  lPar:=ser.rx
  ser.dec(lPar)
  ser.str(string(CR,"Actual Value : "))
  ser.str(string(CR,"New Value : 1..30000 : "))
  case lPar
    "0": ser.str(string("Kp : "))
       ser.dec(PID.GetKp(ActPID))
       Choice:=0
    "1": ser.str(string("K : "))
       ser.dec(PID.GetK(ActPID))
       Choice:=1
    "2": ser.str(string("KI : "))
       ser.dec(PID.GetKI(ActPID))
       Choice:=2
    "3": ser.str(string("Ilimit : "))
       ser.dec(PID.GetILimit(ActPID))
       Choice:=3
    "4": ser.str(string("Pos Scale : "))
       ser.dec(PID.GetPosScale(ActPID))
       Choice:=4
    "5": ser.str(string("Vel Scale : "))
       ser.dec(PID.GetVelScale(ActPID))
       Choice:=5
    "9": ser.str(string("Platform ID : "))
       ser.dec(PlatformID)
       Choice:=5
    "x": ClearInputArea 
      Return   
    
  ser.tx(" ")
  lValue:=1 #> ser.DecIn <# 30000
  ser.dec(lValue)
  Case Choice
    0: PID.SetKp(ActPID,lValue) 
    1: PID.SetK(ActPID,lValue) 
    2: PID.SetKI(ActPID,lValue) 
    3: PID.SetIlimit(ActPID,lValue) 
    4: PID.SetPosScale(ActPID,lValue) 
    5: PID.SetVelScale(ActPID,lValue) 
    9: PlatformID:=lValue  
  ClearInputArea
  ShowParameters
  
' ----------------  Set PID pars not default for 8-DOV Rose platform ---------------------------------------
PRI SetPIDPars | i
  'Set control parameters wheels
  PID.SetKi(0,25)
  PID.SetK(0,250)
  PID.SetKp(0,10)
  PID.SetIlimit(0,1500)
  PID.SetPosScale(0,10)
  PID.SetVelScale(0,10)
  PID.SetFeMax(0,200)
  PID.SetMaxCurr(0,4500)
  
  PID.SetKi(2,25)
  PID.SetK(2,250)
  PID.SetKp(2,10)
  PID.SetIlimit(2,1500)
  PID.SetVelScale(2,10)
  PID.SetPosScale(2,10)
  PID.SetFeMax(2,200)
  PID.SetMaxCurr(2,4500)

  PID.SetKi(4,25)
  PID.SetK(4,250)
  PID.SetKp(4,10)
  PID.SetIlimit(4,1500)
  PID.SetVelScale(4,10)
  PID.SetPosScale(4,10)
  PID.SetFeMax(4,200)
  PID.SetMaxCurr(4,4500)

  PID.SetKi(6,25)
  PID.SetK(6,250)
  PID.SetKp(6,10)
  PID.SetIlimit(6,1500)
  PID.SetVelScale(6,10)
  PID.SetPosScale(6,10)
  PID.SetFeMax(6,200)
  PID.SetMaxCurr(6,4500)
  
  'Set control parameters steer motors
  PID.SetKi(1,200)
  PID.SetK(1,900)
  PID.SetKp(1,5000)
  PID.SetIlimit(1,800)
  PID.SetVelScale(1,1)
  PID.SetPosScale(1,1)
  PID.SetMaxVel(1,800)
  PID.SetFeMax(1,700)
  PID.SetMaxCurr(1,5500)
  
  PID.SetKi(3,200)
  PID.SetK(3,900)
  PID.SetKp(3,5000)
  PID.SetIlimit(3,800)
  PID.SetVelScale(3,1)
  PID.SetPosScale(3,1)
  PID.SetMaxVel(3,800)
  PID.SetFeMax(3,700)
  PID.SetMaxCurr(3,5500)
  
  PID.SetKi(5,200)
  PID.SetK(5,900)
  PID.SetKp(5,5000)
  PID.SetIlimit(5,800)
  PID.SetVelScale(5,1)
  PID.SetPosScale(5,1)
  PID.SetMaxVel(5,800)
  PID.SetFeMax(5,700)
  PID.SetMaxCurr(5,5500)
  
  PID.SetKi(7,200)
  PID.SetK(7,900)
  PID.SetKp(7,5000)
  PID.SetIlimit(7,800)
  PID.SetVelScale(7,1)
  PID.SetPosScale(7,1)
  PID.SetMaxVel(7,800)
  PID.SetFeMax(7,700)
  PID.SetMaxCurr(7,5500)
  
  MAEOffs[0]:=2000
  MAEOffs[1]:=2000
  MAEOffs[2]:=2000
  MAEOffs[3]:=2000

  PlatformID:=1001
  DirRamp:=35
  SpeedRamp:=14

' ----------------  Set PID pars not default for 8DOV testplatform ---------------------------------------
PRI SetPIDParsTP | i
  'Set control parameters wheels
  PID.SetKi(0,200)
  PID.SetK(0,900)
  PID.SetKp(0,1000)
  PID.SetIlimit(0,1000)
  PID.SetPosScale(0,1)
  PID.SetFeMax(0,200)
  PID.SetMaxCurr(0,6500)
  
  PID.SetKi(2,200)
  PID.SetK(2,900)
  PID.SetKp(2,1000)
  PID.SetIlimit(2,1000)
  PID.SetPosScale(2,1)
  PID.SetFeMax(2,200)
  PID.SetMaxCurr(2,6500)

  PID.SetKi(4,200)
  PID.SetK(4,900)
  PID.SetKp(4,1000)
  PID.SetIlimit(4,1000)
  PID.SetPosScale(4,1)
  PID.SetFeMax(4,200)
  PID.SetMaxCurr(4,6500)

  PID.SetKi(6,200)
  PID.SetK(6,900)
  PID.SetKp(6,1000)
  PID.SetIlimit(6,1000)
  PID.SetPosScale(6,1)
  PID.SetFeMax(6,200)
  PID.SetMaxCurr(6,6500)
  
  'Set control parameters steer motors
  PID.SetKi(1,200)
  PID.SetK(1,900)
  PID.SetKp(1,5000)
  PID.SetIlimit(1,800)
  PID.SetPosScale(1,1)
  PID.SetMaxVel(1,800)
  PID.SetFeMax(1,700)
  PID.SetMaxCurr(1,5500)
  
  PID.SetKi(3,200)
  PID.SetK(3,900)
  PID.SetKp(3,5000)
  PID.SetIlimit(3,800)
  PID.SetPosScale(3,1)
  PID.SetMaxVel(3,800)
  PID.SetFeMax(3,700)
  PID.SetMaxCurr(3,5500)
  
  PID.SetKi(5,200)
  PID.SetK(5,900)
  PID.SetKp(5,5000)
  PID.SetIlimit(5,800)
  PID.SetPosScale(5,1)
  PID.SetMaxVel(5,800)
  PID.SetFeMax(5,700)
  PID.SetMaxCurr(5,5500)
  
  PID.SetKi(7,200)
  PID.SetK(7,900)
  PID.SetKp(7,5000)
  PID.SetIlimit(7,800)
  PID.SetPosScale(7,1)
  PID.SetMaxVel(7,800)
  PID.SetFeMax(7,700)
  PID.SetMaxCurr(7,5500)
  
  MAEOffs[0]:=2000
  MAEOffs[1]:=2000
  MAEOffs[2]:=2000
  MAEOffs[3]:=2000

  PlatformID:=1001
  DirRamp:=35
  SpeedRamp:=14

'=============================================================================
' ----------------  Init main program ---------------------------------------
PRI InitMain
  dira[Led]~~                             'Set I/O pin for LED to output…
  !outa[Led]                              'Toggle I/O Pin for debug
 ' SerCog:=ser.start(Baud)                               'Start serial port start(rxpin, txpin, mode, baudrate)
'  if DebugUSB
    SerCog:=ser.StartRxTx(cRXD, cTXD, 0, cBaud)                       'Start debug port via standard usb
'  else    
'    SerCog:=ser.StartRxTx(cxRXD, cxTXD, 0, cxBaud)                    'Start debug port via Xbee
    
  t.Pause1ms(200)
  ser.Clear
  
  InitXbeeCmd

  !outa[Led]                               'Toggle I/O Pin for debug

  MAECog:=CogNew(MAESense, @MAEStack)                   'Start MAE sensing
  SafetyCog:= CogNew(DoSafety, @SafetyStack)
  
  PIDCog:=PID.Start(PIDCTime, @Setp, @MAEPos, @MAEOffs, nPIDLoops)  
  PIDMode:=PID.GetPIDMode(0)                            'Set open loop mode
  repeat while PID.GetPIDStatus<>2                      'Wait while PID initializes
  SetPIDPars

  ShowParameters                                        'Show control parameters
  !outa[Led]                                            'Toggle I/O Pin for debug

  ShowCommands
  t.Pause1ms(2000)
  !outa[Led]                        'Toggle I/O Pin for debug

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

' ----------------  Show PID parameters ---------------------------------------
PRI ShowParameters | i
  ser.Position(0,0)
  ser.str(string("OmniBot (c) 8-DOF platform QiK PID  Opteq HJK V"))
  ser.dec(Version)
  ser.tx(SubVersion)
  ser.str(string(" Max Cog : "))
  ser.dec(pid.GetQIKCog)

  ser.tx(CR)
  ser.Position(0,pControlPars)
  ser.str(string("Platform ID: "))
  ser.dec(PlatformID)
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

' ----------------  Show command menu ---------------------------------------
PRI ShowCommands1
    ser.Position(0,pMenu) 
    ser.str(string("0: Open loop",CR))
    ser.str(string("1: Velocity loop",CR))
    ser.str(string("2: Position loop",CR))
    ser.str(string("3: Select PID loop",CR))
    ser.str(string("4: Set PID parameter",CR))
    ser.str(string("5: Show PID parameter",CR))
    ser.str(string("O: Modify steer Offset parameter",CR))
    ser.str(string("E: Enable Steer",CR))
    ser.str(string("D: Disable All",CR))
    ser.str(string("s: Save settings",CR))
    ser.str(string("r: Restore settings",CR))
    ser.str(string("p: Dumpsettings",CR))
    ser.str(string("i: Init Settings",CR))
    ser.str(string("c: Clear Errors",CR))
    ser.str(string(" : Refresh screen",CR))
    ser.str(string("o: Open loop command",CR))
    ser.str(string("t: Reset max current",CR))
    ser.str(string("f: Reset FE",CR))
    ser.str(string("9: Report toggle",CR))
    ser.str(string("b: Brake wheels",CR))

' ----------------  Show command menu ---------------------------------------
PRI ShowCommands
    ser.Position(0,pMenu) 
    ser.str(string("======================Menu ======================================",CR))
    ser.str(string("0: Open loop 1: Velocity loop 2: Position loop 3: Select PID loop c: Clear Errors b: Brake wheels",CR))
    ser.str(string("4: Set PID parameter o: Open loop command f: Reset FE t: Reset max current",CR,CR))
    ser.str(string("X: SetPoint",CR,CR))

    ser.str(string("O: Mod Steer Offset E: Enable Steer D: Disable All",CR,CR))

    ser.str(string("s: Save settings r: Restore settings i: Init Settings",CR,CR))

    ser.str(string("_: Refresh screen 9: Report toggle "))


' ----------------  Clear input area ---------------------------------------
PRI ClearInputArea  | i
  ser.Position(0,pInput)
  repeat i from 0 to 5
    ser.str(string("                                                                 "))
    ser.tx(CE)
    ser.tx(CR)
  ser.Position(0,pInput)
  ser.tx(">")                              

' ---------------- Dump PID settigns to Xbee ---------------------------------------
PRI DoPIDSettings | i
  Xbee.tx("$")
  Xbee.dec(Sender)        'Last Sender
  Xbee.tx(",")
  xBee.str(string("Platform ID:"))
  xBee.dec(PlatformID)
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
     
' ---------------- Load data from Eprom ---------------------------------------
PRI LoadSettings(Silent) | i
  'PUB VarRestore(startAddr, endAddr) | addr
  'PUB ToRam(startAddr, endAddr, eeStart) | addr
  Disable                             'Disable all loops while restoring
' eprom.VarRestore(@startVar, @endVar)
  eprom.ToRam(@startVar, @endVar, EptromStart)
  if Check == cCheck                  'Check correctness of data
    repeat i from 0 to MAECnt-1       'Copy stored values to working buffer
      MAEOffs[i]:=sMAEOffs[i]
      
    repeat i from 0 to MotorCnt-1       'Copy stored control parameters
      PID.SetK(i,sK[i])
      PID.SetKp(i,sKp[i])
      PID.SetKi(i,sKI[i])
      PID.SetIlimit(i,sIlim[i])
      PID.SetPosScale(i,sPosScale[i])
      
    if not silent
      ser.clear
      ser.str(string("Settings restored from, to"))
      ser.hex(EptromStart,4)
      ser.tx(">")
      ser.hex(@startVar,4)
      ser.tx(",")
      ser.hex(@endVar,4)
    
  else
    ser.clear
    ser.str(string(" Error restoring settings"))
    ser.str(string(CR," Read check value: "))
    ser.dec(Check)
    ser.str(string(" Read Eeprom value: "))
    ser.dec(cCheck)
    ser.str(string(CR,CR," Continue storing defaults? (y/n)"))
    if ser.rx=="y"
      InitEpromSettings

  if not silent
    ser.str(string(CR," Press key to continue: "))
    ser.rx
    ser.clear
    
    ShowParameters 
    ShowCommands
    
' ---------------- Save data to Eprom ---------------------------------------
PRI SaveSettings | i
 ' PUB VarBackup(startAddr, endAddr) | addr, page, eeAddr   'Save a block of varibles from ram -> Rom
  'PUB FromRam(startAddr, endAddr, eeStart) | addr, page, eeAddr
  repeat i from 0 to MAECnt-1   'Copy working values to buffer
    sMAEOffs[i]:=MAEOffs[i]

  repeat i from 0 to MotorCnt-1       'Copy control parameters to storage
    sK[i]:=PID.GetK(i)
    sKp[i]:=PID.GetKp(i)
    sKi[i]:=PID.GetKi(i)
    sIlim[i]:=PID.GetIlimit(i)
    sPosScale[i]:=PID.GetPosScale(i)
    
  Check:=cCheck                 'Save safety value
  eprom.FromRam(@startVar, @endVar, EptromStart)
  'eprom.VarBackup(@startVar, @endVar)
  ser.clear
  ser.str(string("Settings saved from, to"))
  ser.hex(@startVar,4)
  ser.tx(",")
  ser.hex(@endVar,4)
  ser.tx(">")
  ser.hex(EptromStart,4)
  
  ser.str(string(CR," Press key to continue: "))
  ser.rx

  ser.clear
  ShowParameters
  ShowCommands
  
' ---------------- Init data to Eprom ---------------------------------------
PRI InitEpromSettings | i
  repeat i from 0 to MAECnt-1   'Reset
    MAEOffs[i]:=2000
    SetPIDPars
    ShowParameters
        
' ----------------  Show actual value screen ---------------------------------------
PRI ShowScreen | i
    ser.Position(0,pActualPars)                      'Show actual values PID
    ser.str(string(CR,"    Setp: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(Setp[i],6))
      ser.tx("|")

    ser.str(string(CR,"     MAE: |"))
    repeat i from 0 to MAECnt-1
      ser.str(n.decf(pid.GetMAEpos(i),6))
      ser.tx("|")
                                              
    ser.str(string(CR,"ActEncPs: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(pid.GetActEncPos(i),6))
      ser.tx("|")
    ser.str(string(CR,"  ActPos: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(pid.GetActPos(i),6))
      ser.tx("|")
    ser.str(string(CR," PidSetP: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(Pid.GetSetP(i),6))
      ser.tx("|")
      
    ser.str(string(CR,"     Vel: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetActVel(i),6))
      ser.tx("|")
    ser.str(string(CR,"DeltaVel: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetDeltaVel(i),6))
      ser.tx("|")
    ser.str(string(CR," MSetVel: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetSetVel(i),6))
      ser.tx("|")
    ser.str(string(CR,"    Ibuf: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetIbuf(i),6))
      ser.tx("|")
    ser.str(string(CR," PID Out: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetPIDOut(i),6))
      ser.tx("|")
    ser.str(string(CR," Current: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(pid.GetActCurrent(i),6))
      ser.tx("|")
    ser.str(string(CR,"DriveErr: | "))
    repeat i from 0 to MotorIndex
      ser.hex(pid.GetError(i),2)
      ser.str(string("   | "))
    ser.str(string(CR,"MaxStCur: |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(pid.GetMaxSetCurrent(i),6))
      ser.str(string("|"))
    ser.str(string(CR,"MaxCurr : |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(pid.GetMaxCurrent(i),6))
      ser.str(string("|"))
    ser.str(string(CR,"Fe      : |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetFE(i),6))
      ser.str(string("|"))

    ser.str(string(CR,"FeTrip  : |"))
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetFETrip(i),6))
      ser.str(string("|"))
    ser.str(n.decf(PID.GetFEAnyTrip,6))
    ser.str(string(CR,"Cur Trip: |"))                  
    repeat i from 0 to MotorIndex
      ser.str(n.decf(PID.GetCurrError(i),6))
      ser.str(string("|"))
    ser.str(n.decf(pid.getAnyCurrError,6))
      
    ser.str(string(CR,CR,"PIDTime (ms): "))
    ser.dec(PID.GetPIDTime)
    ser.str(string("  PIDLeadTime (ms): "))
    ser.dec(PID.GetPIDLeadTime)
    ser.str(string("  PID LoopTime (ms): "))
    ser.dec(PIDCTime)
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
    ser.str(string(" Xbee Time ms: "))
    ser.dec(XbeeTime/80000)
    ser.str(string(" Xbee Stat: "))
    ser.dec(XbeeStat)
    ser.str(string(" XbeeCmdCntr: "))
    ser.dec(XbeeCmdCntr)
    ser.str(string(" Last Alarm: "))
    ser.dec(LastAlarm)

    ser.tx(" ")
    ser.str(@cStrBuf)
    
    ser.str(string(CE,CR," PcComActive: "))
    ser.str(n.decf(PcComActive,3))
    ser.str(string(" JoyComActive : "))
    ser.str(n.decf(JoyComActive,3))
    ser.str(string(" Enabled: "))
    ser.str(n.decf(Enabled,3))
    ser.str(string(" PC Enable: "))
    ser.str(n.decf(pcEnable,3))
    ser.str(string(" PC MoveMode: "))
    ser.str(n.decf(pcMoveMode,3))
    ser.str(string(" PfStatus: "))
    ser.str(n.ibin(PfStatus,16))
       
    ser.str(string(CE,CR," Sender: "))
    ser.str(n.decf(Sender,4))
    ser.str(string(" JoyCntr: "))
    ser.str(n.decf(JoyCntr,4))
    ser.str(string(" JoyX: "))
    ser.str(n.decf(JoyX,4))
    ser.str(string(" JoyY: "))
    ser.str(n.decf(JoyY,4))
    ser.str(string(" Btn: "))
    repeat i from 0 to ButtonCnt-1
      ser.dec(Button[i])
      ser.tx(" ")
    ser.tx(CE)
    ser.tx(" ")
    ser.str(string(CR," Platf Speed: "))
    ser.str(n.decf(MoveSpeed,4))
    ser.str(string(" Dir: "))
    ser.str(n.decf(MoveDir,4))
    ser.str(string(" MoveMode: "))
    ser.str(n.decf(MoveMode,4))
    ser.str(string(" Ang1: "))
    ser.str(n.decf(A1,4))
    ser.str(string(" Ang2: "))
    ser.str(n.decf(A2,4))
    ser.str(string(" Speed Ratio: "))
    ser.str(n.decf(Rv,4))
    ser.str(string(" MoveMode: "))
    ser.str(n.decf(MoveMode,4))

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


DAT '15x15 cm platform
SB ' hoek1*100, hoek2*100, V1/V2*100 Steering table. Hoek1 is inside corner, V1 is speed inside wheel
'Steer Input is index ==> Inside angle, Outside angle, Inside wheel velocity 
Word  00000,   00002,  0100
Word  00035,   00035,  0099
Word  00070,   00069,  0098
Word  00105,   00102,  0096
Word  00141,   00134,  0095
Word  00176,   00166,  0094
Word  00211,   00196,  0093
Word  00246,   00227,  0092
Word  00281,   00256,  0091
Word  00316,   00285,  0090
Word  00352,   00313,  0089
Word  00387,   00341,  0088
Word  00422,   00368,  0087
Word  00457,   00394,  0086
Word  00492,   00420,  0085
Word  00527,   00446,  0085
Word  00563,   00470,  0084
Word  00598,   00495,  0083
Word  00633,   00519,  0082
Word  00668,   00542,  0081
Word  00703,   00565,  0080
Word  00738,   00588,  0080
Word  00773,   00610,  0079
Word  00809,   00631,  0078
Word  00844,   00653,  0077
Word  00879,   00674,  0077
Word  00914,   00694,  0076
Word  00949,   00714,  0075
Word  00984,   00734,  0075
Word  01020,   00753,  0074
Word  01055,   00773,  0073
Word  01090,   00791,  0073
Word  01125,   00810,  0072
Word  01160,   00828,  0072
Word  01195,   00846,  0071
Word  01230,   00864,  0070
Word  01266,   00881,  0070
Word  01301,   00898,  0069
Word  01336,   00915,  0069
Word  01371,   00931,  0068
Word  01406,   00947,  0068
Word  01441,   00963,  0067
Word  01477,   00979,  0067
Word  01512,   00995,  0066
Word  01547,   01010,  0066
Word  01582,   01025,  0065
Word  01617,   01040,  0065
Word  01652,   01055,  0064
Word  01688,   01069,  0064
Word  01723,   01083,  0063
Word  01758,   01097,  0063
Word  01793,   01111,  0063
Word  01828,   01125,  0062
Word  01863,   01139,  0062
Word  01898,   01152,  0061
Word  01934,   01165,  0061
Word  01969,   01178,  0061
Word  02004,   01191,  0060
Word  02039,   01204,  0060
Word  02074,   01216,  0059
Word  02109,   01228,  0059
Word  02145,   01241,  0059
Word  02180,   01253,  0058
Word  02215,   01265,  0058
Word  02250,   01276,  0058
Word  02285,   01288,  0057
Word  02320,   01300,  0057
Word  02355,   01311,  0057
Word  02391,   01322,  0056
Word  02426,   01333,  0056
Word  02461,   01344,  0056
Word  02496,   01355,  0056
Word  02531,   01366,  0055
Word  02566,   01377,  0055
Word  02602,   01387,  0055
Word  02637,   01398,  0054
Word  02672,   01408,  0054
Word  02707,   01418,  0054
Word  02742,   01429,  0054
Word  02777,   01439,  0053
Word  02813,   01449,  0053
Word  02848,   01458,  0053
Word  02883,   01468,  0053
Word  02918,   01478,  0052
Word  02953,   01487,  0052
Word  02988,   01497,  0052
Word  03023,   01506,  0052
Word  03059,   01516,  0051
Word  03094,   01525,  0051
Word  03129,   01534,  0051
Word  03164,   01543,  0051
Word  03199,   01552,  0051
Word  03234,   01561,  0050
Word  03270,   01570,  0050
Word  03305,   01579,  0050
Word  03340,   01587,  0050
Word  03375,   01596,  0049
Word  03410,   01605,  0049
Word  03445,   01613,  0049
Word  03480,   01622,  0049
Word  03516,   01630,  0049
Word  03551,   01638,  0049
Word  03586,   01647,  0048
Word  03621,   01655,  0048
Word  03656,   01663,  0048
Word  03691,   01671,  0048
Word  03727,   01679,  0048
Word  03762,   01687,  0048
Word  03797,   01695,  0047
Word  03832,   01703,  0047
Word  03867,   01711,  0047
Word  03902,   01718,  0047
Word  03938,   01726,  0047
Word  03973,   01734,  0047
Word  04008,   01741,  0046
Word  04043,   01749,  0046
Word  04078,   01756,  0046
Word  04113,   01764,  0046
Word  04148,   01771,  0046
Word  04184,   01779,  0046
Word  04219,   01786,  0046
Word  04254,   01793,  0046
Word  04289,   01801,  0045
Word  04324,   01808,  0045
Word  04359,   01815,  0045
Word  04395,   01822,  0045
Word  04430,   01829,  0045
Word  04465,   01836,  0045
Word  04500,   01843,  0045
Word  04535,   01851,  0045
Word  04570,   01857,  0045
Word  04605,   01864,  0044
Word  04641,   01871,  0044
Word  04676,   01878,  0044
Word  04711,   01885,  0044
Word  04746,   01892,  0044
Word  04781,   01899,  0044
Word  04816,   01905,  0044
Word  04852,   01912,  0044
Word  04887,   01919,  0044
Word  04922,   01926,  0044
Word  04957,   01932,  0043
Word  04992,   01939,  0043
Word  05027,   01945,  0043
Word  05063,   01952,  0043
Word  05098,   01959,  0043
Word  05133,   01965,  0043
Word  05168,   01972,  0043
Word  05203,   01978,  0043
Word  05238,   01985,  0043
Word  05273,   01991,  0043
Word  05309,   01998,  0043
Word  05344,   02004,  0043
Word  05379,   02010,  0043
Word  05414,   02017,  0043
Word  05449,   02023,  0042
Word  05484,   02029,  0042
Word  05520,   02036,  0042
Word  05555,   02042,  0042
Word  05590,   02048,  0042
Word  05625,   02055,  0042
Word  05660,   02061,  0042
Word  05695,   02067,  0042
Word  05730,   02073,  0042
Word  05766,   02079,  0042
Word  05801,   02086,  0042
Word  05836,   02092,  0042
Word  05871,   02098,  0042
Word  05906,   02104,  0042
Word  05941,   02110,  0042
Word  05977,   02117,  0042
Word  06012,   02123,  0042
Word  06047,   02129,  0042
Word  06082,   02135,  0042
Word  06117,   02141,  0042
Word  06152,   02147,  0042
Word  06188,   02153,  0042
Word  06223,   02159,  0042
Word  06258,   02165,  0042
Word  06293,   02171,  0042
Word  06328,   02177,  0042
Word  06363,   02184,  0042
Word  06398,   02190,  0041
Word  06434,   02196,  0041
Word  06469,   02202,  0041
Word  06504,   02208,  0041
Word  06539,   02214,  0041
Word  06574,   02220,  0041
Word  06609,   02226,  0041
Word  06645,   02232,  0041
Word  06680,   02238,  0041
Word  06715,   02244,  0041
Word  06750,   02250,  0041
Word  06785,   02256,  0041
Word  06820,   02262,  0041
Word  06855,   02268,  0041
Word  06891,   02274,  0041
Word  06926,   02280,  0041
Word  06961,   02286,  0041
Word  06996,   02292,  0041
Word  07031,   02298,  0041
Word  07066,   02304,  0041
Word  07102,   02310,  0041
Word  07137,   02316,  0042
Word  07172,   02323,  0042
Word  07207,   02329,  0042
Word  07242,   02335,  0042
Word  07277,   02341,  0042
Word  07313,   02347,  0042
Word  07348,   02353,  0042
Word  07383,   02359,  0042
Word  07418,   02365,  0042
Word  07453,   02371,  0042
Word  07488,   02377,  0042
Word  07523,   02383,  0042
Word  07559,   02390,  0042
Word  07594,   02396,  0042
Word  07629,   02402,  0042
Word  07664,   02408,  0042
Word  07699,   02414,  0042
Word  07734,   02421,  0042
Word  07770,   02427,  0042
Word  07805,   02433,  0042
Word  07840,   02439,  0042
Word  07875,   02445,  0042
Word  07910,   02452,  0042
Word  07945,   02458,  0042
Word  07980,   02464,  0042
Word  08016,   02471,  0042
Word  08051,   02477,  0042
Word  08086,   02483,  0043
Word  08121,   02490,  0043
Word  08156,   02496,  0043
Word  08191,   02502,  0043
Word  08227,   02509,  0043
Word  08262,   02515,  0043
Word  08297,   02522,  0043
Word  08332,   02528,  0043
Word  08367,   02535,  0043
Word  08402,   02541,  0043
Word  08438,   02548,  0043
Word  08473,   02555,  0043
Word  08508,   02561,  0043
Word  08543,   02568,  0043
Word  08578,   02574,  0044
Word  08613,   02581,  0044
Word  08648,   02588,  0044
Word  08684,   02595,  0044
Word  08719,   02601,  0044
Word  08754,   02608,  0044
Word  08789,   02615,  0044
Word  08824,   02622,  0044
Word  08859,   02629,  0044
Word  08895,   02636,  0044
Word  08930,   02643,  0045
Word  08965,   02649,  0045