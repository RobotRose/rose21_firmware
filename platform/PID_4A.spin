''================================PID V4  + Connect========================================
'' Qic PID Object nov 2010 HJK
'' V1 is more generic. The PID controller only does configurable number of PID-loops.
'' V2 implements separate position and velocity feedback
'' V3 Fe added, open loop added
'' Based on PID V3, but this version includes the QiK connection in one loop to save a Cog
''
'' Controls max 8 motors in velocity mode or position mode via a QiK drive (Pololu 12V10)
'' 
'' Performance in Spin: Velocity control in 8  PID loop at 3400 us for PID and 10500 us for I/O
'' Tested with 8 PID loops at 50 Hz
''
'' Febr 2011: Following error check and setpoint min max settings added and axis shut down on FE
'' Mar 2011 : In position window added
'' May 2011 : Open loop mode added, Scale factors for pos vel and output added
'' July 2011: I/O included in the loop
'' Nov 2011: doc updated, brake mode test

''=============================================================================
{ Documentation:
  This PID module implements Max 8 PID-loops at an update rate of approx 4000 us per loop
  Loop modes:
  0: Open loop
  1: Velocity mode
  2: Position mode
  3: Position mode with velocity limiter
  -1: Open loop Brake mode
  -2: Open loop Direct control of PID output via PID Setpoint 
  =============================================================================
}
CON


  PIDLed = 27         'PID test led

  PIDCnt = 8          'Max PID loop count

  _1ms  = 1_000_000 / 1_000          'Divisor for 1 ms

  cIlimit = 30000          'I- action limiter
  Outlimit = 127           'Output limiter
  nPIDLoops = 8           'Number of PID loops configured
  vel_filter_size = 1
  
  MotorCnt = nPIDLoops

  Drive0   = 10           ' Drive 0 address, 2 motor per address
  Drive1   = 11           ' Drive 1
  Drive2   = 12           ' Drive 2
  Drive3   = 13           ' Drive 3

' Quadrature encoders
  Enc0Pin  = 0            'Start pin
  EncCnt   = 8            'Number of encoders

'Serial pins QiK
  TXQ      = 26           ' Serial out to QiC
  RXQ      = 25           ' Serial in  
    
OBJ
  t             : "timing"
  QiK           : "QiK_commands"                         ' Standard serial for drives
  PosEnc        : "quadrature_encoder"                  ' Position Encoder object

Var Long PotmValue0
    long s, ms, us
    
    Long SetpAddr
    Long lActPos[PIDCnt], lActVelPos[PIDCnt], lActVel[PIDCnt], EncPos[PIDCnt]   'Actual position and velocity
    Long PosScale[PIDCnt], VelScale[PIDCnt], OutputScale[PIDCnt]
    Long PIDMode[PIDCnt]                 'PID loop mode: 0= open 1 = Vel loop 2 = Pos loop
    Long Setp[PIDCnt], SetVel[PIDCnt], preSetVel[PIDCnt]

    'PID parameters
    Long PIDMax, K[PIDCnt], KI[PIDCnt], Kp[PIDCnt], Kd[PIDCnt], Acc[PIDCnt], MaxVel[PIDCnt]
    Long ILimit[PIDCnt], lI[PIDCnt], OpenLoopCmd[PIDCnt], D[PIDCnt]
    Long PrevEncPos[PIDCnt], DVT[PIDCnt], DPT[PIDCnt]
    Long PIDStack[400]
    Long PIDTime, lPeriod, PIDLeadTime, PIDWaitTime
    Long PIDCog, PIDStatus, PIDBusy
    Word PIDCntr
    Long Pos[MotorCnt], Vel[MotorCnt], Input[MotorCnt], Output[MotorCnt]
    Long ActCurrent[PIDCnt], MaxCurrent[PIDCnt], MaxSetCurrent[PIDCnt], CurrError[PIDCnt], AnyCurrError
    Long ConnectionError[PIDCnt]
    'Velocity filter
    Long ActVelFilter[PIDCnt*vel_filter_size]
    Long vel_filter_index[PIDCnt]
    
    'Encoder vars
    Long  EncCog, EncCntr
 
    'QiK I/O parameters
    Long QiKCog, LastErr
    Long Err[PIDCnt]                     'Last error in drive

    'Limits
    Long SetpMaxPlus[PIDCnt], SetpMaxMin[PIDCnt], FE[PIDCnt], FEMax[PIDCnt], FETrip[PIDCnt], FEAny
    Long Tspeed[PIDCnt]

    Long MAEState, mMAEPos, mMAEOffset

    byte enc_semaphore
    long EncPosSemCopy[PIDCnt], EncPosSemCopy_prev[PIDCnt], enc_clk, enc_clk_prev
 
    long drive_braking_value       
'--------------------------- Start QiC PID --------------------------------
'With Period in ns
PUB Start(Period, aSetp, aMAEPos, aMAEOffset, lPIDCnt)  | i

  enc_clk_prev := 0
  enc_clk := 0
  enc_semaphore := FALSE
  bytefill(@EncPosSemCopy, 0, PIDCnt*4)
  bytefill(@EncPosSemCopy_prev, 0, PIDCnt*4)
  
  PIDMax        := lPIDCnt - 1   'Calculate loop max PID
  mMAEPos       := aMAEPos       'Store addres MAE abs encoder data
  mMAEOffset    := aMAEOffset

  lPeriod       := Period        'Save PID cycle time
  SetpAddr      := aSetp

  repeat i from 0 to PIDCnt
    EncPos[i] := 1000           'TODO OH: Why is this?

  if EncCog > 0
    cogstop(EncCog~ - 1)  
  EncCog := PosEnc.start(Enc0Pin, EncCnt, 0, @EncPos) ' Start quadrature encoder

  if QikCog > 0
    cogstop(QikCog~ - 1)  
  QikCog := QiK.Init(RXQ, TXQ)  ' Start QiK serial communication
  QiK.SetProtocol(1)            ' Enable QiK protocol  

  qik.SetSpeedM0(Drive0, 0)
  qik.SetSpeedM1(Drive0, 0)
  qik.SetSpeedM0(Drive1, 0)
  qik.SetSpeedM1(Drive1, 0)
  qik.SetSpeedM0(Drive2, 0)
  qik.SetSpeedM1(Drive2, 0)
  qik.SetSpeedM0(Drive3, 0)
  qik.SetSpeedM1(Drive3, 0)
  BrakeWheels(0)

  if PIDCog > 0
     cogstop(PIDCog~ - 1)  

  PIDCog    :=CogNew(PID(lPeriod), @PIDStack) + 1      'Start PID loop at lPeriod rate
  PIDMode   :=0

Return PIDCog


PUB Stop  
  if PIDCog > 0
     cogstop(PIDCog~ - 1)  
     PIDCog := 0

  if EncCog > 0
    cogstop(EncCog~ - 1)
    EncCog := 0

  if QikCog > 0
    cogstop(QikCog~ - 1)
    QikCog := 0

' ----------------  PID loop ---------------------------------------
PRI PID(Period) | i, j, T1, speed_time_ms, speed_distance, vel_filter_sum, drive_address ' Cycle runs every Period ms

    'Set I/O pin for LED to output
    dira[PIDLed]~~      

    Period              := 1 #> Period <# 1000000   'Limit PID period 
    PIDStatus           := 1
    
    Repeat i from 0 to PIDMax                 'Init temp vars
      PrevEncPos[i]         := EncPos[i]
      K[i]                  := 1000                         ' Loop gain Prop velocity 
      KI[i]                 := 50                           ' Loop gain I- action velocity loop
      Kp[i]                 := 1000                         ' Loop gain Position loop
      Kd[i]                 := 0                            ' Loop gain D action
      PosScale[i]           := 1                            ' Pos scale factor. Divides pos encoder input
      VelScale[i]           := 1                            ' Vel scale factor. Divides vel encoder input
      OutputScale[i]        := 1                            ' Vel scale factor. Divides vel encoder input
      Acc[i]                := 3                            ' Default acc value
      MaxVel[i]             := 250                          ' Default Max vel
      FEMax[i]              := 1100                         ' Following error limit            
      MaxSetCurrent[i]      := 1000                         ' Max allowable current
      lI[i]                 := 0                            ' Integrator value
      ILimit[i]             := cIlimit                      ' Integrator limit
      D[i]                  := 0                            ' Differentiator value
      vel_filter_index[i]   := 0                            ' Index of MAF
      repeat j from 0 to vel_filter_size
        ActVelFilter[i*vel_filter_size + j] := 0            ' Values of MAF

    ResetAllFETrip
    ResetCurrentStatus

    PIDStatus   := 2                      'PID Init done
    T1          := Cnt
    Tspeed      := Cnt

    !outa[PIDLed]                        ' Toggle I/O Pin for debug
                                         
    PIDStatus := 3                       ' PID running      

    Repeat                               ' Main PID loop
      ' Loop MAF filter index
      vel_filter_index    := vel_filter_index + 1
      if vel_filter_index => vel_filter_size
        vel_filter_index := 0

      Repeat i from 0 to PIDMax          'Cycle through the different PID loops
        'Connect sensor inputs
        case i                 
          0: lActPos[0]     := EncPos[0]                                    ' PID 0 Wheel Front Right
             lActVelPos[0]  := EncPos[0]                                    ' Velocity input loop 0
             Setp[0]        := long[SetpAddr][0]/1000                       ' Convert to pulses/ms from pulses/s
          1: lActPos[1]     := long[mMAEOffset][0] - long[mMAEPos][0]       ' PID 1 steer Front right
             lActVelPos[1]  := EncPos[1]                                    ' Vel input steer FR
             Setp[1]        := long[SetpAddr][1]

          2: lActPos[2]     := EncPos[2]                                    ' PID 2 Wheel Front Left 
             lActVelPos[2]  := EncPos[2]                                    ' Velocity input loop 2
             Setp[2]        := long[SetpAddr][2]/1000                       ' Convert to pulses/ms from pulses/s
          3: lActPos[3]     := long[mMAEOffset][1] - long[mMAEPos][1]       ' PID 3 Steer Front Left 
             lActVelPos[3]  := EncPos[3]                                    ' Velocity input loop 3
             Setp[3]        := long[SetpAddr][3]

          4: lActPos[4]     := EncPos[4]                                    ' PID 4 Wheel Rear Right
             lActVelPos[4]  := EncPos[4]                                    ' Velocity input loop 4
             Setp[4]        := long[SetpAddr][4]/1000                       ' Convert to pulses/ms from pulses/s
          5: lActPos[5]     := long[mMAEOffset][2] - long[mMAEPos][2]       ' PID 5 Steer Rear Right
             lActVelPos[5]  := EncPos[5]                                    ' Velocity input loop 5
             Setp[5]        := long[SetpAddr][5]

          6: lActPos[6]     := EncPos[6]                                    ' PID 6 Wheel Rear Left  
             lActVelPos[6]  := EncPos[6]                                    ' Velocity input loop 6
             Setp[6]        := long[SetpAddr][6]/1000                       ' Convert to pulses/ms from pulses/s
          7: lActPos[7]     := long[mMAEOffset][3] - long[mMAEPos][3]       ' PID 7 Steer Rear Left
             lActVelPos[7]  := EncPos[7]                                    ' Velocity input loop 7 steer Rear left
             Setp[7]        := long[SetpAddr][7]
        
      
        ' Moving Average Filter lActVel for drive motors      
        if(vel_filter_size > 1)
          ActVelFilter[i*vel_filter_size + vel_filter_index] := lActVelPos[i]

          vel_filter_sum := 0        
          repeat j from 0 to vel_filter_size
            vel_filter_sum := vel_filter_sum + ActVelFilter[i*vel_filter_size + j]

          lActVelPos[i] := vel_filter_sum/vel_filter_size

        'Calculate velocities D0 - D3 from delta position in [pulses/ms]
        speed_time_ms   := (Cnt-Tspeed[i])/(clkfreq/1000)
        speed_distance  := (lActVelPos[i] - PrevEncPos[i])
        lActVel[i]      := speed_distance/speed_time_ms                       
        Tspeed[i]       := Cnt
        PrevEncPos[i]   := lActVelPos[i]             
        
        'Process various PID modes
        Case PIDMode[i]                                                         
          'Open loop output command
          -2: Output[i]      := OpenLoopCmd[i]                                         
              OutputScale[i] := 1

          'Open loop 
          -1,0: Output[i]       := 0                                                   
                SetVel[i]       := 0
                lI[i]           := 0
                FE[i]           := 0
                DVT[i]          := 0
                OutputScale[i]  := 1

          ' Position mode
          3: FE[i]          := Setp[i] - lActPos[i]                                     'Current set position for limiter calculation                                 
             SetVel[i]      := -MaxVel[i] #> ( FE[i] * Kp[i]) <# MaxVel[i]
             DVT[i]         := SetVel[i] - lActVel[i]                                   'Delta Velocity
             OutputScale[i] := PosScale[i]

          ' Use active brake mode
          2: SetVel[i]      := Setp[i]                                               
             FE[i]          := SetVel[i] - lActVel[i]
             DVT[i]         := -MaxVel[i] #> FE[i] <# MaxVel[i]        'Delta Velocity
             OutputScale[i] := VelScale[i]

          ' Velocity, no brake, mode   
          1: SetVel[i]      := Setp[i]                                                  
             FE[i]          := SetVel[i] - lActVel[i]
             DVT[i]         := -MaxVel[i] #> FE[i] <# MaxVel[i]        'Delta Velocity            
             OutputScale[i] := VelScale[i]

        ' Check following error
        FETrip[i] := ||FE[i] > FEMax[i]
        FEAny     := FETrip[0] or FETrip[1] or FETrip[2] or FETrip[3] or FETrip[4] or FETrip[5] or FETrip[6] or FETrip[7] 

        if PIDMode[i] > 0                               
          'When in 'no active brake' mode, prevent I windup 
          if PIDMode[i] <> 1            
            if SetVel[i] => 0
              lI[i] := 0 #> (lI[i] + DVT[i]) <# Ilimit[i]             'Limit I-action [0, Ilimit] 
            else
              lI[i] := -Ilimit[i] #> (lI[i] + DVT[i]) <# 0            'Limit I-action [-Ilimit, 0] 
          else
            lI[i] := -Ilimit[i] #> (lI[i] + DVT[i]) <# Ilimit[i]      'Limit I-action [-Ilimit, Ilimit] 

          D[i] := DVT[i]/PIDTime                                      'Calculate D 

          Output[i]:=-Outlimit #> ((DVT[i]*K[i] + lI[i]*KI[i] + D[i]*KD[i]))/OutputScale[i]  <# Outlimit 'Calculate limited PID Out      

         ' I decay          
         ' if DVT[i] == 0
         '   if lI[i] > 0
         '     lI[i] := lI[i] - 1          
         '   else
         '     lI[i] := lI[i] + 1
        else
          D[i]      := 0
          lI[i]     := 0
          Output[i] := 0
 
        'Set drive_address  
        case i
           0, 1: drive_address := Drive0
           2, 3: drive_address := Drive1 
           4, 5: drive_address := Drive2
           6, 7: drive_address := Drive3


        case i
           0, 2, 4, 6: ' Drive motors
                       if PIDMode[i] == 2  
                         qik.SetSpeedM0DelayedReverse(drive_address, Output[i], lActVel[i])    
                       elseif drive_braking_value == 0                       'Only set a speed when not braking
                         qik.SetSpeedM0(drive_address, Output[i])
                        
                       LastErr := qik.GetError(drive_address)                'Get drive errors if any and clear error flag
                       if LastErr > 0
                         Err[i] := LastErr

                       ActCurrent[i] := qik.GetCurrentM0(drive_address)      'Get motor 0 current
 
           1, 3, 5, 7: ' Steer motors
                       if PIDMode[i] == 2  
                         qik.SetSpeedM1DelayedReverse(drive_address, Output[i], lActVel[i])
                       else
                         qik.SetSpeedM1(drive_address, Output[i])
                        
                       ActCurrent[i] := qik.GetCurrentM1(drive_address)    'Get motor 1 current

        ' Misread of current?
        if ActCurrent[i] == -1*150
          ConnectionError[i] := TRUE

        if ActCurrent[i] == -1 or ActCurrent[i] == $FF or LastErr => $10
            ActCurrent[i] := 0
            
      
            
        'if qik.GetFirmWare(Drive0) <> FIRMWARE_VERSION 
        '    ConnectionError[0] := ConnectionError[1] := TRUE
        't.Pause10us(10)  
        'if qik.GetFirmWare(Drive1) <> FIRMWARE_VERSION
        '    ConnectionError[1] := ConnectionError[2] := TRUE
        't.Pause10us(10) 
        'if qik.GetFirmWare(Drive2) <> FIRMWARE_VERSION
        '    ConnectionError[3] := ConnectionError[4] := TRUE
        't.Pause10us(10) 
        'if qik.GetFirmWare(Drive3) <> FIRMWARE_VERSION  
        '    ConnectionError[5] := ConnectionError[6] := TRUE

        'if(ConnectionError[0] or ConnectionError[1] or ConnectionError[3] or ConnectionError[5])
        '    qik.rxflush

        MaxCurrent[i]       #>= ActCurrent[i]                                       'Check for current overload 
        CurrError[i]        := CurrError[i] or (ActCurrent[i] > MaxSetCurrent[i])   'Check if any current limit exceeded set alarm if exceeded
        AnyCurrError        := AnyCurrError or CurrError[i]                         'Check if any current error

        PIDLeadTime         := (Cnt-T1)/80000                '[ms]
 
      if enc_semaphore == FALSE
        bytemove(@EncPosSemCopy_prev, @EncPosSemCopy, 32)   ' 32 = PIDCntr * 4
        bytemove(@EncPosSemCopy, @EncPos, 32)               ' 32 = PIDCntr * 4
        enc_clk_prev    := enc_clk
        enc_clk         := cnt
        enc_semaphore   := TRUE
        !outa[PIDLed]                    'Toggle I/O Pin for debug      

      PIDCntr++                                         'Update PIDCounter               
      PIDTime       := (Cnt-T1)/80000                   'Measure actual loop time in [ms] 
      PIDWaitTime   := Period - (PIDTime)     
      if(PIDWaitTime*1000 > 10)
        t.Pause10us(PIDWaitTime*100)        
       
      T1 := Cnt

' ----------------  Brake wheels  ---------------------------------------
PUB BrakeWheels(BrakeValue) | lB              
  'Stop driving motors when braking
  if BrakeValue > 0
    Qik.SetSpeedM0(Drive0, 0)
    Qik.SetSpeedM0(Drive1, 0)
    Qik.SetSpeedM0(Drive2, 0)
    Qik.SetSpeedM0(Drive3, 0)

  'Brake wheels
  QiK.SetBrakeM0(Drive0, BrakeValue)                     
  QiK.SetBrakeM0(Drive1, BrakeValue)                     
  QiK.SetBrakeM0(Drive2, BrakeValue)                    
  QiK.SetBrakeM0(Drive3, BrakeValue)   

  drive_braking_value := BrakeValue               

' ---------------- 'Reset current stuff -------------------------------
PUB ResetCurrentStatus | i
  AnyCurrError      := false
  repeat i from 0 to PIDMax
    CurrError[i]:=false
    MaxCurrent[i]:=0
    lI[i]:= 0 
    Output[i]:= 0
' ----------------  Clear errors of drives ---------------------------------------
PUB ClearErrors | i 
  repeat i from 0 to PIDMax
    Err[i]:=0

' ----------------------- Public functions -----------------------
' ---------------------  Reset ConnectionError ---------------------------
PUB ResetConnectionErrors | i
  repeat i from 0 to PIDMax
    ConnectionError[i] := FALSE

' ---------------------  Get ConnectionError ---------------------------
PUB GetConnectionError(i)
  i:= 0 #> i <# PIDMax
Return ConnectionError[i]

' ---------------------  Set Setpoint Max Min -----------------------------
PUB SetSetpMaxMin(i,lSetpMaxMin)
  i:= 0 #> i <# PIDMax
  SetpMaxMin[i]:=lSetpMaxMin
  
' ---------------------  Get Setpoint Max Min ---------------------------
PUB GetSetpMaxMin(i)
  i:= 0 #> i <# PIDMax
Return SetpMaxMin[i]
' ---------------------  Set Setpoint Max Plus -----------------------------
PUB SetSetpMaxPlus(i,lSetpMaxPlus)
  i:= 0 #> i <# PIDMax
  SetpMaxPlus[i]:=lSetpMaxPlus
  
' ---------------------  Get Setpoint Max Plus---------------------------
PUB GetSetpMaxPlus(i)
  i:= 0 #> i <# PIDMax
Return SetpMaxPlus[i]

' ---------------------  Get Setpoint ----------------------------------
PUB GetSetp(i)
  i:= 0 #> i <# PIDMax
  Return Setp[i]


' --------------------- Reset FolErr Trip -----------------------------
PUB ResetFETrip(i)
  i:= 0 #> i <# PIDMax
  FETrip[i]:=0
  
' --------------------- Reset All FolErr Trip -----------------------------
PUB ResetAllFETrip | i
  repeat i from 0 to PIDMax
    FETrip[i] := false
  FeAny := false  

' --------------------- Set Max FolErr -----------------------------
PUB SetFEMax(i,lFEMax)
  i:= 0 #> i <# PIDMax
  FEMax[i]:=lFEMax
  
' ---------------------   Get MaxFollErr -----------------------------
PUB GetFEMax(i)
  i:= 0 #> i <# PIDMax
Return FEMax[i]

' ---------------------   Get Actual FollErr -----------------------------
PUB GetFE(i)
  i:= 0 #> i <# PIDMax
Return FE[i]
' ---------------------   Get Foll Err trip -----------------------------
PUB GetFETrip(i)
  i:= 0 #> i <# PIDMax
Return FETrip[i]

PUB GetFEAnyTrip          'Any FE trip
Return FEAny

' ---------------------   Set Ki  -----------------------------
PUB SetKI(i,lKi)
  i:= 0 #> i <# PIDMax
  KI[i]:=lKi

' ---------------------   Set Kd  -----------------------------
PUB SetKD(i,lKd)
  i:= 0 #> i <# PIDMax
  Kd[i]:=lKd  

' ---------------------   Get Ki  -----------------------------
PUB GetKI(i)
  i:= 0 #> i <# PIDMax
Return KI[i]

' ---------------------   Set Kp  -----------------------------
PUB SetKp(i,lK)
  i:= 0 #> i <# PIDMax
  Kp[i]:=lK
  
' ---------------------   Get Kp  -----------------------------
PUB GetKp(i)
  i:= 0 #> i <# PIDMax
Return Kp[i]

' ---------------------   Set K   -----------------------------
PUB SetK(i,lK) ' Set K
  i:= 0 #> i <# PIDMax
  K[i]:=lK
  
' ---------------------   Get K   -----------------------------
PUB GetK(i)     'Get K
  i:= 0 #> i <# PIDMax
Return K[i]

' ---------------------   Set Acc   -----------------------------
PUB SetAcc(i,lAcc)
  i:= 0 #> i <# PIDMax
  Acc[i]:=lAcc
  
' ---------------------   Get Acc   -----------------------------
PUB GetAcc(i)
  i:= 0 #> i <# PIDMax
Return Acc[i]

' ---------------------   Set max Vel   -----------------------------
PUB SetMaxVel(i,lVel)
  i:= 0 #> i <# PIDMax
  MaxVel[i]:=lVel
  
' ---------------------   Get max Vel   -----------------------------
PUB GetMaxVel(i)
  i:= 0 #> i <# PIDMax
Return MaxVel[i]


' ---------------------   Set Position Scale factor  -----------------------------
PUB SetPosScale(i,lS)
  i:= 0 #> i <# PIDMax
  PosScale[i]:=lS
  
' ---------------------   Get PosScale -----------------------------
PUB GetPosScale(i)
  i:= 0 #> i <# PIDMax
Return PosScale[i]

' ---------------------   Set Velocity Scale factor  -----------------------------
PUB SetVelScale(i,lS)
  i:= 0 #> i <# PIDMax
  VelScale[i]:=lS
  
' ---------------------   Get  Velocity Scale factor -----------------------------
PUB GetVelScale(i)
  i:= 0 #> i <# PIDMax
Return VelScale[i]

' ---------------------   Set Output Scale factor    -----------------------------
PUB SetOutputScale(i,lS)
  i:= 0 #> i <# PIDMax
  OutputScale[i]:=lS
  
' ---------------------   Get Output Scale factor -----------------------------
PUB GetOutputScale(i)
  i:= 0 #> i <# PIDMax
Return OutputScale[i]

' ---------------------   Set Integral limiter  -----------------------------
PUB SetIlimit(i,lS)
  i:= 0 #> i <# PIDMax
  Ilimit[i]:=lS
  
' ---------------------   Get Integral limiter -----------------------------
PUB GetIlimit(i)
  i:= 0 #> i <# PIDMax
Return Ilimit[i]

' ---------------------   Return Actual Velocity Cnts/sec -----------------------------
PUB GetActVel(i)
  i:= 0 #> i <# PIDMax
  if i == 2 or i == 6
    Return -lActVel[i]
  else
    Return lActVel[i]
' ---------------------   Return Set Velocity Cnts/sec -----------------------------
PUB GetSetVel(i)
  i:= 0 #> i <# PIDMax
Return SetVel[i]
' ---------------------  Return Position in cnts -----------------------------
PUB GetActPos(i)
  i:= 0 #> i <# PIDMax
Return lActPos[i]

' ---------------------  Return Encoder Position in cnts -----------------------------
PUB GetActEncPos(i)
  i:= 0 #> i <# PIDMax
  if i == 2 or i == 6
    Return -EncPosSemCopy[i]
  else
    Return EncPosSemCopy[i]

' ---------------------  Return Encoder Position Difference in cnts in enc_clk time-----------------------------
PUB GetActEncPosDiff(i)
  i:= 0 #> i <# PIDMax
  if i == 2 or i == 6
    Return -(EncPosSemCopy[i] - EncPosSemCopy_prev[i])
  else
    Return EncPosSemCopy[i] - EncPosSemCopy_prev[i]

' ---------------------  Reset the act encoder semaphore -----------------------------
PUB resetActEncPosSem
  enc_semaphore := FALSE 
  
' ---------------------  Return Encoder Position in cnts -----------------------------
PUB getEncSem
  return enc_semaphore

' ---------------------  Return Encoder Clock difference in cnts -----------------------------
PUB getEncClkDiff
  if(enc_clk_prev > enc_clk)
    return enc_clk_prev - enc_clk
  else
    return enc_clk - enc_clk_prev

' ---------------------  Return MAE-Position in cnts -----------------------------
PUB GetMAEpos(i)
  i:= 0 #> i <# PIDMax
Return Long[mMAEPos][i]

' ---------------------  Return actual currents -----------------------------
PUB GetActCurrent(i)
  i:= 0 #> i <# PIDMax
Return ActCurrent[i]

' ---------------------  Set Max Current -----------------------------
PUB SetMaxCurr(i,lS)
  i:= 0 #> i <# PIDMax
  MaxSetCurrent[i]:=lS

' ---------------------  Return Max allowable currents -----------------------------
PUB GetMaxSetCurrent(i)
  i:= 0 #> i <# PIDMax
Return MaxSetCurrent[i]

' ---------------------  Return Max allowable currents -----------------------------
PUB GetMaxCurrent(i)
  i:= 0 #> i <# PIDMax
Return MaxCurrent[i]

' ---------------------  Return current errorss -----------------------------
PUB GetCurrError(i)
  i:= 0 #> i <# PIDMax
Return CurrError[i]

' ---------------------  Return Ibuf -----------------------------
PUB GetIBuf(i)
  i:= 0 #> i <# PIDMax
Return lI[i]

' ---------------------  Return D -----------------------------
PUB GetD(i)
  i:= 0 #> i <# PIDMax
Return D[i]

' ---------------------  Return Delta vel -----------------------------
PUB GetDeltaVel(i)
  i:= 0 #> i <# PIDMax
Return DVT[i]

' ---------------------   Set PID mode     -----------------------------
PUB SetPIDMode(i, lMode)             '0= open loop, 1=Velocity control, 2= position control 3= Pos cntrl Vel limit
  i:= 0 #> i <# PIDMax
'  if (PIDMode[i]==0 and lMode<>0)   'Do something before closing loop to avoid sudden jumps
  
  PIDMode[i] := lMode

' ---------------------   Set command output in open loop mode  ------------------------
PUB SetOpenLoop(i,lOpenloopCmd)            
  i:= 0 #> i <# PIDMax
  OpenloopCmd[i] := lOpenloopCmd

' ---------------------   Kill all motors (open loop) -------------------
PUB KillAll  | i
  repeat i from 0 to PIDMax
    PIDMode[i]:=0
    
' ---------------------   Set all motors in the same PID state  -------------------
PUB SetAllPIDMode(m)  | i
  repeat i from 0 to PIDMax
    PIDMode[i]:=m

' ---------------------  Return PID Mode -----------------------------
PUB GetPIDMode(i)
  i:= 0 #> i <# PIDMax
Return PIDMode[i]

' --------------------- Return PID Time in ms -----------------------------
PUB GetPIDTime
Return PIDTime

' --------------------- Return PIDLead Time in ms -----------------------------
PUB GetPIDLeadTime
Return PIDLeadTime
' --------------------- Return PIDWaitTime in ms -----------------------------
PUB GetPIDWaitTime
Return PIDWaitTime
' ---------------------  Return PID Status -----------------------------
PUB GetPIDStatus 
Return  PIDStatus

' ---------------------  Return PIDOut -----------------------------
PUB GetPIDOut(i) 
  i:= 0 #> i <# PIDMax
Return Output[i]

' ---------------------  Return Get QiK Parameter -----------------------------
PUB GetParameter(Address, Parameter)
Return GetParameter(Address, Parameter)

' ---------------------  Return Return string -----------------------------
PUB Par2Str(ParNr)
Return Par2Str(ParNr)

' ---------------------   Get PID Counter  -----------------------------
PUB GetCntr
Return PIDCntr

' ---------------------   GetAnyCurrError  -----------------------------
PUB GetAnyCurrError
Return AnyCurrError

'----------------- Clear AnyCurrError ------------------------
PUB ClearAnyCurrError | i
  repeat i from 0 to PIDMax
    CurrError[i]:=false
  AnyCurrError:=false
  CurrError:=0
    
    
' ---------------------   Get drive Error  -----------------------------
PUB GetError(i)
  i:= 0 #> i <# PIDMax
Return Err[i]

' ---------------------   GetEncCog  -----------------------------
PUB GetEncCog
Return EncCog

' ---------------------   GetQIKCog  -----------------------------
PUB GetQIKCog
Return QIKCog

' ---------------------   GetEncCntr
PUB GetEncCntr
Return EncCntr



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