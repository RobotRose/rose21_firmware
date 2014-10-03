''=============================================================================
'' Qic object nov 2010 H.J. Kiela Opteq R&D BV
'' V1.1a
'' Minor bug fix
'' This objects holds all methods to operate the QiK drives
'' All drives can be daisy chained, by assigning them unique addresses.
'' The Rx lines can be joined. The Tx lines must be connected via open collector buffers (SN7407 f.e.)
''
'' Use program Qik SetParameters to set address and parameters
''
'' Nov 2011 HJK
'' Time out of response returns negative value -1 from communication. Has to be dealt with one level higher
''=============================================================================

CON

'Pololu QiC serial protocol. For Pololu commands (High bit = 0 otherwise address is ignored)!
  'Serial in parameters
   BaudQ = 115200
   
   Drive0 = 10            ' Drive address constants
   Drive1 = 11            
   Drive2 = 12            
   Drive3 = 13            

  ' Commands
   cM0F = $88              'Motor M0 Forward 
   cM0R = $8A              'Motor M0 reverse 
   cM1F = $8C              'Motor M1 Forward 
   cM1R = $8E              'Motor M1 reverse
   cM0B = $86              'Motor M0 Brake
   cM1B = $87              'Motor M1 Brake
   
   'Get info
   cGetM0Current = $90     'Get motor 0 current 0-127 150mA/unit
   cGetM1Current = $91     'Motor 1
   cGetM0Speed   = $92
   cGetM1Speed   = $93
   cGetError     = $82     'Get Error Byte
   cGetFirmware  = $81     'Get Firmware Version
   cGetPar       = $83     'Get Configuration Parameter
   cSetPar       = $84     'Set Configuration Parameter
   
   'Error bits
   cMotor0Fault   = $01    'bit 0: Motor 0 Fault
   cMotor1Fault   = $02    'bit 1: Motor 1 Fault
   cMotor0Current = $04    'bit 2: Motor 0 Over Current
   cMotor1Current = $08    'bit 3: Motor 1 Over Current
   cSerialError   = $10    'bit 4: Serial Hardware Error
   cCRCError      = $20    'bit 5: CRC Error  
   cFormatError   = $40    'bit 6: Format Error
   cTimeOut       = $80    'bit 7: Timeout
   
  'Return constants on status request
  cDeviceID        = 0
  cPWMParameter    = 1
  cShutDownonError = 2
  cSerialTimeout   = 3
  cMotorM0Acc      = 4
  cMotorM1Acc      = 5
  cMotorM0BrakeDur = 6
  cMotorM1BrakeDur = 7
  cMotorM0CurLimit = 8
  cMotorM1CurLimit = 9
  cMotorM0CurLimResp = 10
  cMotorM1CurLimResp = 11
  MaxParameters = 11

  nMotor = 8    'Max number of motors for brake array. Modify when more needed

  TimeOut = 1  'Timeout in ms for response by Qik on request
  
  speed_hyst = 5
  max_braking_effort = 25
      
OBJ
  serial_interface  : "full_duplex_serial_005"       ' Standard serial communication
  t                 : "timing"
  
Var Byte  ActQiK
    Byte  lTxPin, lRxPin
    Byte  s[256]   ' errorstring
    Long  MotorBrake[NMotor], MotorPWM[Nmotor], motor_prev_direction[NMotor]

' ---------------------  Init QiK object  ------------------
PUB Init( RxPin, TxPin)| lCog, i
  
  repeat i from 0 to (NMotor - 1)
    motor_prev_direction[i] := 0
    
  lTxPin    := TxPin
  lRxPin    := RxPin
  lCog      := serial_interface.start(lRxPin, lTxPin, 0, BaudQ)  ' Start serial port   start(rxpin, txpin, mode, baudrate)
  ActQiK    := 1                           ' 0=Compact protocol 1=Pololu protocol, multi drop daisy chain
  AutoBaud
Return lCog   
' ---------------------  'Auto baud Qik  ------------------
PUB AutoBaud             'Perform this command first to get baud rate rigth of all QiK drives
  serial_interface.tx($AA)            'Auto baud character   

' ---------------------  'Set/Get communication protocol  ------------------
PUB SetProtocol(lQiK)    '0=Compact protocol 1=Pololu protocol for multi drop daisy chain
  ActQiK:=0 #> lQiK <# 1

PUB GetProtocol          'Return actual protocol
Return ActQiK

' ---------------------  'Flush receive buffer ------------------
PUB rxflush   
  serial_interface.rxflush 
  
' -- Givien the motor addresses run from 10 - 14
PRI getMotorIndex(address, motor_id) | lowest_address, diff
  lowest_address := 10
  diff           := address - lowest_address
  return (address - lowest_address + motor_id + diff)

' ----------------  QiC commands motor commands pololu drivers -----
' ---------------------  'Set new speed motor 0 (active brake)  ------------------
PUB SetSpeedM0(Address, ReqEffort) | abs_req_eff, direction
  if ReqEffort < 0
    direction   := cM0R
    abs_req_eff := 0 #> -ReqEffort <# 127 'Limit range
  else
    direction   := cM0F
    abs_req_eff := 0 #> ReqEffort <# 127 'Limit range
    
  if ActQiK==1
    serial_interface.tx($AA)    
    serial_interface.tx(Address)
    direction := direction - $80

  serial_interface.tx(direction) 'Motor speed command
  serial_interface.tx(abs_req_eff)    
  
  ' Store direction for this address
  motor_prev_direction[getMotorIndex(Address, 0)] := direction                    

' ---------------------  'Set new speed motor 0, brake instead of immediatly switching ------------------
PUB SetSpeedM0DelayedReverse(Address, ReqEffort, measured_speed, setp) | abs_req_eff, direction, apply_brake_value
  ' Initialize command to previous direction
  direction         := motor_prev_direction[getMotorIndex(Address, 0)]
  apply_brake_value := 0
  
  ' Take abs value of speed (because we determine direction not by sign but by command)
  ' Also limit to valid range 
  abs_req_eff     := 0 #> (|| ReqEffort) <# 127
  
 {{ 
  if setp => 0
    direction := cM0F
    if ReqEffort < 0
      abs_req_eff := 0
  else
    direction := cM0R
    if ReqEffort > 0
      abs_req_eff := 0   
}}

  if setp > 0 'and measured_speed > -speed_hyst
    direction := cM0F
    if ReqEffort < 0
      abs_req_eff := 0
      apply_brake_value := 0 #> ||(setp - measured_speed) <# 127
  elseif setp < 0 'and measured_speed < speed_hyst
    direction := cM0R
    if ReqEffort > 0
      abs_req_eff := 0
      apply_brake_value := 0 #> ||(setp - measured_speed) <# 127
  elseif || measured_speed > speed_hyst
     abs_req_eff := 0
    apply_brake_value := 0 #> ||(setp - measured_speed) <# 127
  else
    abs_req_eff := 0
    apply_brake_value := 0
  
  ' Store direction for this address
  motor_prev_direction[getMotorIndex(Address, 0)] := direction
        
  ' If not braking, apply direction and requested speed
  if apply_brake_value == 0
    ' Check if we need to adapt the protocol for multi qik communication
    if ActQiK == 1
      serial_interface.tx($AA)    
      serial_interface.tx(Address)
      direction := direction - $80
    
    serial_interface.tx(direction) ' Send motor speed command
    serial_interface.tx(abs_req_eff)                        
  else
    SetBrakeM0(Address, apply_brake_value)    
 
' ---------------------  'Set new speed motor 1   ------------------
PUB SetSpeedM1(Address, ReqEffort) | abs_req_eff, direction 
  If ReqEffort<0
    direction   := cM1R
    abs_req_eff := 0 #> -ReqEffort <# 127 'Limit range
  else
    direction   := cM1F
    abs_req_eff := 0 #> ReqEffort <# 127 'Limit range

  if ActQiK==1
    serial_interface.tx($AA)    
    serial_interface.tx(Address)
    direction := direction - $80

  serial_interface.tx(direction) 'Motor speed command
  serial_interface.tx(abs_req_eff) 
 
  ' Store direction for this address
  motor_prev_direction[getMotorIndex(Address, 1)] := direction                       


' ---------------------  'Set new speed motor 1, change of direction, passive brake instead  ------------------
PUB SetSpeedM1DelayedReverse(Address, ReqEffort, measured_speed, setp) | abs_req_eff, direction, apply_brake_value
  ' Initialize command to previous direction
  direction         := motor_prev_direction[getMotorIndex(Address, 1)]
  apply_brake_value := 0
  
  ' Take abs value of speed (because we determine direction not by sign but by command)
  ' Also limit to valid range 
  abs_req_eff     := 0 #> (|| ReqEffort) <# 127
  
 {{ 
  if setp => 0
    direction := cM1F
    if ReqEffort < 0
      abs_req_eff := 0
  else
    direction := cM1R
    if ReqEffort > 0
      abs_req_eff := 0   
}}

  if setp > 0 'and measured_speed > -speed_hyst
    direction := cM1F
    if ReqEffort < 0
      abs_req_eff := 0
      apply_brake_value := 0 #> ||(setp - measured_speed) <# 127
  elseif setp < 0 'and measured_speed < speed_hyst
    direction := cM1R
    if ReqEffort > 0
      abs_req_eff := 0
      apply_brake_value := 0 #> ||(setp - measured_speed) <# 127
  elseif || measured_speed > speed_hyst
    abs_req_eff := 0
    apply_brake_value := 0 #> ||(setp - measured_speed) <# 127
  else
    abs_req_eff := 0
    apply_brake_value := 0
  
  ' Store direction for this address
  motor_prev_direction[getMotorIndex(Address, 1)] := direction
        
  ' If not braking, apply direction and requested speed
  if apply_brake_value == 0
    ' Check if we need to adapt the protocol for multi qik communication
    if ActQiK == 1
      serial_interface.tx($AA)    
      serial_interface.tx(Address)
      direction := direction - $80
    
    serial_interface.tx(direction) ' Send motor speed command
    serial_interface.tx(abs_req_eff)                        
  else
    SetBrakeM1(Address, apply_brake_value)
      
{{PUB SetSpeedM1DelayedReverse(Address, ReqSpeed, measured_speed) | lS, NewCommand, apply_brake_value
 
  ' Change to reverse or brake?
  if measured_speed =< 0 and ReqSpeed =< 0 
    NewCommand        := cM1R                   ' Change direction
    lS                := 0 #> -ReqSpeed <# 127  ' Limit range
    apply_brake_value := 0
  elseif measured_speed > 0 and ReqSpeed =< 0
    NewCommand        := cM1R                   ' Wait to brake before change direction
    lS                := 0            
    apply_brake_value := 0 #> -ReqSpeed <# 127

  ' Change to forward or brake?
  elseif measured_speed => 0 and ReqSpeed => 0
    NewCommand        := cM1F                   ' Change direction
    lS                := 0 #> ReqSpeed <# 127   ' Limit range
    apply_brake_value := 0
  elseif measured_speed < 0 and ReqSpeed => 0
    NewCommand        := cM1F                   ' Wait to brake before change direction
    lS                := 0            
    apply_brake_value := 0 #> ReqSpeed <# 127    

  if apply_brake_value == 0
      if ActQiK==1
        serial_interface.tx($AA)    
        serial_interface.tx(Address)
        NewCommand := NewCommand - $80
    
      serial_interface.tx(NewCommand) 'Motor speed command
      serial_interface.tx(lS)                        
  else
    SetBrakeM1(Address, apply_brake_value)
}}

' ---------------------  'Set Braking motor 0 ---------------------
PUB SetBrakeM0(Address, Brake) | lS, NewCommand 
  lS:= 0 #> Brake <# 127 'Limit range
  NewCommand:=cM0B       'Motor Brake command
  if ActQiK==1
    serial_interface.tx($AA)    
    serial_interface.tx(Address)
    NewCommand:=NewCommand - $80
  serial_interface.tx(NewCommand) 
  serial_interface.tx(lS)
  MotorBrake[getMotorIndex(Address, 0)]:=Brake  'Store actual brake value             

' ---------------------  'Set Braking motor 1 ----------------------
PUB SetBrakeM1(Address,Brake) | lS, NewCommand 
  lS:= 0 #> Brake <# 127 'Limit range
  NewCommand:=cM1B  'Motor Brake command
  if ActQiK==1
    serial_interface.tx($AA)    
    serial_interface.tx(Address)
    NewCommand:=NewCommand - $80

  serial_interface.tx(NewCommand) 
  serial_interface.tx(lS)                        
  MotorBrake[getMotorIndex(Address, 1)]:=Brake  'Store actual brake value             
                                    
' ---------------------  'Set Parameter  ---------------------------
PUB SetParameter(Address,Parameter, Value) | lS, NewCommand, R
  lS:= 0 #> Parameter <# 11 'Limit range
  NewCommand:=cSetPar 'Set parameter command
  if ActQiK==1
    serial_interface.tx($AA)    
    serial_interface.tx(Address)
    NewCommand:=NewCommand - $80

  serial_interface.tx(NewCommand) 
  serial_interface.tx(Parameter) 'Get requested parameter
  serial_interface.tx(Value)
  serial_interface.tx($55)      'extra bytes for security
  serial_interface.tx($2A)
  R:=serial_interface.rxtime(TimeOut) 'Wait for return charater before continuing max 100 us
Return R           'Return result of parameter set     Check with SetParRes2str(Resnr) result

' --------------------- 'Get Parameter  ----------------------------
PUB GetParameter(Address, Parameter) | R, NewCommand
  NewCommand:=cGetPar 'Get parameter command
  if ActQiK==1
    serial_interface.tx($AA)    
    serial_interface.tx(Address)
    NewCommand:=NewCommand - $80
  serial_interface.tx(NewCommand) 
  serial_interface.tx(Parameter) 'Get requested parameter
  R:=serial_interface.rxtime(TimeOut) 'Expect response within timeout
Return  R


' --------------------- 'Get motor 0 current -----------------------
PUB GetCurrentM0(Address) | R, NewCommand
  NewCommand:=cGetM0Current 'Get current M0
  if ActQiK==1
    serial_interface.tx($AA)    
    serial_interface.tx(Address)
    NewCommand:=NewCommand - $80
  serial_interface.tx(NewCommand) 
  R:=serial_interface.rxtime(TimeOut)     'Expect response within timeout 

  Return R*150            'Scale output to mA

' ---------------------  'Get  motor 1 current    ------------------
PUB GetCurrentM1(Address) | R, NewCommand
  NewCommand:=cGetM1Current 'Get current M1
  if ActQiK==1
    serial_interface.tx($AA)    
    serial_interface.tx(Address)
    NewCommand:=NewCommand - $80
  serial_interface.tx(NewCommand) 
  R:=serial_interface.rxtime(TimeOut)     'Expect response within timeout   

  Return R*150            'Scale output to mA

' ---------------------  'Get  motor 0 speed      ------------------
PUB GetSpeedM0(Address) | R, NewCommand
  NewCommand:=cGetM0Speed 'Get speed M0
  if ActQiK==1
    serial_interface.tx($AA)    
    serial_interface.tx(Address)
    NewCommand:=NewCommand - $80
  serial_interface.tx(NewCommand) 
  R:=serial_interface.rxtime(TimeOut)     'Expect response within timeout   
Return R

' ---------------------  'Get  motor 1 speed      ------------------
PUB GetSpeedM1(Address) | R, NewCommand
  NewCommand:=cGetM1Speed 'Get speed M1
  if ActQiK==1
    serial_interface.tx($AA)    
    serial_interface.tx(Address)
    NewCommand:=NewCommand - $80
  serial_interface.tx(NewCommand) 
  R:=serial_interface.rxtime(TimeOut)     'Expect response within timeout   
Return R

' ---------------------  'Get current motor brake value ------------------
PUB GetBrakeValue(Address, motor_id)
  return MotorBrake[getMotorIndex(Address, motor_id)] 

' ---------------------  'Get  firmware             ------------------
PUB GetFirmWare(Address) | R, NewCommand
  NewCommand:=cGetFirmware 'Get firmware
  if ActQiK==1
    serial_interface.tx($AA)    
    serial_interface.tx(Address)
    NewCommand:=NewCommand - $80
  serial_interface.tx(NewCommand) 
  R:=serial_interface.rxtime(TimeOut)     'Expect response within timeout   
Return R

' ---------------------  'Get  error              ------------------
PUB GetError(Address) | R, NewCommand
  NewCommand:=cGetError     'Get errors
  if ActQiK==1
    serial_interface.tx($AA)    
    serial_interface.tx(Address)
    NewCommand:=NewCommand - $80
  serial_interface.tx(NewCommand) 
  R:=serial_interface.rxtime(TimeOut)     'Expect response within timeout   
Return R

' ---------------------  Return QiC errorstring -----------------------------
PUB Error2Str(Error)| lBuf
   ByteFill(@s,0,256)          'Init result string 
  'Extend with multiple messages in one string
    if (Error AND cMotor0Fault)   == cMotor0Fault
       AddStr(@sMotor0Fault, @s)

    if (Error AND cMotor1Fault)   == cMotor1Fault
       AddStr(@sSpace, @s)
       AddStr(@sMotor1Fault, @s)

    if (Error AND cMotor0Current) == cMotor0Current
       AddStr(@sSpace, @s)
       AddStr(@sMotor0Current, @s)

    if (Error AND cMotor1Current) == cMotor1Current
       AddStr(@sSpace, @s)
       AddStr(@sMotor1Current, @s)

    if (Error AND cSerialError)   == cSerialError
       AddStr(@sSpace, @s)
       AddStr(@sSerialError, @s)

    if (Error AND cCRCError)      == cCRCError
       AddStr(@sSpace, @s)
       AddStr(@sCRCError, @s)

    if Error AND cFormatError   == cFormatError
       AddStr(@sSpace, @s)
       AddStr(@sFormatError, @s)

    if Error AND cTimeOut       == cTimeOut
       AddStr(@sSpace, @s)
       AddStr(@sTimeOut, @s)

    if Error == 0
       AddStr(@sSpace, @s)
       AddStr(@sNoError, @s)
Return @s                        'return error strings

' ---------------------  Add string1 to String2 in returns strin -----------------------------
PUB AddStr(StrAddr1, StrAddr2) | i, j, ch
 i:=0
 j:=0

 repeat while byte[strAddr2][i]>0 and i<128  'search for end of destination string
   i++
 
 repeat while byte[strAddr1+j]>0 and i<128 'search for end of first string
   ch:=byte[strAddr1][j]
   byte[straddr2][i+j]:=ch
   j++

' ---------------------  Return QiC Parameter string -----------------------------
PUB Par2Str(ParNr)

    Case ParNr 
       cDeviceID:               Return @sDeviceID
       cPWMParameter:           Return @sPWMParameter
       cShutDownonError:        Return @sShutDownonError
       cSerialTimeout:          Return @sSerialTimeout
       cMotorM0Acc:             Return @sMotorM0Acc
       cMotorM1Acc:             Return @sMotorM1Acc
       cMotorM0BrakeDur:        Return @sMotorM0BrakeDur
       cMotorM1BrakeDur:        Return @sMotorM1BrakeDur
       cMotorM0CurLimit:        Return @sMotorM0CurLimit
       cMotorM1CurLimit:        Return @sMotorM1CurLimit
       cMotorM0CurLimResp:      Return @sMotorM0CurLimResp
       cMotorM1CurLimResp:      Return @sMotorM1CurLimResp
       Other :                  Return @sUnknown

' ---------------------  Return QiC command response string -----------------------------
PUB SetParRes2str(Resnr)
  Case Resnr
    0: Return @sCommandOK
    1: Return @sBadParameter
    2: Return @sBadvalue
    -1: Return @sNoResponse
    Other: Return @sUnknown
    
DAT
sMotor0Fault    Byte "Motor 0 Fault",0
sMotor1Fault    Byte "Motor 1 Fault",0
sMotor0Current  Byte "Motor 0 Over Current",0
sMotor1Current  Byte "Motor 1 Over Current",0
sSerialError    Byte "Serial Hardware Error",0
sCRCError       Byte "CRC Error",0
sFormatError    Byte "Format Error",0
sTimeOut        Byte "Timeout",0
sNoError        Byte "No Error",0

' Set par result. Wait for response before next command
sCommandOK      Byte "0: Command OK (success)",0
sBadParameter   Byte "1: Bad Parameter (failure due to invalid parameter number)",0
sBadvalue       Byte "2: Bad value (failure due to invalid parameter value for the specified parameter number)",0
sNoResponse     Byte "No Response",0

sDeviceID          Byte "DeviceID: ",0
sPWMParameter      Byte "PWMParameter: ",0
sShutDownonError   Byte "ShutDownon Error: ",0
sSerialTimeout     Byte "Serial Timeout Error: ",0
sMotorM0Acc        Byte "MotorM0Acc: ",0
sMotorM1Acc        Byte "MotorM1Acc: ",0
sMotorM0BrakeDur   Byte "MotorM0BrakeDur: ",0
sMotorM1BrakeDur   Byte "MotorM1BrakeDur: ",0
sMotorM0CurLimit   Byte "MotorM0CurLimit: ",0
sMotorM1CurLimit   Byte "MotorM1CurLimit: ",0
sMotorM0CurLimResp Byte "MotorM0CurLimResp: ",0
sMotorM1CurLimResp Byte "MotorM1CurLimResp: ",0
sUnknown           Byte "Unknown Parameter! ",0
sSpace             Byte " , ", 0

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