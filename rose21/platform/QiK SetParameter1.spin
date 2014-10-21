''=============================================================================
'' QiK Parameter setting program nov 2010 H.J. Kiela Opteq R&D BV
'' Version V1.1
'' With this program you can test Pololu QiC drives, set their parameters and test motors
'' The program communicates via a simple text based menu on the Parallax terminal.
'' Allows you to modify parameters, while the drives are in teh application
''
'' 0: Auto baud drive
'' 1: Qic List Pars
'' 2: Set Parameter Value
'' 3: Toggle Drive address
'' 4: ToggleMotor
'' 5: Get Error from drive
'' 6: Toggle QiC)
'' 7: SetSpeed(ActDrive,ActMotor)
'' 8: SetBrake(ActDrive,ActMotor)
''
''
'' Load this program into the Parallax with the QiC connected to pin 2 and 3
'' and use communication to set QiC parameters and test motors
''
'' Check settings for serial port and drive address
'' Note that Addressed and non-addressed mode are supported, to allow more drives connected
'' Uses QiK Command object
'' 
''=============================================================================

CON
'Set 80Mhz
  _clkmode=xtal1+pll16x
  _xinfreq = 5000000      'MRS1

  Version = 11
'' Led
  Led = 27

  
'' Serial port 
   CR = 13
   LF = 10
   HM = 1
   Cls = 16
   X  = 14
   CLR = 0
   TXD = 30
   RXD = 31
   Baud = 115200

'' QiK connections
   TXQ = 26                'serial out to QiC
   RXQ = 25    
  
OBJ

  ser           : "parallax_serial_terminal"            ' Serial communication object
  t             : "timing"
  QiK           : "QiK_commands"
  n             : "simple_numbers"                      ' Number to string conversion
  
Var Long MSetVel[4]
    Byte ActDrive, ActMotor
  
PUB main | lSpeed, ch
  Init
  !outa[Led]                             ' Toggle I/O Pin for debug
  ListPars(ActDrive)                   ' List parameters

  QiK.SetProtocol(1)
  
  repeat
    ser.str(string(CR,"Actual Drive selected : "))
    ser.dec(ActDrive)
    ser.tx(CR)
    ser.str(string("Actual Motor selected : "))
    ser.dec(ActMotor)
    ser.tx(CR)
    ser.str(string("Qic protocol (0=off 1=on) : "))
    ser.dec(QiK.GetProtocol)
    ser.tx(CR)
    ser.tx(CR)
    ser.str(string("0: Auto Baud",CR))
    ser.str(string("1: Qic List Pars",CR))
    ser.str(string("2: Set Parameter Value",CR))
    ser.str(string("3: Toggle Drive address",CR))
    ser.str(string("4: ToggleMotor ",CR))
    ser.str(string("5: Get Error from drive",CR))
    ser.str(string("6: Toggle QiC)",CR))
    ser.str(string("7: SetSpeed(ActDrive,ActMotor)",CR))
    ser.str(string("8: SetBrake(ActDrive,ActMotor)",CR))
    ser.str(string("9: Continuous checking)",CR))
    
    ser.tx(CR)
    ser.tx("?")
    ch:=ser.Rx
    DoCommand(ch)
    !outa[Led]                           'Toggle I/O Pin for debug
    ser.tx(" ")
    ser.tx("!")
    

' ----------------  Command loop main program ---------------------------------------
PRI DoCommand(Cmd)
  ser.str(string(CR,"Do command: "))
  ser.tx(Cmd)
  ser.tx(CR)
  case Cmd
    "0":   QiK.AutoBaud
    "1":   ListPars(ActDrive)
    "2":   EnterParamValue(ActDrive)
    "3":   ToggleDrive
    "4":   ToggleMotor
    "5":   ReadStatus(ActDrive)
    "6":   ToggleQiC
    "7":   SetSpeed(ActDrive,ActMotor)
    "8":   SetBrake(ActDrive,ActMotor)
    "9":   CheckRepeat

' ----------------  Check selected drive repeatedly ---------------------------------------
PRI CheckRepeat | lch
  ser.rxflush
  repeat until ser.RxCheck > 0
    ReadStatus(ActDrive)
    !outa[Led]                           'Toggle I/O Pin for debug
    t.Pause1ms(200)
    

' ----------------  Toggle QiC on and off ---------------------------------------
PRI ToggleQiC | lActQiC
  lActQiC:=QiK.GetProtocol
  Case lActQiC
    0: QiK.SetProtocol(1)
    1: QiK.SetProtocol(0)

' ----------------  Read drive error ---------------------------------------
PRI ReadStatus(Address) | lError
  lError:=QiK.GetError(Address)
  ser.tx(CR)
  ser.str(string("Error from drive : "))
  ser.hex(lError,2)
  ser.tx(" ")
  ser.str(n.ibin(lError,8))
  ser.tx(" ")
  ser.str(QiK.Error2Str(lError))
  ser.tx(CR)
  ser.tx(CR)

' ---------------- Toggle from Motor 0 to Motor 1 ---------------------------------------
PRI ToggleMotor
  Case ActMotor
    0: ActMotor:=1
    1: ActMotor:=0

' ----------------  Set speed of selected motor ---------------------------------------
PRI SetSpeed(Address,Motor) | lSpeed
  ser.tx(CR)
  ser.tx(CR)
  ser.str(string(CR,"Current Drive : $"))
  ser.hex(ActDrive,2)
  ser.str(string(CR,"Current Motor : "))
  ser.dec(ActMotor)
  ser.str(string(CR,"Actual speed : "))
  case ActMotor
    0: ser.dec(QiK.GetSpeedM0(Address))
    1: ser.dec(QiK.GetSpeedM1(Address))
  ser.str(string(CR,"New Speed : "))
  lSpeed:=ser.DecIn

  Case Motor
    0: QiK.SetSpeedM0(Address, lSpeed)
    1: QiK.SetSpeedM1(Address, lSpeed)



' ----------------  Set Brake of selected motor ---------------------------------------
PRI SetBrake(Address,Motor) | lBrake
  ser.tx(CR)
  ser.tx(CR)
  ser.str(string(CR,"Current Drive : $"))
  ser.dec(ActDrive)
  ser.str(string(CR,"Current Motor : $"))
  ser.dec(ActMotor)
  ser.str(string(CR,"Actual Brake : "))
  case ActMotor
    0: ser.dec(QiK.GetBrakeValue(Address, 0))
    1: ser.dec(QiK.GetBrakeValue(Address, 1))
  ser.str(string(CR,"New Brake : "))
  lBrake:=ser.DecIn

  Case Motor
    0: QiK.SetBrakeM0(Address,lBrake)
    1: QiK.SetBrakeM1(Address,lBrake)

' ----------------  Toggle from drive 0 to drive 1 ---------------------------------------
PRI ToggleDrive    
  Case ActDrive
    QiK#Drive0: ActDrive:=QiK#Drive1
    QiK#Drive1: ActDrive:=QiK#Drive2
    QiK#Drive2: ActDrive:=QiK#Drive3
    QiK#Drive3: ActDrive:=QiK#Drive0

' ----------------  Get user input for QiC parameter ---------------------------------------
PRI EnterParamValue (Drive)| R, V, Lp, Rr, Yn
  ser.RXFlush                'clear inputbuffer
  ser.tx(CR)
  ser.tx(CR)
 ' ser.tx(CLS)
  ser.str(string("New parameter ...",CR))
  Lp:=True

  repeat while Lp
    ser.str(string("Enter Par number (0..9..:..1;) Q=quit: "))
    R:=ser.rx
    ser.tx(CR)
    Lp:=!(R=="q" or R=="Q")   'Check on quit command
    R:=R-"0"  'get character digit
    ser.str(string("Current Value for "))
    ser.str(QiK.Par2Str(R))
    ser.dec(QiK.GetParameter(Drive,R))     'get parameter value
    ser.tx(CR)
    ser.str(string("Enter new Value for "))
    ser.str(QiK.Par2Str(R))
    if R == 0 #> R <# $B
      V:=ser.DecIn
      ser.tx(CR)
      ser.str(string("Program new value : "))
      V := 0 #> V <# 127                         'Limit value
      ser.dec(V)
      ser.str(string("  Are you sure ? (y/n): "))
      Yn:=ser.rx
      if Yn=="y"
        Rr:=QiK.SetParameter(Drive,R,V)          'Program new value
        ser.tx(CR)
        ser.tx(CR)
        ser.str(string("Program result : "))     'Show result
        ser.str(QiK.Par2Str(R))
        ser.tx(" ")
        ser.str(QiK.SetParRes2str(Rr))
      else
        ser.str(string("Programming skipped. ")) 'Show result
        ser.tx(CR)
        ser.str(QiK.Par2Str(R))
'       ser.str(SetParRes2str(Rr))
      ser.tx(CR)
      Lp:=False         'End loop
    
  ser.tx(CR)
  ListPars(Drive)   'List parameters
  ser.str(string("Press key to continue: "))
  R:=ser.rx
  

' ---------------------  Init program  ------------------
PRI Init
  ser.Start( Baud)                 'Start serial terminal 
' ser.start(RXD, TXD, 0, Baud)     'Start serial port   start(rxpin, txpin, mode, baudrate)
  ser.tx(Cls)

  QiK.Init(RXQ, TXQ)                'Init( TxPin, RxPin)
  ActDrive:=QiK#Drive0
  t.Pause1ms(2000)

  dira[Led]~~                      'Set I/O pin for LED to output…
  ser.tx(Cls)
  ser.str(string("QiC set parameter Rose B.V V1.1",CR))
  !outa[Led]                             'Toggle I/O Pin for debug

' ---------------------  'List Parameters Qik drive         ------------------
PRI ListPars(Address) | R, i
  ser.str(string(CR,"List parameters",CR))
  qik.rxflush 
  repeat i from 0 to 11 '' MaxParameters
    ser.dec(i)
    ser.tx(" ")
    R:=QiK.GetParameter(Address,i)  'get parameter value
    if R < 0
       ser.str(string("TIMEOUT",CR))
    ser.str(QiK.Par2Str(i))
    ser.dec(R)
    ser.tx(CR)

  R:=QiK.GetCurrentM0(Address)  
  if R < 0
       ser.str(string("TIMEOUT",CR))
  ser.str(string("M0 current : "))
  ser.dec(R)
  ser.tx(CR)

  R:=QiK.GetCurrentM1(Address)  
  if R < 0
       ser.str(string("TIMEOUT",CR))
  ser.str(string("M1 current : "))
  ser.dec(R)
  ser.tx(CR)

  R:=QiK.GetFirmWare(Address)  
  if R < 0
       ser.str(string("TIMEOUT",CR))
  ser.str(string("Firmware : "))
  ser.dec(R)
  ser.tx(CR)
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