{------------------------------------------------------------------------------
  Main program for controlling the Platform controller level 2.

 
  Ultrasone detection and height control for TSR robot using Parallax
  Propeller Demo Board with USB prop stick interface.

  This board handles the following:
    - Communication with ROS
      - Via USB Prop stick
    - Handles the ultrasound detection
      - Eight (8) I2C ultrasone ranger (Devantech SRF10)
    - Handles the height actuator
      - Pololu VNH5019 Motor Driver Carrier
      - 12-bit ADC (I2C) for determining the height
    - Storeage of data to EEPROM
      - For storing the ADC value of the medium height position

  Author:     Dennis Wokke
  Copyright:  VEDS Group 2011

  Target:     Propeller
  Compiler:   Propeller Tool

  Version history
  ---------------

  - Version 1.00       Date: 2011-08-11        Modifier: Dennis Wokke
    - Initial released version

  - Version 1.01       Date: 2011-23-11        Modifier: Dennis Wokke
    - BUG FIX: ADC was not sampled over complete range. ADC Gain was set
               wrongly to 8, must be 1. This is now fixed.
    - Added upper and lower limits to the height control
  - Version 1.10       Date: 2012-03-02        Modifier: Dennis Wokke
    - Changed: During intialisation no messages will be send. Message may only
               be send after an initial message ("$320") has been received.
    - Added:   A message ("$350,Height reached,#") will be send after the
               requested height position has been reached.

  - Version 1.20       Date: 2012-03-19        Modifier: Dennis Wokke
    - BUG FIX: Sending of message that the height position has been reached
               has been fixed. 


  - Version 1.30       Date: 2013-09-19        Modifier: Okke Hendriks
    - Added:    A message PC.str(string("$350,Waiting for command,#",13)) will be
                send when a 2nd initial message ("$320") is send.
    - Added:    Message 406 : PC.str(string("$350,Height status,")) with appended
                one of the following four numbers:
                1: Undefined position
                2: Low position
                3: Mid position
                4: High position
    - BUG FIX: Sending of message that the height position has been reached
               has been fixed (again? :P)
  - Version 1.31       Date: 2013-09-20        Modifier: Okke Hendriks
    - Added:    A message PC.str(string("$350,Waiting for command,#",13)) will be
                send after every executed command.

------------------------------------------------------------------------------}
{
  Communication commands:
  =======================
  
  - Normal operating mode
    $300 - Start Sensor COG
    $301 - Stop Sensor COG
    $302 - Show Sensor Results

  - Debug mode
    $310                            - Show available Sensors
    $311                            - Show gain values
    $312                            - Show range values
    $313                            - Store Settings to EEPROM
    $314                            - Load Settings from EEPROM
   
   
    $315,(HEX)OldAddress,(HEX)NewAddress,#  - Change Sensor address (Only 1 Sensor Connected)
    $316,Address,Value,#            - Change specific sensor gain
    $317,Address,Value,#            - Change specific sensor range
    $318                            - Load Settings from eeprom
    $319                            - Store settings in eeprom
    $320                            - Start initialization 
   
   
   
  - Respons
    $350,STRING,#                                            - Diverse respons
    $351,STRING,#                                            - ERROR Respons
    $351,DECS1,DECS2,DECS3,DECS4,DECS5,DECS6,DECS7,DECS8,#   - Respons op $301
    $352,DEBUGMODE,#                                         - Respons op $302      
    $353,1,1,1,1,1,1,1,1,#                                   - Respons op $312
   
  - Height actuator commands 
    $400 - Start Actuator COG
    $401 - Stop Actuator COG
    $402 - Get ADC value
    $403 - Get mid position
    $404 - Read adc
    $405 - Read ADC config register
    $406 - Get height status
    $410 - Go to low position
    $420 - Go to middle position
    $430 - Go to heigh position
    $440 - Stop movement
    $451 - Save mid position
    $452 - Save low position 
    $453 - Save high position
    
    $470 - Turn motor CW !WARNING! NO LIMIT, WILL KEEP GOING!
    $480 - Turn motor CCW !WARNING! NO LIMIT, WILL KEEP GOING!
    
    $490 - Version
    $499 - Reboot
}

CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
  _stack   = 500

  POS_STOP      = 1
  POS_LOW       = 2
  POS_MID       = 3
  POS_HIGH      = 4

'PC - Serial communication
  cxTXD = 31 
  cxRXD = 30 
  cxBaud = 115200                    
  MaxStr = 257             ' Stringlength is 256 + 1 for 0 termination
  LineLen = 100           ' Buffer size for incoming line
  SenderLen= 10
  Cmdlen = 10

'EEPROM
  i2cSCL        = 28
  'i2cSDA        = 29
  EEPROMAddr    = $A0
DAT
  version BYTE "1.30",0 ' Software version

OBJ

  PC        : "FullDuplexSerial_rr005"
  Height    : "height_controller"
  i2cObject : "basic_i2c_driver"

VAR
  long key
  long X
  long SensorData[8]
  long Command
  byte Value[2]
  long SpecialCommand
  byte waitInit
  long ser_req_pos
  BYTE pos_reached_send
  long enabled
  'Serial communication
  Long PC_COG
  Byte StrBuf[MaxStr]                               'String buffer for receiving chars from serial port
  Long StrSP, StrCnt, StrP, StrLen, MaxWaitTime     'Instring semaphore, counter, Stringpointer
  Byte Cmd[LineLen] ,LastPar1[CmdLen] 


'======================MAIN============================
PUB Main | lch
    enabled:=FALSE
    InitSerialComm
    EEPROMInit
    waitcnt((clkfreq/8)+cnt)

    repeat      
      'Wait until enabled
      repeat while enabled == FALSE
        CheckForSerialCommands

      main_init
      pos_reached_send := TRUE
      ser_req_pos := -1
      repeat while enabled == TRUE
        CheckForSerialCommands
    
        'Check height status
        if ((Height.get_requested_pos == Height.get_current_pos) AND pos_reached_send == FALSE)
          PC.str(string("$400,"))
          PC.dec(ser_req_pos)
          PC.str(string(",", 13))
          pos_reached_send := TRUE   

      'Not enabled any longer, disable
      Height.height_stop


PRI main_init | ii, T1, temp, lch2

      'Initialise the height controller
      PC.str(string("Initializing Height,",13))
      Height.height_init   
      if not Height.height_get_cog
        Height.height_start

      if (Height.height_get_cog)
        PC.str(string("Height COG started,",13))
      else
        repeat
            PC.str(string("Height COG Error,",13))    
            waitcnt((clkfreq/8)+cnt)
      PC.str(string("Loading settings from EEPROM,",13))
      Height.height_set_low(LoadLowPos)
      waitcnt((clkfreq/8)+cnt)
      Height.height_set_mid(LoadmidPos)                          
      waitcnt((clkfreq/8)+cnt)
      Height.height_set_high(LoadHighPos) 
      waitcnt((clkfreq/8)+cnt)
    
      PC.str(string("$350,Loaded and running,",13))
      PC.str(string("$350,Waiting for command,",13))


'================================ Init Serial comm ==========================
PRI InitSerialComm
  MaxWaitTime := 500                    'ms wait time for incoming string  
  StrSp:=0
  ByteFill(@StrBuf,0,MaxStr)
  PC_COG:=PC.start(cxTXD, cxRXD, 0, cxBaud)     'Start PC serial interface
  PC.str(string("$350,Communication COG started,",13))

'================================ Do Xbee command input and execution ==========================
PRI CheckForSerialCommands

    StrCnt++
    if PC.rxavail
      PC.StrInMaxTime(@StrBuf,MaxStr,MaxWaitTime)   'Non blocking max wait time
      PC.rxflush             
      if Strsize(@StrBuf)>3                           'Received string must be larger than 3 char's skip rest$400,1,
        DoCommand                                   'Check input string for new commands and execute them
        Bytefill(@StrBuf,0,MaxStr)                  'Clear buffer

'===============================================================
PUB DoCommand | Sender, OK, lCh, enable
  StrP:=0  'Reset line pointer
  Sender:=0
  StrLen:=strsize(@StrBuf)  
  OK:=1

  if StrLen > (MaxStr-1)        'Check max len
    OK:=-1                      'Error: String too long
    
  if StrLen == 0                'Check zero length
    OK:=-2                      'Error: Null string
    
  if OK==1                      'Parse string
    lCh:=sGetch
    repeat while (lch<>"$") and (OK == 1)       'Find start char
      lCh:=sGetch
      if StrP == StrLen
        OK:=-3                  'Error: No Command Start  found
        Quit                    'Exit loop

    if OK == 1
      Sender:=sGetPar
      Case Sender

        400 : enable:=sGetPar
              if enable == 1
                enabled:=TRUE
                  PC.str(string("$400,"))
                  PC.dec(1)
              elseif enable == 0
                  enabled:=FALSE 
                  PC.str(string("$400,"))
                  PC.dec(0)
              else
                  PC.str(string("$400,"))
                  PC.dec(-1)

              PC.str(string(",",13))

        401 : ser_req_pos:=sGetPar
              if ser_req_pos == 2
                Height.height_set_position(POS_LOW)  
              elseif ser_req_pos == 3
                Height.height_set_position(POS_MID)  
              elseif ser_req_pos == 4
                Height.height_set_position(POS_HIGH)
              else
                Height.height_set_position(POS_STOP)
              pos_reached_send := FALSE              

        402 : PC.str(string("$402,"))
              PC.dec(Height.get_current_pos)
              PC.str(string(",",13))

        403 : PC.str(string("$403,"))
              PC.dec(Height.height_get_low)
              PC.str(string(","))
              PC.dec(Height.height_get_mid)
              PC.str(string(","))
              PC.dec(Height.height_get_high)
              PC.str(string(",",13)) 

        404 : PC.str(string("$404,"))
              PC.dec(Height.height_get_position)
              PC.str(string(",",13))
  
        451 : specialcommand := Height.height_get_position
              PC.str(string("$451,"))
              PC.dec(specialcommand)
              PC.str(string(",",13))
              Height.height_set_mid(specialcommand)
              waitcnt((clkfreq/8)+cnt)

        452 : specialcommand := Height.height_get_position
              PC.str(string("$452,"))
              PC.dec(specialcommand)
              PC.str(string(",",13))
              Height.height_set_low(specialcommand)
              waitcnt((clkfreq/8)+cnt)

        453 : specialcommand := Height.height_get_position
              PC.str(string("$453,"))
              PC.dec(specialcommand)
              PC.str(string(",",13))
              Height.height_set_high(specialcommand)
              waitcnt((clkfreq/8)+cnt)
        470 : Height.turn_cw
        480 : Height.turn_ccw
                                           
        499 : PC.str(string("$499,REBOOT,",13))
              waitcnt((clkfreq/8)+cnt)
              REBOOT
        Other:PC.str(string("$",1))
              PC.dec(Sender) 
              PC.str(string(",Unknown Command,",13))      

PUB ChartoDec (input,sbit) : output

  case input
    "0"        :  output := 0
    "1"        :  output := 1
    "2"        :  output := 2     
    "3"        :  output := 3         
    "4"        :  output := 4
    "5"        :  output := 5
    "6"        :  output := 6
    "7"        :  output := 7
    "8"        :  output := 8
    "9"        :  output := 9
    "A"        :  output := 10
    "B"        :  output := 11
    "C"        :  output := 12
    "D"        :  output := 13
    "E"        :  output := 14
    "F"        :  output := 15
    "a"        :  output := 10
    "b"        :  output := 11
    "c"        :  output := 12
    "d"        :  output := 13
    "e"        :  output := 14
    "f"        :  output := 15
                                                                     
  if (sbit == 1)
    output := output * 16
    return output
  else
    return output
               
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
 
  LPar:=PC.strtodec(@LastPar1)
Return Lpar

' ---------------- Get next character from string ---------------------------------------
PRI sGetCh | lch 'Get next character from commandstring
   lch:=Byte[@StrBuf][StrP++]
Return lch

'---------------EEPROM FUNCTIONS----
PUB EEPROMInit
  'Initialize I2C communication module
  i2cObject.Initialize(i2cSCL)                                            
  ' pause 2 seconds
  repeat 2
        waitcnt((clkfreq/2)+cnt)

PUB LoadLowPos
  return i2cObject.ReadWord(i2cSCL,EEPROMAddr,$A100)

PUB LoadMidPos
  return i2cObject.ReadWord(i2cSCL,EEPROMAddr,$A000)

PUB LoadHighPos
  return i2cObject.ReadWord(i2cSCL,EEPROMAddr,$A200)

PUB StoreHeightSettings(ADC_store_value)

  ' Store the ADC value of the medium height
  i2cObject.WriteWord(i2cSCL, EEPROMAddr, $A000, ADC_store_value)

PUB StoreHeightSettingsLow(ADC_store_value1)

  ' Store the ADC value of the low height
  i2cObject.WriteWord(i2cSCL, EEPROMAddr, $A100, ADC_store_value1)

PUB StoreHeightSettingsHigh(ADC_store_value2)

  ' Store the ADC value of the high height
  i2cObject.WriteWord(i2cSCL, EEPROMAddr, $A200, ADC_store_value2)
 
  