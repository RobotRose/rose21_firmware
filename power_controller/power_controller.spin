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
'' SEE accompanied TXT file for communication specification
''
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

DAT   
  
CON
   ' Version
   major_version    = 0
   minor_version    = 1 
   CONTROLLER_ID    = 3

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
  Cmdlen = 20
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

'ADC channels
   NCh = 16              'Number of measuring channels

  'ADC1   
   cNTC       = 0
   NC1        = 1        'not used
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

' Safety related values
  c5V = 4800                 ' Minimal 5V supply
  c3V3 = 3200                ' Minimal 3V3 supply
  cMinVin = 2200             ' Absolute minimal supply voltage

  main_led_interval = 250    ' [ms]
                              
  default_output = 1         ' PC1 turns on automatically at startup                             
  
OBJ
  ADC             : "MCP3208_fast_multi"                  ' ADC
  io_manager      : "io_manager"                          ' IO management cog
  sound           : "sound"                               ' Sound 
  ser             : "full_duplex_serial_005"              ' Full duplex serial communication 
  t               : "Timing"
  num             : "simple_numbers"                      ' Number to string conversion
  STRs            : "STRINGS2hk"
  f               : "FloatMath1_1"                        ' Floating point library
  
  
VAR
  LONG ADCCog, ADC2Cog, SerCog, ADCMCog, ADCMStack[50], io_manager_cog
  LONG PLCCog, PLCStack[50], LinMotCog
  Long ADCRaw[NCh], ADCRawMin[NCh], ADCRawMax[NCh], ADCRawAvg[NCh]  ' Raw bit values
  Long ADCch[NCh], ADCchAVG[NCh], ADCchMAX[NCh], ADCchMIN[NCh]      ' Volt
  Long engADCch[NCh], engADCchAVG[NCh], engADCchMAX[NCh], engADCchMIN[NCh] ' Scaled values 
  LONG Switchstate, MainCnt, MaxCh, ADCTime, ADCCnt, DoMotorCnt, ScalingCnt, ScalingTime

  ' Safety related vars
  Long AllOK, s5VOK, s3V3OK, sVInOK, OutofRange, PlcCnt

  Long Scale[Nch]   '' array with scaling factors

  ' Vars for command handling and processing
  Long ConnectCnt, ConnectTime    
  byte data[bytebuffersize]     'Receive buffer
  long DataLen                  'Buffer len
  Byte StrMiscReport[lMaxStr]   'Report string various other data
  Long CommandTime, ConnectionTime
  Long command 
  
  byte last_parameter[CmdLen]
  
  ' Input string handling
  Byte StrBuf[lMaxStr]  'String buffer for receiving chars from serial port
  Long StrSP, StrP, StrLen        'Instring semaphore, counter, Stringpointer, String length
  Byte Ch
  long MaxWaitTime

  ' Debug
  LONG Debug

  ' Watchdog
  long expected_wd, wd, wd_cnt
  
  ' Safety
  long SafetyCog, SafetyStack[40], SafetyCntr, no_alarm, last_alarm
  
  ' Debug
  long DebugCog, DebugStack[40]
  
  ' Errors
  long wd_cnt_threshold
 

PUB Main
  Init

  sound.BeepHz(BUZZ, NOTE_A4, 1000000/16)
  sound.BeepHz(BUZZ, NOTE_A5, 1000000/16)
  sound.BeepHz(BUZZ, NOTE_A6, 1000000/16)
  sound.BeepHz(BUZZ, NOTE_A7, 1000000/16)
  sound.BeepHz(BUZZ, NOTE_F7, 1000000/20)
  sound.BeepHz(BUZZ, NOTE_G7, 1000000/16)
  
  ' Run startup sequence
  startup
  
  ' Main loop
  repeat
    MainCnt++

    handleCommunication 
    
    ' Check if batteries and low and give alarm singal with a certain interval
    if io_manager.getBatteriesLow AND io_manager.getAlarmSound
      if io_manager.getAlarmIntervalTimer => io_manager.getAlarmInterval
        sound.lowVoltageWarning(BUZZ)
        io_manager.setAlarmIntervalTimer(0)      
    else
      io_manager.setAlarmIntervalTimer(0)
     
    ' Indicate that the main loop is running   
    if io_manager.getLedIntervalTimer => main_led_interval
      !OUTA[Led]
      io_manager.setLedIntervalTimer(0)

PRI handleCommunication
  ser.StrInMaxTime(@StrBuf, lMaxStr, MaxWaitTime)   'Non blocking max wait time
  
  if Strsize(@StrBuf) > 3                         'Received string must be larger than 3 char's skip rest (noise)
    DoCommand   'Execute command in received string

' === Main initialization === 
PRI Init
  Debug := false 

  'Reset all min/max values
  resetAllADCVals

  ' Error tresholds (timing 1 count is 1*ms) default values
  wd_cnt_threshold                   := 1000  
  
  InitWatchDog
  
  'Initialize serial communication 
  InitSer
  
  ser.str(string("Main Cog running in cog: "))
  ser.dec(cogid + 1)
  ser.char(CR)

  if ADCCog > 0
    ADC.stop
  ADCCog:= ADC.Start(dpin1, cpin1, spin1, dpin2, cpin2, spin2, dpin3, cpin3, spin3)
  if ADCCog
    ser.str(string("Started ADCCog("))
    ser.dec(ADCCog)
    ser.str(string(")", CR))
  else
    ser.str(string("Unable to start ADCCog", CR))
    
  MaxCh:= NCh-1

  OUTA[Led]:=1
  DirA[Led] ~~     'Set indicator led as output

  sound.init(BUZZ)  

  SwitchState:=0

  if ADCMCog > 0
    cogstop(ADCMCog~ - 1)
  ADCMCog:=cognew(DoADC,@ADCMStack) + 1
  
  if ADCMCog
    ser.str(string("Started ADCMCog("))
    ser.dec(ADCMCog)
    ser.str(string(")", CR))
  else
    ser.str(string("Unable to start ADCMCog", CR))
  
  if PlcCog > 0
    cogstop(PlcCog~ - 1) 
  PlcCog:=cognew(DoPLC,@PLCStack) + 1            'Start PLC cog
  
  if PlcCog
    ser.str(string("Started PlcCog("))
    ser.dec(PlcCog)
    ser.str(string(")", CR))
  else
    ser.str(string("Unable to start PlcCog", CR))
      
  if DebugCog > 0
    cogstop(DebugCog~ - 1)  
  DebugCog := CogNew(DoDisplay, @DebugStack) + 1
  
  if DebugCog
    ser.str(string("Started DebugCog("))
    ser.dec(DebugCog)
    ser.str(string(")", CR))
  else
    ser.str(string("Unable to start DebugCog", CR))
  
  io_manager_cog := io_manager.start
  if io_manager_cog
    ser.str(string("Started io_managerCog("))
    ser.dec(io_manager_cog)
    ser.str(string(")", CR))
  else
    ser.str(string("Unable to start io_managerCog", CR))
    
  io_manager.setBatterySwitchSound(true)
  io_manager.setAutoBatterySelect(true)
  
  if SafetyCog > 0
    cogstop(SafetyCog~ - 1)  
  SafetyCog := CogNew(DoSafety, @SafetyStack) + 1
  
  if SafetyCog
    ser.str(string("Started SafetyCog("))
    ser.dec(SafetyCog)
    ser.str(string(")", CR))
  else
    ser.str(string("Unable to start SafetyCog", CR))
 
' === Shutdown sequence ===  
PRI shutdown
  ser.str(string("Should really shutdown now!", CR))

' === Startup sequence ===   
PRI startup | selected_battery
  
  ser.str(string("Selecting fullest battery, bat1: "))  
  ser.dec(io_manager.getBatteryVoltageAvg(1))
  ser.str(string("mV | bat2: "))
  ser.dec(io_manager.getBatteryVoltageAvg(2))
  ser.str(string("mV -> "))
  selected_battery := io_manager.selectFullestBattery
  ser.dec(selected_battery)
  ser.char(CR)
  if selected_battery <> 0
    ser.str(string("Battery selected, turning on default output.", CR))
    io_manager.setSwitch(default_output, true)
  else
    ser.str(string("Batteries not charged, not turning on default output.", CR))
    
' === Reset communication ===
PRI reset_communication
  InitWatchDog
  resetSafety
  
' === Init serial communication ===
PRI InitSer
  MaxWaitTime   := 5                    'ms wait time for incoming string 'TODO take this lower (5ms when comm with PC) 
  
  ByteFill(@StrBuf,0, lMaxStr)

  if SerCog > 0
    ser.Stop
  SerCog := ser.start(rxd, txd, 0, baud)     'serial port on prop plug
  
  if SerCog
    ser.str(string("Started SerCog("))
    ser.dec(SerCog)
    ser.str(string(")", CR))
  else
    ser.str(string("Unable to start SerCog", CR))

  return SerCog

'=== Init Watchdog ===
PRI InitWatchDog
  expected_wd   := 0                     
  wd            := 0
  wd_cnt        := 0

PRI handleWatchdogError | i
  i := 0
  repeat 6
    if i <> default_output
      io_manager.setSwitch(i, false)
    i++
  
'***************************************  Handle command string received from client
PRI DoCommand | commandOK, i, j, Par1, Par2, lCh, t1, c1, req_id, received_wd, temp, temp2
  t1:=cnt
  commandOK:=1

  StrP:=0                       'Reset line pointer
  command:=0                     'Reset received command
  StrLen:=strsize(@StrBuf)      'Determine length of string in the buffer
  temp:=0
  temp2:=0

  if StrLen > (lMaxStr-1)       'Check max len
    commandOK:=-1                 'Error: String too long
  elseif StrLen == 0            'Check zero length
    commandOK:=-2                 'Error: Null string
  else
    'Get command
    command:=sGetPar  
    
    if Debug
      ser.str(string("Received command: ")) 
      ser.dec(command)
      ser.str(CR)   

    Case command
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
        111: received_wd := sGetPar
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
                
        ' === Get commands ===        
        ' Get power output states
        200: ser.str(string("$200,"))
             ser.dec(io_manager.GetCONTR)
             ser.str(string(","))
             ser.dec(io_manager.GetPC1)
             ser.str(string(","))
             ser.dec(io_manager.GetPC2)
             ser.str(string(","))
             ser.dec(io_manager.GetDR1)
             ser.str(string(","))
             ser.dec(io_manager.GetDR2)
             ser.str(string(","))
             ser.dec(io_manager.GetAUX)
             ser.str(string(",", CR))
        
        ' Get selected battery
        201: ser.str(string("$201,"))
             ser.dec(io_manager.getActiveBattery)
             ser.str(string(",", CR))
        
        ' Get auto switch voltage
        202: ser.str(string("$202,"))
             ser.dec(io_manager.getSwitchVin)
             ser.str(string(",", CR))
             
        ' Get warning voltage 
        203: ser.str(string("$203,"))
             ser.dec(io_manager.getWarningVin)
             ser.str(string(",", CR))
        
        ' Get minimal voltage
        204: ser.str(string("$204,"))
             ser.dec(io_manager.getMinimalVin)
             ser.str(string(",", CR))
             
        ' Get safety EMER input state 
        205: ser.str(string("$205,"))
             ser.dec(io_manager.GetEMERin)
             ser.str(string(",", CR))
             
        ' Get extra IN0 input state
        206: ser.str(string("$206,"))
             ser.dec(io_manager.GetIN0)
             ser.str(string(",", CR))
             
        ' Get ADC raw values
        207: ser.str(string("$207,"))
             i := 0
             repeat NCh
               ser.dec(ADCRaw[i])
               ser.str(string(","))
               ser.dec(ADCRawAVG[i])
               ser.str(string(","))
               ser.dec(ADCRawMIN[i])
               ser.str(string(","))
               ser.dec(ADCRawMAX[i])
               ser.str(string(","))
               i++
             ser.char(CR)
             
        ' Get ADC voltage values
        208: ser.str(string("$208,"))
             i := 0
             repeat NCh
               ser.dec(ADCch[i])
               ser.str(string(","))
               ser.dec(ADCchAVG[i])
               ser.str(string(","))
               ser.dec(ADCchMIN[i])
               ser.str(string(","))
               ser.dec(ADCchMAX[i])
               ser.str(string(","))
             ser.char(CR)
             
        ' Get ADC engineering values
        209: ser.str(string("$209,"))
             i := 0
             repeat NCh
               ser.dec(engADCch[i])
               ser.str(string(","))
               ser.dec(engADCchAVG[i])
               ser.str(string(","))
               ser.dec(engADCchMIN[i])
               ser.str(string(","))
               ser.dec(engADCchMAX[i])
               ser.str(string(","))
               i++
             ser.char(CR)
             
        ' TODO Get ADC integrated engineering values
        210: ser.str(string("$210,"))
             i := 0
             repeat NCh
               ser.dec(0)
               ser.str(string(","))
               ser.dec(0)
               ser.str(string(","))
               i++
             ser.char(CR)
        
        ' Get RAW battery voltages
        211: ser.str(string("$211,"))
             ser.dec(engADCch[cVBAT1])
             ser.str(string(","))
             ser.dec(engADCch[cVBAT2])
             ser.str(string(",", CR))
             
        ' Get AVG battery voltages
        212: ser.str(string("$212,"))
             ser.dec(engADCchAVG[cVBAT1])
             ser.str(string(","))
             ser.dec(engADCchAVG[cVBAT2])
             ser.str(string(",", CR))
             
        ' Get battery switch sound state
        213: ser.str(string("$213,"))
             if io_manager.getBatterySwitchSound == true     
                ser.dec(1)
             else
                ser.dec(0)             
             ser.str(string(",", CR))
             
        ' Get alarm sound state
        214: ser.str(string("$214,"))
             if io_manager.getAlarmSound == true     
                ser.dec(1)
             else
                ser.dec(0)    
             ser.str(string(",", CR))
             
        ' Get alarm sound interval
        215: ser.str(string("$215,"))
             ser.dec(io_manager.getAlarmInterval)
             ser.str(string(",", CR))
             
        ' Get watchdog treshold counter
        216: ser.str(string("$216,"))
             ser.dec(wd_cnt_threshold)
             ser.str(string(",", CR))
                                    
        ' === Set commands ===
        ' Set auto switch voltage
        300: temp := sGetPar    ' Voltage to set
     
             io_manager.setSwitchVin(temp)
             ser.str(string("$300,"))
             ser.dec(temp)
             ser.str(string(",", CR))
        
        ' Set warning voltage
        301: temp := sGetPar    ' Voltage to set
             
             io_manager.setWarningVin(temp)
             ser.str(string("$301,"))
             ser.dec(temp)
             ser.str(string(",", CR))
        
        ' Set minimal voltage
        302: temp := sGetPar    ' Voltage to set
             
             if temp < cMinVin
               temp := cMinVin
               
             io_manager.setMinimalVin(temp)
             ser.str(string("$302,"))
             ser.dec(temp)
             ser.str(string(",", CR))     
             
        ' Set extra OUT0 output state
        303: temp := sGetPar    ' State to set
             
             ser.str(string("$303,"))
             if temp == 1               
              ser.dec(io_manager.requestOUT0OutputState(true))
             else
              ser.dec(io_manager.requestOUT0OutputState(false))
             ser.str(string(",", CR)) 
             
        ' Turn on/off auto battery selecting mode
        304: temp := sGetPar        ' Which battery to select
             
             ' Turn off or on the auto battery selecting mode  (default on) 
             if temp == 0
                io_manager.setAutoBatterySelect(false)
             else
                io_manager.setAutoBatterySelect(true)    
             
             ser.str(string("$304,"))
             if io_manager.getAutoBatterySelect
                ser.dec(1)
             else
                ser.dec(0)
             ser.str(string(",", CR))  
             
        ' Turn on/off a specified output
        305: temp  := sGetPar        ' Which output 
             temp2 := sGetPar        ' On or off
                 
             ser.str(string("$305,"))
             ' Check output number
             if temp => 0 and temp =< 5   
                 ' Turn on or off the specified ouput
                 if temp2 == 1     
                    io_manager.setSwitch(temp, true)
                 else
                    io_manager.setSwitch(temp, false)    
                 ser.dec(temp)
                 ser.str(string(","))
                 ser.dec(temp2)
             else 'Error unkown output number)
                 ser.dec(-1)
                 ser.str(string(","))
                 ser.dec(0)
                 
             ser.str(string(",", CR))  
             
        
        ' On/off battery change sound
        306: temp := sGetPar        ' Sound on or off
             
             ser.str(string("$306,"))
             ' Turn on or off the battery change sound
             if temp == 1
               io_manager.setBatterySwitchSound(true)
             else 
               io_manager.setBatterySwitchSound(false)
             
             ser.dec(temp)                                                ' 
             ser.str(string(",", CR))  
                
        ' Set battery low alarm sound state
        307: temp := 0 #> sGetPar <# 1      ' on/off
             
             ser.str(string("$307,"))
             io_manager.setAlarmSound(temp)             
             ser.dec(io_manager.getAlarmSound)                                                ' 
             ser.str(string(",", CR))
        
        ' Set battery low alarm interval
        308: temp := 1000 #> sGetPar        ' interval
             
             ser.str(string("$308,"))
             io_manager.setAlarmInterval(temp)             
             ser.dec(io_manager.getAlarmInterval)                                                ' 
             ser.str(string(",", CR))
             
        ' Set watchdog treshold counter
        309: wd_cnt_threshold := 0 #> sGetPar
             ser.str(string("$309,"))
             ser.dec(wd_cnt_threshold)
             ser.str(string(",", CR))
      
        ' === Other commands ===
        ' Enable/reset watchdog
        400: reset_communication
             ser.str(string("$400,", CR))
             
        ' Initiate shutdown   
        401: shutdown
             ser.str(string("$401,", CR))
        
        'Enable/disable serial debug mode    
        402: temp := sGetPar        
             if temp == 1
               Debug := true
             elseif temp == 0
               Debug := false
             
             ser.str(string("$402,"))
             ser.dec(temp)
             ser.str(string(",", CR))
     
        'Reset specified ADC min/max values
        403: temp := sGetPar   
             
             resetADCVals(temp)
                          
             ser.str(string("$403,"))
             ser.dec(temp)
             ser.str(string(",", CR))
             
        'Reset ALL ADC min/max values
        404: resetAllADCVals
                          
             ser.str(string("$404,", CR))
             
        'Select fullest battery
        405: ser.str(string("$405,"))
             ser.dec(io_manager.selectFullestBattery)
             ser.str(string(",", CR))
             
        ' Force select battery
        406: temp := sGetPar        ' Which battery to select
             
             ' Turn off auto battery selecting mode   
             io_manager.setAutoBatterySelect(false) 
                                                 ' 
             ser.str(string("$406,"))
             ser.dec(io_manager.requestBattery(temp))
             ser.str(string(",", CR))     
                  
        other:ser.tx("$")
              ser.dec(command)
              ser.tx(",")
              ser.str(string("Unkown command", CR)) 
              commandOK:=-3
              
Return commandOK


PRI resetSafety 
  no_alarm                 := true                'Reset global alarm var
  last_alarm               := 0                   'Reset last alarm message

' ---------------- Check safety of platform and put in safe condition when needed ---------
PRI DoSafety | i, ConnectionError, bitvalue
  resetSafety

  ' Main safety loop    
  repeat
    SafetyCntr++

    '-- Watchdog -- 
    wd_cnt++

    if wd_cnt > wd_cnt_threshold    
      last_alarm := 1
      no_alarm   := false
      handleWatchdogError
      
    t.Pause1ms(1)
   
       
' === Do logic tasks and safety tasks ===
PRI DoPLC 
  Repeat
   'Update battery voltages at io_manager with current values
    io_manager.updateBatteryVoltages(engADCch[cVBAT1], engADCch[cVBAT2], engADCchAVG[cVBAT1], engADCchAVG[cVBAT2])    
    
    ' Check safety related values
    if engADCchAVG[cV5V] < c5V
      s5VOK :=0

    if engADCchAVG[cV3V3] < c3V3                                                               
      s3V3OK :=0

    if engADCchAVG[cV24VBus] < io_manager.getMinimalVin
      sVinOK :=0
      
    if sVinOK==1 and s3V3OK==1 and  s5VOK==1  '' Check power supplies 
      io_manager.requestOut0OutputState(1)
    else
      io_manager.requestOut0OutputState(0)
      
    If io_manager.GetEMERin==1 ' and sVinOK==1 and AllOK==1 ' Process emergency alarms to OK output
      io_manager.requestOkOutputState(1)
    else
      io_manager.requestOkOutputState(0)      
      AllOK:=0
      

    PlcCnt++
    
    
    
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

    
' ADCch[NCh], ADCchAVG[NCh], ADCchMAX[NCh], ADCchMIN[NCh]
' 
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
    repeat 
      t.Pause1ms(10)
      If Debug      
        ser.position(0,0)

        ser.str(CONTROLLER_ID)
        ser.str(string(" "))
        ser.str(major_version)
        ser.str(string("."))
        ser.str(minor_version)
        ser.str(string(" ADCMeasurementCog: "))
        ser.dec(ADCMCog)
        ser.str(string(" PLCCog: "))
        ser.dec(LinMotCog)
        
        ser.str(string(" Selected Battery: "))
        ser.dec(io_manager.getActiveBattery)
        
        ser.str(string(" time since battery switch: "))
        ser.dec(io_manager.getOneMScounter - io_manager.getBatterySwitchTime)
        ser.str(string("ms "))
        
        ser.tx(CR)
        ser.str(string(" oneMScounter: "))
        ser.dec(io_manager.getOneMScounter)
        ser.str(string("ms "))
        ser.str(string(" LedIntervalTimer: "))
        ser.dec(io_manager.getLedIntervalTimer)
        ser.str(string("ms "))
        ser.str(string(" AlarmIntervalTimer: "))
        ser.dec(io_manager.getAlarmIntervalTimer)
        ser.str(string("ms "))
        
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
        ser.str(string("eADCvRAW:"))
        repeat NCh
          ser.str(num.decf(engADCch[i],NumWidth))  'Engineering
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
        ser.tx(" ")
        ser.str(string("IOManagerCnt: "))
        ser.dec(io_manager.getIOManagerCounter)   
        
    
        ser.tx(cr)
        ser.tx(ce)
    
        ser.str(string("EMER: "))
        ser.dec(io_manager.GetEMERin)
        ser.str(string(" OK: "))
        ser.dec(io_manager.GetOK)
        ser.str(string(" IN0: "))
        ser.dec(io_manager.GetIN0)
        ser.str(string(" OUT0: "))
        ser.dec(io_manager.GetOUT0)
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
  ser.str(num.decf(io_manager.GetCONTR,3)) 'Engineering avg
  ser.str(string(" "))
  ser.str(num.decf(io_manager.GetPC1,3)) '
  ser.str(string(" "))
  ser.str(num.decf(io_manager.GetPC2,3)) '
  ser.str(string(" "))
  ser.str(num.decf(io_manager.GetDR1,3)) '
  ser.str(string(" "))
  ser.str(num.decf(io_manager.GetDR2,3)) '
  ser.str(string(" "))
  ser.str(num.decf(io_manager.GetAUX,3)) '
  ser.str(string(" "))
  
  ser.tx(cr)
  ser.tx(ce)


' ----------------------- Reset program ------------------------
PRI DoReset
  sVinOK := 1
  s3V3OK := 1
  s5VOK  := 1
  AllOK  := 1
    
'Measure ADC channels. 
{PRI DoADC | i, j, T1

   repeat
     T1:=cnt

     i:=0
     j:=0
     repeat 8 ' NCh
       ADCRaw[i]:= ADC1.in(i)      
       ADCRawAVG[i]:= (ADCRawAVG[i] *9 + ADCRaw[i] )/10
       ADCRawMIN[i] <#= ADCRawAVG[i]         'save min value
       ADCRawMAX[i] #>= ADCRawAVG[i]         'save max value
       i++
       j++

     ADCCnt++
        
     ADCTime:=(CNT-T1)/80   }
  
'MeasureADC channels.
PRI DoADC | i, T1

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
   
  ' Pre-set AVG value with one measurement
  repeat NCh
    if i<8
      ADCRaw[i] := ADC.in(i,chip1)       ' First 8 channels from ADC1 
    else
      ADCRaw[i] := ADC.in(i-8,chip2)     ' And next 8 from ADC2  
    ADCRawAVG[i] := ADCRaw[i]
  
  repeat
     T1:=cnt
     i:=0
     repeat NCh
       if i<8
         ADCRaw[i]:= ADC.in(i,chip1)       ' First 8 channels from ADC1 
       else
         ADCRaw[i]:= ADC.in(i-8,chip2)     ' And next 8 from ADC2  
       
       ADCRawAVG[i]:= (ADCRawAVG[i] *4 +  ADCRaw[i] )/5
       ADCRawMIN[i] <#= ADCRaw[i]            'save min value
       ADCRawMAX[i] #>= ADCRaw[i]            'save max value
       i++
       
     ' Do scaling from counts to engineering values * 1000
     DoADCScaling

     ADCCnt++        
     ADCTime:=(CNT-T1)/80

' Do scaling from counts to engineering values * 1000
PRI DoADCScaling | T1, i
    T1:=cnt
    i:=0
    Repeat NCh
      ADCch[i]:= f.fround(f.fmul(f.Ffloat(ADCRaw[i]), cADCbits2mV)) 'Calculate mV
      ADCchAVG[i]:= f.fround(f.fmul(f.Ffloat(ADCRawAVG[i]), cADCbits2mV)) 'Calculate mV
      ADCchMax[i]:= f.fround(f.fmul(f.Ffloat(ADCRawMax[i]), cADCbits2mV)) 'Calculate mV
      ADCchMin[i]:= f.fround(f.fmul(f.Ffloat(ADCRawMin[i]), cADCbits2mV)) 'Calculate mV

      engADCch[i]:= f.fround(f.fmul(f.Ffloat(ADCch[i]), Scale[i])) 'Calculate RAW mV
      engADCchAVG[i]:= f.fround(f.fmul(f.Ffloat(ADCchAVG[i]), Scale[i])) 'Calculate AVG mV
      engADCchMAX[i]:= f.fround(f.fmul(f.Ffloat(ADCchMAX[i]), Scale[i])) 'Calculate MAX mV
      engADCchMIN[i]:= f.fround(f.fmul(f.Ffloat(ADCchMIN[i]), Scale[i])) 'Calculate MIN mV
      i++
      
    ScalingCnt++
    ScalingTime:=(CNT-T1)/80

' -----------------Command handling section ---------------------------------------------
' ---------------- Get next parameter from string ---------------------------------------
PRI sGetPar | i, j, lch
  j:=0
  i:=0
  Bytefill(@last_parameter,0,CmdLen)   'Clear buffer
    
  lch:=sGetch                    'Get comma and point to first numeric char
  repeat while lch <> "," and lch <> 0 'Get parameter until comma or unexpected end
    if lch <> "$" and lch <> "," and j<CmdLen
      last_parameter[j++]:=lch
    lch:=sGetch           'get next

Return ser.strtodec(@last_parameter)

' ---------------- Get next character from string ---------------------------------------
Pri sGetCh | lch 'Get next character from commandstring
   lch:=Byte[@StrBuf][StrP++]
Return lch

' Symbols      
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
    
