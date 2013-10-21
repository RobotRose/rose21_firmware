'Ultrasone sensor ring
'10 augustus 2011
'VEDS
'Floris Duine

'Functions:
'StartSensor (Address)                 - Start a ranging of a specific sensor in cm mode
'ReadSensor (Address,Mode)             - Read the result of a ranging from a specific sensor and show the result in the terminal
'CheckSensors                          - Checks if all sensors are present on the address specified in the constants and shows the result in the terminal
'AddressChange (OldAddress,NewAddress) - Used to change the address of one specific sensor
'ChangeGain (Address, GainLevel)       - Used to change the gain of one specific sensor 
'SetGain                               - Used to set the range of all 8 sensors with the address and gain from the constants
'ChangeRange (Address, Range)          - Used to change the range of one specific sensor
'SetRange                              - Used to set the range of all 8 sensors with the address and range from the constants


CON
    
  i2cSCL        = 28
' i2cSDA        = 29

'EEPROM address
  EEPROMAddr    = $A0

'Sensor Mode
  SRF10ModeInch = $50  'Initiatie ranging mode, result in Inch
  SRF10ModeCM   = $51  'Initiatie ranging mode, result in CM
  SRF10ModeUSec = $52  'Initiatie ranging mode, result in uSec   

  
VAR

  long cog
  long SensorResult[8]
  'byte SensorStatus
  long stackspace[100]
  byte X,RL

  byte SRF10Addr[8]
  byte SRF10Gain[8]
  byte SRF10Range[8]  

OBJ
                   
  i2cObject      : "basic_i2c_driver"
  Terminal       : "PC_Interface"



PUB SensorStart : ok

  ok := cog := cognew(SensorSequence, @stackspace) + 1      '(@SensorResult)

PUB SensorStop

  if cog
    cogstop(cog~ - 1)

PUB SensorInit

  'Initialize I2C communication module
  i2cObject.Initialize(i2cSCL)                                            
  ' pause 2 seconds
  repeat 2
        'PC.str(string(".",13))
        waitcnt((clkfreq/2)+cnt)
  CheckSensors
  SetGain
  SetRange

PUB SensorSequence '(SensResult)

  'Initialize I2C communication module
  i2cObject.Initialize(i2cSCL)                                            

  LoadSettings
  CheckSensors
  SetGain
  SetRange

  repeat
      StartSensor(SRF10Addr[1],SRF10ModeCM)          'Start Sensor 1
      StartSensor(SRF10Addr[5],SRF10ModeCM)          'Start Sensor 5
      SensorResult[3] := ReadSensor(SRF10Addr[3])    'Result Sensor 3
      SensorResult[7] := ReadSensor(SRF10Addr[7])    'Result Sensor 7
      StartSensor(SRF10Addr[2],SRF10ModeCM)          'Start Sensor 2
      StartSensor(SRF10Addr[6],SRF10ModeCM)          'Start Sensor 6
      SensorResult[4] := ReadSensor(SRF10Addr[4])    'Result Sensor 4
      SensorResult[8] := ReadSensor(SRF10Addr[8])    'Result Sensor 8
      StartSensor(SRF10Addr[3],SRF10ModeCM)          'Start Sensor 3
      StartSensor(SRF10Addr[7],SRF10ModeCM)          'Start Sensor 7
      SensorResult[1] := ReadSensor(SRF10Addr[1])    'Result Sensor 1
      SensorResult[5] := ReadSensor(SRF10Addr[5])    'Result Sensor 5
      StartSensor(SRF10Addr[4],SRF10ModeCM)          'Start Sensor 4
      StartSensor(SRF10Addr[8],SRF10ModeCM)          'Start Sensor 8
      SensorResult[2] := ReadSensor(SRF10Addr[2])    'Result Sensor 2
      SensorResult[6] := ReadSensor(SRF10Addr[6])    'Result Sensor 6
      
PUB LoadSettings
' FUNCTION : Read the settings of the sensors and ADC from EEPROM
' INPUT    : Address      (I2C Address of the target in HEX)
' OUTPUT   : Result of the ranging in the PC terminal
' REMARKS  : Can only be done when the sensor COG is stopped
  'Load Address Settings from EEPROM to variables
  X := 1
  RL := $20
  repeat 8
    SRF10Addr[X] := i2cObject.readEeprom(i2cSCL,EEPROMAddr,$80,RL)
    X++
    RL++

  'Load Gain Settings from EEPROM to variables
  X := 1
  RL := $30
  repeat 8
    SRF10Gain[X] := i2cObject.readEeprom(i2cSCL,EEPROMAddr,$80,RL)
    X++
    RL++

  'Load Range Settings from EEPROM to variables
  X := 1
  RL := $40
  repeat 8
    SRF10Range[X] := i2cObject.readEeprom(i2cSCL,EEPROMAddr,$80,RL)
    X++
    RL++  

  ' Read medium height value of for the ADC
  return i2cObject.ReadWord(i2cSCL,EEPROMAddr,$A000)

PUB LoadHighPos
  return i2cObject.ReadWord(i2cSCL,EEPROMAddr,$A200)

PUB LoadLowPos
  return i2cObject.ReadWord(i2cSCL,EEPROMAddr,$A100)

  
PUB ShowResult (Num) : Output

Output := SensorResult[Num]
                                                                                          

PUB StartSensor (Address,Mode)
' FUNCTION : Start a ranging in specified mode
' INPUT    : Address      (I2C Address of the target in HEX)
'            Mode         (Mode of the target in HEX)
' OUTPUT   : None
' REMARKS  : None         
  i2cObject.writeLocation(i2cSCL,Address, 0, Mode)

PUB ReadSensor (Address) : result
' FUNCTION : Read the result of a ranging and show this in the PC terminal
' INPUT    : Address      (I2C Address of the target in HEX)
' OUTPUT   : Result of the ranging in the PC terminal
' REMARKS  : None
  'PC.dec(i2cObject.readLocation(i2cSCL,Address, 3))
  result :=  i2cObject.readLocation(i2cSCL,Address, 3)
  'PC.str(string(" ",13))


PUB CheckSensors : SensorStatus
' FUNCTION : Check if all sensors are present and show the result in the PC terminal
' INPUT    : None
' OUTPUT   : Result of the check in the PC terminal
' REMARKS  : None

  SensorStatus := 0
   ' Check sensor 1
    if i2cObject.devicePresent(i2cSCL,SRF10Addr[1]) == true
            'PC.str(string("SRF10 Sensor 1 Present",13))
            SensorStatus := SensorStatus + 1
            
    else
                    'PC.str(string("SRF10 Sensor 1 Missing",13))    
    'waitcnt(clkfreq/2+cnt)
   ' Check sensor 2
    if i2cObject.devicePresent(i2cSCL,SRF10Addr[2]) == true
            'PC.str(string("SRF10 Sensor 2 Present",13))
            SensorStatus := SensorStatus + 2
            
    else
                    'PC.str(string("SRF10 Sensor 2 Missing",13))    
    'waitcnt(clkfreq/2+cnt)
   ' Check sensor 3
    if i2cObject.devicePresent(i2cSCL,SRF10Addr[3]) == true
            'PC.str(string("SRF10 Sensor 3 Present",13))
            SensorStatus := SensorStatus + 4
            
    else
                    'PC.str(string("SRF10 Sensor 3 Missing",13))    
    'waitcnt(clkfreq/2+cnt)       
   ' Check sensor 4
    if i2cObject.devicePresent(i2cSCL,SRF10Addr[4]) == true
            'PC.str(string("SRF10 Sensor 4 Present",13))
            SensorStatus := SensorStatus + 8
            
    else
                    'PC.str(string("SRF10 Sensor 4 Missing",13))    
    'waitcnt(clkfreq/2+cnt)
   ' Check sensor 5
    if i2cObject.devicePresent(i2cSCL,SRF10Addr[5]) == true
            'PC.str(string("SRF10 Sensor 5 Present",13))
            SensorStatus := SensorStatus + 16
            
    else
                    'PC.str(string("SRF10 Sensor 5 Missing",13))    
    'waitcnt(clkfreq/2+cnt)       
   ' Check sensor 6
    if i2cObject.devicePresent(i2cSCL,SRF10Addr[6]) == true
            'PC.str(string("SRF10 Sensor 6 Present",13))
            SensorStatus := SensorStatus + 32
            
    else
                    'PC.str(string("SRF10 Sensor 6 Missing",13))    
    'waitcnt(clkfreq/2+cnt)
  ' Check sensor 7
    if i2cObject.devicePresent(i2cSCL,SRF10Addr[7]) == true
            'PC.str(string("SRF10 Sensor 7 Present",13))
            SensorStatus := SensorStatus + 64
            
    else
                    'PC.str(string("SRF10 Sensor 7 Missing",13))    
    'waitcnt(clkfreq/2+cnt)
   ' Check sensor 8 
    if i2cObject.devicePresent(i2cSCL,SRF10Addr[8]) == true
            'PC.str(string("SRF10 Sensor 8 Present",13))
            SensorStatus := SensorStatus + 128
            
    else
                    'PC.str(string("SRF10 Sensor 8 Missing",13))    
    'waitcnt(clkfreq/2+cnt)

PUB AddressChange (OldAddress,NewAddress)

' FUNCTION : Check if the sensor is connected at the old address and change address to the new address
' INPUT    : OldAddress      (I2C Address of the target in HEX)
'            NewAddress      (I2C Address of the target in HEX) 
' OUTPUT   : None
' REMARKS  : None

  if i2cObject.devicePresent(i2cSCL,OldAddress) == true
    'PC.str(string("Target found",13))  
    'PC.str(string("Old address is: ",13))
    'PC.dec(OldAddress)
    'PC.str(string(" ",13))
    i2cObject.writeLocation(i2cSCL,OldAddress, 0, $A0)
    i2cObject.writeLocation(i2cSCL,OldAddress, 0, $AA)
    i2cObject.writeLocation(i2cSCL,OldAddress, 0, $A5)
    i2cObject.writeLocation(i2cSCL,OldAddress, 0, NewAddress)
    waitcnt((clkfreq/1)+cnt)
    'PC.str(string("Address changed",13))
    'PC.str(string("New address is:",13))
    'PC.dec(NewAddress)
    'PC.str(string(" ",13))
  else
    'PC.str(string("Target not found",13))
    'PC.str(string("Address not changed",13))

PUB ChangeGain (Address, GainLevel)

' FUNCTION : With this function the gain level of a certain SRF10 Sensor can be changed
' INPUT    : Address      (I2C Address of the target in HEX)
'            GainLevel    (Gain number 0..16)
' OUTPUT   : None
' REMARKS  : Gain level table
'            Level    Maximum analog gain
'            0        40
'            1        40
'            2        50
'            3        60
'            4        70
'            5        80
'            6        100
'            7        120
'            8        140
'            9        200
'            10       250
'            11       300
'            12       350
'            13       400
'            14       500
'            15       600
'            16       700
' (NOTE: The relationship between the level and the actual gain is not linear                    

  CASE GainLevel
    00  : i2cObject.writeLocation(i2cSCL,Address, 1, $00)
    01  : i2cObject.writeLocation(i2cSCL,Address, 1, $00)
    02  : i2cObject.writeLocation(i2cSCL,Address, 1, $01)
    03  : i2cObject.writeLocation(i2cSCL,Address, 1, $02)                            
    04  : i2cObject.writeLocation(i2cSCL,Address, 1, $03)
    05  : i2cObject.writeLocation(i2cSCL,Address, 1, $04)
    06  : i2cObject.writeLocation(i2cSCL,Address, 1, $05)
    07  : i2cObject.writeLocation(i2cSCL,Address, 1, $06)
    08  : i2cObject.writeLocation(i2cSCL,Address, 1, $07)
    09  : i2cObject.writeLocation(i2cSCL,Address, 1, $08)
    10  : i2cObject.writeLocation(i2cSCL,Address, 1, $09)
    11  : i2cObject.writeLocation(i2cSCL,Address, 1, $0A)
    12  : i2cObject.writeLocation(i2cSCL,Address, 1, $0B)
    13  : i2cObject.writeLocation(i2cSCL,Address, 1, $0C)
    14  : i2cObject.writeLocation(i2cSCL,Address, 1, $0D)
    15  : i2cObject.writeLocation(i2cSCL,Address, 1, $0E)
    16  : i2cObject.writeLocation(i2cSCL,Address, 1, $0F)
    OTHER : 'PC.str(string("Gain level not recognised",13))

PUB SetGain
' FUNCTION : Set gain of all 8 sensors from the constants specified
' INPUT    : None
' OUTPUT   : None
' REMARKS  : None
  ChangeGain (SRF10Addr[1],SRF10Gain[1])
  ChangeGain (SRF10Addr[2],SRF10Gain[2])
  ChangeGain (SRF10Addr[3],SRF10Gain[3])
  ChangeGain (SRF10Addr[4],SRF10Gain[4])
  ChangeGain (SRF10Addr[5],SRF10Gain[5])
  ChangeGain (SRF10Addr[6],SRF10Gain[6])
  ChangeGain (SRF10Addr[7],SRF10Gain[7])
  ChangeGain (SRF10Addr[8],SRF10Gain[8])

PUB ChangeRange (Address, Range)

' FUNCTION : With this function the range of a certain SRF10 Sensor can be changed
' INPUT    : Address      (I2C Address of the target in HEX)
'            Range        (Range number 0..255)
' OUTPUT   : None
' REMARKS  : The range can be set in steps of about 43mm (0.043m or 1.68 inches) up to 11 metres.
'            The range is ((Range x 43mm) + 43mm) so setting the Range to 0 (0x00) gives a
'            maximum range of 43mm. Setting the Range to 1 (0x01) gives a maximum range of 86mm. More
'            usefully, 24 (0x18) gives a range of 1 metre and 93 (0x5D) is 4 metres. Setting 255 (0xFF) gives the
'            original 11 metres (255 x 43 + 43 is 11008mm)  

i2cObject.writeLocation(i2cSCL,Address, 2, Range)

PUB SetRange
' FUNCTION : Set range of all 8 sensors from the constants specified
' INPUT    : None
' OUTPUT   : None
' REMARKS  : None
  ChangeRange (SRF10Addr[1],SRF10Range[1])
  ChangeRange (SRF10Addr[2],SRF10Range[2])
  ChangeRange (SRF10Addr[3],SRF10Range[3])
  ChangeRange (SRF10Addr[4],SRF10Range[4])
  ChangeRange (SRF10Addr[5],SRF10Range[5])
  ChangeRange (SRF10Addr[6],SRF10Range[6])
  ChangeRange (SRF10Addr[7],SRF10Range[7])
  ChangeRange (SRF10Addr[8],SRF10Range[8])

PUB ReadAddressSetting (Sensor) : value

  case Sensor
    1 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $20)
    2 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $21)
    3 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $22)
    4 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $23)
    5 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $24)
    6 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $25)
    7 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $26)
    8 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $27)

PUB ReadRangeSetting (Sensor) : value

  case Sensor
    1 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $40)
    2 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $41)
    3 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $42)
    4 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $43)
    5 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $44)
    6 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $45)
    7 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $46)
    8 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $47)

PUB ReadGainSetting (Sensor) : value

  case Sensor
    1 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $30)
    2 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $31)
    3 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $32)
    4 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $33)
    5 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $34)
    6 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $35)
    7 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $36)
    8 : value := i2cObject.readEeprom(i2cSCL,EEPROMAddr, $80, $37)

PUB StoreSettings
 
  'Store Address Settings to EEPROM from variables
  X := 1
  RL := $20
  repeat 8
    i2cObject.writeEeprom(i2cSCL,EEPROMAddr,$80,RL,SRF10Addr[X])
    X++
    RL++

  'Store Gain Settings to EEPROM from variables
  X := 1
  RL := $30
  repeat 8
    i2cObject.writeEeprom(i2cSCL,EEPROMAddr,$80,RL,SRF10Gain[X])
    X++
    RL++

  'Store Range Settings to EEPROM from variables
  X := 1
  RL := $40
  repeat 8
    i2cObject.writeEeprom(i2cSCL,EEPROMAddr,$80,RL,SRF10Range[X])
    X++
    RL++


  RETURN 

PUB StoreHeightSettings (ADC_store_value)

  ' Store the ADC value of the medium height
  i2cObject.WriteWord(i2cSCL, EEPROMAddr, $A000, ADC_store_value)

PUB StoreHeightSettingsLow (ADC_store_value1)

  ' Store the ADC value of the low height
  i2cObject.WriteWord(i2cSCL, EEPROMAddr, $A100, ADC_store_value1)

PUB StoreHeightSettingsHigh (ADC_store_value2)

  ' Store the ADC value of the high height
  i2cObject.WriteWord(i2cSCL, EEPROMAddr, $A200, ADC_store_value2)
    
PUB OneTime

SRF10Addr[1] := $E0
SRF10Addr[2] := $E2
SRF10Addr[3] := $E4
SRF10Addr[4] := $E6
SRF10Addr[5] := $E8
SRF10Addr[6] := $EA
SRF10Addr[7] := $EC
SRF10Addr[8] := $EE

SRF10Gain[1] := 16
SRF10Gain[2] := 16
SRF10Gain[3] := 16
SRF10Gain[4] := 16
SRF10Gain[5] := 16
SRF10Gain[6] := 16
SRF10Gain[7] := 16
SRF10Gain[8] := 16

SRF10Range[1] := $30
SRF10Range[2] := $30
SRF10Range[3] := $30
SRF10Range[4] := $30
SRF10Range[5] := $30
SRF10Range[6] := $30
SRF10Range[7] := $30
SRF10Range[8] := $30
    