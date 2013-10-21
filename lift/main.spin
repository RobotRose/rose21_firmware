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

DAT
  version BYTE "1.30",0 ' Software version

OBJ

  PC      : "FullDuplexSerial_rr005"
  Sensor  : "Ultrasoon"
  Height  : "height_controller"

CON

  BaudRate = 115200


VAR
  long key
  long X
  long SensorData[8]
  long CStep
  long Command
  byte Value[2]
  long SpecialCommand
  byte waitInit
  BYTE b_pos_reached


PUB Main | lch
      main_init
      b_pos_reached := FALSE
     
      repeat
        lch := PC.rxcheck
        
        if lch > 0
          'PC.dec(ii)
           'PC.out(lch)
           DoCommand(lch)
           
           lch := 0
    
        'Check height status
        if ((Height.get_requested_pos == Height.get_current_pos) AND b_pos_reached)
          PC.str(string("$350,Height reached,#",13))
          b_pos_reached := FALSE   
    
            
    { 'Disable continuous reading of ADC 
        PC.str(string("$441,"))
        'PC.hex(Height.height_get_position,4)
        PC.dec(Height.height_get_position)
        PC.str(string(", status: "))
        PC.hex(Height.height_get_config_register, 2)
        PC.str(string(",#",13))
        
        waitcnt((clkfreq/2)+cnt)       
     }

PRI main_init | ii, T1, temp, lch2
  
  waitInit := TRUE
  waitcnt((clkfreq/2)+cnt)

  if (PC.start(31,30,0,BaudRate))
    ii := 1
  else
    ii := 0

  repeat while waitInit
    lch2 := PC.rxcheck
        
        if lch2 > 0
          'PC.dec(ii)
           'PC.out(lch)
           DoCommand(lch2)
           
           lch2 := 0      

  if (ii)
    PC.str(string("$350,Communication COG started,#",13))
  else
    PC.str(string("$351,Communication COG error,#",13))

  PC.str(string("$350,Starting up,#",13))

  PC.str(string("$350, Platform controller level 2 --> Version: "))
  PC.str(@version)          
  PC.str(string(",#",13))
   
  PC.str(string("$350,Loading settings from EEPROM,#",13))
  
  
  temp := Sensor.LoadLowPos

  waitcnt((clkfreq/8)+cnt)
  Height.height_set_low(temp)


                             
  temp := Sensor.LoadHighPos
  waitcnt((clkfreq/8)+cnt)
  Height.height_set_high(temp)

  temp := Sensor.LoadSettings
   
  PC.str(string("$350,Checking Sensors,#",13))
  if (Sensor.CheckSensors == 255)
    PC.str(string("$350,All sensors online!,#",13))
    PC.str(string("$350,Sensor checksum: "))
    PC.dec(Sensor.CheckSensors)
    PC.str(string(",#",13))
  else
    PC.str(string("$351,There is a problem with the sensors!,#",13))
    PC.str(string("Sensor checksum: "))
    PC.dec(Sensor.CheckSensors)
    PC.str(string(",#",13))
   
  PC.str(string("$350,Initializing Sensors,#",13))
  PC.str(string("$350,Starting Sensor Sequence,#",13))
  if (Sensor.SensorStart)
    PC.str(string("$350,Sensor COG started,#",13))
  else
    PC.str(string("$351,Sensor COG Error,#",13)) 
   
  waitcnt((clkfreq/8)+cnt)

  'Initialise the heigth controller
  PC.str(string("$350,Initializing Height,#",13))
  Height.height_init (temp)
                              
  
  ii :=Height.height_start
  PC.str(string("$350,Height COG: "))
  PC.dec(ii)
  PC.str(string(",#",13))
  
  if (ii)
    PC.str(string("$350,Height COG started,#",13))
  else
    PC.str(string("$351,Height COG Error,#",13))    
   
  waitcnt((clkfreq/8)+cnt)
   
  PC.str(string("$350,Loaded and running,#",13))
  PC.str(string("$350,Waiting for command,#",13))
  CStep := 0
  
PUB DoCommand (lCmd)

  if lCmd == "$"
    CStep := 1
    PC.rxflush
    Command := 0
  else
    case Cstep
      1  : case lCmd
             "3" : Command := 300
             "4" : Command := 400
           CStep := 2
           PC.rxflush
      2  : case lCmd
             "0" : Command := Command + 0
             "1" : Command := Command + 10
             "2" : Command := Command + 20
             "3" : Command := Command + 30
             "4" : Command := Command + 40
             "5" : Command := Command + 50
             "6" : Command := Command + 60
             "7" : Command := Command + 70
             "8" : Command := Command + 80
             "9" : Command := Command + 90
           CStep := 3
           PC.rxflush  
      3  : case lCmd
             "0" : Command := Command + 0
             "1" : Command := Command + 1
             "2" : Command := Command + 2
             "3" : Command := Command + 3
             "4" : Command := Command + 4
             "5" : Command := Command + 5
             "6" : Command := Command + 6
             "7" : Command := Command + 7
             "8" : Command := Command + 8
             "9" : Command := Command + 9
           CStep := 4      
      4  : if lCmd == ","
             CStep := 5
           elseif lCmd == 13
             ActivateCommand
             PC.rxflush
           else             
             CStep := 0
      5  : Value[1] := 0
           Value[1] := ChartoDec(lCmd,1)
           CStep := 6
      6  : Value[1] := Value[1] + ChartoDec(lCmd,0)
           CStep := 7
      7  : if lCmd == ","
             CStep := 8
           else
             CStep := 0
      8  : Value[2] := 0
           Value[2] := ChartoDec(lCmd,1)
           CStep := 9          
      9  : Value [2] := Value[2] + ChartoDec(lCmd,0)
           CStep:= 10
      10 : if lCmd == ","
             PC.rxflush
             CStep := 11
           else
             CStep := 0        
      11 : if lCmd == "#"
             ActivateSpecialCommand
             PC.rxflush
             CStep := 0
           else
             CStep := 0
      Other : CStep := 0

PUB ActivateSpecialCommand
  case SpecialCommand

    315 : PC.str(string("$350,Execute Special Command 315,#",13))
          PC.dec(Value[1])
          PC.str(string(" ",13))
          PC.dec(Value[2])
          PC.str(string(" ",13))
    316 : PC.str(string("$350,Execute Special Command 316,#",13))
          PC.dec(Value[1])
          PC.str(string(" ",13))
          PC.dec(Value[2])
          PC.str(string(" ",13))    
    317 : PC.str(string("$350,Execute Special Command 317,#",13))
          PC.dec(Value[1])
          PC.str(string(" ",13))
          PC.dec(Value[2])
          PC.str(string(" ",13))     

SpecialCommand := 0

PUB ActivateCommand

  case Command

    300 : PC.str(string("$350,Execute Command 300,#",13))      
          Sensor.SensorStart
          PC.str(string("$350,Sensor COG started,#",13))
    301 : PC.str(string("$350,Execute Command 301,#",13))
          Sensor.SensorStop
          PC.str(string("$350,Sensor COG stopped,#",13)) 
    302 : PC.str(string("$350,Execute Command 302,#",13)) 
          X:=1
          PC.str(string("$350,"))
          repeat 8
            PC.dec(Sensor.ShowResult(X))
            PC.Str(string(","))
            X++
          PC.str(string("#",13)) 
    303 : PC.str(string("$350,Execute Command 303,#",13))
    304 : PC.str(string("$350,Execute Command 304,#",13))
    305 : PC.str(string("$350,Execute Command 305,#",13))
    306 : PC.str(string("$350,Execute Command 306,#",13))
    307 : PC.str(string("$350,Execute Command 307,#",13))
    308 : PC.str(string("$350,Execute Command 308,#",13))
    309 : PC.str(string("$350,Execute Command 309,#",13))
    310 : PC.str(string("$350,Execute Command 310,#",13))
    311 : PC.str(string("$350,Execute Command 311,#",13))
    312 : PC.str(string("$350,Execute Command 312,#",13))
          X:=1
          PC.str(string("$350,"))
          repeat 8
            PC.hex(Sensor.ReadAddressSetting(X),2)
            PC.Str(string(","))
            X++
          PC.str(string("#",13))
    313 : PC.str(string("$350,Execute Command 313,#",13))
          X:=1
          PC.str(string("$350,"))
          repeat 8
            PC.dec(Sensor.ReadGainSetting(X))
            PC.Str(string(","))
            X++
          PC.str(string("#",13))
    314 : PC.str(string("$350,Execute Command 314,#",13))
          X:=1
          PC.str(string("$350,"))
          repeat 8
            PC.hex(Sensor.ReadRangeSetting(X),2)
            PC.Str(string(","))
            X++
          PC.str(string("#",13))    
    315 : PC.str(string("$350,Execute Command 315,#",13))
          SpecialCommand := 315
          CStep := 4    
    316 : PC.str(string("$350,Execute Command 316,#",13))
          SpecialCommand := 316
          CStep := 4
    317 : PC.str(string("$350,Execute Command 317,#",13))
          SpecialCommand := 317
          CStep := 4
    318 : PC.str(string("$350,Execute Command 318,#",13))
    319 : PC.str(string("$350,Execute Command 319,#",13))
    320 : PC.str(string("$350,Execute initialization,#",13))
          if waitInit == FALSE
            PC.str(string("$350,Waiting for command,#",13))
          waitInit := FALSE
          
    400 : PC.str(string("$350, Start height cog,#",13))
          Height.height_start
    401 : PC.str(string("$350,Stop height cog,#",13))
          Height.height_stop
    402 : PC.str(string("$350,"))
          PC.hex(Height.height_get_position,4)
          PC.str(string(",#",13))
    403 : PC.str(string("$350,Stored position: "))
          PC.hex(Height.height_get_low,4)
          PC.str(string(","))
          PC.hex(Height.height_get_mid_position,4)
          PC.str(string(","))
          PC.hex(Height.height_get_high,4)
          PC.str(string(",#",13))
    404 : PC.str(string("$350,Current height: "))
          PC.hex(Height.height_get_position,4)
          PC.str(string(",#",13))            
    405 : PC.str(string("$350,ADC config register: "))
          PC.hex(Height.height_get_config_register,2)
          PC.str(string(",#",13))
    406 : PC.str(string("$350,Height status,"))
          PC.dec(Height.get_current_pos)
          PC.str(string(",#",13))
    410 : PC.str(string("$350,Go to low position,#",13))
          Height.height_set_position(POS_LOW)
          b_pos_reached := TRUE
    420 : PC.str(string("$350,Go to mid position,#",13))
          Height.height_set_position(POS_MID)
          b_pos_reached := TRUE
    430 : PC.str(string("$350,Go to high position,#",13))
          Height.height_set_position(POS_HIGH)
          b_pos_reached := TRUE
    440 : PC.str(string("$350,Stop height adjustment,#",13))
          Height.height_set_position(POS_STOP)
          b_pos_reached := TRUE
          
    451 : specialcommand := Height.height_get_position
          PC.str(string("$350,"))
          PC.hex(specialcommand,4)
          PC.str(string(",#",13))
          Sensor.SensorStop
          Sensor.StoreHeightSettings(specialcommand)
          Height.height_set_mid_position(specialcommand)
          Sensor.SensorStart
          waitcnt((clkfreq/8)+cnt)
    452 : specialcommand := Height.height_get_position
          PC.str(string("$350,"))
          PC.hex(specialcommand,4)
          PC.str(string(",#",13))
          Sensor.SensorStop
          Sensor.StoreHeightSettingsLow(specialcommand)
          Height.height_set_low(specialcommand)
          Sensor.SensorStart
          waitcnt((clkfreq/8)+cnt)
    453 : specialcommand := Height.height_get_position
          PC.str(string("$350,"))
          PC.hex(specialcommand,4)
          PC.str(string(",#",13))
          Sensor.SensorStop
          Sensor.StoreHeightSettingsHigh(specialcommand)
          Height.height_set_high(specialcommand)
          Sensor.SensorStart
          waitcnt((clkfreq/8)+cnt)
    470 : Height.turn_cw
    480 : Height.turn_ccw
    490 : PC.str(string("$350,Version: "))
          PC.str(@version)          
          PC.str(string(",#",13))
                                       
    499 : PC.str(string("$350,REBOOT,#",13))
      REBOOT
    Other : PC.str(string("$351,Unknown Command,#",13))      

 Command := 0

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
               
