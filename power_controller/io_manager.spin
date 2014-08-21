' Output pins manager, runs in a sperate cog and manages the batteries and power outputs
' Rose B.V. 21-08-2014
' Okke Hendriks
' 


CON
  'Safety I/O
  EMERin = 0   'Safety input
  OK     = 1   'Safety output relais
  IN0    = 2   'Extra input optocoupler
  OUT0   = 3   'Extra relais output

  ' Power channels
  PWRBAT1      = 16
  PWRBAT2      = 17
  PWRCONTR     = 18
  PWRPC1       = 19
  PWRPC2       = 20
  PWRDR2       = 22
  PWRDR1       = 21 
  PWRAUX       = 23
  
  ' Buzzer pin
  BUZZ         = 9     

  n_switches   = 6
  
  default_auto_battery_switch_timeout = 2000

OBJ
  sound         : "sound"
  t             : "Timing"

VAR
  ' Cog variables
  long cog
  long cog_stack[40]
  long output_manager_counter

  long oneMScounter

  ' Battery management
  long auto_battery_switch  '1: enables local auto battery switch
  long active_battery       '0: all batteries off, 1: Battery 1 on 2: battery 2 on
  long requested_battery    '0: all batteries off, 1: Battery 1 on 2: battery 2 on
  long battery_switch_sound
  long battery_switch_time
  long auto_battery_switch_timeout
  long v_bat1
  long v_bat2

  ' Voltage tresholds
  long minimal_Vin               ' Minimal Vin supply
  long warning_Vin               ' Warning supply voltage    
  long switch_Vin                ' Switch supply voltage
  
  ' Power outputs
  long switch_state[6]
  long ok_output_state
  long out0_output_state

PUB start
'' Start driver - starts a cog
'' returns false if no cog available
'' may be called again to change settings

  stop                          ' stop cog if running from before
  
  cog := cognew(manage, @cog_stack) + 1 
  t.Pause1ms(1)                 ' wait  for cog to start (300us on 80Mhz)
                                
  initialize
  
  return cog

PUB stop
'' Stop driver - frees a cog
  if cog
    cogstop(cog~ - 1)

PRI initialize | i  
  output_manager_counter := 0
  
  ' Set default auto_battery_switch_timeout
  auto_battery_switch_timeout := default_auto_battery_switch_timeout
  
  ' Set default Vin voltage tresholds
  minimal_Vin := 22000
  warning_Vin := 22600
  switch_Vin  := 24000
  
  ' Reset switch states
  i := 0
  repeat n_switches
    switch_state[i++] := false

PUB updateBatteryVoltages(v1, v2)
  v_bat1 := v1
  v_bat2 := v2

PUB requestBattery(N)
  requested_battery := N

  ' Provide some time for the battery switch
  t.Pause1ms(10)  

  ' Return the currently active battery
  return active_battery
  
    
PRI manage | t1
  
  initialize

  t1 := cnt
  repeat
  
    managePowerOutputs
    manageBatteries
    
    if (||t1) - (||cnt) => (clkfreq/1000)
      oneMScounter++
    
    ' Wrap!! TODO nicer way to do this 1ms timer
    if oneMScounter < 0 
      oneMScounter := 0
      
    output_manager_counter++
    
    sound.BeepHz(output_manager_counter, 1000000,64)
  
PRI managePowerOutputs
  updateSwitches

PRI manageBatteries | switch_time_diff
  ' Switch batteries if requested
  if requested_battery <> active_battery
    selectBattery(requested_battery)

  ' Check voltage levels
  if checkBelowWarningVoltage
     lowVoltageWarning 

  ' Battery management and automatic switch from Bat1 to Bat 2
  ' WARNING!  Can only work if battery properties of both batteries are properly set!!
  if auto_battery_switch == true AND getBatteryVoltage(active_battery) < switch_Vin
    ' Prevent fast back and forth switching by checking if the difference of the voltages is larger than a certain hysteresis value
    switch_time_diff := (oneMScounter - battery_switch_time) '[ms]
    if switch_time_diff => auto_battery_switch_timeout
      selectFullestBattery
  
PUB selectFullestBattery | fullest_battery

    fullest_battery := getFullestBattery
    
    ' Select none of the batteries if the fullest is below the minimal voltage
    if isBatteryVoltageOK(fullest_battery) == false
        fullest_battery := 0

    return requestBattery(fullest_battery)

' === Return the index of the fullest battery ===
PUB getFullestBattery
    if getBatteryVoltage(1) => getBatteryVoltage(2)
      return 1
    else
      return 2
  
' === Returns the voltage of the specified battery ===      
PUB getBatteryVoltage(i)
    case i
        1: return v_bat1
        2: return v_bat2
    return 0
        
' === Check if battery voltage is above minimal voltage === 
PUB isBatteryVoltageOK(i)
    return (getBatteryVoltage(i) => minimal_Vin) 

' === Check if battery warning is required (both batteries below warning_Vin) ===
PRI checkBelowWarningVoltage
    return (getBatteryVoltage(1) =< warning_Vin AND getBatteryVoltage(2) =< warning_Vin)
   

' === Produce battery warning signal ===
PRI lowVoltageWarning
    sound.BeepHz(BUZZ, 1000, 1000000/4)
    sound.BeepHz(BUZZ, 500, 1000000/2)
    sound.BeepHz(BUZZ, 1000, 1000000/4)
    sound.BeepHz(BUZZ, 500, 1000000/2)
    sound.BeepHz(BUZZ, 1000, 1000000/4)
    sound.BeepHz(BUZZ, 500, 1000000/2)
    
'Switch battery 1 or 2. Disable outputs to cut all output current off from darlington
'Otherwise the Fet won't switch off properly
'Check for minimal voltage here?
'Ensures that all ouputs are off when switching from no battery to A battery
PRI selectBattery(N)
  if active_battery == 0 OR N == 0 
    SwitchAllOff         ' Switch off all outputs

  case N
    0: OUTA[PWRBAT1]:=0   
       OUTA[PWRBAT2]:=0
       DIRA[PWRBAT1]:=0
       DIRA[PWRBAT2]:=0
       active_battery := N 
       
       if battery_switch_sound == true
         sound.BeepHz(BUZZ, NOTE_E7, 1000000/16) 
         sound.BeepHz(BUZZ, 0, 1000000/32) 
         sound.BeepHz(BUZZ, NOTE_E3, 1000000/8)    
       
    1: DIRA[PWRBAT1]:=1
       OUTA[PWRBAT1]:=1 
       DIRA[PWRBAT2]:=0
       OUTA[PWRBAT2]:=0
       active_battery := N
       
       if battery_switch_sound == true
         sound.BeepHz(BUZZ, NOTE_E7, 1000000/16)  
         sound.BeepHz(BUZZ, 0, 1000000/32)
         sound.BeepHz(BUZZ, NOTE_E5, 1000000/32)

    2: DIRA[PWRBAT1]:=0
       OUTA[PWRBAT1]:=0 
       DIRA[PWRBAT2]:=1
       OUTA[PWRBAT2]:=1
       active_battery := N
       
       if battery_switch_sound == true
         sound.BeepHz(BUZZ, NOTE_E7, 1000000/16) 
         sound.BeepHz(BUZZ, 0, 1000000/32) 
         sound.BeepHz(BUZZ, NOTE_E5, 1000000/32)
         sound.BeepHz(BUZZ, 0, 1000000/32)
         sound.BeepHz(BUZZ, NOTE_E5, 1000000/32)
  
  battery_switch_time := oneMScounter
    
  return N
  
  
  
' ====== OUTPUTS ======
' Switch output 0 to 5. 0= all off
PRI SwitchAllOff
  setSwitch(0, false)
  setSwitch(1, false)
  setSwitch(2, false)
  setSwitch(3, false)
  setSwitch(4, false) 
  setSwitch(5, false) 

PUB setSwitch(N, req_state)
  switch_state[N] := req_state

PRI updateSwitches | i

  updateOkOutput
  updateOut0Output
  
  i := 0
  repeat n_switches
    if switch_state[i] == true
      SwitchOn(i)
    elseif switch_state[i] == false
      SwitchOff(i)
    i++

PRI updateOkOutput
  if ok_output_state == true
    SetOK(1)
  else
    SetOK(0)
  
PRI updateOut0Output
  if out0_output_state == true
    SetOUT0(1)
  else
    SetOUT0(0)
    
PRI SwitchOn(N)
  Case N
    0: DIRA[PWRCONTR]:=1
       OUTA[PWRCONTR]:=1
    1: DIRA[PWRPC1]:=1
       OUTA[PWRPC1]:=1
    2: DIRA[PWRPC2]:=1
       OUTA[PWRPC2]:=1
    3: DIRA[PWRDR1]:=1
       OUTA[PWRDR1]:=1
    4: DIRA[PWRDR2]:=1
       OUTA[PWRDR2]:=1
    5: DIRA[PWRAUX]:=1
       OUTA[PWRAUX]:=1
  
  'Ensure that not all outputs can be turned on at the same time (to limit peak currents)
  t.Pause1ms(10)

PRI SwitchOff(N)
  Case N
    0: DIRA[PWRCONTR]:=0
       OUTA[PWRCONTR]:=0
    1: DIRA[PWRPC1]:=0
       OUTA[PWRPC1]:=0
    2: DIRA[PWRPC2]:=0
       OUTA[PWRPC2]:=0
    3: DIRA[PWRDR1]:=0
       OUTA[PWRDR1]:=0
    4: DIRA[PWRDR2]:=0
       OUTA[PWRDR2]:=0
    5: DIRA[PWRAUX]:=0
       OUTA[PWRAUX]:=0
       


PUB requestOkOutputState(value)
  ok_output_state := value
  t.Pause10us(10)
  return GetOK 
  
PUB requestOUT0OutputState(value)
  out0_output_state := value
  t.Pause10us(10)
  return GetOUT0
  
' ---------------- Set OK output on or off (1 or 0) ---------------------------------------
PRI SetOK(Value)
  if Value==0
    DirA[OK]:=0
    OUTA[OK]:=0
  else
    DirA[OK]:=1
    OUTA[OK]:=1
    
' ---------------- Set OUT0 output on or off (1 or 0) ---------------------------------------
PRI SetOUT0(Value)
  if Value==0
    DirA[OUT0]:=0  
    OUTA[OUT0]:=0
  else  
    DirA[OUT0]:=1  
    OUTA[OUT0]:=1

' ---------------- Get status OK output (on=1 or off=0) ---------------------------------------
PUB GetOK
  if InA[OK]==1
    Return 1
  else
    Return 0  

' ---------------- Get status OK output (on=1 or off=0) ---------------------------------------
PUB GetOUT0
  if InA[OUT0]==1
    Return 1
  else
    Return 0   

' ---------------- Get EMERin input (on=1 or off=0) ---------------------------------------
PUB GetEMERin
  Return InA[EMERin]  

' ---------------- Get IN0 input (on=1 or off=0) ---------------------------------------
PUB GetIN0
  Return InA[IN0]
  
' ---------------- Get status CONTR output (on=1 or off=0) ---------------------------------------
PUB GetCONTR
  if InA[PWRCONTR]==1
    Return 1
  else
    Return 0   

' ---------------- Get status PC1 output (on=1 or off=0) ---------------------------------------
PUB GetPC1
  if InA[PWRPC1]==1
    Return 1
  else
    Return 0                 

' ---------------- Get status PC2 output (on=1 or off=0) ---------------------------------------
PUB GetPC2
  if InA[PWRPC2]==1
    Return 1
  else
    Return 0                 

' ---------------- Get status DR1 output (on=1 or off=0) ---------------------------------------
PUB GetDR1
  if InA[PWRDR1]==1
    Return 1
  else
    Return 0                 

' ---------------- Get status DR2 output (on=1 or off=0) ---------------------------------------
PUB GetDR2
  if InA[PWRDR2]==1
    Return 1
  else
    Return 0                 

  
' ---------------- Get status AUX output (on=1 or off=0) ---------------------------------------
PUB GetAUX
  if InA[PWRAUX]==1
    Return 1
  else
    Return 0         
        


' ====== getters and setters ======

PUB getActiveBattery
  return active_battery
  
PUB getBatterySwitchTime
  return battery_switch_time
  
PUB getOneMScounter
  return oneMScounter
  
PUB setAutoBatterySelect(state)
  auto_battery_switch := state

PUB getAutoBatterySelect
  return auto_battery_switch

PUB setMinimalVin(Vin)
  minimal_vin := Vin

PUB getMinimalVin
  return minimal_vin

PUB setWarningVin(Vin)
  warning_vin := Vin

PUB getWarningVin
  return warning_vin

PUB setSwitchVin(Vin)
  switch_vin := Vin

PUB getSwitchVin
  return switch_vin
  
PUB setBatterySwitchSound(state)
  battery_switch_sound := state

PUB getBatterySwitchSound
  return battery_switch_sound  
  
PUB setBatterySwitchTimeout(timeout)
  auto_battery_switch_timeout := timeout

PUB getBatterySwitchTimeout
  return auto_battery_switch_timeout 
  
DAT   
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