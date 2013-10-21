{{
  height_controller.spin

  Controls the height actuator using an 12-bit ADC as input and a
  Pololu VNH5019 motor driver carrier

  This code contains the following functions:
    - init
    - Handles the height actuator
        - Low position (automatic, limit switch is in the actuator)
        - High position (automatic, limit switch is in the actuator)
        - Mid position calculated using ADC
        - Max. time allowed for no movement is 500ms, after that the
          actuator is turned off


  Author:     Dennis Wokke
  Copyright:  VEDS Group 2011
  
  - Version 1.30       Date: 2013-09-19        Modifier: Okke Hendriks

  - Added:      PUB get_current_pos, returns the current position of the
                lift
    
  - Added:      At height_start the current position of the lift is
                determined and stored in current_pos_MLHS.
                1: Undefined position
                2: Low position
                3: Mid position
                4: High position
  - BUG FIX:    Sending of message that the height position has been reached
                has been fixed (again? :P). Commented 'requested_position := POS_IDLE'
                at function motor_stop. Added functions that are used to compare
                requested and actual position 
  
}}

'' Declare symbolic, global constants 
CON
  'Pinning
  PIN_INA       = 0  'Clockwise motor direction
  PIN_INB       = 1  'Counterclockwise motor direction

  'Status
  DEACTIVATED   = 0
  POS_STOP      = 1
  POS_LOW       = 2
  POS_MID       = 3
  POS_HIGH      = 4
  POS_IDLE      = -1

  HEIGHT_HYSTERESIS = 36 ' 1 bit in ADC ~=0.04mm max. deviation = 2 * 24 * 0.04 = 1.92mm
  WAIT_TIME     = 10000000

'' Declare symbol object references
OBJ
  ADC      : "adc"
  PC2      : "FullDuplexSerial_rr005"

VAR
  LONG mid_position
  LONG high_position
  LONG low_position
  LONG current_pos
  BYTE requested_position '0 COG stopped, 1 stopped, 2 low, 3 mid, 4 high
  LONG heightstackspace[100]
  BYTE height_cog
  BYTE direction
  LONG height_timer
  LONG prev_pos
  LONG current_pos_MLHS

' ---------------- Main program ---------------------------------------
PUB height_start : ok1

  ok1 := FALSE

  if (ADC.adc_init)
    if (ADC.adc_Start)
       ok1 := height_cog := cognew(height_handler, @heightstackspace) + 1
  
  'Check at which position the lift is positioned
  current_pos := ADC.adc_get_result
  if (current_pos < low_position)
    current_pos_MLHS := POS_LOW        
  elseif (current_pos > high_position)
    current_pos_MLHS := POS_HIGH             
  elseif (current_pos > (mid_position - HEIGHT_HYSTERESIS) and (current_pos < (mid_position + HEIGHT_HYSTERESIS))  )
    current_pos_MLHS := POS_MID
  else
    current_pos_MLHS := POS_STOP


  RETURN ok1      
  
PUB height_init (val) 
' FUNCTION : Initializes the height controller
' INPUT    : ADC value of the mid position
' OUTPUT   : TRUE on success
'            FALSE on failure
' REMARKS  : None

  RESULT :=FALSE

  'Set pin directions
  DIRA[PIN_INA] := 1
  DIRA[PIN_INB] := 1
  'Turn all pins off
  OUTA[PIN_INA]~
  OUTA[PIN_INB]~

  mid_position := val

  RETURN
   
PUB height_stop

  ADC.adc_stop

  if height_cog
    cogstop(height_cog~ - 1)

  requested_position := DEACTIVATED

PUB height_set_position (pos)
  if( pos =>1 and pos =<4)
    requested_position := pos
    height_timer := CNT
    prev_pos := current_pos + 100  

PRI height_handler

  DIRA[PIN_INA] := 1
  DIRA[PIN_INB] := 1

  repeat
    current_pos := ADC.adc_get_result
    
    if (requested_position == POS_LOW)
      'Drive motor to low position
      if (current_pos < low_position)
        current_pos_MLHS := POS_LOW
        motor_stop
      else
        turn_cw
        
    elseif (requested_position == POS_HIGH)
      'Drive motor to heigh position
      if (current_pos > high_position)
        current_pos_MLHS := POS_HIGH
        motor_stop
      else
        turn_ccw
                
    elseif (requested_position == POS_MID)
      'Drive motor to mid position
      if (current_pos < (mid_position - HEIGHT_HYSTERESIS))
        'Move up
        turn_ccw

      elseif (current_pos > (mid_position + HEIGHT_HYSTERESIS))
        'Move down
        turn_cw
      else
        'Stop! Mid position reached
        current_pos_MLHS := POS_MID
        motor_stop
    elseif (requested_position == POS_STOP)
      current_pos_MLHS := POS_STOP
      motor_stop
             

    
    'Stop all movement when no change in ADC value after 0.5s    
    if( CNT => height_timer + WAIT_TIME)
      if (((prev_pos - HEIGHT_HYSTERESIS/2)  < current_pos) and ((prev_pos + HEIGHT_HYSTERESIS/2)  > current_pos))
        current_pos_MLHS := POS_STOP   
        motor_stop
      else
        height_timer := CNT

        prev_pos := current_pos
           
PUB get_current_pos
  'Check at which position the lift is positioned

  if (current_pos < low_position)
    current_pos_MLHS := POS_LOW        
  elseif (current_pos > high_position)
    current_pos_MLHS := POS_HIGH             
  elseif (current_pos > (mid_position - HEIGHT_HYSTERESIS) and (current_pos < (mid_position + HEIGHT_HYSTERESIS))  )
    current_pos_MLHS := POS_MID
  else
    current_pos_MLHS := POS_STOP
    
  RETURN current_pos_MLHS  
  
PUB get_requested_pos

  RETURN requested_position
 
PUB height_get_mid_position

  RETURN mid_position
  
PUB height_get_position

  RETURN adc.adc_get_result

PUB turn_cw

  OUTA[PIN_INB]~
  OUTA[PIN_INA]~~
  
PUB turn_ccw

  OUTA[PIN_INA]~
  OUTA[PIN_INB]~~
  
PRI motor_stop

  OUTA[PIN_INB]~
  OUTA[PIN_INA]~
   
  'Mid position reached
  'requested_position := POS_IDLE

PUB height_get_config_register
  return adc.adc_get_control_register

PUB height_set_mid_position (val2)

  mid_position := val2

  RETURN

PUB height_get_cog
  RETURN height_cog

PUB height_set_high(valhigh)

  high_position := valhigh

PUB height_set_low (vallow)

  low_position := vallow

PUB height_get_high
  return high_position

PUB height_get_low

  return low_position
  
