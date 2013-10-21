{{
  ADC controller

  Reads the ADS1000 ADC form Texas Instruments

  Author:     Dennis Wokke
  Copyright:  VEDS Group 2011

  Target:     Propeller
  Compiler:   Propeller Tool

}}

CON

'ADC
  i2cSCL        = 16
  i2cSDA        = 17 
  ADC_ADDR      = $90
  C_ADC_OVERSAMPLING_LOG = 4 'To make 12-bit value

  
VAR

  long cog
  long ADCResults[64]
  BYTE counter
  BYTE ADC_Status
  long tValue
  long stackspace[100]
  long medium_height

  
OBJ
                   
  i2cObject      : "basic_i2c_driver"

PUB adc_Start : ok

  ok := cog := cognew(ADC_Handler, @stackspace) + 1      '(@SensorResult)

PUB adc_Stop

  if cog
    cogstop(cog~ - 1)

PUB adc_init

  counter := 0
  'Initialize I2C communication module
  i2cObject.Initialize(i2cSCL)                                            
  ' pause 2 seconds
  repeat 2
        'PC.str(string(".",13))
        waitcnt((clkfreq/2)+cnt)

  adc_config

  waitcnt((clkfreq/2)+cnt)  


  waitcnt((clkfreq/2)+cnt)
  counter := 0
  repeat until counter == 64
    tValue := adc_read
    ADC_Status := tValue & $000000FF
    ADCResults[counter] := tValue >> 8
    counter++

  counter := 0

  RETURN  ADC_Check

PUB adc_Handler '(ADCResult)

  'Initialize I2C communication module
  i2cObject.Initialize(i2cSCL)                                            

  adc_check

  repeat
    ' Read the ADC value
    tValue := adc_read
    ADC_Status := tValue & $000000FF
    ADCResults[counter] := tValue >> 8
    counter++
    counter &= 63 'Set to zero if 64 is reached


PUB adc_get_result 

  RETURN FilterMean

PUB adc_read
' FUNCTION : Read the result of the ADC sensor
' INPUT    : Address      (I2C Address of the ADS1000 in HEX)
' OUTPUT   : Result of the ADC value
' REMARKS  : Two bytes

  RETURN i2cObject.readADCValue(i2cSCL,ADC_ADDR)

PUB adc_get_control_register
' FUNCTION : Read the ADS100 config register
' INPUT    : Address      (I2C Address of the ADS1000 in HEX)
' OUTPUT   : Config register
' REMARKS  : None

  RETURN ADC_Status
  
PUB adc_check
' FUNCTION : Check if all sensors are present and show the result in the PC terminal
' INPUT    : None
' OUTPUT   : Result of the check in the PC terminal
' REMARKS  : None

  RETURN i2cObject.devicePresent(i2cSCL,ADC_ADDR)

PUB FilterMean | nIndex, nTemp

  nIndex := 0
  nTemp  := 0

  REPEAT WHILE nIndex < 32
    nTemp := nTemp + ADCResults[nIndex]
    nIndex++
   
  RETURN (nTemp >> C_ADC_OVERSAMPLING_LOG)

PRI adc_config
  'Continouos sampling and a gain of 1 is set!
  i2cObject.writeADCConfig(i2cSCL,ADC_ADDR, $00)   