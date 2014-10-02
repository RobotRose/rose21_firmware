{{
************
*  Timing  *
************

This object provides time delay and time synchronization functions.
}}

CON
  
  _10us = 1_000_000 /        10                   'Divisor for 10 us
  _1ms  = 1_000_000 /     1_000                   'Divisor for 1 ms
  _1s   = 1_000_000 / 1_000_000                   'Divisor for 1 s


VAR
  long Delay
  long SyncPoint
  long ClkCycles
  

PUB Pause10us(Period) 
{{Pause execution for Period (in units of 10 us).}}

  ClkCycles := ((clkfreq / _10us * Period) - 4296) #> 381    'Calculate 10 us time unit
  waitcnt(ClkCycles + cnt)                                   'Wait for designated time


PUB Pause1ms(Period) 
{{Pause execution for Period (in units of 1 ms).}}

  ClkCycles := ((clkfreq / _1ms * Period) - 4296) #> 381     'Calculate 1 ms time unit
  waitcnt(ClkCycles + cnt)                                   'Wait for designated time
  

PUB Pause1s(Period) 
{{Pause execution for Period (in units of 1 s).}}

  ClkCycles := ((clkfreq / _1s * Period) - 4296) #> 381      'Calculate 1 s time unit
  waitcnt(ClkCycles + cnt)                                   'Wait for designated time

  
PUB MarkSync10us(Period)
  Delay := (clkfreq / _10us * Period) #> 381                 'Calculate 10 us time unit 
  SyncPoint := cnt

  
PUB WaitSync  
  waitcnt(SyncPoint += Delay)     
  
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