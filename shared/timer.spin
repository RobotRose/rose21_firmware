'' Rose timer
'' Author: Okke Hendriks
'' Date: 26-08-2014
''

DAT

CON   
  
  
OBJ
  t             : "Timing"             ' Full duplex serial communication                   
  
VAR
  byte cog  
  long cog_stack[50]
  byte memory_set
  byte memory_initialized
  long timer_address
  long timer_value_address
  long timer_running_address
  long nr_timers
  
PUB start(new_timer_address, new_timer_value_address, new_timer_running_address, new_nr_timers)
  '' Start driver - starts a cog
  '' returns 0 if no cog available

  stop                          ' stop cog if running from before
  
  cog := cognew(updateTimers, @cog_stack) + 1 
  t.Pause10us(50)               ' wait for cog to start (300us on 80Mhz)
  
  setMemory(new_timer_address, new_timer_value_address, new_timer_running_address, new_nr_timers)
                              
  return cog
  
PUB stop
  '' Stop driver - frees a cog
  memory_set          := false
  memory_initialized  := false
  
  if cog
    cogstop(cog~ - 1)
  return 0

PUB setMemory(new_timer_address, new_timer_value_address, new_timer_running_address, new_nr_timers)
  timer_address         := new_timer_address
  timer_value_address   := new_timer_value_address
  timer_running_address := new_timer_running_address
  nr_timers             := new_nr_timers
  
  memory_set            := true
 
PUB isMemorySet
  return memory_set

PUB isMemoryInitialized
  return memory_initialized

PUB isReady
  return memory_set AND memory_initialized
       
PUB getTimerAddress
  return timer_address

PUB getTimerSetValueAddress
  return timer_value_address
  
PUB getTimerRunningAddress
  return timer_running_address  
  
pub getNrOfTimers
  return nr_timers

PRI initMem | i 
  i := 0
  repeat nr_timers       
    long[timer_address][i]          := -1
    long[timer_value_address][i]    := -1
    byte[timer_running_address][i]  := false
    i++
  return true

PRI updateTimers | t1, clk_cycles, i
  
  ' Wait for memory addresses assigned before initializing it to default values
  repeat while not memory_set
  memory_initialized := initMem
  
  ' Keep looping
  t1 := cnt
  repeat
    ' Update all timers
    i := 0
    repeat nr_timers 
      if byte[timer_running_address][i] AND long[timer_address][i] > 0
        long[timer_address][i]--
      i++

    ' Loop at 1ms rate
    ' Calculate clockcycles to wait, takin into account the timer updating
    clk_cycles := 1 #> ((clkfreq / 1_000) - 4296) - (cnt - t1) #> 381 
    waitcnt(cnt + clk_cycles)
    t1 := cnt
  
    
PUB setTimer(i, delay)
  if i => 0 AND i < nr_timers
    long[timer_value_address][i] := 0 #> delay <# 2_147_483_647
    return true
    
  return false
  
' Get the actual counting down value
PUB getTimer(i)
  if i => 0 AND i < nr_timers
    return long[timer_address][i]

  return -1

' Get the set max value  
PUB getTimerSetValue(i)
  if i => 0 AND i < nr_timers
    return long[timer_value_address][i]

  return -1
  
PUB isTimerRunning(i)
  return byte[timer_running_address][i]

PUB resetTimer(i)
  if i => 0 AND i < nr_timers
    long[timer_address][i] := long[timer_value_address][i]
    return true

  return false
  
PUB startTimer(i)
  if i => 0 AND i < nr_timers
    resetTimer(i)
    byte[timer_running_address][i] := true
    return true

  return false
  
PUB stopTimer(i)
  if i => 0 AND i < nr_timers
    resetTimer(i)
    byte[timer_running_address][i] := false
    return true

  return false
  
  
PUB pauseTimer(i)
  if i => 0 AND i < nr_timers
    byte[timer_running_address][i] := false
    return true

  return false
  
PUB unpauseTimer(i)
  if i => 0 AND i < nr_timers
    byte[timer_running_address][i] := true
    return true

  return false  

  
PUB checkTimer(i)
  if i => 0 AND i < nr_timers
    return long[timer_address][i] == 0

  return false
  
PUB checkAndResetTimer(i)
  if i => 0 AND i < nr_timers
    ' Reset the timer if it is at zero
    if long[timer_address][i] == 0
      resetTimer(i)
      return true
      
  return false
  
