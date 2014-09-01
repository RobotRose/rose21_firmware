'' Rose communications
'' Author: Okke Hendriks
'' Date: 26-08-2014
''
''
''


DAT

CON   
  ' Characters
  CR = 13
  LF = 10
  CS = 0
  CE = 11                 
                          
  '' Strings
  lMaxStr         = 257             ' Stringlength is 256 + 1 for 0 termination
  StringSize      = lMaxStr-1
  bytebuffersize  = 2048
  LineLen         = bytebuffersize  ' Buffer size for incoming line
  MaxWaitTime     = 5               ' [ms] wait time for incoming string 'TODO take this lower (5ms when comm with PC) 
   
  ' Command handling
  max_parameters        = 50
  max_substring_length  = 10
  
  UNKOWN_COMMAND_NUMBER = 102
  
OBJ
  ser           : "full_duplex_serial_005"             ' Full duplex serial communication 
  sn            : "simple_numbers"                     ' For conversion of variables to strings
  strings       : "STRINGS2hk"                        
  
VAR
  ' State
  byte intitialized
  
  ' Input string handling
  byte StrBuf[lMaxStr]            'String buffer for receiving chars from serial port
  long StrSP, StrP, StrLen        'Instring semaphore, counter, Stringpointer, String length
  byte Ch
  
  ' Command handling
  long command  
  long parameters[max_parameters]
  byte substring[max_substring_length]
  long nr_of_parameters
  
  ' Output strings
  byte substr1[2*max_substring_length]
  byte substr2[max_substring_length]
  
PUB initialize
    intitialized := false
    
    command           := 0
    nr_of_parameters  := 0
    bytefill(@StrBuf, 0, lMaxStr)
    bytefill(@substring, 0, max_substring_length)
    longfill(@parameters, -1, max_parameters)
    
    intitialized := true

PUB isInitialized
  return intitialized

PUB checkForCommand | check_command
  check_command := extractCommand
  
  ' Valid command received
  if check_command <> -1
    command := check_command
    return true
  else
    return false

PUB getCommand
  return command      

' === Extract command from string === 
PRI extractCommand | i, char, extracted_command
  i                 := 0
  extracted_command := -1 
  
  ' Reset the string pointer
  StrP  := 0
  bytefill(@substring, 0, max_substring_length)  
  
  ' Eat string, look for start character $, stop when at max length
  char := getch
  repeat while char <> "$" AND char <> 0 AND i++ < max_substring_length
    char := getch
  
  ' Check if $ was found, otherwise return -1
  if char <> "$"
    return -1
  
  ' Eat string, look for comma, stop when at end-of-string character or at max length
  i     := 0
  char  := getch
  repeat while char <> "," AND char <> 0 AND i < max_substring_length
    if isNumber(char) OR isMinus(char)
      substring[i++] := char
    'else
      ' Not a valid character, skip
    char := getch
    
  ' Check if last char was a comma or end-of-string character, then we have a command
  ' Try to parse it to a decimal number
  ' If not return -1 to indicate no command
  if isComma(char)
    extracted_command := ser.strtodec(@substring)
    extractParameters    
    return extracted_command
  else
    return -1
  
  
' === Extract parameters from string ===
PRI extractParameters | i, char
  nr_of_parameters := 0
  i := 0
  
  ' Clear buffers
  bytefill(@substring, 0, max_substring_length)  
  longfill(@parameters, -1, max_parameters)
  
  ' Get parameters until end-of-string character or maxlength
  ' Always end a parameter with a comma, also the last one!
  char := getch  
  repeat while char <> 0 AND i < max_substring_length  
    if isNumber(char) OR isMinus(char)
      substring[i++] := char
    elseif isComma(char) AND nr_of_parameters < max_parameters
      ' Store parameter
      parameters[nr_of_parameters++] := ser.strtodec(@substring)
      ' Clear substring
      i := 0
      bytefill(@substring, 0, max_substring_length)
    'else
      ' Invalid character detected, just skip it 
    
    ' Get next character
    char := getch                 
  
  return nr_of_parameters

' ---------------- Get next character from string ---------------------------------------
PRI getch
  return Byte[@StrBuf][StrP++]

PRI isNumber(char)
  return (char == "0" OR char == "1" OR char == "2" OR char == "3" OR char == "4" OR char == "5" OR char == "6" OR char == "7" OR char == "8" OR char == "9")
  
PUB isMinus(char)
  return char == "-"
  
pub isComma(char)
  return char == ","

' === Get the number of parameters with the current command === 
PUB getNrOfParameters
  return nr_of_parameters
 
' === Get a specific parameter numbered from 1 === 
PUB getParam(i)
  i--
  if i < 0 OR i > nr_of_parameters
    return -1
    
  return parameters[i]
  
' === Get a specific boolean parameter numbered from 1 === 
' if value is tother than 1 or 0 it will return false
PUB getBoolParam(i)
  i--
  if i < 0 OR i > nr_of_parameters
    return false
   
  if parameters[i] == 1 
    return true
  else
    return false

PUB nrOfParametersCheck(expected_nr_of_parameters)
  return expected_nr_of_parameters == nr_of_parameters
  
PUB getMaxStringLength
  return lMaxStr
  
PUB getStringBuffer
  return @StrBuf
  
PUB getMaxWaitTime
  return MaxWaitTime
  
' === Standard string generation functions ===
PUB getCommandStr(command_number) 
  clearSubstrings
  substr1[0] := "$"
  substr2[0] := ","
  return strings.concatenate(strings.concatenate(@substr1, sn.dec(command_number)), @substr2)

PUB getBoolStr(boolean)
  clearSubstrings
  
  if boolean
    substr1[0] := "1"
    substr1[1] := ","
    return @substr1
  else
    substr1[0] := "0"
    substr1[1] := ","
    return @substr1 
    
PUB getDecStr(decimal) | decimal_string
  clearSubstrings
  decimal_string := sn.dec(decimal)
  bytemove(@substr1, decimal_string, strsize(decimal_string))  
  substr2[0] := ","
  return strings.concatenate(@substr1, @substr2)
  
PUB getUnkownCommandStr
  return getCommandStr(UNKOWN_COMMAND_NUMBER)

PUB getEOLStr
  return string(CR)

PRI clearSubstrings
  bytefill(@substr1, 0 , max_substring_length)
  bytefill(@substr2, 0 , max_substring_length)
  