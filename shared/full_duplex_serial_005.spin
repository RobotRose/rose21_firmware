''********************************************
''*  Full-Duplex Serial Driver v1.2          *
''*  Author: Chip Gracey, Jeff Martin        *
''*  Copyright (c) 2006-2009 Parallax, Inc.  *
''*  See end of file for terms of use.       *
''********************************************

{-----------------REVISION HISTORY-----------------
 v1.2 - 5/7/2009 fixed bug in dec method causing largest negative value (-2,147,483,648) to be output as -0.
 v1.1 - 3/1/2006 first official release.

 RR20090815  _rr001   Allow variable buffer size in CON
                      Add rxavail - returns true if received bytes available.

 RR20090816  _rr002   Add compatable calls from Kye's SerialEngine for testing by Dr_Acula
 RR20090817  _rr003   Add mode bit 4 (add LF after CR), remove some calls
 RR20090818  _rr004   Remove mode bit 4 (add LF after CR) as per Dr_Acula
 rr0005               HJK Extended with functions from Parallax serial terminal StrIn, etc. 

}

CON

  bufsiz = 128 '16              'buffer size (16, 32, 64, 128, 256, 512) must be factor of 2, max is 256
  bufmsk = bufsiz - 1           'buffer mask used for wrap-around ($00F, $01F, $03F, $07F, $0FF, $1FF)

  ''    Control Character Constants
''─────────────────────────────────────
  CS = 16  ''CS: Clear Screen      
  CE = 11  ''CE: Clear to End of line     
  CB = 12  ''CB: Clear lines Below 
  CR = 13  ''CR: Carriage Return

  HM =  1  ''HM: HoMe cursor       
  PC =  2  ''PC: Position Cursor in x,y          
  PX = 14  ''PX: Position cursor in X         
  PY = 15  ''PY: Position cursor in Y         

  NL = 13  ''NL: New Line        
  LF = 10  ''LF: Line Feed       
  ML =  3  ''ML: Move cursor Left          
  MR =  4  ''MR: Move cursor Right         
  MU =  5  ''MU: Move cursor Up          
  MD =  6  ''MD: Move cursor Down
  TB =  9  ''TB: TaB          
  BS =  8  ''BS: BackSpace          
           
  BP =  7  ''BP: BeeP speaker 

VAR

  long  cog                     'cog flag/id

  long  rx_head                 '9 contiguous longs
  long  rx_tail
  long  tx_head
  long  tx_tail
  long  rx_pin
  long  tx_pin
  long  rxtx_mode
  long  bit_ticks
  long  buffer_ptr
                     
  byte  rx_buffer[bufsiz]           'transmit and receive buffers
  byte  tx_buffer[bufsiz]

'  Byte StrBuf[bufsiz]           'String buffer for receiving line from serial port  (Readln)

PUB start(rxpin, txpin, mode, baudrate) : okay

'' Start serial driver - starts a cog
'' returns false if no cog available
''
'' mode bit 0 = invert rx
'' mode bit 1 = invert tx
'' mode bit 2 = open-drain/source tx
'' mode bit 3 = ignore tx echo on rx

  stop
  longfill(@rx_head, 0, 4)
  longmove(@rx_pin, @rxpin, 3)
  bit_ticks := clkfreq / baudrate
  buffer_ptr := @rx_buffer
  okay := cog := cognew(@entry, @rx_head) + 1


PUB stop

'' Stop serial driver - frees a cog

  if cog
    cogstop(cog~ - 1)
  longfill(@rx_head, 0, 9)

PUB rxflush

'' Flush receive buffer

  repeat while rxcheck => 0
  
PUB rxavail : truefalse

'' Check if byte(s) available
'' returns true (-1) if bytes available

  truefalse := rx_tail <> rx_head

PUB Char(ch)
'' Send char. Added for compatibility with Parallax serial terminal
  Tx(Ch)
  
PUB rxcheck : rxbyte

'' Check if byte received (never waits)
'' returns -1 if no byte received, $00..$FF if byte

  rxbyte--
  if rx_tail <> rx_head
    rxbyte := rx_buffer[rx_tail]
    rx_tail := (rx_tail + 1) & bufmsk


PUB rxtime(ms) : rxbyte | t

'' Wait ms milliseconds for a byte to be received
'' returns -1 if no byte received, $00..$FF if byte
  
  t := cnt
  repeat until (rxbyte := rxcheck) => 0 or (cnt - t) / (clkfreq / 1000) > ms
  
PUB rxtime_us(us) : rxbyte | t

'' Wait us microseconds for a byte to be received
'' returns -1 if no byte received, $00..$FF if byte

  t := cnt
  repeat until (rxbyte := rxcheck) => 0 or (cnt - t) / (clkfreq / 1000000) > us

PUB rx : rxbyte

'' Receive byte (may wait for byte)
'' returns $00..$FF

  repeat while (rxbyte := rxcheck) < 0


PUB tx(txbyte)

'' Send byte (may wait for room in buffer)

  repeat until (tx_tail <> (tx_head + 1) & bufmsk)
  tx_buffer[tx_head] := txbyte
  tx_head := (tx_head + 1) & bufmsk

  if rxtx_mode & %1000
    rx

PUB CharIn : bytechr
{{Receive single-byte character.  Waits until character received.
  Returns: $00..$FF}}

  repeat while (bytechr := RxCheck) < 0

PUB str(stringptr)

'' Send string                    

  repeat strsize(stringptr)
    tx(byte[stringptr++])
    
PUB StrIn(stringptr)
{{Receive a string (carriage return terminated) and stores it (zero terminated) starting at stringptr.
Waits until full string received.
  Parameter:
    stringptr - pointer to memory in which to store received string characters.
                Memory reserved must be large enough for all string characters plus a zero terminator.}}
    
  StrInMax(stringptr, -1)

PUB StrInTime(stringptr,ms)
{{Receive a string (carriage return terminated) and stores it (zero terminated) starting at stringptr.
Waits until full string received or timeout.
  Parameter:
    stringptr - pointer to memory in which to store received string characters.
                Memory reserved must be large enough for all string characters plus a zero terminator.}}
    
  StrInMaxTime(stringptr, -1,ms)

PUB StrInMax(stringptr, maxcount)
{{Receive a string of characters (either carriage return terminated or maxcount in length) and stores it (zero terminated)
starting at stringptr.  Waits until either full string received or maxcount characters received.
  Parameters:
    stringptr - pointer to memory in which to store received string characters.
                Memory reserved must be large enough for all string characters plus a zero terminator (maxcount + 1).
    maxcount  - maximum length of string to receive, or -1 for unlimited.}}
    
  repeat while (maxcount--)                                                     'While maxcount not reached
    if (byte[stringptr++] := CharIn) == NL                                      'Get chars until NL
      quit
  byte[stringptr+(byte[stringptr-1] == NL)]~                                    'Zero terminate string; overwrite NL or append 0 char

PUB StrInMaxTime(stringptr, maxcount,ms)
{{Receive a string of characters (either carriage return terminated or maxcount in length) and stores it (zero terminated)
starting at stringptr.  Waits until either full string received, timed out, or maxcount characters received.
  Parameters:
    stringptr - pointer to memory in which to store received string characters.
                Memory reserved must be large enough for all string characters plus a zero terminator (maxcount + 1).
    maxcount  - maximum length of string to receive, or -1 for unlimited.}}
    
  repeat while (maxcount--)                                                     'While maxcount not reached
    case  (byte[stringptr++] := CharInTime(ms))
      NL,-1:quit                                                                'Get chars until NL or -1
  byte[stringptr+(byte[stringptr-1] == NL)]~  

PUB StrInMaxTimeUs(stringptr, maxcount,us)
{{Receive a string of characters (either carriage return terminated or maxcount in length) and stores it (zero terminated)
starting at stringptr.  Waits until either full string received, timed out, or maxcount characters received.
  Parameters:
    stringptr - pointer to memory in which to store received string characters.
                Memory reserved must be large enough for all string characters plus a zero terminator (maxcount + 1).
    maxcount  - maximum length of string to receive, or -1 for unlimited.}}
    
  repeat while (maxcount--)                                                     'While maxcount not reached
    case  (byte[stringptr++] := CharInTimeUs(us))
      NL,-1:quit                                                                'Get chars until NL or -1
  byte[stringptr+(byte[stringptr-1] == NL)]~  

PUB CharInTime(ms) : rxbyte | t      
{{
  Receive byte with timeout
}}
  t := cnt
  repeat until (rxbyte := rxcheck) => 0 or (cnt - t) / (clkfreq / 1000) > ms

PUB CharInTimeUs(us) : rxbyte | t      
{{
  Receive byte with timeout
}}
  t := cnt
  repeat until (rxbyte := rxcheck) => 0 or (cnt - t) / (clkfreq) > us

PUB dec(value) | i, x

'' Print a decimal number

  x := value == NEGX                                                            'Check for max negative
  if value < 0
    value := ||(value+x)                                                        'If negative, make positive; adjust for max negative
    tx("-")                                                                     'and output sign

  i := 1_000_000_000                                                            'Initialize divisor

  repeat 10                                                                     'Loop for 10 digits
    if value => i                                                               
      tx(value / i + "0" + x*(i == 1))                                          'If non-zero digit, output digit; adjust for max negative
      value //= i                                                               'and digit from value
      result~~                                                                  'flag non-zero found
    elseif result or i == 1
      tx("0")                                                                   'If zero digit (or only digit) output it
    i /= 10                                                                     'Update divisor


PUB hex(value, digits)

'' Print a hexadecimal number

  value <<= (8 - digits) << 2
  repeat digits
    tx(lookupz((value <-= 4) & $F : "0".."9", "A".."F"))


PUB bin(value, digits)

'' Print a binary number

  value <<= 32 - digits
  repeat digits
    tx((value <-= 1) & 1 + "0")

PUB StrToDec(stringptr) : value | lChar, index, multiply

    '' Converts a zero terminated string representation of a decimal number to a value

    value := index := 0
    repeat until ((lChar := byte[stringptr][index++]) == 0)
       if lChar => "0" and lChar =< "9"
          value := value * 10 + (lChar - "0")
    if byte[stringptr] == "-"
       value := - value
Return Value

PUB Clear
{{Clear screen and place cursor at top-left.}}
  
  Char(CS)

PUB ClearEnd
{{Clear line from cursor to end of line.}}
  
  Char(CE)
  
PUB ClearBelow
{{Clear all lines below cursor.}}
  
  Char(CB)
  
PUB Home
{{Send cursor to home position (top-left).}}
  
  Char(HM)
  
PUB Position(x, y)
{{Position cursor at column x, row y (from top-left).}}
  
  Char(PC)
  Char(x)
  Char(y)
  
PUB PositionX(x)
{{Position cursor at column x of current row.}}
  Char(PX)
  Char(x)
  
PUB PositionY(y)
{{Position cursor at row y of current column.}}
  Char(PY)
  Char(y)

PUB NewLine
{{Send cursor to new line (carriage return plus line feed).}}
  
  Char(NL)
  
PUB LineFeed
{{Send cursor down to next line.}}
  
  Char(LF)
  
PUB MoveLeft(x)
{{Move cursor left x characters.}}
  
  repeat x
    Char(ML)
  
PUB MoveRight(x)
{{Move cursor right x characters.}}
  
  repeat x
    Char(MR)
  
PUB MoveUp(y)
{{Move cursor up y lines.}}
  
  repeat y
    Char(MU)
  
PUB MoveDown(y)
{{Move cursor down y lines.}}
  
  repeat y
    Char(MD)
  
PUB Tab
{{Send cursor to next tab position.}}
  
  Char(TB)
  
PUB Backspace
{{Delete one character to left of cursor and move cursor there.}}
  
  Char(BS)
  
PUB Beep
{{Play bell tone on PC speaker.}}
  
  Char(BP)

DAT

'***********************************
'* Assembly language serial driver *
'***********************************

                        org     0
'
'
' Entry
'
entry                   mov     t1,par                'get structure address
                        add     t1,#4 << 2            'skip past heads and tails

                        rdlong  t2,t1                 'get rx_pin
                        mov     rxmask,#1
                        shl     rxmask,t2

                        add     t1,#4                 'get tx_pin
                        rdlong  t2,t1
                        mov     txmask,#1
                        shl     txmask,t2

                        add     t1,#4                 'get rxtx_mode
                        rdlong  rxtxmode,t1

                        add     t1,#4                 'get bit_ticks
                        rdlong  bitticks,t1

                        add     t1,#4                 'get buffer_ptr
                        rdlong  rxbuff,t1
                        mov     txbuff,rxbuff
                        add     txbuff,#bufsiz

                        test    rxtxmode,#%100  wz    'init tx pin according to mode
                        test    rxtxmode,#%010  wc
        if_z_ne_c       or      outa,txmask
        if_z            or      dira,txmask

                        mov     txcode,#transmit      'initialize ping-pong multitasking
'
'
' Receive
'
receive                 jmpret  rxcode,txcode         'run a chunk of transmit code, then return

                        test    rxtxmode,#%001  wz    'wait for start bit on rx pin
                        test    rxmask,ina      wc
        if_z_eq_c       jmp     #receive

                        mov     rxbits,#9             'ready to receive byte
                        mov     rxcnt,bitticks
                        shr     rxcnt,#1
                        add     rxcnt,cnt                          

:bit                    add     rxcnt,bitticks        'ready next bit period

:wait                   jmpret  rxcode,txcode         'run a chuck of transmit code, then return

                        mov     t1,rxcnt              'check if bit receive period done
                        sub     t1,cnt
                        cmps    t1,#0           wc
        if_nc           jmp     #:wait

                        test    rxmask,ina      wc    'receive bit on rx pin
                        rcr     rxdata,#1
                        djnz    rxbits,#:bit

                        shr     rxdata,#32-9          'justify and trim received byte
                        and     rxdata,#$FF
                        test    rxtxmode,#%001  wz    'if rx inverted, invert byte
        if_nz           xor     rxdata,#$FF

                        rdlong  t2,par                'save received byte and inc head
                        add     t2,rxbuff
                        wrbyte  rxdata,t2
                        sub     t2,rxbuff
                        add     t2,#1
                        and     t2,#bufmsk
                        wrlong  t2,par

                        jmp     #receive              'byte done, receive next byte
'
'
' Transmit
'
transmit                jmpret  txcode,rxcode         'run a chunk of receive code, then return

                        mov     t1,par                'check for head <> tail
                        add     t1,#2 << 2
                        rdlong  t2,t1
                        add     t1,#1 << 2
                        rdlong  t3,t1
                        cmp     t2,t3           wz
        if_z            jmp     #transmit

                        add     t3,txbuff             'get byte and inc tail
                        rdbyte  txdata,t3
                        sub     t3,txbuff
                        add     t3,#1
                        and     t3,#bufmsk
                        wrlong  t3,t1

                        or      txdata,#$100          'ready byte to transmit
                        shl     txdata,#2
                        or      txdata,#1
                        mov     txbits,#11
                        mov     txcnt,cnt

:bit                    test    rxtxmode,#%100  wz    'output bit on tx pin according to mode
                        test    rxtxmode,#%010  wc
        if_z_and_c      xor     txdata,#1
                        shr     txdata,#1       wc
        if_z            muxc    outa,txmask        
        if_nz           muxnc   dira,txmask
                        add     txcnt,bitticks        'ready next cnt

:wait                   jmpret  txcode,rxcode         'run a chunk of receive code, then return

                        mov     t1,txcnt              'check if bit transmit period done
                        sub     t1,cnt
                        cmps    t1,#0           wc
        if_nc           jmp     #:wait

                        djnz    txbits,#:bit          'another bit to transmit?

                        jmp     #transmit             'byte done, transmit next byte
'
'
' Uninitialized data
'
t1                      res     1
t2                      res     1
t3                      res     1

rxtxmode                res     1
bitticks                res     1

rxmask                  res     1
rxbuff                  res     1
rxdata                  res     1
rxbits                  res     1
rxcnt                   res     1
rxcode                  res     1

txmask                  res     1
txbuff                  res     1
txdata                  res     1
txbits                  res     1
txcnt                   res     1
txcode                  res     1

{{

┌──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                                   TERMS OF USE: MIT License                                                  │                                                            
├──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation    │ 
│files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,    │
│modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software│
│is furnished to do so, subject to the following conditions:                                                                   │
│                                                                                                                              │
│The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.│
│                                                                                                                              │
│THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE          │
│WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR         │
│COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,   │
│ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                         │
└──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
}}