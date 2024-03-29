''***************************************
''*  Floating-Point Math                *
''*  Single-precision IEEE-754          *
''*  Author: Chip Gracey                *
''*  Copyright (c) 2006 Parallax, Inc.  *
''*  See end of file for terms of use.  *
''***************************************
''***************************************
''*  now with trig and exponential fcns *
''*  Author: Marty Lawson               *
''*  upgrade version: 1.1               *
''*  eliminated variables from trig fcns*
''*  See end of file for terms of use.  *
''***************************************
{{obj
  C : "cordic_obj"              'use to impliment all trig functions  '}

{{var  ''these variables are an alternate escape route for information during debugging.
  long  debug
  long  debug2

pub get_debug
'' used to export intermediate results of computations for debugging.
  result := debug

pub get_debug2
'' used to export intermediate results of computations for debugging.
  result := debug2    '}}

con
  nan_con_mask = %0111_1111_1000_0000__0000_0000_0000_0000   'exponent is $FF signaling that the result is something special.
  plus_inf = nan_con_mask
  minus_inf = %1111_1111_1000_0000__0000_0000_0000_0000
  nan_con = $7FFF_FFFF                                  'largest value of NaN
  lfsr_scale = 1.0/float(posx)
  
var
  long sprout
  long spud

pub seed(intA, intB)
''seed the pseudo-random number generator with two 32-bit integers.
  sprout := intA
  spud := intB


pub random
''returns a random, uniformly distrubuted float within the range of -1 to 1
  repeat (((spud <<13) >> 30)+1)                        'run the LFSR 1-4 times bassed on center bits of spud.
    ?sprout
  repeat (((sprout <<23) >> 30)+1)                      'run the LFSR 1-4 times bassed on center bits of sprout.  
    ?spud
  result := Fmul( Ffloat(spud), lfsr_scale)


pub isNaN(singleA) | m
''returns True if singleA is NaN (Not a Number)
''returns false if singleA is a number
  result := false               'set default answer
  if (singleA & nan_con_mask) == nan_con_mask           'if exponent is $FF
    m := singleA & $007F_FFFF      'unpack mantissa
    if m <> 0                   'and if mantissa is nonzero
      result := true            'singleA is a NaN

 
pub isInf(singleA) | s, m
'' returns 1 if singleA is +inf
'' returns 0 if singleA is finite or NaN
'' returns -1 if singleA is -inf
  result := 0               'set default answer
  if (singleA & nan_con_mask) == nan_con_mask           'if exponent is $FF
    m := singleA & $007F_FFFF      'unpack mantissa
    if m == 0                   'and if mantissa is zero
      s := singleA >> 31             'unpack sign
      if s                      'if the sign is negative                      
        result := -1
      else                      'if the sign is posative
        result := 1 


PUB Fcmp(singleA, singleB) | single
''exactly compare two floating point values
''returns an integer containing the results of the comparison
''result := 1 if singleA > singleB
''result := 0 if singleA == singleB
''result := -1 if singleA < singleB

  single := Fsub(singleA, singleB)                      'slow and simple way to compare floats.
  if single & $8000_0000        'if the sign bit is set
    result := -1                'result of subtraction is negative
  else
    result := 1                 'result of subtraction is posative
  if not(single << 1)           'mask off sign bit because subtraction result can be +-zero
    result := 0                 'inputs are the same.


con 'exp and log constants
  mantissa_one = 1<<29            '1.0 expressed in the same binary fractions as the mantissa
  log_base2_baseE = 0.693147180559945                   'result of  1.0/log2(e)
  log_base2_base10 = 0.301029995663981                  'result of 1.0/log2(10)
  exp_base2_baseE = 1.44269504088896                    'result of log2(e)
  exp_base2_base10 = 3.32192809488736                   'result of log2(10) 

pub exp(singleA)
''evaluate the base 'e' exponential
''uses base 2 exponential and change of base identity. ((x^a)^b) = x^(a*b) with x = 2 and x^a = new_base then a = log2(new_base)
  result := exp2( Fmul(singleA, exp_base2_baseE) )


pub exp10(singleA)
''evaluate the base '10' exponential
''uses base 2 exponential and change of base identity. ((x^a)^b) = x^(a*b) with x = 2 and x^a = new_base then a = log2(new_base)
  result := exp2( Fmul(singleA, exp_base2_base10) ) 


pub pow(singleA, singleB)
''evaluate the base 'singleB' exponential (i.e. singleA^singleB )
''uses base 2 exponential and change of base identity. ((x^a)^b) = x^(a*b) with x = 2 and x^a = new_base then a = log2(new_base)
  singleA := log2(singleA)
  result := exp2( Fmul(singleA, singleB) )


pub log(singleA)
''natural logarithim of SingleA
''uses change of base identity and Log2()
  result := Fmul(log2(singleA), log_base2_baseE)


pub log10(singleA)
''base 10 logarithim of SinglA
''uses change of base identity and Log2()
  result := Fmul(log2(singleA), log_base2_base10)
  

pub logB(singleA, singleB)
''logarithim with base SingleA of SingleB
''uses change of base identity and Log2()
  result := Fdiv( log2(singleB), log2(singleA) )
   

pub log2(singleA) | s, x, m, temp, work, idx', temp2
''evaluate the base 2 logarithim using an invariant,
''and successive approximation from a table of "nice" numbers.
''bassed on algorythm shown at http://www.quinapalus.com/efunc.html
''valid for all posative numbers greater than zero.
''screws up sub-normal numbers so keep inputs above ~1e-38

  unpack(@s, singleA)           'unpack the floating point input
  if s                          'if the input is negative
    return nan_con              'the result is not a number (well and imaginary number anyway)
  if singleA == 0               'trap error with zero input.
    return nan_con              'should output -inf instead
  
  m ~>= 1                       'divide mantissa by 2 so it ranges from 0.5 to .99999999999
  x += 1                        'adjust integer portion of result for the mantissa division

  work := 0                     'start with a fractional portion of zero
  repeat idx from 1 to 24'29
    temp := m + m ~> idx        'multiply 'm' by a "nice" number and temperarialy store the result
    if temp < mantissa_one           'if temp is less than a mantissa of 1
      m := temp                 'keep the updated value of 'm'
      work -= exp_lut[idx-1]    'adjust work to match

  'adjust for residual
  work += m - mantissa_one

  'add "x + work"
  'check if integer portion is negative
  if x < 0
    s := 1
  x := ||x                      'take the absolute value of the integer portion
  temp := 0 #> ((>|x) -1)              'what's the msb of "x"
  m := x << (29-temp)           'justify x to bit_29  
  work ~>= temp                 'allign fractional part with x
  if s                          'if 'x' was negative, subtract fractional part
    m -= work
  else                          'if 'x' was posative add fractional part
    m += work                   'add fractional portion to integer
  x := temp                     'update final exponent

  if m < 0                      'if mantissa is negative
    s := 1                      'change sign to negative
    ||m                         'absolute value the mantissa
    
  result := pack(@s)            'pack and return result.


pub exp2(singleA) | s, x, m, temp, work, idx
''evaluate the base 2 exponential (anti-log) using an invariant,
''and successive approximation from a table of "nice" numbers.
''bassed on algorythm shown at http://www.quinapalus.com/efunc.html  
  'unpack(@s, singleA)           'unpack the floating point input
  s := singleA >> 31             'unpack sign
  singleA := singleA & $7FFF_FFFF 'clear the sign bit.  aka abs()
  x := Ftrunc(singleA)          'get the integer portion of the input which is the exponent of the result
  if x => 127                   'if result would excede the size of a float
    return nan_con              'return NaN for now, should return +infinity?
  m := frac_int(singleA)        'get the fractional part of the input
  'start core calculation      
  work := mantissa_one          'start with a result mantissa of 1.0
  repeat idx from 1 to 25'29    '25 rounds is optimal for single precision numbers.
    temp := exp_lut[idx-1]      'access current table index
    if temp < m                 'if table entry is less than "m"
      m -= temp                 'subtract table entry from "m"
      work += work >> idx       'multiply work by the corresponding "nice" number
  'use small value approximation to correct residual error (only gives 1-2 bits extra so not worth the effort)
  {m := 1<<29 + m                'relative error is exp(m) which for small values is approximately 1+m.
  work := (work ** m) << 3      'multiply by error correction  }
  
  'move result into mantissa
  m := work
  'pack up result and deal with sign
  result := pack(@s)
  if s
    result := Fdiv(-1.0, result)


dat
exp_lut long      314049351     '<0> results of log2(1 + 2^-(array_idx+1)) expressed as a binary fraction over 2^29 
        long      172833830      
        long       91227790
        long       46956255
        long       23833911
        long       12008628     '<5>
        long        6027587
        long        3019657
        long        1511300
        long         756019
        long         378102     '<10>
        long         189074
        long          94543
        long          47273
        long          23637
        long          11818     '<15>
        long           5909
        long           2955
        long           1477
        long            739
        long            369     '<20>
        long            185
        long             92
        long             46
        long             23
        long             12     '<25>
        long              6
        long              3
        long              1     '<28>


con
  cordic_precision = 23         'number of bits to keep when converting to cordic angles.  Lower numbers of bits allow +-2^(31-cordic_precision) turns outside of the -pi to pi range.
  rad_to_subcor = float(1<<cordic_precision)/2.0/pi
  trig_vector_bits = 29         'number of bits long the CORDIC vector should be.  
  trig_vector_int = 1<<trig_vector_bits
  trig_vector_float = float(trig_vector_int)


pub sin(singleA) | a, x, y
''use cordic to calculate Sin(singleA) where singleA is the angle in radians
''only valid from -(2^(31-cordic_precision)-1)*2*pi to (2^(31-cordic_precision)-1)*2*pi {default of -1605 to 1605 radians}
'unless FRound is modified to allow truncating the MSB of restults larger than POSX or NEGX

  're-scale to cordic angles
  singleA := Fmul(singleA, rad_to_subcor)
  'convert to integer
  singleA := FRound(singleA)
  'shift to complete conversion to cordic angle
  singleA <<= 32-cordic_precision
  'feed to cordic (use a LONG vector to rotate)  
  'cor(singleA, trig_vector_int, 0, 0 )
  a := singleA
  x := trig_vector_int
  y := 0
  cordic(@a, 0)
  'convert back to a float
  singleA := FFloat( y{get_y})
  'scale output to -1.0 to 1.0 range
  result := Fmul(singleA, constant(1.0/trig_vector_float))


pub cos(singleA) | a, x, y
''use cordic to calculate cos(singleA) where singleA is the angle in radians
''only valid from -(2^(31-cordic_precision)-1)*2*pi to (2^(31-cordic_precision)-1)*2*pi {default of -1605 to 1605 radians}
'unless FRound is modified to allow truncating the MSB of restults larger than POSX or NEGX

  're-scale to cordic angles
  singleA := Fmul(singleA, rad_to_subcor)
  'convert to integer
  singleA := FRound(singleA)
  'shift to complete conversion to cordic angle
  singleA <<= 32-cordic_precision
  'feed to cordic (use a LONG vector to rotate)  
  'cor(singleA, trig_vector_int, 0, 0 )
  a := singleA
  x := trig_vector_int
  y := 0
  cordic(@a, 0)
  'convert back to a float
  singleA := FFloat(x {get_x})
  'scale output to -1.0 to 1.0 range
  result := Fmul(singleA, constant(1.0/trig_vector_float))'}


pub tan(singleA) | a, x, y
''use cordic to calculate tan(singleA) where singleA is the angle in radians.  (uses tan(x) = sin(x)/cos(x) identity)
''only valid from -(2^(31-cordic_precision)-1)*2*pi to (2^(31-cordic_precision)-1)*2*pi {default of -1605 to 1605 radians}
'unless FRound is modified to allow truncating the MSB of restults larger than POSX or NEGX

  're-scale to cordic angles
  singleA := Fmul(singleA, rad_to_subcor)
  'convert to integer
  singleA := FRound(singleA)
  'shift to complete conversion to cordic angle
  singleA <<= 32-cordic_precision
  'feed to cordic (use a LONG vector to rotate)  
  'cor(singleA, trig_vector_int, 0, 0 )
  a := singleA
  x := trig_vector_int
  y := 0
  cordic(@a, 0)
  'convert back to a float and calculate sin(x)/cos(x)
  result := Fdiv(FFloat(y {get_y}), FFloat(x {get_x}))


con
  cordic_to_rad = pi/float(1<<30) 're-scales cordic/2 angular units to radians.


pub atan2(singleA, singleB) | sa, xa, ma, sb, xb, mb, a, x, y 
''use cordic to calculate atan2(y,x)
''outputs an angle over the range of -pi to pi radians.
  're-scale inputs.  (same front end as addition)
  Unpack(@sa, singleB)          'unpack inputs
  Unpack(@sb, singleA)

  if sa                         'handle mantissa negation
    -ma
  if sb
    -mb

  result := ||(xa - xb) <# 31   'get exponent difference
  if xa > xb                    'shift lower-exponent mantissa down
    mb ~>= result
  else
    ma ~>= result
    xa := xb

  'feed to cordic code
  ma += 1                       'round instead of truncate
  mb += 1                       'round instead of truncate
  ma ~>= 1                      'avoid overflows in the CORDIC code
  mb ~>= 1
  
  'cor(0, ma, mb, 1)           'feed the inputs to the CORDIC code in Atan2 mode
  a := 0
  x := ma
  y := mb
  cordic(@a, 1)
  result := a 'get_a             'return the angle

  're-scale and output the angle
  result += 1                   'round instead of truncate
  result ~>= 1                  'keep the resulting angles in the valid range for FFloat
  result := Fmul(FFloat(result), cordic_to_rad)  'convert back to radians.


pub atan(singleA)
'' arctangent
'' atan( A ) = atan2( A , 1.0 )
  result := atan2(singleA, 1.0)


pub asin(SingleA) | singleB, temp
'' arcsine
'' asin( x ) = atan2( x, sqrt( 1 - x*x ) )
'' only valid with inputs between -1 and 1 inclusive.

  'calculate adjacent side of triangle
  singleB := Fsqr( Fsub( 1.0 , Fmul( singleA, singleA) ) )
  'check for valid range.
  if isNaN(singleB)
    return nan_con
  'calculate angle
  result := atan2( singleA, singleB)


pub acos(SingleA) | singleB, temp
'' arccosine
'' acos( x ) = atan2( sqrt( 1 - x*x ), x )
'' only valid with inputs between -1 and 1 inclusive.

  'calculate oposite side of triangle
  singleB := Fsqr( Fsub( 1.0 , Fmul( singleA, singleA) ) )
  'check for valid range.
  if isNaN(singleB)
    return nan_con
  'calculate angle
  result := atan2( singleB, singleA)    


{var
  long a, xc, y


pub Cor(_a, _x, _y, mode)
'calls the cordic algorithm with the given parameters
'returns the address of "a"
  a := _a
  xc := _x
  y := _y
  cordic(mode)
  result := @a


pub get_a
'gets the results of a cordic run
  result := a


pub get_x
'gets the results of a cordic run
  result := xc


pub get_y
'gets the results of a cordic run
  result := y '}
  

pub cordic(ptr, mode) | negate, i, da, dx, dy, a, x, y

'' CORDIC algorithm
''
'' if mode = 0: x,y are rotated by angle in a
'' if mode = 1: x,y are converted from cartesian to polar with angle->a, length->x
''
''   - angle range: $00000000-$FFFFFFFF = 0-359.9999999 degrees
''   - hypotenuse of x,y must be within ±1_300_000_000 to avoid overflow
''   - algorithm works best if x,y are kept large:
''       example: x=$40000000, y=0, a=angle, cordic(0) performs cos,sin into x,y

  'copy in data
  longmove(@a, ptr, 3)      'move calling a,x,y structure into local a,x,y structure


  if mode                      'if atan2 mode, reset a
    a~    
    negate := x < 0             'check quadrant 2 | 3 for either atan2 or rotate mode                        
  else
    negate := (a ^ a << 1) & $80000000
  
  if negate                     'if quadrant 2 | 3, (may be outside ±106° convergence range)                                                                 
    a += $80000000              '..add 180 degrees to angle
    -x                          '..negate x
    -y                          '..negate y

  repeat i from 0 to 26         'do CORDIC iterations (27 is optimal for 32-bit values)
    da := corlut[i]
    dx := y ~> i
    dy := x ~> i

    if mode
      negate := y < 0           'if atan2 mode, drive y towards 0
    else
      negate := a => 0          'if rotate mode, drive a towards 0

    if negate
      -da
      -dx
      -dy

    a += da
    x += dx
    y -= dy

  'remove CORDIC gain by multiplying by ~0.60725293500888        
  i := $4DBA76D4  
  x := (x ** i) << 1 + (x * i) >> 31                           
  y := (y ** i) << 1 + (y * i) >> 31

  'copy out data
  longmove(ptr, @a, 3)          'move local a,x,y structure into calling a,x,y structure 
  

dat
corlut  long $20000000          'CORDIC angle lookup table
        long $12E4051E
        long $09FB385B
        long $051111D4
        long $028B0D43
        long $0145D7E1
        long $00A2F61E
        long $00517C55
        long $0028BE53
        long $00145F2F
        long $000A2F98
        long $000517CC
        long $00028BE6
        long $000145F3
        long $0000A2FA
        long $0000517D
        long $000028BE
        long $0000145F
        long $00000A30
        long $00000518
        long $0000028C
        long $00000146
        long $000000A3
        long $00000051
        long $00000029
        long $00000014
        long $0000000A


PUB FFloat(integer) : single | s, x, m
''Convert integer to float
  if m := ||integer             'absolutize mantissa, if 0, result 0
    s := integer >> 31          'get sign
    x := >|m - 1                'get exponent
    m <<= 31 - x                'msb-justify mantissa
    m >>= 2                     'bit29-justify mantissa

    return Pack(@s)             'pack result
   

PUB FRound(single) : integer
''Convert float to rounded integer
  return FInteger(single, 1)    'use 1/2 to round


PUB FTrunc(single) : integer
''Convert float to truncated integer
  return FInteger(single, 0)    'use 0 to round


con
  Mask29 = $1FFF_FFFF


Pub frac(singleA) : single | s, x, m    
''extract the fractional portion of a floating point number.
''translated from F32 and Float32full.
{'------------------------------------------------------------------------------
' fraction
' fnumA = fractional part of fnumA
'------------------------------------------------------------------------------
_Frac                   call    #_Unpack                ' get fraction
                        test    expA, Bit31 wz          ' check for exp < 0 or NaN
          if_c_or_nz    jmp     #:exit
                        max     expA, #23               ' remove the integer
                        shl     manA, expA    
                        and     manA, Mask29
                        mov     expA, #0                ' return fraction

:exit                   call    #_Pack
                        andn    fnumA, Bit31            'clear sign
_Frac_ret               ret}
  unpack(@s, singleA)           'unpack the float
  if x < 0                      'if NaN do nothing, if exponent < 0 there is no whole part so input is already a fraction, return singleA
    return singleA'pack(@s)
  x := x <# 23                  'if exponent is larger than 23, we have no fractional significant figures
  m <<= x                       'shift mantissa left by exponent
  m &= Mask29                   'mask off extra bits
  x := 0                        'update exponent for fraction
  s := 0
  single := pack(@s)            'pack up and return result.


pri frac_int(singleA) : int | s, x, m    
''extract the fractional portion of a floating point number.
''returns an integer binary fraction for use in exp2() function.
  unpack(@s, singleA)           'unpack the float
  'if x < 0                      'if NaN do nothing, if exponent < 0 there is no whole part so input is already a fraction, return singleA
  '  return singleA'pack(@s)
  if x => 0                     'input is greater than 1.
    x := x <# 23                  'if exponent is larger than 23, we have no fractional significant figures
    m <<= x                       'shift mantissa left by exponent
    m &= Mask29                   'mask off extra bits
  else                          'input is less than one
    x := -29 #> x               'no bits left in binary fraction if x < -29
    m >>= -x                    'justify
  x := 0                        'update exponent for fraction
  s := 0
  int := m                      'return fraction of single A as a binary fraction.  int/(1<<29) = int/(2^29) = fractional portion of singleA


PUB FNeg(singleA) : single
''Negate singleA
  return singleA ^ $8000_0000   'toggle sign bit
  

PUB FAbs(singleA) : single
''Absolute singleA
  return singleA & $7FFF_FFFF   'clear sign bit


PUB FAbsNeg(singleA) : single
''ABS singleA then Negate singleA
  return singleA | $8000_0000   'set sign bit  


PUB FSqr(singleA) : single | s, x, m, root
''Compute square root of singleA
  if singleA > 0                'if a =< 0, result 0

    Unpack(@s, singleA)         'unpack input

    m >>= !x & 1                'if exponent even, shift mantissa down
    x ~>= 1                     'get root exponent

    root := $4000_0000          'compute square root of mantissa
    repeat 31
      result |= root
      if result ** result > m
        result ^= root
      root >>= 1
    m := result >> 1
  
    return Pack(@s)             'pack result
  if Fcmp(singleA, 0.0) < 0     'input is negative.  
    return nan_con


PUB FAdd(singleA, singleB) : single | sa, xa, ma, sb, xb, mb
''Add singleA and singleB
  Unpack(@sa, singleA)          'unpack inputs
  Unpack(@sb, singleB)

  if sa                         'handle mantissa negation
    -ma
  if sb
    -mb

  result := ||(xa - xb) <# 31   'get exponent difference
  if xa > xb                    'shift lower-exponent mantissa down
    mb ~>= result
  else
    ma ~>= result
    xa := xb

  ma += mb                      'add mantissas
  sa := ma < 0                  'get sign
  ||ma                          'absolutize result

  return Pack(@sa)              'pack result


PUB FSub(singleA, singleB) : single
''Subtract singleB from singleA
  return FAdd(singleA, FNeg(singleB))

             
PUB FMul(singleA, singleB) : single | sa, xa, ma, sb, xb, mb
''Multiply singleA by singleB
  Unpack(@sa, singleA)          'unpack inputs
  Unpack(@sb, singleB)

  sa ^= sb                      'xor signs
  xa += xb                      'add exponents
  ma := (ma ** mb) << 3         'multiply mantissas and justify

  return Pack(@sa)              'pack result


PUB FDiv(singleA, singleB) : single | sa, xa, ma, sb, xb, mb
''Divide singleA by singleB
  Unpack(@sa, singleA)          'unpack inputs
  Unpack(@sb, singleB)

  sa ^= sb                      'xor signs
  xa -= xb                      'subtract exponents

  repeat 30                     'divide mantissas
    result <<= 1
    if ma => mb
      ma -= mb
      result++        
    ma <<= 1
  ma := result

  return Pack(@sa)              'pack result


pub FMod(singleA, singleB) | tempA
''impliments [a - float(floor(a/b)) * b] calculation of Mod function
'this is likely pretty slow
  tempA := FDiv(singleA, singleB)
  tempA := Ffloat(Ftrunc(tempA))
  tempA := FNeg(FMul(tempA, singleB))
  result := FAdd(singleA, tempA)
  'correct the sign
  if Fcmp(singleA, 0.0) == -1
    result := FAbsNeg(result)
  else
    result := FAbs(result)
  

PRI FInteger(singleA, r) : integer | s, x, m

'Convert float to rounded/truncated integer

  Unpack(@s, singleA)                 'unpack input

  if x => -1 and x =< 30        'if exponent not -1..30, result 0
    m <<= 2                     'msb-justify mantissa
    m >>= 30 - x                'shift down to 1/2-lsb
    m += r                      'round (1) or truncate (0)
    m >>= 1                     'shift down to lsb
    if s                        'handle negation
      -m
    return m                    'return integer

      
Pri Unpack(pointer, single) | s, x, m

'Unpack floating-point into (sign, exponent, mantissa) at pointer

  s := single >> 31             'unpack sign
  x := single << 1 >> 24        'unpack exponent
  m := single & $007F_FFFF      'unpack mantissa

  if x                          'if exponent > 0,
    m := m << 6 | $2000_0000    '..bit29-justify mantissa with leading 1
  else
    result := >|m - 23          'else, determine first 1 in mantissa
    x := result                 '..adjust exponent
    m <<= 7 - result            '..bit29-justify mantissa

  x -= 127                      'unbias exponent

  longmove(pointer, @s, 3)      'write (s,x,m) structure from locals
  
  
pri Pack(pointer) : single | s, x, m

'Pack floating-point from (sign, exponent, mantissa) at pointer

  longmove(@s, pointer, 3)      'get (s,x,m) structure into locals

  if m                          'if mantissa 0, result 0
  
    result := 33 - >|m          'determine magnitude of mantissa
    m <<= result                'msb-justify mantissa without leading 1
    x += 3 - result             'adjust exponent

    m += $00000100              'round up mantissa by 1/2 lsb
    if not m & $FFFFFF00        'if rounding overflow,
      x++                       '..increment exponent
    
    x := x + 127 #> -23 <# 255  'bias and limit exponent

    if x < 1                    'if exponent < 1,
      m := $8000_0000 +  m >> 1 '..replace leading 1
      m >>= -x                  '..shift mantissa down by exponent
      x~                        '..exponent is now 0

    return s << 31 | x << 23 | m >> 9 'pack result


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