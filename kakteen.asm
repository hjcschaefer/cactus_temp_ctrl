/*

Kakteen"uberwachung

Hardware:
- Atmega328p 3Mhz (does not really matter)
- HD44780 compatible LCD display, 16x2

PINS:

- PORTD 8 data lines go to LCD
- PORTC 0,1,2 are used for LCD control lines
- PORTC 3 used for alarm led, which is on if heater is on
- PORTB Pin 0 = One Wire Bus
- PORTB Pin 1 = Pin to switch heater on



Software:


Note about the LCD: it really needs the 2ms delay, especially the clear screen
command. That did not work at all with anything faster. It would be better to
program that with checking the busy flag.


Use of Register:
----------------

- r0-15 are global variables
- r16, r17, r18 pass/return function args, get used all the time

Global Vars:
------------
- r1:r0   current temperature
- r3:r2   minimal temperature
- r5:r4   maximal temperature
- r7:r6   lower threshold temperature
- r9:r8   upper threshold temperature
- r11:r10 heater on periods counter

*/

.include "m328pdef.inc"

.equ FCPU = 3000000              ; 3 Mhz atmega328

; **** The temperature bounds!! ****** * 16 because of the 0.0625 = 1/16 degree resolution
.equ LOWER_BOUND = 16*5;
.equ UPPER_BOUND = 16*7;

/* Each instruction is 1/3 of a us long
This gives exact 2us long pulses
End:
                    NOP      ; 1 cycle
                    NOP      ; 1  ; if the carry is set, we are below zero, also if we have identical zero
                    COM r16  ; 1
                    out PORTB, r16  ; 1
                    rjmp End  ; 2 -> overall 6 cycles, = 2us
*/

; prescalers
.equ PRESCALE_1024 = (1<<CS12) | (1<<CS10)
.equ PRESCALE_256 = (1<<CS12)
.equ PRESCALE_64 = (1<<CS11) | (1<<CS10)
.equ PRESCALE_8 = (1<<CS11)
.equ PRESCALE_1 = (1<<CS10)

; global vars
.def TEMPL = r0
.def TEMPH = r1
.def MINL  = r2
.def MINH  = r3
.def MAXL  = r4
.def MAXH  = r5
.def LOWL  = r6  ; lower bound
.def LOWH  = r7
.def HIGHL = r8  ; upper bound
.def HIGHH = r9
.def CNTL  = r10 ; heat periods counter
.def CNTH  = r11


; *** 8 BIT LCD INTERFACE -------------------------
.equ LCD_DATA     = PORTD
.equ LCD_DATA_DIR = DDRD
.equ LCD_CTL      = PORTC
.equ LCD_CTL_DIR  = DDRC
.equ LCD_RS       = 0
.equ LCD_RW       = 1
.equ LCD_E        = 2

; some Lcd command stuff
.equ DISPLAY_ON = 0b00000100
.equ CURSOR_ON  = 0b00000010
.equ BLINK_ON   = 0b00000001


; *** ONE WIRE -----------------
.equ ONEWIRE_PORT   = PORTB
.equ ONEWIRE_PORTIN = PINB
.equ ONEWIRE_DIR    = DDRB
.equ ONEWIRE_BUS    = 0     ; pin 0

; Control Pin for heater
.equ HEATER_PORT = PORTB
.equ HEATER_DIR  = DDRB
.equ HEATER_PIN  = 1
; Alarm pin for led
.equ ALARM_PORT = PORTC
.equ ALARM_DIR = DDRC
.equ ALARM_PIN = 3

.cseg ;------------  FLASH ----------------------------------
.org 0x0000
                    rjmp Reset
.org OVF0addr       rjmp Timer0Overflow



;; ------------- INTERRUPT HANDLERS ------------------------
Timer0Overflow:     ; signal via r18
                    ldi r18, 0xFF
                    reti

;; ------------ WATCHDOG  ----------------------------------
WatchdogOff:        ;switches watchdog off
                    push r16
                    cli
                    wdr
                    in r16, MCUSR  ; WDRF 
                    ; WDRF is set if watchdog occurs unclear why we need to clear it
                    ; but this code is copied from data sheet
                    andi r16, (0xff & (0<<WDRF))  ; set watchdog reset flag to zero
                    out MCUSR, r16
                    lds r16, WDTCSR
                    ori r16, (1<<WDCE) | (1<<WDE)  ; enter configuration mode
                    sts WDTCSR, r16
                    ldi r16, (0<<WDE)  ; clear WDE (and everything else also)
                    sts WDTCSR, r16
                    sei
                    pop r16
                    ret

Watchdog8sOn:       ; switch on watchdog with 8s interval
                    push r16
                    cli
                    wdr
                    lds r16, WDTCSR
                    ori r16, (1<<WDCE) | (1<<WDE)  ; enter configuration mode
                    sts WDTCSR, r16
                    ldi r16, (1<<WDE) | (1<<WDP3) | (1<<WDP0); 8 second interval
                    sts WDTCSR, r16
                    sei
                    pop r16
                    ret

; ------------------ DELAY SUBS ------------------------------
; *** we really need 2ms delays!
; TODO do this via Timer0 as well
Delay2ms:           push r16
                    ldi r16, 6
DelayLoop:          rcall Delay300ms
                    dec r16
                    brne DelayLoop
                    pop r16
                    ret

; --------  300 ms delay
Delay300ms:         push r17
                    ldi r17, 0xFF  ;
Delay300Loop:          nop
                    dec r17
                    brne Delay300Loop
                    pop r17
                    ret


; ------------------- 8 Bit LCD routines ----------------------------------------

Lcd8BitCommand:     ; write a 8 bit command given in r16
                    cbi LCD_CTL, LCD_RS ; R/S = 0 -> command
                    out LCD_DATA, r16
                    sbi LCD_CTL, LCD_E ; E = 1 -> now strobe it
                    rcall Delay2ms
                    cbi LCD_CTL, LCD_E ; E = 0 -> cmd ends
                    rcall Delay2ms
                    ret

Lcd8BitData:        ; write a 8 bit data given in r16
                    sbi LCD_CTL, LCD_RS ; R/S = 1 -> Data
                    out LCD_DATA, r16
                    sbi LCD_CTL, LCD_E ; E = 1 -> now strobe it
                    rcall Delay2ms
                    cbi LCD_CTL, LCD_E ; E = 0 -> cmd ends
                    rcall Delay2ms
                    ret


LcdInit:            ; 8 bit lcd init blinking cursor
                    ; init 8 bit mode
                    rcall Delay2ms

                    ldi r16, 0xFF
                    out LCD_DATA_DIR, r16
                    sbi LCD_CTL_DIR, LCD_RS
                    sbi LCD_CTL_DIR, LCD_RW
                    sbi LCD_CTL_DIR, LCD_E

                    ; control lines to zero
                    cbi LCD_CTL, LCD_RS
                    cbi LCD_CTL, LCD_RW
                    cbi LCD_CTL, LCD_E

                    ldi r16, 0x38
                    rcall Lcd8BitCommand

                    ; now cursor
                    ldi r16, 0x08 | DISPLAY_ON
                    rcall Lcd8BitCommand

                    ; now home
                    ldi r16, 0x02
                    rcall Lcd8BitCommand
                    ret


LcdFlashString:     ; put out string in flash, pointed to by ZH/ZL
                    lpm r16, Z+
                    cpi r16, 0
                    breq LcdFlashStringExit
                    rcall Lcd8BitData
                    rjmp LcdFlashString
LcdFlashStringExit: ret

LcdString:          ; put out string in memory, pointed to by ZH/ZL
                    ld r16, Z+
                    cpi r16, 0
                    breq LcdStringExit
                    rcall Lcd8BitData
                    rjmp LcdString
LcdStringExit: ret

LcdSetXY:           ; r16: bit 7 = 1-> line 1, bit 7 = 0 -> line 0
                    ; the rest is column
                    ; DDRAM for 16x2 -> line 0 has memory 0... line 1 has memory 0x40....
                    sbrc r16, 7  ; if it is  line 0 skip next instruction
                    ori r16, 0x40 ; add offset for line 1
                    ori r16, 0x80 ; always set command bit 7
                    rcall Lcd8BitCommand
                    ret

LcdClearScreen:     ; just clear screen
                    ldi r16, 0x01
                    rcall Lcd8BitCommand
                    ret

LcdHome:            ldi r16, 0x02 ; move cursor to home position
                    rcall Lcd8BitCommand
                    ret

LcdBCDOut:          ; r16 contains a BCD number, print it out as 2 digit decimal
                    push r16
                    swap r16
                    andi r16, 0x0F  ; zero out MSNibble
                    ori r16, 0x30   ; this adds '0'
                    rcall Lcd8BitData
                    pop r16
                    andi r16, 0x0F  ; zero out MSNibble
                    ori r16, 0x30   ; this adds '0'
                    rcall Lcd8BitData
                    ret

LcdBinaryOut:       push r17
                    push r16
                    ldi r17, 8
LcdBinaryOutNext:   pop r16
                    sbrs r16, 7
                    jmp LcdBinOutZero
                    lsl r16
                    push r16
                    ldi r16, '1'
                    rcall Lcd8BitData
                    rjmp LcdBinaryOutX
LcdBinOutZero:      lsl r16
                    push r16
                    ldi r16, '0'
                    rcall Lcd8BitData
LcdBinaryOutX:      dec r17
                    brne LcdBinaryOutNext
                    pop r16
                    pop r17
                    ret

; --------------- conversion int -> ASCII ----

Div8by8:            ; r17/r18 -> quotient in r17, remainder r16
                    clr r16
                    ldi r19, 9
Div8by8Loop:        rol r17
                    dec r19
                    breq Div8by8Exit
                    rol r16
                    sub r16, r18
                    brcc Div8by8X
                    add r16, r18
                    clc
                    rjmp Div8by8Loop
Div8by8X:           sec
                    rjmp Div8by8Loop
Div8by8Exit:        ret

u2a:                ; convert unsigned in r16 to 3 bytes in buffer
                    push r17
                    mov r17, r16
                    push r18
                    push r20
                    ldi XL, LOW(LcdBuffer+3)
                    ldi XH, HIGH(LcdBuffer+3)
                    ldi r18, 10
                    ldi r20, 3
u2aNext:            rcall Div8by8
                    ori r16, '0'
                    st -X, r16
                    dec r20
                    breq u2aDone
                    tst r17
                    brne u2aNext
                    ldi r16, ' '
u2aLeading:         st -X, r16
                    dec r20
                    brne u2aLeading
u2aDone:            pop r20
                    pop r18
                    pop r17
                    ret

; ------------  16 bit conversion --------------------------------

Unsigned16Bit2Ascii: ; convert r17:16 to Ascii and store in LcdBuffer
                    push ZL
                    push ZH
                    push XL
                    push XH
                    push r18
                    push r19
                    push r20
                    push r21
                    ldi ZL, LOW(Power10)
                    ldi ZH, HIGH(Power10)
                    ldi XL, LOW(LcdBuffer)
                    ldi XH, HIGH(LcdBuffer)

				    ldi r18, 5  ; 5 digits
U16ToAL2:
				    ld r19, Z+   ; power 10 in r20:r19
				    ld r20, Z+
				    ldi r21, 0xFF  ; our counter
U16ToAL1:           inc r21
                    sub r16, r19
			    	sbc r17, r20
	     			brcc U16ToAL1
	     			ori r21, '0'
		    		st X+, r21
		    		; add the factoor back in!
		    		add r16, r19
		    		adc r17, r20
				    dec r18
			    	brne  U16ToAL2
                    ; add zero termination for string
                    ldi r21, 0x00
                    st X+, r21

                    pop r21
                    pop r20
                    pop r19
                    pop r18
                    pop XH
                    pop XL
                    pop ZH
                    pop ZL
                    ret




; ============== DELAYS ==========================================
;; 2us delay
Delay_2us:          ; rcall to call us took 3 cycles
                    ; we need to spend 6 cycles at 3Mhz, so
                    ret   ;4 cycles -> this is 2.33333 us long!

; --------------  TIMER DELAYS ------------------------------
;
Timer0Init:         ; initialize timer0 and set it to zero
                    cli  ; avoid asll interrupts while setting
                    ldi r16, (1<<TOIE0)
                    sts TIMSK0, r16 ; enable overflow interrupt
                    ldi r16, 0x00
                    out TCCR0B, r16
                    sei
                    ret

; generic delay routine
; r16 : counter to start from
; r17 : prescaler  setting, i.e.
Timer0Delay:        push r18
                    ; stop the timer
                    eor r18, r18
                    out TCCR0A, r18
                    out TCCR0B, r18
                    out TCNT0, r16   ; the counter to start with
                    out TCCR0B, r17  ; set prescaler also starts the clock
Timer0DelayWait:    ; checking against TOV0 bit in TIFR0 did not work
                    ; now we use r18 as a flag, it gets set in the interrupt handler
                    tst r18
                    breq Timer0DelayWait
                    ; stop timer this is super important, also to set back TOV0
                    ; otherwise I got the wrong timing!
                    ldi r16, 1 << TOV0
                    out TIFR0, r16
                    eor r18, r18
                    out TCCR0B, r18
                    pop r18
                    ret

Timer0Delay500us:   push r16
                    push r17
                    ldi r16, 68  ; -> count 187 times
                    ldi r17, PRESCALE_8
                    rcall  Timer0Delay
                    pop r17
                    pop r16
                    ret

Timer0Delay100us:   push r16
                    push r17
                    ldi r16, 218  ; -> count 37 times
                    ldi r17, PRESCALE_8
                    rcall  Timer0Delay
                    pop r17
                    pop r16
                    ret

Timer0Delay44us:    push r16
                    push r17
                    ldi r16, 123  ; -> count 132 times
                    ldi r17, PRESCALE_1
                    rcall  Timer0Delay
                    pop r17
                    pop r16
                    ret

Timer0Delay15us:    push r16
                    push r17
                    ldi r16, 210  ; -> count 45 times
                    ldi r17, PRESCALE_1
                    rcall  Timer0Delay
                    pop r17
                    pop r16
                    ret

Timer0Delay100ms:   push r16
                    push r17
                    ldi r16, 38  ; -> count 217 times
                    ldi r17, PRESCALE_1024
                    rcall  Timer0Delay
                    pop r17
                    pop r16
                    ret

Timer0Delay1s:      push r18
                    ldi r18, 10
Timer0Delay1sLoop:  rcall Timer0Delay100ms
                    dec r18
                    brne Timer0Delay1sLoop
                    pop r18
                    ret


; ----------------- ONE WIRE ------------------------------
OneWireInit:        ; set bus to output and high
                    sbi ONEWIRE_DIR, ONEWIRE_BUS
                    sbi ONEWIRE_PORT, ONEWIRE_BUS
                    ret

OneWireReset:       ; reset bus and see if something is ther
                    ; r16 flags if we found something
                    ; r16 must be 0xFF if we found it
                    ldi r16, 0x00  ; if
                    sbi ONEWIRE_DIR, ONEWIRE_BUS ; to output
                    cbi ONEWIRE_PORT, ONEWIRE_BUS ; pull low
                    rcall Timer0Delay500us  ; wait 500 us
                    cbi ONEWIRE_DIR, ONEWIRE_BUS ; to input
                    sbi ONEWIRE_PORT, ONEWIRE_BUS ; pull up to 1
                    rcall Timer0Delay44us  ; wait
                    ; note that for reading we need to take the IN port!
                    sbis ONEWIRE_PORTIN, ONEWIRE_BUS  ; if it stays high, there is no one
                    ldi r16, 0xFF
                    rcall Timer0Delay500us  ; wait 500 us
                    ret

OneWirePutBit:      ; put a single bit over the one wire interface, we use
                    ; bit 0 of 16 -> one wire pushes least significant bit
                    ; first if we transfer a byte. Thus we can just shift
                    ; left the r16 value. We do not touch r16 otherwise
                    sbi ONEWIRE_DIR, ONEWIRE_BUS ; set to output
                    cbi ONEWIRE_PORT, ONEWIRE_BUS ; pull low
                    rcall Delay_2us
                    rcall Delay_2us

                    ;rcall Timer0Delay15us ; minimum delay
                    sbrs r16, 0 ;
                    rjmp OneWirePut0
                    ; we write a '1' so we pull up after the short 15us low
                    cbi ONEWIRE_DIR, ONEWIRE_BUS ; make input
                    sbi ONEWIRE_PORT, ONEWIRE_BUS ; pull up to 1
OneWirePut0:        ; just keep port low for longer
                    rcall Timer0Delay44us
                    cbi ONEWIRE_DIR, ONEWIRE_BUS ; make input
                    sbi ONEWIRE_PORT, ONEWIRE_BUS ; pull up to 1
                    ; delay 2us
                    rcall Delay_2us
                    ret

OneWireGetBit:     ; get a single bit over the one wire iterfacr
                   ; place it in bit 7 of r16 without touching any
                   ; of the other bits, so that we can shift it right
                   ; one wire sends least significant bit first, so
                   ; we can juse push right 8 times
                   ; clear bit 7 in r16
                   andi r16, 0x7F
                   sbi ONEWIRE_DIR, ONEWIRE_BUS ; set to output
                   cbi ONEWIRE_PORT, ONEWIRE_BUS; pull low
                   rcall Delay_2us ; short wait
                   cbi ONEWIRE_DIR, ONEWIRE_BUS ; set to input
                   sbi ONEWIRE_PORT, ONEWIRE_BUS ; let it pull to one
                   rcall Delay_2us   ; calibrated via osci....
                   rcall Delay_2us
                   rcall Delay_2us
                   rcall Delay_2us
                   rcall Delay_2us
                   sbic ONEWIRE_PORTIN, ONEWIRE_BUS ; skip if bit is cleared
                   ori r16, 0x80   ; set bit 7, as the bus is high
                   rcall Timer0Delay44us ret

OneWireWriteByte:  ; byte is in r16
                   push r17   ; need a counter
                   ldi r17, 8
OWWriteLoop:       rcall OneWirePutBit
                   lsr r16
                   dec r17
                   brne OWWriteLoop
                   pop r17
                   ret

OneWireReadByte:   ; returns byte in r16
                   push r17
                   ldi r17, 8
                   ldi r16, 0x00
OWReadLoop:        lsr r16
                   rcall OneWireGetBit
                   dec r17
                   brne OWReadLoop
                   pop r17
                   ret

; ------------- DS18 B 20 stufff --------------

DS1820Init:         ; --------- INIT DS18B20 : check it is there
                    rcall OneWireReset
                    ; found -> r16 = 0xff
                    tst r16
                    breq  DS1820NotFound
                    ldi ZH, HIGH(2*DS1820OK)
                    ldi ZL, LOW(2*DS1820OK)
                    rcall LcdFlashString
                    ret
DS1820NotFound:     ldi ZH, HIGH(2*Error)
                    ldi ZL, LOW(2*Error)
                    rcall LcdFlashString
                    rjmp End   ; finito




DS1820ReadRam:    ; read all the ram and store it in DS1820Ram
                  push ZL
                  push ZH
                  push r17
                  push r16
                  ldi ZH, HIGH(DS1820Ram)
                  ldi ZL, LOW(DS1820Ram)

                  rcall OneWireReset
                  ldi r16, 0xCC  ; skip identification
                  rcall OneWireWriteByte
                  ldi r16, 0x44  ; start measuring temperature
                  rcall OneWireWriteByte
                  ; wait 1 second!
                  rcall Timer0Delay1s

                  rcall OneWireReset
                  ldi r16, 0xCC  ; skip identification
                  rcall OneWireWriteByte
                  ldi r16, 0xBE  ; read ram
                  rcall OneWireWriteByte

                  ldi r17, 9;
DS1820ReadLoop:   rcall OneWireReadByte
                  st Z+, r16
                  dec r17
                  brne DS1820ReadLoop

                  pop r16
                  pop r17
                  pop ZH
                  pop ZL
                  ret



; ------------------- TEMPERATURE ROUTINES -----------------------

GetTemperature:     ; initiates temp conversion and stored temperature in
                    ; r1:r0 Note: takes about a full second
                    rcall DS1820ReadRam
                    lds TEMPL, DS1820Ram
                    lds TEMPH, DS1820Ram+1
                    ret

DisplayTemperature: ; displays full temperature given in r17:r16
                    ; at current location of Lcd
                    ; handle negative temp
                    push r19
                    ldi r19, 0x00               ; signals positive number
                    sbrs r17, 7
                    rjmp PositiveTemperature
                    ; ok, bit 7 was set -> we have negative temperature
                    ; take 2 complement and keep in mind that we had a negative number
                    com r17
                    neg r16                    ; sets carry
                    adc r17, r19               ; just add the carry, r19 is zero
                    ldi r19, 0xFF              ; signal negative number
PositiveTemperature:
                    ; at this point we have a positvie 16 bit number in r17:r16
                    ; and r19 flags with 0xFF if we have < 0 temp
                    push r16                   ; we are going to ignore the lower nibble,
                                               ; i.e fractional part for noe
                                               ; get rid of fractional part
                    lsr r16
                    lsr r16
                    lsr r16
                    lsr r16
                    ; shift MSB right, we do not need the most significant nibble
                    lsl r17
                    lsl r17
                    lsl r17
                    lsl r17
                    add r16, r17               ; argument for u2a is in r16
                    rcall u2a                  ; Now we have it in LcdBuffer
                                               ; X is where the result is
                    ldi XL, LOW(LcdBuffer)
                    ldi XH, HIGH(LcdBuffer)
                    ldi r16, '+'               ; we put a nice '+' in the front
                    st X, r16
                    tst r19
                    breq NoMinusSign
                    ldi r16, '-'               ; overwrite the + with -
                    st X, r16
NoMinusSign:        ld r16, X+
                    rcall Lcd8BitData
                    ld r16, X+
                    rcall Lcd8BitData
                    ld r16, X+
                    rcall Lcd8BitData

                    ; now the fractional part.
                    pop r16
                    andi r16, 0x0F            ; wipe out higher nibble
                    ; each fractional string is  8 bytes long so times 8
                    lsl r16
                    lsl r16
                    lsl r16
                    ldi ZL, LOW(2*Fractional)
                    ldi ZH, HIGH(2*Fractional)
                    add ZL, r16
                    eor r17, r17
                    adc ZH, r17
                    rcall LcdFlashString
                    pop r19
                    ret

; ------------  Heater Code -------------------------------------
HeaterInit:         push r16
                    eor r16, r16
                    mov CNTL, r16
                    mov CNTH, r16
                    sbi HEATER_DIR, HEATER_PIN
                    cbi HEATER_PORT, HEATER_PIN   ; low -> heater is off
                    sbi ALARM_DIR, ALARM_PIN
                    cbi ALARM_PORT, ALARM_PIN
                    ; also initialize the bounds
                    ldi r16, LOW(LOWER_BOUND)
                    mov LOWL, r16
                    ldi r16, HIGH(LOWER_BOUND)
                    mov LOWH, r16
                    ldi r16, LOW(UPPER_BOUND)
                    mov HIGHL, r16
                    ldi r16, HIGH(UPPER_BOUND)
                    mov HIGHH, r16
                    pop r16
                    ret

HeaterOn:           sbi HEATER_PORT, HEATER_PIN
                    sbi ALARM_PORT, ALARM_PIN
                    push r16                ; increase number of heater on periods
                    push r17
                    ldi r16, 0x01
                    ldi r17, 0x00
                    add CNTL, r16
                    adc CNTH, r17
                    pop r17
                    pop r16
                    ret

HeaterOff:          cbi HEATER_PORT, HEATER_PIN
                    cbi ALARM_PORT, ALARM_PIN
                    ret

; --------------------------- MAIN ------------------------------
Reset:
                    ;; set up return stack
                    ldi r16, LOW(RAMEND)
                    out SPL, r16
                    ldi r16, HIGH(RAMEND)
                    out SPH, r16

                    ; set up powers of 10 in ram, used in unsigned -> ascii conversion
                    ldi ZL, LOW(Power10)
                    ldi ZH, HIGH(Power10)
                    ldi r16, LOW(10000)
                    st Z+, r16
                    ldi r16, HIGH(10000)
                    st Z+, r16
                    ldi r16, LOW(1000)
                    st Z+, r16
                    ldi r16, HIGH(1000)
                    st Z+, r16
                    ldi r16, LOW(100)
                    st Z+, r16
                    ldi r16, HIGH(100)
                    st Z+, r16
                    ldi r16, LOW(10)
                    st Z+, r16
                    ldi r16, HIGH(10)
                    st Z+, r16
                    ldi r16, LOW(1)
                    st Z+, r16
                    ldi r16, HIGH(1)
                    st Z+, r16
                    rcall WatchdogOff
                    rcall Watchdog8sOn

                    cli
                    rcall Timer0Init
                    sei


                    rcall OneWireInit

                    rcall HeaterInit

                    rcall LcdInit
                    rcall LcdClearScreen

                    ; just some init message, not really needed
                    ; only for show
                    ldi ZH, HIGH(2*MsgLcd)
                    ldi ZL, LOW(2*MsgLcd)
                    rcall LcdFlashString
                    rcall Timer0Delay1s
                    rcall LcdClearScreen
                    rcall LcdHome

                    rcall DS1820Init
                    rcall Timer0Delay1s

                    rcall LcdClearScreen

                    ;; initialize min and max
                    rcall GetTemperature
                    mov MINL, TEMPL
                    mov MINH, TEMPH
                    mov MAXL, TEMPL
                    mov MAXH, TEMPH

MainLoop:
                    wdr
                    rcall LcdHome
                    rcall GetTemperature
                    wdr
                    ; set min and max
                    cp MINL, TEMPL
                    cpc MINH, TEMPH
                    brlt  NoNewMinimum
                    mov MINL, TEMPL
                    mov MINH, TEMPH

NoNewMinimum:
                    cp MAXL, TEMPL
                    cpc MAXH, TEMPH
                    brge NoNewMaximum
                    mov MAXL, TEMPL
                    mov MAXH, TEMPH

NoNewMaximum:
                    ; Heater Handling

                    sbic HEATER_PORT, HEATER_PIN ; is the heater already on?
                    rjmp HeaterIsOn

                    cp LOWL, TEMPL               ; do we need to switch the heater on?
                    cpc LOWH, TEMPH
                    brlt NoNeedForHeating
                    rcall HeaterOn
                    rjmp MainDisplay

HeaterIsOn:         ; the heater is on, can we switch it off?
                    cp HIGHL, TEMPL
                    cpc HIGHH, TEMPH
                    brge  MainDisplay
                    ; heater is but temperature is high enough
                    rcall HeaterOff


NoNeedForHeating:

MainDisplay:
                    wdr
                    ; *** Display Section
                    mov r16, TEMPL
                    mov r17, TEMPH
                    rcall DisplayTemperature

                    ldi r16, 0x80
                    rcall LcdSetXY
                    mov r16, MINL
                    mov r17, MINH
                    rcall DisplayTemperature

                    ldi r16, 0x88
                    rcall LcdSetXY
                    mov r16, MAXL
                    mov r17, MAXH
                    rcall DisplayTemperature

                    ldi r16, 0xA
                    rcall LcdSetXY
                    mov r16, CNTL
                    mov r17, CNTH
                    rcall Unsigned16Bit2Ascii
                    ldi ZL, LOW(LcdBuffer)
                    ldi ZH, HIGH(LcdBuffer)
                    rcall LcdString
                    rjmp MainLoop
End:
                    rjmp End


;                      "0123456789012345"  ; maximal row length indicator
MsgLcd:            .db "Starting....", 0, 0, 0
DS1820OK:          .db "Found DS1820", 0, 0
Error:             .db "DS1820 Dead?!", 0, 0
Fractional:        .db ".0000", 0, 0, 0  ; padding to 8 for easier addressing
                   .db ".0625", 0, 0, 0
                   .db ".1250", 0, 0, 0
                   .db ".1875", 0, 0, 0
                   .db ".2500", 0, 0, 0
                   .db ".3125", 0, 0, 0
                   .db ".3750", 0, 0, 0
                   .db ".4375", 0, 0, 0
                   .db ".5000", 0, 0, 0
                   .db ".5625", 0, 0, 0
                   .db ".6250", 0, 0, 0
                   .db ".6875", 0, 0, 0
                   .db ".7500", 0, 0, 0
                   .db ".8125", 0, 0, 0
                   .db ".8750", 0, 0, 0
                   .db ".9375", 0, 0, 0
                   .db 0, 0, 0

.dseg ;------------  RAM  -------------------------------------

.org SRAM_START
DS1820Ram: .byte 10
LcdBuffer: .byte 10  ; used for LCD output
Power10: .byte 10


