/*

Kakteen"uberwachung

Hardware: 
- Atmega328p 3Mhz (does not really matter)
- HD44780 compatible LCD display, 16x2

PINS:

- PORTD 8 data lines go to LCD
- PORTC 0,1,2 are used for LCD control lines


Software:


Note about the LCD: it really needs the 2ms delay, especially the clear screen
command. That did not work at all with anything faster. It would be better to
program that with checking the busy flag.

*/

.include "m328pdef.inc"

.equ FCPU = 3000000              ; 3 Mhz atmega328

/* Each instruction is 1/3 of a us long
This gives exact 2us long pulses
End:
                    NOP      ; 1 cycle 
                    NOP      ; 1 
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




; *** 8 BIT LCD INTERFACE -------------------------
.equ LCD_DATA     = PORTD
.equ LCD_DATA_DIR = DDRD
.equ LCD_CTL      = PORTC
.equ LCD_CTL_DIR  = DDRC
.equ LCD_RS       = 0
.equ LCD_RW       = 1
.equ LCD_E        = 2


; *** ONE WIRE -----------------
.equ ONEWIRE_PORT   = PORTB
.equ ONEWIRE_PORTIN = PINB
.equ ONEWIRE_DIR    = DDRB
.equ ONEWIRE_BUS    = 0     ; pin 0

.cseg ;------------  FLASH ----------------------------------
.org 0x0000
                    rjmp Reset
.org OVF0addr       rjmp Timer0Overflow




Timer0Overflow:     ; signal via r18
                    ldi r18, 0xFF
                    reti

; ------------------ DELAY SUBS ------------------------------
; *** we really need 2ms delays!
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

LongWait:           push r16
                    push r17
                    push r18
                    ldi r18, 0x20
LW1:                ldi r17, 0xFF
LW2:                ldi r16, 0xFF
LW3:                nop
                    dec r16
                    brne LW3
                    dec r17
                    brne LW2
                    dec r18
                    brne LW1
                    pop r18
                    pop r17
                    pop r16
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
                    ldi r16, 0x0f
                    rcall Lcd8BitCommand

                    ; now home
                    ldi r16, 0x02
                    rcall Lcd8BitCommand
                    ret


LcdString:          ; put out string in flash, pointed to by ZH/ZL 
                    lpm r16, Z+
                    cpi r16, 0
                    breq LcdStringExit
                    rcall Lcd8BitData
                    rjmp LcdString
LcdStringExit:      ret

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

u2a:                ; convert unsigned in r17 to 3 bytes in buffer
                    ldi XL, LOW(Buffer+3)
                    ldi XH, HIGH(Buffer+3)
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
u2aDone:            ret

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
Timer0DelayWait:    ;in r16, TIFR0
                    ;andi r16, 1<<TOV0
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
                    sbis ONEWIRE_PORTIN, ONEWIRE_BUS  ; if it stays high, there is no one
                    ldi r16, 0xFF
                    rcall Timer0Delay500us  ; wait 500 us
                    ret



; --------------------------- MAIN ------------------------------
Reset:
                    ;; set up return stack
                    ldi r16, LOW(RAMEND)
                    out SPL, r16
                    ldi r16, HIGH(RAMEND)
                    out SPH, r16

                    cli
                    rcall Timer0Init
                    sei

                    rcall OneWireInit

                    rcall LcdInit
                    rcall LcdClearScreen

                    ; debugging port
                    sbi DDRB, 1  ;; for debugging
                    cbi PORTB, 1

                    ; just some init message, not really needed
                    ; only for show
                    ldi ZH, HIGH(2*MsgLcd)
                    ldi ZL, LOW(2*MsgLcd)
                    rcall LcdString
                    rcall LongWait
                    rcall LcdClearScreen

/*
                    sbi PORTB, 1
                    rcall Timer0Delay100us
                    cbi PORTB, 1
                    rcall Timer0Delay100us
                    sbi PORTB, 1
                    rcall Timer0Delay100us
                    cbi PORTB, 1
                    rcall Timer0Delay100us
                    sbi PORTB, 1
                    rcall Timer0Delay100us
                    rjmp End

*/
                    
                    rcall OneWireReset
                    ; found -> r16 = 0xff
                    tst r16
                    breq  DS1820NotFound

                    ldi ZH, HIGH(2*DS1820OK)
                    ldi ZL, LOW(2*DS1820OK)
                    rcall LcdString
                    rjmp End

DS1820NotFound:     ldi ZH, HIGH(2*Error)
                    ldi ZL, LOW(2*Error)
                    rcall LcdString


                    
End:
                    rjmp End
 

MsgLcd:            .db "Starting....", 0, 0, 0
DS1820OK:          .db "Found DS1820", 0, 0 
Error:             .db "ERROR :-(", 0, 0 


.dseg ;------------  RAM  -------------------------------------

.org SRAM_START
Buffer: .byte 4

