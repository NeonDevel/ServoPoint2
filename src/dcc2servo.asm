;;======================================================================;;
;;			DCC 2 SERVO DECODER				;;
;;======================================================================;;
;;									;;
;; Program:         DCC2SERVO_RELE -- DCC accesory decoder for 2 servo	;;
;; Code:            Paco Cañada						;;
;; Platform:        Microchip PIC12F629, PIC12F675 ; 4 Mhz				;;
;; Modifications for PIC12F675 and more by Arkadiusz Hahn 
;; Date:            08.06.2005						;;
;; First release:   13.06.2005						;; 
;; Last release by Paco  16.09.2005
;; LastDate:        23.05.2017	  				;;



;;									;;
;;======================================================================;;

; Minimal external components, uses internal oscilator at 4 MHz


; Decodes only 3 byte packets
;
; 1111111111 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1

; This program is distributed as is but WITHOUT ANY WARRANTY
; I hope you enjoy!!
;
; Revisions:
; 08.06.2005	Start of writing code
; 11.06.2005	all servos work and page programming
; 13.06.2005	changed range for Hitec servos (0,5ms=0º, 1,5ms=90º, 2,5ms=180º)
; 16.09.2005	interrupt controlled pulses, saving output state, changes in CV number, direct programming
; 15.08.2011	changed for 2 servo and 2 relays changing at center position
; 20.03.2017  modification for PIC12F675 
; 23.05.2017  Long pressing SWICH for about 2.5s (depends on CV545) enters programming mode, both servos 
;               move to central positions, outputs set on; 
;             Direct CV programming and address change is only possibile in programing mode.
;             Capturing new address or short pressing SWITCH exits programming mode.
 

; ----- Definitions

#define		__VERNUM	  D'2'
#define		__VERDAY	  0x23
#define		__VERMONTH	0x05
#define		__VERYEAR	  0x17

;This project can be compiled either for PIC12F629 or PIC12F675 processor
  LIST	   p=12F675	; default target processor
; processor 12F675 ; alternative directive for LIST p=
; LIST	   p=12F629	; alternative target CPU
;NOTE:  Processor type is superseded by command line switch: C:\MPASM MYFILE.ASM /PIC12F629  

#IFDEF __12F675
  #include p12F675.inc
#endif

#IFDEF __12F629
  #include p12F629.inc
#endif

	errorlevel -305,-302
	

	__CONFIG  _BODEN_ON & _CP_OFF & _WDT_OFF & _MCLRE_OFF & _PWRTE_OFF & _INTRC_OSC_NOCLKOUT 

				; Make sure that internal osc. is calibrated
				; Value has to be read before reprogramming the device.
        ; Preferred programmer for WIN10 - Michrochip PICkit2

; --- Constant values

FXTAL		equ	D'4000000'		; internal oscilator

GP_TRIS         equ     0x0C			; GP2,GP3: inputs
GP_INI          equ     0x00			; all zero
OPTION_INI	    equ	    0x88			; Option register: no pull-up, falling GP2, no prescaler, wdt 1:1
WPU_INI		      equ	    0x33			; Weak pull-up enable. default, no pull-ups

INTC_INI	equ	0xD0			; GIE, INTE enable, PEIE enable
PIE1_INI	equ	0x01			; interrupt TMR1


#define		SERVO1	GPIO,5			; Servo 1 output
#define		SERVO2	GPIO,4			; Servo 2 output
#define		SWITCH	GPIO,3			; Programm switch (GPIO,3 only input)
#define		DCCIN	GPIO,2			; DCC input pin
#define		RELE2	1			; Servo 3 output
#define		RELE1	0			; Servo 4 output

REACH_PULS	equ	0x04			 ; aditional pulses when reached position
SWITCH_PRESS_LONG equ 0x7d  ; 20ms*125 =  2.5s -  Time to enter programming mode
SWITCH_PRESS_SHORT equ 0x0A ; 20ms * 10 = 0.2s  - time to exit programming mode

; --- EEPROM Section

#define		EE_INI		0x00

E_CV513		equ	EE_INI+0x00		; CV513	Primary Adress low
E_CV515		equ	EE_INI+0x02		; CV515	Range Servo 1
E_CV516		equ	EE_INI+0x03		; CV516	Range Servo 2
;E_CV517	equ	EE_INI+0x04		; CV517	Range Servo 3
;E_CV518	equ	EE_INI+0x05		; CV518	Range Servo 4
E_CV7		equ	EE_INI+0x06		; Manufacturer Version
E_CV8		equ	EE_INI+0x07		; Manufacturer ID
E_CV521		equ	EE_INI+0x08		; CV521	Primary Adress high
E_CV541		equ	EE_INI+0x1C		; config
E_CV545		equ	EE_INI+0x20		; CV545 Spacing
E_CV546		equ	EE_INI+0x21		; CV546	Accesory flags
E_CV547		equ	EE_INI+0x22		; CV547 Speed Servo 1
E_CV548		equ	EE_INI+0x23		; CV548 Speed Servo 2
;E_CV549	equ	EE_INI+0x24		; CV549 Speed Servo 3
;E_CV550	equ	EE_INI+0x25		; CV550 Speed Servo 4

EE_OUT		equ	EE_INI+0x7F		; saved outputs

; ----- Variables

; --- Internal RAM Section

#define		RAMINI0		0x020		; 64 bytes of RAM

INT_W		equ	RAMINI0+0x00		; interrupt context registers
INT_STAT	equ	RAMINI0+0x01

SHIFT0		equ	RAMINI0+0x02
SHIFT1		equ	RAMINI0+0x03		; interrupt shift register
SHIFT2		equ	RAMINI0+0x04
SHIFT3		equ	RAMINI0+0x05
SHIFT4		equ	RAMINI0+0x06
SHIFT5		equ	RAMINI0+0x07
DATA00		equ	RAMINI0+0x08
DATA0		equ	RAMINI0+0x09		; received packet
DATA1		equ	RAMINI0+0x0A
DATA2		equ	RAMINI0+0x0B
DATA3		equ	RAMINI0+0x0C

PAGEREG		equ	RAMINI0+0x0D		; Page register

CV513		equ	RAMINI0+0x10		; Primary Adress low byte
CV521		equ	RAMINI0+0x11		; Primary Adress high byte

CV515		equ	RAMINI0+0x12		; Range servo 1
CV516		equ	RAMINI0+0x13		; Range servo 2
;CV517		equ	RAMINI0+0x14		; Range servo 3
;CV518		equ	RAMINI0+0x15		; Range servo 4

CV545		equ	RAMINI0+0x16		; Spacing
CV546		equ	RAMINI0+0x17		; accessory flags

CV547		equ	RAMINI0+0x18		; Speed servo 1
CV548		equ	RAMINI0+0x19		; Speed servo 2
;CV549		equ	RAMINI0+0x1A		; Speed servo 3
;CV550		equ	RAMINI0+0x1B		; Speed servo 4

CV547CNT	equ	RAMINI0+0x1C		; Speed servo 1
CV548CNT	equ	RAMINI0+0x1D		; Speed servo 2
;CV549CNT	equ	RAMINI0+0x1E		; Speed servo 3
;CV550CNT	equ	RAMINI0+0x1F		; Speed servo 4

FLAGS		equ	RAMINI0+0x20
EEDATA0		equ	RAMINI0+0x21		; data to write in EEPROM
MOVING		equ	RAMINI0+0x22		; moving flags
POSITION	equ	RAMINI0+0x23		; current position flags
STATE		  equ	RAMINI0+0x24		; current state
REACHED		equ	RAMINI0+0x25		; position reached


TEMP		  equ	RAMINI0+0x28
COUNT		  equ	RAMINI0+0x29

PULSE1		equ	RAMINI0+0x30		; pulse duration
PULSE2		equ	RAMINI0+0x31		; pulse duration
;PULSE3		equ	RAMINI0+0x32		; pulse duration
;PULSE4		equ	RAMINI0+0x33		; pulse duration
SPACE_H		equ	RAMINI0+0x34		; spacing duration
SPACE_L		equ	RAMINI0+0x35
MODE      equ RAMINI0+0x36    ;work mode (normal operation | programming)
DEBOUNCE	equ	RAMINI0+0x27		;programming key debouncing
OUTPUT		equ	RAMINI0+0x3F

; --- Flags

#define		NEW_PACKET	FLAGS,0		; New 3 byte packet received
#define		NOCV		FLAGS,1		; No CV finded
#define		RDONLY		FLAGS,2		; CV read only
#define		DCC4BYTE	FLAGS,3		; DCC command 4 bytes
#define		DO_PULSE	FLAGS,5		; do pulse, TMR1 end
#define		PROG_2X		FLAGS,6		; 2x prog
#define		RESET_FLG	FLAGS,7		; reset packet

#define		SAVE_OUTPUTS	CV546,0		; Save outputs to EEPROM
#define		PAIR_SEL	CV546,2		; Pair selection

#define PROGRAM_MODE       MODE,0   ;programming mode - CV and address programming enabled


; --------------- Program Section --------------------------------------


		org	0x000

PowerUp:
		clrf	STATUS			; Bank 0 default
		clrf	INTCON			; Disable all interrupts
		clrf	PCLATH			; tables on page 0
		goto	INIT

; ----------------------------------------------------------------------

		org	0x004

Interrupt:
		movwf	INT_W			; save context registers
		swapf	STATUS,w
		movwf	INT_STAT
		clrf	STATUS			; interrupt uses bank 0

		btfss	PIR1,TMR1IF		; end of servo pulse?
		goto	Int_DCC
		movf	OUTPUT,w		; yes, clear ports
		movwf	GPIO
		bcf	PIR1,TMR1IF
		bsf	DO_PULSE

Int_DCC:
		btfss	INTCON,INTF		; GPIO,2 interrupt?
		goto	EndInt

		btfsc	DCCIN
		goto	Int_High_Half
Int_Low_Half:
		movlw	d'256' - d'80'		; 80us: between 64us (one) and 90us (zero)
		movwf	TMR0
		bcf	INTCON,T0IF		; clear overflow flag for counting

		bcf	INTCON,INTF		
		bsf	STATUS,RP0
		bsf	OPTION_REG,INTEDG	; next interrupt on rising edge GP2
		goto	EndInt

; 1111111111 CCCCCCCC 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1
; 'x1111111''111 0 CCCC''CCCC 0 AAA' 'AAAAA 0 DD' DDDDDD 0 E' 'EEEEEEE 1'
;   SHIFT0     SHIFT1      SHIFT2       SHIFT3      SHIFT4      SHIFT5

; 1111111111 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1
; 'xxxxxxxx''xx111111''1111 0 AAA' 'AAAAA 0 DD' DDDDDD 0 E' 'EEEEEEE 1'
;   SHIFT0    SHIFT1     SHIFT2       SHIFT3      SHIFT4      SHIFT5


Int_High_Half:
		bsf	STATUS,C							;8
		btfsc	INTCON,T0IF		; if timer 0 overflows then is a DCC zero;9
		bcf	STATUS,C							;10
		rlf	SHIFT5,f		; receiver shift register		;11
		rlf	SHIFT4,f							;12
		rlf	SHIFT3,f							;13
		rlf	SHIFT2,f							;14
		rlf	SHIFT1,f							;15
		rlf	SHIFT0,f							;16

		movlw	0xC0			; ignore older bits			;17
		iorwf	SHIFT1,w		; 					;18
		xorlw	0xFF			;'xx111111' ? first 6 preamble bits	;19
		btfss	STATUS,Z							;20
		goto	Int_High_Check4							;21,22

Int_High_Check3:
		movlw	0xF8			; check 4 preamble and start bit	;23
		andwf	SHIFT2,w		; '1111 0 xxx'				;24
		xorlw	0xF0								;25
		btfss	STATUS,Z		; 					;26
		goto	EndHighHalf							;27,28

		btfss	SHIFT5,0		; check end bit				;29
		goto	EndHighHalf		; is zero, may be 4 byte packet		;30,31

		bsf	NEW_PACKET		; fill received packet			;31
		movf	SHIFT5,w							;32
		movwf	DATA3								;33
		movf	SHIFT4,w							;34
		movwf	DATA2								;35
		movf	SHIFT3,w							;36
		movwf	DATA1								;37
		movf	SHIFT2,w							;38
		movwf	DATA0								;39
		clrf	SHIFT1			; prevent 4 byte command decoding	;40

EndHighHalf:
		bcf	INTCON,INTF						;46	;31
		bsf	STATUS,RP0						;47	;32
		bcf	OPTION_REG,INTEDG	; next int. on falling edge GP2	;48	;33
EndInt:
		swapf	INT_STAT,w		; restore context registers	;49	;34
		movwf	STATUS							;50	;35
		swapf	INT_W,f							;51	;36
		swapf	INT_W,w							;52	;37
		retfie								;53,54	;38,39

Int_High_Check4:
		bsf	SHIFT0,7		; command 4 bytes, check 7 preamble bits;23
		incfsz	SHIFT0,w		;'x1111111'				;24
		goto	EndHighHalf							;25,26

		movlw	0xF0			; check 4 preamble and start bit	;26
		andwf	SHIFT1,w		; '1111 0 xxx'				;27
		xorlw	0xE0								;28
		btfss	STATUS,Z		; 					;29
		goto	EndHighHalf							;30,31

		bsf	NEW_PACKET		; fill received packet			;32
		bsf	DCC4BYTE		; command 4 bytes			;33
		movf	SHIFT5,w							;34
		movwf	DATA3								;35
		movf	SHIFT4,w							;36
		movwf	DATA2								;37
		movf	SHIFT3,w							;38
		movwf	DATA1								;39
		movf	SHIFT2,w							;40
		movwf	DATA0								;41
		movf	SHIFT1,w							;42
		movwf	DATA00								;43
		goto	EndHighHalf							;44,45

; ----------------------------------------------------------------------

BitPos:
		addwf	PCL,f

		retlw	0x01
		retlw	0x02
		retlw	0x04
		retlw	0x08
		retlw	0x10
		retlw	0x20
		retlw	0x40
		retlw	0x80

; ----------------------------------------------------------------------

INIT:
		clrf	GPIO
		movlw	0x07
		movwf	CMCON			; set GP2:0 to digital I/O

		bsf	STATUS,RP0		; bank 1
#IFDEF __12F675 
    clrf ANSEL   ; for PIC12F675 pins need to be configured as digital inputs
#endif    
		movlw	GP_TRIS
		movwf	TRISIO
		call	0x3FF			; get OSCCAL value
		movwf	OSCCAL
		movlw	WPU_INI			; pull-ups
		movwf	WPU
		clrf	IOC			; interrupt on change
		clrf	VRCON			; voltage reference off
		movlw	OPTION_INI		; Option register: no pull-up, falling GP2, no prescaler, wdt 1:1
		movwf	OPTION_REG
		movlw	PIE1_INI
		movwf	PIE1
		bcf	STATUS,RP0		; bank 0
		clrf	PIR1
		movlw	0x01			; Timer 1 on, 1:1
		movwf	T1CON

		movlw	0x20			; clear RAM
		movwf	FSR
ClearRAM:
		clrf	INDF
		incf	FSR,f
		movlw	0x60
		xorwf	FSR,w
		btfss	STATUS,Z
		goto	ClearRAM
    movlw	SWITCH_PRESS_LONG ; init SWITCH press time
		movwf	DEBOUNCE
    bcf PROGRAM_MODE  ; normal operation mode
		movlw	INTC_INI
		movwf	INTCON			; enable GP2 external interrupt

		movlw	d'150'			; init servos default
		movwf	PULSE1
		movwf	PULSE2
;		movwf	PULSE3
;		movwf	PULSE4
		clrf	MOVING
		clrf	POSITION
		clrf	STATE

		clrf	PAGEREG			; page register default

		call	LoadCV			; load CV values

		btfsc	SAVE_OUTPUTS		; load saved outputs
		call	LoadOutputs

; ----------------------------------------------------------------------

MainLoop:
		btfsc	NEW_PACKET		; new packet?
		call	Decode			; yes, decode

		btfsc	DO_PULSE		; end of servo pulse?
		call	DoServo			; yes, next pulse
    clrwdt
		goto	MainLoop

; ----------------------------------------------------------------------

Accessory:
		bcf	RESET_FLG

		;btfss	SWITCH			; pressed switch?    ;; ToDO: remove this line
    ;goto	AccessProg		; yes, program new address ;;ToDo: remove this line

    btfsc PROGRAM_MODE ; new programming mode 
		goto	AccessProg		; yes, program new address

		movf	CV521,w			; check high address bits
		iorlw	0x80			;'1AAAxxxx'
		btfsc	PAIR_SEL		; 
		iorlw	0x04
		xorwf	DATA2,w
		andlw	0xF4
		btfss	STATUS,Z
		goto	ExitDecode
		movf	DATA1,w			; check low address bits
		andlw	0x3F
		xorwf	CV513,w
		btfss	STATUS,Z
		goto	ExitDecode

AccessOut:

		movf	DATA2,w			; activate outputs
		andlw	0x0B			; 'xxxxCDDD'
		addwf	PCL,f

		goto	ExitDecode
		goto	ExitDecode
		goto	ExitDecode
		goto	ExitDecode
		goto	ExitDecode
		goto	ExitDecode
		goto	ExitDecode
		goto	ExitDecode
		goto	F1A_Set
		goto	F1B_Set
		goto	F2A_Set
		goto	F2B_Set
		goto	ExitDecode
		goto	ExitDecode
		goto	ExitDecode
		goto	ExitDecode

AccessProg:
		btfss	DATA2,7			; '10AAAAAA'1AAAxxxx'? accesory operation
		goto	ExitDecode
		clrf	GPIO
		movf	DATA1,w			; get 6 low bits
		andlw	0x3F
		movwf	CV513
		movwf	EEDATA0
		movlw	E_CV513
		call	SetParm

		swapf	DATA2,w			; get 3 high bits
		andlw	0x07
		movwf	CV521
		swapf	CV521,f
		xorlw	0x07			; complement bits
		movwf	EEDATA0
		movlw	E_CV521
		call	SetParm


		btfss	DATA2,2			; save pair
		bcf	PAIR_SEL
		btfsc	DATA2,2
		bsf	PAIR_SEL
		movf	CV546,w
		movwf	EEDATA0
		movlw	E_CV546
		call	SetParm
    call EndProgMode
		goto	AccessOut

; ----------------------------------------------------------------------
DoServo:
		bcf	DO_PULSE
;		clrf	GPIO			; clear all outputs
		btfsc	INTCON,INTE		; disabled interrupts?
		goto	DoServoJump
		clrf	SHIFT5			; yes, clear for decoding
		clrf	SHIFT4
		clrf	SHIFT3
		clrf	SHIFT2
		clrf	SHIFT1
		bsf	INTCON,INTE		; re-enable interrupts
DoServoJump:
		movf	STATE,w
		addwf	PCL,f
    goto	PulseServo1
		goto	PulseServo2
		goto	Spacing
		clrf	STATE			; prevents erroneus contents
		clrf	STATE
		clrf	STATE
		clrf	STATE
		clrf	STATE


PulseServo1:
		movf	CV545,w			; init spacing value
		sublw	0x00
		movwf	SPACE_H
		clrf	SPACE_L

		movf	PULSE1,w		; 200: 2ms, 100: 1ms
		call	PulseServo

		btfsc	MOVING,0
		bcf	INTCON,INTE		; disable DCC interrupts for time accuracy

		btfsc	MOVING,0
		bsf	SERVO1

		bsf	T1CON,TMR1ON		; run timer 1
		incf	STATE,f
		goto	EndSpacing
PulseServo2:
		movf	PULSE2,w		; 200: 2ms, 100: 1ms
		call	PulseServo

		btfsc	MOVING,1
		bcf	INTCON,INTE		; disable DCC interrupts for time accuracy

		btfsc	MOVING,1
		bsf	SERVO2

		bsf	T1CON,TMR1ON		; run timer 1
		incf	STATE,f
		goto	EndServo1


CheckProgKey:    
    btfsc	SWITCH			; check program switch  
    goto SwitchIsOff
    decfsz	DEBOUNCE,f		; wait debounce time (2,5s) = 20ms * 125 
		return    ; time not elapsed 
    ; mode changed
    btfss PROGRAM_MODE
    goto EnterProgMode
EndProgMode:
    bcf PROGRAM_MODE;
    ; move servos to saved positions
    call LoadOutputs
    return 
SwitchIsOff:  ; init debounce time
    movlw	SWITCH_PRESS_LONG  
    btfsc PROGRAM_MODE
    movlw	SWITCH_PRESS_SHORT
    movwf	DEBOUNCE
    return
EnterProgMode:
    bsf PROGRAM_MODE;
    ;; move all servos to central position
    movlw d'150'
    movwf	PULSE1
    movwf	PULSE2
    movlw	0x33			; do move to reached position
		movwf	REACHED
		movwf	MOVING
    return
    
    
Spacing:
		bcf	T1CON,TMR1ON
		movf	SPACE_H,w
		movwf	TMR1H
		movf	SPACE_L,w
		movwf	TMR1L
;		bcf	PIR1,TMR1IF
		bsf	T1CON,TMR1ON		; run timer 1
		clrf	STATE
    
		movf	PULSE1,w		; check position and change relays
		sublw	d'150'  ;
    btfsc PROGRAM_MODE
    bcf STATUS,C    ; set outputs in program mode
    btfss	STATUS,C
		bsf	OUTPUT,RELE1
		btfsc	STATUS,C
		bcf	OUTPUT,RELE1

		movf	PULSE2,w
		sublw	d'150'
    btfsc PROGRAM_MODE
    bcf STATUS,C    ; set outputs in program mode
		btfss	STATUS,C
		bsf	OUTPUT,RELE2
		btfsc	STATUS,C
		bcf	OUTPUT,RELE2
		call	EndServo2		; *** 15.08.11
    goto CheckProgKey     ; ** 20.05.2017

PulseServo:
		bcf	T1CON,TMR1ON		; stop timer 1
		clrf	TMR1H			; do x10
		movwf	TEMP
		movwf	TMR1L
		rlf	TMR1L,f
		rlf	TMR1H,f
		rlf	TMR1L,f			; x4
		rlf	TMR1H,f
		rlf	TMR1L,f			; x8
		rlf	TMR1H,f
		movlw	0xF8
		andwf	TMR1L,f
		movf	TEMP,w			; x8 + x1 = x9
		addwf	TMR1L,f
		btfsc	STATUS,C
		incf	TMR1H,f
		addwf	TMR1L,f			; x9 + x1 = x10
		btfsc	STATUS,C
		incf	TMR1H,f

		movf	TMR1L,w			; correct spacing
		addwf	SPACE_L,f
		btfsc	STATUS,C
		incf	SPACE_H,f
		movf	TMR1H,w
		addwf	SPACE_H,f

		comf	TMR1L,f			; negative for incrementing
		comf	TMR1H,f
		movlw	0x01
		addwf	TMR1L,f
		btfsc	STATUS,C
		incf	TMR1H,f
;		bcf	PIR1,TMR1IF
		return


;----------------------------------------------------------------------

EndSpacing:
    
		return

EndServo1:
		btfss	MOVING,0		; moving servo?
		return				; no

		btfsc	POSITION,0		; yes
		goto	EndServo1Nxt
		
		btfsc	REACHED,0
		goto	EndServo1WW1

		decfsz	CV547CNT,f		; speed
		goto	EndServo1W1
		movf	CV547,w
		movwf	CV547CNT
		decf	PULSE1,f
EndServo1W1:
		comf	CV515,w			; range (negative)
		addlw	0x01
		addlw	d'150'
		xorwf	PULSE1,w
		btfss	STATUS,Z
		return
		movlw	REACH_PULS
		movwf	CV547CNT
		bsf	REACHED,0
		return
EndServo1WW1:
		decfsz	CV547CNT,f
		return
		bcf	REACHED,0
		bcf	MOVING,0
		bsf	POSITION,0
		return
EndServo1Nxt:
		btfsc	REACHED,0
		goto	EndServo1WW2

		decfsz	CV547CNT,f		; speed
		goto	EndServo1W2
		movf	CV547,w
		movwf	CV547CNT
		incf	PULSE1,f
EndServo1W2:
		movlw	d'150'			; range
		addwf	CV515,w
		xorwf	PULSE1,w
		btfss	STATUS,Z
		return
		movlw	REACH_PULS
		movwf	CV547CNT
		bsf	REACHED,0
		return
EndServo1WW2:
		decfsz	CV547CNT,f
		return
		bcf	REACHED,0
		bcf	MOVING,0
		bcf	POSITION,0
		return

EndServo2:
		btfss	MOVING,1		; moving servo?
		return				; no

		btfsc	POSITION,1		; yes
		goto	EndServo2Nxt
		
		btfsc	REACHED,1
		goto	EndServo2WW1

		decfsz	CV548CNT,f		; speed
		goto	EndServo2W1
		movf	CV548,w
		movwf	CV548CNT
		decf	PULSE2,f
EndServo2W1:
		comf	CV516,w			; range (negative)
		addlw	0x01
		addlw	d'150'
		xorwf	PULSE2,w
		btfss	STATUS,Z
		return
		movlw	REACH_PULS
		movwf	CV548CNT
		bsf	REACHED,1
		return
EndServo2WW1:
		decfsz	CV548CNT,f
		return
		bcf	REACHED,1
		bcf	MOVING,1
		bsf	POSITION,1
		return
EndServo2Nxt:
		btfsc	REACHED,1
		goto	EndServo2WW2

		decfsz	CV548CNT,f		; speed
		goto	EndServo2W2
		movf	CV548,w
		movwf	CV548CNT
		incf	PULSE2,f
EndServo2W2:
		movlw	d'150'			; range
		addwf	CV516,w
		xorwf	PULSE2,w
		btfss	STATUS,Z
		return
		movlw	REACH_PULS
		movwf	CV548CNT
		bsf	REACHED,1
		return
EndServo2WW2:
		decfsz	CV548CNT,f
		return
		bcf	REACHED,1
		bcf	MOVING,1
		bcf	POSITION,1
		return

; ----------------------------------------------------------------------


Decode:
		bcf	NEW_PACKET		; prepare for next packet
		bcf	INTCON,INTE		; disable DCC interrupts for more speed
		btfsc	DCC4BYTE
		goto	Decode4bytes

; 3 byte packets:
; 1111111111 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1
; '1111 0 AAA' 'AAAAA 0 DD' DDDDDD 0 E' 'EEEEEEE 1'
;    DATA0        DATA1       DATA2        DATA3

Decode3bytes:
		rrf	DATA0,f
		rrf	DATA1,f
		rrf	DATA2,f
		rrf	DATA3,f			; 'x1111 0 AA''AAAAAA 0 D''DDDDDDD 0''EEEEEEE'
		btfss	STATUS,C		; check end bit
		goto	ExitDecode

		rrf	DATA0,f			; 'xx1111 0 A''AAAAAAA 0''DDDDDDDD''EEEEEEEE'
		rrf	DATA1,f
		rrf	DATA2,f
		btfsc	STATUS,C		; check start bit
		goto	ExitDecode

		rrf	DATA0,f
		rrf	DATA1,f			; 'xxx1111 0''AAAAAAAA''DDDDDDDD''EEEEEEEE'
		btfsc	STATUS,C		; check start bit
		goto	ExitDecode

		movf	DATA1,w			; exclusive or check
		xorwf	DATA2,w
		xorwf	DATA3,w
		btfss	STATUS,Z		; valid packet?
		goto	ExitDecode		; no, return

; 'AAAAAAAA''DDDDDDDD''EEEEEEEE'		; 3 byte packet
;   DATA1     DATA2     DATA3

		movf	DATA1,w			; address = '00000000' ?
		btfsc	STATUS,Z
		goto	Broadcast
;		movf	DATA1,w
		andlw	0xF0
		xorlw	0x70			; '0111xxxx'?
		btfsc	STATUS,Z
		goto	CheckSM			; yes, may be service mode

		movf	DATA1,w
		andlw	0xC0
		xorlw	0x80			;'10xxxxxx'? accessory operation
		btfsc	STATUS,Z
		goto	Accessory

		incfsz	DATA1,w			;'11111111' idle packet
		goto	ExitDecode
		bsf	INTCON,INTE		; yes, don't clear reset flag
		return

; 4 byte packets:
; 1111111111 CCCCCCCC 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1
; '111 0 CCCC''CCCC 0 AAA' 'AAAAA 0 DD' DDDDDD 0 E' 'EEEEEEE 1'
;    DATA00      DATA0        DATA1       DATA2        DATA3


Decode4bytes:
		bcf	DCC4BYTE
		rrf	DATA00,f
		rrf	DATA0,f
		rrf	DATA1,f
		rrf	DATA2,f
		rrf	DATA3,f			; 'x111 0 CCC''CCCCC 0 AA''AAAAAA 0 D''DDDDDDD 0''EEEEEEE'
		btfss	STATUS,C		; check end bit
		goto	ExitDecode

		rrf	DATA00,f
		rrf	DATA0,f			; 'xx111 0 CC''CCCCCC 0 A''AAAAAAA 0''DDDDDDDD''EEEEEEEE'
		rrf	DATA1,f
		rrf	DATA2,f
		btfsc	STATUS,C		; check start bit
		goto	ExitDecode

		rrf	DATA00,f
		rrf	DATA0,f
		rrf	DATA1,f			; 'xxx111 0 C''CCCCCCC 0''AAAAAAAA''DDDDDDDD''EEEEEEEE'
		btfsc	STATUS,C		; check start bit
		goto	ExitDecode

		rrf	DATA00,f
		rrf	DATA0,f			; 'xxxx111 0''CCCCCCCC''AAAAAAAA''DDDDDDDD''EEEEEEEE'
		btfsc	STATUS,C		; check start bit
		goto	ExitDecode


		movf	DATA0,w			; exclusive or check
		xorwf	DATA1,w
		xorwf	DATA2,w
		xorwf	DATA3,w
		btfss	STATUS,Z		; valid packet?
		goto	ExitDecode		; no, return

; '0111CCAA''AAAAAAAA''DDDDDDDD''EEEEEEEE'	; Direct mode
;    DATA0    DATA1     DATA2     DATA3

		movf	DATA0,w
		andlw	0xF0
		xorlw	0x70			; '0111xxxx'?
		btfss	STATUS,Z
		goto	ExitDecode		; yes, may be service mode

		btfss	RESET_FLG		; check for SM, reset packet has to come first
		goto	ServModeError
		btfss	PROG_2X
		goto	SetSM_Flag

		bcf	PROG_2X
;		bcf	RESET_FLG
		movf	DATA2,w			; save data
		movwf	EEDATA0

		btfsc	DATA0,0			; CV513.. or CV1..
		goto	ExitProg

		movf	DATA1,w			; x0AAAAAAAA
		call	FindCV
		btfsc	NOCV
		goto	ExitProg
ProgDirect:
		btfsc	DATA0,3	
		goto	RomNxt
		btfss	DATA0,2
		goto	ExitProg		;00 not defined
RomNxt:
		btfss	DATA0,2
		goto	BitMan			;10 Bit Manipulation
		btfss	DATA0,3
		goto	EEVERI			;01 Verify byte
WriteDirect:					;11 Write byte
		btfsc	RDONLY
		goto	CheckCV8
		call	SetParm
		call	AckPulse
		bcf	PROG_2X
		bcf	NEW_PACKET
		call	LoadCV
		goto	ExitProg

BitMan:
		call	EE_Read
		movwf	EEDATA0
		movlw	EEDATA0
		movwf	FSR
		movlw	b'00000111'
		andwf	DATA2,w
		call	BitPos
		btfss	DATA2,4			; K
		goto	Vbit			; K=0,verify bit
		btfsc	DATA2,3
		iorwf	INDF,f			; D=1,set bit
		xorlw	0xFF
		btfss	DATA2,3
		andwf	INDF,f			; D=0,clear bit
		movf	DATA1,w
		call	FindCV
		goto	ProgDirect		; write complete byte

Vbit:
		andwf	INDF,w
		btfsc	STATUS,Z
		goto	BitClear
BitSet:
		btfsc	DATA2,3			;D=0
		goto	DoAck			;D=1, ack
		goto	ExitProg
BitClear:
		btfss	DATA2,3			;D=1
		goto	DoAck			;D=0, ack
		goto	ExitProg
		


; ----------------------------------------------------------------------

F1A_Clear:
F1B_Clear:
F2A_Clear:
F2B_Clear:

		goto	ExitDecode


F1A_Set:
		btfss	POSITION,0
		bsf	MOVING,0
		movf	CV547,w
		movwf	CV547CNT
		goto	ExitFunction
F1B_Set:
		btfsc	POSITION,0
		bsf	MOVING,0
		movf	CV547,w
		movwf	CV547CNT
		goto	ExitFunction

F2A_Set:
		btfss	POSITION,1
		bsf	MOVING,1
		movf	CV548,w
		movwf	CV548CNT
		goto	ExitFunction
F2B_Set:
		btfsc	POSITION,1
		bsf	MOVING,1
		movf	CV548,w
		movwf	CV548CNT
		goto	ExitFunction
ExitFunction:
		bcf	RESET_FLG
		bsf	INTCON,INTE		; enable DCC interrupts
		btfss	SAVE_OUTPUTS
		return
		comf	POSITION,w		; save new output state
		xorwf	MOVING,w
		movwf	EEDATA0
		movlw	EE_OUT
		goto	SetParm


; ----------------------------------------------------------------------

Broadcast:
		movf	DATA2,w			; reset packet?
		btfss	STATUS,Z
		goto	ExitDecode
		bcf	PROG_2X
		bsf	RESET_FLG
						; reset decoder
		clrf	GPIO
;		clrf	MOVING
		bsf	INTCON,INTE		; enable DCC interrupts
		return


ExitDecode:
		bcf	RESET_FLG
		bsf	INTCON,INTE		; enable DCC interrupts
		return

;************* SM Mode *******************************************
; Service Mode
CheckSM:
		btfss	RESET_FLG		; check for SM, reset packet has to come first
		goto	ServModeError
		btfss	PROG_2X
		goto	SetSM_Flag

		bcf	PROG_2X
;		bcf	RESET_FLG
		movf	DATA2,w				; save data
		movwf	EEDATA0

		movf	DATA1,w				; 3 byte programming
		andlw	b'11110111'
		xorlw	b'01110101'			; Reg6
		btfsc	STATUS,Z		
		goto	REG6
		xorlw	(b'01110101')^(b'01110100')	; Reg5
		btfsc	STATUS,Z		
		goto	REG5
		xorlw	(b'01110100')^(b'01110110')	; reg7
		btfsc	STATUS,Z
		goto	REG7	
		xorlw	(b'01110110')^(b'01110111')	; reg8
		btfsc	STATUS,Z
		goto	REG8	

		movf	DATA1,w
		andlw	0x03
		addwf	PAGEREG,w
		call	FindCV
		btfsc	NOCV
		goto	ExitProg
ProgReg:
		btfss	DATA1,3
		goto	EEVERI
		goto	EEPROG

REG5:
		movlw	E_CV541			; CV541 configuration
		goto	ProgReg

REG6:
		btfss	DATA1,3			; read or write
		goto	REG6RD
		decf	DATA2,f			; Page register
		rlf	DATA2,f
		rlf	DATA2,w
		andlw	b'11111100'		; page 1 and 129 are the same. CV1 & CV513
		movwf	PAGEREG
		goto	ExitProg
REG6RD:
		decf	DATA2,f			; read page register
		rlf	DATA2,f
		rlf	DATA2,w
		andlw	b'11111100'		; page 1 and 129 are the same. CV1 & CV513
		xorwf	PAGEREG,w
		goto	EEVERIP

REG7:   
		movlw	E_CV7			; only read
		btfss	DATA1,3
		goto	EEVERI
		goto	ExitProg

REG8:
		movlw	E_CV8			; only read
		btfss	DATA1,3
		goto	EEVERI
		goto	CheckResetCV		; if CV8 = 33 reset CV

	
EEPROG:
		btfsc	RDONLY
		goto	CheckCV8
		call	SetParm			; program EEPROM
		call	AckPulse		; do ACK
		bcf	PROG_2X
		bcf	NEW_PACKET
		call	LoadCV
		goto	ExitProg


EEVERI:
		call	EE_Read			; check data
		xorwf	DATA2,w
EEVERIP:
		btfss	STATUS,Z
		goto	ExitProg
DoAck:
		call	AckPulse		; equal, do ACK
		bcf	PROG_2X
;		bcf	NEW_PACKET
		goto	ExitProg

SetSM_Flag:
		bsf	PROG_2X
		goto	ExitProg

ServModeError:
		bcf	RESET_FLG
		bcf	PROG_2X
		goto	ExitProg

CheckCV8:
		xorlw	E_CV8			; CV8?
		btfss	STATUS,Z
		goto	ExitProg
CheckResetCV:
		movlw	d'33'			; CV8 = 33 -> reset CV
		xorwf	DATA2,w
		btfss	STATUS,Z
		goto	ExitProg
		call	ResetCV			; program CV defaults
		call	AckPulse		; do ACK
		bcf	PROG_2X
		bcf	NEW_PACKET
		call	LoadCV
		goto	ExitProg

ExitProg:
		bsf	INTCON,INTE		; enable interrupts
		return


; -----------------------------------------------------------------------------------

AckPulse:
		clrf 	TEMP
		movlw	0x10			; all outputs on (1,5ms)
		movwf	GPIO
		incfsz	TEMP,f			; 765uS ON	1
		goto	$-1			;		2,3

		movlw	0x30
		movwf	GPIO
		incfsz	TEMP,f			; 765uS ON	1
		goto	$-1			;		2,3
		movlw	0x20
		movwf	GPIO
		incfsz	TEMP,f			; 765uS ON	1
		goto	$-1			;		2,3
		movlw	0x00
		movwf	GPIO
		incfsz	TEMP,f			; 765uS ON	1
		goto	$-1			;		2,3
		movlw	0x10
		movwf	GPIO
		incfsz	TEMP,f			; 765uS ON	1
		goto	$-1			;		2,3

		movlw	0x30
		movwf	GPIO
		incfsz	TEMP,f			; 765uS ON	1
		goto	$-1			;		2,3
		movlw	0x20
		movwf	GPIO
		incfsz	TEMP,f			; 765uS ON	1
		goto	$-1			;		2,3
		movlw	0x00
		movwf	GPIO
		incfsz	TEMP,f			; 765uS ON	1
		goto	$-1			;		2,3
		clrf	GPIO
		return


LoadCV:
		movlw	E_CV513			; address low
		call	EE_Read
		movwf	CV513
		movlw	E_CV521			; address high
		call	EE_Read
		movwf	CV521
		swapf	CV521,f
		comf	CV521,w			; top address is complemented
		andlw	0x70
		movwf	CV521			; now CV521='0AAA0000'

		movlw	E_CV515			; range servo1
		call	EE_Read
		movwf	CV515
		movlw	E_CV516			; range servo2
		call	EE_Read
		movwf	CV516

		movlw	E_CV547			; speed servo1
		call	EE_Read
		movwf	CV547
		movlw	E_CV548			; speed servo2
		call	EE_Read
		movwf	CV548

		movlw	E_CV545			; spacing
		call	EE_Read
		movwf	CV545

		movlw	E_CV546			; flags
		call	EE_Read
		movwf	CV546

		return


LoadOutputs:
		movlw	EE_OUT			; read saved outputs
		call	EE_Read
		movwf	POSITION			

		movf	CV515,w			; calculate pulse for position
		btfss	POSITION,0
		xorlw	0xFF
		btfss	POSITION,0
		addlw	0x01
		addlw	d'150'
		movwf	PULSE1

		movf	CV516,w			; calculate pulse for position
		btfss	POSITION,1
		xorlw	0xFF
		btfss	POSITION,1
		addlw	0x01
		addlw	d'150'
		movwf	PULSE2


		movlw	0x10			; pulses for setting position
		movwf	CV547CNT
		movwf	CV548CNT
		movlw	0x33			; do move to reached position
		movwf	REACHED
		movwf	MOVING
		return


; -----------------------------------------------------------------------------------

FindCV:
    btfss PROGRAM_MODE ;CV write is possibile only in PROGRAM_MODE
    goto CvNotFound
		bcf	NOCV
		bcf	RDONLY

    xorlw	0x00			; CV513
		btfsc	STATUS,Z
		retlw	E_CV513
		xorlw	(0x00 ^ 0x08)		; CV521
		btfsc	STATUS,Z
		retlw	E_CV521
		xorlw	(0x08 ^ 0x1C)		; CV541
		btfsc	STATUS,Z
		retlw	E_CV541
		xorlw	(0x1C ^ 0x02)		; CV515
		btfsc	STATUS,Z
		retlw	E_CV515
		xorlw	(0x02 ^ 0x03)		; CV516
		btfsc	STATUS,Z
		retlw	E_CV516
		xorlw	(0x03 ^ 0x20)		; CV545 
		btfsc	STATUS,Z
		retlw	E_CV545
		xorlw	(0x20 ^ 0x21)		; CV546
		btfsc	STATUS,Z
		retlw	E_CV546
		xorlw	(0x21 ^ 0x22)		; CV547
		btfsc	STATUS,Z
		retlw	E_CV547
		xorlw	(0x22 ^ 0x23)		; CV548
		btfsc	STATUS,Z
		retlw	E_CV548


		bsf	RDONLY
		xorlw	(0x23 ^ 0x06)		; CV519
		btfsc	STATUS,Z
		retlw	E_CV7
		xorlw	(0x06 ^ 0x07)		; CV520
		btfsc	STATUS,Z
		retlw	E_CV8
CvNotFound:
		bsf	NOCV			; CV not finded
		retlw	0x7F			; return last location

;---------------------------------------------------------------------------

ResetCV:
		movlw	0x01			; reset CV to default values
		movwf	EEDATA0
		movlw	E_CV513
		call	SetParm
		
		movlw	0x32
		movwf	EEDATA0
		movlw	E_CV515
		call	SetParm
		movlw	E_CV516
		call	SetParm

		movlw	0x00
		movwf	EEDATA0
		movlw	E_CV521
		call	SetParm

		movlw	0x01
		movwf	EEDATA0
		movlw	E_CV547
		call	SetParm
		movlw	E_CV548
		call	SetParm

		movlw	0x4E
		movwf	EEDATA0
		movlw	E_CV545
		call	SetParm

		movlw	0x80
		movwf	EEDATA0
		movlw	E_CV541
		call	SetParm

		movlw	0x01
		movwf	EEDATA0
		movlw	E_CV546
		call	SetParm

		return


;---------------------------------------------------------------------------

EE_Read:
		bsf	STATUS,RP0		; w=ADR
		movwf	EEADR
		bsf	EECON1,RD
		movf	EEDATA,w
		bcf	STATUS,RP0
		return

SetParm:
		call	EE_Read			; w=ADR, EEDATA0=data. Write only changes
		xorwf	EEDATA0,w
		btfsc	STATUS,Z
		return
EE_Write:		
		movf	EEDATA0,w
		bsf	STATUS,RP0
		movwf	EEDATA
		bsf	EECON1,WREN
		bcf	INTCON,GIE
		movlw	0x55
		movwf	EECON2
		movlw	0xAA
		movwf	EECON2
		bsf	EECON1,WR
		bsf	INTCON,GIE
		bcf	EECON1,WREN
EEWrite0:
		btfsc	EECON1,WR
		goto	EEWrite0
		bcf	STATUS,RP0
		return



; ----- EEPROM default values


		org	0x2100

		dw	0x01			; CV513	Primary Adress (low bits)
		dw	0xFF			; 
		dw	0x32			; CV515 Range servo 1 (in 10us)
		dw	0x32			; CV516 Range servo 2
		dw	0x32			; CV517 Range servo 3
		dw	0x32			; CV518 Range servo 4
		dw	0x14			; CV519 Manufacturer Version
		dw	0x0D			; CV520	Manufacturer ID
		dw	0x00			; CV521 Primary Adress (high bits)
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0x80			; CV541 Config
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0xFF			; 
		dw	0x4E			; CV545	Spacing (in 256us)
		dw	0x01			; CV546 Accesory flags
		dw	0x01			; CV547	Speed servo 1
		dw	0x01			; CV548 Speed servo 2
		dw	0x01			; CV549 Speed servo 3
		dw	0x01			; CV550 Speed servo 4


		org	0x2130

		dt	"2SRV2RLY"
		dt	"F.Cañada"
		dt	(__VERDAY   >> 4)  +0x30
		dt	(__VERDAY   & 0x0F)+0x30,"/"
		dt	(__VERMONTH >> 4)  +0x30
		dt	(__VERMONTH & 0x0F)+0x30,"/"
		dt	(__VERYEAR  >> 4)  +0x30
		dt	(__VERYEAR  & 0x0F)+0x30

		org	0x217F

		dw	0x00			; default position


	end
