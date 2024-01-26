# WORKS FINE , PD5 TX , PD6 RX , on receiving 8 bytes DMA receive complete interrupt fires , prints out received bytes to terminal 
# Baud 9600 for receive and transmit
#BRR = BAUD  
#13.6=38400
#4.4 = 115200
#52.1 = 9600
#8.7 =  57600
fclk = 24000000   # 24Mhz RCO internal , AHB =8Mhz by default
#USARTx_TX Full-duplex mode Push-pull multiplexed outputs
#USARTx_RX Full-duplex mode Floating input or pull-up input
#PD5 -tx
#PD6 -rx

buffer0 = 0x20000004
statusreg = 0x20000004
buffer1 = 0x20000008
result_low = 0x20000004
result_hi = 0x20000008
state = 0x2000000C
result1 = 0x20000010
result2 = 0x20000014
dividend = 0x20000018 
divisor = 0x2000001C
scratch = 0x20000020
mem = 0x20000024
include CH32V003_reg1.asm

vtable:
	j reset_handler		#  longs 0x00000000 # RESERVED 0
align 4
  longs   0x00000000 # RESERVED 1
  longs   0x00000000 #pack <l longs NMI_IRQhandler
  longs   0x00000000 #pack <l HardFault_IRQhandler
  longs   0x00000000 # RESERVED 4
  longs   0x00000000 # RESERVED 5
  longs   0x00000000 # RESERVED 6
  longs   0x00000000 # RESERVED 7
  longs   0x00000000 # RESERVED 8
  longs   0x00000000 # RESERVED 9
  longs   0x00000000 # RESERVED 10
  longs   0x00000000 # RESERVED 11
  longs   0x00000000 # pack <l SysTick_IRQhandler	#; place the address of the mtime ISR subroutine in the vector table position 7,assembler will store isr address here, longs 0x00000000 # RESERVED 12	
  longs   0x00000000 # RESERVED 13
  longs   0x00000000 #pack <l SW_Software_IRQhandler
  longs   0x00000000 # RESERVED 15
  longs   0x00000000 #pack <l WWDG_IRQhandler
  longs   0x00000000 #pack <l PVD_IRQhandler
  longs   0x00000000 #pack <l FLASH_IRQhandler
  longs   0x00000000 #pack <l RCC_IRQhandler
  longs   0x00000000 #pack <l EXTI7_0_IRQhandler
  longs   0x00000000 #pack <l AWU_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH1_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH2_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH3_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH4_IRQhandler
pack <l DMA1_CH5_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH6_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH7_IRQhandler
  longs   0x00000000 #pack <l ADC1_IRQhandler
  longs   0x00000000 #pack <l I2C1_EV_IRQhandler
  longs   0x00000000 #pack <l I2C1_ER_IRQhandler
  longs   0x00000000 #pack <l USART1_IRQhandler
  longs   0x00000000 #pack <l SPI1_IRQhandler
  longs   0x00000000 #pack <l TIM1BRK_IRQhandler
  longs   0x00000000 #pack <l TIM1UP_IRQhandler
  longs   0x00000000 #pack <l TIM1TRG_COM_IRQhandler
  longs   0x00000000 #pack <l TIM1CC_IRQhandler
  longs   0x00000000 #pack <l TIM2_IRQhandler

reset_handler:


    	li sp, STACK			# load stack pointer with stack end address
	 
    	li t0, vtable			#BASEADDR[31:2],The interrupt vector table base address,which needs to be 1KB aligned
    	ori t0, t0, 3			#BASEADDR[31:2],1: Identify by absolute address,1: Address offset based on interrupt number *4
    	#csrrw zero,t0, mtvec		# write to mtvec
	longs 0x30529073  
    
   	li t0,main
	longs 0x34129073          	#csrw	mepc,t0 :mepc updated with address of main
	longs 0x30200073		# mret ( return from interrupt)	.
  
	align 4
main:
	nop


#enable periphrel clocks
	li x10,R32_RCC_APB2PCENR	# load address of APB2PCENR register to x10 ,for enabling GPIO A,D,C peripherals
	lw x11,0(x10)			# load contents from peripheral register R32_RCC_APB2PCENR pointed by x10
	li x7,((1<<0)|(1<<2)|(1<<4)|(1<<5)|(1<<14))	# 1<<IOPA_EN,1<<IOPC_EN,1<<IOPD_EN,1<<USART_EN
	or x11,x11,x7			# or values 
	sw x11,0(10)			# store modified enable values in R32_RCC_APB2PCENR

# enable DMA clock
	li x10,R32_RCC_AHBPCENR
	lw x11,0(x10)
	ori x11,x11,(1<<0)		# 1<<DMA1EN , enable DMA1 clock
	sw x11,0(10)

#enable DMA receive in USART control register3
	li x10,R32_USART_CTLR3		# 
	lw x11,0(x10)
	li x7,(1<<6)			# 1<<DMAR enable DMA RX
	or x11,x11,x7
	sw x11,0(x10)


#configure GPIO 
	li x10,R32_GPIOD_CFGLR		# load pointer x10 with address of R32_GPIOD_CFGLR , GPIO configuration register
	lw x11,0(x10)			# load contents from register pointed by x10
	li x7,~((0xf<<20)|(0xf<<24)|(0xf<<16))	#clear pd4,pd5,pd6. we need to setup PD5 & PD6 for usart tx and rx and pd4 for led
	and x11,x11,x7			# clear pd4,pd5,pd6 mode and cnf bits for selected pin D4,D5,D6
	li x7,((0x8<<24)|(0xB<<20)|(0x3<<16))	# pd6 = input with PU/PD,pd5= multiplex pushpull output 50mhz,pd4= normal pushpull output 50hz
	or x11,x11,x7			# OR value to register
	sw x11,0(x10)			# store in R32_GPIOD_CFGLR

#enable pull up for input	
	li x10,R32_GPIOD_OUTDR		# enable pullup resistor by setting OUTDR register
	lw x11,0(x10)			# this setting of GPIO_OUTDR for pullup resistor effects if corresonding pin is selected as input
	li x7,(1<<6)			#when PD6 is input with resistor selected 1= pullup and 0 = pulldown
	or x11,x11,x7
	sw x11,0(x0)

#configure USART baud
	li x10,R32_USART_BRR		# USART BAUD setting
	lw x11,0(x10)			# copy R32_USART_BRR to x11
	li x7,((52<<4)|(1<<0))		# 52.1 in BRR =9600
	or x11,x11,x7			# or registers
	sw x11,0(x10)			# store in R32_USART_BRR


#configure DMA
	li x10,R32_DMA_CFGR5
	lw x11,0(x10)
	li x7,((3<<12)|(1<<7)|(1<<5)|(1<<1))	#0<<MEM2MEM,0<<PL(13,12),0<<Msize(12,10),0<<Psize(9,8),1<<MINC(7),0<<PINC(6),1<<CIRC(5),0<<DIR(4),0<<TEIE(3),0<<HTIE(2),1<<TCIE(1),1<<EN(0)
	or x11,x11,x7
	sw x11,0(x10)

	li x10,R32_DMA_CNTR5		# data size expected
	li x7,8				# 8 bytes
	sw x7,0(x10)

	li x10,R32_DMA_PADDR5		# pripheral register address (source)
	li x7,R32_USART_DATAR		# address of usart data register
	sw x7,0(x10)

	li x10,R32_DMA_MADDR5		# memrory destination address register
	li x7,result1			# result1 is in SRAM ,address of result1 loaded in x7
	sw x7,0(x10)

	li x10,R32_DMA_CFGR5
	lw x11,0(x10)
	li x7,(1<<0)			# 1<<EN(0)
	or x11,x11,x7
	sw x11,0(x10)

#setup UART control and enable	
	li x10,R32_USART_CTLR1		# load x10 with R32_USART_CTLR1 address
	lw x11,0(x10)			# load to x11 contents
	li x7,(1<<13)|(1<<3)|(1<<2)	# enable USART UE, TX,RX bits		# UE 
	or x11,x11,x7
	sw x11,0(x10)			# store back new values


PFIC_CONFIG:
	li x10,R32_PFIC_CFGR		# reset core PFIC register for interrupts
	lw x11,0(x10)
	li x7,((PFIC_KEY3<<16)|(1<<7))	# key3  and SYSRESET , reference manual tells to do it
	or x11,x11,x7
	sw x11,0(x10)			# store back new values

	li x10,R32_PFIC_IENR1		# PFIC Interrupt Enable in core PFIC
	lw x11,0(x10)
	li x7,(1<<26)			# enabled DMA1 CH5 interrupts in PFIC
	or x11,x11,x7
	sw x11,0(x10)			# store back new values

# enabling GLOBAL INTERRUPTS
	li t0, 0x88			# load MPIE and MIE bits , 1<<MIE in mstatus is enabling GLOBAL INTERRUPTS
	longs 0x30029073        	#csrw	mstatus,t0 ,manually assembled opcade to do csrrw the values in t0,  


#blink led
	call PD4_ON			# flash/blink led once for debugging

#main endless loop for uart transmit

example:

	li x10,name			# load address of label "name" to x10, string to be transmitted
string_loop:
	lb x8,0(x10)			# load 1 byte from 0 offset of "name"
	beqz x8,finish			# if byte in x8 null branch to label "finish"
	call USART_TX			# call subroutine USART_TX to transmit byte
	addi x10,x10,1			# increase pointer by 1 byte
	j string_loop			# jump back to label string_loop until null is encountered
finish:
	#call delay			# some delay
	j finish			# continoues loop, keep on sending the string with delay
#############################################################################################################
# SUBROUTINES
############################################################################################################	
USART_TX:
	addi sp,sp,-16			# add space in stack
	sw ra,0(sp)			# push ra
	sw x7,4(sp)			# push x7
	sw x10,8(sp)			# push x10
	sw x11,12(sp)			# push x11

	li x10,R32_USART_STATR		# load address of usart status register
	lw x11,0(x10)			# load contents of status register in x11
	andi x11,x11,(1<<7)		# mask out 7th bit, transmit buffer empty flag
	beqz x11,USART_TX		# if 0 transmit buffer full, wait until bit is set
	#li x8,0x30
	mv x7,x8			# move byte in x8 to x7
	li x10,R32_USART_DATAR		# x10 has the address of data register
	sb x7,0(x10)			#store byte in x7 to data register
TC_check:
	li x10,R32_USART_STATR		# get contents of status register again
	lw x11,0(x10)
	andi x11,x11,(1<<6)		# check transmit complete bit
	beqz x11,TC_check		# wait if bit is 0 , when transmit complete = 1
		
	lw x11,12(sp)			# pop x11
	lw x10,8(sp)			# pop x10
	lw x7,4(sp)			# pop x7
	lw ra,0(sp)			# pop ra
	addi sp,sp,16			# set SP back 16 bytes
	ret				# return to caller
######################################################################################################
# Blinks LED on pd4, used for debugging
#######################################################################################################
PD4_ON:
	addi sp,sp,-16			# move sp 16 bytes downward(4 words)
	sw ra,0(sp)			# push ra
	sw x7,4(sp)			# push x7
	sw x10,8(sp)			# push x10
	sw x11,12(sp)			# push x11
	li x10,R32_GPIOD_BSHR		# R32_GPIOD_BSHR register sets and resets GPIOD pins, load address into pointer x10
	lw x11,0(x10)			# load contents to x11
	li x7,1<<20			# reset pd4 by shifting 1 into bit position 20 of R32_GPIOD_BSHR
	or x11,x11,x7			# OR with x11
	sw x11,0(x10)			# store x11 to R32_GPIOD_BSHR
	

	call delay			# delay subroutine

PD4_OFF:
	li x10,R32_GPIOD_BSHR		# R32_GPIOD_BSHR register sets and resets GPIOD pins, load address into pointer x10
	lw x11,0(x10)			# load contents to x11
	li x7,(1<<4)			# set pd4 by shifting 1 to bit position 4
	or x11,x11,x7			# OR with x11
	sw x11,0(x10)			# store x11 to R32_GPIOD_BSHR

	

	call delay			# delay subroutine

	lw x11,12(sp)			# pop x11
	lw x10,8(sp)			# pop x10
	lw x7,4(sp)			# pop x7
	lw ra,0(sp)			# pop ra
	addi sp,sp,16			# move sp back 4 words
	ret				# return to caller
###################################################
# DELAY
###################################################
delay:	
	addi sp,sp,-8			# move sp 2 words
	sw ra,0(sp)			# push ra
	sw x6,4(sp)			# push x6
	li x6,2000000			# load an arbitarary value 20000000 to t1 register		
dloop:
	addi x6,x6,-1			# subtract 1 from t1
	bne x6,zero,dloop		# if t1 not equal to 0 branch to label loop
	lw x6,4(sp)			# pop x6
	lw ra,0(sp)			# pop ra
	addi sp,sp,8			# sp back 2 words
	ret				# return to caller
###########################################################
name:
string SAJEEV SANKARAN CH32V003 UART DMA RECEIVE
eol:
bytes 0x0d,0x0a,0x00

##########################################################
# DMA ISR
##########################################################

DMA1_CH5_IRQhandler:
	addi sp,sp,-60    		# adjust stack pointer
	sw x15,56(sp)			# PUSH
	sw x14,52(sp)			# PUSH
	sw x13,48(sp)			# PUSH
	sw x12,44(sp)			# PUSH
	sw x11,40(sp)			# PUSH
	sw x10,36(sp)			# PUSH
	sw x9,32(sp)			# PUSH
	sw x8,28(sp)			# PUSH
	sw x7,24(sp)			# PUSH
	sw x6,20(sp)			# PUSH
	sw x5,16(sp)			# PUSH
	sw x4,12(sp)			# PUSH
	sw x3,8(sp)			# PUSH
	sw x2,4(sp)			# PUSH
	sw x1,0(sp)			# PUSH
	
###########
	li x10,R32_DMA_INTFCR
	lw x11,0(x10)
	li x7,(1<<16)
	or x11,x11,x7
	sw x11,0(x10)
#print result of receive buffer	
	li x10,result1			# load address of variable "result1" to x10, string to be transmitted
	li x7,10
transmit_loop:
	lb x8,0(x10)			# load 1 byte from 0 offset of "result1"
	beqz x7,Tfinish			# if byte in x8 null branch to label "finish"
	call USART_TX			# call subroutine USART_TX to transmit byte
	addi x10,x10,1			# increase pointer by 1 byte
	addi x7,x7,-1			# decrease couter of 10 bytes
	j transmit_loop			# jump back to label string_loop until null is encountered
Tfinish:
	nop
blank_line:
	li x8,' '
	call USART_TX			# call uart
	li x8,0x0d			# line feed
	call USART_TX			# call uart
	li x8,0x0a			# carriage feed
	call USART_TX			# call uart
############
	lw x1,0(sp)			# POP
	lw x2,4(sp)			# POP
	lw x3,8(sp)			# POP
	lw x4,12(sp)			# POP
	lw x5,16(sp)			# POP
	lw x6,20(sp)			# POP
	lw x7,24(sp)			# POP
	lw x8,28(sp)			# POP
	lw x9,32(sp)			# POP
	lw x10,36(sp)			# POP
	lw x11,40(sp)			# POP
	lw x12,44(sp)			# POP
	lw x13,48(sp)			# POP
	lw x14,52(sp)			# POP
	lw x15,56(sp)			# POP
	addi sp,sp,60			# adjust stack pointer
	longs 0x30200073		# mret (manually assembled opcode for mret as per RISCV spec)
	
#################	