/*
 * File:        uart.c
 * Purpose:     Provide UART routines for serial IO
 *
 * UART0 Notes:		
 *	Clock enabled/disable: SIM_SCGC4, bit 10
 *	Baud rate: UART0_BDH (bits 4-0), UART0_BDL (bits 7-0)
 *	Baud rate adjust: UART0_C4, bits 4-0
 *	Transmit enable/disable: UART0_C2, bit 3
 *	Recieve enable/disable: UART0_C2, bit 2
 *	Recieve source select: SIM_SOPT5, bits 3-2
 *	Transmit source select: SIM_SOPT5, bits 1-0
 *	Transmit/Recieve: Port B, bits 16/17
 */

#include "MK64F12.h"
#include "uart.h"
#define BAUD_RATE 9600      // Default baud rate 
#define SYS_CLOCK 20485760  // Default system clock (see DEFAULT_SYSTEM_CLOCK in system_MK64F12.c)

void init_uart()
{
  // Define variables for baud rate and baud rate fine adjust
  uint16_t ubd, ubdH, brfa;
  
  // Enable clock for UART
  SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
  
  // Configure the port control register to alternative 3 (which is UART mode for K64)
  PORTB_PCR16 = PORT_PCR_MUX(3);
  PORTB_PCR17 = PORT_PCR_MUX(3);

  
  /* Configure the UART for establishing serial communication */
  // Disable transmitter and receiver until proper settings are chosen for the UART module
  UART0_C2 &= ~(UART_C2_TE_MASK);
	UART0_C2 &= ~(UART_C2_RE_MASK);
  
  // Select default transmission/reception settings for serial communication of UART by clearing the control register 1
  UART0_C1 = 0x00; // 8 data bits, no parity, 1 stop, 50MHz
  
  // UART Baud rate is calculated by: baud rate = UART0 module clock / (16 × (SBR[12:0] + BRFD))
  // 13 bits of SBR are shared by the 8 bits of UART0_BDL and the lower 5 bits of UART0_BDH 
  // BRFD is dependent on BRFA, refer Table 52-234 in K64 reference manual
  // BRFA is defined by the lower 4 bits of control register, UART0_C4 
  
  // Calculate baud rate settings: ubd = UART module clock/(16 * baud rate)
  ubd = (uint16_t) ((SYS_CLOCK) / (BAUD_RATE * 16));
  
  // Retrieve the upper half of ubd
  ubdH = (ubd >> 8) & 1;
  
  // Clear SBR bits of BDH
  UART0_BDH &= ~UART_BDH_SBR_MASK;
  
  // Distribute this ubd in BDH and BDL
  UART0_BDH |= (ubdH & UART_BDH_SBR_MASK);
  UART0_BDL |= (ubd & UART_BDL_SBR_MASK);
  
  // BRFD = (1/32)*BRFA 
  // Make the baud rate closer to the desired value by using BRFA
  brfa = (((SYS_CLOCK * 32) / (BAUD_RATE * 16)) - (ubd * 32));
  
  // Write the value of brfa in UART0_C4
  UART0_C4 |= (brfa & UART_C4_BRFA_MASK);
      
  // Enable transmitter and receiver of UART0
  UART0_C2 |= UART_C2_TE_MASK;
	UART0_C2 |= UART_C2_RE_MASK; 
}

// Switch 2: Port C, Pin 6
void init_button(void){
	// Enable clock for Port C PTC6 button
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; // Enables Clock on PORTC
	
	// Configure the Mux for the button
    PORTC_PCR6 = PORT_PCR_MUX(1); // Enable Switch 2

	// Set the push button as an input
	GPIOC_PDDR = (1 >> 6); // Set Switch 2 as input

}

void put(char *ptr_str)
{
	while(*ptr_str)
    uart_putchar(*ptr_str++);
}

void putnumU(int i)
{
	
}

uint8_t uart_getchar()
{
    // Wait until there is space for more data in the receiver buffer
    while((UART0_S1 & UART_S1_RDRF_MASK) == 0){}

    // Return the 8-bit data from the receiver
    return UART0_D;
}

void uart_putchar(char ch)
{
    // Wait until transmission of previous bit is complete
    while((UART0_S1 & UART_S1_TDRE_MASK) == 0){}
	
    // Send the character
    UART0_D = ch;
}
