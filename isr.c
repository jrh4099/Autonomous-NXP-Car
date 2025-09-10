/*
 * isr.c
 */

#include "isr.h"
#include "MK64F12.h"
#include <stdio.h>
#include "uart.h"

//variables global to the IRQ handlers which dictates if timer is enabled & timer counter
boolean isPressed = false;
boolean LEDon = true;
int counter = 0;

void PDB0_IRQHandler(void){ // For PDB timer
	// Clear the interrupt in register PDB0_SC
	PDB0_SC &= ~PDB_SC_PDBIF_MASK;
    
    // Toggle the output state for LED1
    LEDon = ~LEDon;
    
	return;
}
	
void FTM0_IRQHandler(void){ // For FTM timer
	// Clear the interrupt in register FTM0_SC
	FTM0_SC &= ~FTM_SC_TOF_MASK;
    
	// If SW2 set global button press variable
    if(isPressed){
        // Increment global counter
        counter++;
    } 
    // Else do nothing
    
	return;
}
	
void PORTA_IRQHandler(void){ // For switch 3
	// Clear the interrupt
	PORTA_PCR4 |= PORT_PCR_ISF_MASK;
    
    // If the timer is enabled
	if((PDB0_SC & PDB_SC_PDBEN_MASK) == 128){
		PDB0_SC &= ~PDB_SC_PDBEN_MASK;
    }
    // Else
	else{
        // Enable the timer and start it with a trigger
		PDB0_SC |= PDB_SC_PDBEN_MASK;
    }
	
	return;
}
	
void PORTC_IRQHandler(void){ // For switch 2
	// Clear the interrupt
    PORTC_PCR6 |= PORT_PCR_ISF_MASK;
    
    // If SW2 was being pressed
	if((GPIOC_PDIR & (1 << 6)) == 0){
        // Set a global variable to affect timer2 function
        isPressed = true;
        
        // Reset the FlexTimer
        FTM0_CNT &= 0x0000;
        
        // Reset the timer counter
        counter = 0;
        
        // Turn on the blue LED while button is pressed
		if((PDB0_SC & PDB_SC_PDBEN_MASK) == 128){
			GPIOB_PSOR = (1 << RED);
			GPIOE_PSOR = (1 << GREEN);
			GPIOB_PCOR = (1 << BLUE);
		}
    }
    else{
        // Reset the global variable to affect timer2 function
        isPressed = false;
        
        // Turn off the blue LED while button is up
		GPIOB_PSOR = (1 << RED);
		GPIOE_PSOR = (1 << GREEN);
        GPIOB_PSOR = (1 << BLUE);
        
        // Print out "Button held for XX milliseconds!" (use put)
		char ptr_str[50] = "";
		sprintf(ptr_str, "\r\nButton held for %d milliseconds!", counter);
		put(ptr_str);
		//put("test");
    }
	return;
}
