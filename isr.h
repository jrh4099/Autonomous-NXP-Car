#ifndef ISR_H_
#define ISR_H_

typedef enum {false=0, true=1} boolean;

// Definitions for each of the LED colors
#define RED 22
#define GREEN 26
#define BLUE 21

void PDB0_IRQHandler(void);
void FTM0_IRQHandler(void);
void PORTA_IRQHandler(void);
void PORTC_IRQHANDLER(void);

#endif /* ifndef ISR_H_ */
