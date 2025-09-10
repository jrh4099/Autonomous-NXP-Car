#ifndef PWM_H_
#define PWM_H_

void SetDutyCycle1(unsigned int DutyCycle, unsigned int Frequency, int dir);
void SetDutyCycle2(unsigned int DutyCycle, unsigned int Frequency, int dir);
void SetDutyCycleServo(unsigned int DutyCycle, unsigned int Frequency);
void InitPWM();
void PWM_ISR();

#endif /* PWM_H_ */
