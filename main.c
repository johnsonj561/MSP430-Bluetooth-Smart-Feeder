/*Justin Johnson - FAU 2015
 *MSP430 communication with BlueSmirf Bluetooth Module V5
 *Micro remains in low power mode until interrupt is triggered by incoming data
 *Commands: W = measures weight (W) sensor and sends value to bluetooth module for to provide user feedback
 *Commands: F = activates servo motor on P1.6
 *baudrate = 9600
 *TXD = P1.1
 *RXD = P1.2
 *IR Sensor to detect Hopper food supply = P1.3
 *Pressure Sensor to detect Bowl  food supply = P1.4
 *IR Sensors to detect bowl activity (presence of animal) = P1.5
 *Servo Motor Dispenses Food = P1.6
 *IR Light to Detect Hopper Supply = P1.7
 *
 */
#include <msp430g2253.h>

#define MCU_CLOCK			1000000
#define PWM_FREQUENCY		50

unsigned int PWM_Period		= (MCU_CLOCK / PWM_FREQUENCY);	// PWM Period
unsigned int PWM_Duty		= 500;							// %
unsigned char Rx_Data = 0;					//temp RX variable
unsigned char Tx_Data = 0;					//temp TX variable
int weight = 0;								//stores weight of food supply
int ir = 0;									//stores value of IR sensor
int light = 0;								//light value to detect hopper food shortage
int ADCReading [3];							//array to store analog digital conversion values
int motorActive = 0;						//flag to control motor activity
int i;										//counter

int stopMotor(void);						//prevents motor operation if object is detected within 2 ft proximity
void dispenseFood(void);					//activates servo motor to dispense food
void sendFoodSupply(int);					//sends appropriate message to mobile device via Bluetooth
void sendHopperSupply(int);					//sends appropriate message to mobile device via Bluetooth
void UARTSendArray(char *TxArray, int ArrayLength);//sends message through Bluetooth Module
void clearUART(void);						//clear UART buffer to ensure all data has been sent/received
void configureADC(void);					//configure analog digital convertor - P1.4 (weight sensor) P1.5 (IR sensor)
void readADC(void);							//read values from weight sensor and IR sensor

int main(void)
{
  /*** Set-up system clocks ***/
  WDTCTL = WDTPW + WDTHOLD;				// Stop WDT
  if (CALBC1_1MHZ == 0xFF)				// If calibration constant erased
  {
    while (1);							// do not load, trap CPU!
  }
  DCOCTL = 0;							// Select lowest DCOx and MODx settings
  BCSCTL1 = CALBC1_1MHZ;				// Set DCO
  DCOCTL = CALDCO_1MHZ;

  /*** Set-up GPIO ***/
  P1SEL = BIT1 + BIT2 + BIT6;			// P1.1 = TXD, P1.2=RXD
  P1SEL2 = BIT1 + BIT2;
  P1DIR |= BIT6 + BIT7;					//P1.6 output for Servo motor, P1.7 output for IR light detection
  P1OUT &= ~BIT7;						//turn off BIT7

  /*** Set-up PWM for Servo ***/
  TACCTL1	= OUTMOD_7;            		// TACCR1 reset/set
  TACTL	= TASSEL_2 + MC_1;     	   		// SMCLK, upmode
  TACCR0	= PWM_Period-1;        		// PWM Period
  TACCR1	= PWM_Duty;            		// TACCR1 PWM Duty Cycle

  /*** Set-up USCI A ***/
  UCA0CTL1 |= UCSSEL_2;					// SMCLK
  UCA0BR0 = 104;						// 1MHz 9600
  UCA0BR1 = 0;							// 1MHz 9600
  UCA0MCTL = UCBRS0;					// Modulation UCBRSx = 1
  UCA0CTL1 &= ~UCSWRST;					// Initialize USCI state machine
  IE2 |= UCA0RXIE;						// Enable USCI_A0 RX interrupt
  __bis_SR_register(LPM0_bits + GIE);	// Enter LPM0 with interrupts enabled

  /*** Set-up ADC Conversion ***/
  configureADC();

  while(1){
    switch (Rx_Data){
    	//Weight Command 'W' = 0x57; Receives signal from pressure sensor and transmits data to BlueSmirf//
    	case 0x57:
    		readADC();						//read analog values from sensors
    		sendFoodSupply(weight);			//send weight value to TX Buffer
    		clearUART();					//clear buffer
    		sendHopperSupply(light);		//send hopper supply value to TX Buffer
    		clearUART();					//clear TX Buffer
		break;

		//Feed Command 'F' = 0x46; Activates servo to dispense food from hopper to bowl//
    	case 0x46:
    		if(!stopMotor()){				//check IR sensor to make sure it's safet to operate motor
    			if(!motorActive){
    				motorActive = 1;		//motorActive flag prevents user from activating motor repeatedly
    				dispenseFood();			//activate motor
    				readADC();
    				sendFoodSupply(weight);
    				clearUART();
    				sendHopperSupply(light);
    				clearUART();
    			}
    			__delay_cycles(3000000);	//delay forces motor to return to 0 and properly refill dispenser with food
    			motorActive = 0;			//reset flag
    		}
		break;
      //additional commands to be added at later time
      default: break;
    }
    __bis_SR_register(LPM0_bits);		// Enter LPM0, interrupts enabled
  }
}

/**
 * Set PWM values to rotate servo motor 180 degrees, delay, then return to original position
 */
void dispenseFood(){
	//rotate 180deg and delay
	TACCR1 = 2500;
	__delay_cycles(3000000);
	//rotate back to 0deg
	TACCR1 = 500;
}

/**
 * Check IR sensors and return true if object is present within 2 feet of dog bowl
 */
int stopMotor(){
	readADC();					//read analog values from sensors
	if(ir > 200){				//if IR detects object in close proximity stop motor for safety
		return 1;
	}
	else{						//else - allow motor operation, no object detected
		return 0;
	}
}

/**
 * Send appropriate message to Bluetooth device
 */
void sendFoodSupply(int weight){
  if(weight < 140){
    UARTSendArray("B0", 2);	//B0 = empty bowl (0%)
  }
  else if(weight < 150){
    UARTSendArray("B25", 3); //B25 = 25%
  }
  else if(weight < 200){
    UARTSendArray("B50", 3); //B50 = 50%
  }
  else if(weight < 220){
    UARTSendArray("B75", 3); //B75 = 75%
  }
  else{
    UARTSendArray("B100", 4); //B100 = 100%
  }
}

void sendHopperSupply(int light){
	if(light > 75){
		UARTSendArray("H0", 2);	//H0 = empty hopper
	}
}

void configureADC(){
   ADC10CTL1 = INCH_5 | CONSEQ_1; 				//A5, A4 and A3, single sequence
   ADC10CTL0 = ADC10SHT_2 | MSC | ADC10ON;
   while (ADC10CTL1 & BUSY);
   ADC10DTC1 = 0x03; 							// 2 conversions - this prevents conversions on A3 A2 and A1
   ADC10AE0 |= (BIT3 + BIT4 + BIT5); 			//Disables digital input on P1.4 and P1.5
}

void readADC(){
	P1OUT |= BIT7;						//turn on BIT7 to check hopper supply
	__delay_cycles(200000);
	weight = 0;
	ir = 0;
	light = 0;
	for(i=1; i<=10 ; i++){
				ADC10CTL0 &= ~ENC;					//ADC10 Enable Conversion
				while (ADC10CTL1 & BUSY);			//Wait while ADC is busy
				ADC10SA = (unsigned)&ADCReading[0]; //RAM Address of ADC Data, must be reset every conversion
				ADC10CTL0 |= (ENC | ADC10SC); 		//Start ADC Conversion
				while (ADC10CTL1 & BUSY); 			//Wait while ADC is busy

				ir += ADCReading[0];
				weight += ADCReading[1];
				light += ADCReading[2];
			}
	ir = ir/10;
	weight = weight/10;
	light = light/10;
	P1OUT &= ~BIT7;		//turn off led

}
/**
 * Clear TX Buffer and push any pending data to Bluetooth module
 */
void clearUART(){
  while (!(IFG2&UCA0TXIFG));		//USCI_A0 TX buffer ready?
  UCA0TXBUF = Tx_Data;			//Send Tx_data
  Tx_Data = 0;					//clear Tx_Data
}

/**
 * Send array of data to Bluetooth module
 */
void UARTSendArray(char *TxArray, int ArrayLength){
  while(ArrayLength--){             //Loop until StringLength == 0 and post decrement
    while(!(IFG2 & UCA0TXIFG));       //Wait for TX buffer to be ready for new data
    UCA0TXBUF = *TxArray++;           //Write the character at the location specified by the pointer and post increment
  }
}

//USCI A interrupt handler
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void){
  Rx_Data = UCA0RXBUF;					//Store RX Data
  Tx_Data = Rx_Data;
  __bic_SR_register_on_exit(LPM0_bits);//Wake-up CPU
}

//TX Interrupt. Use enable/disable to call/cancel interrupt
//UC0IE |= UCA0TXIE;			//Enable USCI_AO TX interrupt
//UC0IE &= ~UCA0TXIE; 			//Disable USCI_A0 TX interrupt
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
  UCA0TXBUF = Tx_Data;		//send data
  Tx_Data = 0;				//clear data
  UC0IE &= ~UCA0TXIE; 		//Disable USCI_A0 TX interrupt
}
