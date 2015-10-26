/*Justin Johnson - FAU 2015
 *MSP430 communication with BlueSmirf Bluetooth Module
 *Micro remains in low power mode until interrupt is triggered by incoming data
 *Commands: W = measures weight (W) sensor and sends value to bluetooth module for to provide user feedback
 *Commands: F = flashes LED P1.6, motor to be added to dispense food (F) to dog bowl
 *baudrate = 9600
 *TXD = P1.1
 *RXD = P1.2
 *
 */
#include <msp430g2253.h>

unsigned char Rx_Data = 0;					//temp RX variable
unsigned char Tx_Data = 0;					//temp TX variable
int weight = 0;								//stores weight of food supply
int count = 0;								//Timer A
int motorActive = 0;

void sendFoodSupply(int);
void UARTSendArray(char *TxArray, int ArrayLength);
void clearUART(void);

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
  P1SEL = BIT1 + BIT2;					// P1.1 = TXD, P1.2=RXD
  P1SEL2 = BIT1 + BIT2;
  P1DIR |= (~BIT0); 					// set bit 4 as input
  P1DIR |= BIT6;						// P1.6 set as output
  P1OUT &= ~(BIT6);						// P1.6 OFF

  /*** Set-up USCI A ***/
  UCA0CTL1 |= UCSSEL_2;					// SMCLK
  UCA0BR0 = 104;						// 1MHz 9600
  UCA0BR1 = 0;							// 1MHz 9600
  UCA0MCTL = UCBRS0;					// Modulation UCBRSx = 1
  UCA0CTL1 &= ~UCSWRST;					// Initialize USCI state machine
  IE2 |= UCA0RXIE;						// Enable USCI_A0 RX interrupt
  __bis_SR_register(LPM0_bits + GIE);	// Enter LPM0 with interrupts enabled

  while(1){
    switch (Rx_Data){
      case 0x57:						//If Weight Command 'W'
      TA0CCTL0 &= ~CCIE;				//Disable Timer0_A interrupts
      __delay_cycles(200);
      int i = 0;
      for(i=1; i<=10 ; i++){			//sample weight 10 times
        ADC10CTL1 |= CONSEQ1;		// Set conversion to single channel and continuous-sampling mode
        ADC10CTL0 |= ADC10SHT_2 + ADC10ON + MSC;   //Set S/H time, 10-bit converter, and continuous sampling:
        ADC10AE0 |= 1;				//Choose P1.0 (channel 1) as an analog input pin:
        ADC10CTL0 |= ADC10SC + ENC;// Start A/D conversion; The result will appear in the memory variable "ADC10MEM"
        weight += ADC10MEM;		//store weight sensor reading
      }
      weight = weight/10;				//average weight
      sendFoodSupply(weight);			//send weight value to TX Buffer
      clearUART();						//clear TX Buffer
      break;

      case 0x46:					//If Feed Command 'F', Rotate Motor
    	  motorActive = !motorActive;
    	  if(motorActive){
    	      /*** Timer0_A Set-Up ***/		//FLASHING LEDS symbolizing motor rotation
    	      TA0CCR0 |= 10000-1;			// Counter value
    	      TA0CCTL0 |= CCIE;				// Enable Timer0_A interrupts
    	      TA0CTL |= TASSEL_2 + MC_1;	// ACLK, Up Mode (Counts to TA0CCR0)
    	      /*** Timer0_A Set-Up ***/
    	  }
    	  else{
    	      TA0CCTL0 &= ~CCIE;				//Disable Timer0_A interrupts
    		  P1OUT &= ~(BIT6);				// P1.6 OFF
    	  }
      break;

      //additional commands to be added at later time

      default: break;
    }
    __bis_SR_register(LPM0_bits);		// Enter LPM0, interrupts enabled
  }
}

/**
 * Send appropriate message to Bluetooth device
 */
void sendFoodSupply(int weight){
  if(weight < 5){
    UARTSendArray("Empty Bowl", 10);
  }
  else if(weight < 8){
    UARTSendArray("10 %", 4);
  }
  else if(weight < 12){
    UARTSendArray("20 %", 4);
  }
  else if(weight < 15){
    UARTSendArray("30 %", 4);
  }
  else if(weight < 18){
    UARTSendArray("40 %", 4);
  }
  else if(weight < 22){
    UARTSendArray("50 %", 4);
  }
  else if(weight < 26){
    UARTSendArray("60 %", 4);
  }
  else if(weight < 29){
    UARTSendArray("70 %", 4);
  }
  else if(weight < 32){
    UARTSendArray("80 %", 4);
  }
  else if(weight < 35){
    UARTSendArray("90 %", 4);
  }
  else{
    UARTSendArray("Full Bowl", 9);
  }
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


//Timer0 A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0 (void) {
	count++;
	if (count == 20){
		P1OUT ^= BIT6;					//Toggle Green LEDs
		count =0;
	}
}
