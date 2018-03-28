
#include "io430.h"
int a,b,c,d, new_pos =1000,pos=0;
void find_dist();

void putc(unsigned char c)
{
  while (!IFG2_bit.UCA0TXIFG);{
  UCA0TXBUF = c;}
}

unsigned char getc(void)
{
 while (!IFG2_bit.UCA0RXIFG);
return (UCA0RXBUF);
}

void main( void )
{
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;
  BCSCTL1 = CALBC1_16MHZ;
  DCOCTL = CALDCO_16MHZ;
  P2DIR=0xFF;
  //P1DIR_bit.P0 = 1;
  //P1OUT_bit.P0 = 0;
  __enable_interrupt();
  //initialize timer 1
  TA1CTL_bit.TASSEL1 = 1;
  TA1CTL_bit.TASSEL0 = 0;
  TA1CTL_bit.MC1 = 0;
  TA1CTL_bit.MC0 = 1;
  TA1CTL_bit.ID1 = 1;
  TA1CTL_bit.ID0 = 0;
  TA1CCTL0=CCIE;
  TA1CCR0=20000;
  // Initialize timer 0
  P1IFG &= ~BIT3;
  P1IE |= BIT3;
  TA0CTL_bit.TASSEL1 = 1;
  TA0CTL_bit.TASSEL0 = 0;
  TA0CTL_bit.MC1 = 1;
  TA0CTL_bit.MC0 = 0;
  TA0CTL_bit.ID1 = 1;
  TA0CTL_bit.ID0 = 1;
  P2OUT =1;
  P1SEL |= BIT1 + BIT2;
  P1SEL2 |=BIT1+BIT2;
  UCA0CTL1 |=UCSSEL1;
  UCA0BR1 = 0;
  UCA0BR0 = 103;
  UCA0CTL1 &= ~UCSWRST;
  
  while(1){
    new_pos = getc()*3;
    //find_dist();
    //__delay_cycles(300000);
    
  }
}
/*
#pragma vector=PORT1_VECTOR
__interrupt void bit3(void) { 
  int buffer;
  TA0R = 0;
  while(P1IN_bit.P3){
  buffer = TA0R;
  }
  putc(buffer);
  putc(buffer >>8);
  P1IFG_bit.P3 = 0;
}
*/

#pragma vector=TIMER1_A0_VECTOR
__interrupt void timer2(void) {
  if (pos < new_pos){
    a = 1; b = 2; c = 4; d = 8;

   if(P2OUT==a)
      P2OUT =a+b;
   else if (P2OUT == a+b)
     P2OUT = b;
   else if (P2OUT == b)
     P2OUT = b+c;
   else if (P2OUT == b+c)
     P2OUT = c;
   else if (P2OUT == c)
     P2OUT = c+d;
   else if (P2OUT==c+d)
     P2OUT =d;
   else if (P2OUT== d)
     P2OUT= d+a;
   else if (P2OUT == d+a)
     P2OUT = a;
   pos++;
  }
   else if (pos > new_pos){
    a = 8; b = 4; c = 2; d = 1;

   if(P2OUT==a)
      P2OUT =a+b;
   else if (P2OUT == a+b)
     P2OUT = b;
   else if (P2OUT == b)
     P2OUT = b+c;
   else if (P2OUT == b+c)
     P2OUT = c;
   else if (P2OUT == c)
     P2OUT = c+d;
   else if (P2OUT==c+d)
     P2OUT =d;
   else if (P2OUT== d)
     P2OUT= d+a;
   else if (P2OUT == d+a)
     P2OUT = a;
   pos--;
  }
   else;
  TA1R=0;
}


void find_dist() {
  int buffer, i=0; 
  P1OUT_bit.P0 = 1;
  __delay_cycles(20);
  P1OUT_bit.P0 = 0;
  while(!P1IN_bit.P3) {
    TA0R = 0;
    i++;
    if (i>30000){
      break;
    }
  }
  while(P1IN_bit.P3) {
    buffer = TA0R;
  }
  putc(buffer);
  putc(buffer >>8);
}
