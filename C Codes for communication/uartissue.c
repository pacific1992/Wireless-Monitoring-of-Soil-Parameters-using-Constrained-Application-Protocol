#include<msp430.h>

#include<stdio.h>

#include "contiki.h"

#include "isr_compat.h"
#include "string.h"
#include "dev/leds.h"

 void uart0_init()
{
	printf("entered init\n");
	WDTCTL=WDTPW+WDTHOLD;
        P3SEL |= 0x30;
        P3DIR &= ~0x20;                       
        P3DIR |= 0x10;                        
        P3OUT |= 0x10;                        
        ME1 |= URXE0 + UTXE0;
        U0CTL |=CHAR+LISTEN;
        U0TCTL |= SSEL0;
        U0RCTL=0x00;
        U0BR0=0x03;
        U0BR1=0x00;
        U0MCTL=0x4a;
	printf("a\n");
        U0CTL &= ~SWRST;
	printf("b\n");
       IE1 |=URXIE0 +UTXIE0;
	printf("c\n");
  //      _BIS_SR(GIE);
	printf("exit init");
	printf("exit init");
}


ISR(UART0RX,uart0_rx_interrupt)

{	
	printf("entered isr\n");
        unsigned char c;
	int rx_in_progress;
        if(!(IFG1 & URXIFG0))
        {
	printf("%d\n",IFG1);
	printf("%d\n",URXIFG0);
           U0TCTL &= ~URXSE;
	printf("%d\n",U0TCTL);
           U0TCTL |= URXSE;
	printf("%d\n",U0TCTL);
                rx_in_progress = 1;
                leds_toggle(LEDS_BLUE);
	printf("D\n");
                printf("%c\n",RXBUF0);
	printf("exit if loop\n");
        }

        else{
                        rx_in_progress = 0;

                            if(URCTL0 & RXERR )
	{		
		printf("%d\n",RXERR);
        
                                       leds_on(LEDS_GREEN);
                                        c=RXBUF0;
                                        }
                                                else{

                                                leds_on(LEDS_GREEN);
                                                        c=RXBUF0;
                                                      }
                                        IFG1 &=~URXIFG0;
        }

}


void uart0_send_data()
{
               while(!(IFG1 & UTXIFG0));
               TXBUF0 = 0x41;
               while(!(IFG1 & UTXIFG0));
               TXBUF0 = 0x42;
  
}


PROCESS(uart_check,"uart-communication");
AUTOSTART_PROCESSES(&uart_check);

PROCESS_THREAD(uart_check,ev,data)
{
PROCESS_BEGIN();
uart0_init();
uart0_send_data();
PROCESS_END();
}
