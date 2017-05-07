#include<msp430.h>

#include<stdio.h>

#include "contiki.h"

#include "isr_compat.h"

#include "string.h"
#include "dev/leds.h"


char dest[20];
void uart0_send_data()
{	 
printf("entered send fn");
		char c[ ]="My name is girish";
		
		
		int j,l;
			
		printf("Transmitted string:\t");
		printf("%s",c);
		
		for(j=0;c[j]!='\0';j++)
		{
		
		TXBUF0 = c[j];
		for(l=0;l<5000;l++)
		_NOP();
		}
		
		printf("\nthe received string is:\t");
		printf("%s",dest);
		printf("\nexit send data\n");

}
void uart0_init()
{	
	
	printf("init entered\n");
	WDTCTL=WDTPW+WDTHOLD;
	
        P3SEL |= 0x30;
        P3DIR &= ~0x20;                      
        P3DIR |= 0x10;                       
        P3OUT |= 0x10;                       
        ME1 |= UTXE0 + URXE0;
        U0CTL = CHAR+LISTEN;
        U0TCTL = SSEL0;
	U0RCTL = 0;
        U0BR0=0x03;
        U0BR1=0x00;
        U0MCTL=0x4a;
        U0CTL &= ~SWRST;
      
	IE1 = URXIE0+UTXIE0;
	
	
	_BIS_SR(GIE);
	printf("init exit\n");
	
     
}

ISR(UART0RX,uart0_rx_interrupt)
{
 	
	//static int k=0;
	//int i;
	//for(i=0;i<10000;i++)
	//_NOP();
	
	//if(RXBUF0 != 0x0A)

	//{
		
		printf("%c",RXBUF0);		
		//dest[k-1]=RXBUF0;
		//k++;
		
	//}
	//printf("%s",dest);
	
	IFG1 &= ~URXIFG0;
	return;

}



PROCESS(uart_check,"uart-communication");
AUTOSTART_PROCESSES(&uart_check);

PROCESS_THREAD(uart_check,ev,data)
{
PROCESS_BEGIN();
uart0_init();
while(1)
leds_toggle(LEDS_BLUE);
uart0_send_data();
PROCESS_END();
}


