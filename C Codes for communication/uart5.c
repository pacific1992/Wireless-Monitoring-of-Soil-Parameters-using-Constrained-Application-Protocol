#include<msp430.h>

#include<stdio.h>

#include "contiki.h"

#include "isr_compat.h"

#include "string.h"



char dest[30];

void uart0_send_data()
{	 
		char c[ ]="My name is girish";
		
		
		int j,l;
			
		printf("Transmitted string:\t");
		printf("%s",c);
		
		for(j=0;c[j]!='\0';j++)
		{
		TXBUF0 = c[j];
		for(l=0;l<1000;l++)
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
        U0CTL = 0x10;
        U0TCTL = 0x93;
        U0RCTL |= 0x08;
        U0BR0=0x03;
        U0BR1=0x00;
        U0MCTL=0x4a;
        U0CTL &= ~SWRST;
      
	IE1 = 0xc0;
	
	
	_BIS_SR(GIE);
	printf("init exit\n");
	
     
}

ISR(UART0RX,uart0_rx_interrupt)
{
 	
	static int k=0;
	
	
	
	if(RXBUF0 != 0x0A)
	{
		printf("%c",RXBUF0);		
		dest[k]=RXBUF0;
		k++;
		
	}
	
	
	IFG1 &= ~URXIFG0;
	return;

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


