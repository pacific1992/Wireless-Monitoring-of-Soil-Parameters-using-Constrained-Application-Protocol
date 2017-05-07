#include<msp430.h>

#include<stdio.h>

#include "contiki.h"

#include "isr_compat.h"

#include "string.h"



char dest[20];						//Global Variable

void uart0_send_data()
{	 
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
	
	
	WDTCTL=WDTPW+WDTHOLD;			//Stop Watch Dog Timer
	
        P3SEL |= 0x30;				//Select UART0 Transmit and Receive

        P3DIR &= ~0x20;                      	//Make URXD0 as input

        P3DIR |= 0x10;                       	//make UTCD0 as output

        P3OUT |= 0x10;                       	
        ME1 |= UTXE0 + URXE0;			//module enable 

        U0CTL = CHAR;				//Select 8 bit transceive mode

        U0TCTL = SSEL0;				//ACLK = UCLK
        U0RCTL |= 0x00;
        U0BR0=0x1b;				//Baud rate register
        U0BR1=0x00;
        U0MCTL=0x24;				//Modulation control register
        U0CTL &= ~SWRST;			
      
	IE1 = URXIE0+UTXIE0;			//Enable receive and transmit interrupt
	
	
	_BIS_SR(GIE);
	
	
     
}

ISR(UART0RX,uart0_rx_interrupt)
{
 	
	static int k=0;
	int i;
	for(i=0;i<5000;i++)
	_NOP();
		
	printf("%c",RXBUF0);		
	dest[k-1]=RXBUF0;
	k++;
	
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


