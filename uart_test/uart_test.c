#include<msp430.h>

#include<stdio.h>

#include "contiki.h"

#include "isr_compat.h"

#include "dev/leds.h"

 void uart0_init()
{
       
        P3SEL |= 0x30;
        P3DIR &= ~0x20;                      
        P3DIR |= 0x10;                       
        P3OUT |= 0x10;                       
        ME1 |= 0xc0;
        U0CTL =0x10;
        U0TCTL |= 0x00;
        U0RCTL=0x00;
        U0BR0=0x03;
        U0BR1=0x00;
        U0MCTL=0x4a;
        _BIS_SR(GIE);
        IE1|=UTXIE0;
      
       
}


ISR(UART0RX,uart0_rx_interrupt)
{
     printf("%d",3);
        


        if(!(IFG1 & URXIFG0))
        {
 printf("%d\n",2);
           U0TCTL &= ~URXSE;
           U0TCTL |= URXSE;
                leds_toggle(LEDS_BLUE);
                printf("recbuf=%c\n",RXBUF0);
        }

        else{
                        

                          
                                   

                                                leds_on(LEDS_GREEN);
                                       
                                       
                                              
                                                      
                                        IFG1 &=~URXIFG0;
        }

}


ISR(UART0TX,uart0_tx_interrupt)
{
          
           printf("%d",5);
         
          while(!(IFG1 & UTXIFG0));  
              TXBUF0=0X41;
 printf("TUXbuf=%c\n",TXBUF0);
     
              
 
}


PROCESS(uart_check,"uart-communication");
AUTOSTART_PROCESSES(&uart_check);

PROCESS_THREAD(uart_check,ev,data)
{
PROCESS_BEGIN();
uart0_init();


PROCESS_END();
}
