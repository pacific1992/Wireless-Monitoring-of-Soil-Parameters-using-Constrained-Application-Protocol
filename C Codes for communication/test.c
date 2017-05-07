
#include "contiki.h"
#include "dev/leds.h"
#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{
  PROCESS_BEGIN();
while(1)
{
PROCESS_WAIT_EVENT();
  printf("Hello, world\n");
leds_toggle(LEDS_RED);
 } 
  PROCESS_END();
}
