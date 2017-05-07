module SoilProbeP 
{
  uses 
  {
    interface Boot;
    interface Timer<TMilli>;
    interface Leds;
    interface Resource;
    interface UartStream;
  }
  provides
  {
    interface Msp430UartConfigure;
    interface Read;
  }
}
implementation 
{



  msp430_uart_union_config_t msp430_uart_115200_config = {
    {
      utxe : 1,
      urxe : 1,
      ubr : UBR_1MHZ_115200,
      umctl : UMCTL_1MHZ_115200,
      ssel : 0x02,
      pena : 0,
      pev : 0,
      spb : 0,
      clen : 1,
      listen : 0,
      mm : 0,
      ckpl : 0,
      urxse : 0,
      urxeie : 1,
      urxwie : 0,
      utxe : 1,
      urxe : 1
    }
  };


  uint8_t data;

  void report_success() { call Leds.led1Toggle(); }
  void report_problem() { call Leds.led0Toggle(); }

  event void Boot.booted()
   {
     call Timer.startPeriodic(1000);
   }


   task void sendDoneSuccessTask()
  {
    call Resource.release();
    report_success();
  }



  task void sendDoneFailTask()
  {
    call Resource.release();
    report_problem();
  }





  command error_t Read.read()
  {
    return call Resource.request();
  }



  
  event void Resource.granted(){ call Leds.led2Toggle(); }
  
  async command msp430_uart_union_config_t* Msp430UartConfigure.getConfig()
  {
    return &msp430_uart_115200_config;
  }

 

    async event void UartStream.receivedByte(uint8_t value)
  {
 
     data = value;
     

  }



  async event void UartStream.receiveDone(uint8_t* buf, uint16_t len, error_t error)
  {
    
     if(error == SUCCESS)
      post sendDoneSuccessTask();
      else
      post sendDoneFailTask();
    

  }




  async event void UartStream.sendDone(uint8_t *buf, uint16_t len, error_t error)
  {

  }

   event void Timer.fired(){
   
    
  
  }

}
