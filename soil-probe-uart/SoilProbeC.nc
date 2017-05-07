

configuration SoilProbeC
{
  provides interface Read;
}
implementation 
{
  components LedsC, MainC;
  components new TimerMilliC();
  components SoilProbeP as App;
  components new Msp430Uart0C() as UartC;
  components HplMsp430GeneralIOC;
  components new Msp430GpioC();
  


  App.Boot -> MainC;
  App.Timer -> TimerMilliC;
  App.Leds -> LedsC;
  App.Resource -> UartC.Resource;
  App.UartStream -> UartC.UartStream;
  UartC.Msp430UartConfigure -> App.Msp430UartConfigure;
  Msp430GpioC.HplGeneralIO  -> HplMsp430GeneralIOC.ADC0;
  Read = App;
}
