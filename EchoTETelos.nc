interface EchoTETelos
{
command error_t read();
event error_t readDone(error_t result ,soil_reading_t reading);
}
