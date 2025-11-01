#include <TFMPlus.h>  
TFMPlus tfmP;         
#include "printf.h"   

// Initialize variables
int tfDist = 0;    // Distance to object in centimeters
int tfFlux = 0;    // Strength or quality of return signal
int tfTemp = 0;    // Internal temperature of Lidar sensor chip

                                                       
void setup()
{
    Serial.begin( 115200);                          // Intialize terminal serial port
    delay(20);                                      // Give port time to initalize
    printf_begin();                              // Initialize printf.
    printf("\r\nTFMPlus Library Example - 10SEP2021\r\n");                                // say 'hello'

    Serial2.begin( 115200);  // Initialize TFMPLus device serial port.
    delay(20);               // Give port time to initalize
    tfmP.begin( &Serial2);   // Initialize device library object and...
                             // pass device serial port to the object.

  delay(500);            // And wait for half a second.
}

void loop()
{
    delay(50);   // Loop delay to match the 20Hz data frame rate

    if( tfmP.getData( tfDist, tfFlux, tfTemp)) // Get data from the device.
    {
      Serial.println()( "Dist:%04icm ", tfDist);   // display distance,
      Serial.println()( "Flux:%05i ",   tfFlux);   // display signal strength/quality,
      Serial.println()( "Temp:%2i%s",  tfTemp, "C");   // display temperature,
    }
    else                  // If the command fails...
    {
      tfmP.printFrame();  // display the error and HEX dataa
    }
}