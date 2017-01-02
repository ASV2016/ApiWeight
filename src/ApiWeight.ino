// API WEIGHT // adesaintvenant@gmail.com // 2016-2017 //
// Merci � Objenious & Bouygues Telecom //
//Hardware : TheAirBoard.cc ; DHT11 ; //


// ---------------------------------------------------------------------
// Include
// ---------------------------------------------------------------------

#include <Arduino.h>
#include <dht11.h>
#include <HX711.h>
#include <TheAirBoard.h>
#include <arm.h> //ATIM library for LoRaWAN connection
#include <SoftwareSerial.h> // More information at https://www.arduino.cc/en/Reference/SoftwareSerial

// We define the pins for the software Serial that will allows us to
// debug our code.
SoftwareSerial mySerial(10, 11); // Pin 10 will work as TX and Pin 11 as RX

// ---------------------------------------------------------------------
// Define
// ---------------------------------------------------------------------

#define DHT11PIN  3
#define PHOTOCELL 4
#define GREEN	    5               // GREEN dimmable LED
#define BLUE	    6               // BLUE dimmable LED
#define RED       9               // RED dimmable LED
#define HX711SCK  12
#define HX711DT   13
#define VBAT      A7

// ---------------------------------------------------------------------
// Global variables
// ---------------------------------------------------------------------
dht11 DHT11;
TheAirBoard board;
Arm Objenious;
HX711 scale(HX711DT,HX711SCK);

volatile boolean f_wdt = true;
int timeout = 0;

byte msgg[7];

int count = 1;
int Temperature;
int Humidity;
int Luminosity;
int Weight;
float Battery;

// ---------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------

void setup()
{
  mySerial.begin(9600);
  delay(8000);
  board.setWatchdog(8000);
  mySerial.println("Setup OK");

 //Setup of scale
 scale.set_scale(2280.f);                      // this value is obtained by calibrating the scale with known weights; see the README for details
 scale.tare();	 // This line reset the reading to 0


  //Init of the LoRaWAN module - Red light if error, Green light if Ok
  if(Objenious.Init(&Serial) != ARM_ERR_NONE)
  {
      digitalWrite(RED, HIGH);
      mySerial.println("Network Error"); // Debug
  }
  else
  {
      digitalWrite(GREEN, HIGH);
      mySerial.println("Connected to Objenious"); // Debug
  }

  // Configuration of the LoRaWAN module
  Objenious.SetMode(ARM_MODE_LORAWAN);

  Objenious.LwEnableRxWindows(true);
  Objenious.LwEnableTxAdaptiveSpeed(true);
  Objenious.LwEnableDutyCycle(true);
  Objenious.LwEnableTxAdaptiveChannel(true);
  Objenious.LwEnableRx2Adaptive(true);

  Objenious.LwEnableOtaa(true);

  //Apply the configuration to the module.
  Objenious.UpdateConfig();

  delay(8000); // delay needed for the module to connect to Objenious

  // We turn the LED OFF after 8 seconds
  digitalWrite(GREEN, LOW);
  digitalWrite(RED, LOW);
//  msgg[0]=0; //define kind of sketch to be used by Objenious platform
// ------------------------------

}



void loop()
{
  if(f_wdt == true) {            // on watchdog expire (every 8 seconds)
    timeout++;
    if(timeout == 2) {           // timeout every 7*8 = 56 seconds

      int chk = DHT11.read(DHT11PIN);
      mySerial.print(count);
      mySerial.print(" - Sensor: ");
      switch (chk)
      {
        case DHTLIB_OK:
          mySerial.println("OK");
        break;
        case DHTLIB_ERROR_CHECKSUM:
          mySerial.println("Checksum error");
        break;
        case DHTLIB_ERROR_TIMEOUT:
          mySerial.println("Time out error");
        break;
        default:
          mySerial.println("Unknown error");
        break;
      }

      Humidity = DHT11.humidity;
      Temperature = DHT11.temperature;
      Luminosity = analogRead(PHOTOCELL)/8; // Divided by 8 For 0 -> 255 range
      Weight = -scale.get_units(5)*10;
      Battery = (3.3*analogRead(VBAT)/1024 + 1.2)*10; // Battery x10 (en dV)

      mySerial.print(Temperature);
      mySerial.print(Humidity);
      mySerial.print(Luminosity);
      mySerial.print(Weight);
      mySerial.println(Battery);

      LedDisplay(Weight);

      msgg[0] = 0;
      msgg[1] = (byte) Temperature;
      msgg[2] = (byte) Humidity;
      msgg[3] = (byte) Luminosity;
      msgg[4] = (byte) (Weight>>8);
      msgg[5] = (byte) Weight;
      msgg[6] = (byte) Battery;

      mySerial.print(msgg[1]);
      mySerial.print(msgg[2]);
      mySerial.print(msgg[3]);
      mySerial.print(msgg[4]);
      mySerial.print(msgg[5]);
      mySerial.println(msgg[6]);

      Objenious.Send(msgg, sizeof(msgg));               // Send the msgg to Objenious network
      mySerial.println("Msgg sent");
      Blink(GREEN,100);

      //LedDisplay(count);
      count++;
          timeout=0;
    }
  f_wdt = false;
  board.powerDown();
  }
}


ISR(WDT_vect) {f_wdt = !f_wdt;} //Interupt instruction for deep sleep


void LedDisplay(int Valeur)
{
/*  Blink(RED,100);
  Blink(BLUE,100);
  Blink(GREEN,100);
  delay(1000);
*/
  for (int i=0; i<Valeur/100;i++ )
  {
    Blink(RED,300);
  }
  for (int i=0; i<(Valeur%100)/10;i++ )
  {
    Blink(BLUE,300);
  }
  for (int i=0; i<(Valeur%10);i++)
  {
    Blink(GREEN,300);
  }
}

void Blink(byte PIN, int DELAY_MS)
{
  digitalWrite(PIN,120);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
  delay(DELAY_MS);
}
