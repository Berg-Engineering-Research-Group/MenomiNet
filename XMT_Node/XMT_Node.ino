/*  
    * UW-Stout: Lakes REU Water Quality Monitor      
    * Authors: Sahi Chundu, Cody Lundquist, Devin Berg Ph.D
    * About: The goal of this project is to create a water quality monitor that is
    inexpensive and relatively simple to build.  Not only does this project 
    work with a smaller budget, it has the additional feature of broadcasting
    the data received over UHF frequency. With some minor adjustments to the code,
    this can be done without the need for an amateur radio operator's license.

    * Inspiration:
      - https://www.fondriest.com/news/data-buoys-study-turbid-water-environments-lake-erie-basin.htm
      - 
    
    * Sources:
      - Temperature: 
      https://lastminuteengineers.com/ds18b20-arduino-tutorial/#:~:text=Wiring%20DS18B20%20Temperature%20Sensor%20to%20Arduino%20If%20you,a%204.7K%20pullup%20resistor%20from%20data%20to%205V.
      - DO2:
      https://wiki.dfrobot.com/Gravity__Analog_Dissolved_Oxygen_Sensor_SKU_SEN0237
      - Turbidity:
      https://how2electronics.com/diy-turbidity-meter-using-turbidity-sensor-arduino/
      - pH:
      https://www.atlas-scientific.com/files/gravity-pH-ardunio-code.pdf
      
    * Considerations for future additions:
      - GPS (Time and Location)
      https://wiki.seeedstudio.com/Grove-GPS-Air530/
      Recommendation:  Seek out an english translated manual
      - Conductance
      - Fluorometer:
      https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3715229/
*/

//Libraries
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "ph_grav.h"



//Pins                     
#define Temp_Pwr 4        // Pins used for power can be freed if moved to power rail.  
#define DO_Pwr 3            // Power consumption is low enough that there isn't
#define Turb_Pwr 5          // much need to remove power from sensors if utilizing
#define PH_Pwr 8            // solar charging batteries.
#define Temp_Pin  A3
#define Pull_Up A5        // Pull Up Resister required to stabalize Temp Input
#define DO_PIN A0
#define Tur_Pin A1                       
Gravity_pH pH = Gravity_pH(A2);   

#define RFM95_CS 10       // Note: Troubleshooting may be required.  Our system experienced
#define RFM95_RST 9         // power output fluctuations on some pins once radio was utilized.  Suspect
#define RFM95_INT 2         // possible interference from RF signals.  Try installing ferrite rings
RH_RF95 rf95(RFM95_CS, RFM95_INT);

OneWire oneWire(Temp_Pin);
DallasTemperature sensors(&oneWire);


//Constants
#define RF95_FREQ 915.0                 // Radio Frequency

#define TWO_POINT_CALIBRATION 1         // DO2 Calibration Mode
  //Single-point calibration Mode=0 || CAL1 ONLY
  #define CAL1_V (3020) //mv
  #define CAL1_T (35.4)   //℃
  //Two-point calibration Mode=1 || CAL 1(High Temp) & CAL 2(Low Temp)
  #define CAL2_V (1880) //mv
  #define CAL2_T (18.6)   //℃

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

const uint8_t bufferlen = 32;   // for pH

//Data Storage
float value;
int temprdg;
int do2rdg;
int turbrdg;
int phrdg;
int radiopacket[4];
uint8_t user_bytes_received = 0;                                 
char user_data[bufferlen];  

void setup()
{ 
  sensors.begin();                                // Start up temperature library
  Serial.begin(9600);
  pinMode(RFM95_RST, OUTPUT);                     // Start up radio
  digitalWrite(RFM95_RST, HIGH);
  while (!Serial);              
  delay(100);
  // Serial.println("Arduino LoRa TX Test!");
  digitalWrite(RFM95_RST, LOW);                   // Perform radio check
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (!rf95.init())                            // time-out if radio cannot be detected
    {
    //Serial.println("LoRa radio init failed");
    while (1);
    }
  //Serial.println("LoRa radio init OK!");
  if (!rf95.setFrequency(RF95_FREQ))              // time-out if radio cannot be set to 915MHz
    {
    //Serial.println("setFrequency failed");
    while (1);
    }
  //Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
  pinMode(Pull_Up, INPUT_PULLUP);                 // Designated pull-up resister
  pinMode(Temp_Pwr, OUTPUT);
  pinMode(DO_Pwr, OUTPUT);
  pinMode(Turb_Pwr, OUTPUT);
  pinMode(PH_Pwr, OUTPUT);

  // pH Test && Calibrate
  digitalWrite(PH_Pwr, HIGH);                            
  delay(200);
  //Serial.println(F("Use commands \"CAL,7\", \"CAL,4\", and \"CAL,10\" to calibrate the circuit to those respective values"));
  //Serial.println(F("Use command \"CAL,CLEAR\" to clear the calibration"));
  //if (pH.begin()) {Serial.println("Loaded EEPROM");}
  digitalWrite(PH_Pwr, LOW);
  delay(2000);
}

int16_t packetnum = 0;

void loop()
{
  // Power ON
  digitalWrite(Temp_Pwr, HIGH);
  digitalWrite(DO_Pwr, HIGH);
  digitalWrite(Turb_Pwr, HIGH);
  digitalWrite(PH_Pwr, HIGH);
  delay(1500);


  
  // Temperature
  sensors.requestTemperatures();
  temprdg = 100 * (sensors.getTempCByIndex(0));           // Results need to be stored in float and divided by 100 by the receiver.
  //Serial.println("Temperature:\t" + String((sensors.getTempCByIndex(0))) + "C  |  " + String((9 * (sensors.getTempCByIndex(0)) / 500) + 32) + "F");
  digitalWrite(Temp_Pwr, LOW);



  //DO
  do2rdg = readDO((uint32_t(5000) * analogRead(DO_PIN) / 1024), temprdg);
  //Serial.println("DO:\t\t" + String(uint32_t(5000) * analogRead(DO_PIN) / 1024) + "mV\t|  " + String(do2rdg));
  digitalWrite(DO_Pwr, LOW);
  

  
  // Turbidity
  value = 0;
  for(int i=0; i<1000; i++)
    {
    value = value + ((int)analogRead(Tur_Pin)*(5/1024.0));
    }
  value = value/1000;
  value = round_to_dp(value,2);
  //Serial.print("Turbidity:\t" + String(value) + "V\t" + "|  ");  
  if(value > 2.5)
    {
    turbrdg = -52.4*square(value)+274.3*value-235.8;    // This equation needs to be worked from scratch if utilizing less than 5V.
    }                                                      // Use turbidity standards and read in voltages.  Graph Turbidity vs Voltage and
  else                                                     // find equation of slope.  If utilizing 5V, refer to turbidity source code to replace
    {                                                      // with original equation.
    turbrdg = 3000;                   // Set max turbidity rating
    }
  //Serial.print(String(turbrdg) + "NTU\n");
  digitalWrite(Turb_Pwr, LOW);



  // pH
  if (Serial.available() > 0)         // Calibration command read.  Commands listed in void Setup()
  {                                                      
    user_bytes_received = Serial.readBytesUntil(13, user_data, sizeof(user_data));   
  }

  if (user_bytes_received) 
  {                                                      
    parse_cmd(user_data);                                                          
    user_bytes_received = 0;                                                        
    memset(user_data, 0, sizeof(user_data));                                         
  }

  phrdg = 100 * pH.read_ph();         // Results need to be stored in float and divided by 100 by the receiver
  //Serial.println("pH:\t\t" + String(pH.read_ph()));
  digitalWrite(PH_Pwr, LOW);
  delay(500);
  


  // Send the data
  //Serial.println("Transferring to rf95_server");
  radiopacket[0] = temprdg;
  radiopacket[1] = do2rdg;
  radiopacket[2] = turbrdg;
  radiopacket[3] = phrdg;
  //Serial.print("Transferring: "); 
  //Serial.println(String(String(radiopacket[0])+"\t"+String(radiopacket[1])+"\t"+String(radiopacket[2])+"\t"+String(radiopacket[3])));  
  delay(10);
  xmt(radiopacket);
  delay(1000);
}



// Round Floats
// Send data here if you decide to utilize floats and need them rounded.
float round_to_dp( float in_value, int decimal_place )
  {
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
  }

// DO Voltage to Measurement conversion.  Results given in mg/L
int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
  {
  #if TWO_POINT_CALIBRATION == 0
    uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
    return (voltage_mv * DO_Table[temperature_c] / V_saturation);
  #else
    uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
    return (voltage_mv * DO_Table[temperature_c] / V_saturation);
  #endif
  }



// Integer to Character Array
// Unused in current configuration.  If data needs to be sent as a character array,
// can send integers here for conversion.
char* Array_print(int N)
{
 
    // Count digits in number N
    int m = N;
    int digit = 0;
    while (m) {
 
        // Increment number of digits
        digit++;
 
        // Truncate the last
        // digit from the number
        m /= 10;
    }
 
    // Declare char array for result
    char* arr;
 
    // Declare duplicate char array
    char arr1[digit];
 
    // Memory allocation of array
    arr = (char*)malloc(digit);
 
    // Separating integer into digits and
    // accommodate it to character array
    int index = 0;
    while (N) {
 
        // Separate last digit from
        // the number and add ASCII
        // value of character '0' is 48
        arr1[++index] = N % 10 + '0';
 
        // Truncate the last
        // digit from the number
        N /= 10;
    }
 
    // Reverse the array for result
    int i;
    for (i = 0; i < index; i++) {
        arr[i] = arr1[index - i];
    }
 
    // Char array truncate by null
    arr[i] = '\0';
 
    // Return char array
    return (char*)arr;
}



// Radio Commands
void xmt(int* message)
{
  //Serial.println("Broadcasting: " + String(String(message[0])+"\t"+String(message[1])+"\t"+String(message[2])+"\t"+String(message[3])));
  rf95.send((uint8_t *)message, 20);      // Send the array and the transmission size to the radio.
  //Serial.println("Waiting for packet to complete..."); delay(100);
  rf95.waitPacketSent();                  // Do not proceed until transmission has finished.
}
// Receive currently unused.  Intended to modify then use for poll based transmission. 
void rcv()
{
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  delay(10);
  
  if (rf95.waitAvailableTimeout(1000))
  {
    if (rf95.recv(buf, &len))
    {
      Serial.print("Received: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    
    else
    {
      //Serial.println("Receive failed");
    }
  }
  
  else
  {
    //Serial.println("Nothing Heard");
  }
  delay(1000);
} 



// pH Commands
void parse_cmd(char* string) {                   
  strupr(string);                                
  if (strcmp(string, "CAL,7") == 0) {       
    pH.cal_mid();                                
    Serial.println("MID CALIBRATED");
  }
  else if (strcmp(string, "CAL,4") == 0) {            
    pH.cal_low();                                
    Serial.println("LOW CALIBRATED");
  }
  else if (strcmp(string, "CAL,10") == 0) {      
    pH.cal_high();                               
    Serial.println("HIGH CALIBRATED");
  }
  else if (strcmp(string, "CAL,CLEAR") == 0) { 
    pH.cal_clear();                              
    Serial.println("CALIBRATION CLEARED");
  }
}
