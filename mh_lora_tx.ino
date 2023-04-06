/*
  Temperature sensor to RESTDB.io collection
  LoRa WAN version (part 1):
  Temperature sensor via LoRa WAN MKR 1300 comms
  By: Erik Harg <erik@harg.no>

*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_SleepyDog.h>
#include <CRC32.h>
#include "TimeLib.h"

// Initialize Dallas Temperature sensors
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Timekeeping
unsigned long sendData = 0;         // next time we'll send data
unsigned long SEND_WAIT = 5;     // how long to wait between submissions -- 1800 = 30 min
unsigned long LOOP_WAIT_MS = 5000;  // how long to wait between loops -- 2000 ms = 2 sec
unsigned long lastLoopMillis = 0; // time of last loop execution

// Storage between loops
int counter = 0;
String readBuffer = "";

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  int countdownMS = Watchdog.enable(16000); // 16s is max timeout
  
  Serial.begin(9600);
  Serial.print("Enabled the watchdog with max countdown of ");
  Serial.print(countdownMS, DEC);
  Serial.println(" milliseconds!");
    
  delay(5000);

  Serial.println("LoRa Sender");

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Watchdog.reset();
  
  LoRa.enableCrc();
  Serial.println("LoRa started - starting service...");
  
  sensors.begin();
  Serial.println("Service started");
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  unsigned long currentMillis = millis();
  if(currentMillis - lastLoopMillis >= LOOP_WAIT_MS)
  {
    Watchdog.reset();
    time_t ticktime = now();

    digitalWrite(LED_BUILTIN, HIGH);
    // we should send data
    if (ticktime > sendData)
    {
      Serial.print("Send data at ");
      Serial.print(formatDateTime(ticktime) + "\n");
      sendDataNow();
    }
    Serial.println("Loop done");
  
    Watchdog.reset();
    digitalWrite(LED_BUILTIN, LOW);
    lastLoopMillis = millis(); // set the timing for the next loop
    Watchdog.reset();
  }
}

void sendDataNow() 
{
  String tempString = "";
  String voltageString = "";
  float tempVal = getTemp();
  Watchdog.reset();
  Serial.println("Got temp: " + String(tempVal));

  if (tempVal != DEVICE_DISCONNECTED_C && tempVal > -127.0f)
  {
      tempString = tempAsString(tempVal);
      Watchdog.reset();

      int sensorValue = analogRead(ADC_BATTERY);
      // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.3V):
      float voltage = sensorValue * (4.3 / 1023.0);
      voltageString = formatVoltageAsString(voltage);
      Serial.print("Sending temperature data:");
      String sendString = tempString + ";" + voltageString;

      unsigned int checksum = computeCRC(sendString);
      Watchdog.reset();

      sendString += ";" + String(checksum);
      
      Serial.println(sendString);

      // send packet
      LoRa.beginPacket();
      LoRa.print(sendString.c_str());
      LoRa.endPacket();

      Serial.println("Sent, waiting for reply...");

      Watchdog.reset();

      delay(250);
      
      // try to parse response packet
      readBuffer = "";
      int packetSize = LoRa.parsePacket();
      Watchdog.reset();
      if (packetSize) {
        // received a packet
        Serial.print("Received packet '");
    
        // read packet
        while (LoRa.available()) {
          int msg = LoRa.read();
          readBuffer += (char)msg;
        }
        Watchdog.reset();
        Serial.print(readBuffer);
    
        // print RSSI of packet
        Serial.print("' with RSSI ");
        Serial.println(LoRa.packetRssi());

        if (readBuffer.startsWith("ACK"))
        {
            Serial.println("Got response message:");  
            Serial.println(readBuffer);
            Serial.println("End of message");
            sendData = now() + SEND_WAIT; // wait this long until we send data again
        } else {
            Serial.println("Got unsupported message:");
            Serial.println(readBuffer);
            Serial.println("End of message");
        }
        Watchdog.reset();
      } else {
        Serial.println("Could not receive response!");
      }
      Serial.println("Waiting until " + formatDateTime(sendData) + " to send data again");
  }
}

float getTemp()
{    
    sensors.requestTemperatures(); // Send the command to get temperatures
    return sensors.getTempCByIndex(0);
}

String tempAsString(float tempC)
{
    String tempString = String(tempC);
    char tempChars[6];
    tempString.toCharArray(tempChars, 5);
    return String(tempChars);
}

String formatVoltageAsString(float voltage)
{
    String tempString = String(voltage);
    char vChars[4];
    tempString.toCharArray(vChars, 4);
    return String(vChars);
}

String formatDateTime(time_t t)
{
    int y = year(t);
    int mn = month(t);
    int d = day(t);
    int h = hour(t);
    int mi = minute(t);
    int s = second(t);

    String y_s = String(y);
    String mn_s = mn > 9 ? String(mn) : ("0" + String(mn));
    String d_s = d > 9 ? String(d) : ("0" + String(d));
    String h_s = h > 9 ? String(h) : ("0" + String(h));
    String mi_s = mi > 9 ? String(mi) : ("0" + String(mi));
    String s_s = s > 9 ? String(s) : ("0" + String(s));

    String retval = y_s + "-" + mn_s + "-" + d_s + " " + h_s + ":" + mi_s + ":" + s_s + " UTC";

    return retval;
}

uint32_t computeCRC(String input)
{
  CRC32 crc;
  char stringToCRC[255];
  int len = input.length() + 1;
  if(len > 255)
    len = 254;
  
  input.toCharArray(stringToCRC, len);
  
  crc.add((uint8_t*)stringToCRC, len);
  return crc.getCRC();
}
