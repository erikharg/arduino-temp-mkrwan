#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_SleepyDog.h>
#include "TimeLib.h"

// Timekeeping
unsigned long sendData = 0;         // next time we'll send data
unsigned long SEND_WAIT = 5;     // how long to wait between submissions -- 1800 = 30 min
unsigned long LOOP_WAIT_MS = 5000;  // how long to wait between loops -- 2000 ms = 2 sec
unsigned long lastLoopMillis = 0; // time of last loop execution

int counter = 0;

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
  Serial.println("LoRa started - starting service...");
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
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;
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
