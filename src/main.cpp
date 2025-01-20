#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <DHT.h>
#include <SPI.h>
#include <Wire.h>

#define SCREEN_WIDTH 128      // OLED display width, in pixels
#define SCREEN_HEIGHT 64      // OLED display height, in pixels
#define OLED_RESET     -1     // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C   //< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define DHTPIN 4
#define DHTTYPE DHT22         //DHT11 // DHT21
#define PID_COOLANT_TEMP "0105"
#define m5Name "PandaBT"

BluetoothSerial ELM_PORT;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DHT dht;

bool ELMinit();
bool BTconnect();
bool sendAndReadCommand(const char* cmd, String& response, int delayTime);
void sendOBDCommand(const char* cmd);
void writeToCircularBuffer(char c);
void handleOBDResponse();

//float parseOBDVoltage(const String& response);

String readFromCircularBuffer(int numChars);
String bufferSerialData(int timeout, int numChars);
void parseOBDData(const String& response);
float parseCoolantTemp(const String& response);

//uint8_t BLEAddress[6] = {0x00, 0x10, 0xCC, 0x4F, 0x36, 0x03};  // Indirizzo Bluetooth del modulo ELM327 

uint8_t BLEAddress[6] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xBA};  // Indirizzo Bluetooth del modulo ELM327 box piccolo

//float obdVoltage = 0.0;

uint16_t coolantTemp = 0;
uint16_t lastCoolantTemp = -999; 

//const unsigned long voltageQueryInterval = 1000; // Intervallo di 4 secondi
//unsigned long lastVoltageQueryTime = 0;

unsigned long lastOBDQueryTime = 0;
unsigned long OBDQueryInterval = 150; // Intervallo di query OBD in millisecondi
const int BUFFER_SIZE = 256;
char circularBuffer[BUFFER_SIZE];
int writeIndex = 0;
int readIndex = 0;

void setup() {
  Serial.begin(115200);
  dht.setup(DHTPIN, dht.DHTTYPE);
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); 
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.setTextSize(1);
  BTconnect();
  delay(1000);
  display.setCursor(0,0);
  ELMinit();
  delay(500);
  display.setCursor(0,0);
  display.setTextSize(2);
}

void loop() {
  delay(1500);
  uint16_t hh = dht.getHumidity();
  uint16_t tt = dht.getTemperature();
  sendOBDCommand(PID_COOLANT_TEMP);
  delay(30);
  //sendOBDCommand("ATRV");
  //handleOBDResponse();
  /*
  Serial.print(F("Humidity: "));
  Serial.print(hh);
  Serial.print(F("Temperature: "));
  Serial.print(tt);
  */
  display.clearDisplay();
  
  //display.drawLine(0 ,64, 0 , 0, SSD1306_WHITE);
  display.setCursor(0,0);
  //display.print("Volt:");
  //display.println(obdVoltage);
  display.print("Acqua:");
  display.print(coolantTemp);
  display.println("C");
  display.setCursor(0,23);
  display.print("Ext:  ");
  display.print(tt);
  display.print("C");
  display.setCursor(0,46);
  display.print("Hum:  ");
  display.print(hh);
  display.print("%");
  display.display();
}

void coolantRequest(){
  sendOBDCommand(PID_COOLANT_TEMP);
  handleOBDResponse();
  lastCoolantTemp = coolantTemp; // Aggiorna lastCoolantTemp solo quando il valore cambia
  }

bool BTconnect() {
  ELM_PORT.begin(m5Name, true);  // Avvia la connessione Bluetooth
  display.println("Connessione BT...");
  display.display();
  int retries = 0;  // Tentativo connessione bluetooth ELM327 (5 try)
  bool connected = false;
  while (retries < 2 && !connected) {
    connected = ELM_PORT.connect(BLEAddress);
    if (!connected) {
      display.println("BT Conn FAIL");
      display.display();
      retries++;
      delay(500);
    }
  }

  if (!connected) {
    display.println("ELM BT NOT FOUND");
    display.display();
    return false;  // Loop infinito se non riesce a connettersi
  } else {
    display.println("Connessione BT OK!");
    display.display();
    return true;
  }
}

bool sendAndReadCommand(const char* cmd, String& response, int delayTime) {
  response = "";
  unsigned long startTime = millis();
  ELM_PORT.print(cmd);
  ELM_PORT.print("\r\n");

  while (millis() - startTime < delayTime) {
    if (ELM_PORT.available()) {
      char c = ELM_PORT.read();
      response += c;
    }
    delay(30); // Breve delay per non sovraccaricare la CPU
  }
  response.trim(); // Rimuove spazi bianchi

  if (response.length() > 0) {
    display.println(response.c_str());
    display.display();
  }

  if (response.length() == 0) {
    Serial.println("No RCV");
    return false;
  }

  if (response.indexOf("OK") >= 0 || response.length() > 0) {
    return true;
  } else {
    Serial.println("Err: " + response);
    return false;
  }
}

void sendOBDCommand(const char* cmd) {
  ELM_PORT.print(cmd);
  ELM_PORT.print("\r\n");
}

String bufferSerialData(int timeout, int numChars) {
    unsigned long startTime = millis();
    while (millis() - startTime < timeout) {
        while (ELM_PORT.available()) {
            char c = ELM_PORT.read();
            writeToCircularBuffer(c);
        }
        delay(30); // Breve delay per non sovraccaricare la CPU
    }
    return readFromCircularBuffer(numChars);
}

void handleOBDResponse() {
  String response = bufferSerialData(250, 100);  // Riempimento del buffer
  Serial.println(response);
  parseOBDData(response);  // Parsing del buffer
}

void parseOBDData(const String& response) {
  if (response.indexOf("4105") >= 0) {
    coolantTemp = parseCoolantTemp(response);
  }
  /*
   else if ((response.indexOf("V") >= 0) || ((response.indexOf("v") >= 0))) {
    obdVoltage = parseOBDVoltage(response);
  }
  */ 
}

// Funzione per analizzare la temperatura del liquido di raffreddamento
float parseCoolantTemp(const String& response) {
  if (response.indexOf("4105") == 0) {
    byte tempByte = strtoul(response.substring(4, 6).c_str(), NULL, 16);
    return tempByte - 40;
  }
  return lastCoolantTemp;
}

// Funzione per analizzare la tensione OBD

/*
float parseOBDVoltage(const String& response) {
  int indexV = response.indexOf('V');
  if (indexV >= 0) {
    String voltageStr = response.substring(0, indexV);
    return voltageStr.toFloat();
  }
  return 0.0;
}

*/

bool ELMinit() {
  String response;
  display.clearDisplay();
  display.println("ELM init...");
  display.display();

  if (!sendAndReadCommand("ATZ", response, 1500)) {
      display.println("err ATZ");
      display.display();
    return false;
  }

  if (!sendAndReadCommand("ATE0", response, 1500)) {
    display.println("err ATE0");
    display.display();
    return false;
  }

  if (!sendAndReadCommand("ATL0", response, 1500)) {
    display.println("Err ATL0");
    display.display();
    return false;
  }

  if (!sendAndReadCommand("ATS0", response, 1500)) {
    display.println("Err ATS0");
    display.display();
    return false;
  }

  if (!sendAndReadCommand("ATST0A", response, 1500)) {
    display.println("Err ATST0A");
    display.display();
    return false;
  }

  if (!sendAndReadCommand("ATSP0", response, 15000)) {  // Imposta protocollo automatico SP 0 e gestire la risposta "SEARCHING..."
    display.println("Err ATSP0");
    display.display();
    return false;
  }
  
  return true;
}

void writeToCircularBuffer(char c) {
    circularBuffer[writeIndex] = c;
    writeIndex = (writeIndex + 1) % BUFFER_SIZE;
    if (writeIndex == readIndex) {
        readIndex = (readIndex + 1) % BUFFER_SIZE; // Sovrascrivi i dati più vecchi
    }
}

String readFromCircularBuffer(int numChars) {
    String result = "";
    int charsRead = 0;
    
    // Leggi dal buffer finché ci sono caratteri da leggere
    // e non si è raggiunto il numero massimo di caratteri richiesti
    while (readIndex != writeIndex && charsRead < numChars) {
        result += circularBuffer[readIndex];
        readIndex = (readIndex + 1) % BUFFER_SIZE;
        charsRead++;
    }
    result.trim();  // Rimuove gli spazi bianchi extra all'inizio e alla fine
    return result;
}




