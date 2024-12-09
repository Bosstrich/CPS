#define BLYNK_TEMPLATE_ID "TMPL63P8WCj6y"
#define BLYNK_TEMPLATE_NAME "Cyber Physical System"

#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Firebase_ESP_Client.h>
#include <Adafruit_ADS1X15.h>
#include <DFRobot_ESP_EC.h>
#include <DFRobot_ESP_PH_WITH_ADC.h>
#include <U8g2lib.h>
#include <DTH_Turbidity.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#define BLYNK_AUTH_TOKEN "V4BhQCxm4UxVxfUDzmDNQcFSWtKRAopN"
#define BLYNK_PRINT Serial
#define WIFI_SSID "PORTABLE WIFI"
#define WIFI_PASSWORD "37918120"

#define API_KEY "AIzaSyDr0U9ToKbAct3uEC8Ltif2GiEdedaRTAU"
#define DATABASE_URL "https://thesis-e576f-default-rtdb.asia-southeast1.firebasedatabase.app/"

#define ONE_WIRE_BUS 18
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature temp(&oneWire);

FirebaseData fbdo;
FirebaseAuth fauth;
FirebaseConfig config;
FirebaseJson json;
BlynkTimer timer;

DFRobot_ESP_EC ec;
DFRobot_ESP_PH_WITH_ADC ph;
Adafruit_ADS1115 ads;
int16_t ph_adc, ec_adc;

#define turbidity_pin A0
DTH_Turbidity turbSensor(turbidity_pin);


const int EEPROM_INPUT_CONTAINER_ADDR = 0;      // Address for inputContainerIsFull
const int EEPROM_OUTPUT_CONTAINER_ADDR = 1;     // Address for outputContainerIsFull
const int EEPROM_FILTRATION_CONTAINER_ADDR = 2; // Address for filtrationContainerIsFull

int relay_in1 = 23; // washer pump 5
int relay_in2 = 19; // Washer pump 4
int relay_in3 = 5;  // input washer pump and input solenoid 1
int relay_in4 = 17; // UV
int relay_in5 = 16; // Pump 1
int relay_in6 = 4;  // Pump 2
int relay_in7 = 21; //washer pump 3
int relay_in8 = 15; //washer pump 2
int trig_pin = 13;  // Output Container Ultrasonic 
int echo_pin = 12;  // Output Container Ultrasonic

int outputDistance = 0;
int currentParameterMonitored = 1; // 1 = NTU; 2 = EC and TDS; 3=  pH; 4 = Temperature; 
int currentFiltrationCount = 0;
int dayCount = 1;
int intervalCount = 0;
int timerId;

float temperature_value = 0;
float ph_value = 0;
float ec_value = 0;
float ec_microSiemens = 0;
float tds_value = 0;
float ntu_value = 0;
float CCMEWQI = 0;

bool inputContainerIsFull = false;
bool outputContainerIsFull = false;
bool filtrationContainerIsFull = false;
bool systemIsFiltering = false;
bool filtrationIsFinished = false;
bool dataReadyToSend = true;
bool dataReadyToReceive = false;
bool signupOK = false;
bool isInitialStartTimeSet = false;

unsigned long initialStartTime = 0;
String ccmewqiExpression = ""; 

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

int get_distance(int TRIG_PIN, int ECHO_PIN);
void sendToFirebase(float pH, float ec, float tds, float turbidity, float temperature);
void displayString(String task, String status, String status2);
void fillContainer(int container, bool feedback);
void displayToBlynk();
void flipReady();
void checkOutputContainer();
float read_temperature();
String interpretCCMEWQI(float index);
float getMinExcursion(float failed_value, float objective);
float getMaxExcursion(float failed_value, float objective);

void setup() {

  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);
  EEPROM.begin(32);
  ec.begin();
  ph.begin();
  temp.begin();
  ads.begin();
  u8g2.begin();

  if (EEPROM.read(EEPROM_INPUT_CONTAINER_ADDR) == 255) {
    EEPROM.write(EEPROM_INPUT_CONTAINER_ADDR, 0); // Write false (0) to EEPROM
  }
  if (EEPROM.read(EEPROM_OUTPUT_CONTAINER_ADDR) == 255) {
    EEPROM.write(EEPROM_OUTPUT_CONTAINER_ADDR, 0); // Write false (0) to EEPROM
  }
  if (EEPROM.read(EEPROM_FILTRATION_CONTAINER_ADDR) == 255) {
    EEPROM.write(EEPROM_FILTRATION_CONTAINER_ADDR, 0); // Write false (0) to EEPROM
  } 

  inputContainerIsFull = EEPROM.read(EEPROM_INPUT_CONTAINER_ADDR);
  outputContainerIsFull = EEPROM.read(EEPROM_OUTPUT_CONTAINER_ADDR);
  filtrationContainerIsFull = EEPROM.read(EEPROM_FILTRATION_CONTAINER_ADDR);

  timerId = timer.setInterval(10800000, flipReady);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  if (Firebase.signUp(&config, &fauth, "", "")) {
    Serial.println("Sign up success");
    signupOK = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &fauth);
  Firebase.reconnectWiFi(true);

  pinMode(relay_in1, OUTPUT);
  pinMode(relay_in2, OUTPUT);
  pinMode(relay_in3, OUTPUT);
  pinMode(relay_in4, OUTPUT);
  pinMode(relay_in5, OUTPUT);
  pinMode(relay_in6, OUTPUT);
  pinMode(relay_in7, OUTPUT);
  pinMode(relay_in8, OUTPUT);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

  digitalWrite(relay_in1, HIGH);
  digitalWrite(relay_in2, HIGH);
  digitalWrite(relay_in3, HIGH);
  digitalWrite(relay_in4, HIGH);
  digitalWrite(relay_in5, HIGH);
  digitalWrite(relay_in6, HIGH);
  digitalWrite(relay_in7, HIGH);
  digitalWrite(relay_in8, HIGH);
}

void loop() {
  timer.run();
  Blynk.run();

  displayString("Checking Output", "Calibrating", "");
  delay(500);

  displayToBlynk();

  checkOutputContainer();

  String output = "";

  if (outputContainerIsFull) {
    output = "Full";
  }
  else {
    output = "Empty";
  }

  displayString("Done Checking", "Output is ", output);
  delay(500);
  
  displayString("Checking containers", "calibrating.. ", output);
  delay(500);

  if (filtrationContainerIsFull && !inputContainerIsFull) {
    displayString("Done!", "Filter Full &.. ", "Input not Full");
    delay(500);  
    Serial.println("Condition 1: Pass");
    monitorWaterQuality();
  }
  else if (!inputContainerIsFull && !systemIsFiltering) {
    displayString("Done!", "Not filtering  ", " & Input not Full");
    delay(500); 
    Serial.println("Condition 2: Pass");
    fillContainer(1, false);
  }
  else if (inputContainerIsFull) {
    displayString("Done!", "Input is Full ", "");
    delay(500);
    Serial.println("Condition 3: Pass");
    fillContainer(2, false);
  }

  ec.calibration(ec_adc, temperature_value);
  ph.calibration(ph_adc, temperature_value);
  delay(500);
}

void displayString(String task, String status, String status2){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr); 
  u8g2.drawStr(0, 10, (task + "....").c_str());
  u8g2.drawStr(0, 25, ("Status: " + status).c_str()); 
  u8g2.drawStr(0, 40, (status2).c_str()); 
  u8g2.sendBuffer();
  delay(500);
}

void monitorWaterQuality(){

  systemIsFiltering = true;

  ph_adc = ads.readADC_SingleEnded(0);
  ec_adc = ads.readADC_SingleEnded(1);

  temperature_value = read_temperature();
  ph_value = ph.readPH(ph_adc, temperature_value);
  ec_value = ec.readEC(ec_adc, temperature_value);
  ec_microSiemens = ec_value * 1000;
  tds_value = ((ec_value * 1000) / 2);
  ntu_value = turbSensor.getVoltage();

  if (dataReadyToReceive) {

    CCMEWQI = evaluateCCMEWQI();
    displayString("Evaluating CCME WQI", "Index Complete", "");
    delay(1000);
    displayString("Interpreting CCME WQI", "Processing..", "");
    delay(1000);
    displayString("The Index is:" + String(CCMEWQI), "", "");
    delay(500);
    ccmewqiExpression = interpretCCMEWQI(CCMEWQI);
    displayString("Interpreting CCME WQI", "Done!", "");
    delay(500);

  }
  
  displayString("Evaluating Quality", "", "");
  delay(500);

  switch (currentParameterMonitored) {
    case 1:
      displayString("Evaluating Quality", "Reading NTU", "");
      delay(500);

      if (ntu_value <= 5) {
        displayString("Turbidity Test Done", "Passed", "");
        delay(500);
        Serial.println("Computation: turbidity pass");
        currentParameterMonitored = 2;
        // Move to case 2
      }
      else {
        displayString("Turbidity Test Done", "Failed", "");
        delay(500);

        if (currentFiltrationCount == 3) {
          currentFiltrationCount = 0;
          currentParameterMonitored = 2;
          // Move to case 2
        }
        else {
          Serial.println("Computation: turbidity fail");
          currentFiltrationCount++;
          filterCurrentParameter(1);
          break;
        }
      }
    case 2: 
      displayString("Evaluating Quality", "Reading EC & TDS", "");
      delay(500);

      if ((ec_microSiemens < 500 && ec_microSiemens > 100) || (tds_value < 300)) {
        displayString("EC/TDS Test Done", "Passed", "");
        delay(500);
        currentParameterMonitored = 3;
        Serial.println("Computation: ec and tds pass");
        // Move to case 3 
      }
      else {
        displayString("EC/TDS Test Done", "Failed", "");
        delay(500);
        
        if (currentFiltrationCount == 3) {
          Serial.println("Computation: ec and tds fail");
          currentFiltrationCount = 0;
          currentParameterMonitored = 3;
          // Move to case 3
        }
        else {
          currentFiltrationCount++;
          filterCurrentParameter(2);
          break;
        }
      }  
    case 3:
      displayString("Evaluating Quality", "Reading pH", "");
      delay(500);

      if (ph_value >= 6.5 && ph_value <= 8.5) {
        displayString("pH Test Done", "Passed", "");
        delay(500);
        Serial.println("Computation: pH pass");
        currentParameterMonitored = 4;
        // Move to case 4
      }
      else if (ph_value >= 15) {
        if (ph_value >= 6.5 && ph_value <= 8.5) {
        Serial.println("Computation: pH pass");
        currentParameterMonitored = 4;
        // Move to case 4
        }
      }
      else {
        displayString("pH Test Done", "Failed", "");
        delay(500);

        if (currentFiltrationCount == 3) {
          currentFiltrationCount = 0;
          currentParameterMonitored = 4;
          // Move to case 4
        }
        else {
          Serial.println("Computation: pH failed");
          currentFiltrationCount++;
          filterCurrentParameter(3);
          break;
        }
      } 
    case 4:
      Serial.print("Ready to Send Status:");
      Serial.println(String(dataReadyToSend));

      if (dataReadyToSend) {
        displayString("Sending Data to Firebase", "Processing..", "");
        Serial.println("Data ready to send");
        sendToFirebase(ph_value, ec_microSiemens, tds_value, ntu_value, temperature_value);
      }

      displayString("Into Final Phase", "Executing..", "");
      delay(500);
      Serial.println("Computation: Moving to Final Phase");
      filterCurrentParameter(4);
      Serial.print("Output Container Status:");
      Serial.println(outputContainerIsFull);
      Serial.print("System is Filtering Status:");
      Serial.println(systemIsFiltering);

      displayString("Checking Output", "Calibrating..", "");

      if (!systemIsFiltering && !outputContainerIsFull) {
        displayString("Checking Output", "Output not..", "full");
        Serial.println("Final Condition: Passed");
        fillContainer(3, false);
        currentParameterMonitored = 1;
      }
  }
  
}
void filterCurrentParameter(int filterStage) {
  if (outputContainerIsFull) {
    return;
  }

  // Where filterStage: 1 = PP; 2 = GAC; 3 = CTO; 4 = T33;
  fillContainer(1, true);

  int ppFilled = 0;
  String ppFilledStr;
  int gacFilled = 0;
  String gacFilledStr;
  int ctoFilled = 0;
  String ctoFilledStr;
  int t33Filled = 0;
  String t33FilledStr;

  switch (filterStage) {
    case 1:
      do {
        ppFilled = getPercentageFilled(2640.0, 28.29);
        ppFilledStr = String(ppFilled);
        displayString("Using PP Filter", ppFilledStr + "% Full", "");
        digitalWrite(relay_in8, LOW);   // Open PP Filter 
        Serial.println("Actuation: PP Valve Activated");

        if (ppFilled >= 100) {
          digitalWrite(relay_in8, HIGH); // Close PP Filter 
          Serial.println("Actuation: PP Valve Deactivated");
          inputContainerIsFull = false;
          EEPROM.write(EEPROM_INPUT_CONTAINER_ADDR, inputContainerIsFull);
          filtrationContainerIsFull = true;
          EEPROM.write(EEPROM_FILTRATION_CONTAINER_ADDR, filtrationContainerIsFull);

          break; 
        }

        delay(1000);
      } while (ppFilled < 100);

      resetStartTime();
      break;

    case 2:
      do {
        gacFilled = getPercentageFilled(2640.0, 28.29);
        gacFilledStr = String(gacFilled);
        displayString("Using GAC Filter", gacFilledStr + "% Full", "");
        digitalWrite(relay_in7, LOW);   // Open GAC Filter 
        Serial.println("Actuation: GAC Valve Activated");

        if (gacFilled >= 100) {
          digitalWrite(relay_in7, HIGH); // Close GAC Filter 
          Serial.println("Actuation: GAC Valve Deactivated");
          inputContainerIsFull = false;
          EEPROM.write(EEPROM_INPUT_CONTAINER_ADDR, inputContainerIsFull);
          filtrationContainerIsFull = true;
          EEPROM.write(EEPROM_FILTRATION_CONTAINER_ADDR, filtrationContainerIsFull);

          break; 
        }

        delay(1000);
      } while (gacFilled < 100);

      resetStartTime();
      break;

    case 3:
      do {
        ctoFilled = getPercentageFilled(2640.0, 28.29);
        ctoFilledStr = String(ctoFilled);
        displayString("Using CTO Filter", ctoFilledStr + "% Full", "");
        digitalWrite(relay_in2, LOW);   // Open CTO Filter 
        Serial.println("Actuation: CTO Valve Activated");

        if (ctoFilled >= 100) {
          digitalWrite(relay_in2, HIGH); // Close CTO Filter 
          Serial.println("Actuation: CTO Valve Deactivated");
          inputContainerIsFull = false;
          EEPROM.write(EEPROM_INPUT_CONTAINER_ADDR, inputContainerIsFull);
          filtrationContainerIsFull = true;
          EEPROM.write(EEPROM_FILTRATION_CONTAINER_ADDR, filtrationContainerIsFull);

          break; 
        }

        delay(1000);
      } while (ctoFilled < 100);

      resetStartTime();
      break;

    case 4:
      do {
        t33Filled = getPercentageFilled(2640.0, 28.29);
        t33FilledStr = String(t33Filled);
        displayString("Using T33 Filter", t33FilledStr + "% Full", "");
        digitalWrite(relay_in1, LOW);   // Open T33 Filter 
        Serial.println("Actuation: T33 Valve Activated");

        if (t33Filled >= 100) {
          digitalWrite(relay_in1, HIGH); // Close T33 Filter 
          Serial.println("Actuation: T33 Valve Deactivated");
          inputContainerIsFull = false;
          EEPROM.write(EEPROM_INPUT_CONTAINER_ADDR, inputContainerIsFull);
          filtrationContainerIsFull = true;
          EEPROM.write(EEPROM_FILTRATION_CONTAINER_ADDR, filtrationContainerIsFull);

          systemIsFiltering = false;
          break; 
        }

        delay(1000);
      } while (t33Filled < 100);

      resetStartTime();
      break;
    
  }
}

void fillContainer(int container, bool feedback) {
  // Let container 1 = input, 2 = filtration, 3 = output, 4 = feedback

  int inputFilled = 0;
  String inputFilledStr;
  int feedbackFilled = 0;
  String feedbackFilledStr;
  int t33Filled = 0;
  String t33FilledStr;
  int outputFilled = 0;
  String outputFilledStr;

  switch (container) {
    case 1:
      if (feedback) {
        do {
          feedbackFilled = getPercentageFilled(2970.0, 13.2);
          feedbackFilledStr = String(feedbackFilled);
          displayString("Feedbacking Input", feedbackFilledStr + "% Full", "");
          digitalWrite(relay_in5, LOW);  // Activate pump1
          Serial.println("Actuation: Pump1 Activated");

          if (feedbackFilled >= 100) {
            digitalWrite(relay_in5, HIGH); // Deactivate pump1
            Serial.println("Actuation: Pump1 Deactivated");
            inputContainerIsFull = true;
            EEPROM.write(EEPROM_INPUT_CONTAINER_ADDR, inputContainerIsFull);
            filtrationContainerIsFull = false;
            EEPROM.write(EEPROM_FILTRATION_CONTAINER_ADDR, filtrationContainerIsFull);

            break;
          }

          delay(1000);
        } while (feedbackFilled < 100);

        resetStartTime();
        break;

      }
      else {
        do {
          inputFilled = getPercentageFilled(2970.0, 25.82);
          inputFilledStr = String(inputFilled);
          displayString("Filling Input", inputFilledStr + "% Full", "");
          digitalWrite(relay_in3, LOW);  // Input Valve Open and Input pump activate
          Serial.println("Actuation: Input Valve Activated");

          if (inputFilled >= 100) {
            digitalWrite(relay_in3, HIGH); // Input Valve Close and Input pump deactivate
            Serial.println("Actuation: Input Valve Deactivated");
            inputContainerIsFull = true;
            EEPROM.write(EEPROM_INPUT_CONTAINER_ADDR, inputContainerIsFull);
            break; 
          }

          delay(1000);
        } while (inputFilled < 100);

        resetStartTime();
        break;

      }
    
    case 2:
      do {
        t33Filled = getPercentageFilled(2640.0, 28.29);
        t33FilledStr = String(t33Filled);
        displayString("Filling Filter", t33FilledStr + "% Full", "");
        digitalWrite(relay_in1, LOW);   // Open T33 Filter 
        Serial.println("Actuation: T33 Valve Activated");

        if (t33Filled >= 100) {
          digitalWrite(relay_in1, HIGH); // Close T33 Filter 
          Serial.println("Actuation: T33 Valve Deactivated");
          filtrationContainerIsFull = true;
          EEPROM.write(EEPROM_FILTRATION_CONTAINER_ADDR, filtrationContainerIsFull);

          inputContainerIsFull = false;
          EEPROM.write(EEPROM_INPUT_CONTAINER_ADDR, inputContainerIsFull);
          break; 
        }

        delay(1000);
      } while (t33Filled < 100);

      resetStartTime();
      break;
    
    case 3:
      do {
        outputFilled = getPercentageFilled(2640.0, 8.12);
        outputFilledStr = String(outputFilled);
        displayString("Filling Output", outputFilledStr + "% Full", "");
        digitalWrite(relay_in6, LOW);  // Activate Pump2
        digitalWrite(relay_in4, LOW);  // Activate UV
        Serial.println("Actuation: Pump2 and UV Activated");

        if (outputFilled >= 100) {
          digitalWrite(relay_in6, HIGH);  // Activate Pump2
          digitalWrite(relay_in4, HIGH);  // Activate UV
          Serial.println("Actuation: Pump2 and UV Deactivated");
          outputContainerIsFull = true;
          filtrationContainerIsFull = false;
          EEPROM.write(EEPROM_FILTRATION_CONTAINER_ADDR, filtrationContainerIsFull);

          break; 
        }

        delay(1000);
      } while (outputFilled < 100);

      resetStartTime();
      break;
  }
}

void flipReady(){
  if (!dataReadyToSend) {
    dataReadyToSend = !dataReadyToSend;
  }
}

void sendToFirebase(float pH, float ec, float tds, float turbidity, float temperature) {
  displayString("Sending To Firebase", "Processing", "");
  delay(500);
  intervalCount++;

  Serial.print("interval count = ");
  Serial.print(intervalCount);
  Serial.print("; day count = ");
  Serial.println(dayCount);

  String partOfDay;

  switch (intervalCount) {
    case 1:
      partOfDay = "Morning";
      break;
    case 2:
      partOfDay = "Afternoon";
      break;
    case 3:
      partOfDay = "Evening";
      break;
  }

  String path = "/Day " + String(dayCount) + "/" + partOfDay + "/";

  json.set("/pH", String(pH));
  json.set("/Temperature", String(temperature));
  json.set("/EC", String(ec));
  json.set("/Turbidity", String(turbidity));
  json.set("/TDS", String(tds));
  Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());

  displayString("Sending To Firebase", "Success!", "");
  delay(500);

  dataReadyToSend = false;

  if (intervalCount == 3) {
    intervalCount = 0;
    dayCount++;

    if (dayCount == 3) {
      dayCount = 1;
      dataReadyToReceive = true;
    }
  }

}

void checkOutputContainer() {
  outputDistance = get_distance(trig_pin, echo_pin);

  Serial.print("Output distance: ");
  Serial.println(outputDistance);

  if (outputDistance >=10) {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
    outputContainerIsFull = false;
  }
  else if (outputDistance <=2) {
    outputContainerIsFull = true;
  }
}

int getPercentageFilled(float containerVolume, float fillingRate) {
  if (!isInitialStartTimeSet) {
    initialStartTime = millis();
    isInitialStartTimeSet = true;
  }

  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - initialStartTime) / 1000.0; 
  float volumeOfWater = fillingRate * elapsedTime; 
  int percentageFilled = round(volumeOfWater / containerVolume * 100);

  if (percentageFilled >= 100) {
    percentageFilled = 100;
  }

  return percentageFilled;
}

void resetStartTime() {
  initialStartTime = 0;
  isInitialStartTimeSet = false;
}

float evaluateCCMEWQI() {
  displayString("Getting Firebase Data", "Processing", "");
  delay(1000);

  float avgPH[2] = {0}, avgTemperature[2] = {0}, avgEC[2] = {0}, avgTurbidity[2] = {0}, avgTDS[2] = {0};

  for (int day = 1; day <= 2; day++) {

    String dayPath = "/Day " + String(day);

    float totalPH = 0, totalTemperature = 0, totalEC = 0, totalTurbidity = 0, totalTDS = 0;
    int count = 0;

    for (int interval = 1; interval <= 3; interval++) {

      String partOfDay;

      switch (interval) {
          case 1:
              partOfDay = "Morning";
              break;
          case 2:
              partOfDay = "Afternoon";
              break;
          case 3:
              partOfDay = "Evening";
              break;
      }

      String path = dayPath + "/" + partOfDay + "/";

      float pH = 0, temperature = 0, ec = 0, turbidity = 0, tds = 0;

      if (Firebase.RTDB.getJSON(&fbdo, path.c_str())) {
        FirebaseJson &json = fbdo.jsonObject();
        FirebaseJsonData jsonData;

        json.get(jsonData, "/pH");
        pH = jsonData.to<float>();

        json.get(jsonData, "/Temperature");
        temperature = jsonData.to<float>();

        json.get(jsonData, "/EC");
        ec = jsonData.to<float>();

        json.get(jsonData, "/TDS");
        tds = jsonData.to<float>();

        json.get(jsonData, "/Turbidity");
        turbidity = jsonData.to<float>();
      }

      totalPH += pH;
      totalTemperature += temperature;
      totalEC += ec;
      totalTurbidity += turbidity;
      totalTDS += tds;
      count++;
    }

    if (day == 1) {
      avgPH[0] = totalPH / count;
      avgTemperature[0] = totalTemperature / count;
      avgEC[0] = totalEC / count;
      avgTurbidity[0] = totalTurbidity / count;
      avgTDS[0] = totalTDS / count;
    }
    else {
      avgPH[1] = totalPH / count;
      avgTemperature[1] = totalTemperature / count;
      avgEC[1] = totalEC / count;
      avgTurbidity[1] = totalTurbidity / count;
      avgTDS[1] = totalTDS / count;
    }
  }

  displayString("Retrieving Firebase Data", "Data Received!", "");
  delay(500);

  displayString("Evaluating CCME WQI", "Calculating F1", "");
  delay(1000);

  int total_variables = 5;
  int total_tests = 10;
  int failed_tests = 0;
  int failed_variables = 0;
  float sumOfExcursions = 0;
  float nse = 0;

  for(int i = 0; i < 2; i++ ){
    if (avgPH[i] < 6.5 || avgPH[i] > 8.5) {
      if (avgPH[i] < 6.5) {
        sumOfExcursions += getMinExcursion(avgPH[i], 6.5);
      } 
      else {
        sumOfExcursions += getMaxExcursion(avgPH[i], 8.5);
      }
      failed_tests++;
      failed_variables = 1;
    }

    if (avgTurbidity[i] > 5) {
      sumOfExcursions += getMaxExcursion(avgTurbidity[i], 5);
      failed_tests++;
      failed_variables = 2;
    }

    if (avgTDS[i] > 300) {
      sumOfExcursions += getMaxExcursion(avgTDS[i], 300);
      failed_tests++;
      failed_variables = 3;
    }

    if (avgEC[i] > 500 || avgEC[i]  < 100) {
      if (avgEC[i] > 500) {
        sumOfExcursions += getMaxExcursion(avgEC[i], 500);
      } 
      else {
        sumOfExcursions += getMinExcursion(avgEC[i], 100);
      }
      failed_tests++;
      failed_variables = 4;

    }

    if (avgTemperature[i] > 50 || avgTemperature[i] < 20) {
      if (avgTemperature[i] > 50) {
        sumOfExcursions += getMaxExcursion(avgTemperature[i], 50);
      } 
      else {
        sumOfExcursions += getMinExcursion(avgTemperature[i], 20);
      }
      failed_tests++;
      failed_variables = 5;
    }
  }

  float factor1 = (failed_variables / (float)total_variables) * 100;
  displayString("Evaluating CCME WQI", "Calculating F2", "");
  delay(1000);
  float factor2 = failed_tests/total_tests;

  nse = sumOfExcursions / total_variables;

  float factor3 = nse / (0.01 * nse + 0.01);

  displayString("Evaluating CCME WQI", "Calculating F3", "");
  delay(1000);
  
  dataReadyToReceive = false;

  return (100 - (sqrt(pow(factor1, 2) + pow(factor2, 2) + pow(factor3, 2)) / 1.732));
}

float getMaxExcursion(float failed_value, float objective) {
  return (failed_value / objective) - 1;
}

float getMinExcursion(float failed_value, float objective) {
  return (objective / failed_value) - 1;
}

String interpretCCMEWQI(float index) {
  if (index >= 95 && index <= 100) {
    return "Virtual absence of threat or impairment";
  } else if (index >= 80 && index <= 94) {
    return "Minor degree of threat to water quality";
  } else if (index >= 65 && index <= 79) {
    return "Water quality occasionally threatened or impaired";
  } else if (index >= 45 && index <= 64) {
    return "Frequent threat to the water quality";
  } else {
    return "Water quality almost always threatened or impaired";
  }
}

int get_distance(int TRIG_PIN, int ECHO_PIN) {
  long duration;
  int distance;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.0344 / 2;

  return distance;
}

void displayToBlynk(){

  Blynk.virtualWrite(V0, CCMEWQI);
  Blynk.virtualWrite(V1, ccmewqiExpression);
  Blynk.virtualWrite(V3, ntu_value);
  Blynk.virtualWrite(V2, ph_value);
  Blynk.virtualWrite(V4, tds_value);
  Blynk.virtualWrite(V5, ec_microSiemens);
  Blynk.virtualWrite(V6, temperature_value);
  displayString("Sending Data to Blynk", "Success!", "");

}

float read_temperature() {
  temp.requestTemperatures();
  return temp.getTempCByIndex(0);
}




