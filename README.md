#define RXp2 16
#define TXp2 17

#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
//#include "DHT.h"
#include <SimpleTimer.h>
#include <wifi_provisioning/manager.h>

// Set Defalt Values
//#define DEFAULT_RELAY_MODE true
String Methane;
String CO2;
String O2;
//String Humidity;

// BLE Credentils
const char *service_name = "Kar13";
const char *pop = "Kar13022003";

// GPIO
static uint8_t gpio_reset = 0;
//static uint8_t DHTPIN = 19;
//static uint8_t relay = 21;
//bool relay_state = true;

static uint8_t PIR_PIN    = 34;
static uint8_t Led    = 35;


bool wifi_connected = 0;

//DHT dht(DHTPIN, DHT11);

SimpleTimer Timer;

//------------------------------------------- Declaring Devices -----------------------------------------------------//

//The framework provides some standard device types like switch, lightbulb, fan, temperature sensor.
static TemperatureSensor methane("Methane");

void sysProvEvent(arduino_event_t *sys_event)
{
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32
      Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
      printQR(service_name, pop, "ble");
#else
      Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
      printQR(service_name, pop, "softap");
#endif
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.printf("\nConnected to Wi-Fi!\n");
      wifi_connected = 1;
      delay(500);
      break;
    case ARDUINO_EVENT_PROV_CRED_RECV: {
        Serial.println("\nReceived Wi-Fi credentials");
        Serial.print("\tSSID : ");
        Serial.println((const char *) sys_event->event_info.prov_cred_recv.ssid);
        Serial.print("\tPassword : ");
        Serial.println((char const *) sys_event->event_info.prov_cred_recv.password);
        break;
      }
    case ARDUINO_EVENT_PROV_INIT:
      wifi_prov_mgr_disable_auto_stop(10000);
      break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      Serial.println("Stopping Provisioning!!!");
      wifi_prov_mgr_stop_provisioning();
      break;
  }
}



void setup()
{

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2);

  // Configure the input GPIOs
  pinMode(gpio_reset, INPUT);
 // pinMode(relay, OUTPUT);
 // digitalWrite(relay, DEFAULT_RELAY_MODE);

 pinMode(PIR_PIN, INPUT);
 //pinMode(LED, OUTPUT);

  //Beginning Sensor
  //dht.begin();

  //------------------------------------------- Declaring Node -----------------------------------------------------//
  Node my_node;
  my_node = RMaker.initNode("Oxygen");

  //Standard switch device
  //my_switch.addCb(write_callback);

  //------------------------------------------- Adding Devices in Node -----------------------------------------------------//
  my_node.addDevice(methane);
  my_node.addDevice(co2);
  
  my_node.addDevice(o2);
  //my_node.addDevice(humidity);
  //my_node.addDevice(my_switch);


  //This is optional
  RMaker.enableOTA(OTA_USING_PARAMS);
  //If you want to enable scheduling, set time zone for your region using setTimeZone().
  //The list of available values are provided here https://rainmaker.espressif.com/docs/time-service.html
  // RMaker.setTimeZone("Asia/Shanghai");
  // Alternatively, enable the Timezone service and let the phone apps set the appropriate timezone
  RMaker.enableTZService();
  RMaker.enableSchedule();

  Serial.printf("\nStarting ESP-RainMaker\n");
  RMaker.start();

  // Timer for Sending Sensor's Data
  Timer.setInterval(1000);

  WiFi.onEvent(sysProvEvent);

#if CONFIG_IDF_TARGET_ESP32
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
#else
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
#endif

}


void loop()
{

  if (Timer.isReady() && wifi_connected) {                    // Check is ready a second timer
    Serial.println("Sending Sensor's Data");
    Send_Sensor();
    Timer.reset();                        // Reset a second timer
  }



  //-----------------------------------------------------------  Logic to Reset RainMaker

  // Read GPIO0 (external button to reset device
  if (digitalRead(gpio_reset) == LOW) { //Push button pressed
    Serial.printf("Reset Button Pressed!\n");
    // Key debounce handling
    delay(100);
    int startTime = millis();
    while (digitalRead(gpio_reset) == LOW) delay(50);
    int endTime = millis();

    if ((endTime - startTime) > 10000) {
      // If key pressed for more than 10secs, reset all
      Serial.printf("Reset to factory.\n");
      wifi_connected = 0;
      RMakerFactoryReset(2);
    } else if ((endTime - startTime) > 3000) {
      Serial.printf("Reset Wi-Fi.\n");
      wifi_connected = 0;
      // If key pressed for more than 3secs, but less than 10, reset Wi-Fi
      RMakerWiFiReset(2);
    }
  }
  delay(100);
}


void Send_Sensor()
{


       methane.updateAndReportParam("Temperature", vv);
       co2.updateAndReportParam("Temperature", cc);
       o2.updateAndReportParam("Temperature", tt);
       // humidity.updateAndReportParam("Temperature", hh);
       

       digitalRead(PIR_PIN); //read new state
       //------------------------------------------------------------------------
        if(digitalRead(PIR_PIN) == LOW) {

        Serial.println("Motion detected!");
        esp_rmaker_raise_alert("Alert!\LOW Oxygen");
        delay(500);
        }
        //------------------------------------------------------------------------
  


       }
   }  
    
}
