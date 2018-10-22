/**********************************************************************************************************************/
/* header files and Libraries */
/**********************************************************************************************************************/
#include "bsec_integration.h"
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
/**********************************************************************************************************************/
/* Variables, Constans and Others  */
/**********************************************************************************************************************/
//WiFi and Mqtt Variables
const char* ssid     = "iot";                           // your network SSID "iot"; "FRITZ!Box Fon WLAN 7270";
const char* password = "iotAccess!888";                                  // your network password "iotAccess!888"; "9128579217939524";
const char* mqtt_server = "192.168.32.79";                                // your mqtt server ip "192.168.32.78"; "192.168.204.1";
const int mqtt_port = 1883;                // your mqtt server port
const char* mqtt_topic = "test";           // topic 
const char* mqttUser = "praktikum";
const char* mqttPassword = "praktipass";

WiFiClient espClient;                   // allows to establish a connection to a defined IP and port
PubSubClient client(espClient);         // input of the constructor the previously defined WiFiClient

//int64_t timestamp;
//float iaq;
//uint8_t iaq_accuracy;
//float temperature;
//float humidity;
//float pressure;
//float raw_temperature;
//float raw_humidity; 
//float gas;
//float static_iaq;
//float co2_equivalent;
//float breath_voc_equivalent;

//Struct
//struct measurement {
//  String sensor;
//  String mac;
//  float iaq, temperature, humidity, pressure, raw_temperature, raw_humidity, gas, static_iaq, co2_equivalent, breath_voc_equivalent;
//  String error;
//};

/**********************************************************************************************************************/
/* Callback function  */
/**********************************************************************************************************************/
// Callback function
void callback(char* topic, byte* payload, unsigned int length) {

  // Allocate the correct amount of memory for the payload copy
  byte* p = (byte*)malloc(length);
  // Copy the payload to the new buffer
  memcpy(p,payload,length);
  client.publish("test", p, length);
  // Free the memory
  free(p);
}
/**********************************************************************************************************************/
/* functions */
/**********************************************************************************************************************/
/*!
 * @brief           Write operation in either Wire or SPI
 * param[in]        dev_addr        Wire or SPI device address
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 * @return          result of the bus communication function
 */
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);    /* Set register address to start writing to */
 
    /* Write the data */
    for (int index = 0; index < data_len; index++) {
        Wire.write(reg_data_ptr[index]);
    } 
    Wire.endTransmission();
    return 0;
}
/*!
 * @brief           Read operation in either Wire or SPI (look for rest "write"
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 */
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    int8_t comResult = 0;
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);                    /* Set register address to start reading from */
    comResult = Wire.endTransmission();

    delayMicroseconds(150);                 /* Precautionary response delay */
    Wire.requestFrom(dev_addr,(uint8_t)data_len);    /* Request data */

    int index = 0;
    while (Wire.available()){ /* The slave device may send less than requested*/
        reg_data_ptr[index] = Wire.read();
        index++;
    }
    return 0;
}
/*!
 * @brief           System specific implementation of sleep2 function
 * @param[in]       t_ms    time in milliseconds
 * @return          none */
void sleep2(uint32_t t_ms)  /* an sämtlichen stellen wurde sleep durch sleep2 ersetzt!! */
{
    delay(t_ms);
}
/*! 
 *  @brief           Capture the system time in microseconds
 *  @return          system_current_time    current system timestamp in microseconds */
int64_t get_timestamp_us()
{
    return (int64_t) millis() * 1000;
}

//timestamo meinteimstamp;
/*!
 * @brief           Handling of the ready outputs
 *
 * @param[in]       timestamp       time in nanoseconds
 * @param[in]       iaq             IAQ signal
 * @param[in]       iaq_accuracy    accuracy of IAQ signal
 * @param[in]       temperature     temperature signal
 * @param[in]       humidity        humidity signal
 * @param[in]       pressure        pressure signal
 * @param[in]       raw_temperature raw temperature signal
 * @param[in]       raw_humidity    raw humidity signal
 * @param[in]       gas             raw gas sensor signal
 * @param[in]       bsec_status     value returned by the bsec_do_steps() call
 *
 * @return          none
 */

void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
     float static_iaq, float co2_equivalent, float breath_voc_equivalent)
{
//  meintimetsamp = timestamp;
  
//  allaedate = timesta
    Serial.print("[");
    Serial.print(timestamp/1e6);
    Serial.print("] T: ");
    Serial.print(temperature);
    Serial.print("| rH: ");
    Serial.print(humidity);
    Serial.print("| IAQ: ");
    Serial.print(iaq);
    Serial.print(" (");
    Serial.print(iaq_accuracy);
    Serial.print("| Static IAQ: ");
    Serial.print(static_iaq);
    Serial.print("| CO2e: ");
    Serial.print(co2_equivalent);
    Serial.print("| bVOC: ");
    Serial.println(breath_voc_equivalent);
    
//Json Objekt Buffer -> Speicher größe
 StaticJsonBuffer<500> JSONbuffer;
//Json Objekt wird erstllt
 JsonObject& JSONencoder = JSONbuffer.createObject();
//Werte werden umgewandelt
 JSONencoder["Timestamp"] = timestamp/1e6;
 JSONencoder["Temperature"] = temperature;
 JSONencoder["Humidity"] = humidity;
 JSONencoder["IAQ"] = iaq;
 JSONencoder["IAQ Accuracy"] = iaq_accuracy;
 JSONencoder["Static IAQ"] = static_iaq;
 JSONencoder["CO2"] = co2_equivalent;
 JSONencoder["bVOC"] = breath_voc_equivalent; 
//Objekt wird befüllt
 char JSONmessageBuffer[300];
 JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));

 //client.publish("test", JSONmessageBuffer);  // publish wird nur 1 mal ausgeführt evtl. wegen keine abfrage ob verbindung noch besteht ?
 Serial.println(JSONmessageBuffer);

    /* hier drinne die Werte in Jason objekt verwandeln und anschließend per client.publish an den mqtt broker senden!! */
}

/*!
 * @brief           Load previous library state from non-volatile memory
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 * @return          number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    return 0;
}
/*!
 * @brief           Save library state to non-volatile memory
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
}

/*!
 * @brief           Load library config from non-volatile memory
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    return 0;
}

/*!
 * @brief       Main function which configures BSEC library and then reads and processes the data from sensor based
 *              on timer ticks
 * @return      result of the processing
 */
void setup()
{
    return_values_init ret;

    /* Init I2C and serial communication */
    Wire.begin(21,22);
    Serial.begin(115200);
    //Serial.print("test1");
    /* WiFi and Mqtt setup */
//  WiFi.begin(ssid, password); 
//    while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.println("Connecting to WiFi..");
//}
//  Serial.println("Connected to the WiFi network");
// //connection to the mqtt server
//  client.setServer(mqtt_server, mqtt_port);
//  while (!client.connected()) {
//    Serial.println("Connecting to MQTT...");
// 
//    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
//      //client.publish("test","hello world");
//      Serial.println("connected");  
// 
//    } else { 
//      Serial.print("failed with state ");
//      Serial.print(client.state());
//      delay(2000); 
//    }
//}
  
    /* Call to the function which initializes the BSEC library 
     * Switch on low-power mode and provide no temperature offset */
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 5.0f, bus_write, bus_read, sleep2, state_load, config_load);
    if (ret.bme680_status)
    {
        /* Could not intialize BME680 */
        Serial.println("Error while initializing BME680");
        return;
    }
    else if (ret.bsec_status)
    {
        /* Could not intialize BSEC library */
        Serial.println("Error while initializing BSEC library");
        return;
    }

   // jsonencoder();
    /* Call to endless loop function which reads and processes data based on sensor settings */
    /* State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes  */
    bsec_iot_loop(sleep2, get_timestamp_us, output_ready, state_save, 10000);
}
//JsonObjekt und Mqtt publish
//void jsonencoder (){
////Json Objekt Buffer -> Speicher größe
// StaticJsonBuffer<500> JSONbuffer;
////Json Objekt wird erstllt
// JsonObject& JSONencoder = JSONbuffer.createObject();
////Werte werden umgewandelt
// JSONencoder["Timestamp"] = timestamp/1e6;
// JSONencoder["Temperature"] = temperature;
// JSONencoder["Humidity"] = humidity;
// JSONencoder["IAQ"] = iaq;
// JSONencoder["IAQ Accuracy"] = iaq_accuracy;
// JSONencoder["Static IAQ"] = static_iaq;
// JSONencoder["CO2"] = co2_equivalent;
// JSONencoder["bVOC"] = breath_voc_equivalent; 
////Objekt wird befüllt
// char JSONmessageBuffer[300];
// JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
//
// client.publish("test", JSONmessageBuffer);
// Serial.println(JSONmessageBuffer);
//}

void loop()
{
//bsec_iot_loop sorgt dafür, dass diese methode nicht aufgerufen wird
}

/*! @}*/
