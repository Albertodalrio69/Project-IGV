//===Libraries===//
#include <Arduino.h>            //Default Arduino Library
#include <WiFi.h>               //WIFI Library
#include <HTTPClient.h>         //HTTP Library for Microphone Server
#include "I2SSampler.h"         //I2S Microphone Tasks Library
#include <ESPmDNS.h>            //DNS Library
#include <AsyncElegantOTA.h>    //ESP32 OTA update with HTTP
#include <ESPAsyncWebServer.h>  //AsyncWebServer Library
#include <Servo.h>              //Servo Motor Library
#include <TinyGPS++.h>          //GPS Library
//===Libraries===//

//===Library-Objects===//
//Create GPS Object to control the GPS
TinyGPSPlus gps;  
//Create Servo Object to control the Servo         
Servo myservo;           
//Create AsyncWebServer named server on port 80 (default HTTP port)  
AsyncWebServer server(80); 
//===Library-Objects===//

//===WIFI-Credentials===//
//SSID of WiFi network
const char* ssid     = "Hotspot-Persona4Golden";  
//Password of WiFi network
const char* password = "ohaw71444";               
//===WIFI-Credentials===//

//===Variables&Constants===//
//ESP32 name shown on Hotspot (By default, the hostname of the ESP32 is espressif)
String HostName = "Project IGV ESP32"; 
//Current String to get send via HTTP 
String thisStringCurrent;
//Servo angle to get from LabVIEW in String foramt
String thisStringServo, ServoBufferOld, ServoBufferNew;
//Servo angle to move the servo to
int thisIntegerServo;
//Speed of GPS & Number of satellites in use
String thisStringGPSSpeed, thisStringGPSNOSIU;  
String thisStringGPSLocationTrue, thisStringGPSLocationFalse;
String thisStringGPSLocationLL, thisStringGPSLocationLLF;
String SlashN = "\n";
//===Variables&Constants===//

//===Defines===//
//Define Server URL : Default Gateway of Network (NoWiFi) + Port + Directory
#define I2S_SERVER_URL "http://192.168.137.1:5003/i2s_samples" 
//Define Servo PWN pin
#define ServoGPIO 14
//Define ESP32 UART RX pin                                           
#define ESP32RX 25
//Define ESP32 UART TX pin                                             
#define ESP32TX 26                                             
//===Defines===//

void ServerNotFound(AsyncWebServerRequest *request) {
    //When the Server is not found send error 404 and show Server Not Found
    request->send(404, "text/plain", "Server Not found"); 
}

//===Microphone===//
// I2S config - this is set up to read from the left channel
i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// I2S pins
i2s_pin_config_t i2s_pins = {
    //SCK - Serial Clock
    .bck_io_num   = GPIO_NUM_32, 
    //WS - Word Select       
    .ws_io_num    = GPIO_NUM_22,   
    // not used (only for speakers)     
    .data_out_num = I2S_PIN_NO_CHANGE,
    //SD - Serial Data
    .data_in_num  = GPIO_NUM_21};       

I2SSampler *i2s_sampler = NULL;

// Task to write samples to our server
void i2sWriterTask(void *param) {
  I2SSampler *sampler = (I2SSampler *)param;
  WiFiClient *wifi_client = new WiFiClient();
  HTTPClient *http_client = new HTTPClient();
  //Declare 10 ms delay
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(10);  
  while (true) {
    // wait for some samples to save
    //10ms wait/delay
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime); 
    // only start conncetion after time delay
    if (ulNotificationValue > 0) {
      //Start connection to I2S Server
      http_client->begin(*wifi_client, I2S_SERVER_URL); 
      //Send Binary file data                                                
      http_client->addHeader("content-type", "application/octet-stream");     
      //Send to server the raw data from the microphone with the size of the data                          
      http_client->POST((uint8_t *)sampler->getCapturedAudioBuffer(), sampler->getBufferSizeInBytes()); 
      //End connection to I2S Server
      http_client->end();
    }
  }
}
//===Microphone===//

//===GPS===//
void displayInfo(){
  //The isValid() method tells whether the object "gps" contains any valid data and is safe to query
  if (gps.location.isValid()){
    //Save the latitude, longitude and altitude of the GPS to the string
    thisStringGPSLocationTrue = "Latitude: " + String(gps.location.lat(),6) + "\n" 
                                + "Longitude: " + String(gps.location.lng(),6) + "\n" 
                                + "Altitude: " + String(gps.altitude.meters());                   
    //Save only the latitude and longitude of the GPS to the string
    thisStringGPSLocationLL = String(gps.location.lat(),6) + "," + String(gps.location.lng(),6);  
    thisStringGPSLocationFalse = "";
    thisStringGPSLocationLLF = "";
  }
  else{
    //When location is not valid send Location: Not Available
    thisStringGPSLocationFalse = "Location: Not Available"; 
    //When location is not valid send Default location (ORT Hermelin) to show on Google Maps
    thisStringGPSLocationLLF = "32.291772,34.864417";       
  }
  //Save the number of satellites that the GPS is connected to, to the string
  thisStringGPSNOSIU = "Number of satellites in use: " + String(gps.satellites.value()); 
  //Save the speed of the GPS in meters per second to the string
  thisStringGPSSpeed = "Speed: " + String(gps.speed.mps()) + " m/s";                     
}
//===GPS===//

//===Current Sensor===//
void CurrentSensor(){
  //Number Of Samples
  unsigned int x=0, NOS=200;  
  float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0;
  //Get 200 samples
  for (int x = 0; x < NOS; x++){   
    //Read current sensor values
    AcsValue = analogRead(34);        
    //Add samples together
    Samples = Samples + AcsValue;  
    // let ADC settle before next sample for 3ms
    delay(3);                      
  }
  //Taking Average of Samples + Error repair
  AvgAcs=(Samples/float(NOS))+100; 
  //Current equation
  AcsValueF = abs(((AvgAcs * (3.3 / 4096.0)) -2.5 )/0.185);
  //Float to String conversion 
  thisStringCurrent = String(AcsValueF,2); 
}
//===Current-Sensor===//

//===Servo===//
void ServoMotor(){
  //Take the string value of the servo and convert to integer
  thisIntegerServo = thisStringServo.toInt(); 
  //Move the servo to the specified angle
  myservo.write(thisIntegerServo);            
}
//===Servo===//

//===WIFI-CONNECTION+DNS===//
void WIFI_Connection() {
  //Counter for the time that the ESP32 is not connected to a network
  uint32_t notConnectedCounter = 0;       
  //Set ESP32 Hostname
  WiFi.setHostname(HostName.c_str());     
  //Begin Connection to the Hotspot/WiFi network
  WiFi.begin(ssid, password);             
  //Trying to connect to WiFi
  while (WiFi.status() != WL_CONNECTED) { 
    //Wait 1 second before next try
    delay(1000);       
    //Increment variable every second                   
    notConnectedCounter++;  
    // Reset board if not connected after 10s              
    if(notConnectedCounter > 10) {        
        //Resetting due to connection problems
        ESP.restart();                    
    }
  }
  //Print ESP32 IP
  Serial.println(WiFi.localIP());
  //DNS responder started at http://Alberto69_ESP32       
  if (MDNS.begin("Alberto69_ESP32")) {  
    Serial.print("MDNS responder started at http://");
    Serial.println("Alberto69_ESP32");
  }
}
//===WIFI-CONNECTION===//

void setup() {
  //ESP32 defaults to 115200 baud on its serial interface
  Serial.begin(115200);   
  //NEO-6M GPS Chip baud rate is 9600         
  //8N1 - asynchronous mode, in which there is one start bit, eight (8) data bits, no (N) parity bit, and one (1) stop bit      
  //RX - GPIO 25, TX - GPIO 26 (connected oppositely to the GPS)            
  Serial1.begin(9600, SERIAL_8N1, ESP32RX, ESP32TX);                                                      
  //Attach the Servo variable to GPIO 14                         
  myservo.attach(ServoGPIO);  
  //Change Servo angle to 0                       
  myservo.write(0); 
  //Call the function to Start the WIFI-CONNECTION+DNS                                 
  WIFI_Connection();                                 
  
//===HTTP Servers===//
  //Get Servo Angle from LabVIEW through HTTP GET
  server.on("/Servo", HTTP_GET, [] (AsyncWebServerRequest *request) {
    //assign parameter (servo angle) to a pointer
    AsyncWebParameter* p = request->getParam(0); 
    //get pointer value   
    thisStringServo = p->value();   
    //Save the buffer in the string                
    ServoBufferNew = thisStringServo;  
    //Check if Servo angle received from LabVIEW has changed             
    if (ServoBufferNew != ServoBufferOld) {
      //If buffer changed save the new angle         
      ServoBufferOld = ServoBufferNew; 
      //Call the function to move the servo to the specified angle             
      ServoMotor();                                 
    }
    //Send HTTP answer to request
    request->send(200, "text/plain", "Servo Angle: " + thisStringServo);  
  });
  
  //Current sensor measurement 
  server.on("/Current", HTTP_GET, [](AsyncWebServerRequest *request){
    //Call function to get Current Sensor measurement   
    CurrentSensor();                  
    //Send HTTP answer to request            
    request->send(200, "text/plain", "Current: " + thisStringCurrent); 
  });
 
  server.on("/GPS", HTTP_GET, [](AsyncWebServerRequest *request){
    //GPS Full Location+NOSIU(Number of satellites in use)+Speed
    request->send(200, "text/plain", "GPS Data: " + SlashN + thisStringGPSLocationFalse + thisStringGPSLocationTrue 
                                                  + SlashN + thisStringGPSNOSIU
                                                  + SlashN + thisStringGPSSpeed);         
  });
  
  //GPS location (Latitude and Longitude)
  server.on("/Location", HTTP_GET, [](AsyncWebServerRequest *request){
    //Send HTTP answer to request
    request->send(200, "text/plain", thisStringGPSLocationLLF + thisStringGPSLocationLL); 
  });

  //ESP32 IP Address
  server.on("/IP", HTTP_GET, [](AsyncWebServerRequest *request){
    //Send HTTP answer to request  
    request->send(200, "text/plain", WiFi.localIP().toString());                       
  });
  
  //Received Signal Strength Indicator (the lower in absolute value the better)
  server.on("/RSSI", HTTP_GET, [](AsyncWebServerRequest *request){
    //Send HTTP answer to request  
    request->send(200, "text/plain", "RSSI: " + String(WiFi.RSSI()));                 
  });

  server.on("/ESP_Restart", HTTP_GET, [](AsyncWebServerRequest *request){
    //Restart ESP32
    ESP.restart(); 
    //Send HTTP answer to request                                                                        
    request->send(200, "text/plain", "ESP32 Wrvoer B Restarted"); 
  });
  
  //update ESP32 OTA with the inbuilt AsyncElegantOTA function
  AsyncElegantOTA.begin(&server); 
  
  //If the server is not found call ServerNotFound function
  server.onNotFound(ServerNotFound);  

  //Start the server
  server.begin();                     
//===HTTP Servers===//

//===Microphone-Tasks===//
  // set up i2s to read from our microphone
  i2s_sampler = new I2SSampler();

  // set up the i2s sample writer task
  TaskHandle_t writer_task_handle;
  xTaskCreate(i2sWriterTask, "I2S Writer Task", 4096, i2s_sampler, 1, &writer_task_handle);

  // start sampling from i2s device
  i2s_sampler->start(I2S_NUM_1, i2s_pins, i2s_config, 32768, writer_task_handle);
//===Microphone-Tasks===//

}

void loop(){
  //Check if ESP32 is still connected to the Hotspot
  if(WiFi.status() != WL_CONNECTED) { 
    //Reconnect  
    WIFI_Connection();                  
  }
  else {
    //Check if GPS is sending data
    while (Serial1.available() > 0) { 
      //Encode the data  
      if (gps.encode(Serial1.read())) { 
        //Show the Data in human readable language
        displayInfo();                  
      }
    }
  }
}
