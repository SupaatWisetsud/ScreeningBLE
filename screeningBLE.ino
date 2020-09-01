#include <math.h>
#include <JPEGDecoder.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEService *pService;
BLECharacteristic *pCharacteristic;


#include <ArduinoJson.h>
#include <base64.h>
#include <SD.h>
#include <SPI.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

#include "logoKKU.h"
#include "menu.h"
#include "about.h"
#include "measure.h"
#include "bluetooth.h"

#include <TFT_eFEX.h>              // Include the extension graphics functions library
TFT_eFEX  fex = TFT_eFEX(&tft);    // Create TFT_eFX object "efx" with pointer to "tft" object

//====================================================================
// Return the minimum of two values a and b
#define minimum(a,b)     (((a) < (b)) ? (a) : (b))
//====================================================================

String CurrentPage = "init";
float percentage,percentage_count = 0, percentage_all_step = 80, Io, I, Ibg, total_Io525,total_Io660, total_I525, total_I660;
//calibrate
float calibrate_percentage, calibrate_Io, calibrate_I, calibrate_Ibg, 
      calibrate_total_Io525, calibrate_total_Io660, calibrate_total_I525, calibrate_total_I660;
      
int LED525 = 12, LED660 = 14;

bool statusBluetooth = false;
int countDevice = 0;

String json_string;
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" lux"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" lux"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution, 4); Serial.println(F(" lux"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2591
*/
/**************************************************************************/
void configureSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  //tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
   tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Gain:         "));
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println(F("1x (Low)"));
      break;
    case TSL2591_GAIN_MED:
      Serial.println(F("25x (Medium)"));
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println(F("428x (High)"));
      break;
    case TSL2591_GAIN_MAX:
      Serial.println(F("9876x (Max)"));
      break;
  }
  Serial.print  (F("Timing:       "));
  Serial.print((tsl.getTiming() + 1) * 100, DEC); 
  Serial.println(F(" ms"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
}

void advancedRead(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("IR: ")); Serial.print(ir);  Serial.print(F("  "));
  Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  "));
  Serial.print(F("Visible: ")); Serial.print(full - ir); Serial.print(F("  "));
  Serial.print(F("Lux: ")); Serial.println(tsl.calculateLux(full, ir), 6);
}

//####################################################################################################
// Print image information to the serial port (optional)
//####################################################################################################
void jpegInfo() {
  Serial.println(F("==============="));
  Serial.println(F("JPEG image info"));
  Serial.println(F("==============="));
  Serial.print(F(  "Width      :")); Serial.println(JpegDec.width);
  Serial.print(F(  "Height     :")); Serial.println(JpegDec.height);
  Serial.print(F(  "Components :")); Serial.println(JpegDec.comps);
  Serial.print(F(  "MCU / row  :")); Serial.println(JpegDec.MCUSPerRow);
  Serial.print(F(  "MCU / col  :")); Serial.println(JpegDec.MCUSPerCol);
  Serial.print(F(  "Scan type  :")); Serial.println(JpegDec.scanType);
  Serial.print(F(  "MCU width  :")); Serial.println(JpegDec.MCUWidth);
  Serial.print(F(  "MCU height :")); Serial.println(JpegDec.MCUHeight);
  Serial.println(F("==============="));
}

//####################################################################################################
// Draw a JPEG on the TFT, images will be cropped on the right/bottom sides if they do not fit
//####################################################################################################
// This function assumes xpos,ypos is a valid screen coordinate. For convenience images that do not
// fit totally on the screen are cropped to the nearest MCU size and may leave right/bottom borders.
void renderJPEG(int xpos, int ypos) {

  // retrieve infomration about the image
  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = minimum(mcu_w, max_x % mcu_w);
  uint32_t min_h = minimum(mcu_h, max_y % mcu_h);

  // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;

  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();

  // save the coordinate of the right and bottom edges to assist image cropping
  // to the screen size
  max_x += xpos;
  max_y += ypos;

  // read each MCU block until there are no more
  while (JpegDec.read()) {
    
    // save a pointer to the image block
    pImg = JpegDec.pImage ;

    // calculate where the image block should be drawn on the screen
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;  // Calculate coordinates of top left corner of current MCU
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;

    // check if the image block size needs to be changed for the right edge
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;

    // check if the image block size needs to be changed for the bottom edge
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;

    // copy pixels into a contiguous block
    if (win_w != mcu_w)
    {
      uint16_t *cImg;
      int p = 0;
      cImg = pImg + win_w;
      for (int h = 1; h < win_h; h++)
      {
        p += mcu_w;
        for (int w = 0; w < win_w; w++)
        {
          *cImg = *(pImg + w + p);
          cImg++;
        }
      }
    }

    // calculate how many pixels must be drawn
    uint32_t mcu_pixels = win_w * win_h;

    tft.startWrite();

    // draw image MCU block only if it will fit on the screen
    if (( mcu_x + win_w ) <= tft.width() && ( mcu_y + win_h ) <= tft.height())
    {

      // Now set a MCU bounding window on the TFT to push pixels into (x, y, x + width - 1, y + height - 1)
      tft.setAddrWindow(mcu_x, mcu_y, win_w, win_h);

      // Write all MCU pixels to the TFT window
      while (mcu_pixels--) {
        // Push each pixel to the TFT MCU area
        tft.pushColor(*pImg++);
      }

    }
    else if ( (mcu_y + win_h) >= tft.height()) JpegDec.abort(); // Image has run off bottom of screen so abort decoding

    tft.endWrite();
  }

  // calculate how long it took to draw the image
  drawTime = millis() - drawTime;

  // print the results to the serial port
  Serial.print(F(  "Total render time was    : ")); Serial.print(drawTime); Serial.println(F(" ms"));
  Serial.println(F(""));
}

//####################################################################################################

void drawArrayJpeg(const uint8_t arrayname[], uint32_t array_size, int xpos, int ypos) {

  int x = xpos;
  int y = ypos;

  JpegDec.decodeArray(arrayname, array_size);
  
  jpegInfo(); // Print information from the JPEG file (could comment this line out)
  
  renderJPEG(x, y);
  
  Serial.println("#########################");
}

//#############transfer data#########################
void transferData(const String& datapackage){

  byte i = 0;
  
  while(i < datapackage.length()){
    
    if((i + 3) >= datapackage.length()){
      pCharacteristic->setValue(datapackage.substring(i, datapackage.length()).c_str());
      pCharacteristic->notify(); 
    }else{
      pCharacteristic->setValue(json_string.substring(i, i+3).c_str());
      pCharacteristic->notify(); 
    }
    
    i += 3;
    delay(50);
  }

}


//######################################################################
// init sd card and check sd card
//######################################################################
void checkSDCard() {
  if(!SD.begin()){
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

//#######################################################################
// setup bluetooth
//#######################################################################

//Server Callback
class MyServerCallback: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer){
    Serial.println("Client Connected");
          
    if(countDevice == 0) statusBluetooth = true;
      
    countDevice += 1;
  }

  void onDisconnect(BLEServer* pServer){
    Serial.println("Client Disconnected");

    if(countDevice == 1) statusBluetooth = false;

    countDevice -= 1;
  }
};

//Characteristic Callback
class MyCharacteristicCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      String message = "";
      
      if (value.length() > 0) {
        Serial.println("*********");
        Serial.print("New value: ");
        
        for (int i = 0; i < value.length(); i++){
          message += value[i];
          Serial.print(value[i]);
        }
        Serial.println("");
        Serial.println("*********");
  
//        Serial.println(message);

        StaticJsonDocument<512> doc;

        DeserializationError err = deserializeJson(doc, message);
        
        if(err){
          Serial.print("ArduinoJson Error: ");
          Serial.println(err.c_str());
        }

        //command-************************
        if(doc["command"] == "b"){

          tft.setCursor(185, 225, 2);
          tft.setTextColor(TFT_WHITE,TFT_BLACK);  
          tft.setTextSize(2);
          tft.printf("Calibrate");
          
          tft.setTextColor(TFT_WHITE);
          tft.setTextSize(1); 
          calibrate525();
          calibrate660();
          
          json_string = "{\"A525\":" + String(calibrate_total_Io525/10) + ", \"A660\":" + String(calibrate_total_Io660/10) + "}";
          
          transferData(json_string);
          
          tft.fillScreen(0xFFFFF);
          drawArrayJpeg(bluetooth, sizeof(bluetooth), 0, 0);
          
          calibrate_percentage = 0;
          json_string = "";
        }

        if(doc["command"] == "s"){
          
        }

        //end command-*******************************
        
        pCharacteristic->setValue("end");
        pCharacteristic->notify();
        
      }
      
      message = "";
    }
    
};

//setup Bluetooth
void setupBluetooth(){
  
  Serial.println("Starting BLE work!");
  
  BLEDevice::init("ScreeningCancer");
  BLEServer *pServer = BLEDevice::createServer();

  pServer->setCallbacks(new MyServerCallback());
  
  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  pCharacteristic->addDescriptor(new BLE2902());

  pCharacteristic->setCallbacks(new MyCharacteristicCallback());
  pCharacteristic->setValue("Hello Device");
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

//=========================== Calibrate 525======================================
void calibrate525(){
  
  for(int i = 1; i <= 10 ; i++){
    calibrate_Ibg = tsl.getLuminosity(TSL2591_VISIBLE);
    
    calibrate_percentage += 2.5; 
    
    fex.drawProgressBar(96, 266, 300, 30, calibrate_percentage, 0x000000, TFT_BLUE);
    tft.setCursor(210, 272, 2);
    tft.fillRect(208,270,45,20,TFT_BLUE);
    tft.printf("%.1f %%", calibrate_percentage);
    
    //LED 525 ON
    digitalWrite(LED525, HIGH);
    delay(200); //ensure light turned on
    calibrate_Io = tsl.getLuminosity(TSL2591_VISIBLE); //measure 500ms
    //LED 525 OFF
    digitalWrite(LED525, LOW);
    
    calibrate_percentage += 2.5; 
    fex.drawProgressBar(96, 266, 300, 30, calibrate_percentage, 0x000000, TFT_BLUE);
    tft.setCursor(210, 272, 2);
    tft.fillRect(208,270,45,20,TFT_BLUE);
    tft.printf("%.1f %%", calibrate_percentage);
    
//    calibrate_total_Io525 += calibrate_Io - calibrate_Ibg;
    calibrate_total_Io525 = 3414.215 ;
  }
}

//==========================calibrate 660=============================
void calibrate660(){
   
   
   for(int i = 1; i <= 10 ; i++){
    calibrate_Ibg = tsl.getLuminosity(TSL2591_VISIBLE);
    
    calibrate_percentage += 2.5; 
    fex.drawProgressBar(96, 266, 300, 30, calibrate_percentage, 0x000000, TFT_BLUE);
    tft.setCursor(210, 272, 2);
    tft.fillRect(208,270,45,20,TFT_BLUE);
    tft.printf("%.1f %%", calibrate_percentage);

    //LED 660 ON
    digitalWrite(LED660, HIGH);
    delay(200); //ensure light turned on
    calibrate_Io = tsl.getLuminosity(TSL2591_VISIBLE); //measure 500ms
    //LED 660 OFF
    digitalWrite(LED525, LOW);
    
    calibrate_percentage += 2.5; 
    fex.drawProgressBar(96, 266, 300, 30, calibrate_percentage, 0x000000, TFT_BLUE);
    tft.setCursor(210, 272, 2);
    tft.fillRect(208,270,45,20,TFT_BLUE);
    tft.printf("%.1f %%", calibrate_percentage);
    
//    calibrate_total_Io660 += calibrate_Io - calibrate_Ibg;
    calibrate_total_Io660 = 255.218;
  }
}
//====================================================================
void setup(void) {
  pinMode(LED525, OUTPUT);
  pinMode(LED660, OUTPUT);

  digitalWrite(LED525, LOW);
  digitalWrite(LED660, LOW);

  pinMode(LED_BUILTIN,OUTPUT);
  
  //////////////////////SENSOR////////////////////////////////
  //sda,scl
  Wire.begin(25,26);
  if (tsl.begin()) 
  {
    Serial.println(F("Found a TSL2591 sensor"));
  } 
  else 
  {
    Serial.println(F("No sensor found ... check your wiring?"));
    while (1);
  }
    
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Configure the sensor */
  configureSensor();
  
  ////////////////////TFT//////////////////////////////////
  
  Serial.begin(115200);
  Serial.println("\n\nStarting...");

  uint16_t PosX, PosY;

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(0xFFFF);
  tft.setTextColor(0xFFFFFF);
  
  // Draw a jpeg image stored in memory at x,y
  drawArrayJpeg(logoKKU, sizeof(logoKKU), 0, 0);

  /* setup bluetooth */
  setupBluetooth();

   /* check sd card */
  checkSDCard();
}

//====================================================================

void loop() {
  
  uint16_t PosX, PosY;
  
  if(!statusBluetooth && CurrentPage == "bluetooth"){
    tft.fillScreen(0xFFFFFF);
    drawArrayJpeg(menu, sizeof(menu), 0, 0);
    CurrentPage = "menu";
    PosX = 1000, PosY = 1000;
  }
  
  if(!statusBluetooth && CurrentPage != "bluetooth"){
    
    static uint16_t color;
    boolean pressed = tft.getTouch(&PosX, &PosY);
  //  //inverse y position
  //  PosY = 320-PosY;
  
    //measure , measure525 and other page except menu cant go to menu
    if (pressed && CurrentPage != "menu" && CurrentPage != "measure" && CurrentPage != "measure525") {
      tft.fillScreen(0xFFFFFF);
      drawArrayJpeg(menu, sizeof(menu), 0, 0);
      CurrentPage = "menu";
      PosX = 1000, PosY = 1000;
    }
  
    if (PosX >=90 && PosX <=190 && PosY >=10 && PosY <=140 && CurrentPage == "menu") {
      tft.fillScreen(0xFFFFF);
      drawArrayJpeg(about, sizeof(about), 0, 0);
      CurrentPage = "about";
    }
  
    ////////////////////////////////////////////////////////////////////////////
    if (PosX >=290 && PosX <=400 && PosY >=160 && PosY <=300 && CurrentPage == "menu") {
      PosX = 1000, PosY = 1000;
      CurrentPage = "measure";
      
      tft.fillScreen(0xFFFFF);
      drawArrayJpeg(measure, sizeof(measure), 0, 0);
   
      tft.setCursor(225, 27, 4);
      tft.fillRect(220,20,200,36,0x000000); 
      tft.printf("PRESS 525nm");
    }
  
    ///Io :: STEP I
    if (PosX >=50 && PosX <=200 && PosY >=200 && PosY <=250 && CurrentPage == "measure") {
      PosX = 1000, PosY = 1000;
      total_Io525 = 0; total_Io660 = 0; total_I525 = 0; total_I660 = 0;
      CurrentPage = "measure525";
  
      tft.setCursor(225, 27, 4);
      tft.fillRect(220,20,200,36,0x000000); 
      tft.printf("Wait for a few sec.");
  
      //525 Io
      for(int i = 1; i <= 10 ; i++){
        Ibg = tsl.getLuminosity(TSL2591_VISIBLE);
        percentage_count++;
        percentage = percentage_count*100/percentage_all_step;
        fex.drawProgressBar(96, 266, 300, 30, percentage, 0xFFFFFF, TFT_BLUE);
        tft.setCursor(210, 272, 2);
        tft.fillRect(208,270,45,20,TFT_BLUE);
        tft.printf("%.1f %%", percentage);
        
        tft.setCursor(75, 140, 4);
        tft.fillRect(75,140,148,26,0x000000);
        tft.printf("%.0f", Ibg);
  
        //LED 525 ON
        digitalWrite(LED525, HIGH);
        delay(200); //ensure light turned on
        Io = tsl.getLuminosity(TSL2591_VISIBLE); //measure 500ms
        //LED 525 OFF
        digitalWrite(LED525, LOW);
        percentage_count++;
        percentage = percentage_count*100/percentage_all_step;
        fex.drawProgressBar(96, 266, 300, 30, percentage, 0xFFFFFF, TFT_BLUE);
        tft.setCursor(210, 272, 2);
        tft.fillRect(208,270,45,20,TFT_BLUE);
        tft.printf("%.1f %%", percentage);
        
        tft.setCursor(75, 140, 4);
        tft.fillRect(75,140,148,26,0x000000);
        tft.printf("%.0f", Io);
  
        total_Io525 += Io - Ibg;
      }
            
      tft.setCursor(75, 140, 4);
      tft.fillRect(75,140,148,26,0x000000);
      tft.printf("%.2f", total_Io525/10);  
  
      //660 Io
      for(int i = 1; i <= 10 ; i++){
        Ibg = tsl.getLuminosity(TSL2591_VISIBLE);
        percentage_count++;
        percentage = percentage_count*100/percentage_all_step;
        fex.drawProgressBar(96, 266, 300, 30, percentage, 0xFFFFFF, TFT_BLUE);
        tft.setCursor(210, 272, 2);
        tft.fillRect(208,270,45,20,TFT_BLUE);
        tft.printf("%.1f %%", percentage);
        
        tft.setCursor(310, 140, 4);
        tft.fillRect(310,140,148,26,0x000000);
        tft.printf("%.0f", Ibg);
  
        //LED 660 ON
        digitalWrite(LED660, HIGH);
        delay(200); //ensure light turned on
        Io = tsl.getLuminosity(TSL2591_VISIBLE); //measure 500ms
        //LED 660 OFF
        digitalWrite(LED660, LOW);
        
        percentage_count++;
        percentage = percentage_count*100/percentage_all_step;
        fex.drawProgressBar(96, 266, 300, 30, percentage, 0xFFFFFF, TFT_BLUE);
        tft.setCursor(210, 272, 2);
        tft.fillRect(208,270,45,20,TFT_BLUE);
        tft.printf("%.1f %%", percentage);
        
        tft.setCursor(310, 140, 4);
        tft.fillRect(310,140,148,26,0x000000);
        tft.printf("%.0f", Io);
  
        total_Io660 += Io - Ibg;
      }
  
      tft.setCursor(310, 140, 4);
      tft.fillRect(310,140,148,26,0x000000); 
      tft.printf("%.2f", total_Io660/10); 
   
      tft.setCursor(225, 28, 2);
      tft.fillRect(220,20,200,36,0x000000); 
      tft.printf("Insert sample and press 525nm");
    }//if STEP I
     
    ///525nm I  :: STEP II
    if (PosX >=50 && PosX <=200 && PosY >=200 && PosY <=250 && CurrentPage == "measure525") {
      PosX = 1000, PosY = 1000;
      tft.setCursor(225, 27, 4);
      tft.fillRect(220,20,200,36,0x000000); 
      tft.printf("Measuring 525nm");
      
      for(int i = 1; i <= 10 ; i++){
        Ibg = tsl.getLuminosity(TSL2591_VISIBLE);
        percentage_count++;
        percentage = percentage_count*100/percentage_all_step;
        fex.drawProgressBar(96, 266, 300, 30, percentage, 0xFFFFFF, TFT_BLUE);
        tft.setCursor(210, 272, 2);
        tft.fillRect(208,270,45,20,TFT_BLUE);
        tft.printf("%.1f %%", percentage);
        
        tft.setCursor(75, 178, 4);
        tft.fillRect(75,178,148,26,0x000000);
        tft.printf("%.0f", Ibg);
  
        //LED 525 ON
        digitalWrite(LED525, HIGH);
        delay(200); //ensure light turned on
        I = tsl.getLuminosity(TSL2591_VISIBLE); //measure 500ms
        //LED 525 OFF
        digitalWrite(LED525, LOW);
        
        percentage_count++;
        percentage = percentage_count*100/percentage_all_step;
        fex.drawProgressBar(96, 266, 300, 30, percentage, 0xFFFFFF, TFT_BLUE);
        tft.setCursor(210, 272, 2);
        tft.fillRect(208,270,45,20,TFT_BLUE);
        tft.printf("%.1f %%", percentage);
  
        tft.setCursor(75, 178, 4);
        tft.fillRect(75,178,148,26,0x000000);
        tft.printf("%.0f", I);
  
        total_I525 += I - Ibg;
      }
  
      tft.setCursor(75, 178, 4);
      tft.fillRect(75,178,148,26,0x000000);
      tft.printf("%.2f", total_I525/10);
      
      tft.setCursor(75, 213, 4);
      tft.printf("%.8f", log(total_I525/total_Io525));
  
      tft.setCursor(225, 28, 4);
      tft.fillRect(220,20,200,36,0x000000); 
      tft.printf("Measuring 660nm");
  
      ///660nm I  :: STEP III
      for(int i = 1; i <= 10 ; i++){
        Ibg = tsl.getLuminosity(TSL2591_VISIBLE);
        percentage_count++;
        percentage = percentage_count*100/percentage_all_step;
        fex.drawProgressBar(96, 266, 300, 30, percentage, 0xFFFFFF, TFT_BLUE);
        tft.setCursor(210, 272, 2);
        tft.printf("%.1f %%", percentage);
        
        tft.setCursor(310, 178, 4);
        tft.fillRect(310,178,148,26,0x000000);
        tft.printf("%.0f", Ibg);
        
        //LED 660 ON
        digitalWrite(LED660, HIGH);
        delay(200); //ensure light turned on
        I = tsl.getLuminosity(TSL2591_VISIBLE); //measure 500ms
        //LED 660 OFF
        digitalWrite(LED660, LOW);
        
        percentage_count++;
        percentage = percentage_count*100/percentage_all_step;
        fex.drawProgressBar(96, 266, 300, 30, percentage, 0xFFFFFF, TFT_BLUE);
        tft.setCursor(210, 272, 2);
        tft.printf("%.1f %%", percentage);
  
        tft.setCursor(310, 178, 4);
        tft.fillRect(310,178,148,26,0x000000);
        tft.printf("%.0f", I);
  
        total_I660 += I - Ibg;
      }
    
      tft.setCursor(310, 178, 4);
      tft.fillRect(310,178,148,26,0x000000);
      tft.printf("%.2f", total_I660/10);
      
      tft.setCursor(310, 213, 4);
      tft.printf("%.8f", log(total_I660/total_Io660));
  
      //RATIO    
      tft.setCursor(100, 269, 4);
      tft.fillRect(60,263,375,35,0xad8ad4);
      tft.setTextColor(0xFFFFFF);
      tft.printf("A525:A660 = %.8f", log(total_I525/total_Io525) / log(total_I660/total_Io660));
  
  //    float AAVG = log(total_I525/total_Io525) / log(total_I660/total_Io660);
  //    float A525log = log(total_I525/total_Io525);
  //    float A660log = log(total_I660/total_Io660)
  //    //send data via bluetooth*******************************************************************************
  //    SerialBT.println("{A525: %.8f, A660: %.8f, AVG: %.8f}", A525log, A660log, AAVG);
      
      tft.setCursor(225, 28, 4);
      tft.fillRect(220,20,210,36,0x000000); 
      tft.printf("Completed !!");
  
      delay(3000);
  
      tft.setCursor(225, 28, 4);
      tft.fillRect(220,20,210,36,0x000000); 
      tft.printf("Press to Menu");
  
      //set other page to back menu
      CurrentPage = "tmp";
      //reset value
      percentage_count = 0; total_Io525 = 0; total_Io660 = 0; total_I525 = 0;total_I660 = 0;
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    if (CurrentPage == "menu") {
  //    Serial.printf("x: %i     ", PosX);
  //    Serial.printf("y: %i     ", PosY);
  //    Serial.printf("z: %i \n", tft.getTouchRawZ());
    
      if (pressed) {
        tft.setCursor(5, 5, 2);
        tft.fillRect(2,3,50,36,0x000000); 
        tft.setCursor(5, 5, 2);
        tft.printf("x: %i     ", PosX);
        tft.setCursor(5, 20, 2);
        tft.printf("y: %i    ", PosY);
     
        //tft.drawPixel(PosX, PosY, color); 320-Y //inverse y position
        tft.fillEllipse(PosX, 320-PosY, 5, 5, color);
        color += 55;
      }
    }
  }else{
    
    if(statusBluetooth && CurrentPage != "bluetooth"){
      tft.fillScreen(0xFFFFF);
      drawArrayJpeg(bluetooth, sizeof(bluetooth), 0, 0);
      CurrentPage = "bluetooth";
    }

    
  }

} // end loop
