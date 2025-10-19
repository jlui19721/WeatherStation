/***************************************************************************
  This is a library for the BME680 gas, humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/3660

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "Adafruit_LTR329_LTR303.h"

#include <WiFi.h>
#include <WebServer.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

#define SEALEVELPRESSURE_HPA (1013.25)

class WeatherStation
{
private:
    Adafruit_BME680 bme_;    ///< Adafruit BME-680 Sensor (Temp, Humidity, etc.)
    Adafruit_LTR329 ltr_;    ///< Adafruit LTR329 Sensor (Light)

    void InitializeBME680()
    {
        if (!bme_.begin()) {
          Serial.println("Could not find a valid BME680 sensor, check wiring!");
          while (1) delay(10);
        }
        Serial.println("Found BME sensor!");

        // Set up oversampling and filter initialization
        bme_.setTemperatureOversampling(BME680_OS_8X);
        bme_.setHumidityOversampling(BME680_OS_2X);
        bme_.setPressureOversampling(BME680_OS_4X);
        bme_.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme_.setGasHeater(320, 150); // 320*C for 150 ms
    }

    void InitializeLTR329()
    {
      if (!ltr_.begin()) {
        Serial.println("Couldn't find LTR sensor!");
        while (1) delay(10);
      }
      Serial.println("Found LTR sensor!");

      ltr_.setGain(LTR3XX_GAIN_2);
      Serial.print("Gain : ");
        switch (ltr_.getGain()) {
        case LTR3XX_GAIN_1: Serial.println(1); break;
        case LTR3XX_GAIN_2: Serial.println(2); break;
        case LTR3XX_GAIN_4: Serial.println(4); break;
        case LTR3XX_GAIN_8: Serial.println(8); break;
        case LTR3XX_GAIN_48: Serial.println(48); break;
        case LTR3XX_GAIN_96: Serial.println(96); break;
      }

      ltr_.setIntegrationTime(LTR3XX_INTEGTIME_100);
      Serial.print("Integration Time (ms): ");
      switch (ltr_.getIntegrationTime()) {
        case LTR3XX_INTEGTIME_50: Serial.println(50); break;
        case LTR3XX_INTEGTIME_100: Serial.println(100); break;
        case LTR3XX_INTEGTIME_150: Serial.println(150); break;
        case LTR3XX_INTEGTIME_200: Serial.println(200); break;
        case LTR3XX_INTEGTIME_250: Serial.println(250); break;
        case LTR3XX_INTEGTIME_300: Serial.println(300); break;
        case LTR3XX_INTEGTIME_350: Serial.println(350); break;
        case LTR3XX_INTEGTIME_400: Serial.println(400); break;
      }

      ltr_.setMeasurementRate(LTR3XX_MEASRATE_200);
      Serial.print("Measurement Rate (ms): ");
      switch (ltr_.getMeasurementRate()) {
        case LTR3XX_MEASRATE_50: Serial.println(50); break;
        case LTR3XX_MEASRATE_100: Serial.println(100); break;
        case LTR3XX_MEASRATE_200: Serial.println(200); break;
        case LTR3XX_MEASRATE_500: Serial.println(500); break;
        case LTR3XX_MEASRATE_1000: Serial.println(1000); break;
        case LTR3XX_MEASRATE_2000: Serial.println(2000); break;
      }
    }

  public:
    WeatherStation()
      : bme_(&Wire),
        ltr_()
    {
      // Start-up Little Filesystem
      if (!LittleFS.begin()) {
        Serial.println("An Error has occured while mounting LittleFS");
      }

      InitializeBME680();
      InitializeLTR329();
    }

    String readTemperature() {
        // Read temperature as a Celsius (the default)
        float t = bme_.readTemperature();
        // Convert temperature to Fahrenheit
        t = 1.8 * t + 32;

        if (isnan(t)) {
            Serial.println("Failed to read temperature from BME-680");
            return "";
        } else {
            Serial.println("Temp: " + String(t) + " F");
            return String(t);
        }
    }

    String readHumidity() {
      float h = bme_.readHumidity();
      if (isnan(h)) {
        Serial.println("Failed to read humidity from BME-680");
        return "";
      } else {
        Serial.println("Humidity: " + String(h) + " %");
        return String(h);
      }
    }

    String readPressure() {
      float p = bme_.readPressure() / 100.0F;
      if (isnan(p)) {
        Serial.println("Failed to read pressure from BME-680");
        return "";
      }

      Serial.println("Pressure: " + String(p) + " hPa");
      return String(p);
    }

    String readGas() {
      uint32_t g = bme_.readGas() / 1000.0;
      Serial.println("Gas: " + String(g) + " KOhms");
      return String(g);
    }

    String readAltitude() {
      float a = bme_.readAltitude(SEALEVELPRESSURE_HPA);
      if (isnan(a)) {
        Serial.println("Failed to read altitude from BME-680");
        return "";
      }
      Serial.print("Approx. Altitude: " + String(a) + " m");
      return String(a);
    }

    String readLight() {
      bool valid;
      uint16_t visible_plus_ir, infrared;

      if (ltr_.newDataAvailable()) {
        valid = ltr_.readBothChannels(visible_plus_ir, infrared);
        if (valid) {
          Serial.print("CH0 Visible + IR: ");
          Serial.print(visible_plus_ir);
          Serial.print("\t\tCH1 Infrared: ");
          Serial.println(infrared);

          return String(visible_plus_ir) + "," + String(infrared);
        }
      }

      Serial.println("Failed to read light from LTR-329");
      return "";
    }
};

// Wi-Fi credentials
const char* SSID = "Troy and Abed in the Modem";
const char* PASSWORD = "His-Name-Is-Karl";

AsyncWebServer server(80);
WeatherStation* weatherStation = nullptr;

void SetupWebServer()
{
    // Connect to local Wi-F- network
    WiFi.begin(SSID, PASSWORD);

    // Check wi-fi is connected to wi-fi network
    while (WiFi.status() != WL_CONNECTED) {
        Serial.println("Attempting to connect to " + WiFi.SSID());
        delay(1000);
    }

    Serial.println("Wi-Fi connected to " + WiFi.SSID() + "!");
    Serial.print("Local IP: ");
    Serial.println(WiFi.localIP());

    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(LittleFS, "/index.html");
    });

    // Poll BME-680 Sensor
    server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", weatherStation->readTemperature().c_str());
    });

    server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", weatherStation->readHumidity().c_str());
    });

    server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", weatherStation->readPressure().c_str());
    });

    server.on("/gas", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", weatherStation->readGas().c_str());
    });

    server.on("/altitude", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", weatherStation->readAltitude().c_str());
    });

    // Poll LTR-329 Sensor
    server.on("/light", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", weatherStation->readLight().c_str());
    });

    server.begin();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  weatherStation = new WeatherStation;
  SetupWebServer();  // Connect to Wi-Fi & launch server
}

void loop() {
}