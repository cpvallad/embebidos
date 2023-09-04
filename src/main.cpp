#include <Arduino.h>
#include "EmonLib.h"             // Include Emon Library
#include <driver/adc.h>
#include <Wire.h> 
#include "UbidotsEsp32Mqtt.h"
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <WiFi.h>
#include <ESP_Mail_Client.h>

#define I2C_SDA 21
#define I2C_SCL 22

#define VOLT_CAL 280
#define CURRENT_CAL 50
#define ADC_BITS    10
#define ADC_COUNTS  (1<<ADC_BITS)
#define SMTP_server "smtp.gmail.com"

#define SMTP_Port 465

#define sender_email "YourMonitoringDevice@gmail.com"

#define sender_password "ncbnzfxzvycxqbiy"

#define Recipient_email "crisva212v@gmail.com"

#define Recipient_name ""

#define ALERT_MODE 1
#define AUTOMATIC_MODE 0
SMTPSession smtp;

int btn = 12;
int leda = 26;
int ledr = 27;
float potencia;

/******
 * Define Constants
 ******/
const char *UBIDOTS_TOKEN = "BBFF-HtnhApqTYy7v8w8QgO7ywSN1aBLdqw";  // Put here your Ubidots TOKEN
const char *WIFI_SSID = "NETLIFE-VALLADAREZ";      // Put here your Wi-Fi SSID
const char *WIFI_PASS = "CPVG2001A";      // Put here your Wi-Fi password
const char *DEVICE_LABEL = "Esp32_monitoreo";   // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL = "voltaje"; // sensor de voltaje
const char *VARIABLE_LABEL1 = "corriente"; // sensor de corriente
const char *VARIABLE_LABEL2 = "potencia"; //potencia 
static String htmlMsg;
SMTP_Message message;
ESP_Mail_Session session;
const int PUBLISH_FREQUENCY = 10000; // Update rate in milliseconds
const int LCD_FREQUENCY = 3000;
const int MAIL_FREQUENCY = 120000;

unsigned long timer_ubidots;
unsigned long timer_lcd;
unsigned long timer_mail;
int voltajePin = 36; // Pin used to read data from GPIO34 ADC_CH6.
int corrientePin = 39; // Pin used to read data from GPIO34 ADC_CH6.
Ubidots ubidots(UBIDOTS_TOKEN);
float value;
float value1;
float value2;
unsigned long lastDebounceTime = 0; // the last time the button was pressed
unsigned long debounceDelay = 500;   // the debounce time; increase if needed
int flag = 0;
/******
 * Auxiliar Functions
 ******/
void callback(char *topic, byte *payload, unsigned int length);
void SendingMail();
void StartingEmail();
void StartingUbiDots();
void writeEnergyToDisplay(double voltage, double amps);
void LedsOff();
void Leds_Alert_Mode_Ok();
void Leds_Alert_NotOk();
static int action = 0;
LiquidCrystal_I2C lcd(0x27, 20, 4); //LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
EnergyMonitor emon1;             // Create an instance

void IRAM_ATTR buttonInterrupt() {
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime > debounceDelay) {
     if(action == AUTOMATIC_MODE){
      action= ALERT_MODE;
     }else{
      action= AUTOMATIC_MODE;
     }
    lastDebounceTime = currentTime;
  }
}
void setup()
{  
  Serial.begin(9600);
  StartingUbiDots();
  // ubidots.setDebug(true);  // uncomment this to make debug messages available

  lcd.init(I2C_SDA, I2C_SCL); // initialize the lcd to use user defined I2C pins
	lcd.backlight();
  pinMode(btn,INPUT_PULLUP);
  pinMode(leda, OUTPUT);
  pinMode(ledr, OUTPUT);
  emon1.voltage(35, VOLT_CAL, 1.7);  // Voltage: input pin, calibration, phase_shift
  emon1.current(32, CURRENT_CAL);       // Current: input pin, calibration.
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  analogReadResolution(10);
  attachInterrupt(digitalPinToInterrupt(btn),buttonInterrupt,FALLING);
  StartingEmail();
  timer_ubidots = millis(); 
  timer_lcd = millis();
  timer_mail =millis();
}

void loop()
{
  emon1.calcVI(20,2000);         // Calculate all. No.of half wavelengths (crossings), time-out
  float currentDraw      = emon1.Irms;             //extract Irms into Variable
  float supplyVoltage   = emon1.Vrms;                   //extract Vrms into Variable
  if(currentDraw >1.3){
    currentDraw = currentDraw-0.3;
   }
  if (action == ALERT_MODE){
    if(currentDraw > 1.3 || supplyVoltage > 130 || supplyVoltage < 90){
      if(flag ==0){
        if(currentDraw > 1.3){
          htmlMsg = "<div style=\"color:#000000;\"><h1> ALERTA, REVISAR AMPERAJE!</h1><p> Valor de Amperaje fuera del rango definido </p></div>";
          }
        if(supplyVoltage > 130 || supplyVoltage < 90){
          htmlMsg = "<div style=\"color:#000000;\"><h1> ALERTA, REVISAR VOLTAJE!</h1><p> Valor de Voltaje fuera del rango establecido</p></div>";
          }
         Serial.println("Encendiendo LED de problema");
         Leds_Alert_NotOk();
         SendingMail();
         timer_mail = millis();
         flag =1;
      }else{
      if (abs(int (millis() - timer_mail)) > MAIL_FREQUENCY) // triggers LCD every 3 seconds
        {
          htmlMsg = "<div style=\"color:#000000;\"><h1> ALERTA, REVISAR INSTALACION!</h1><p> Valores fuera de Rango</p></div>";  
          SendingMail();
          timer_mail = millis();
        }
      }          
    }
    else 
    {
      Leds_Alert_Mode_Ok();
    } 
  }else{
    flag =0;
    LedsOff();
    }
  if (abs(int (millis() - timer_lcd)) > LCD_FREQUENCY) // triggers LCD every 3 seconds
  {
    Serial.print("Voltage: ");
    Serial.println(supplyVoltage);
    Serial.print("Current: ");
    Serial.println(currentDraw);
    Serial.print("Watts: ");
    potencia = currentDraw * supplyVoltage;
    Serial.println(potencia);
    Serial.println("\n\n");
    writeEnergyToDisplay(supplyVoltage,currentDraw);
    timer_lcd = millis();
  }

  if (!ubidots.connected())
  {
    ubidots.reconnect();
  }

  if (abs(int (millis() - timer_ubidots)) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {  
    ubidots.add(VARIABLE_LABEL, supplyVoltage); // Insert your variable Labels and the value to be sent
    ubidots.publish(DEVICE_LABEL);   
    ubidots.add(VARIABLE_LABEL1, currentDraw); // Insert your variable Labels and the value to be sent
    ubidots.publish(DEVICE_LABEL);
    ubidots.add(VARIABLE_LABEL2, potencia); // Insert your variable Labels and the value to be sent
    ubidots.publish(DEVICE_LABEL);
    timer_ubidots = millis();
  }
  
  ubidots.loop();    
}

void StartingEmail(){
  Serial.print("Connecting...");

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED)

  { Serial.print(".");

    delay(200);

   }

  Serial.println("");

  Serial.println("WiFi connected.");

  Serial.println("IP address: ");

  Serial.println(WiFi.localIP());

  Serial.println();

  smtp.debug(1);



  session.server.host_name = SMTP_server ;

  session.server.port = SMTP_Port;

  session.login.email = sender_email;

  session.login.password = sender_password;

  session.login.user_domain = "";

}
void SendingMail(){
  message.sender.name = "IOT DEVICE";

  message.sender.email = sender_email;

  message.subject = "Alerta de Monitoreo";

  message.addRecipient(Recipient_name,Recipient_email);
  message.html.content = htmlMsg.c_str();

  message.html.content = htmlMsg.c_str();

  message.text.charSet = "us-ascii";

  message.html.transfer_encoding = Content_Transfer_Encoding::enc_7bit;
    if (!smtp.connect(&session))

    return;

  if (!MailClient.sendMail(&smtp, &message))

    Serial.println("Error sending Email, ");
}
void StartingUbiDots(){
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  if(ubidots.connected()) Serial.println("conectado");
  else Serial.println("no conectado");
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
}
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
void writeEnergyToDisplay(double voltage, double amps){
  // MODIFICAR PARA UN DISPLAY BONITO
  lcd.clear();  
  lcd.setCursor(1, 1); // Start from column 3, first two are broken :/
  lcd.print("Voltaje:");
  lcd.print(voltage);
  lcd.setCursor(1,2);
  lcd.print("Amps");
  lcd.print(amps);
  if(action ==1){
    lcd.setCursor(1,3);
    lcd.print("Mode: ");
    lcd.print("Aler");
  }else{
    lcd.setCursor(1,3);
    lcd.print("Mode: ");
    lcd.print("Auto");
  } 
}
void LedsOff(){
  digitalWrite(ledr, LOW);
  digitalWrite(leda,LOW);
}
void Leds_Alert_Mode_Ok(){
  digitalWrite(ledr, LOW);
  digitalWrite(leda, HIGH);
}
void Leds_Alert_NotOk(){
  digitalWrite(ledr, HIGH);
  digitalWrite(leda, LOW);
}