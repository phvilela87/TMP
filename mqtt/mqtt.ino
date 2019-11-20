/**
  @file: main.c
  @class: main
  @version: v0r1
  @author: pvilela
  @date: 19/10/2019
  @brief: main class implementation.
  This is the implementation of mqtt broker to interface with nodeMCU board.
  @since  VER   DATE        COMMENTS
  @since  v0r1  19/10/2019  Class creation
*/

/*****************************
    NodeMCU pin definitions
 *****************************
        D0          16
        D1          5
        D2          4
        D3          0
        D4          2
        D5          14
        D6          12
        D7          13
        D8          15
        D9          3
        D10         1
 *****************************/

/*********************************************************
   Includes.
*********************************************************/
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/*********************************************************
   Private definitions.
*********************************************************/
#define DOOR_PIN            13
#define ALARM_PIN           12
#define SCALE_PIN           A0
#define BUZZER_PIN          14
#define SENSOR_PIN          4

#define SERIAL_BAUDRATE     115200

#define WIFI_SSID           "WLL-Inatel"
#define WIFI_PASSWORD       "inatelsemfio"

#define MQTT_SERVER         "broker.mqtt-dashboard.com"
#define MQTT_PORT           1883
#define MQTT_PUB_TOPIC      "icc/truck_report"
#define MQTT_SUB_TOPIC      "icc/truck_command"
#define MQTT_USERNAME       "mosquitto"
#define MQTT_PASSWORD       "mosquitto"
#define MQTT_CLIENT_ID      "phvilela87"

#define BUZZER_ON()         digitalWrite(BUZZER_PIN, HIGH)
#define BUZZER_OFF()        digitalWrite(BUZZER_PIN, LOW)

/*********************************************************
   Private types.
*********************************************************/
WiFiClient wifiClient;
PubSubClient client(MQTT_SERVER, MQTT_PORT, wifiClient);
StaticJsonDocument<256> tx_doc;
StaticJsonDocument<256> rx_doc;

typedef enum enu_BuzzerLoopControl
{
  BUZZER_INIT = 0,
  BUZZER_ENABLE,
  BUZZER_ENABLE_VERIFY,
  BUZZER_DISABLE,
  BUZZER_IDLE,
} tenu_BuzzerLoopControl;

typedef struct tag_truckCtrl
{
  uint8_t                   ucAlarmStatus;
  uint8_t                   ucDoorStatus;
  uint16_t                  ulTruckWeight;
  uint8_t                   ucRxAlarmCtrl;
  uint8_t                   ucRxDoorCommand;
  uint32_t                  ulRxTruckWeight;
  uint32_t                  ulReferenceTime;
  uint32_t                  ulCurrentTime;
  uint32_t                  ulLastMessageArrived;
  tenu_BuzzerLoopControl    enuBuzzerLoopControl;
} ttag_truckCtrl;

/*********************************************************
   Private variables.
*********************************************************/
int8_t status = WL_IDLE_STATUS;

ttag_truckCtrl  tagTruckCtrl;

/*********************************************************
   Private function prototypes.
*********************************************************/
void setup_wifi(void);
void callback(char* topic, unsigned char* payload, unsigned int length);
void system_init(void);
void lock_door(void);
void unlock_door(void);
void activate_alarm(void);
void deactivate_alarm(void);

/*********************************************************
   Private functions.
*********************************************************/
/**
  Main class set-up.

    @param  void
    @return void
    @author pvilela
    @date   20/10/2019
*/
void setup(void)
{
  pinMode(DOOR_PIN, OUTPUT);
  pinMode(ALARM_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SCALE_PIN, INPUT);
  pinMode(SENSOR_PIN, INPUT_PULLUP);

  digitalWrite(DOOR_PIN, HIGH);
  digitalWrite(ALARM_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  system_init();
  unlock_door();
  deactivate_alarm();

  Serial.begin(SERIAL_BAUDRATE);

  delay(10);

  setup_wifi();
  client.setServer( MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);

  // Connecting to MQTT Broker
  if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD))
  {
    Serial.println("Connected to MQTT Broker!");
    client.subscribe(MQTT_SUB_TOPIC);
  }
  else
  {
    Serial.println("Connection to MQTT Broker has failed...");
  }
}

/**
  WiFi set-up.

    @param  void
    @return void
    @author pvilela
    @date   20/10/2019
*/
void setup_wifi(void)
{
  Serial.print("\nConnecting to ");
  Serial.println(WIFI_SSID);

  // Connects to the WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Wait until the connection has been confirmed before continuing
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  // Outputs the IP Address of the ESP8266
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

/**
  Recconects to MQTT broker.

    @param  void
    @return void
    @author pvilela
    @date   20/10/2019
*/
void reconnect(void)
{
  status = WiFi.status();
  if ( status != WL_CONNECTED)
  {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to WiFi");
  }

  while (!client.connected())
  {
    Serial.println("Attempting MQTT connection...");
    if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD))
    {
      Serial.println("MQTT connected");
      client.subscribe(MQTT_SUB_TOPIC);
    }
    else
    {
      Serial.println("Failed to connect, rc=");
      Serial.println(client.state());
      Serial.println("Trying again in 5 seconds...");
      delay(5000);
    }
  }
}

/**
  Initialises all global variables.
    @param  void
    @return void
    @author pvilela
    @date   20/10/2019
*/
void system_init(void)
{
  tagTruckCtrl.ucAlarmStatus = 0;
  tagTruckCtrl.ucDoorStatus = 0;
  tagTruckCtrl.ulTruckWeight = 0;
  tagTruckCtrl.ucRxAlarmCtrl = 0;
  tagTruckCtrl.ucRxDoorCommand = 0;
  tagTruckCtrl.ulRxTruckWeight = 0;
  tagTruckCtrl.ulReferenceTime = 0;
}

/**
  Sends message via MQQT protocol.

    @param  void
    @return void
    @author pvilela
    @date   20/10/2019
*/
void send_message(void)
{
  char tx_buffer[256];

  tagTruckCtrl.ulTruckWeight = get_truckWeight()/100;

  tx_doc["truckWeight"] = tagTruckCtrl.ulTruckWeight;
  tx_doc["alarmStatus"] = tagTruckCtrl.ucAlarmStatus;
  serializeJsonPretty(tx_doc, tx_buffer);

  client.publish(MQTT_PUB_TOPIC, tx_buffer);

  //Serial.println(tx_buffer);
}

/**
  Locks the door.

    @param  void
    @return void
    @author pvilela
    @date   20/10/2019
*/
void lock_door(void)
{
  if ((tagTruckCtrl.ucDoorStatus == 0) && (read_sensor() == 0) && (tagTruckCtrl.ucAlarmStatus == 0))
  {
    tagTruckCtrl.ucDoorStatus = 1;
    digitalWrite(DOOR_PIN, LOW);
    Serial.println("Door has been locked!");
  }
  else if ((tagTruckCtrl.ucDoorStatus == 1) && (tagTruckCtrl.ucAlarmStatus == 0))
  {
    Serial.println("Door is already locked!");
  }
  else if ((tagTruckCtrl.ucDoorStatus == 0) && (read_sensor() == 1) && (tagTruckCtrl.ucAlarmStatus == 0))
  {
    Serial.println("Sensor Failure!");
  }
  else
  {
    Serial.println("Operation not allowed!");
  }
}

/**
  Unlocks the door.

    @param  void
    @return void
    @author pvilela
    @date   20/10/2019
*/
void unlock_door(void)
{
  if ((tagTruckCtrl.ucDoorStatus == 1) && (tagTruckCtrl.ucAlarmStatus == 0))
  {
    tagTruckCtrl.ucDoorStatus = 0;
    digitalWrite(DOOR_PIN, HIGH);
    Serial.println("Door has been unlocked!");
  }
  else if ((tagTruckCtrl.ucDoorStatus == 0) && (tagTruckCtrl.ucAlarmStatus == 0))
  {
    Serial.println("Door is already unlocked!");
  }
  else
  {
    Serial.println("Operation not allowed!");
  }
}

/**
  gets truck weight.

    @param  void
    @return weight in kg.
    @author pvilela
    @date   20/10/2019
*/
uint32_t get_truckWeight(void)
{
  
  return analogRead(SCALE_PIN);
}

/**
  Checks door sensor state.

    @param  void
    @return 0 - if door is closed ; 1 - otherwise
    @author pvilela
    @date   20/10/2019
*/
uint8_t read_sensor(void)
{
  return digitalRead(SENSOR_PIN);
}

/**
  Verifies system integrity.

    @param  void
    @return void
    @author pvilela
    @date   20/10/2019
*/
void verify_system(void)
{
  if ((tagTruckCtrl.ucDoorStatus == 1) && (read_sensor() == 1))
  {
    if (tagTruckCtrl.ucAlarmStatus == 0)
    {
      activate_alarm();
    }
  }
}

/**
  Informs any system violation and sends alarm message via MQTT protocol.

    @param  void
    @return void
    @author pvilela
    @date   20/10/2019
*/
void activate_alarm(void)
{
  if (tagTruckCtrl.ucAlarmStatus == 0)
  {
    tagTruckCtrl.ucAlarmStatus = 1;
    send_message();
    tagTruckCtrl.enuBuzzerLoopControl = BUZZER_ENABLE;
    digitalWrite(ALARM_PIN, HIGH);
    Serial.println("System Violation Alarm!");
  }
}

void deactivate_alarm(void)
{
  if (tagTruckCtrl.ucAlarmStatus == 1)
  {
    tagTruckCtrl.ucAlarmStatus = 0;
    unlock_door();
    tagTruckCtrl.enuBuzzerLoopControl = BUZZER_INIT;
    digitalWrite(ALARM_PIN, LOW);
    Serial.println("Alarm deactivated!");
  }
}

/**
  Buzzer process loop.

    @param  void
    @return void
    @author pvilela
    @date   20/10/2019
*/
void buzzerCtrl_processLoop(void)
{
  switch (tagTruckCtrl.enuBuzzerLoopControl)
  {
    case BUZZER_INIT:
      BUZZER_OFF();
      tagTruckCtrl.enuBuzzerLoopControl = BUZZER_IDLE;
      break;
    case BUZZER_ENABLE:
      BUZZER_ON();
      tagTruckCtrl.enuBuzzerLoopControl = BUZZER_ENABLE_VERIFY;
      tagTruckCtrl.ulReferenceTime = millis();
      break;
    case BUZZER_ENABLE_VERIFY:
      if ((millis() - tagTruckCtrl.ulReferenceTime) > 50)
      {
        BUZZER_OFF();
        tagTruckCtrl.enuBuzzerLoopControl = BUZZER_DISABLE;
        tagTruckCtrl.ulReferenceTime = millis();
      }
      break;
    case BUZZER_DISABLE:
      if ((millis() - tagTruckCtrl.ulReferenceTime) > 200)
      {
        if (tagTruckCtrl.ucAlarmStatus == 1)
        {
          tagTruckCtrl.enuBuzzerLoopControl = BUZZER_ENABLE;
        }
        else
        {
          tagTruckCtrl.enuBuzzerLoopControl = BUZZER_INIT;
        }
      }
      break;
    case BUZZER_IDLE:
      break;
    default:
      break;
  }
}

/**
  Parsers received messages from callback.

    @param  void
    @return void
    @author pvilela
    @date   20/10/2019
*/
void rx_message_parser(void)
{

  if ((tagTruckCtrl.ucRxDoorCommand == 0) && (tagTruckCtrl.ucRxAlarmCtrl == 0))
  {
    unlock_door();
  }

  else if ((tagTruckCtrl.ucRxDoorCommand == 1) && (tagTruckCtrl.ucRxAlarmCtrl == 0))
  {
    lock_door();
  }

  else
  {
    /* Does nothing */
  }


  if (tagTruckCtrl.ucRxAlarmCtrl == 1)
  {
    deactivate_alarm();
  }

  else if (tagTruckCtrl.ucRxAlarmCtrl == 2)
  {
    activate_alarm();
  }

  else
  {
    /*  Does nothing */
  }
}

/**
  Main process loop.

    @param  void
    @return void
    @author pvilela
    @date   20/10/2019
*/
void loop(void)
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  verify_system();
  buzzerCtrl_processLoop();

  tagTruckCtrl.ulCurrentTime = millis();
  if (tagTruckCtrl.ulCurrentTime - tagTruckCtrl.ulLastMessageArrived > 5000)
  {
    send_message();
    tagTruckCtrl.ulLastMessageArrived = tagTruckCtrl.ulCurrentTime;
  }
}

/**
  MQTT callback function.

    @param  topic       pointer to the subscribed topic
            payload     pointer to payload
            length      payload length in bytes
    @return void
    @author pvilela
    @date   20/10/2019
*/
void callback(char* topic, unsigned char* payload, unsigned int length)
{
  DeserializationError err = deserializeJson(rx_doc, payload);

  tagTruckCtrl.ucRxDoorCommand = rx_doc["doorCommand"];
  tagTruckCtrl.ulRxTruckWeight = rx_doc["truckWeight"];
  tagTruckCtrl.ucRxAlarmCtrl = rx_doc["alarmCtrl"];

  if (err)
  {
    //Serial.print("Error: ");
    //Serial.println(err.c_str());
    return;
  }

  rx_message_parser();

  //Serial.println(tagTruckCtrl.ucRxDoorCommand);
  //Serial.println(tagTruckCtrl.ulRxTruckWeight);
  //Serial.println(tagTruckCtrl.ucRxAlarmCtrl);
}
