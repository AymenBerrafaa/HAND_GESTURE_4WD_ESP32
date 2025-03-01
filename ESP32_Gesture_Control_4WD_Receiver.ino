#include <esp_now.h>
#include <WiFi.h>
#include <vector>
int Speed = 255;
int enA = 22, enB = 23;
int IN1 = 16, IN2 = 17, IN3 = 18, IN4 = 19;
int enC = 32, enD = 14;
int IN5 = 33, IN6 = 25, IN7 = 26, IN8 = 27;
int buzzer = 34;
#define SIGNAL_TIMEOUT 1000  
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define STOP 0
unsigned long lastRecvTime = 0;

typedef struct PacketData
{
  byte xAxisValue;
  byte yAxisValue;
  byte zAxisValue;
}PacketData;
PacketData receiverData;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  String inputData ;
  inputData = inputData + "values " + receiverData.xAxisValue + "  " + receiverData.yAxisValue + "  " + receiverData.zAxisValue;
  Serial.println(inputData);

  if (receiverData.zAxisValue > 175)
  {
    processCarMovement(RIGHT);
  }
  else if (receiverData.zAxisValue < 75)
  {
    processCarMovement(LEFT);
  }
  else if (receiverData.yAxisValue < 75)
  {
    processCarMovement(RIGHT);  
  }
  else if (receiverData.yAxisValue > 175)
  {
    processCarMovement(LEFT);     
  }
  else if (receiverData.xAxisValue > 175)
  {
    processCarMovement(BACKWARD);   
  }
  else if (receiverData.xAxisValue < 75)
  {
    processCarMovement(FORWARD);    
  } 
  else
  {
    processCarMovement(STOP);     
  }

  lastRecvTime = millis();   
}
void processCarMovement(int inputValue)
{
  switch(inputValue)
  {
    case FORWARD:
      ledcWrite(0, Speed); ledcWrite(1, Speed);
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
      ledcWrite(2, Speed); ledcWrite(3, Speed);
      digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW);
      digitalWrite(IN7, LOW); digitalWrite(IN8, HIGH);             
      break;
  
    case BACKWARD:
      ledcWrite(0, Speed); ledcWrite(1, Speed);
      digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      ledcWrite(2, Speed); ledcWrite(3, Speed);
      digitalWrite(IN5, LOW); digitalWrite(IN6, HIGH);
      digitalWrite(IN7, HIGH); digitalWrite(IN8, LOW);
      break;
  
    case LEFT:
      ledcWrite(0, Speed); ledcWrite(1, Speed);
      digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
      ledcWrite(2, Speed); ledcWrite(3, Speed);
      digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW);
      digitalWrite(IN7, HIGH); digitalWrite(IN8, LOW);
      break;
      
    case RIGHT:
      ledcWrite(0, Speed); ledcWrite(1, Speed);
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      ledcWrite(2, Speed); ledcWrite(3, Speed);
      digitalWrite(IN5, LOW); digitalWrite(IN6, HIGH);
      digitalWrite(IN7, LOW); digitalWrite(IN8, HIGH);
      break;
  
    case STOP:
      ledcWrite(0, 0); ledcWrite(1, 0);
      digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
      ledcWrite(2, 0); ledcWrite(3, 0);
      digitalWrite(IN5, LOW); digitalWrite(IN6, LOW);
      digitalWrite(IN7, LOW); digitalWrite(IN8, LOW); 
      break;
  }
}
void setup() {
  Serial.begin(115200);
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcSetup(2, 5000, 8);
  ledcSetup(3, 5000, 8);
  ledcAttachPin(enA, 0);
  ledcAttachPin(enB, 1);
  ledcAttachPin(enC, 2);
  ledcAttachPin(enD, 3);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT); pinMode(IN8, OUTPUT);
  pinMode(buzzer, OUTPUT);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() 
{
  //Check Signal lost.
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    processCarMovement(STOP); 
  }
}
