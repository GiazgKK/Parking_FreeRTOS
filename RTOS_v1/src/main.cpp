#include <Arduino_FreeRTOS.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <queue.h>

#define RST_PIN 9
#define SS_PIN 10
String checkinUID = "43a2fefc";
String checkoutUID = "7345ef7"; // Thay thế "your_checkout_uid" bằng UID thẻ RFID cho chức năng check-out
String currentUID = "";
Servo myservo1;
Servo myservo2;
const int buzzer = 6;
int pos = 0;
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define NUMBER_OF_SPOT 1
bool sensorStatus[NUMBER_OF_SPOT] = {false};
QueueHandle_t StatusQueue = xQueueCreate(NUMBER_OF_SPOT, sizeof(sensorStatus));

MFRC522 mfrc522(SS_PIN, RST_PIN);

enum SystemState
{
  IDLE,
  CHECKIN,
  CHECKOUT
};
SystemState currentState = IDLE;

void handleInvalidCard();
void handleCheckin();
void handleCheckout();

void HandleRFID(void *pvParameters);
void ReadSensor(void *pvParameters);
void DisplayLCD(void *pvParameters);
void Servo_control(void *pvParameters);

void setup()
{

  // initialize serial communication at 9600 bits per second:

  Serial.begin(9600);
  SPI.begin();
  mfrc522.PCD_Init();
  myservo1.attach(7);
  myservo2.attach(8);
  pinMode(buzzer, OUTPUT);
  lcd.init();
  lcd.backlight();
  lcd.print("Parking system!!");
  // delay(3000);
  // lcd.clear();
  xTaskCreate(ReadSensor, "ReadSensor", 128, NULL, 1, NULL);
  xTaskCreate(HandleRFID, "HandleRFID", 128, NULL, 2, NULL);

  xTaskCreate(DisplayLCD, "DisplayLCD", 128, NULL, 1, NULL);
  // xTaskCreate(Servo_control, "Servo_control", 128, NULL, 1, NULL );

  vTaskStartScheduler();
}

void loop() {}

void HandleRFID(void *pvParameters)
{
  while (1)
  {
    Serial.println("HandleRFID Task ");
    if (mfrc522.PICC_IsNewCardPresent())
    {
      if (mfrc522.PICC_ReadCardSerial())
      {
        Serial.print("The UID: ");
        for (byte i = 0; i < mfrc522.uid.size; i++)
        {
          currentUID = currentUID + String(mfrc522.uid.uidByte[i], HEX);
        }
        Serial.println('check UID');
        Serial.println(currentUID);

        // Handle card operations based on the currentUID
        if (currentUID == checkinUID)
        {
          handleCheckin();
        }
        else if (currentUID == checkoutUID)
        {
          handleCheckout();
        }
        else
        {
          handleInvalidCard();
        }

        // Reset the currentUID and clear the card data
        currentUID = "";
        mfrc522.PICC_HaltA();
      }
    }

    // Allow other tasks to run by yielding the processor
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void ReadSensor(void *pvParameters)
{
  bool sensorStatusTx[NUMBER_OF_SPOT] = {false};
  while (1)
  {
    Serial.println("ReadSensor Task ");
    // Đọc trạng thái của tất cả các cảm biến hồng ngoại
    for (int i = 2; i < NUMBER_OF_SPOT + 2; i++)
    {
      int val = digitalRead(i);
      Serial.print("ReadSensor ");
      Serial.print(i - 1);
      Serial.print(" :");
      Serial.print(val);
      Serial.println("");
      sensorStatusTx[i - 2] = val;
    }
    xQueueSend(StatusQueue, &sensorStatusTx, 0);
    // Tạm dừng 100ms
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void DisplayLCD(void *pvParameters)
{
  lcd.clear();
  bool sensorStatusRx[NUMBER_OF_SPOT] = {false};
  while (1)
  {
    Serial.println("DisplayLCD Task ");
    int spot;
    int spot_counter = 1;
    xQueueReceive(StatusQueue, &sensorStatusRx, 0);
    for (spot = 0; spot < NUMBER_OF_SPOT; spot++)
    {
      lcd.setCursor(spot, 0);
      lcd.print(spot_counter);
      lcd.setCursor(spot, 1);
      if (!sensorStatusRx[spot])
      {
        lcd.print('X');
        Serial.print(sensorStatusRx[spot]);
      }
      else
      {
        lcd.print('O');
        Serial.print(sensorStatusRx[spot]);
      }
      spot_counter++;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// void Servo_control(void *pvParameters)
//  {
//    while(1)
//    {
//      for (pos = 0; pos <= 90; pos += 1) {
//        myservo2.write(pos);
//        //delay(100);
//        vTaskDelay(100 / portTICK_PERIOD_MS);
//      }
//    }
//  }

void handleCheckin()
{
  if (currentState == IDLE)
  {
    currentState = CHECKIN;
    digitalWrite(buzzer, HIGH);
    // delay(200);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(buzzer, LOW);
    for (pos = 0; pos <= 90; pos += 1)
    {
      myservo1.write(pos);

      // delay(100);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    currentState = IDLE;
    // lcd.print(currentState);
  }
}

void handleCheckout()
{
  if (currentState == IDLE)
  {
    currentState = CHECKOUT;
    digitalWrite(buzzer, HIGH);
    // delay(200);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(buzzer, LOW);
    for (pos = 0; pos <= 90; pos += 1)
    {
      myservo2.write(pos);
      // delay(100);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    // Perform checkout actions here
    currentState = IDLE;
  }
}

void handleInvalidCard()
{
  if (currentState == IDLE)
  {
    currentState = IDLE;
    digitalWrite(buzzer, HIGH);
    // delay(1000);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(buzzer, LOW);
  }
}