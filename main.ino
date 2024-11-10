#include <Keypad.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#define RST_PIN 5 // Configurable, see typical pin layout above
#define SS_PIN 53 // Configurable, see typical pin layout above
MFRC522 mfrc522(SS_PIN, RST_PIN); // Create MFRC522 instance
#include <Stepper.h>
const int stepsPerRevolution = 2038; // the number of step in 1 revolution according to 28BYJ-48
// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 10,9,11);
float stepPerangle = 5.66111111; //step per angle
float anGle = 90; //set angle

String pwdcreate = "";
const int builtinLED = 13;
String password = "3210";
int servoState = 0;
Servo myservo;
LiquidCrystal_I2C lcd(0x27, 16, 2);
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'0','1','2','3'},
  {'4','5','6','7'},
  {'8','9','A','B'},
  {'C','D','E','F'}
};
byte rowPins[ROWS] = {34, 33, 32, 31}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {38, 37, 36, 35}; //connect to the column pinouts of the keypad

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

const int trigPin = 6;  // Ultrasonic Trig pin
const int echoPin = 7; // Ultrasonic Echo pin
const int irObsPin = 4; // IR obstacle Out pin
const int R = 44; // Red channel LED
const int G = 45; // Green channel LED
const int B = 46; // Blue channel LED

// defines variables
long duration;
int distance;

void setup() {
  lcd.begin(); 
  Wire.begin();

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  pinMode (builtinLED, OUTPUT); // built-in LED pin set to output
  pinMode (irObsPin, INPUT); // module signal output connected to Arduino pin 8

  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);

  SPI.begin(); // Init SPI bus
  mfrc522.PCD_Init(); // Init MFRC522 card
  Serial.println(F("Read personal data on a MIFARE PICC:")); //shows in serial that it is ready to read

  Serial.begin(9600); // Starts the serial communication
}

void loop() {
  lcd.setCursor(0,0);
  lcd.print("PASS: ");
  // -- keypad + servo stuffs --
  char customKey = customKeypad.getKey();
  if (customKey){
    //Serial.println(customKey);
    pwdcreate = pwdcreate + customKey;
    Serial.println(pwdcreate);
    lcd.setCursor(6,0);
    lcd.print(pwdcreate);
    if (pwdcreate.length() == password.length()) {
      Serial.print("Entered password: ");
      Serial.print(pwdcreate);
      if (password == pwdcreate) { 
        Serial.print("\nStatus: Correct\n");
        // lcd.setCursor(0,1);
        // lcd.print("STATUS:");
        // lcd.print("CORRECT  ");
        // digitalWrite(builtinLED, HIGH);
        analogWrite(R, 0);
        analogWrite(G, 255);
        analogWrite(B, 0);
        delay(1000);
        analogWrite(R, 0);
        analogWrite(G, 0);
        analogWrite(B, 255);
        delay(100);

        if (servoState == 0) {
          myservo.attach(3);
          myservo.write(180);
          delay(1000);
          myservo.detach();
          servoState = 1;
        }
        else {
          myservo.attach(3);
          myservo.write(0);
          delay(1000);
          myservo.detach();
          servoState = 0;
        }
        
        pwdcreate = "";
        lcd.setCursor(6,0);
        lcd.print("    ");
      }
      else { 
        Serial.print("\nStatus: Wrong\n");
        // lcd.setCursor(0,1);
        // lcd.print("STATUS:");
        // lcd.print("INCORRECT");
        // digitalWrite(builtinLED, LOW);
        // myservo.attach(3);
        // myservo.write(0);
        // delay(1000);
        // myservo.detach();
        analogWrite(R, 255);
        analogWrite(G, 0);
        analogWrite(B, 0);
        delay(1000);
        analogWrite(R, 0);
        analogWrite(G, 0);
        analogWrite(B, 255);
        delay(100);
        pwdcreate = "";
        lcd.setCursor(6,0);
        lcd.print("    ");
      }
    }
  }
  else {
      analogWrite(R, 0);
      analogWrite(G, 0);
      analogWrite(B, 255);
      delay(100);
  }

  // -- Ultrasonic stuffs --
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  // Serial.print("Distance: ");
  // Serial.println(distance);

  // IR obstacle tracking
  // if (digitalRead(irObsPin) == LOW) { // if module detects an obstacle,
  // Serial.println("Obstacle Detected"); // show message on serial monitor and
  // digitalWrite (builtinLED, HIGH); // switch-On built-in LED
  // }
  // else {
  // Serial.println("No Obstacle");
  // digitalWrite (builtinLED, LOW);
  // }

  // -- empty lot count  --
  if (digitalRead(irObsPin) == LOW && distance <= 5) {
  // analogWrite(R, 255);
  // analogWrite(G, 0);
  // analogWrite(B, 0);
  // delay(100);
  lcd.setCursor(0,1);
  lcd.print("EMPTY LOT: 0");
  }
  else if (digitalRead(irObsPin) == LOW || distance <= 5) {
    // analogWrite(R, 0);
    // analogWrite(G, 0);
    // analogWrite(B, 255);
    // delay(100);
    lcd.setCursor(0,1);
    lcd.print("EMPTY LOT: 1");
  }
  else {
    // analogWrite(R, 0);
    // analogWrite(G, 255);
    // analogWrite(B, 0);
    // delay(100);
    lcd.setCursor(0,1);
    lcd.print("EMPTY LOT: 2");
  }

  // -- RFID stuffs --
  // Prepare key - all keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
  MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;
  //some variables we need
  byte block;
  byte len;
  MFRC522::StatusCode status;
  // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
  if ( ! mfrc522.PICC_IsNewCardPresent()) {
  return;
  }
  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial()) {
  return;
  }
  Serial.println(F("**Card Detected:**"));
  if (servoState == 0) {
          myservo.attach(3);
          myservo.write(180);
          delay(1000);
          myservo.detach();
          servoState = 1;
        }
  else {
          myservo.attach(3);
          myservo.write(0);
          delay(1000);
          myservo.detach();
          servoState = 0;
  }
  // myStepper.setSpeed(1); // 360 degree per minute
  // myStepper.step(stepPerangle*anGle); //turn 90 degree = 15second
  // delay(1000);
  // Serial.print("/nDone rotate");
  //-------------------------------------------
  //mfrc522.PICC_DumpDetailsToSerial(&(mfrc522.uid)); //dump some details about the card
  //mfrc522.PICC_DumpToSerial(&(mfrc522.uid)); //uncomment this to see all blocks in hex
  //-------------------------------------------
  
  // Serial.println(servoState);
}
