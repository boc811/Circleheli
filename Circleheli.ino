/*
Circleheli
by Ben and Bjoern 2023
*/
#include <IBusBM.h>
#include <Servo.h>
#include <NewPing.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <LiquidCrystal_I2C.h>
#pragma region Library Settings
// Motorshield Create the DC motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which plugs/ports the DC Motor should use M1, M2, M3 or M4.
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
// M1 Kran Motor, M2 Linear Motor, M3 Magnet

// PID settings
double mSetpoint = 27, mInput, mOutput, sSetpoint = 0, sInput, sOutput;
PID motorPID(&mInput, &mOutput, &mSetpoint, 4, 1, 3, DIRECT);
PID steerPID(&sInput, &sOutput, &sSetpoint, 2, 1, 2, DIRECT);

// create IBus objekt fpr the communication from the RC receiver to arduino
IBusBM IBus; // IBus object

// create servo
Servo motor; // create servo object to control a Motor
Servo steer; // create servo object to controö the servo for steering

// ultra sonar settings
NewPing sonar(12, 11, 70); // TRIGGER_PIN, ECHO_PIN, maximum distance in cm.

// encoder settings
Encoder myEnc(2, 3);

// 1602 Display -> initialize  the library with the numbers of the interface pins
// LiquidCrystal_I2C lcd(0x27, 16, 2);
LiquidCrystal_I2C lcd(0x3F, 16, 2);
#pragma endregion Library Settings
void setup()
{

  //Serial.begin(115200); //aktivate serial communikation to serial  monitor

  // PID settings
  motorPID.SetOutputLimits(1000, 1700); // min max values
  motorPID.SetMode(AUTOMATIC);
  steerPID.SetOutputLimits(1100, 1900); // min max values
  steerPID.SetMode(AUTOMATIC);

  // IBus settings
  IBus.begin(Serial, IBUSBM_NOTIMER); // iBUS from FS-IA6B connected to Serial0

  // servo settings
  motor.attach(9);               // attaches motor-Servo to pin 9
  motor.writeMicroseconds(1000); // set brushless motor to minimum power
  steer.attach(10);              // attaches steering-Servo to pin 10
  steer.writeMicroseconds(1500); // set steerig to middle position

  // DC Motor Settings
  AFMS.begin(); // create with the default frequency 1.6KHz
  // AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor1->setSpeed(0);
  myMotor1->run(FORWARD);
  // turn on motor
  myMotor1->run(RELEASE);

  myMotor2->setSpeed(0);
  myMotor2->run(FORWARD);
  // turn on motor
  myMotor2->run(RELEASE);

  myMotor3->setSpeed(0);
  myMotor3->run(FORWARD);
  // turn on motor
  myMotor3->run(RELEASE);

  // Display 1602 I2C Settingns
  lcd.init();      // initialize the lcd
  lcd.backlight(); // Turn on backlight
                   // lcd.print("Dreh-Heli");// Print a message to the LCD
}
#pragma region Defining variables
// variables for brushless Motor, Steering Servos and Sensors
int motor_val = 1000, steer_val = 1500, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10;              // acual values
int save_motor_val = 1000, save_steer_val, save_ch5, save_ch6, save_ch7, save_ch8, save_ch9, save_ch10; // values for comparison, that function will only be called if value changed
bool mH = 0, sS = 0;

// variables for DC Motor and Magnet
int saveval1 = 0, saveval2 = 0, saveval3 = 0, val1 = 0, val2 = 0, val3 = 0, i = 0;

// variables for LIPO 3S Spannungsmessung
int AD_Pin = A0; // select the Arduino AD input
double Lipo3SVolt;

// ultra sonic variable settings
int trigger = 7;
int dist = 0, dist_pre = 0;
// ultra sonic variable settings to avoid the delay function
long myTimer = 0;
long myTimeout = 30; // this value determines the time distance between two function calls of the sonar ultra sonic sensor in ms (minimum 29ms) used instead of delay

// encoder variable settimgs
long oldPosition = 0;

// variable for serial communication to reduce traffic
int t = 0;

// variablen für cycle count
unsigned long cycleTime = 0, cycleTimesave = 0;
long cycleCount = 0;

#pragma endregion Defining variables
void loop()
{
#pragma region Variables for IBus Channels and Lipo3svolt
  // ibus reading and saving it to variables
  ch1 = IBus.readChannel(0);                      // get latest value for servo channel 1 -> Hauptlenkung Servo
  ch2 = IBus.readChannel(1);                      // get latest value for servo channel 2 -> DC Motor Kranseil hoch/runter /Note: Reverse an Fernebdienung
  ch3 = IBus.readChannel(2);                      // get latest value for servo channel 3 -> Brushless Motor
  ch4 = IBus.readChannel(3);                      // get latest value for servo channel 4 -> Linear Motor Verschiebung Seil vor/zurück
  ch5 = IBus.readChannel(4);                      // get latest value for servo channel 5 -> Kontrolle über die Höhenregelung
  ch6 = IBus.readChannel(5);                      // get latest value for servo channel 6 -> Kontrolle über die Kreis-Positionsregelung
  ch7 = IBus.readChannel(6);                      // get latest value for servo channel 7 -> Speicherung des Höhen-Sollwertes
  ch8 = IBus.readChannel(7);                      // get latest value for servo channel 8 -> Speicherung des Kreis-Position Sollwertes
  ch9 = IBus.readChannel(8);                      // get latest value for servo channel 9 ->  Höhenregelung ab 1500 an/ Höhen + Kreisregelung ab 2000 an
  ch10 = IBus.readChannel(9);                     // get latest value for servo channel 10 -> Hebe Magnet an
  Lipo3SVolt = analogRead(AD_Pin) * 0.0555 - 0.2; // get latest AD Value from A0 Input for 3sLipo (Spannnungsteiler mit 100k und 10K vorgeschaltet)
#pragma endregion
#pragma region Ultrasonic
  // ultrasonic reading. millis() is used to avoid the delay function
  if (millis() > myTimeout + myTimer)
  {
    dist = sonar.ping_cm();
    myTimer = millis();
    //dist = constrain(((dist_pre - 19) * 1.95), 0, 80); // kalibrierung Armlänge zu Distance(Höhe)
  }

#pragma endregion Ultrasonic
#pragma region Encoder
  long actualPosition = myEnc.read(); // read the position of the encoder
#pragma endregion encoder
#pragma region Height Regulation
  if (ch7 == 2000) // if function to set the Setpoint for Height Regulation
  {
    mSetpoint = dist;
  }
  if (ch5 > 1500) // if functions to increase or decrease  the Setpoint for Height Regulation
  {
    mSetpoint = mSetpoint + 0.002;
  }
  if (ch5 > 1850)
  {
    mSetpoint = mSetpoint + 0.01;
  }
  if (ch5 < 1500 and ch5 > 999)
  {
    mSetpoint = mSetpoint - 0.002;
  }
  if (ch5 < 1150 and ch5 > 999)
  {
    mSetpoint = mSetpoint - 0.01;
  }
  if (ch9 < 1450) // if function to decide if the Motor PID starts or if the remote control is working
  {
    motor_val = ch3;
    mOutput = 1000; // to ensure that the height regulation starts at low values
    mH = 1;
    motorPID.SetMode(MANUAL); // Sets PID Values to default
  }
  else  //Height regulation starts here
  {
    if (mH == 1)
    {
      mOutput = ch3;
      motorPID.SetMode(AUTOMATIC);
      mH = 0;
    }
    mInput = dist;
    motorPID.Compute();
    motor_val = mOutput;
  }
#pragma endregion Height Regulation
#pragma region Rotational Regulation

  if (ch8 == 2000) // if function to set the Setpoint for Rotational Position
  {
    sSetpoint = actualPosition;
  }
  if (ch6 > 1500) // if functions to increase or decrease  the Setpoint for Rotational Position
  {
    sSetpoint = sSetpoint + 0.005;
  }
  if (ch6 > 1850)
  {
    sSetpoint = sSetpoint + 0.4;
  }
  if (ch6 < 1500 and ch6 > 999)
  {
    sSetpoint = sSetpoint - 0.005;
  }
  if (ch6 < 1150 and ch6 > 999)
  {
    sSetpoint = sSetpoint - 0.4;
  }
  if (ch9 < 1950) // if function to decide if the steering PID starts or if the remote control is working
  {
    steer_val = ch1;
    steerPID.SetMode(MANUAL);
    sS = 1;
  }
  else //steer regulation starts here
  {
    if (sS == 1)
    { sSetpoint=sSetpoint+2400*(round((float)(actualPosition-sSetpoint)/2400.0)); //Formel damit der kuerzeste Weg gefunden wird
      steerPID.SetMode(AUTOMATIC);
      sS = 0;
    }
    sInput = actualPosition;
    steerPID.Compute();
    steer_val = sOutput;
  }
#pragma endregion Rotational Regulation
#pragma region value changed perform function
  // if function to print and perform an action if the value changed
  if ((save_motor_val != motor_val) || (dist != 0) || (save_steer_val != steer_val) || (actualPosition != oldPosition) || (save_ch7 != ch7) || (save_ch8 != ch8) || (save_ch9 != ch9) || (save_ch10 != ch10))
  {

    // motor and steering oonly be called if the values are changed. See if function above
    // motor_val = constrain(motor_val, 0, 1200); // Bregrenzung der Motorleistung zum Testen
    motor.writeMicroseconds(motor_val);        // sets the brushless motor
    if (steer_val == 0)
    {
      steer_val = 1500;
    } // sets the steering to the middle position if ibus is not connected or started
    steer.writeMicroseconds(steer_val);

    // saving the new values to check next time if something changed.see if function above
    save_motor_val = motor_val;
    save_steer_val = steer_val;
    save_ch7 = ch7;
    save_ch8 = ch8;
    save_ch9 = ch9;
    save_ch10 = ch10;
    oldPosition = actualPosition;
  }
#pragma endregion value changed perform function
#pragma region DC Motor and Magnet Steering
  // DC Motor and Magnet Steering
  int val1;
  val1 = ch2; // get latest values for Kran Motor
  if ((val1 == 0))
  {
    val1 = 1500;
  }; // Damit der Motor bei 0 nicht losläuft
  val1 = map(val1, 1000, 2000, -255, 255);
  int val2;
  val2 = ch4; // get latest value Linearmotor
  if ((val2 == 0))
  {
    val2 = 1500;
  }; // Damit der Motor bei 0 nicht losläuft
  val2 = map(val2, 1000, 2000, -255, 255);
  int val3;
  val3 = ch10; // get latest values
  if ((val3 == 2000))
  {
    val3 = 2000;
  };

  if ((saveval1 != val1) && (val1 > 125))
  {
    myMotor1->run(FORWARD);
    myMotor1->setSpeed(abs(val1));
    saveval1 = val1;
  }
  if ((saveval1 != val1) && (val1 < -125))
  {
    myMotor1->run(BACKWARD);
    myMotor1->setSpeed(abs(val1));
    saveval1 = val1;
  }
  if (abs(val1) <= 125)
  {
    myMotor1->setSpeed(0);
  }

  if ((saveval2 != val2) && (val2 > 125))
  {
    myMotor2->run(FORWARD);
    myMotor2->setSpeed(abs(val2));
    saveval2 = val2;
  }
  if ((saveval2 != val2) && (val2 < -125))
  {
    myMotor2->run(BACKWARD);
    myMotor2->setSpeed(abs(val2));
    saveval2 = val2;
  }
  if (abs(val2) <= 125)
  {
    myMotor2->setSpeed(0);
  }

  if ((saveval3 != val3) && (val3 > 1600))
  {
    myMotor3->run(FORWARD);
    myMotor3->setSpeed(abs(2000));
    saveval3 = val3;
  }
  if ((saveval3 != val3) && (val3 < 1600))
  {
    myMotor3->run(BACKWARD);
    myMotor3->setSpeed(abs(0));
    saveval3 = val3;
  }
#pragma endregion DC Motor and Magnet Steering
#pragma region LCD Display
  // Display LCD 1602 Übertragung jedes 100mal i==100
  if (i == 100)
  {
    lcd.setCursor(0, 0);
    lcd.print(mSetpoint, 0); // 19cm offset abziehen
    lcd.print(" ");
    lcd.setCursor(5, 0);
    lcd.print(sSetpoint, 0);
    lcd.print("  ");
    lcd.setCursor(12, 0);     // set the cursor to column 11, line 0
    lcd.print(Lipo3SVolt, 1); //
    lcd.setCursor(0, 1);
    // lcd.print((dist - 19)*2); //19cm offset abziehen
    lcd.print(dist); // 19cm offset abziehen
    lcd.print(" ");
    lcd.setCursor(5, 1);
    lcd.print(actualPosition);
    lcd.print("  ");
    i = 0;
  }
  i = i + 1;
#pragma endregion LCD Display
#pragma region IBUS Loop
  // iBus function call
  IBus.loop();
#pragma endregion IBUS Loop
#pragma region serial print values
  // Display values with serial monitor not too often
 /* if (t == 1000)
  {
    Serial.print(round((float)(actualPosition-sSetpoint)/2400.0));
    Serial.print("\t");
    Serial.print(ch3);
    Serial.print("\t");
    Serial.print(mOutput);
    Serial.print("\t");
    Serial.print(motor_val);
    Serial.print("\t");
    Serial.print(cycleTime);
    Serial.print(" milliseconds for 10000");
    Serial.println();
    t = 0;
  }
  t = t + 1;
  */
#pragma endregion serial print values
#pragma region cycle time
  // cycle time
 /* if (cycleCount >= 10000)
  {
    cycleTime = ((micros() - cycleTimesave) / 1000);
    cycleTimesave = micros();
    cycleCount = 0;
  }
  cycleCount++;
  */
#pragma endregion cycle time
}
