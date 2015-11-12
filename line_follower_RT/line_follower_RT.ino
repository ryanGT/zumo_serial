/*
 * Demo line-following code for the Pololu Zumo Robot
 *
 * This code will follow a black line on a white background, using a
 * PID-based algorithm.  It works decently on courses with smooth, 6"
 * radius curves and has been tested with Zumos using 30:1 HP and
 * 75:1 HP motors.  Modifications might be required for it to work
 * well on different courses or with different motors.
 *
 * http://www.pololu.com/catalog/product/2506
 * http://www.pololu.com
 * http://forum.pololu.com
 */
#define numsensors 6

#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <ZumoMotors.h>
//#include <ZumoBuzzer.h>
#include <Pushbutton.h>


//ZumoBuzzer buzzer;
ZumoReflectanceSensorArray reflectanceSensors;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);
int lastError = 0;

unsigned int sensors[numsensors];
int position;
int error;

// This is the maximum speed the motors will be allowed to turn.
// (400 lets the motors go at top speed; decrease to impose a speed limit)

// note that A0 is pin 14, so 18 is A4
#define proxPin 18
#define otherPin 19

const int MAX_SPEED = 400;

int n=550;
int oldiff=400;
int max_n=5000;
int inByte;
int fresh;
int v;
int state;
bool send_ser;
int nISR=0;
int vL;
int vR;
int nIn;
int v_out;

void setup()
{
  // Play a little welcome song
  //buzzer.play(">g32>>c32");

  //=======================================================
  // set up the Timer2 interrupt to trigger at 100Hz
  //=======================================================
  cli();          // disable global interrupts
  //OCR0A = 124;
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 250;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  //  TCCR2B |= (1 << CS21);
  bitSet(TCCR2B, CS22);
  bitSet(TCCR2B, CS21);
  bitSet(TCCR2B, CS20);
  //!//bitClear(TCCR2B, CS20);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  sei();// re-enable global interrupts
  //=======================================================

  Serial.begin(115200);

  pinMode(proxPin, OUTPUT);
  pinMode(otherPin, OUTPUT);
  //Serial.print("zumo serial");
  //Serial.print("\n");

  // Initialize the reflectance sensors module
  reflectanceSensors.init();

  // Wait for the user button to be pressed and released
  //button.waitForButton();

  // Turn on LED to indicate we are in calibration mode
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  motors.setSpeeds(0,0);

  // Turn off LED to indicate we are through with calibration
  digitalWrite(13, LOW);
  //buzzer.play(">g32>>c32");

  // Wait for the user button to be pressed and released
  //button.waitForButton();
  digitalWrite(proxPin, HIGH);
}


void calibrate_sensor(){
  //Wait 0.5 seconds and then begin automatic sensor calibration
  //by rotating in place to sweep the sensors over the line
  delay(500);
  int i;
  int cal_speed = 300;
  for(i = 0; i < 80; i++)
  {
    if ((i > 10 && i <= 30) || (i > 50 && i <= 70))
      motors.setSpeeds(-cal_speed, cal_speed);
    else
      motors.setSpeeds(cal_speed, -cal_speed);
    reflectanceSensors.calibrate();

    // Since our counter runs to 80, the total delay will be
    // 80*20 = 1600 ms.
    delay(20);
  }
  motors.setSpeeds(0,0);
}

unsigned char getsecondbyte(int input){
    unsigned char output;
    output = (unsigned char)(input >> 8);
    return output;
}

 

int reassemblebytes(unsigned char msb, unsigned char lsb){
    int output;
    output = (int)(msb << 8);
    output += lsb;
    return output;
}

int readtwobytes(void){
    unsigned char msb, lsb;
    int output;
    int iter = 0;
    while (Serial.available() <2){
      iter++;
      if (iter > 1e5){
	break;
      }
    }
    msb = Serial.read();
    lsb = Serial.read();
    output = reassemblebytes(msb, lsb);
    return output;
}

void SendTwoByteInt(int intin){
    unsigned char lsb, msb;
    lsb = (unsigned char)intin;
    msb = getsecondbyte(intin);
    Serial.write(msb);
    Serial.write(lsb);
}



void loop()
{
  position = reflectanceSensors.readLine(sensors);
  // Get the position of the line.  Note that we *must* provide the "sensors"
  // argument to readLine() here, even though we are not interested in the
  // individual sensor readings

  
  // Our "error" is how far we are away from the center of the line, which
  // corresponds to position 2500.
  error = position - 2500;
  
    // Get motor speed difference using proportional and derivative PID terms
    // (the integral term is generally not very useful for line following).
    // Here we are using a proportional constant of 1/4 and a derivative
    // constant of 6, which should work decently for many Zumo motor choices.
    // You probably want to use trial and error to tune these constants for
    // your particular Zumo and line course.
    /* int speedDifference = error / 4 + 6 * (error - lastError); */
  
    /* lastError = error; */
  
    /* // Get individual motor speeds.  The sign of speedDifference */
    /* // determines if the robot turns left or right. */
    /* int m1Speed = MAX_SPEED + speedDifference; */
    /* int m2Speed = MAX_SPEED - speedDifference; */
  
    /* // Here we constrain our motor speeds to be between 0 and MAX_SPEED. */
    /* // Generally speaking, one motor will always be turning at MAX_SPEED */
    /* // and the other will be at MAX_SPEED-|speedDifference| if that is positive, */
    /* // else it will be stationary.  For some applications, you might want to */
    /* // allow the motor speed to go negative so that it can spin in reverse. */
    /* if (m1Speed < 0) */
    /*   m1Speed = 0; */
    /* if (m2Speed < 0) */
    /*   m2Speed = 0; */
    /* if (m1Speed > MAX_SPEED) */
    /*   m1Speed = MAX_SPEED; */
    /* if (m2Speed > MAX_SPEED) */
    /*   m2Speed = MAX_SPEED; */
  if ( fresh > 0 ){
    digitalWrite(proxPin, HIGH);
    //digitalWrite(otherPin, LOW);
    fresh = 0;
  
    if ( nISR > max_n){
      motors.setSpeeds(0,0);
    }
    else{
      motors.setSpeeds(vL,vR);
    }

    if (send_ser){
      //send_ser = false;
      SendTwoByteInt(nISR);
      //SendTwoByteInt(v_out);
      for (int q=0; q<numsensors; q++) {
	SendTwoByteInt(sensors[q]);
      }
      SendTwoByteInt(error);
      Serial.write(10);
    }
    if (nISR > 5000){
      send_ser = false;
      vL = 0;
      vR = 0;
    }
  /*   //digitalWrite(otherPin, LOW); */
  digitalWrite(proxPin, LOW);
  }// end if fresh
  
  if (Serial.available() > 0) {
    inByte = Serial.read();
    //Serial.print("hello");
    //Serial.print("\n");
    if (inByte == 0){
        Serial.print("zumo serial RT array");
  	Serial.print("\n");
    }
    else if (inByte == 1){
      //main control case
      //send_ser = true;
      nIn = readtwobytes();
      vL = readtwobytes();
      vR = readtwobytes();
    }
    else if (inByte == 2){
      //start new test
      send_ser = true;
      nISR = -1;
      Serial.write(2);
    }
    else if (inByte == 3){
      send_ser = false;
      v = 0;
      vL = 0;
      vR = 0;
      motors.setSpeeds(0,0);
      Serial.write(3);
    }
    else if (inByte == 4){
      calibrate_sensor();
      Serial.write(4);
    }
    else if (inByte == 5){
      //check current error
      SendTwoByteInt(error);
    }

  }//end if serial available
}//end void loop{}


ISR(TIMER2_COMPA_vect)
{     
  nISR++;
  //analogWrite(pwmA, v1);
  //v_out = v1*v1;
  fresh = 1;
  if ( state == 1){
    // pin was HIGH, send it LOW
    state = 0;
    digitalWrite(otherPin, LOW);
  }
  else{
    // pin was LOW, send it HIGH
    state = 1;
    digitalWrite(otherPin, HIGH);
  }
}
