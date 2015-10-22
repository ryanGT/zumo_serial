/* This example uses the line sensors on the Zumo 32U4 to follow
a black line on a white background, using a PID-based algorithm.
It works decently on courses with smooth, 6" radius curves and
has been tested with Zumos using 75:1 HP motors.  Modifications
might be required for it to work well on different courses or
with different motors.

This demo requires a Zumo 32U4 Front Sensor Array to be
connected, and jumpers on the front sensor array must be
installed in order to connect pin 4 to DN4 and pin 20 to DN2. */

#include <Wire.h>
#include <Zumo32U4.h>

// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
const uint16_t maxSpeed = 400;
//const int16_t minSpeed = -0;
const int16_t minSpeed = -400;

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors reflectanceSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
//Zumo32U4LCD lcd;

int16_t error=0;
int16_t lastError = 0;
//int16_t n;
int16_t olspeed;
//int16_t maxn;
 
#define numsensors 5
unsigned int sensors[numsensors];

int isrpin=14;
int n=550;
int oldiff=400;
int max_n=2000;
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

// Sets up special characters in the LCD so that we can display
// bar graphs.
/* void loadCustomCharacters() */
/* { */
/*   static const char levels[] PROGMEM = { */
/*     0, 0, 0, 0, 0, 0, 0, 63, 63, 63, 63, 63, 63, 63 */
/*   }; */
/*   lcd.loadCustomCharacter(levels + 0, 0);  // 1 bar */
/*   lcd.loadCustomCharacter(levels + 1, 1);  // 2 bars */
/*   lcd.loadCustomCharacter(levels + 2, 2);  // 3 bars */
/*   lcd.loadCustomCharacter(levels + 3, 3);  // 4 bars */
/*   lcd.loadCustomCharacter(levels + 4, 4);  // 5 bars */
/*   lcd.loadCustomCharacter(levels + 5, 5);  // 6 bars */
/*   lcd.loadCustomCharacter(levels + 6, 6);  // 7 bars */
/* } */

/* void printBar(uint8_t height) */
/* { */
/*   if (height > 8) { height = 8; } */
/*   const char barChars[] = {' ', 0, 1, 2, 3, 4, 5, 6, 255}; */
/*   lcd.print(barChars[height]); */
/* } */

void calibrate_sensor()
{
  //lcd.clear();

  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
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
  motors.setSpeeds(0, 0);
}

// Displays a bar graph of sensor readings on the LCD.
// Returns after the user presses A.
/* void showReadings() */
/* { */
/*   lcd.clear(); */

/*   while(!buttonA.getSingleDebouncedPress()) */
/*   { */
/*     reflectanceSensors.readCalibrated(sensors); */

/*     lcd.gotoXY(0, 0); */
/*     for (uint8_t i = 0; i < numsensors; i++) */
/*     { */
/*       uint8_t barHeight = map(sensors[i], 0, 1000, 0, 8); */
/*       printBar(barHeight); */
/*     } */
/*   } */
/* } */

void setup()
{
  pinMode(isrpin, OUTPUT);
  //digitalWrite(isrpin, HIGH);

  Serial.begin(115200);
  Serial.print("Z34 serial");
  Serial.print("\n");

  //!// Uncomment if necessary to correct motor directions:
  //!//motors.flipLeftMotor(true);
  //!//motors.flipRightMotor(true);

  /* reflectanceSensors.initFiveSensors(); */

  /* loadCustomCharacters(); */

  /* //!// Play a little welcome song */
  cli();          // disable global interrupts
  //-----------------------
  // my hacking
  //-----------------------
  //!//OCR0A = 124;
  //TCCR4A = 0;// set entire TCCR4A register to 0
  //TCCR4B = 0;// same for TCCR4B
  //TCNT4  = 0;//initialize counter value to 0
  //!// set compare match register for 8khz increments
  //OCR4A = 2500;// = (16*10^6) / (8000*8) - 1 (must be <256)
  //!// turn on CTC mode
  //TCCR4A |= (1 << WGM12);
  //!// Set CS41 bit for 8 prescaler
  //!//  TCCR4B |= (1 << CS41);
  //bitSet(TCCR4B, CS42);
  //!//bitSet(TCCR4B, CS11);
  //bitSet(TCCR4B, CS40);
  //!//bitClear(TCCR4B, CS40);
  //!// enable timer compare interrupt
  //TIMSK4 |= (1 << OCIE4A);
  //-----------------------
  // random from internet
  //-----------------------
  // I have dialed in the combination of TCCR4B and OCR4C to give me 50 Hz 
  // squarewave, which is really a 100 Hz ISR freq.
  TCCR4A = 0x00;
  TCCR4B = 0x0B;  //  7 = 16/64 = 0.25 MHz
  TCCR4C = 0x00;
  TCCR4D = 0x00;
  TCCR4E = 0x00;

  OCR4C  =  155;
  OCR4A  =   10;
  TIMSK4 = 0x00;  // OCIE D, A, B,
  TIMSK4 = (1<<OCIE4A);
  //-----------------------
  // different internet source
  //-----------------------
  /* TCCR4B = ((1 & 0xf) << CS40); */
  /* TCCR4C = (0 << COM4D1) | (1 << COM4D0) | (0 << PWM4D); */
  /* TCCR4A = (0 << COM4A1) | (1 << COM4A0) | (0 << PWM4A); */
  /* // Set OCR4A and OCR4D outputs */
  /* DDRC |= (1 << 7); */
  /* DDRD |= (1 << 7); */
  /* // Set period */
  /* OCR4C = 200; */
  /* // Set both outputs to be shifted by T/4 */
  /* OCR4A = 200; */
  /* OCR4D = 100; */
  sei();// re-enable global interrupts
  /* buzzer.play(">g32>>c32"); */

  /* //!// Wait for button A to be pressed and released. */
  //lcd.clear();
  //lcd.print(F("Z34 Serial"));
  //lcd.gotoXY(0, 1);
  //!//lcd.print(F("KD=12; 0"));
  //lcd.print(F("hello"));
  /* buttonA.waitForButton(); */

  /* calibrate_sensor(); */

  /* showReadings(); */

  /* //!// Play music and wait for it to finish before we start driving. */
  /* lcd.clear(); */
  /* lcd.print(F("Go!")); */
  /* buzzer.play("L16 cdegreg4"); */
  /* while(buzzer.isPlaying()); */
  /* n = 0; */
  /* maxn = 200; */
  /* olspeed = 400; */
  //delay(50);
  //Serial.print("hello again");
  //Serial.print("\n");
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
  if ( fresh > 0 ){
    fresh = 0;

    // Get the position of the line.  Note that we *must* provide
    // the "sensors" argument to readLine() here, even
    // though we are not interested in the individual sensor
    // readings.
    int16_t position = reflectanceSensors.readLine(sensors);

    // Our "error" is how far we are away from the center of the
    // line, which corresponds to position 2000.
    error = position - 2000;

    // Get motor speed difference using proportional and derivative
    // PID terms (the integral term is generally not very useful
    // for line following).  Here we are using a proportional
    // constant of 1/4 and a derivative constant of 6, which should
    // work decently for many Zumo motor choices.  You probably
    // want to use trial and error to tune these constants for your
    // particular Zumo and line course.
    //int16_t speedDifference = error / 4 + 6 * (error - lastError);//<-- default values
    int16_t speedDifference = error / 4 + 12 * (error - lastError);

    lastError = error;

    // Get individual motor speeds.  The sign of speedDifference
    // determines if the robot turns left or right.
    int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
    int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;

    // Constrain our motor speeds to be between 0 and maxSpeed.
    // One motor will always be turning at maxSpeed, and the other
    // will be at maxSpeed-|speedDifference| if that is positive,
    // else it will be stationary.  For some applications, you
    // might want to allow the motor speed to go negative so that
    // it can spin in reverse.

    // by default, negative track rotation is not allowed
    leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
    rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);
  
    //leftSpeed = constrain(leftSpeed, minSpeed, (int16_t)maxSpeed);
    //rightSpeed = constrain(rightSpeed, minSpeed, (int16_t)maxSpeed);

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
    if (nISR > 500){
      send_ser = false;
      vL = 0;
      vR = 0;
    }

  }//end if fresh

  if (Serial.available() > 0) {
    inByte = Serial.read()-'0';
    //Serial.println("hello from serial receive");
    if (inByte == 0){
      //main control case
      //send_ser = true;
      Serial.print("zumo serial 32U4 RT array");
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

  //motors.setSpeeds(leftSpeed, rightSpeed);
  /* if ( n < max_n){ */
  /*   n++; */
  /*   motors.setSpeeds(olspeed, olspeed); */
  /* } */
  /* else{ */
  /*   motors.setSpeeds(0,0); */
  /* } */
  //delay(100);
  //digitalWrite(isrpin, LOW);
  delay(10);
}//end void loop{}

ISR(TIMER4_COMPA_vect)
{     
  nISR++;
  //analogWrite(pwmA, v1);
  if ( state == 1){
    // pin was HIGH, send it LOW
    state = 0;
    digitalWrite(isrpin, LOW);
  }
  else{
    // pin was LOW, send it HIGH
    state = 1;
    digitalWrite(isrpin, HIGH);
  }
}
