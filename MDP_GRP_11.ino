#include "PinChangeInt.h"

#include <DualVNH5019MotorShield.h>

DualVNH5019MotorShield md;

#define motor_R_enconder 11
#define motor_L_enconder 3

// SR is short range
#define SRmodel 1080
#define LRmodel 20150

#define FR A0   //PS1
#define R A1   //PS2 
#define L A4   //PS3    
#define FL A3   //PS4
#define FC A5   //PS5
#define LR A2    //PS6
//
//SharpIR FR =  SharpIR(PS1, SRmodel);
//SharpIR R =  SharpIR(PS2, SRmodel);
//SharpIR L =  SharpIR(PS5, SRmodel);
//SharpIR FL =  SharpIR(PS4, SRmodel);
//SharpIR C =  SharpIR(PS6, SRmodel);
//SharpIR LR = SharpIR(PS3, LRmodel);

// ================== Global Variables ===================
int left_encoder_val = 0, right_encoder_val = 0, left_encoder_val2 = 0;
float prevError = 0.0;
float integral = 0.0;
const float WHEELCCF = 2 * PI * 3;
int test = 1;
String commandString = "";
int sensorCounter = 0;
bool FP_FLAG = false;

// ==================== ROBOT OFFSETS FOR ADJUSTMENT HERE ==========================
float testSpeed = 280; //180                              //
float PID_KP = 29.5; // lower = right, higher = left     // computePID()
float PID_KI = 0.001;  //0.01                                       // computePID()
float PID_KD = 0.05;  //0.05                                       // computePID()
float GRID_DISTANCE[5] = {9.68, 4.8, 29.7, 40.1, 51};   // moveByGrids(int)
float DG_GRID_DISTANCE[5] = {14.7, 20.5, 29.7, 40.1, 51}; // moveByDgGrids(int)
float LEFT_ROTATION_DEGREE_OFFSET = -2.83;                   // rotateLeft(int)
float RIGHT_ROTATION_DEGREE_OFFSET = -2.58;                  // rotateRight(int)
int NUM_OF_SENSOR_READINGS_TO_TAKE = 15;                  // getDistance()
int NUM_OF_SENSOR_READINGS_TO_TAKE_CALIBRATION = 5;       // calAngle() and calDistance()
int COMMAND_DELAY = 60;

// Below Values for HWLab3 Use
float SENSOR_L_RANGE[4] = {41, 31.0, 20, 12};                  
float SENSOR_R_RANGE[4] = {41, 31.0, 20, 12};  
float SENSOR_FL_RANGE[4] = {35, 28.5, 17.5, 10.5};           
float SENSOR_FC_RANGE[4] = {35, 28.5, 17.5, 10.5};
float SENSOR_FR_RANGE[4] = {35, 28.5, 17.5, 10.5};
float SENSOR_LR_RANGE[7] = {85, 75, 66, 55, 43, 26, 18};     

//Below Values for Lounge Use
//float SENSOR_L_RANGE[4] = {41, 31.0, 22, 10};                  
//float SENSOR_R_RANGE[4] = {41, 31.0, 24, 12};  
//float SENSOR_FL_RANGE[4] = {35, 25.5, 18, 12};           
//float SENSOR_FC_RANGE[4] = {35, 25.5, 18, 12};
//float SENSOR_FR_RANGE[4] = {35, 25.5, 18, 12};
//float SENSOR_LR_RANGE[7] = {85, 75, 63, 50, 39, 26, 20};     

int NUM_OF_TRIES_ANGLE_CALIBRATION = 15;                  // calAngle()
float CAL_ANGLE_MIN_DISTANCE_TO_CALIBRATE = 19;           // calAngle() and calDistance()
float ANGLE_CALIBRATION_THRESHOLD = 0.25;                  // calAngle()
float LEFT_WALL_OFFSET_CALIBRATION = 0.0;
float RIGHT_WALL_OFFSET_CALIBRATION = -0.3;
float LEFT_TO_WALL_DISTANCE_THRESHOLD[2] = {12.7, 13.3};  // calDistance()
float RIGHT_TO_WALL_DISTANCE_THRESHOLD[2] = {12.7, 13.3}; // calDistance()

// ==================== END OF ROBOT ADJUSTMENT VARIABLES ==========================

// ===================== Tick Counter ========================
void lTick()  {
  left_encoder_val++;
  left_encoder_val2++;
}

void rTick()  {
  right_encoder_val++;
}

void resetCounter() {
  left_encoder_val = 0;
  right_encoder_val = 0;
  left_encoder_val2 = 0;
}

// ======================== SETUP =========================
void setup()  {
  Serial.begin(115200);

  pinMode(motor_R_enconder, INPUT);
  pinMode(motor_L_enconder, INPUT);

  md.init();

  PCintPort::attachInterrupt(motor_R_enconder, lTick, RISING);
  PCintPort::attachInterrupt(motor_L_enconder, rTick, RISING);

  Serial.flush();
}

void sendSensorReadings();
void calibrate();
void printDistance();
// ======================== LOOP ==========================
void loop() {
  while (commandString.length() > 0) {
    executeCommand(commandString[0]);
    if (commandString.length() == 1) {
      commandString = "";
      break;
    }
    commandString = commandString.substring(1, -1);
    
  }

}

void serialEvent()  {
  while (Serial.available()) {
    commandString = commandString + char(Serial.read());
  }
}

// ==================== COMMANDS ===========================

void executeCommand(char command) {
  switch (command)  {
    case '1':
      moveByGrids(1);
      delay(COMMAND_DELAY);
      Serial.println("k");
      break;
    case '2':
      moveByDgGrids(1);
      delay(COMMAND_DELAY);
      Serial.println("k");
      break;
    case '3':
      moveByGrids(2);
      delay(COMMAND_DELAY);
      calibrate(30);
      delay(COMMAND_DELAY);
      Serial.println("k");
      break;
    case 'a':
      rotateLeft(90);
      delay(COMMAND_DELAY);
      Serial.println("k");
      break;
    case 'd':
      rotateRight(90);
      delay(COMMAND_DELAY);
      Serial.println("k");
      break;
    case 'q':
      rotateLeft(51);
      delay(COMMAND_DELAY);
      Serial.println("k");
      break;
    case 'e':
      rotateRight(51);
      delay(COMMAND_DELAY);
      Serial.println("k");
      break;
    case 's':
      rotateLeft(180);
      delay(COMMAND_DELAY);
      Serial.println("k");
      break;
    case 'p':
      delay(70);
      sendSensorReadings();
      Serial.println("k");
      break;
    case 'c':
      calibrate(CAL_ANGLE_MIN_DISTANCE_TO_CALIBRATE);
      delay(COMMAND_DELAY);
      Serial.println("k");
      break;
    case 't':                  // for testing purpose only
      printDistance();
      Serial.println("k");
      break;
    case 'z':
        //test1();
      break;
    case 'm':
      FP_FLAG = true;
      GRID_DISTANCE[0] = 10.6;
      LEFT_ROTATION_DEGREE_OFFSET = -1.55;
      RIGHT_ROTATION_DEGREE_OFFSET = -1.42;
      COMMAND_DELAY = 0;
      Serial.println("k");
      break;
  }
}

// ==================== PID Compute =========================

float pidControlForward(int left_encoder_val, int right_encoder_val) {
  //P = High correction to value, D = Add resistance to P, I = Correct error value to original value
  float error = left_encoder_val - right_encoder_val;
  float derivative, output;

  integral += error;
  derivative = error - prevError;
  output = PID_KP * error + PID_KI * integral + PID_KD * derivative;
  prevError = error;
  
  return output;
}

void resetPID() {
  prevError = 0;
  integral = 0;
  resetCounter();
}

// =================== Movement ===================
void moveByGrids(int grids) {
  moveForward(testSpeed, GRID_DISTANCE[grids - 1]);
}

void moveByDgGrids(int grids) {
  moveForward(testSpeed, DG_GRID_DISTANCE[grids - 1]);
}

void moveForward(int sped, float distance)  {
  float output;
  float distanceInTicks;

  distanceInTicks = (distance / WHEELCCF) * 526.25;

  md.setSpeeds(sped, sped-70);
  resetPID();
  while (left_encoder_val2 < distanceInTicks)  {
    output = pidControlForward(left_encoder_val, right_encoder_val);
//    Serial.println(output);
    delay(1);
    md.setSpeeds(-(sped + output), -(sped + 70 - output));
  }
  md.setBrakes(375, 375);
//  if(!FP_FLAG)
//      delay(50);
  resetPID();
}

// ================ ROTATION =====================
void rotateLeft(int degree) {
  float output;
  float dis = (degree + LEFT_ROTATION_DEGREE_OFFSET) / 90.0;
  int left_speed = 200;
  int right_speed = 200;
  float actual_distance = (dis * 405) - (5 * dis);

  delay(1);
  md.setSpeeds(left_speed, -right_speed);
  resetPID();
  while (left_encoder_val2 < actual_distance) {
    output = pidControlForward(left_encoder_val, right_encoder_val);
    delay(1);
    //        Serial.println(output);
    md.setSpeeds(left_speed + output, -right_speed + output);
  }
  md.setBrakes(400, 400);
  resetPID();
//  delay(75);
}

void rotateRight(int degree) {
  float output;
  float dis = (degree + RIGHT_ROTATION_DEGREE_OFFSET) / 90.0;
  int left_speed = 200;
  int right_speed = 200 ;
  float actual_distance = (dis * 405) - (5 * dis);

  delay(1);
  md.setSpeeds(-left_speed, right_speed);
  resetPID();
  while (left_encoder_val2 < actual_distance) {
    output = pidControlForward(left_encoder_val, right_encoder_val);
    delay(1);

    md.setSpeeds(-left_speed - output, right_speed - output);
  }
  md.setBrakes(400, 400);
  resetPID();

}
       
        //=======================================Pre Calibration to get Error====================================
        void calRight(float error) {
          delay(1);
          if (error > 0.5)
          {
            md.setSpeeds(-100, 100);
            delay(abs(error * 50));
            md.setBrakes(300, 300);
          }

          else if (error < 0.5)
          {
            md.setSpeeds(-100, 100);
            delay(abs(error * 75));
            md.setBrakes(300, 300);
          }
        }

        void calLeft(float error) {
          delay(1);
          if (error > 0.5)
          {
            md.setSpeeds(100, -100);
            delay(abs(error * 50));
            md.setBrakes(300, 300);
          }
          else if (error < 0.5)
          {
            md.setSpeeds(100, -100);
            delay(abs(error * 75));
            md.setBrakes(300, 300);
          }
        }

        void calibrate(int CAL_ANGLE_MIN_DISTANCE_TO_CALIBRATE)  {
          delay(1);
          calAngle(CAL_ANGLE_MIN_DISTANCE_TO_CALIBRATE);
          delay(10);
          calDistance(CAL_ANGLE_MIN_DISTANCE_TO_CALIBRATE);
          delay(10);
          calAngle(CAL_ANGLE_MIN_DISTANCE_TO_CALIBRATE);
        }

        void calAngle(int CAL_ANGLE_MIN_DISTANCE_TO_CALIBRATE) {
          float leftToWallDistance, rightToWallDistance;
          int count = 0;
          float error = 10;

          while (1) {
            leftToWallDistance = getDistance(sensorRead(NUM_OF_SENSOR_READINGS_TO_TAKE, FL), FL) + LEFT_WALL_OFFSET_CALIBRATION;
            rightToWallDistance = getDistance(sensorRead(NUM_OF_SENSOR_READINGS_TO_TAKE, FR),FR) + RIGHT_WALL_OFFSET_CALIBRATION;
            error = leftToWallDistance - rightToWallDistance;

            if (abs(error) <= ANGLE_CALIBRATION_THRESHOLD)
              break;

               // can't calibrate cause too far
            if (leftToWallDistance > CAL_ANGLE_MIN_DISTANCE_TO_CALIBRATE ||
                rightToWallDistance > CAL_ANGLE_MIN_DISTANCE_TO_CALIBRATE)      
              break;

            //force a reverse
            if(leftToWallDistance< 12 && rightToWallDistance < 12)
              {
              md.setSpeeds(200, 200);
              delay(abs(error) * 100);
              md.setBrakes(400, 400);
              }
            
            if (error > 0) // left further than right
              calRight(error);
            else
              calLeft(error);
            //    count++;
          }
          md.setBrakes(400, 400);
          delay(1);
        }

        void calDistance(int CAL_ANGLE_MIN_DISTANCE_TO_CALIBRATE) {
          float leftToWallDistance = 99.0, rightToWallDistance = 99.0;
          float error;

          while (1)  {
            leftToWallDistance = getDistance(sensorRead(NUM_OF_SENSOR_READINGS_TO_TAKE, FL), FL);
            rightToWallDistance = getDistance(sensorRead(NUM_OF_SENSOR_READINGS_TO_TAKE, FR),FR);
            error = leftToWallDistance - rightToWallDistance;
            if (leftToWallDistance > CAL_ANGLE_MIN_DISTANCE_TO_CALIBRATE ||
                rightToWallDistance > CAL_ANGLE_MIN_DISTANCE_TO_CALIBRATE)
              break;
            //    Serial.print(leftToWallDistance);
            //    Serial.print(":");
            //    Serial.print(rightToWallDistance);
            //    Serial.print("\n");
            if ((leftToWallDistance >= LEFT_TO_WALL_DISTANCE_THRESHOLD[0] &&
                 leftToWallDistance < LEFT_TO_WALL_DISTANCE_THRESHOLD[1]
                ) || (rightToWallDistance >= RIGHT_TO_WALL_DISTANCE_THRESHOLD[0] &&
                      rightToWallDistance < RIGHT_TO_WALL_DISTANCE_THRESHOLD[1]))
              break;

            
            
            if (rightToWallDistance < RIGHT_TO_WALL_DISTANCE_THRESHOLD[0] || leftToWallDistance < LEFT_TO_WALL_DISTANCE_THRESHOLD[0])
            {
              if(leftToWallDistance< 10 && rightToWallDistance < 12) // when robot is too close to the wall
              { // do a big reverse
              md.setSpeeds(200, 200);
              delay(abs(error) * 100);  //120
              md.setBrakes(400, 400);
              }
              else //reverse normally
              md.setSpeeds(200, 200);
              delay(abs(error) * 80);    //100
              md.setBrakes(400, 400);
            }
            else {  //move forward
              //      moveForward(100,0.9);
             // Serial.println("Moving forwards");
              md.setSpeeds(-200, -200);
              delay(abs(error) * 80);    //100
              md.setBrakes(400, 400);
            }
          }
        }

        //============================================Print Distance in CM=========================================
        void printDistance() {
            float fl,fc,fr,r,l,lr;
            fl = getDistance(sensorRead(NUM_OF_SENSOR_READINGS_TO_TAKE, FL), FL);
            fr = getDistance(sensorRead(NUM_OF_SENSOR_READINGS_TO_TAKE, FR), FR);
            fc = getDistance(sensorRead(NUM_OF_SENSOR_READINGS_TO_TAKE, FC), FC);
            l = getDistance(sensorRead(NUM_OF_SENSOR_READINGS_TO_TAKE, L), L);
            r = getDistance(sensorRead(NUM_OF_SENSOR_READINGS_TO_TAKE, R), R);
            lr = getDistance(sensorRead(NUM_OF_SENSOR_READINGS_TO_TAKE, LR), LR);

          Serial.print("Left: ");
          Serial.println(l);
          Serial.print("Right: ");
          Serial.println(r);
          Serial.print("Front Left(^): ");
          Serial.println(fl);
          Serial.print("Front Centre(^): ");
          Serial.println(fc);
          Serial.print("Front Right(^): ");
          Serial.println(fr);
          Serial.print("Long Range: ");
          Serial.println(lr);

            
         
        }
      //============================================Print Distance in Grids for Algo Team=========================================
        void sendSensorReadings() {
          
           int fl,fc,fr,r,l,lr;
            fl = getDistanceinGrids(getDistance(sensorRead(NUM_OF_SENSOR_READINGS_TO_TAKE, FL), FL), FL);
            fr = getDistanceinGrids(getDistance(sensorRead(NUM_OF_SENSOR_READINGS_TO_TAKE, FR), FR), FR);
            fc = getDistanceinGrids(getDistance(sensorRead(NUM_OF_SENSOR_READINGS_TO_TAKE, FC), FC), FC);
            l = getDistanceinGrids(getDistance(sensorRead(NUM_OF_SENSOR_READINGS_TO_TAKE, L), L), L);
            r = getDistanceinGrids(getDistance(sensorRead(NUM_OF_SENSOR_READINGS_TO_TAKE, R), R), R);
            lr = getDistanceinGrids(getDistance(sensorRead(NUM_OF_SENSOR_READINGS_TO_TAKE, LR), LR), LR);
                    

          Serial.print(l);
          Serial.print(",");
          Serial.print(r);
          Serial.print(",");
          Serial.print(fl);
          Serial.print(",");
          Serial.print(fc);
          Serial.print(",");
          Serial.print(fr);
          Serial.print(",");
          Serial.print(lr);
          Serial.print(",");
          Serial.print(sensorCounter);

          Serial.print("\n");

          sensorCounter++;
        }
//============================================Gets Distance in CM=========================================
        float getDistance(int reading, int sensor){
          float cm;
        
          switch (sensor) {
            case FL:
              cm = 6088.0 / (reading); 
              break;
            case FC: 
              cm = 6088.0 / (reading); 
              break;
            case FR:
              cm = 6088.0 / (reading); 
              break;
            case R:
              cm = 6088.0 / (reading); 
              break;
            case L:
              cm = 6088.0 / (reading); 
              break;
            case LR:
              cm = 15500.0 / (reading); 
              cm = cm-5;
              break;
            default:
              return -1;
          }
          
          return cm;
        }
//============================================Reads Sensor Values=========================================
        int sensorRead(int n, int sensor) {
          int x[n];
          int i;
          int sum = 0;
          for (i = 0; i < n; i++) {
            delay(1);
            x[i] = analogRead(sensor);
          }
          insertionsort(x, n);
          return x[n / 2];        //Return Median
        }
//============================================Sorts Sensor Values=========================================
        void insertionsort(int array[], int length) {
          int i, j;
          int temp;
          for (i = 1; i < length; i++) {
            for (j = i; j > 0; j--) {
              if (array[j] < array[j - 1]) {
                temp = array[j];
                array[j] = array[j - 1];
                array[j - 1] = temp;
              }
              else
                break;
            }
          }
        }

//============================================Logic to convert CM to Grids=========================================
          int getDistanceinGrids(int reading, int sensor){
//            Serial.println(reading);
            int grid;
            switch(sensor)
            {
              case L:
                if(reading<SENSOR_L_RANGE[3])
                    grid = -1;
                else if(reading<=SENSOR_L_RANGE[2])
                    grid = 1;
                else if(reading<=SENSOR_L_RANGE[1])
                    grid = 2;
                else
                    grid = 0;
                break;

              case R:
                 if(reading<SENSOR_R_RANGE[3])
                    grid = -1;
                else if(reading<=SENSOR_R_RANGE[2])
                    grid = 1;
                else if(reading<=SENSOR_R_RANGE[1])
                    grid = 2;
                else
                    grid = 0;
                break;

              case FL:
                     if(reading<SENSOR_FL_RANGE[3])
                        grid = -1;
                    else if(reading<=SENSOR_FL_RANGE[2])
                        grid = 1;
                    else if(reading<=SENSOR_FL_RANGE[1])
                        grid = 2;
                    else
                        grid = 0;
                    break;

                case FC:
                     if(reading<SENSOR_FC_RANGE[3])
                          grid = -1;
                      else if(reading<=SENSOR_FC_RANGE[2])
                          grid = 1;
                      else if(reading<=SENSOR_FC_RANGE[1])
                          grid = 2;
                      else
                          grid = 0;
                      break;

                case FR:
                     if(reading<SENSOR_FR_RANGE[3])
                    grid = -1;
                else if(reading<=SENSOR_FR_RANGE[2])
                    grid = 1;
                else if(reading<=SENSOR_FR_RANGE[1])
                    grid = 2;
                else
                    grid = 0;
                break;

                case LR:
                    if(reading<SENSOR_LR_RANGE[6])
                      grid = -1;
                    else if(reading <=SENSOR_LR_RANGE[5])
                      grid = 2;
                    else if(reading <=SENSOR_LR_RANGE[4])
                      grid = 3;
                    else if(reading <=SENSOR_LR_RANGE[3])
                      grid = 4;
                    else if(reading <=SENSOR_LR_RANGE[2])
                      grid = 5;
                    else if(reading <=SENSOR_LR_RANGE[1])
                      grid = 6;
                    else
                      grid = 0;
                    break;
                      
            }
            return grid;
          }
