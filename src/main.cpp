#include <Arduino.h>
#include <math.h>
#include "Wall_Follower.h"
#include "Romi32U4.h"
#include "IR_sensor.h"
#include "ultrasonic.h"
#include "PololuBuzzer.h"
#include "Position_estimation.h"
#include "speed_controller.h"


Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
WallFollower wallFollower;
Position position;
Ultrasonic ultrasonic;
SpeedController romi;
PololuBuzzer buzzer;

enum ROBOT_STATE  //cases for robot operation
{
  IDLE,
  ROTATING,
  WALL_FOLLOWING,
  OUTPUT_DATA,
  COLLECT_DATA,
  OUTPUT_TESTING,
};

ROBOT_STATE robot_state = IDLE;

//MAPPING VALUES
//distance to "chunk" values
float chunkSize = 5.0; //cm measurement for real life equivalent of a "pixel" on the map
//6.35cm from main
const int MAP_RES = 14; // 14 chunks

//DATA COLLECTION VALUES
int SAMPLING_DISTANCE; //distance between measurements in cm
float filterDistance = 42.0; //any values larger than this from the ultrasonic sensor will be disregarded, as the column or row is empty
float readDistance; //the distance between the robot and the object being mapped
int currentChunk = 0; //current chunk the robot is located at on either the x or y axis depending on which side the robot is on
int dataCounts = 0; //number of data collection periods

//NAVIGATION VALUES
// movement values
float baseSpeed = 30;

//side location values
int cornersTraveledTo = 0; //counter to keep track of which direction we're moving in

//odometry values
float previousXDistance = 0;

//turning values
float thetaTolerance = 0.1; // in radians, but approx equivalent is 2.3 degrees

//wall following values
const float DISTANCE_FROM_WALL = 10; //distance from wall before turning in cm (taken from forward IR sensor)
const int WALLFOLLOWING_DISTANCE = 155; //distance from wall while wall following in mm

//ARRAY VALUES AND INSTANTIATION
//data placement values
int column; //the column of the array we're placing a value into
int row; //the row of the array we're placing a value into
int chunkOutput = 1; //value that represents a filled chunk/pixel of the map

//array instantiation
static int mapObjects[MAP_RES][MAP_RES];  //14 by 14 array of ints that holds our data values

//FORWARD IR SENSOR
const float ADCbit     = 1024;
const float refVoltage = 5.0;
const float ADCfactor  = 21.1;
const float ADCoffset  = 0.211;

//median filter sample count for IR sensor readings
int SAMPLE_COUNT1 = 75;

//BUZZER VALUES
int volume = 10;
int duration = 500; // ms

int NOTE_C;
int NOTE_E;
int NOTE_F;

//TIME VALUES
long previous = millis(); //utilized to update pose

void setup() {
  ultrasonic.init();

  //This is just a fun start up sqeuence
  buzzer.playFrequency(261, 500, volume);
  delay(500);
  buzzer.playFrequency(329, 500, volume);
  delay(500);
  buzzer.playFrequency(440, 500, volume);

  wallFollower.Init(A0);
  romi.Init();
  Serial1.begin(115200);
}
// Helper Functions
//IR SENSOR FUNCTIONS
float forwardReadData() { //reads distance from forward-facing IR sensor
  float ADCdistance = analogRead(A4);
  float voltage = (ADCdistance/ADCbit)*refVoltage; //convert ADC value into a voltage
  float distance = ADCfactor/(voltage-ADCoffset); //convert voltage into a distance according to sensor data
  return(distance);
}

float mapIRread() { //reads distance from center-facing IR sensor
  float ADCdistance = analogRead(A2);
  float voltage = (ADCdistance/ADCbit)*refVoltage; //convert ADC value into a voltage
  float distance = ADCfactor/(voltage-ADCoffset); //convert voltage into a distance according to sensor data
  return(distance);
}

//MEDIAN FILTER FUNCTIONS FOR IR SENSORS
// helper for selection sort
void swap2(float *xp, float *yp)
{
	float temp = *xp;
	*xp = *yp;
	*yp = temp;
}

// implementation of selection sort
void selectionSort2(float arr[], int n)
{
	int i, j, min_idx;

	// One by one move boundary of unsorted subarray
	for (i = 0; i < n-1; i++)
	{
		// Find the minimum element in unsorted array
		min_idx = i;
		for (j = i+1; j < n; j++)
		if (arr[j] < arr[min_idx])
			min_idx = j;

		// Swap the found minimum element with the first element
		swap2(&arr[min_idx], &arr[i]);
	}
}

float filter2(float arr[]) {
    int n = SAMPLE_COUNT1;
	  selectionSort2(arr, n);
    int middle = (n / 2);
    return (arr[middle]);
}

float readData(void) {
  float values[SAMPLE_COUNT1];

  for (int i = 0; i < SAMPLE_COUNT1; i++) {
    float distance = forwardReadData();  //converts the pulse length to cm
    values[i] = distance;
  }

  return filter2(values);   // returns distance in cm
}

float filterMapIR(void) {
  float values[SAMPLE_COUNT1];

  for (int i = 0; i < SAMPLE_COUNT1; i++) {
    float distance = mapIRread();  //converts the pulse length to cm
    values[i] = distance;
  }

  return filter2(values);   // returns distance in cm
}

//POSE FUNCTIONS
void updatePose() { //updates the position every 50ms
  if(millis() - previous >= 50) {
    position.UpdatePose(romi.getLeftVelocity(), romi.getRightVelocity());
    previous = millis();
    } 
}

//determines if a value is within a given tolerance
bool inTolerance(float target, float current, float tolerance) {
  return abs(target - current) <= tolerance;
}

//the robot turns an additional angle amount specified
void turnTo(float angle) {

  float endAngle = angle + position.ReadPose().THETA;

  if(endAngle < -2.0*PI) {
    endAngle += 2.0*PI;
  }

  if(endAngle > 2.0 * PI){
    endAngle -= 2.0 * PI;
  }

//while the value is not within the specified tolerance, continue turning
  while(!inTolerance(endAngle, position.ReadPose().THETA, thetaTolerance)) {
        romi.Move(baseSpeed, -baseSpeed);
        updatePose(); 
  }
  romi.Stop();
}

//converts the distance reading from the robot to the object into the chunks/pixels of our map
int distanceToChunk(float distanceReading) {
  return ceil(distanceReading / chunkSize);
}

//NAVIGATION FUNCTIONS
//sets the pixel size based on what side of the field the robot is currently on
// this is done because of the irregularity of the shape of the field. 
// Ideally, a sensor would be used to perform this task, however, the IR sensor nor the ultrasonic
// sensor was capable of getting this desired distance

void setSampleSize(){

  if(cornersTraveledTo == 0){
    SAMPLING_DISTANCE = (10.0 * (104.0 - 9.0))/ 14.0;
  }
  if(cornersTraveledTo == 1){
    SAMPLING_DISTANCE = (10.0 * (91.0 - 9.0))/ 14.0;
  }
  if(cornersTraveledTo == 2){
    SAMPLING_DISTANCE = (10.0 * (99.0 - 9.0))/ 14.0;
  }
  if(cornersTraveledTo == 3){
    SAMPLING_DISTANCE = (10.0* (94.0 - 9.0)) / 14.0;
  }
}

//MAPPING FUNCTIONS
//adds a value into the array at a specified location
void add(int row, int column, int value) {
    mapObjects[row][column] = value;
}

//TRANSMITTING FUNCTIONS
void sendMessage(const String& topic, const String& message)
{
    Serial1.println(topic + String(':') + message);
}
//prints a visualization of the map into the serial monitor
void outputMap() {
    for (int i = 0; i < MAP_RES; i++){
      Serial.println();
      // for each row, go through it and check if it has anything
      for (int j = 0; j < MAP_RES; j++){
        if (mapObjects[i][j] == 1){
          Serial.println(i);
          String x = String(i);
          Serial.println(j);
          String y = String(j);
          String topic = x + "," + y;
          sendMessage(topic, "1");
          Serial.print(1);
          delay(5);
      }
        else{
          Serial.print(0);
        }
        }
    }
}

void loop(){
  updatePose();

  switch (robot_state){ 

  case IDLE:
    //robot remains idle, awaiting for input from buttonA to begin mapping
    //default state, the robot returns to this state after mapping and data sending has been completed
    //robot is not moving (setEfforts(0,0))
    romi.Stop();
    if (buttonA.getSingleDebouncedPress())
    {
      robot_state = WALL_FOLLOWING;
       setSampleSize();
    }
    if(buttonB.getSingleDebouncedPress()){

    }
    
    

    break;

  /////////////////////////////////////////////

    case WALL_FOLLOWING:
    //  maintain X distance away from the wall

    //checks if we've been to all sides of the robot and goes right to output data if so
    if(cornersTraveledTo >= 4) {
       robot_state = OUTPUT_DATA;
        break;  //breaks out of the switch and goes to output data
    }

    // If data read from the front sensor is less than the distance we want to follow, we go to rotating.
    if(readData() <= DISTANCE_FROM_WALL) {
      //Serial.println("I'm at the wall");
      robot_state = ROTATING;
      break;
    }

    //moves straight forward until it finds the wall and continues
    if(dataCounts >= 14) {
      romi.Move(baseSpeed, baseSpeed);
      break;
    }

    wallFollower.wallFollow(WALLFOLLOWING_DISTANCE);
 
    if(currentChunk == 0 && abs(position.ReadPose().X - previousXDistance) >= SAMPLING_DISTANCE / 2) {
      //if currentChunk == 0
      //Then you read pos
      previousXDistance = position.ReadPose().X;
      robot_state = COLLECT_DATA; 
    }
 
     else if(abs(position.ReadPose().X - previousXDistance) >= SAMPLING_DISTANCE) {
      previousXDistance = position.ReadPose().X;
      robot_state = COLLECT_DATA; 
     }
     
      break;

    /////////////////////////////////////////////


   case ROTATING:
   //robot is turning, attempting to get set up for the next leg of mapping
   //turns 90 degrees according to point turn from position tracking (theta increases by pi/4 radians each time)
      //turnTo is blocking;
      turnTo(- PI / 2.0);
      cornersTraveledTo++;

      //Robot state goes back to wall follow
      robot_state = WALL_FOLLOWING;
      currentChunk = 0; //currentChunk reset
      position.resetPose(); //Resets odometery pos
      dataCounts = 0; //sort of redundant, but similar use as current chunk
      setSampleSize(); //determines what side we are on and uses the sample size we measured from there
      previousXDistance = 0; //resets distanc counter

      break;//end of case

      /////////////////////////////////////////////

  case COLLECT_DATA:
      romi.Stop(); //pauses romi
     
    //Read data from ultrasonic
    readDistance = filterMapIR();
    

    //filters if the read distance is greater than a filter
    if(readDistance > filterDistance){ chunkOutput = 0; } 
    else {
      chunkOutput = 1;
    }

    if(cornersTraveledTo == 0){
      column = distanceToChunk(readDistance);
      add(currentChunk, column, chunkOutput);
    }
    else if(cornersTraveledTo == 1){
      row = (MAP_RES - 1) - distanceToChunk(readDistance);
      add(row, currentChunk, chunkOutput);
    }
    else if(cornersTraveledTo == 2){
      column = (MAP_RES-1) - distanceToChunk(readDistance);
      row = (MAP_RES - 1) - currentChunk;
      add(row, column, chunkOutput);
    }
    else if (cornersTraveledTo == 3){
      column = (MAP_RES - 1) - currentChunk;
      row = distanceToChunk(readDistance);
      add(row, column, chunkOutput);
    }
    
      buzzer.playFrequency(distanceToChunk(readDistance) * 100, 500, volume);
      currentChunk++;
      dataCounts++;

      robot_state = WALL_FOLLOWING;
      
     
     //Record this in "Chunks" length    

    break;
  /////////////////////////////////////////////

     case OUTPUT_DATA:

     outputMap();

     //Serial.println("yup");

     robot_state = IDLE;

     break;
     /////////////////////////////////////////////
     // Use case if you want to see a cool map
   
    case OUTPUT_TESTING:
    // PURELY FOR TESTING (as mention in README.md under State Diagram)
    if(buttonA.getSingleDebouncedPress()){
      // testing data
      // SIDE_0
        add(2, 5, 1);
        add(3, 5, 1);
        add(4, 5, 1);

        add(5, 2, 1);
        add(6, 2, 1);
        add(7, 2, 1);
        add(8, 2, 1);

        add(9, 5, 1);
        add(10, 5, 1);
        add(11, 5, 1);

        outputMap();

        }
        break;

    }
}
