# Team 11 Mapping Final Project

## Objective: 
The robot is tasked with creating a map of an area with an object in the center utilizing what we've learned in RBE2002 and previous robotics courses. There are two arenas with two different objects in the center, one a 7x7 arena, the other with a finer resolution of 14x14.

## Strategic Approach
To map, our robot is initialized with a two-dimensional 14 by 14 array. The first array denotes the row of the data, where as the second represeted the column of the data. When an array is initialized, all values default to zero. As our robot traversed the field, it would take a sample every X centimeters traveled. If an object was located, the robot would keep note of what "row" and "column" the object was found in. The item would be placed as a 1 in the array at the given row and column index.

## Board
![Pololu Romi 32U4 Board](/romi_32u4.jpg "ESP32 Pinout Diagram")

Our robot used Pololu's 32U4 Board which used the Arduino framework. With that said, any board that uses the Arduino framework should suffice for implementation of our project. 

## Sensors:
![Infrared Sensors](/ir_sensor.jpg "Image of Infrared Sensor")

There are 3 infrared sensors, one facing forwards, one facing out from the right side of the robot, and the last pointed at a 45 degree angle from the front left side of the robot. The forward facing sensor is utilized to detect turn conditions, the IR sensor pointed inwards towards the object in the center of the field is used to gather data values for mapping, and the final angled sensor is used to wall follow with the wall on the left side of the robot. We selected IR sensors as we preferred their consistency along with the fact that they shoot a beam to a specific point rather than a cone like an ultrasonic sensor would. 
Originally, we used an ultrasonic sensor to get mapping data values, however we were concerned by it's irregular data even with a median filter. 

## ESP32
![ESP32 Pinout Diagram](/esp32-devkitC-v4-pinout.png "ESP32 Pinout Diagram")

The ESP32 module is utilized for wireless communication to transmit map data to an MQTT server, from there the data is received, and implemented with visualization software to interpret the data more conveniently. We utilized the the program created for the course, [2002 Field Visualization](https://github.com/savardnm/2002-Field-Visualization). 

## Buzzer

The buzzer on board the Romi helped with debugging as it gave audible signals of if the robot was starting up and was reading distances.

The initial start up sequence is a C E then A note playing. These frequencies we're determined by this chart : https://pages.mtu.edu/~suits/notefreqs.html

## State Diagram
![state diagram showing idle, wall follow, rotate, collect data and transmit data states](/RBE_2002_State_Diagram.drawio.png "Team 11 State Diagram!")
States:
    - Idle
    - Wall Follow
    - Rotate
    - Collect Data
    - Transmit Data
    
(Within the code there is a Output testing, but this is purely for debugging and is not used in the final robot)

### Idle
The default state of the robot is Idle. It will remain there until either button A is pressed. Button A tells the robot to begin the mapping sequence, and it transitions to the Wall Follow state. After a finished mapping run, the robot returns to the Idle state.

### Wall Follow
During the Wall Follow state, the robot is controlled with PD control law. Every defined chunk distance, the robot switches to Collect Data. Before it switches to Collect Data, it updates the previous distance counter. 

### Rotate
During the rotate state the robot rotates clockwise ninety degrees, with a tolerance of 0.1 which is ~2.3 degrees. As soon as the robot is within that tolerance, it stops moving and then transitions back to the Wall Following state. The robot also increments it's turn counter by 1 during this state, keeping track of how many times it has turned during a single run. When the counter == 4, the robot switches state to Transmit Data. 

### Collect Data
During Collect Data, the robot stops temporarily and uses the median of 75 values from its rightmost IR sensor to determine the distance away from the obstacle. Due to the range limitation of the IR sensor, any value over 42 cm is thrown out and the robot records that position as having no obstacles. Otherwise, the robot divides the read distance by the chunk distance to find the number of chunks away the visible obstacle is. The value is rounded to the integer and the robot uses it's current chunk to determine where to place a 1, meaning there is an obstacle, in the final array. During the brief pause in movement, the buzzer goes off with a frequency in proportion to the distance between the robot and the object being mapped. Higher frequencies play at larger distances, and lower frequencies play while the object being mapped is closer.

After this is done, the robot immediately switches back to Wall Follow.

### Transmit Data
Transmit data iterates through the rows and columns of the final array and Serial prints the locations of the created map where the number 1 is present. This transmission goes to the ESP32 on board the Romi, which takes the data and sends it to an MQTT server which our visualizer reads from. 

After transmitting data, the robot returns automatically back to IDLE unless ButtonB is pressed on the Romi, telling the robot to send the data it collected a second time.

## Improvements

After further discussion, an improvement to this project would be having a more precise and more accurate close range IR sensor facing the object to be mapped, as the current one's minimum measurement is very close to the distance between the robot and the object being mapped. 

The fields were inconsistent in size, which led to the offset of data collection and ultimately mapping. If this system were to developed further, the robot would drive around the field once to get the distances over which the 14 data points used would have to map and during the second drive around the field, the robot would just collect the data with the found intervals from the first run. 

