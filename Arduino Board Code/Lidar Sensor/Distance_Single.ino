 /* ============================================================================= 
2   LIDAR-Lite v2: Single sensor get single distance measurement 
3  
4   This example file demonstrates how to take a single distance measurement with 
5   LIDAR-Lite v2 "Blue Label". 
6  
7   The library is in BETA, so subscribe to the github repo to recieve updates, or 
8   just check in periodically: 
9   https://github.com/PulsedLight3D/LIDARLite_v2_Arduino_Library 
10    
11   To learn more read over lidarlite.cpp as each function is commented 
12 =========================================================================== */ 

 
#include <Wire.h> 
#include <LIDARLite.h> 
 
 
LIDARLite myLidarLite; 
 
 
void setup() { 
  Serial.begin(115200); 
  myLidarLite.begin(); 
} 
 
 
void loop() { 
  Serial.println(myLidarLite.distance()); 
} 

