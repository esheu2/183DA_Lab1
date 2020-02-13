/*
  Wireless Servo Control, with ESP as Access Point

  Usage: 
    Connect phone or laptop to "ESP_XXXX" wireless network, where XXXX is the ID of the robot
    Go to 192.168.4.1. 
    A webpage with four buttons should appear. Click them to move the robot.

  Installation: 
    In Arduino, go to Tools > ESP8266 Sketch Data Upload to upload the files from ./data to the ESP
    Then, in Arduino, compile and upload sketch to the ESP

  Requirements:
    Arduino support for ESP8266 board
      In Arduino, add URL to Files > Preferences > Additional Board Managers URL.
      See https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/installing-the-esp8266-arduino-addon

    Websockets library
      To install, Sketch > Include Library > Manage Libraries... > Websockets > Install
      https://github.com/Links2004/arduinoWebSockets
    
    ESP8266FS tool
      To install, create "tools" folder in Arduino, download, and unzip. See 
      https://github.com/esp8266/Arduino/blob/master/doc/filesystem.md#uploading-files-to-file-system

  Hardware: 
  * NodeMCU Amica DevKit Board (ESP8266 chip)
  * Motorshield for NodeMCU 
  * 2 continuous rotation servos plugged into motorshield pins D1, D2
  * Ultra-thin power bank 
  * Paper chassis

*/

#include <Arduino.h>

#include <Hash.h>
#include <FS.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>

#include <Servo.h>
//#include <vector.h>

#include <VL53L0X.h>
#define SDA_port 14
#define SCL_port 12
VL53L0X sensor2;
VL53L0X sensor1;


unsigned long StartTime = millis();
unsigned long OldTime = StartTime;

#include "debug.h"
#include "file.h"
#include "server.h"

/* FROM SENSOR TEST */
#include <Wire.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define SDA_PORT 14
#define SCL_PORT 12
/* END FROM SENSOR TEST */


float old_angle = 0;

const int SERVO_LEFT = D2;
const int SERVO_RIGHT = D1;
Servo servo_left;
Servo servo_right;
int servo_left_ctr = 90;
int servo_right_ctr = 90;


// WiFi AP parameters
char ap_ssid[13];
char* ap_password = "";

// WiFi STA parameters
char* sta_ssid = 
  "...";
char* sta_password = 
  "...";

char* mDNS_name = "paperbot";

String html;
String css;
WiFiServer server(100); //transmission server

typedef struct Sensors
{
  float len_x;
  float len_y;
  float theta;
};

/* FUNCTIONS FROM SENSOR TEST */
// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}
/* END FUNCTIONS FROM SENSOR TEST */

    int x = 2; //original x-coordinate
    int y = 2; //original y-coordinate
    float theta = 90; //original rotation angle
    double state[] = {0, 0, 0, 0};
  
    //new positions
    int new_x = 0; //x'
    int new_y = 0; //y'
    float new_theta; //theta'
    
    //constants
    double h = 2; //distance travelled by robot in one f/b pulse
    double phi = 5; //angle rotated by robot in one l/r pulse
    double t_interval = 0.08; //time interval in seconds used to calculate rotational speed
    double L = 1000; //known distance between origin and right wall
    double W = 1000; //known distance between origin and front wall
    
    //new constants
    double v_r = 2; //translational right velocity
    double v_l = 2; //translational left velocity
    double v = 0; //translational velocity
    double delta_theta = 0; //change in rotational velocity
    double d = 25.4; //half of the width of the car
    double R;
    float delta_x;
    float delta_y;
    double d1, d2, d3, d4;
    double l1, l2, l3, l4;
    
    // Variables initiated to calculate Gaussian noise metrics
    float noise_value_right = 0;
    float right_mean = 0;
    float error_sum_total_right = 0;
    int running_count_right = 0;
    float saved_sensor_right;
    float right_values_array[10000];

    float noise_value_front = 0;
    float front_mean = 0;
    float error_sum_total_front = 0;
    int running_count_front = 0;
    float saved_sensor_front;
    float front_values_array[10000];

    float noise_value_theta = 0;
    float theta_mean = 0;
    float error_sum_total_theta = 0;
    int running_count_theta = 0;
    float saved_sensor_theta;
    float theta_values_array[10000];
    
    // bool running = true;
    // string input = "";

// This function will determine means and variances for the right sensor distance, as well as a state-estimated x position.
void gaussian_calcs_right() {
  float z = saved_sensor_right;
  right_values_array[running_count_right] = z;
  error_sum_total_right = z - x;
  running_count_right++;
  right_mean = error_sum_total_right / running_count_right; // Mean of the right sensor determined by taking cumulative error derived by a running count of instances.

  float temp;
  float sigma = 0;
  for (int i = 0; i < running_count_right; i++)
  {
    temp = right_values_array[i] - right_mean;
    temp = temp*temp;
    sigma += temp;
  }
  sigma = sigma / running_count_right;
  sigma = sqrt(sigma); // Final step for determining variance of right sensor at current time.

  noise_value_right = exp(-0.5*pow((z-right_mean)/sigma,2.)); // Function for creating Gaussian distribution curve.

  int s_e_right = x + noise_value_right; //calculate new distance b/w robot and right wall with sensor readings & noise modeling
  Serial.println("State-Estimated X position:");
  Serial.print(s_e_right);
}

// This function will determine means and variances for the front sensor distance, as well as a state-estimated y position.
void gaussian_calcs_front() {
  float z_f = saved_sensor_front;
  front_values_array[running_count_front] = z_f;
  error_sum_total_front = z_f - y;
  running_count_front++;
  front_mean = error_sum_total_front / running_count_front; // Mean of the front sensor determined by taking cumulative error derived by a running count of instances.

  float temp_f;
  float sigma_f = 0;
  for (int q = 0; q < running_count_front; q++)
  {
    temp_f = front_values_array[q] - front_mean;
    temp_f = temp_f*temp_f;
    sigma_f += temp_f;
  }
  sigma_f = sigma_f / running_count_front;
  sigma_f = sqrt(sigma_f); // Final step for determining variance of front sensor at current time.

  noise_value_front = exp(-0.5*pow((z_f-front_mean)/sigma_f,2.)); // Function for creating Gaussian distribution curve.

  int s_e_front = y + noise_value_front; //calculate new distance b/w robot and front wall with sensor readings & noise modeling
  Serial.print(", State-Estimated Y position:");
  Serial.print(s_e_front);
}

// This function will determine means and variances for theta, as well as a state-estimated theta position.
void gaussian_calcs_theta() {
  float z_t = saved_sensor_theta;
  theta_values_array[running_count_theta] = z_t;
  error_sum_total_theta = z_t - theta;
  running_count_theta++;
  theta_mean = error_sum_total_theta / running_count_theta; // Mean of the theta measurement determined by taking cumulative error derived by a running count of instances.

  float temp_t;
  float sigma_t = 0;
  for (int c = 0; c < running_count_theta; c++)
  {
    temp_t = theta_values_array[c] - theta_mean;
    temp_t = temp_t*temp_t;
    sigma_t += temp_t;
  }
  sigma_t = sigma_t / running_count_theta;
  sigma_t = sqrt(sigma_t); // Final step for determining variance of front sensor at current time.

  noise_value_theta = exp(-0.5*pow((z_t-theta_mean)/sigma_t,2.)); // Function for creating Gaussian distribution curve.

  int s_e_theta = theta + noise_value_theta; //calculate new theta with sensor readings & noise modeling
  Serial.println("State-Estimated Theta position:");
  Serial.print(s_e_theta);
}

void calculations() {
        /*      SENSOR MEASUREMENT MODELS      */
        int d_f = W - new_y; //calculate new distance b/w robot and front wall
        int d_r = L - new_x; //calculate new distance b/w robot and right wall
        int rot_sp = (new_theta - theta) / t_interval; //calculate rotational speed
        
        // mathematical model
        v = (v_r + v_l) / 2;
        R = (2*d*v_r) / (v_l - v_r);
        delta_x = v * cos(new_theta * M_PI / 180.0);
        delta_y = v * sin(new_theta * M_PI / 180.0);
        new_x = x + delta_x;
        new_y = y + delta_y;
        
        // measurement model
        d1 = abs((L - new_x)/cos(new_theta * M_PI / 180.0));
        d2 = abs((W - new_y)/sin(new_theta * M_PI / 180.0));
        d3 = abs((new_x)/cos((new_theta+180.0)* M_PI / 180.0));
        d4 = abs((new_y)/sin((new_theta+180.0)* M_PI / 180.0));
        
        l1 = abs((L-new_x)/sin(new_theta * M_PI / 180.0));
        l2 = abs((W-new_y)/cos((new_theta+180.0)* M_PI / 180.0));
        l3 = abs((new_x)/sin((new_theta+180.0)* M_PI / 180.0));
        l4 = abs((new_y)/cos(new_theta* M_PI / 180.0));
        
        state[0] = new_x;
        state[1] = new_y;
        state[2] = new_theta;
        state[3] = rot_sp;

        // cout << "new_theta = " << new_theta << endl;
        // cout << "delta x = " << delta_x << endl;
        Serial.println ("Current position: (");
        Serial.print(new_x);
        Serial.print(", ");
        Serial.print(new_y);
        Serial.print(")");
        // cout << "), Current rotation angle: " << new_theta << endl;
        // cout << "New front wall distance: " << d_f;
        // cout << ", New right wall distance: " << d_r;
        // cout << ", Rotational speed: " << rot_sp;
        // cout << ", Translational velocity: " << v;
        // cout << ", Distance R: " << R << endl;
        // cout << "d1 = " << d1 << ", d2 = " << d2 << ", d3 = " << d3 << ", d4 = " << d4 << endl;
        // cout << "l1 = " << l1 << ", l2 = " << l2 << ", l3 = " << l3 << ", l4 = " << l4 << endl;
        // cout << "state: " << state[0] << " " << state[1] << " " << state[2] << " " << state[3] << endl;
        // cout << endl;
        
        //update positions
        x = new_x;
        y = new_y;
        theta = new_theta;
    }



void setup() {
    setupPins();
    sensorsSetup();
    sprintf(ap_ssid, "ESP_%08X", ESP.getChipId());

    for(uint8_t t = 4; t > 0; t--) {
        Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
        Serial.flush();
        LED_ON;
        delay(500);
        LED_OFF;
        delay(500);
    }
    LED_ON;
    //setupSTA(sta_ssid, sta_password);
    setupAP(ap_ssid, ap_password);
    LED_OFF;

    setupFile();
    html = loadFile("/controls.html");
    css = loadFile("/style.css");
    registerPage("/", "text/html", html);
    registerPage("/style.css", "text/css", css);

    setupHTTP();
    setupWS(webSocketEvent);
    //setupMDNS(mDNS_name);

    /* FROM SENSOR TEST */
    // Arduino initializations
    Wire.begin(SDA_PORT,SCL_PORT);
    Serial.begin(115200);

    // Set by pass mode for the magnetometers
    I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
    // Request first magnetometer single measurement
    I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
    /* END SENSOR TEST */

    stop();
    //server.begin();
}

void loop() {
    wsLoop(); // Helps to set up the wireless server for the Arduino of the paperbot.
    httpLoop(); // Allows us to wirelessly transmit data between the PC and the Arduino.
    actionLoop(); // Allows us to assign inputs to the Arduino and access the parameters delivered by the sensors.
    gaussian_calcs_right(); // Determines the noise model, mean, variances and state-estimated x position for the right sensor.
    //gaussian_calcs_front(); // Determines the noise model, mean, variances and state-estimated y position for the front sensor. Currently does not compile with other gaussian calculations; most likely due to memory issues associated with arrays in each function.
    //gaussian_calcs_theta(); // Determines the noise model, mean, variances and state-estimated theta position for theta. Currently does not compile with other gaussian calculations; most likely due to memory issues associated with arrays in each function.
}

float time_interval = 0.08; // Time interval between measurements for our Arduino.

void actionLoop()
{
  unsigned long time1 = millis();
  unsigned long time2 = millis();
  while (time2-time1 < 5000) // Move forward for 5 seconds.
  {
    time2 = millis();
    forward();
  }
  time1 = millis();
  time2 = millis();
  while (time2-time1 < 5000) // Move backward for 5 seconds.
  {
    time2 = millis();
    backward();
  }
  time1 = millis();
  time2 = millis();
  while (time2-time1 < 5000) // Move left for 5 seconds.
  {
    time2 = millis();
    left();
  }
  time1 = millis();
  time2 = millis();
  while (time2-time1 < 5000) // Move right for 5 seconds.
  {
    time2 = millis();
    right();
  }
  struct Sensors s = measureSensors();
  float x = s.len_x;
  float y = s.len_y;
  float angle = s.theta;
  //float angle_diff = old_angle - angle;
  //float angular_vel = angle_diff / time_interval;
  //unsigned long CurrentTime = millis();
  //unsigned long PeriodTime = CurrentTime - OldTime;
  
  Serial.print ("\t x:");
  Serial.print (x);  
  Serial.println ("\t");
  Serial.print ("\t y:");
  Serial.print (y);  
  Serial.println ("\t");
  /*
  Serial.print ("\t Current Time:");
  Serial.print (CurrentTime);  
  Serial.println ("\t");
  Serial.print ("\t Old Time:");
  Serial.print (OldTime);  
  Serial.println ("\t");
  Serial.print ("\t Period Time:");
  Serial.print (PeriodTime);
  */
  Serial.println ("\t");

  //OldTime = CurrentTime;
}

void sensorsSetup() {  
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);
  delay(500);
  Wire.begin(SDA_port,SCL_port);
  digitalWrite(D3, HIGH);
  delay(150);
  Serial.println("00");  
  sensor1.init(true);
  Serial.println("01");
  delay(100);
  sensor1.setAddress((uint8_t)22);
  digitalWrite(D4, HIGH);
  delay(150);
  sensor2.init(true);
  Serial.println("03");
  delay(100);
  sensor2.setAddress((uint8_t)25);
  Serial.println("04");
  Serial.println("addresses set");  
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;
  Wire.begin(SDA_PORT,SCL_PORT);  
   // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);  
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  for (byte i = 1; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
  delay(3000);
}

/* LOOP FROM SENSOR TEST */
//timing tests
bool first_record = true;
long int cpt=0;


// Main loop, read and display data
struct Sensors measureSensors()
{
  
  // _______________
  // ::: Counter :::
  
  // Display data counter
  Serial.print (cpt++,DEC);
  Serial.print ("\t");
  
  // _____________________
  // :::  Magnetometer ::: 

  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  
  // Read register Status 1 and wait for the DRDY: Data Ready
  
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);

  // Create 16 bits values from 8 bits data
  
  // Magnetometer
  int16_t mx=(Mag[1]<<8 | Mag[0]);
  int16_t my=(Mag[3]<<8 | Mag[2]);
  int16_t mz=(Mag[5]<<8 | Mag[4]);

  float heading = atan2(mx, my);

  // Once you have your heading, you must then add your 'Declination Angle',
  // which is the 'Error' of the magnetic field in your location. Mine is 0.0466
  // Find yours here: http://www.magnetic-declination.com/
  
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.0466;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/PI; 

  struct Sensors s;
  s.len_x = sensor2.readRangeSingleMillimeters(); // front
  s.len_y = sensor1.readRangeSingleMillimeters(); // right
  s.theta = heading;


  /* Current Position Calculation */
  int d_f_sensor = 1000 - s.len_y; //calculate new distance b/w robot and front wall with sensor readings
  int d_r_sensor = 1000 - s.len_x; //calculate new distance b/w robot and right wall with sensor readings
  saved_sensor_right = d_r_sensor; // Stores right sensor-determined position into global variable.
  saved_sensor_front = d_f_sensor; // Stores front sensor-determined position into global variable.
  // First numbers (e.g. 1000) can be changed based upon bounds of the box
  Serial.println("X position:");
  Serial.print(d_f_sensor);
  Serial.print(", Y position:");
  Serial.print(d_r_sensor);

  Serial.print("\rHeading:\t");
  Serial.print(heading);
  Serial.print(" Radians   \t");
  Serial.print(headingDegrees);
  Serial.println(" Degrees   \t");

  Serial.print ("Magnetometer readings:"); 
  Serial.print ("\tMx:");
  Serial.print (mx); 
  Serial.print ("\tMy:");
  Serial.print (my);
  Serial.print ("\tMz:");
  Serial.print (mz);  
  Serial.println ("\t");

  //angular velocity
  if (first_record == true)
  {
    first_record = false;
    old_angle = mx;
  }

  else
  {
    float angle_diff;
    float angular_velocity;
    angle_diff = mx - old_angle;
    angular_velocity = angle_diff / .08;
    // Angular velocity is calculated by taking the derivative of dmx/dt, and this becomes possible through the magnetometer readings.

    Serial.print("Old angle: ");
    Serial.print(old_angle);
    Serial.print("\t");
    Serial.print("Current angle: ");
    Serial.print(mx);
    Serial.print("\t");
    Serial.print("Angle diff: ");
    Serial.print(angle_diff);
    Serial.print("\t");
    Serial.print("Angular velocity: ");
    Serial.print(angular_velocity);

    old_angle = mx;
  }
  saved_sensor_theta = mx;
  
  // End of line
  return s; 
}
/* END LOOP FROM SENSOR TEST */


//
// Movement Functions //
//

void drive(int left, int right) {
  servo_left.write(left);
  servo_right.write(right);
}

void stop() {
  DEBUG("stop");
  drive(servo_left_ctr, servo_right_ctr);
  LED_OFF;
}

void forward() {
  DEBUG("forward");
  drive(0, 180);
  new_theta = theta;
            
  //new
  v_r = 2; // Simulated (data-driven) right wheel rotational velocity
  v_l = 2; // Simulated (data-driven) left wheel rotational velocity
  delta_theta = 0;

  calculations(); // Calls calculations to determine mathematical model and measurement model
}

void backward() {
  DEBUG("backward");
  drive(180, 0);

  new_theta = theta;
            
  //new
  v_r = -2; // Simulated (data-driven) right wheel rotational velocity
  v_l = -2; // Simulated (data-driven) left wheel rotational velocity
  delta_theta = 0;

  calculations(); // Calls calculations to determine mathematical model and measurement model
}

void left() {
  DEBUG("left");
  drive(180, 180);

  //new
  v_r = 2; // Simulated (data-driven) right wheel rotational velocity
  v_l = -2; // Simulated (data-driven) left wheel rotational velocity
  delta_theta = 4 / 2*d;
  new_theta = theta + delta_theta;

  calculations(); // Calls calculations to determine mathematical model and measurement model
}

void right() {
  DEBUG("right");
  drive(0, 0);

  //new
  v_r = -2; // Simulated (data-driven) right wheel rotational velocity
  v_l = 2; // Simulated (data-driven) left wheel rotational velocity
  delta_theta = -4 / 2*d;
  new_theta = theta + delta_theta;

  calculations(); // Calls calculations to determine mathematical model and measurement model
}



//
// Setup //
//

void setupPins() {
    // setup Serial, LEDs and Motors
    Serial.begin(115200);
    DEBUG("Started serial.");

    pinMode(LED_PIN, OUTPUT);    //Pin D0 is LED
    LED_OFF;                     //Turn off LED
    DEBUG("Setup LED pin.");

    servo_left.attach(SERVO_LEFT);
    servo_right.attach(SERVO_RIGHT);
    DEBUG("Setup motor pins");
}

void webSocketEvent(uint8_t id, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            DEBUG("Web socket disconnected, id = ", id);
            break;
        case WStype_CONNECTED: 
        {
            // IPAddress ip = webSocket.remoteIP(id);
            // Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", id, ip[0], ip[1], ip[2], ip[3], payload);
            DEBUG("Web socket connected, id = ", id);

            // send message to client
            wsSend(id, "Connected to ");
            wsSend(id, ap_ssid);
            break;
        }
        case WStype_BIN:
            DEBUG("On connection #", id)
            DEBUG("  got binary of length ", length);
            for (int i = 0; i < length; i++)
              DEBUG("    char : ", payload[i]);

            if (payload[0] == '~') 
              drive(180-payload[1], payload[2]);

        case WStype_TEXT:
            DEBUG("On connection #", id)
            DEBUG("  got text: ", (char *)payload);

            if (payload[0] == '#') {
                if(payload[1] == 'C') {
                  LED_ON;
                  wsSend(id, "Hello world!");
                }
                else if(payload[1] == 'F') 
                  forward();
                else if(payload[1] == 'B') 
                  backward();
                else if(payload[1] == 'L') 
                  left();
                else if(payload[1] == 'R') 
                  right();
                else if(payload[1] == 'U') {
                  if(payload[2] == 'L') 
                    servo_left_ctr -= 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr += 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else if(payload[1] == 'D') {
                  if(payload[2] == 'L') 
                    servo_left_ctr += 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr -= 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else 
                  stop();
            }

            break;
    }

    
}
