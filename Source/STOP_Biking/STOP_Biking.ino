/* 
  ____ _____ ___   ____    ____  _ _    _             
 / ___|_   _/ _ \ |  _ \  | __ )(_) | _(_)_ __   __ _ 
 \___ \ | || | | || |_) | |  _ \| | |/ / | '_ \ / _` |
  ___) || || |_| ||  __/  | |_) | |   <| | | | | (_| |
 |____(_)_(_)___(_)_| (_) |____/|_|_|\_\_|_| |_|\__, |
                                                |___/ 

 @authors: George Fang, Luke Andresen

 @date: 12/12/2024

 @brief: Auto-stopping bike system built on a Teensy4.1

*/

#include "config.h"
#include "libraries.h"

// declare screen object
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// declare LIDAR object
LIDARLite_v3HP myLidarLite;

// declare servo object
Servo myservo;  // create servo object to control a servo

// declare imu object
Adafruit_ICM20948 icm;

// Global Vars:
int rev_counter;  // count wheel revs for distance
double dist_m;    // total distance (m)

unsigned long elapsed_ms = 0;  // total time active (ms)
unsigned long prev_elapsed_ms = 0;
unsigned long cur_itr_time = 0;   // current interrupt time (ms)
unsigned long prev_itr_time = 0;  // previous interrupt time (ms)

int gap;         // gap between intervals (ms)
int prev_gap;    // track previous gap for averaging (ms)
double avg_gap;  // average gap (ms)

double speed_rps;  // speed in revolutions per second
double speed_mps;  // speed in meters per second
double speed_mph;  // speed in miles per hour

double obj_dist_m = 0;       // what the LIDAR sees
double prev_obj_dist_m = 0;  // what the LIDAR previously sees
double obj_speed_mps = 0;    // speed of the object
double time_to_obj_s = 0;

double brake_time = 999999; // time it would take to brake at the current speed
double brake_dist = 999999; // distance it would take to stop at the current speed

double obj_time_hist[HIST_LEN]; // back log of time data for filtering
double obj_dist_hist[HIST_LEN]; // back log of distance data for filtering

// sensor event handelers for imu
sensors_event_t accel; 
sensors_event_t gyro;
sensors_event_t mag;
sensors_event_t temp;

void setup() {
  // serial init
  Serial.begin(9600);
  Serial.println("Begin S.T.O.P");

  // pin mode inits
  pinMode(HALL, INPUT);
  pinMode(BUZZER, OUTPUT);

  // attatch servo
  myservo.attach(BRAKE);
  myservo.write(SERVO_MIN);

  // add interrupts on falling edge of pull-up hall effect
  attachInterrupt(digitalPinToInterrupt(HALL), hall_interrupt, FALLING);

  // initialize I2C
  Wire.begin();
  TWBR = ((F_CPU / 400000UL) - 16) / 2;  // Set I2C frequency to 400kHz

  // Fill backlog with null data
  for (int i = 0; i < HIST_LEN; i++) {
    obj_time_hist[i] = -1.0;
    obj_dist_hist[i] = -1.0;
  }

  // configure LIDAR to long distance mode
  myLidarLite.configure(3);

  // init imu
  icm.begin_I2C();
  icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
  icm.setAccelRateDivisor(200);

  // init LCD screen
  tft.init(240, 320);
  tft.setSPISpeed(40000000);

  // set screen to basic layout
  reset_screen();
}

// control loop
void loop() {
  // update nearst object information
  update_object();

  // check for collision above a certain speed
  if (collision_check() && speed_mps > 2.0) {
    // execute breaking functionality
    warn(); // flashes screen red
    digitalWrite(BUZZER, HIGH); 
    brake();
    digitalWrite(BUZZER, LOW);
    reset_screen(); // reset screen to nominal state after the red flash
  }
  else{
    display_dashboard(); // display data as usual
  }

  delay(50); // tuned delay value for best LiDAR behavior
}

void update_object() {

  // Check for large gap between interrupts to zero out speed
  if(millis()-prev_itr_time > 1000){
    speed_rps = 0;
    speed_mps = 0;
    speed_mph = 0;
  }

  // Shift back data down
  shift_data();

  // update newest distance
  obj_dist_hist[0] = distance_single();

  // collect time
  elapsed_ms = millis();

    // average the distances
  double sum = 0.0;
  int count = 0;
  for (int i = 0; i < HIST_LEN; i++) {
    if(obj_dist_hist[i] == -1){
      break;
    }
    sum += obj_dist_hist[i];
    count++;
  }
  obj_dist_m = sum / count;

  // calculate expected distance of the object based on its previous speed
  double expected_obj_dist_m = prev_obj_dist_m - (obj_speed_mps * ((elapsed_ms - prev_elapsed_ms) / 1000.0));

  // check if the distance is beyond what we would expect it to be
  // if so, this must be a new object, so we shouldn't consider this jump in distance as a fast moving pbject
  if (obj_dist_hist[0] * JUMP_FACTOR < expected_obj_dist_m) {
    // reset data
    dump_data();  // clear previous history
    obj_dist_m = obj_dist_hist[0];
    prev_obj_dist_m = obj_dist_hist[0];
  }

  // calculate the speed of the approching object
  obj_speed_mps = (prev_obj_dist_m - obj_dist_m) / ((elapsed_ms - prev_elapsed_ms) / 1000.0);

  // remove slow objects and objects moving away
  if (obj_speed_mps < 0.5) {
    obj_speed_mps = 0.0;
  } 

  // if valid, calcualte time to object
  if(obj_speed_mps != 0){
    time_to_obj_s = obj_dist_m / obj_speed_mps;
  }
  else{
    time_to_obj_s = -1.0;
  }

  // update goal posts
  prev_obj_dist_m = obj_dist_m;
  prev_elapsed_ms = elapsed_ms;
}

// moves data in backlog
void shift_data(){
  for (int i = HIST_LEN - 1; i > 0; i--) {
    obj_time_hist[i] = obj_time_hist[i - 1];
    obj_dist_hist[i] = obj_dist_hist[i - 1];
  }
}

// dumps data in back log
void dump_data() {
  for (int i = 1; i < HIST_LEN; i++) {
    obj_time_hist[i] = -1.0;
    obj_dist_hist[i] = -1.0;
  }
}

// check for collision using tuned braking functions
bool collision_check() {

  // check for the void case
  if(time_to_obj_s < 0){
    return false;
  }

  // functions found through testing braking at different speeds
  brake_time = BRAKE_FACTOR*speed_mps-BRAKE_ADJUSTMENT;
  brake_dist = 0.0938118*obj_dist_m*obj_dist_m+0.377968*obj_dist_m;

  // check to see if the current state of the bike is within a certain threshold of distance and time ot actuate braking
  return (brake_time > time_to_obj_s*0.65) && (brake_dist < obj_dist_m*1.35);
}

// warn the user
void warn() {
  tft.fillScreen(ST77XX_RED);
}

// actuates the brakes and ABSs until the calculated brake time is done
void brake(){
  myservo.write(SERVO_MAX);
  delay(75);
  unsigned long b_time = millis();
  while(millis() < b_time+(((int)brake_time*1000)+2000)){
    myservo.write(SERVO_MAX+5);
    delay(15);
    myservo.write(SERVO_MAX);
    delay(15);
  }
  myservo.write(SERVO_MIN);
}

// reads a distance from the LiDAR
double distance_single() {
  // wait for device to be idle
  myLidarLite.waitForBusy();

  // take the data
  myLidarLite.takeRange();

  // wait for lidar to process the reading
  myLidarLite.waitForBusy();

  // convert to meters on return
  return myLidarLite.readDistance() / 100.0;
}

// interrupt for each 1/8th revolution
void hall_interrupt() {
  // update interrupt time and gaps
  cur_itr_time = millis();
  gap = cur_itr_time - prev_itr_time;

  // convert all speed values
  speed_rps = 125.0 / gap;
  speed_mps = speed_rps * CIRC;
  speed_mph = speed_mps * MPH_CONV;

  // increment distance trackers
  rev_counter++;
  dist_m = rev_counter * CIRC_8TH;

  // set the prev trackers
  prev_itr_time = cur_itr_time;
  prev_gap = gap;
}

// display data to user
void display_dashboard() {
  // pre-calculate time and distance values
  int hours = (elapsed_ms / (1000 * 60 * 60)) % 24;
  int minutes = (elapsed_ms / (1000 * 60)) % 60;
  int seconds = (elapsed_ms / 1000) % 60;
  float km = dist_m / 1000.0;

  // display Speed (xx.x MPH)
  tft.setCursor(10, 10);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(5);
  if (speed_mph < 10) {
    tft.print("0");
  }
  tft.print(speed_mph, 1);

  // display Distance (xxx.xxx km)
  tft.setCursor(10, 80);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setTextSize(4);
  if (km < 100) {
    tft.print("0");
  }
  if (km < 10) {
    tft.print("0");
  }
  tft.print(km, 3);

  // display Nearest Object Distance (xx.x m)
  tft.setCursor(10, 130);
  tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  if (obj_dist_m < 10) {
    tft.print("0");
  }
  tft.print(obj_dist_m, 1);

  // display speed approaching nearest obj (xx.x m/s)
  tft.setCursor(10, 180);
  tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
  if (obj_speed_mps < 10) {
    tft.print("0");
  }
  tft.print(obj_speed_mps, 1);

  // display time to nearest obj (xx.x s)
  tft.setCursor(10, 230);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  if (time_to_obj_s < 0) {
    tft.print("NA     ");
  } else {
    if (time_to_obj_s < 10) {
      tft.print("0");
    }
    tft.print(time_to_obj_s, 1);
  }

  // display Time (HH:MM:SS)
  tft.setCursor(tft.width() / 2 - 70, 290);
  tft.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  tft.setTextSize(3);
  if (hours < 10) tft.print("0");
  tft.print(hours);
  tft.print(":");
  if (minutes < 10) tft.print("0");
  tft.print(minutes);
  tft.print(":");
  if (seconds < 10) tft.print("0");
  tft.print(seconds);
}

// Reset screen to standard display of information
void reset_screen() {
  // blackout the screen
  tft.fillScreen(ST77XX_BLACK);

  // write "MPH"
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  tft.setCursor(tft.width() - 55, 10);
  tft.print("MPH");

  // write "km"
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(tft.width() - 55, 80);
  tft.print("km");

  // write "m"
  tft.setTextColor(ST77XX_RED);
  tft.setCursor(tft.width() - 55, 130);
  tft.print("m");

  // write "m/s"
  tft.setTextColor(ST77XX_ORANGE);
  tft.setCursor(tft.width() - 55, 180);
  tft.print("m/s");

  // write "s"
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(tft.width() - 55, 230);
  tft.print("s");
}
