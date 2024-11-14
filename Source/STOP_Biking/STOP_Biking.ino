/* 
  ____ _____ ___   ____    ____  _ _    _             
 / ___|_   _/ _ \ |  _ \  | __ )(_) | _(_)_ __   __ _ 
 \___ \ | || | | || |_) | |  _ \| | |/ / | '_ \ / _` |
  ___) || || |_| ||  __/  | |_) | |   <| | | | | (_| |
 |____(_)_(_)___(_)_| (_) |____/|_|_|\_\_|_| |_|\__, |
                                                |___/ 

 @authors: George Fang, Luke Andresen

 @date: 10/30/2024

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

double obj_time_hist[HIST_LEN];
double obj_dist_hist[HIST_LEN];

void setup() {
  // serial init
  Serial.begin(9600);
  Serial.println("Begin S.T.O.P");

  // pin mode inits
  pinMode(HALL, INPUT);

  // attatch servo
  myservo.attach(BRAKE);

  // add interrupts on falling edge of pull-up hall effect
  attachInterrupt(digitalPinToInterrupt(HALL), hall_interrupt, FALLING);

  // initialize I2C
  Wire.begin();
  TWBR = ((F_CPU / 400000UL) - 16) / 2;  // Set I2C frequency to 400kHz

  // Fill backlog with null data
  for (int i = 0; i < HIST_LEN; i++) {
    obj_time_hist[i] = -1.0;
  }

  // configure LIDAR to long distance
  myLidarLite.configure(3);

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

  // TODO: Replace with real system
  // if (collision_check()) {
  //   warn_and_brake();
  // }
  // else{
  display_dashboard();
  // }

  delay(10);
}

void update_object() {
  // Shift back data down
  for (int i = HIST_LEN - 1; i > 0; i--) {
    obj_time_hist[i] = obj_time_hist[i - 1];
    obj_dist_hist[i] = obj_dist_hist[i - 1];
  }
  // update newest distance
  obj_dist_hist[0] = distance_single();

  // collect time
  elapsed_ms = millis();

  double expected_obj_dist_m = prev_obj_dist_m - (obj_speed_mps * ((elapsed_ms - prev_elapsed_ms) / 1000.0));

  // check if the new distance is a significant jump
  if (obj_dist_hist[0] * JUMP_FACTOR < expected_obj_dist_m) {
    dump_data();  // clear previous history
    obj_dist_m = obj_dist_hist[0];
    prev_obj_dist_m = obj_dist_hist[0];
  }
  // average the distances
  double sum = 0.0;
  int count = 0;
  for (int i = 0; i < HIST_LEN; i++) {
    if (obj_dist_hist[i] == -1) {
      break;
    }
    sum += obj_dist_hist[i];
    count++;
  }
  obj_dist_m = sum / count;

  // calculate speed
  obj_speed_mps = (prev_obj_dist_m - obj_dist_m) / ((elapsed_ms - prev_elapsed_ms) / 1000.0);

  // remove slow objects
  if (obj_speed_mps < 0.5) {
    obj_speed_mps = 0.0;
    obj_time_hist[0] = -1.0;
  } else if (obj_speed_mps != 0) {
    obj_time_hist[0] = obj_dist_m / obj_speed_mps;
  }
  // average time to impact
  sum = 0;
  count = 0;
  for (int i = 0; i < HIST_LEN; i++) {
    // remove invalid data points
    if (obj_time_hist[i] != -1) {
      sum += obj_time_hist[i];
      count++;
    }
  }

  // look for full invalid data
  if (count == 0) {
    time_to_obj_s = -1;
  } else {
    time_to_obj_s = sum / count;
    if (time_to_obj_s > 12) {
      time_to_obj_s = -1;
    }
  }
  // update goal posts
  prev_obj_dist_m = obj_dist_m;
  prev_elapsed_ms = elapsed_ms;
}

void dump_data() {
  for (int i = 1; i < HIST_LEN; i++) {
    obj_time_hist[i] = -1.0;
    obj_dist_hist[i] = -1.0;
  }
}

bool collision_check() {
  return ((elapsed_ms % 10000) <= 100 || (elapsed_ms % 10000) >= 9900);
}

void warn_and_brake() {
  tft.fillScreen(ST77XX_RED);
  delay(500);
  myservo.write(0);
  delay(1000);
  myservo.write(180);
  reset_screen();
}

double distance_single() {
  // 1. Wait for busyFlag to indicate device is idle. This must be
  //    done before triggering a range measurement.
  myLidarLite.waitForBusy();

  // 2. Trigger range measurement.
  myLidarLite.takeRange();

  // 3. Wait for busyFlag to indicate device is idle. This should be
  //    done before reading the distance data that was triggered above.
  myLidarLite.waitForBusy();

  return myLidarLite.readDistance() / 100.0;
}

// interrupt for each 1/8th revolution
void hall_interrupt() {
  // update interrupt time and gaps
  cur_itr_time = millis();
  gap = cur_itr_time - prev_itr_time;

  // calculate average game for smoothing
  avg_gap = (gap + prev_gap) / 2.0;

  // convert all speed values
  speed_rps = 125.0 / avg_gap;
  speed_mps = speed_rps * CIRC;
  speed_mph = speed_mps * MPH_CONV;

  // set the prev trackers
  prev_itr_time = cur_itr_time;
  prev_gap = gap;

  // increment distance trackers
  rev_counter++;
  dist_m = rev_counter * CIRC_8TH;
}



void display_dashboard() {
  // pre-calculate values
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
