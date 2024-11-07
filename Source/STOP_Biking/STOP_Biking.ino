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

// Global Vars:
int rev_counter = 0; // count wheel revs for distance
double dist_m;       // total distance (m)

unsigned long elapsed_ms = 0; // total time active (ms)
unsigned long cur_time;       // current interrupt time (ms)
unsigned long prev_time = 0;  // previous interrupt time (ms)

int gap;          // gap between intervals (ms)
int prev_gap = 0; // track previous gap for averaging (ms)
double avg_gap;   // average gap (ms)

double speed_rps;       // speed in revolutions per second
double speed_mps;       // speed in meters per second
double speed_mph;       // speed in miles per hour

float nearest_obj_dist_m; // what the LIDAR sees

void setup() {
  // serial init
  Serial.begin(9600);
  Serial.println("Begin S.T.O.P");
  
  // pin mode inits
  pinMode(HALL, INPUT);

  // add interrupts on falling edge of pull-up hall effect
  attachInterrupt(digitalPinToInterrupt(HALL), hall_interrupt, FALLING);

  // initialize I2C
  Wire.begin();
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz

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
  // read LIDAR
  nearest_obj_dist_m = distance_single();

  // update time
  elapsed_ms = millis();

  // display new data
  display_dashboard();
  delay(10);
}

// interrupt for each 1/8th revolution
void hall_interrupt(){
  // update interrupt time and gaps
  cur_time = millis();
  gap = cur_time - prev_time;

  // calculate average game for smoothing
  avg_gap = (gap+prev_gap)/2.0;

  // convert all speed values
  speed_rps = 125.0/avg_gap;
  speed_mps = speed_rps * CIRC;
  speed_mph = speed_mps * MPH_CONV;

  // set the prev trackers
  prev_time = cur_time;
  prev_gap = gap;

  // increment distance trackers
  rev_counter++;
  dist_m = rev_counter*CIRC_8TH;
}

void display_dashboard() {
    // pre-calculate values
    int hours = (elapsed_ms / (1000 * 60 * 60)) % 24;
    int minutes = (elapsed_ms / (1000 * 60)) % 60;
    int seconds = (elapsed_ms / 1000) % 60;
    float km = dist_m / 1000.0;

    // display Speed (xx.x MPH)
    tft.setCursor(10, 80);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setTextSize(5);
    if(speed_mph<10){
      tft.print("0");
    }
    tft.print(speed_mph, 1);

    // display Distance (xxx.xxx km)
    tft.setCursor(10, 150);
    tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    tft.setTextSize(4);
    if(km<100){
      tft.print("0");
    }
    if(km<10){
      tft.print("0");
    }
    tft.print(km, 3);
      
    // display Nearest Object Distance (xx.x m)
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.setCursor(10, 200);
    if(nearest_obj_dist_m < 10) {
      tft.print("0");
    }
    tft.print(nearest_obj_dist_m, 1);

    // display Time (HH:MM:SS)
    tft.setCursor(dial_center_x - 70, dial_center_y + DIAL_RAD + 160);
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

float distance_single()
{
    // 1. Wait for busyFlag to indicate device is idle. This must be
    //    done before triggering a range measurement.
    myLidarLite.waitForBusy();

    // 2. Trigger range measurement.
    myLidarLite.takeRange();

    // 3. Wait for busyFlag to indicate device is idle. This should be
    //    done before reading the distance data that was triggered above.
    myLidarLite.waitForBusy();

    // 4. Read new distance data from device registers
    return myLidarLite.readDistance()/100.0;
}

// Reset screen to standard display of information
void reset_screen(){
  // blackout the screen
  tft.fillScreen(ST77XX_BLACK);

  // write "MPH"
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  tft.setCursor(tft.width()-55, 80);
  tft.print("MPH");

  // write "km"
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(tft.width()-55, 150);
  tft.print("km");

  // write "m"
  tft.setTextColor(ST77XX_RED);
  tft.setCursor(tft.width()-55, 200);
  tft.print("m");
}
