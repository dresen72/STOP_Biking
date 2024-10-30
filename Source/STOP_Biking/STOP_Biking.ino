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

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); // declare screen object

int rev_counter = 0; // Count wheel revs for distance
unsigned long prev_time = 0; // Track previous time of rev for speed
double rps;
double mps;


void setup() {
  // Serial monitor init
  Serial.begin(9600);
  Serial.println("Begin S.T.O.P");
  
  // pin mode inits
  pinMode(HALL, INPUT);

  // Add interrupts on falling edge of pull-up hall effect
  attachInterrupt(digitalPinToInterrupt(HALL), hall_interrupt, FALLING);

  tft.init(240, 320); // Init LCD screen
  tft.fillScreen(ST77XX_BLACK);
  display_speedometer();
}

void loop() {
  display_speedometer();
  delay(10);
}

void hall_interrupt(){
  unsigned long cur_time = millis();
  unsigned long gap = cur_time - prev_time;
  rps = 1000.0/gap;
  mps = rps * CIRC;
  Serial.print("RPS: ");
  Serial.println(rps);
  Serial.print("M/S: ");
  Serial.println(mps);
  prev_time = cur_time;
  rev_counter++;
}


void display_speedometer() {
  double speed_mph = mps * MPH_CONV;
  tft.fillScreen(ST77XX_BLACK); // Clear screen each update

  // Draw concentric circles for the speedometer look
  int radius = 60;
  uint16_t colors[] = {ST77XX_WHITE, ST77XX_CYAN, ST77XX_MAGENTA};
  for (int i = 0; i < 3; i++) {
     tft.drawCircle(tft.width() / 2, tft.height() / 2, radius + i * 15, colors[i]);
  }

  // Draw "speed lines" around the circle to give a sense of motion
  for (int16_t angle = 0; angle < 360; angle += 30) {
    float radians = angle * (3.14159 / 180);
    int xStart = (tft.width() / 2) + cos(radians) * (radius + 20);
    int yStart = (tft.height() / 2) + sin(radians) * (radius + 20);
    int xEnd = (tft.width() / 2) + cos(radians) * (radius + 30);
    int yEnd = (tft.height() / 2) + sin(radians) * (radius + 30);
    tft.drawLine(xStart, yStart, xEnd, yEnd, ST77XX_YELLOW);
  }

  // Display speed in the center
  tft.setCursor(tft.width() / 2 - 30, tft.height() / 2 - 20);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(3);
  tft.print((int)speed_mph); // Cast speed to int for simplicity
  tft.print(" MPH");

  // Draw a needle pointing to speed (map speed to a needle angle)
  double max_speed = 30; // Example max speed
  double angle = (speed_mph / max_speed) * 270 - 135; // Maps speed to angle from -135 to +135 degrees
  double rad = angle * (3.14159 / 180);
  int xEnd = (tft.width() / 2) + cos(rad) * (radius + 15);
  int yEnd = (tft.height() / 2) + sin(rad) * (radius + 15);
  tft.drawLine(tft.width() / 2, tft.height() / 2, xEnd, yEnd, ST77XX_RED);
}
