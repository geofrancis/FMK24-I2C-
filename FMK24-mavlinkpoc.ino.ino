

#include "mavlink/common/mavlink.h"  // Mavlink interface
#include "mavlink/common/mavlink_msg_distance_sensor.h"
#include <SoftwareSerial.h>

SoftwareSerial SoftSerial(10, 11);

int target = 0;
int oldtarget = 0;
int range = 0;

int minimum = 50;
int maximum = 2000;


int Distance1 = 0;
int Distance2 = 0;
int Distance3 = 0;
int Distance4 = 0;


uint8_t orientation = 25; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/

int sysid = 1;
int compid = 196;
uint32_t time_boot_ms = 0;   /*< Time since system boot*/
uint16_t min_distance = 5;   /*< Minimum distance the sensor can measure in centimeters*/
uint16_t max_distance = 600; /*< Maximum distance the sensor can measure in centimeters*/
uint8_t type = 0;            /*< Type from MAV_DISTANCE_SENSOR enum.*/
uint8_t covariance = 0;      /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/
float horizontal_fov = 0;
float vertical_fov = 0;
const float* signal_quality = 0;
uint8_t quaternion = 0;
uint8_t system_type = MAV_TYPE_GCS;
uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

uint8_t system_mode = 0;
uint32_t custom_mode = 0;
uint8_t system_state = 0;



unsigned long previousMillis = 0;  // will store last time LED was updated
const long interval = 1000;        // interval at which to blink (milliseconds)


void setup() {
  Serial.begin(115200);
  SoftSerial.begin(57600);
}



void loop() {
  command_heartbeat();
  command_RADAR();
  command_distance_1();
  command_distance_2();
  command_distance_3();
  command_distance_4();
  command_distance_5();
  printvalue();
}



void command_heartbeat() {

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(sysid, compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void command_RADAR() {
  if (target != oldtarget) {
    target = SoftSerial.parseInt();  //dataIn now holds 0
    range = SoftSerial.parseInt();   //dataIn now holds 0
  } else if (target = 1) {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis < interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      range = range;  //dataIn now holds 0
    }
    else {
      range = 0;  //dataIn now holds 0
      range = 0;  //dataIn now holds 0
    }
    oldtarget = target;
  }
}


void command_distance_1() {
  if (target = 1) {

    Distance1 = range;
    uint8_t id = 1; /*< Onboard ID of the sensor*/

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, Distance1, type, id, orientation, covariance, horizontal_fov, vertical_fov, signal_quality, quaternion);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }
}

void command_distance_2() {
  if (target = 2) {

    Distance2 = range;
    uint8_t id = 1; /*< Onboard ID of the sensor*/

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, Distance1, type, id, orientation, covariance, horizontal_fov, vertical_fov, signal_quality, quaternion);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }
}


void command_distance_3() {
  if (target = 3) {

    Distance1 = range;
    uint8_t id = 3; /*< Onboard ID of the sensor*/

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, Distance1, type, id, orientation, covariance, horizontal_fov, vertical_fov, signal_quality, quaternion);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }
}


void command_distance_4() {
  if (target = 1) {

    Distance4 = range;
    uint8_t id = 4; /*< Onboard ID of the sensor*/

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, Distance1, type, id, orientation, covariance, horizontal_fov, vertical_fov, signal_quality, quaternion);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }
}


void command_distance_5() {
  if (target = 5) {

    Distance1 = range;
    uint8_t id = 5; /*< Onboard ID of the sensor*/

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, Distance1, type, id, orientation, covariance, horizontal_fov, vertical_fov, signal_quality, quaternion);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }
}







void printvalue() {
  Serial.print("Distance1:");
  Serial.print(Distance1);
  Serial.print("mm");
  Serial.print("   ");
  Serial.print("Distance2:");
  Serial.print(Distance2);
  Serial.print("mm");
  Serial.print("   ");
  Serial.print("Distance3:");
  Serial.print(Distance3);
  Serial.print("mm");
  Serial.print("   ");
  Serial.print("Distance4:");
  Serial.print(Distance4);
  Serial.print("mm");
  Serial.print("   ");
  Serial.print(Distance4);
  Serial.print("mm");
  Serial.println("   ");
}

