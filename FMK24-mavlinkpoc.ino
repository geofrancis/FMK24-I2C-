

#include "mavlink/common/mavlink.h"  // Mavlink interface
#include "mavlink/common/mavlink_msg_distance_sensor.h"
#include <SoftwareSerial.h>

SerialPIO sonar1(2, 3);

int target = 0;
int oldtarget = 0;
int range = 0;

int minimum = 50;
int maximum = 2000;


int Distance1 = 0;
int Distance2 = 0;
int Distance3 = 0;
int Distance4 = 0;
int Distance5 = 0;

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
  Serial1.begin(115200);  // 0,1
  sonar1.begin(57600);
}

void loop() {
  command_RADAR();
}

void setup1() {}

void loop1() {
  printvalue();
  mavlink();
}



void command_RADAR() {
  target = sonar1.parseInt();  //dataIn now holds 0
  range = sonar1.parseInt();   //dataIn now holds 0
  if (target == 1) {
    Distance1 = 0;
    Distance2 = 0;
    Distance3 = 0;
    Distance4 = 0;
    Distance5 = 0;
    Distance1 = range;
  }
    if (target == 2) {
      Distance2 = range;
    }
    if (target == 3) {
      Distance3 = range;
    }
    if (target == 4) {
      Distance4 = range;
    }
    if (target == 5) {
      Distance5 = range;
    }
  }

  void mavlink() {
    command_heartbeat();
    command_distance_1();
    command_distance_2();
    command_distance_3();
    command_distance_4();
    command_distance_5();
  }
  void command_heartbeat() {

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(sysid, compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }





  void command_distance_1() {
    uint8_t id = 1;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, Distance1, type, id, orientation, covariance, horizontal_fov, vertical_fov, signal_quality, quaternion);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }


  void command_distance_2() {

    uint8_t id = 2;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, Distance2, type, id, orientation, covariance, horizontal_fov, vertical_fov, signal_quality, quaternion);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }



  void command_distance_3() {
    uint8_t id = 3;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, Distance3, type, id, orientation, covariance, horizontal_fov, vertical_fov, signal_quality, quaternion);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }



  void command_distance_4() {
    uint8_t id = 4;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, Distance4, type, id, orientation, covariance, horizontal_fov, vertical_fov, signal_quality, quaternion);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }



  void command_distance_5() {
    uint8_t id = 5;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, Distance5, type, id, orientation, covariance, horizontal_fov, vertical_fov, signal_quality, quaternion);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }








  void printvalue() {


    Serial.print("   ");
    Serial.print("target:");
    Serial.print(target);
    Serial.print("range");
    Serial.print(range);
    Serial.print("   ");


    Serial.print("   ");
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

    Serial.print("Distance5:");
    Serial.print(Distance5);
    Serial.print("mm");
    Serial.println("   ");
  }
