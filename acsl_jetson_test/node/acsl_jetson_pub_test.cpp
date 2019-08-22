/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <acsl_jetson/boxInfo.h>

#define FINAL_STATE 0x0A

uint16_t mode = 0x00;

uint16_t pos_x = 0;
uint16_t pos_y = 0;
uint16_t size_x = 0;
uint16_t size_y = 0;
uint8_t checksum = 0;
uint16_t sequence = 0;

void write_callback(const acsl_jetson::boxInfo& msg){
    ROS_INFO_STREAM("get data");
    ROS_INFO_STREAM("pos_x :"<<msg.pos_x);
    ROS_INFO_STREAM("pos_y :"<<msg.pos_y);
    ROS_INFO_STREAM("size_x :"<<msg.size_x);
    ROS_INFO_STREAM("size_y :"<<msg.size_y);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "acsl_jetson_pub_test");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("boxInfo", 1, write_callback);

    ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();
        ROS_INFO_STREAM("waiting");
        loop_rate.sleep();

    }
}

