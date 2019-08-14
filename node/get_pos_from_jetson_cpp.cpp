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

            uint16_t mode = 0x00;

            uint16_t pos_x = 0;
            uint16_t pos_y = 0;
            uint16_t size_x = 0;
            uint16_t size_y = 0;
            uint8_t checksum = 0;
serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);

            for(int i = 0 ; i < result.data.length(); i++)
                printf("%02X, ", (uint8_t)result.data.at(i));
            std::cout<<std::endl;



            //state machine//
            for(int i = 0 ; i < result.data.length(); i++)
            {
                uint8_t data = result.data.at(i);
                if(mode == 0x00)
                {
                    if(data == 0xFF)
                        mode = 0x01;
                    else
                        mode = 0x00;
                }
                else if(mode == 0x01)
                {
                    if(data == 0xFF)
                        mode = 0x02;
                    else
                        mode = 0x00;
                }
                else if(mode == 0x02)
                {
                    //get pos_x
                    pos_x = data;
                    mode = 0x03;
                }
                else if(mode == 0x03)
                {
                    pos_x = pos_x | (uint16_t)data << 8;
                    mode = 0x04;
                }
                else if(mode == 0x04)
                {
                    pos_y = data;
                    mode = 0x05;
                }
                else if(mode == 0x05)
                {
                    pos_y = pos_y | (uint16_t)data << 8;
                    mode = 0x06;
                }
                else if(mode == 0x06)
                {
                    size_x = data;
                    mode = 0x07;
                }
                else if(mode == 0x07)
                {
                    size_x = size_x | (uint16_t)data << 8;
                    mode = 0x08;
                }
                else if(mode == 0x08)
                {
                    size_y = data;
                    mode = 0x09;
                }
                else if(mode == 0x09)
                {
                    size_y = size_y | (uint16_t)data << 8;
                    mode = 0x0A;
                }
                else if(mode == 0x0A)
                {
                    std::cout<<"check"<<std::endl;
                    if(checksum == data)
                    {
                        //TODO publish topic
                    }
                    pos_x = 0;
                    pos_y = 0;
                    size_x = 0;
                    size_y = 0;
                    checksum = 0;
                    mode = 0x00;
                }
                checksum += data;
            }
        }
        loop_rate.sleep();

    }
}

