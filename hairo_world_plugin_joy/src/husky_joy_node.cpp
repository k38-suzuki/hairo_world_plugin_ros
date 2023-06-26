/**
   ROS Husky Publisher
   @author Kenta Suzuki
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cnoid/Joystick>
#include <iostream>
#include <unistd.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "husky_joy");

    ros::NodeHandle nh("~");
    string device;
    nh.getParam("dev", device);
    cnoid::Joystick joystick(device.c_str());
    
    ros::NodeHandle node;
    ros::Publisher publisher = node.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::Rate loop_rate(60);
    int seq = 0;
    bool stateChanged = false;

    joystick.sigButton().connect(
        [&](int, bool){ stateChanged = true; });
    joystick.sigAxis().connect(
        [&](int, double){ stateChanged = true; });

    if(!joystick.isReady()) {
        cout << "Joystick is not ready." << endl;
    }
    bool isBeforeInitialReading = true;

    while(ros::ok()){
        if(!joystick.isReady()) {
            if(!joystick.makeReady()) {
                usleep(500000);
                continue;
            }
        }
        if(isBeforeInitialReading) {
            cout << "Joystick \"" << joystick.device() << "\" is ready." << endl;
            isBeforeInitialReading = false;
        }
        joystick.readCurrentState();
        if(stateChanged) {
            geometry_msgs::Twist twist;
            double pos[2];
            for(int i = 0; i < 2; ++i) {
                pos[i] = joystick.getPosition(
                    i == 0 ? cnoid::Joystick::L_STICK_H_AXIS : cnoid::Joystick::L_STICK_V_AXIS);
                if(fabs(pos[i]) < 0.2) {
                    pos[i] = 0.0;
                }
            }           

            double k = 0.0;
            for(int i = 0; i < 2; ++i) {
                bool buttonState = joystick.getButtonState(
                    i == 0 ? cnoid::Joystick::L_BUTTON : cnoid::Joystick::R_BUTTON);
                if(buttonState) {
                    if(i == 0) {
                        k = 0.4;
                    } else if(i == 1) {
                        k = 2.0;
                    }
                }
            }

            twist.linear.x = k * pos[1];
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = 1.4 * pos[0];

            publisher.publish(twist);
            stateChanged = false;
        }
        loop_rate.sleep();
    }
    return 0;
}
