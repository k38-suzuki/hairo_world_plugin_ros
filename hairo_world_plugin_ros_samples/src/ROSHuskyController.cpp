/**
   ROS Husky Controller
   @author Kenta Suzuki
*/

#include <cnoid/SimpleController>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <mutex>

using namespace std;
using namespace cnoid;

class ROSHuskyController : public SimpleController
{
    ros::NodeHandle node;
    ros::Subscriber twistSubscriber;
    geometry_msgs::Twist latestTwistState;
    std::mutex twistMutex;
    
    Link* wheel[4];

public:

    virtual bool configure(SimpleControllerConfig* config) override
    {
        //config->sigChanged().connect();
        return true;
    }
    
    virtual bool initialize(SimpleControllerIO* io) override
    {
        ostream& os = io->os();
        Body* body = io->body();

        static const char* linknames[] = {
            "FRONT_LEFT", "FRONT_RIGHT", "REAR_LEFT", "REAR_RIGHT"
        };

        for(int i = 0; i < 4; ++i) {
            wheel[i] = body->link(linknames[i]);
            Link* joint = wheel[i];
            if(!joint) {
                os << "Wheel " << i << " is not found." << endl;
                return false;
            }
            joint->setActuationMode(Link::JointVelocity);
            io->enableIO(joint);
        }

        twistSubscriber = node.subscribe("husky_velocity_controller/cmd_vel", 100, &ROSHuskyController::twistCallback, this);

        return true;
    }

    void twistCallback(const geometry_msgs::Twist& msg)
    {
        std::lock_guard<std::mutex> lock(twistMutex);
        latestTwistState = msg;
    }

    virtual bool control() override
    {
        geometry_msgs::Twist twist;
        {
            std::lock_guard<std::mutex> lock(mutex);
            twist = latestTwistState;
        }

        double pos[2];
        pos[0] = twist.linear.x / 0.1651;
        pos[1] = twist.angular.z * 0.555;

        for(int i = 0; i < 2; ++i) {
            Link* wheelL = wheel[2 * i];
            Link* wheelR = wheel[2 * i + 1];
            wheelL->dq_target() = pos[0] - pos[1];
            wheelR->dq_target() = pos[0] + pos[1];
        }       

        return true;
    }

    virtual void stop() override
    {
        twistSubscriber.shutdown();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ROSHuskyController)