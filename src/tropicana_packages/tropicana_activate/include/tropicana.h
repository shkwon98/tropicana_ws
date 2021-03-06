#ifndef __TROPICANA_H
#define __TROPICANA_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <termios.h>
#include <math.h>

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include <vision_msgs/Detection3DArray.h>

#define NUM_OF_JOINT 4


// Relative Position of D435i Camera from the Base of Manipulator
const float x_offset = 0.0900;    // Unit : Meter, Degree
const float y_offset = 0.0075;    // Unit : Meter
const float z_offset = 0.0600;    // Unit : Meter
const float deg_offset = 15.0;    // Unit : Degree
const float rad_offset = deg_offset * M_PI / 180;

const float y_gain = 1.75;
const float cutter_z_offset = 0.055;


namespace tropicana
{
    class TROPICANA
    {
    private:
        // ROS NodeHandle
        ros::NodeHandle node_handle_;

        ros::ServiceClient goal_joint_space_path_client_;
        ros::ServiceClient goal_task_space_path_position_only_client_;
        ros::ServiceClient goal_tool_control_client_;

        ros::Subscriber open_manipulator_states_sub_;
        ros::Subscriber chain_joint_states_sub_;
        ros::Subscriber chain_kinematics_pose_sub_;
        ros::Subscriber centroid_pose_array_sub;


        std::vector<double> present_joint_angle;
        std::vector<double> present_kinematic_position;

        open_manipulator_msgs::KinematicsPose kinematics_pose_;

    public:
        bool flag;
        std::vector<double> goalPose;
        TROPICANA();
        ~TROPICANA();

        void initPublisher();
        void initSubscriber();

        void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
        void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
        void centroidPoseArrayMsgCallback(const vision_msgs::Detection3DArray::ConstPtr &msg);

        std::vector<double> getPresentJointAngle();
        std::vector<double> getPresentKinematicsPose();

        bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
        bool setTaskSpacePath(std::vector<double> kinematics_pose, double path_time);
        bool setToolControl(std::vector<double> joint_angle);

        void setGoal();

        //  bool setToolControl(std::vector<double> joint_angle);
        void cutter_open(double);
        void cutter_close(double);

        void objectPublisher(void);
        void process(void);
        void init_pose(double);
        void place_pose(double);
        void drop_pose(double);
    };
}

#endif //__TROPICANA_H