#ifndef TROPICANA_TEST_H
#define TROPICANA_TEST_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <termios.h>

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include "vision_msgs/BoundingBox3DArray.h"

#define NUM_OF_JOINT 4

namespace tropicana_test
{

    class TROPICANA_TEST
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
        TROPICANA_TEST();
        ~TROPICANA_TEST();

        void initPublisher();
        void initSubscriber();

        void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
        void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
        void centroidPoseArrayMsgCallback(const vision_msgs::BoundingBox3DArray &msg);

        std::vector<double> getPresentJointAngle();
        std::vector<double> getPresentKinematicsPose();

        bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
        bool setTaskSpacePath(std::vector<double> kinematics_pose, double path_time);
        bool setToolControl(std::vector<double> joint_angle);

        void setGoal();

      //  bool setToolControl(std::vector<double> joint_angle);
        void gripper_open(void);
        void gripper_close(void);

        void objectPublisher(void);
        void process(void);
        void init_pose(void);
        void home_pose(void);
        void place_pose(void);

    };
}

#endif //TROPICANA_TEST_H