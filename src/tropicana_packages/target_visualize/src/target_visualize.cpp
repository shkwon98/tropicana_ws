#include "target_visualize.h"

visualization_msgs::MarkerArray m_point_markers;

void point_sub_callback(const vision_msgs::Detection3DArray::ConstPtr &msg)
{
    for (int i = 0; i < m_point_markers.markers.size(); i++)
    {
        m_point_markers.markers.at(i).action = visualization_msgs::Marker::DELETEALL;

    }
    m_point_markers.markers.clear();

    if (msg->detections.size() == 0)
    {
        ROS_INFO("No object detected.");
        return;
    }

    else
    {
        for (int i = 0; i < msg->detections.size(); i++)
        {
            // Target point with respect to camera coordinate
            const float c_point_x = msg->detections.at(i).bbox.center.position.x;
            const float c_point_y = msg->detections.at(i).bbox.center.position.y;
            const float c_point_z = msg->detections.at(i).bbox.center.position.z;

            // Target point with respect to manipulator coordinate
            const float m_point_x = cos(rad_offset) * c_point_x - sin(rad_offset) * c_point_z + x_offset;
            const float m_point_y = (c_point_y + y_offset) * y_gain;
            const float m_point_z = sin(rad_offset) * c_point_x + cos(rad_offset) * c_point_z + z_offset;

            ROS_INFO("Target %d", i);
            ROS_INFO("Before transformation :  %.3f, %.3f, %.3f  ", c_point_x, c_point_y, c_point_z);
            ROS_INFO("After transformation : %.3f, %.3f, %.3f  ", m_point_x, m_point_y, m_point_z);

            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.id = i;
            marker.ns = "target object";
            //marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = m_point_x;
            marker.pose.position.y = m_point_y;
            marker.pose.position.z = m_point_z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration(0.25);

            m_point_markers.markers.push_back(marker);
        }
    }
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "target_visualize_node");
    ros::NodeHandle nh;
    ROS_INFO("==== Target visualize node is started\n");

    ros::Subscriber point_sub = nh.subscribe("/detection_3d/detection_3d", 1, point_sub_callback);
    ros::Publisher transformed_point_pub = nh.advertise<visualization_msgs::MarkerArray>("/transformed_point", 5);
    ros::Rate r(10);


    while (ros::ok())
    {
        ros::spinOnce();
        transformed_point_pub.publish(m_point_markers);
        r.sleep();
    }



    return 0;
}