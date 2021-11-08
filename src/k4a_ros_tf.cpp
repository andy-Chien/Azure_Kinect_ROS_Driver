#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "k4a_ros_tf");

    ros::NodeHandle node;

    std::vector<double> base_h_rgb_list;
    node.getParam("/base_h_rgb", base_h_rgb_list);
    tf2::Transform base_h_rgb(tf2::Quaternion(base_h_rgb_list[4], 
                                               base_h_rgb_list[5],
                                               base_h_rgb_list[6],
                                               base_h_rgb_list[3]),
                                    tf2::Vector3(base_h_rgb_list[0],
                                                 base_h_rgb_list[1],
                                                 base_h_rgb_list[2]));

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped camera_h_rgb_stamped;


    while (node.ok()){
        try{
            camera_h_rgb_stamped = tfBuffer.lookupTransform("camera_base", "rgb_camera_link",
                                    ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            ros::spinOnce();
            continue;
        }
        break;
    }

        
    tf2::Transform camera_h_rgb(tf2::Quaternion(camera_h_rgb_stamped.transform.rotation.x, 
                                                camera_h_rgb_stamped.transform.rotation.y,
                                                camera_h_rgb_stamped.transform.rotation.z,
                                                camera_h_rgb_stamped.transform.rotation.w),
                                tf2::Vector3(camera_h_rgb_stamped.transform.translation.x,
                                                camera_h_rgb_stamped.transform.translation.y,
                                                camera_h_rgb_stamped.transform.translation.z));

    tf2::Transform base_h_camera;
    base_h_camera.mult(base_h_rgb, camera_h_rgb.inverse());

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "base";
    static_transformStamped.child_frame_id = "camera_base";
    static_transformStamped.transform.translation.x = base_h_camera.getOrigin()[0];
    static_transformStamped.transform.translation.y = base_h_camera.getOrigin()[1];
    static_transformStamped.transform.translation.z = base_h_camera.getOrigin()[2];
    static_transformStamped.transform.rotation.x = base_h_camera.getRotation().x();
    static_transformStamped.transform.rotation.y = base_h_camera.getRotation().y();
    static_transformStamped.transform.rotation.z = base_h_camera.getRotation().z();
    static_transformStamped.transform.rotation.w = base_h_camera.getRotation().w();
    static_broadcaster.sendTransform(static_transformStamped);
    ros::spin();
    return 0;
};