#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdio.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
 
int main(int argc, char **argv)
{
   ros::init(argc, argv, "pub_setpoints");
   ros::NodeHandle n;
 
   ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);
   ros::Rate loop_rate(20);
   ros::spinOnce();
 
   geometry_msgs::PoseStamped msg;
   int count = 1;  
        //PositionReciever qp;:
        //Body some_object;
        //qp.connect_to_server();
 
     
   while(ros::ok()){
       //some_object = qp.getStatus();
        // some_object.print();
        //printf("%f\n",some_object.position_x);
       msg.header.stamp = ros::Time::now();
       msg.header.seq=count;
       msg.header.frame_id = "abc";
       msg.pose.position.y = -1.561;//0.001*some_object.position_x;
       msg.pose.position.x = -5.860;//0.001*some_object.position_y;
       msg.pose.position.z = 1.0;//0.001*some_object.position_z;
       //msg.pose.orientation.x = -0.0925;
       //msg.pose.orientation.y = -0.0324;
       //msg.pose.orientation.z = 0.0523;
       //msg.pose.orientation.w = 1;
 
       chatter_pub.publish(msg);
       ros::spinOnce();
       loop_rate.sleep();
   }
    
       
   return 0;
}
