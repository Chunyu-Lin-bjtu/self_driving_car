#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>


ros::Publisher mapPub;

void fixOrigin(nav_msgs::OccupancyGrid msg){
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;
    mapPub.publish(msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "corrected_map_publisher");
    ros::NodeHandle n;
    mapPub  = n.advertise<nav_msgs::OccupancyGrid>("map",1, true);
    ros::Subscriber projectMapSub = n.subscribe("projected_map", 1, &fixOrigin);
    ros::Rate loopRate = 0.1;
    ros::spinOnce();
    while (ros::ok()){
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}
