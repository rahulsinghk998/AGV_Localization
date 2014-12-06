#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
#include <eklavya_odometry/odometry.h>

using namespace std;

odometry_space::OdometryFactory *odometry_factory;

/* This method cannot return void, it must return a msg or similar. Fix countless of such errors first. */
void encoderCallback(const encoder_space::EncoderData& msg) {

    ROS_INFO("Encoder data received...");

    odometry_factory->updateOdometryData(msg);
    odometry_factory->NUM_COMMANDS++;

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "odometry");

    odometry_factory = new odometry_space::OdometryFactory();

    ros::NodeHandle n;
    ros::Publisher odometry_publisher = n.advertise<nav_msgs::Odometry > ("odometry", 30, true);
    ros::Subscriber encoder_subscriber = n.subscribe("encoderData", 1, encoderCallback);

    nav_msgs::Odometry odometry_message;

    ros::Rate publisher_rate(10);

    printf("Odometry node initialized...\n");

    while (ros::ok()) {

        ros::spinOnce();

        /* So you mean to say you get data members from the odometry_factory object and then save the same data members to another attribute of odometry factory????*/
        /* This is so wrong!!!*/
        odometry_message = odometry_factory->getOdometryData();
        odometry_factory->file<<odometry_message.pose.pose.position.x<<"\t"<<odometry_message.pose.pose.position.y<<"\n";
        char * commandsForGnuplot[] = {"set title \"TITLEEEEE\"", "plot 'data.txt' with lines"};
        FILE * gnuplotPipe = popen ("gnuplot -persistent", "w");
        int i;
        for (i=0; i < odometry_factory->NUM_COMMANDS; i++){
            fprintf(gnuplotPipe, "%s \n", commandsForGnuplot[i]); //Send commands to gnuplot one by one.
        }
        odometry_publisher.publish(odometry_message);

        publisher_rate.sleep();

    }

    return 0;
}
