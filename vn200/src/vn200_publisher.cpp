#include <vn200_node.hpp>
#include <ros/ros.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "IMU");

    ros::NodeHandle n;
    ros::Publisher imu_publisher = n.advertise<sensor_msgs::Imu > ("imu", 30, true);

    vectornav::VN200* vn200 = new vectornav::VN200();

    /* File pointer to output values. */
    FILE *acc_out, *angRate_out;

    /* Variable from get_param() that decides plotting. */
    bool plot;
    n.param("/vn200_node/plot", plot, false);

    if(plot) {
        ROS_INFO("Plot will be formed after %d iterations\n", iterPlot);
    }
    else {
        ROS_INFO("Plot not being formed\n");
    }

    if(plot) {
        acc_out = fopen("acc.dat","w");
        angRate_out = fopen("angRate.dat","w");
    }
    
    sensor_msgs::Imu imu_data;

    ros::Rate publisher_rate(10);

    ROS_INFO("Start publishing...\n");

    while (ros::ok()) {

        ros::spinOnce();

        imu_data = vn200->getIMUData();

        imu_publisher.publish(imu_data);

        if(plot){
            fprintf(acc_out,"%d %lf %lf %lf\n",
                imu_data.header.seq,
                imu_data.linear_acceleration.x,
                imu_data.linear_acceleration.y,
                imu_data.linear_acceleration.z);
            fprintf(angRate_out,"%d %lf %lf %lf\n",
                imu_data.header.seq,
                imu_data.orientation.x,
                imu_data.orientation.y,
                imu_data.orientation.z);
            if(imu_data.header.seq > iterPlot){
                FILE* p=popen("gnuplot -persist","w");
                fprintf(p, "plot \"acc.dat\" using 1:2 title 'x' with lines, \"acc.dat\" using 1:3 title 'y' with lines, \"acc.dat\" using 1:4 title 'z' with lines\n");
                fflush(p);
                fclose(p);
                p=popen("gnuplot -persist","w");
                fprintf(p, "plot 'angRate.dat' using 1:2 title 'yaw' with lines, 'angRate.dat' using 1:3 title 'pitch' with lines, 'angRate.dat' using 1:4 title 'roll' with lines\n");
                fflush(p);
                fclose(p);
                exit(0);
            }
        }
        publisher_rate.sleep();

    }
    if(plot == 1){
        fclose(acc_out);
        fclose(angRate_out);
    }
    return 0;
}
