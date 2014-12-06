#include <vn200_node.hpp>
#include <ros/ros.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "IMU");

    ros::NodeHandle n;
    ros::Publisher imu_publisher = n.advertise<sensor_msgs::Imu > ("imu", 30, true);

    vectornav::VN200* vn200 = new vectornav::VN200();

    /* File pointer to output values. */
    FILE *acc_out, *angRate_out;

    /* Number of iterations to draw plot. */
    int iterPlot;
    n.param("/vn200_node/iterations", iterPlot, 100);

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
            	/* To do: Cleanup of this terrible hack. */
                fflush(acc_out);
                fclose(acc_out);
                fflush(angRate_out);
                fclose(angRate_out);

                FILE *p = fopen("command.sh", "w");
                fprintf(p, "plot '~/.ros/acc.dat' using 1:2 title 'AccX' with lines, '~/.ros/acc.dat' using 1:3 title 'AccY' with lines, '~/.ros/acc.dat' using 1:4 title 'AccZ' with lines\n");
                fflush(p);
                fclose(p);
                system("gnuplot -persist < ~/.ros/command.sh");
                
                p = fopen("command.sh", "w");
                fprintf(p, "plot '~/.ros/angRate.dat' using 1:2 title 'AngRateX' with lines, '~/.ros/angRate.dat' using 1:3 title 'AngRateY' with lines, '~/.ros/angRate.dat' using 1:4 title 'AngRateZ' with lines\n");
                fflush(p);
                fclose(p);
                system("gnuplot -persist < ~/.ros/command.sh");
                
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
