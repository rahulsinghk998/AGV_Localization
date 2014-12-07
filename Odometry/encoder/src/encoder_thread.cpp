#include <encoder/encoder.h>
#include <geometry_msgs/Point.h>

int main() {
    
    encoder::Encoder encoder(ENCODER_COM_PORT, ENCODER_BAUD_RATE);
    encoder::EncoderData encoderData;

    ros::NodeHandle n;
    ros::Publisher encoder_pub = n.advertise<geometry_msgs::Point>("encoderData", 1000);
   
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        /* Fetch data from Shaft Encoder and load it in local vars */
        encoderData = encoder.fetchEncoderData();

        geometry_msgs::Point msg;
        msg.x = encoderData.leftCount;
        msg.y = encoderData.rightCount;

        encoder_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();

        // pthread_mutex_lock(&pose_mutex);

        /* Update the pose_data using the data in local vars */

        // pthread_mutex_unlock(&pose_mutex);

        // usleep(10);
    }

    return 0;
}

