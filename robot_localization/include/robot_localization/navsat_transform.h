#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_datatypes.h>

#include <Eigen/Dense>
#include <fstream>

//##@Rahul:The main aim of the this node is to convert gps data into <UTM> coordinate and the use this to fuse with <wheel Odometry> and <IMU> data to find the fused <GPS_Odometry>

namespace RobotLocalization
{
  class NavSatTransform
  {
    public:

	  ofstream file;
      file.open("data_filtered.txt");
      //! @brief Constructor
      //!
=======


      NavSatTransform();

//@Rahul:<run> function is used by the <navsat_transform_node> 
      void run();

    private:

//@Rahul:We would set this parameter to <false> as we're not broadcasting the UTM transform.
      bool broadcastUtmTransform_;

//@Rahul:Its the offset value of <YAW> given by Vecternov needs to be set in the <NAVSAT_TRANSFORM.LAUNCH> & <NAVSAT_TRANSFORM.CPP>
      double magneticDeclination_;

//@Rahul:These store the <YPR> data to be converted to <UTM> via utm transform.As <NO GPS> means no concerned to these data.
      double utmOdomTfRoll_;
      double utmOdomTfPitch_;
      double utmOdomTfYaw_;

//@Rahul:Initially all these <Variables> are set as <FALSE> and it changes to true when <navsat_transform_node> subscribes any data.

      bool hasGps_;		//@Rahul:As we are not using GPS so it will be set as <FALSE> always as <navsat_transform_node> 				  will not subscribe any gps data as we are not publishing gps data.
				//@Rahul:If we dont have <GPS_FIX> i.e gps so set it as <FALSE>
      bool hasOdom_;		//@Rahul:We are using wheel Odometry so it will be set as <TRUE> when <navsat_transform_node> 					  subscribes odometry data.
      bool hasImu_;		//@Rahul:We are using <VectorNav> for IMU so it will be set <TRUE> when IMU data is subscribed.

//@Rahul:This is set as <TRUE> when all the data has been received from all the 3 sensors namely <gps>, <imu> and <odometry> and UTM transfrom of all the sensors have been calculcated.It give whether we have good heading or not.
      bool transformGood_;

//@Rahul:Use for Whether or not we have new GPS data.We only want to compute and broadcast our transformed GPS data if it's new. This variable keeps track of that.
      bool gpsUpdated_;

      //! @brief Timestamp of the latest good GPS message
      //!
      //! We assign this value to the timestamp of the odometry
      //! message that we output
      ros::Time gpsUpdateTime_;

//@Rahul:There are <OFFSET> parameters
      double rollOffset_;
      double pitchOffset_;
      double yawOffset_;

      //! @brief Whether or not to report 0 altitude
      //!
      //! If this parameter is true, we always report 0 for the altitude
      //! of the converted GPS odometry message.

//@Rahul:We set <zeroAltitude> to <FALSE> as we have <2D Localization> and our assumption is that surface is <Planar>
      bool zeroAltitude_;

      //! @brief Frame ID of the GPS odometry output
      //!
      //! This will just match whatever your odometry message has
      //!
      std::string worldFrameId_;

//@Rahul:Variables to store the latest subscribed data from the node.i.e. <imu> <gps> <odometry>
      tf::Pose latestWorldPose_;
      tf::Pose latestUtmPose_;
      tf::Quaternion latestOrientation_;

      //! @brief Covariane for most recent GPS/UTM data
      Eigen::MatrixXd latestUtmCovariance_;

      //! @brief Holds the UTM->odom transform
      tf::Transform utmWorldTransform_;

//@Rahul:CallBack functions when ever <Odom> and <gps> and <imu> data is received.

      void odomCallback(const nav_msgs::OdometryConstPtr& msg);
      void gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg);
      void imuCallback(const sensor_msgs::ImuConstPtr& msg);

      //! @brief Computes the transform from the UTM frame to the
      //! odom frame
      void computeTransform();

      //! @brief Prepares the GPS odometry message before sending
      bool prepareGpsOdometry(nav_msgs::Odometry &gpsOdom);
  };
}
