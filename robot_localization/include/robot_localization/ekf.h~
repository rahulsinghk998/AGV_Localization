#ifndef RobotLocalization_Ekf_h
#define RobotLocalization_Ekf_h

#include "robot_localization/filter_base.h"

#include <fstream>
#include <vector>
#include <set>
#include <queue>

namespace RobotLocalization
{
  //! @brief Extended Kalman filter class
  //!
  //! Implementation of an extended Kalman filter (EKF). This
  //! class derives from FilterBase and overrides the predict()
  //! and correct() methods in keeping with the discrete time
  //! EKF algorithm.
  //!
  class Ekf: public FilterBase
  {
    public:

      //! @brief Constructor for the Ekf class
      //!
      //! @param[in] args - Generic argument container (not used here, but
      //! needed so that the ROS filters can pass arbitrary arguments to
      //! templated filter types.
      //!
      Ekf(std::vector<double> args = std::vector<double>());

      //! @brief Destructor for the Ekf class
      //!
      ~Ekf();

    protected:

      //! @brief Carries out the correct step in the predict/update cycle.
      //!
      //! @param[in] measurement - The measurement to fuse with our estimate
      //!
      void correct(const Measurement &measurement);

      //! @brief Carries out the predict step in the predict/update cycle.
      //!
      //! Projects the state and error matrices forward using a model of
      //! the vehicle's motion.
      //!
      //! @param[in] delta - The time step over which to predict.
      //!
      void predict(const double delta);

  };
}

#endif
