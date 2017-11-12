#include <gtest/gtest.h>

#include "Filter/extendedkalmanfilter.hh"
#include "Filter/kalmanfilter.hh"
#include "SystemModel/linearsystemmodel.hh"
#include "ObservationModel/linearobservationmodel.hh"

TEST(ExtendedKalmanFilter, linear)
{
  // An EKF with linear system and observation model should perform
  // the same as a regular Kalman filter
  auto ekf = rbest::ExtendedKalmanFilter<
    rbest::LinearSystemModel<double, 2, 1>,
    rbest::LinearObservationModel<double, 2, 1>>{};
  ekf.init(Eigen::Vector2d::Zero(), Eigen::Matrix2d::Ones());

  auto kf = rbest::KalmanFilter<double, 2, 1, 1>{};
  kf.init(Eigen::Vector2d::Zero(), Eigen::Matrix2d::Ones());

  auto systemModel = rbest::LinearSystemModel<double, 2, 1>{};
  systemModel.setTransitionMatrix((Eigen::Matrix2d{} << 1., 1., 0., 1.).finished());
  systemModel.setSystemNoiseCovar((Eigen::Matrix2d::Identity() * 0.1));
  
  auto observationModel = rbest::LinearObservationModel<double, 2, 1>{};
  observationModel.setObservationMatrix(Eigen::Vector2d{1., 0.}.transpose());
  observationModel.setObservationNoiseCovar(Eigen::Matrix<double, 1, 1>{0.1});

  for (int i = 0; i < 100; ++i)
  {
    kf.predict(systemModel, Eigen::Matrix<double, 1, 1>{0.0});
    ekf.predict(systemModel, Eigen::Matrix<double, 1, 1>{0.0});
    ASSERT_EQ(kf.getState(), ekf.getState());
    ASSERT_EQ(kf.getStateCovar(), ekf.getStateCovar());

    kf.update(observationModel, Eigen::Matrix<double, 1, 1>{i * 0.1});
    ekf.update(observationModel, Eigen::Matrix<double, 1, 1>{i * 0.1});
    ASSERT_EQ(kf.getState(), ekf.getState());
    ASSERT_EQ(kf.getStateCovar(), ekf.getStateCovar());
  }
}
