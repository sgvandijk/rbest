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

class SinObservationModel : public rbest::DifferentiableObservationModel<double, 1, 1>
{
public:
  using StateVector = Eigen::Matrix<double, 1, 1>;
  using ObservationVector = Eigen::Matrix<double, 1, 1>;

  Eigen::Matrix<double, 1, 1> getJacobian(StateVector const& state) const override
  {
    return state.array().cos();
  }
  
  ObservationVector observe(StateVector const& state) const override
  {
    return state.array().sin();
  }
};

TEST(ExtendedKalmanFilter, cos_observation)
{
  // Stationary system where observations are passed through a sine
  auto filter = rbest::ExtendedKalmanFilter<rbest::LinearSystemModel<double, 1, 1>, SinObservationModel>{};
  using OneVector = Eigen::Matrix<double, 1, 1>;
  
  auto systemModel = rbest::LinearSystemModel<double, 1, 1>{};  
  systemModel.setTransitionMatrix((OneVector{} << 1.0).finished());
  systemModel.setControlMatrix((OneVector{} << 0.0).finished());
  systemModel.setSystemNoiseCovar(OneVector::Zero());

  auto observationModel = SinObservationModel{};
  observationModel.setObservationNoiseCovar(OneVector::Ones() * 0.001);

  filter.init(OneVector::Ones(), OneVector::Ones());

  auto value = 0.6;
  
  for (int i = 0; i < 100; ++i)
  {
    filter.predict(systemModel, OneVector::Zero());
    OneVector obs = OneVector{value}.array().sin() + OneVector::Random().array() * 0.0005;
    filter.update(observationModel, obs);
  }

  ASSERT_NEAR(value, filter.getState()[0], 1e-3);
  ASSERT_GT(1e-3, filter.getStateCovar()(0, 0));
}
