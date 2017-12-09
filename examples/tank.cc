#include "exampleapp.hh"

#include "Filter/extendedkalmanfilter.hh"

#include <random>
#include <Eigen/Eigenvalues>
#include <iostream>

constexpr int SCREEN_WIDTH = 800;
constexpr int SCREEN_HEIGHT = 800;

constexpr double LANDMARK_X = 0.33;
constexpr double LANDMARK_Y = 0.33;

using namespace rbest;
using namespace Eigen;

static constexpr double controlVelocity = 0.01;
static constexpr double controlTurn = 5.0 / 180.0 * M_PI;
  

class TankSystemModel : public DifferentiableSystemModel<double, 3, 2>
{
public:
  using Base = DifferentiableSystemModel<double, 3, 2>;
  using typename Base::StateVector;
  using typename Base::ControlVector;
  using typename Base::Jacobian;

  StateVector predict(StateVector const& state, ControlVector const& control) const override;
  Jacobian getJacobian(StateVector const& state, ControlVector const& control) const override;
};

/** Model observation of a landmark and a compass
 */
class TankObservationModel : public DifferentiableObservationModel<double, 3, 3>
{
public:
  using Base = DifferentiableObservationModel<double, 3, 3>;
  using typename Base::StateVector;
  using typename Base::ObservationVector;
  using typename Base::Jacobian;

  ObservationVector observe(StateVector const& state) const override;
  Jacobian getJacobian(StateVector const& state) const override;
};

class TankApp : public ExampleApp
{
protected:
  void initSystem() override;
  void initFilter() override;
  
  void onStepStart() override;
  void handleEvent(SDL_Event const& event) override;
  void updateState() override;
  void updateFilter() override;
  void render() override;

private:
  static constexpr double systemNoiseStd = 0.001;
  static constexpr double observationNoiseStd = 0.5;

  Vector3d mState;

  std::default_random_engine mRandGen;
  std::function<double()> mObservationNoise;
  std::function<double()> mSystemNoise;

  using Filter = ExtendedKalmanFilter<TankSystemModel, TankObservationModel>;
  Filter mFilter;
  Filter::SystemModelType mSystemModel;
  Filter::ObservationModelType mObservationModel;
  
  Filter::ControlVector mControl;
  Filter::ObservationVector mObservation;
};



void TankApp::initSystem()
{
  mState = Vector3d::Zero();
  
  mSystemNoise = [this]() {
      auto systemNoiseDist = std::normal_distribution<double>{0., systemNoiseStd};
      return systemNoiseDist(mRandGen);
  };

  mObservationNoise = [this]() {
    auto observationNoiseDist = std::normal_distribution<double>{0., observationNoiseStd};
    return observationNoiseDist(mRandGen);
  };
}

void TankApp::initFilter()
{
  mFilter.init(Filter::StateVector::Ones() * 0.2,
               Filter::StateCovar::Identity() * 0.25 + Filter::StateCovar::Ones() * 0.25);
}

void TankApp::onStepStart()
{
}

void TankApp::handleEvent(SDL_Event const& event)
{
  switch (event.type)
  {
  case SDL_KEYDOWN:
  case SDL_KEYUP:
    switch (event.key.keysym.sym)
    {
    case SDLK_UP:
      mControl(0) = event.key.state == SDL_PRESSED ? 1.0 : 0.0;
      break;
    case SDLK_DOWN:
      mControl(0) = event.key.state == SDL_PRESSED ? -1.0 : 0.0;
      break;
    case SDLK_LEFT:
      mControl(1) = event.key.state == SDL_PRESSED ? -1.0 : 0.0;
      break;
    case SDLK_RIGHT:
      mControl(1) = event.key.state == SDL_PRESSED ? 1.0 : 0.0;
      break;
    }
    break;
  }      
}

void TankApp::updateState()
{
  mState = mSystemModel.predict(mState, mControl);
  mState.x() += mSystemNoise();
  mState.y() += mSystemNoise();
  mState.z() += mSystemNoise();

  mObservation = mObservationModel.observe(mState);
  mObservation(0) += mObservationNoise();
  mObservation(1) += mObservationNoise();
  mObservation(2) += mObservationNoise();
}

void TankApp::updateFilter()
{
  mFilter.predict(mSystemModel, mControl);
  mFilter.update(mObservationModel, mObservation);
}

void TankApp::render()
{
  // Function to determine pixel coordinate from x,y in tank space
  auto toScreenCoord = [this](Vector2d const& coord) {
    auto border = 50;
    return Vector2i{
      border + (coord.x() + 1) * (mScreenWidth / 2 - border),
      border + (coord.y() + 1) * (mScreenHeight / 2 - border),
    };
  };

  // Clear screen
  SDL_SetRenderDrawColor(mRenderer, 0x2E, 0x34, 0x36, 0xFF );
  SDL_RenderClear(mRenderer);

  // Draw border
  auto border1 = toScreenCoord(Vector2d{-1, -1});
  auto border2 = toScreenCoord(Vector2d{1, 1});
  auto border = SDL_Rect{border1.x(), border1.y(),
                         border2.x() - border1.x(), border2.y() - border1.y()};
  SDL_SetRenderDrawColor(mRenderer, 0x55, 0x57, 0x53, 0xFF );
  SDL_RenderDrawRect(mRenderer, &border);

  // Draw estimate
  auto stateEstimate = mFilter.getState();
  auto stateEstimateCovar = mFilter.getStateCovar();
  auto stateEstimateCoord = toScreenCoord(stateEstimate.head<2>());
  filledCircleRGBA(mRenderer, stateEstimateCoord.x(), stateEstimateCoord.y(), 4, 0x8A, 0xE2, 0x34, 0xFF);
  
  // Covar elipse
  // Find axes
  auto solver = EigenSolver<Matrix<double, 2, 2>>{stateEstimateCovar.block<2, 2>(0, 0)};
  auto eigenVectors = solver.eigenvectors().real();
  auto eigenValues = solver.eigenvalues().real();

  std::cout << "Vectors:\n" << eigenVectors << "\n";
  std::cout << "Values:\n" << eigenValues << "\n";

  auto angle = atan2(eigenVectors(0, 1), eigenVectors(1, 1));
  std::cout << "Angle: " << angle << "\n";
  auto rot = Rotation2Dd{angle};
  std::cout << "Rotation:\n" << rot.matrix() << "\n";

  auto ellipsPoints = std::array<Vector2i, 40>{};
  for (int i = 0; i < ellipsPoints.size(); ++i)
  {
    ellipsPoints[i] = toScreenCoord((stateEstimate.head<2>() + rot * Vector2d{
        eigenValues.x() * cos(M_PI * 2 * i / ellipsPoints.size()),
          eigenValues.y() * sin(M_PI * 2 * i / ellipsPoints.size())
          }));
  }

  auto p1 = ellipsPoints[0];
  for (int i = 1; i <= ellipsPoints.size(); ++i)
  {
    auto p2 = i == ellipsPoints.size() ? ellipsPoints[0] : ellipsPoints[i];
    SDL_RenderDrawLine(mRenderer, p1.x(), p1.y(), p2.x(), p2.y());
    p1 = p2;
  }

  // Draw landmark
  auto landmarkCoord = toScreenCoord(Vector2d{LANDMARK_X, LANDMARK_Y});
  filledCircleRGBA(mRenderer, landmarkCoord.x(), landmarkCoord.y(), 5, 0xFC, 0xE9, 0x4F, 0xFF);
  circleRGBA(mRenderer, landmarkCoord.x(), landmarkCoord.y(), 8, 0xED, 0xD4, 0x00, 0xFF);
  circleRGBA(mRenderer, landmarkCoord.x(), landmarkCoord.y(), 11, 0xC4, 0xA0, 0x00, 0xFF);
  
  // Draw agent
  auto agentCoord = toScreenCoord(mState.head<2>());
  circleRGBA(mRenderer, agentCoord.x(), agentCoord.y(), 20, 0xFC, 0xAF, 0x3E, 0xFF);
  auto thetaDeg = mState.z() / M_PI * 180.0;
  filledPieRGBA(mRenderer, agentCoord.x(), agentCoord.y(), 20, thetaDeg - 10, thetaDeg + 10, 0xFC, 0xAF, 0x3E, 0xFF);
}



TankSystemModel::StateVector TankSystemModel::predict(StateVector const& state, ControlVector const& control) const
{
  auto newState = StateVector{};
  newState.x() = state.x() + cos(state.z()) * control(0) * controlVelocity;
  newState.y() = state.y() + sin(state.z()) * control(0) * controlVelocity;
  newState.z() = state.z() + control(1) * controlTurn;
  return newState;
}

TankSystemModel::Jacobian TankSystemModel::getJacobian(StateVector const& state, ControlVector const& control) const
{
  /*
    dx1/dx0 = 1  dx1/dy0 = 0  dx1/dt0 = -sin(t0)*u0
    dy1/dx0 = 0  dy1/dy0 = 1  dy1/dt0 = cos(t0)*u0
    dt1/dx0 = 0  dt1/dy0 = 0  dt1/dt0 = 1
   */

  Jacobian jacobian = Jacobian::Identity();
  jacobian(0, 2) = -sin(state.z()) * control(0) * controlVelocity;
  jacobian(1, 2) = cos(state.z()) * control(0) * controlVelocity;
  return jacobian;
}



TankObservationModel::ObservationVector TankObservationModel::observe(StateVector const& state) const
{
  auto observation = ObservationVector{};
  Vector2d landmarkDelta = Vector2d{LANDMARK_X, LANDMARK_Y} - state.head<2>();
  
  observation(0) = landmarkDelta.norm();
  observation(1) = atan2(landmarkDelta.y(), landmarkDelta.x()) - state.z();
  observation(2) = state.z();
  return observation;
}

TankObservationModel::Jacobian TankObservationModel::getJacobian(StateVector const& state) const
{
  Vector2d landmarkDelta = Vector2d{LANDMARK_X, LANDMARK_Y} - state.head<2>();
  return (Jacobian{} <<
    -landmarkDelta.x() / landmarkDelta.norm(), -landmarkDelta.y() / landmarkDelta.norm(), 0,
          landmarkDelta.y() / landmarkDelta.squaredNorm(), landmarkDelta.x() / landmarkDelta.squaredNorm(), -1,
          0, 0, 1
    ).finished();
}

int main(int argc, char const** argv)
{
  auto app = TankApp{};

  app.init(SCREEN_WIDTH, SCREEN_HEIGHT, "RBEst - Tank [Extended Kalman Filter]");
  app.run();
    
  return 0;
}
