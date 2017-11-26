#include "exampleapp.hh"

#include "Filter/kalmanfilter.hh"

#include <iostream>
#include <cmath>
#include <list>
#include <random>

constexpr int SCREEN_WIDTH = 1024;
constexpr int SCREEN_HEIGHT = 768;

using namespace rbest;
using namespace Eigen;

class FlyerApp : public ExampleApp
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
  // System constants
  static constexpr double xSpeed = 5.0;
  static constexpr double controlAccel = 0.5;
  
  static constexpr double systemNoiseStd = 0.1;
  static constexpr double observationNoiseStd = 25.0;

  // Render constants
  static constexpr int gridSize = 100;
  
  Eigen::Vector3d mState;

  using Filter = KalmanFilter<double, 2, 1, 1>;
  Filter mFilter;
  Filter::SystemModelType mSystemModel;
  Filter::ObservationModelType mObservationModel;

  std::default_random_engine mRandGen;
  std::function<double()> mObservationNoise;
  std::function<double()> mSystemNoise;

  Filter::ControlVector mControl;
  Filter::ObservationVector mObservation;
  
  std::list<Vector3d> mStateHistory;
  std::list<Vector3d> mEstHistory;
  std::list<Vector2d> mStdHistory;
  std::list<Vector2d> mObsHistory;

};

constexpr double FlyerApp::xSpeed;
constexpr double FlyerApp::controlAccel;
  
constexpr double FlyerApp::systemNoiseStd;
constexpr double FlyerApp::observationNoiseStd;

constexpr int FlyerApp::gridSize;

void FlyerApp::initSystem()
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

void FlyerApp::initFilter()
{
  mFilter.init(Eigen::Vector2d::Zero(), Eigen::Matrix2d::Ones() * 1e4);
  
  mSystemModel = Filter::SystemModelType{};
  mSystemModel.setTransitionMatrix((Filter::SystemModelType::TransitionMatrix{} <<
                                   1., 1.,
                                   0., 1.).finished());
  mSystemModel.setControlMatrix((Filter::SystemModelType::ControlMatrix{}
<< 0., controlAccel).finished());

  auto systemNoiseVar = systemNoiseStd * systemNoiseStd;
  mSystemModel.setSystemNoiseCovar((Filter::SystemModelType::SystemNoiseCovar{} <<
                                   systemNoiseVar, 0.,
                                   0., systemNoiseVar).finished());

  mObservationModel = Filter::ObservationModelType{};
  mObservationModel.setObservationMatrix((Filter::ObservationModelType::ObservationMatrix{} << 1., 0.).finished());
  auto observationNoiseVar = observationNoiseStd * observationNoiseStd;
  mObservationModel.setObservationNoiseCovar((Filter::ObservationModelType::ObservationNoiseCovar{} << observationNoiseVar).finished());
}


void FlyerApp::onStepStart()
{
  mControl.array() = 0.;
}

void FlyerApp::handleEvent(SDL_Event const& event)
{
  switch (event.type)
  {
  case SDL_KEYDOWN:
    switch (event.key.keysym.sym)
    {
    case SDLK_UP:
      mControl(0) = 1.;
      break;
    case SDLK_DOWN:
      mControl(0) = -1.;
    }
    break;
  }      
}

void FlyerApp::updateState()
{
  // Store history
  auto oldStateEst = mFilter.getState();
  auto oldStateCovar = mFilter.getStateCovar();
  auto oldStateStd = std::sqrt(oldStateCovar(0, 0));
  mObservation(0) = mState.y() + mObservationNoise();

  auto nHistoryOnScreen = mScreenWidth / xSpeed;
  
  mStateHistory.push_back(mState);
  while (mStateHistory.size() > nHistoryOnScreen)
    mStateHistory.pop_front();

  mEstHistory.push_back(Eigen::Vector3d{mState.x(), oldStateEst(0), oldStateEst(1)});
  while (mEstHistory.size() > nHistoryOnScreen)
    mEstHistory.pop_front();
  
  mStdHistory.push_back(Eigen::Vector2d{mState.x(), oldStateStd});
  while (mStdHistory.size() > nHistoryOnScreen)
    mStdHistory.pop_front();
  
  mObsHistory.push_back(Eigen::Vector2d{mState.x(), mObservation(0)});
  while (mObsHistory.size() > nHistoryOnScreen)
    mObsHistory.pop_front();
  
  // Update state
  mState.x() += xSpeed;
  mState.y() += mState.z() + mSystemNoise();
  mState.z() += mControl(0) * controlAccel + mSystemNoise();
}

void FlyerApp::updateFilter()
{
  mFilter.predict(mSystemModel, mControl);
  mFilter.update(mObservationModel, mObservation);
}

void FlyerApp::render()
{
  auto stateEst = mFilter.getState();
  auto stateCovar = mFilter.getStateCovar();
  auto stateStd = std::sqrt(stateCovar(0, 0));


  // Function to determine pixel coordinate from x,y in flyer space
  // Always has flyer at centre vertically and 100 pixels from right window edge
  auto toScreenCoord = [this](Vector2d const& coord) {
    return Vector2i{
      mScreenWidth - 100 + coord.x() - mState.x(),
      mScreenHeight / 2 - coord.y() + mState.y()
    };
  };
  
  // Clear screen
  SDL_SetRenderDrawColor(mRenderer, 0x2E, 0x34, 0x36, 0xFF );
  SDL_RenderClear(mRenderer);

  // Draw grid
  SDL_SetRenderDrawColor(mRenderer, 0x55, 0x57, 0x53, 0xFF );
  for (auto gx = std::fmod(-mState.x(), gridSize); gx < mScreenWidth; gx += gridSize)
    SDL_RenderDrawLine(mRenderer, gx, 0, gx, mScreenHeight);

  for (auto gy = std::fmod(mState.y(), gridSize); gy < mScreenHeight; gy += gridSize)
    SDL_RenderDrawLine(mRenderer, 0, gy, mScreenWidth, gy);

  // Draw history
  // Observation history
  SDL_SetRenderDrawColor(mRenderer, 0xA4, 0x00, 0x00, 0xFF );
  for (auto const& obs : mObsHistory)
  {
    auto coord = toScreenCoord(obs);
    auto obsRect = SDL_Rect{coord.x() - 2, coord.y() - 2, 4, 4};
    SDL_RenderFillRect(mRenderer, &obsRect);
  }

  // State history
  auto siter = mStateHistory.begin();
  auto hs1 = *siter++;
  for ( ; siter != mStateHistory.end(); ++siter)
  {
    auto hs2 = *siter;
    auto coord1 = toScreenCoord(hs1.head<2>());
    auto coord2 = toScreenCoord(hs2.head<2>());
    
    thickLineRGBA(mRenderer, coord1.x(), coord1.y(), coord2.x(), coord2.y(),
                  2, 0xf5, 0x79, 0x00, 0xFF);
    hs1 = hs2;
  }

  // Estimation history
  auto eiter = mEstHistory.begin();
  auto he1 = *eiter++;
  for ( ; eiter != mEstHistory.end(); ++eiter)
  {
    auto he2 = *eiter;
    auto coord1 = toScreenCoord(he1.head<2>());
    auto coord2 = toScreenCoord(he2.head<2>());
    thickLineRGBA(mRenderer, coord1.x(), coord1.y(), coord2.x(), coord2.y(),
                  2, 0x73, 0xD2, 0x16, 0xFF);
    he1 = he2;
  }

  // Standard deviation history
  auto vx = std::vector<int16_t>(mStdHistory.size() * 2);
  auto vy = std::vector<int16_t>(mStdHistory.size() * 2);
  int i = 0;
  auto estIter = mEstHistory.begin();
  for (auto const& v : mStdHistory)
  {
    auto coord = toScreenCoord(estIter->head<2>());
    vx[i] = coord.x();
    vy[i] = coord.y() - v.y() * 2;
        
    vx[vx.size() - i - 1] = coord.x();
    vy[vy.size() - i - 1] = coord.y() + v.y() * 2;

    ++i;
    ++estIter;
  }
  filledPolygonRGBA(mRenderer, vx.data(), vy.data(), vx.size(), 0x4E, 0x9A, 0x06, 0x40);
    
  // Draw flyer
  SDL_SetRenderDrawColor(mRenderer, 0xfC, 0xAF, 0x3E, 0xFF );
  auto coord = toScreenCoord(mState.head<2>());
  auto flyer = SDL_Rect{coord.x() - 2, coord.y() - 2, 4, 4};
  SDL_RenderFillRect(mRenderer, &flyer);
    
  // Draw estimated state
  SDL_SetRenderDrawColor(mRenderer, 0x8A, 0xE2, 0x34, 0xFF );
  coord = toScreenCoord(Vector2d{mState.x(), stateEst(0)});
  auto flyerEst = SDL_Rect{coord.x() - 2, coord.y() - 2,
                           4, 4};
  SDL_RenderFillRect(mRenderer, &flyerEst);

  SDL_SetRenderDrawColor(mRenderer, 0x4E, 0x9A, 0x06, 0xFF );
  auto flyerError1 = SDL_Rect{coord.x() - 2, coord.y() - stateStd * 2 - 2,
                              4, 4};
  SDL_RenderFillRect(mRenderer, &flyerError1);
  auto flyerError2 = SDL_Rect{coord.x() - 2, coord.y() + stateStd * 2 - 2,
                              4, 4};
  SDL_RenderFillRect(mRenderer, &flyerError2);
}

int main(int argc, char const** argv)
{
  auto app = FlyerApp{};

  app.init(SCREEN_WIDTH, SCREEN_HEIGHT, "RBEst - Flyer [Kalman Filter]");
  app.run();
    
  return 0;
}
