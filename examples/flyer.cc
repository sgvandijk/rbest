#include "Filter/kalmanfilter.hh"

#include <SDL.h>
#include <SDL2_gfxPrimitives.h>

#include <iostream>
#include <cmath>
#include <list>
#include <random>

constexpr int SCREEN_WIDTH = 1024;
constexpr int SCREEN_HEIGHT = 768;

int main(int argc, char const** argv)
{
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << "\n";
    return -1;
  }

  auto window = SDL_CreateWindow("RBEst - Flyer [Kalman Filter]",
                                 SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                 SCREEN_WIDTH, SCREEN_HEIGHT,
                                 SDL_WINDOW_SHOWN );
  if (window == nullptr)
  {
    std::cerr << "Window could not be created! SDL_Error: " << SDL_GetError() << "\n";
    return -1;
  }

  auto renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED );
  if(renderer == nullptr )
  {
    std::cerr << "Renderer could not be created! SDL Error: " << SDL_GetError() << "\n";
    return -1;
  }
  
  // State consists of height and vertical velocity
  // Flyer moves horizontally at constant speed, which is not filtered
  auto realState = Eigen::Vector3d{0.0, 0.0, 0.0};
  auto stateHistory = std::list<Eigen::Vector3d>{};  
  auto estHistory = std::list<Eigen::Vector3d>{};
  auto stdHistory = std::list<Eigen::Vector2d>{};
  auto obsHistory = std::list<Eigen::Vector2d>{};
  
  auto xSpeed = 5.0;

  auto randGen = std::default_random_engine{};

  auto systemNoiseStd = 0.1;
  auto systemNoiseVar = systemNoiseStd * systemNoiseStd;
  auto systemNoiseDist = std::normal_distribution<double>{0., systemNoiseStd};
  auto systemNoise = [&]() { return systemNoiseDist(randGen); };

  auto observationNoiseStd = 25.;
  auto observationNoiseVar = observationNoiseStd * observationNoiseStd;
  auto observationNoiseDist = std::normal_distribution<double>{0., observationNoiseStd};
  auto observationNoise = [&]() { return observationNoiseDist(randGen); };
  
  // Filter
  // State to be filtered consists of height and vertical velocity
  // Only vertical velocity is controlled, and only height is observed
  using Filter = rbest::KalmanFilter<double, 2, 1, 1>;
  auto filter = Filter{};
  filter.init(Eigen::Vector2d(0, 0), Eigen::Matrix2d::Ones() * 1e4);
  
  auto systemModel = Filter::SystemModelType{};
  systemModel.setTransitionMatrix((Filter::SystemModelType::TransitionMatrix{} <<
                                   1., 1.,
                                   0., 1.).finished());
  systemModel.setControlMatrix((Filter::SystemModelType::ControlMatrix{} << 0., 0.05).finished());
  systemModel.setSystemNoiseCovar((Filter::SystemModelType::SystemNoiseCovar{} <<
                                   systemNoiseVar, 0.,
                                   0., systemNoiseVar).finished());

  auto observationModel = Filter::ObservationModelType{};
  observationModel.setObservationMatrix((Filter::ObservationModelType::ObservationMatrix{} << 1., 0.).finished());
  observationModel.setObservationNoiseCovar((Filter::ObservationModelType::ObservationNoiseCovar{} << observationNoiseVar).finished());
  
  // Visualisation details
  auto gridSize = 100;
  
  auto quit = false;
  auto paused = false;
  auto event = SDL_Event{};
  while (!quit)
  {
    auto control = 0.0;
    
    while (SDL_PollEvent(&event) != 0)
    {
      switch (event.type)
      {
      case SDL_QUIT:
        quit = true;
        break;
        
      case SDL_KEYDOWN:
        switch (event.key.keysym.sym)
        {
        case SDLK_q:
          quit = true;
          break;
        case SDLK_p:
          paused = !paused;
          break;
        case SDLK_UP:
          control = 1.;
          break;
        case SDLK_DOWN:
          control = -1.;
        }
        break;
      }      
    }

    if (paused)
    {
      SDL_Delay(10);
      continue;
    }
    
    auto yAccel = control * 0.1;

    // Store history
    auto oldStateEst = filter.getState();
    auto oldStateCovar = filter.getStateCovar();
    auto oldStateStd = std::sqrt(oldStateCovar(0, 0));
    auto observation = realState.y() + observationNoise();

    stateHistory.push_back(realState);
    while (stateHistory.size() > SCREEN_WIDTH / xSpeed)
      stateHistory.pop_front();

    estHistory.push_back(Eigen::Vector3d{realState.x(), oldStateEst(0), oldStateEst(1)});
    while (estHistory.size() > SCREEN_WIDTH / xSpeed)
      estHistory.pop_front();

    stdHistory.push_back(Eigen::Vector2d{realState.x(), oldStateStd});
    while (stdHistory.size() > SCREEN_WIDTH / xSpeed)
      stdHistory.pop_front();

    obsHistory.push_back(Eigen::Vector2d{realState.x(), observation});
    while (obsHistory.size() > SCREEN_WIDTH / xSpeed)
      obsHistory.pop_front();

    // Update state
    realState.x() += xSpeed;
    realState.y() += realState.z() + systemNoise();
    realState.z() += yAccel + systemNoise();

    // Run filter
    filter.predict(systemModel, Eigen::Matrix<double, 1, 1>{control});
    filter.update(observationModel, Eigen::Matrix<double, 1, 1>{observation});

    auto stateEst = filter.getState();
    auto stateCovar = filter.getStateCovar();
    auto stateStd = std::sqrt(stateCovar(0, 0));

    // render

    // Clear screen
    SDL_SetRenderDrawColor(renderer, 0x2E, 0x34, 0x36, 0xFF );
    SDL_RenderClear(renderer);

    // Draw grid
    SDL_SetRenderDrawColor(renderer, 0x55, 0x57, 0x53, 0xFF );
    for (auto gx = std::fmod(-realState.x(), gridSize); gx < SCREEN_WIDTH; gx += gridSize)
    {
      SDL_RenderDrawLine(renderer, gx, 0, gx, SCREEN_HEIGHT);
    }

    for (auto gy = std::fmod(realState.y(), gridSize); gy < SCREEN_HEIGHT; gy += gridSize)
    {
      SDL_RenderDrawLine(renderer, 0, gy, SCREEN_WIDTH, gy);
    }

    // Draw history
    // Observation history
    SDL_SetRenderDrawColor(renderer, 0xA4, 0x00, 0x00, 0xFF );
    for (auto const& obs : obsHistory)
    {
      auto obsRect = SDL_Rect{int(SCREEN_WIDTH - 100 + obs.x() - realState.x() - 2),
                              int(SCREEN_HEIGHT / 2 - obs.y() + realState.y() - 2),
                              4, 4};
      SDL_RenderFillRect(renderer, &obsRect);
    }

    // State history
    auto siter = stateHistory.begin();
    auto hs1 = *siter++;
    for ( ; siter != stateHistory.end(); ++siter)
    {
      auto hs2 = *siter;
      thickLineRGBA(renderer,
                     SCREEN_WIDTH - 100 + hs1.x() - realState.x(), SCREEN_HEIGHT / 2 - hs1.y() + realState.y(),
                     SCREEN_WIDTH - 100 + hs2.x() - realState.x(), SCREEN_HEIGHT / 2 - hs2.y() + realState.y(),
                     2, 0xf5, 0x79, 0x00, 0xFF);
      hs1 = hs2;
    }

    // Estimation history
    auto eiter = estHistory.begin();
    auto he1 = *eiter++;
    for ( ; eiter != estHistory.end(); ++eiter)
    {
      auto he2 = *eiter;
      thickLineRGBA(renderer,
                    SCREEN_WIDTH - 100 + he1.x() - realState.x(), SCREEN_HEIGHT / 2 - he1.y() + realState.y(),
                    SCREEN_WIDTH - 100 + he2.x() - realState.x(), SCREEN_HEIGHT / 2 - he2.y() + realState.y(),
                    2, 0x73, 0xD2, 0x16, 0xFF);
      he1 = he2;
    }

    // Standard deviation history
    auto vx = std::vector<int16_t>(stdHistory.size() * 2);
    auto vy = std::vector<int16_t>(stdHistory.size() * 2);
    int i = 0;
    auto estIter = estHistory.begin();
    for (auto const& v : stdHistory)
    {
        vx[i] = SCREEN_WIDTH - 100 + v.x() - realState.x();
        vy[i] = SCREEN_HEIGHT / 2 - estIter->y() + realState.y() - v.y() * 2;
        
        vx[vx.size() - i - 1] = SCREEN_WIDTH - 100 + v.x() - realState.x();
        vy[vy.size() - i - 1] = SCREEN_HEIGHT / 2 - estIter->y() + realState.y() + v.y() * 2;

        ++i;
        ++estIter;
    }
    filledPolygonRGBA(renderer, vx.data(), vy.data(), vx.size(), 0x4E, 0x9A, 0x06, 0x40);
    
    // Draw flyer
    SDL_SetRenderDrawColor(renderer, 0xfC, 0xAF, 0x3E, 0xFF );
    auto flyer = SDL_Rect{SCREEN_WIDTH - 100 - 2, SCREEN_HEIGHT / 2 - 2, 4, 4};
    SDL_RenderFillRect(renderer, &flyer);
    
    // Draw estimated state
    SDL_SetRenderDrawColor(renderer, 0x8A, 0xE2, 0x34, 0xFF );
    auto flyerEst = SDL_Rect{int(SCREEN_WIDTH - 100 - 2),
                             int(SCREEN_HEIGHT / 2 - stateEst(0) + realState.y() - 2),
                             4, 4};
    SDL_RenderFillRect(renderer, &flyerEst);

    SDL_SetRenderDrawColor(renderer, 0x4E, 0x9A, 0x06, 0xFF );
    auto flyerError1 = SDL_Rect{int(SCREEN_WIDTH - 100 - 2),
                                int(SCREEN_HEIGHT / 2 - stateEst(0) + realState.y() - stateStd * 2 - 2),
                                4, 4};
    SDL_RenderFillRect(renderer, &flyerError1);
    auto flyerError2 = SDL_Rect{int(SCREEN_WIDTH - 100 - 2),
                                int(SCREEN_HEIGHT / 2 - stateEst(0) + realState.y() + stateStd * 2 - 2)
                                , 4, 4};
    SDL_RenderFillRect(renderer, &flyerError2);
    
    SDL_RenderPresent(renderer);
    SDL_UpdateWindowSurface(window);

    SDL_Delay(20);
  }
  
  SDL_DestroyWindow(window);

  SDL_Quit();

  return 0;
}
