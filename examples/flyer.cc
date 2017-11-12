#include "Filter/kalmanfilter.hh"

#include <SDL.h>
#include <iostream>
#include <cmath>
#include <list>
#include <random>

constexpr int SCREEN_WIDTH = 800;
constexpr int SCREEN_HEIGHT = 600;

int main(int argc, char const** argv)
{
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << "\n";
    return -1;
  }
  
  auto window = SDL_CreateWindow("RBEst - Flyer",
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
        case SDLK_UP:
          control = 1.;
          break;
        case SDLK_DOWN:
          control = -1.;
        }
        break;
      }      
    }

    auto yAccel = control * 0.1;

    // Update state
    realState.x() += xSpeed;
    realState.y() += realState.z() + systemNoise();
    realState.z() += yAccel + systemNoise();

    // Run filter
    filter.predict(systemModel, Eigen::Matrix<double, 1, 1>{control});
    filter.update(observationModel, Eigen::Matrix<double, 1, 1>{realState.y() + observationNoise()});
    
    // render

    // Clear screen
    SDL_SetRenderDrawColor(renderer, 0x20, 0x20, 0x20, 0xFF );
    SDL_RenderClear(renderer);

    // Draw grid
    SDL_SetRenderDrawColor(renderer, 0x40, 0x40, 0x40, 0xFF );
    for (auto gx = std::fmod(-realState.x(), gridSize); gx < SCREEN_WIDTH; gx += gridSize)
    {
      SDL_RenderDrawLine(renderer, gx, 0, gx, SCREEN_HEIGHT);
    }

    for (auto gy = std::fmod(realState.y(), gridSize); gy < SCREEN_HEIGHT; gy += gridSize)
    {
      SDL_RenderDrawLine(renderer, 0, gy, SCREEN_WIDTH, gy);
    }

    // Draw history
    SDL_SetRenderDrawColor(renderer, 0x34, 0x65, 0xA4, 0xFF );
    auto siter = stateHistory.begin();
    auto hs1 = *siter++;
    for ( ; siter != stateHistory.end(); ++siter)
    {
      auto hs2 = *siter;
      SDL_RenderDrawLine(renderer,
                         700 + hs1.x() - realState.x(), 300 - hs1.y() + realState.y(),
                         700 + hs2.x() - realState.x(), 300 - hs2.y() + realState.y());
      hs1 = hs2;
    }
    
    SDL_SetRenderDrawColor(renderer, 0x73, 0xD2, 0x16, 0xFF );
    auto eiter = estHistory.begin();
    auto he1 = *eiter++;
    for ( ; eiter != estHistory.end(); ++eiter)
    {
      auto he2 = *eiter;
      SDL_RenderDrawLine(renderer,
                         700 + he1.x() - realState.x(), 300 - he1.y() + realState.y(),
                         700 + he2.x() - realState.x(), 300 - he2.y() + realState.y());
      he1 = he2;
    }
    
    // Draw flyer
    SDL_SetRenderDrawColor(renderer, 0x72, 0x9F, 0xCF, 0xFF );
    auto flyer = SDL_Rect{700 - 2, 300 - 2, 4, 4};
    SDL_RenderFillRect(renderer, &flyer);
    
    // Draw estimated state
    SDL_SetRenderDrawColor(renderer, 0x8A, 0xE2, 0x34, 0xFF );
    auto stateEst = filter.getState();
    auto flyerEst = SDL_Rect{700 - 2, 300 - stateEst(0) + realState.y() - 2, 4, 4};
    SDL_RenderFillRect(renderer, &flyerEst);

    // Store history
    stateHistory.push_back(realState);
    while (stateHistory.size() > SCREEN_WIDTH / xSpeed)
      stateHistory.pop_front();
    estHistory.push_back(Eigen::Vector3d{realState.x(), stateEst(0), stateEst(1)});
    while (estHistory.size() > SCREEN_WIDTH / xSpeed)
      estHistory.pop_front();
    
    SDL_RenderPresent(renderer);
    SDL_UpdateWindowSurface(window);

    SDL_Delay(20);
  }
  
  SDL_DestroyWindow(window);

  SDL_Quit();

  return 0;
}
