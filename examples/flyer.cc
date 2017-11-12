#include "Filter/kalmanfilter.hh"

#include <SDL.h>
#include <iostream>
#include <cmath>
#include <list>

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
  
  auto quit = false;
  auto event = SDL_Event{};

  // State consists of height and vertical velocity
  auto realState = Eigen::Vector3d{0.0, 0.0, 0.0};

  auto stateHistory = std::list<Eigen::Vector3d>{};
  
  // Flyer moves horizontally at constant speed, which is not filtered
  auto xSpeed = 5.0;

  
  auto gridSize = 100;
  
  while (!quit)
  {
    auto yAccel = 0.;
    
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
          yAccel = 0.05;
          break;
        case SDLK_DOWN:
          yAccel = -0.05;
        }
        break;
      }      
    }

    stateHistory.push_back(realState);
    while (stateHistory.size() > SCREEN_WIDTH / xSpeed)
      stateHistory.pop_front();
    
    // Update state
    realState.x() += xSpeed;
    realState.y() += realState.z();
    realState.z() += yAccel;

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
    auto iter = stateHistory.begin();
    auto hs1 = *iter++;
    for ( ; iter != stateHistory.end(); ++iter)
    {
      auto hs2 = *iter;
      SDL_RenderDrawLine(renderer,
                         700 + hs1.x() - realState.x(), 300 - hs1.y() + realState.y(),
                         700 + hs2.x() - realState.x(), 300 - hs2.y() + realState.y());
      hs1 = hs2;
    }
    
    // Draw flyer
    SDL_SetRenderDrawColor(renderer, 0x72, 0x9F, 0xCF, 0xFF );
    auto flyer = SDL_Rect{700 - 2, 300 - 2, 4, 4};
    SDL_RenderFillRect(renderer, &flyer);
    

    SDL_RenderPresent(renderer);
    SDL_UpdateWindowSurface(window);

    SDL_Delay(20);
  }
  
  SDL_DestroyWindow(window);

  SDL_Quit();

  return 0;
}
