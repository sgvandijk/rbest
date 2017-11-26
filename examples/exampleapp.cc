#include "exampleapp.hh"

#include <stdexcept>

namespace rbest
{
  ExampleApp::ExampleApp()
    : mWindow{nullptr},
      mRenderer{nullptr},
      mScreenWidth{0},
      mScreenHeight{0},
      mQuit{false},
      mPaused{false}
  {}
  
  void ExampleApp::init(int screenWidth, int screenHeight, std::string const& title)
  {
    mScreenWidth = screenWidth;
    mScreenHeight = screenHeight;
    
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
      throw std::runtime_error{std::string{"SDL could not initialize! Error: "} + SDL_GetError()};

    mWindow = SDL_CreateWindow(title.c_str(),
                               SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                               screenWidth, screenHeight,
                               SDL_WINDOW_SHOWN );

    if (mWindow == nullptr)
      throw std::runtime_error{std::string{"Window could not be created! Error: "} + SDL_GetError()};

    mRenderer = SDL_CreateRenderer(mWindow, -1, SDL_RENDERER_ACCELERATED);
    if (mRenderer == nullptr)
      throw std::runtime_error{std::string{"Renderer coul dnot be created! Error: "} + SDL_GetError()};

    // Let app initialise its things
    initSystem();
    initFilter();
  }

  void ExampleApp::run()
  {
    auto event = SDL_Event{};
    while (!mQuit)
    {
      onStepStart();
      while (SDL_PollEvent(&event) != 0)
      {
        // Handle quit events ourselves
        switch (event.type)
        {
        case SDL_QUIT:
          quit();
          break;
        case SDL_KEYDOWN:
          switch (event.key.keysym.sym)
          {
          case SDLK_q:
            quit();
            break;
          case SDLK_p:
            mPaused = !mPaused;
            break;
          }
          break;
        }

        // Let app handle event, too
        handleEvent(event);

      }

      if (mPaused)
      {
        SDL_Delay(10);
        continue;
      }
      
      // Run app stages
      updateState();
      updateFilter();
      render();

      // Finish loop
      SDL_RenderPresent(mRenderer);
      SDL_UpdateWindowSurface(mWindow);
      
      SDL_Delay(20);
    }
  }

  void ExampleApp::quit()
  {
    mQuit = true;
  }
}
