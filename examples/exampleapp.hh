#pragma once

#include <SDL.h>
#include <SDL2_gfxPrimitives.h>
#include <string>

namespace rbest
{
  class ExampleApp
  {
  public:
    ExampleApp();
    
    void init(int screenWidth, int screenHeight, std::string const& title);
    void run();
    void quit();
    
  protected:
    virtual void initSystem() {};
    virtual void initFilter() {};

    virtual void onStepStart() {};
    virtual void handleEvent(SDL_Event const& event) {};
    virtual void updateState() {};
    virtual void updateFilter() {};
    virtual void render() {};
    
    SDL_Window* mWindow;
    SDL_Renderer* mRenderer;

    int mScreenWidth;
    int mScreenHeight;
    
  private:
    bool mQuit;
    bool mPaused;
  };
}
