#ifndef HVMC_GRAPHICS_H
#define HVMC_GRAPHICS_H

#include <vector>
#include <SDL2/SDL.h>

#include "hvmc_math.h"

struct SDL_Texture;
struct SDL_Renderer;

struct GraphicsComponent
{
    SDL_Rect* rect = nullptr;
    SDL_Texture* texture = nullptr;

    f32 rotation = 0.f;
};

extern void GraphicsComponentRender( GraphicsComponent* component, SDL_Renderer* renderer );
extern void GraphicsComponentCleanup( GraphicsComponent* component );

struct GraphicsSystem
{
    bool Init( SDL_Renderer* renderer );
    void Cleanup();

    void Render();
    
    GraphicsComponent* AddSphere( vec2 const& pos, f32 radius );
    GraphicsComponent* AddBox( vec2 const& pos, vec2 const& dim );
    GraphicsComponent* AddWall( vec2 const& pos, vec2 const& dim );

    SDL_Renderer* renderer;
    std::vector<GraphicsComponent*> components;

private:
    SDL_Texture* wallVertTexture;
    SDL_Texture* wallHoriTexture;
    SDL_Texture* crateTexture;
    SDL_Texture* smileyTexture;

    bool wallVertTextureInit;
    bool wallHoriTextureInit;
    bool crateTextureInit;
    bool smileyTextureInit;
};

#endif
