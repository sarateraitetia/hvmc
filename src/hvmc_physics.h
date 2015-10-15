#ifndef HVMC_PHYSICS_H
#define HVMC_PHYSICS_H

#include <vector>
#include "hvmc_math.h"
#include "hvmc_collisions.h"

enum RigidBodyType
{
    RIGID_BODY_SPHERE,
    RIGID_BODY_BOX,
};

struct Collider
{
    Collider() {}
    RigidBodyType type;

    union
    {
        f32 radius; // Sphere
        vec2 dims; // Box
    };
};

struct RigidBody
{
    RigidBody() {}
    ~RigidBody() {}
    
    void Update( f32 dt ); // mise à jour de la position
    
    void ApplyForce( vec2 const& force );//
    void ApplyImpulse( vec2 const& impulse, vec2 const& contactVector );// pour les collisions
    
    void SetKinematic(); // utilisé pour les murs, si on veut dire que l'objet a un masse infini. met la Imasse à 0 
    void SetGravityMode( int mode ); // on peut ignorer c'est pour la gravity
    
    // étant donné qu'on aime pas les divisions, on a créer l'inverse de la masse et celle de l'inertie 
    // pour faire en sorte que les objets ne bougent pas, il faut metter une masse d'infini, hors pas bon donc on fait l'inverse => 0
    
    f32 I = 0.f;  // inertia (pareil que pour la masse mais en rotation ex: plus elle est grde, plus il est dur de faire tourner
    f32 iI = 0.f; // inverse inertia
    f32 m = 0.f;  // mass
    f32 im = 0.f; // inverse mass
    
    int gravityMode = 1;

    vec2 forces;
    vec2 position;
    vec2 velocity;

    f32 torque = 0.f;
    f32 rotation = 0.f; // quantité de rotation
    f32 angularVelocity = 0.f; // vitesse de rotation

    Collider collider;
};

struct PhysicsSystem
{
    bool Init(); // met la gravité à 9.8
    void Cleanup();
    
    void Update( f32 dt ); // todo
    
    RigidBody* AddSphere( vec2 const& pos, f32 radius );//ignorer
    RigidBody* AddBox( vec2 const& pos, vec2 const& dims );//ignorer
    RigidBody* AddWall( vec2 const& pos, vec2 const& dims );//ignorer

    std::vector<RigidBody*> rigidBodies;//ignorer
    vec2 gravity;
};

#endif

