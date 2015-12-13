#ifndef HVMC_COLLISIONS_H
#define HVMC_COLLISIONS_H

#include "hvmc_math.h"

struct RigidBody;

struct CollisionInfo{
    RigidBody *a; // 2 RigidBody en collision
    RigidBody *b;

    vec2 normal; // la normale au contact
    f32 penetration; // distance de pénétration entre les objets

    vec2 contactPoint; // point de contact

    f32 coefRestitution; // coefficient de restitution (entre 0 et 1)

    void Solve() const;
    void CorrectPositions() const;
};


bool collide(RigidBody* a,RigidBody* b, CollisionInfo &info);
bool CollideSphereSphere(RigidBody* a, RigidBody* b, CollisionInfo& info);
bool collideBoxBox(RigidBody* a, RigidBody* b, CollisionInfo& info);
bool collideBoxSphere(RigidBody* b, RigidBody* a, CollisionInfo& info);

#endif

