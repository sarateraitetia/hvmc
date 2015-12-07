#include "hvmc_physics.h"
#include "hvmc_collisions.h"
#include <iostream>

void RigidBody::Update( f32 dt )
{

}

void RigidBody::ApplyForce( vec2 const& f )
{
    // mg

    // calcul de l'accélération
    //vec2 a = (1/m) * forces;
    //this->velocity = this->velocity + dt * a;
    //this->position = this->position + (dt * this->velocity);

}

// ----------------- SARA
void RigidBody::IntegrateForces(f32 dt)
{
    vec2 a = this->im * this->forces;

    this->velocity += a * dt;
    //this->angularVelocity += rb->torque * dt; // 2ème méthode
}

void RigidBody::IntegrateVelocities(f32 dt)
{
    if(I!=0.f && m!=0.f)
    {
        this->position += this->velocity * dt;
    }
    //this->rotation += this->w * dt
}

// ----------------


void RigidBody::ApplyImpulse( vec2 const& impulse, vec2 const& contactVector )
{
}

void RigidBody::SetKinematic()
{
    I = iI = m = im = 0.f;
}

bool PhysicsSystem::Init()
{
    gravity = vec2{ 0.f, -9.81f };

    return true;
}

void PhysicsSystem::Cleanup()
{
    rigidBodies.clear();

}

RigidBody* PhysicsSystem::AddSphere( vec2 const& pos, f32 radius )
{
    RigidBody* body = new RigidBody;

    body->forces = { 0.f, 0.f };
    body->im = 1.f; // 1 kg
    body->iI = 1.f;
    // ----------------- SARA
    body->m = 1.f;
    body->I = 1.f;
    // ----------------
    body->position = pos;
    body->velocity = { 0.f, 0.f };
    //    body->angularVelocity = 1.0f;
    body->collider.type = RIGID_BODY_SPHERE;
    body->collider.radius = radius;

    rigidBodies.push_back( body );
    return body;
}

RigidBody* PhysicsSystem::AddBox( vec2 const& pos, vec2 const& dims )
{
    RigidBody* body = new RigidBody;

    body->forces = { 0.f, 0.f };
    body->im = 1.f; // 1 kg

    // ----------------- SARA
    body->iI = 1.f;
    body->m = 1.f;
    body->I = 1.f;
    // ----------------

    body->position = pos;
    body->velocity = { 0.f, 0.f };

    body->collider.type = RIGID_BODY_BOX;
    body->collider.dims = dims;

    rigidBodies.push_back( body );
    return body;
}

RigidBody* PhysicsSystem::AddWall( vec2 const& pos, vec2 const& dims )
{
    RigidBody* body = new RigidBody;

    body->im = 0.f;

    // ----------------- SARA
    //    body->iI = 1.f;
    //    body->m = 0.f;
    //    body->I = 1.f;
    // ----------------

    body->position = pos;

    body->collider.type = RIGID_BODY_BOX;
    body->collider.dims = dims;

    rigidBodies.push_back( body );
    return body;
}


void PhysicsSystem::Update( f32 dt )
{

    // Add Gravity
    //    std::cerr << dt << std::endl;
    for(auto & rb: rigidBodies)
    {
        rb->forces += rb->m * gravity;
        //rb->ApplyForce(rb->m * gravity); // 2ème méthode

        //rb->IntegrateForces(dt);

        //rb->IntegrateVelocities(dt);
    }

    // Détection de collision entre les objets
    std::vector<CollisionInfo> collisions;
    u32 count = rigidBodies.size();
    for(u32 i=0; i<count-1; ++i)
    {
        for(u32 j=i+1; j<count; ++j)
        {
            CollisionInfo info;
            RigidBody* a = rigidBodies[i];
            RigidBody* b = rigidBodies[j];

            if(collide(a, b, info))
                collisions.push_back(info);
        }
    }

    // Integrate forces
    for(auto& rb : rigidBodies)
        rb->IntegrateForces(dt);

    // Solve contacts (appliquer impulsions)
    for(auto const& collision: collisions)
        collision.Solve();

    // Integrate velocities
    for(auto& rb : rigidBodies)
        rb->IntegrateVelocities(dt);

//    for(auto const & collision : collisions )
//        collision.CorrectPositions();
    // Clear forces
    for(auto & rb: rigidBodies)
    {
        rb->forces = {0.f, 0.f};
        rb->torque = {0.f};
    }




    // Passe de correction de position par rapport à la normale au contact
}





