#include "hvmc_physics.h"
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
void RigidBody::IntegradeForces(f32 dt)
{
    vec2 a = this->im * this->forces;

    this->velocity += a * dt;
    //this->angularVelocity += rb->torque * dt; // 2ème méthode
}

void RigidBody::IntegradeVelocities(f32 dt)
{
    this->position += this->velocity * dt;
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
    body->iI = 1.f;
    body->m = 1.f;
    body->I = 1.f;
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
    for(auto & rb: rigidBodies)
    {
        rb->forces += rb->m * gravity;
        rb->ApplyForce(rb->m * gravity);

        rb->IntegradeForces(dt);

        rb->IntegradeVelocities(dt);
    }

    // Clear forces
    for(auto & rb: rigidBodies)
    {
        rb->forces = {0.0, 0.0};
        rb->torque = {0.0};
    }

    // Détection de collision
    for(auto & rb1: rigidBodies)
    {
        for(auto & rb2: rigidBodies)
        {
            vec2 tmp = (rb1->position - rb2->position);
            f32 diff_centre = tmp{0} + tmp{1};
            f32 diff_rayon = (rb1->collider.radius + rb2->collider.radius) * (rb1->collider.radius + rb2->collider.radius);

            if(diff_rayon > diff_centre)
                std::cerr << "il y a eu collision" << std::endl;
        }
    }

}

