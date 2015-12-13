#include "hvmc_physics.h"


void RigidBody::IntegrateForces(f32 dt)
{
    if (m != 0){
        this->velocity += dt * this->im * this->forces;
        this->angularVelocity += dt * this->torque ;
    }

}

void RigidBody::IntegrateVelocities(f32 dt)
{

   if(m !=0 && im != 0){
        this->position += dt * this->velocity;
        this->rotation += dt * this->angularVelocity;
   }
}

void RigidBody::ApplyForce( vec2 const& f )
{
    this->forces += f;

}

void RigidBody::ApplyImpulse( vec2 const& impulse, vec2 const& contactVector )
{
    velocity += im * impulse;
    angularVelocity += iI * Cross( contactVector, impulse );
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
    body->m = 1.0f;
    body->iI = 1.0f;
    body->I = 1.0f;
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
    body->m = 1.0f;
    body->iI = 1.0f;
    body->I = 1.0f;
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

    body->m = 0;
    body->iI = 0;
    body->I = 0;
    body->im = 0.f;
    body->position = pos;
    body->angularVelocity=0.0f;
    body->velocity = {0.0f,0.0f};

    body->collider.type = RIGID_BODY_BOX;
    body->collider.dims = dims;

    rigidBodies.push_back( body );
    return body;
}

void PhysicsSystem::Update( f32 dt )
{    
    // Add gravity
    for ( auto& rb : rigidBodies ) rb->ApplyForce( rb->m * gravity);


    std::vector<CollisionInfo> collisions;
    u32 count = rigidBodies.size();
    // Generate contact infos
    // NOTE: This double loop prevents testing
    //an object against another multiple times

    for ( u32 i = 0; i < count - 1; ++i )
    {
        for ( u32 j = i + 1; j < count; ++j )
        {
            RigidBody* a = rigidBodies[i];
            RigidBody* b = rigidBodies[j];
            CollisionInfo info;
            // Test collisions, add to list if colliding
            if ( collide( a, b, info ) )
            {
                collisions.push_back( info );
            }
        }
    }
    // Integrate forces
    for ( auto& rb : rigidBodies ) rb->IntegrateForces( dt );

    for ( auto const& collision : collisions ) collision.Solve();


    for ( auto& rb : rigidBodies ) rb->IntegrateVelocities( dt );

    for (auto const& collision : collisions) collision.CorrectPositions();

    // Clear forces
    for ( auto& rb : rigidBodies ) {
        rb->forces = vec2{ 0.f, 0.f };
        rb->torque = 0.f;
    }
}

