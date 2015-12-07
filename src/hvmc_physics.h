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
        f32 radius; // Sphere rayon
        vec2 dims; // Box
    };
};



struct RigidBody
{
    RigidBody() {}
    ~RigidBody() {}

    void Update( f32 dt ); // MAJ de la vitesse et de la position

    void ApplyForce( vec2 const& force ); // A FAIRE !!
    void ApplyImpulse( vec2 const& impulse, vec2 const& contactVector ); // A FAIRE !!
    // pour le collisions (plus tard)

    void SetKinematic(); // pour les murs, mais la masse et l'inverse de
    // la masse à 0
    void SetGravityMode( int mode ); // on s'en fout


    // ------------------------------------------------ SARA
    void IntegrateForces(f32 dt);
    void IntegrateVelocities(f32 dt);
    // -----------------------------------------------------



    f32 I = 0.f;  // inertia -> masse mais pour les rotations
    f32 iI = 0.f; // inverse inertia est précalculé
    // 1) on n'aime pas les divisions en info
    // 2) on veut pas que les objets bougent (= masse de 0)
    f32 m = 0.f;  // mass
    f32 im = 0.f; // inverse mass

    int gravityMode = 1; // objet tombe ou pas
    // on peut l'ignorer en disant que les objets tombent par défaut

    vec2 forces;
    vec2 position;
    vec2 velocity; // vitesse

    f32 torque = 0.f;
    f32 rotation = 0.f;
    f32 angularVelocity = 0.f; // vitesse angulaire

    Collider collider;
};


struct PhysicsSystem // moteur physique
{
    bool Init(); // mets la gravité à 0.9
    void Cleanup();

    void Update( f32 dt ); // A FAIRE !!
    // à appeler sur les rigidBodies

    // Déjà écris - ne pas toucher sauf pour changer la massed 'un objet
    RigidBody* AddSphere( vec2 const& pos, f32 radius );
    RigidBody* AddBox( vec2 const& pos, vec2 const& dims );
    RigidBody* AddWall( vec2 const& pos, vec2 const& dims );

    std::vector<RigidBody*> rigidBodies;
    vec2 gravity;

    // ------------------------------------------ SARA

    // ------------------------------------------------
};

#endif

