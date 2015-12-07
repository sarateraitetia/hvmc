#include "hvmc_collisions.h"
#include "hvmc_math.h"
#include <iostream>
#include "hvmc_physics.h"



struct RigidBody;

// ---------------------------------------------------------------------------- SARA
bool collideSphereSphere(RigidBody* a, RigidBody* b, CollisionInfo& info)
{
    vec2 vecAB = (b->position - a->position);
    f32 diff_centre = LengthSquared(b->position - a->position);
    f32 diff_rayon = (a->collider.radius + b->collider.radius) * (a->collider.radius + b->collider.radius);


    if(diff_rayon >= diff_centre)
    {
//        std::cerr << "il y a eu collision cercle-cercle" << std::endl;

        // 1) On fige les RigidBody
        //a->SetKinematic();
        //b->SetKinematic();

        // 2) On crée notre CollisionInfo
        info.a = a;
        info.b = b;
        info.normal =  vecAB / Length(vecAB);

        info.penetration = b->collider.radius + a->collider.radius - Length(vecAB);

        info.contactPoint = a->position + a->collider.radius * info.normal;


//        f32 p = Length(vecAB)-(a->collider.radius + b->collider.radius);
//        info.penetration = p * info.normal;
        return true;
    }

    return false;
}

bool collideBoxBox(RigidBody* a, RigidBody* b, CollisionInfo& info){


        vec2 D = Abs( b->position-a->position ) - (a->collider.dims/2+ b->collider.dims/2);
        // si il y a collision
        if (D.x < 0 && D.y < 0)
        {
            //std::cerr << "il y a eu collision box-box" << std::endl;

            // 1) On fige les RigidBody
//            a->SetKinematic();
//            b->SetKinematic();

            // 2) On crée notre CollisionInfo
            info.a = a;
            info.b = b;
            info.contactPoint = a->position + (b->position - a->position)/2;
            // approximation ?
            if (D.x < D.y){
                if(b->position.x > a->position.x){
                    info.normal = {1.0f,0.0f};

                }
                else{
                    info.normal = {-1.0f,0.0f};
                }
                info.penetration = D.x;
            }else{
                if(b->position.y > a->position.y){
                    info.normal = {0.0f,1.0f};

                }
                else{
                    info.normal = {0.0f,-1.0f};
                }
                info.penetration = D.y;

            }
//            info.normal = {0.0f,1.0f};
            std::cout << "info.normal : "  <<info.normal.x << " "<<info.normal.y << std::endl;
            return true;
        }

        return false;
}

// a = box et b = circle
bool collideBoxSphere(RigidBody* a, RigidBody* b, CollisionInfo& info){
 vec2 p;
 f32 xextend = a->collider.dims.x/2;

 // demi longueur
 f32 yextend = a->collider.dims.y/2;
 // demi hauteur
 vec2 ab = b->position - a->position;
 p.x = clamp(a->position.x+ab.x,a->position.x-xextend, a->position.x+xextend);
 p.y = clamp(a->position.y+ab.y,a->position.y-yextend, a->position.y+yextend);
 if(LengthSquared(p-b->position) < (b->collider.radius*b->collider.radius)){
     //std::cerr << "il y a eu collision box-cercle" << LengthSquared(p-a->position) << "<" << (b->collider.radius*b->collider.radius) <<std::endl;

    info.a = a;
    info.b = b;
//    info.penetration =  p.y-b->position.y;
//    info.contactPoint = p;

//    info.normal = b->position - a->position;
//    info.normal = info.normal / Length(info.normal);

     // 1) On fige les RigidBody
     a->SetKinematic();
     b->SetKinematic();

     return true;
 }

 return false;
}


bool collide( RigidBody* a, RigidBody* b, CollisionInfo& info ) {

    if ( a->collider.type == RIGID_BODY_BOX )
    {
        if ( b->collider.type == RIGID_BODY_BOX )
            return collideBoxBox( a, b, info );
        else if ( b->collider.type == RIGID_BODY_SPHERE )
            return collideBoxSphere( a, b, info );
    }
    else if ( a->collider.type == RIGID_BODY_SPHERE )
    {
        if ( b->collider.type == RIGID_BODY_SPHERE )
            return collideSphereSphere( a, b, info );
        else if ( b->collider.type == RIGID_BODY_BOX )
            return collideBoxSphere( b, a, info );
    }
    // Should not get there
    return false;
}



void CollisionInfo::Solve() const // à finir !!
{

//    if(a->collider.type == RIGID_BODY_SPHERE && b->collider.type == RIGID_BODY_SPHERE)
//    {
        // prendre le min des coefficients de restitution
        // e est entre 0(énergie mal restituée) et 1(bonne restitution)
        f32 e = 1;  // e = min(a,b)

        // résolution des collisions
        vec2 rA =  a->position - contactPoint ;
        vec2 rB =  b->position - contactPoint ;

        vec2 vRel = (b->velocity + Cross(b->angularVelocity, rB)) - (a->velocity + Cross(a->angularVelocity, rA));

        f32 J = (-(1+e) * Dot(vRel, normal)) / (a->im+b->im+a->iI*Cross(rA, normal)+b->iI*Cross(rB, normal));

        if(Dot(vRel,normal ) < 0)
        {
            // impulsion
            vec2 jA =-J* normal;
            a->velocity = a->velocity + (jA * a->im);
            a->angularVelocity = a->angularVelocity + Cross(rA/*erreur ici*/, (jA * a->iI));

            vec2 jB = J* normal;
            b->velocity = b->velocity + (jB * b->im);
            std::cerr << "normal : "<< normal.x << " "<<normal.y << std::endl;
            b->angularVelocity = b->angularVelocity + Cross(rB, (jB * b->iI));
        }
//    }
}

void CollisionInfo::CorrectPositions() const
{
    if(penetration > 0.01f) // ne pas résoudre si penetration < à un seuil (entre 0.1 et 0.01)
    {
    const f32 threshold = 0.01f;
    const f32 percentage = 0.5f; // entre 20 et 80%

    vec2 correction = (std::max(penetration - threshold, 0.0f) / (a->im + b->im)) * percentage * normal;

    a->position = a->position - (a->im * correction);
    b->position = b->position + (b->im * correction);
    }
}

// ----------------------------------------------------------------------------------
