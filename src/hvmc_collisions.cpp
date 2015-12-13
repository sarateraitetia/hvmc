#include "hvmc_collisions.h"
#include "hvmc_physics.h"
#include <iostream>

bool CollideSphereSphere(RigidBody* a, RigidBody* b, CollisionInfo& info)
{
    vec2 dist = b->position - a->position;
    f32 r = b->collider.radius + a->collider.radius;

    if(LengthSquared(dist) >r*r)
        return false;

    f32 d = Length(dist);

    info.a = a;
    info.b = b;
    // si les deux sphere ne sont pas collées
    if(d!=0)
    {
        info.penetration = r - d;
        info.normal = dist / d;
        info.contactPoint = a->position + a->collider.radius * info.normal;

        return true;
    }
    else // sinon elle sont collées
    {
        info.penetration = a->collider.radius;
        vec2 n; n.x = 1; n.y=0;
        info.normal = n;
        return true;
    }
}

bool CollideBoxBox(RigidBody* a, RigidBody* b, CollisionInfo& info)
{

    vec2 D = Abs( b->position-a->position ) - (a->collider.dims/2+ b->collider.dims/2);
    // si il y a collision

    if (D.x < 0 && D.y < 0)
    {


        info.a = a;
        info.b = b;

        // approximation ?
        if (D.x > D.y){
            if(b->position.x > a->position.x){
                info.normal = {1.0f,0.0f};

            }
            else{
                info.normal = {-1.0f,0.0f};
            }
            info.penetration = -D.x;
        }else{
            if(b->position.y > a->position.y){
                info.normal = {0.0f,1.0f};

            }
            else{
                info.normal = {0.0f,-1.0f};
            }
            info.penetration = -D.y;

        }
        return true;
    }


    // SAT IMPLEMENTATION //prblème de positions T_T
    //    vec2 dist = b->position - a->position;

    //    f32 a_ext = a->collider.dims.x /2 ;
    //    f32 b_ext = b->collider.dims.x/2;

    //    f32 x_overlap = a_ext + b_ext - abs(dist.x);

    //    info.a = a;
    //    info.b = b;
    //    if(x_overlap > 0)// collision en x
    //    {

    //        f32 aa_ext = a->collider.dims.y/2;
    //        f32 bb_ext = b->collider.dims.y/2;

    //        f32 y_overlap = aa_ext + bb_ext - abs(dist.y);

    //        if(y_overlap > 0)// collision en y
    //        {
    //            if(x_overlap > y_overlap)
    //            {
    //                if(dist.x < 0)//b à gauche de a
    //                    info.normal = {-1.0f,0.0f};

    //                else//b à droite de a
    //                    info.normal = {1.0f,0.0f};

    //                info.penetration = x_overlap;
    //            }
    //            else
    //            {
    //                if(dist.y < 0)//b en bas de a
    //                    info.normal = {0.0f,-1.0f};
    //                else//b en haut de a
    //                    info.normal = {0.0f,1.0f};

    //                info.penetration = y_overlap;


    //            }
    //            return true;
    //        }
    //    }

    return false;

}
// b = box / a = sphere
bool CollideBoxSphere(RigidBody* b, RigidBody* a, CollisionInfo& info)
{
    vec2 dist = b->position - a->position ;

    f32 x_ext = b->collider.dims.x /2;
    f32 y_ext = b->collider.dims.y /2;

    vec2 cp = dist; // contact point
    cp.x = clamp(cp.x,-x_ext,x_ext);
    cp.y = clamp(cp.y,-y_ext,y_ext);

    info.a = a;
    info.b = b;


    bool inside = false;

    if(dist.x == cp.x && dist.y == cp.y)
    {
        inside = true;

        if(abs(dist.x) > abs(dist.y))

        {
            if(cp.x > 0) // coté droit
                cp.x = x_ext;
            else// coté gauche
                cp.x = -x_ext;
        }
        else
        {
            if(cp.y >0) // haut
                cp.y = y_ext;
            else // bas
                cp.y = -y_ext;
        }

    }

    vec2 normal = dist - cp;
    f32 d = LengthSquared(normal);
    f32 r = a->collider.radius;

    if((d > r*r) && !inside)
        return false;

    d = Length(normal);

    if(inside)
    {
        info.normal = -normal/d;
        info.penetration = r-d;
    }
    else
    {
        info.normal = normal/d;
        info.penetration = r - d;
    }

    return true;

}

bool collide(RigidBody* a,RigidBody* b, CollisionInfo &info)
{
    if ( a->collider.type == RIGID_BODY_BOX )
    {
        if ( b->collider.type == RIGID_BODY_BOX )
            return CollideBoxBox( a, b, info );
        else if ( b->collider.type == RIGID_BODY_SPHERE )
            return CollideBoxSphere( a, b, info );
    }
    else if ( a->collider.type == RIGID_BODY_SPHERE )
    {
        if ( b->collider.type == RIGID_BODY_SPHERE )
            return CollideSphereSphere( a, b, info );
        else if ( b->collider.type == RIGID_BODY_BOX )
            return CollideBoxSphere( b, a, info );
    }
    return false;
}

void CollisionInfo::Solve()const
{


    if(Length(a->velocity) == 0 && Length(b->velocity) == 0)
        return;

    vec2 rA = a->position - this->contactPoint ;
    vec2 rB = b->position - this->contactPoint ;
    vec2 vrel = ( b->velocity) - ( a->velocity );
    f32 vSurNorm = Dot( vrel, normal );

    if(vSurNorm > 0)
        return;

    f32 e = std::min( a->restitution, b->restitution);
    f32 j = (-(1 + e) * vSurNorm )/ (a->im + b->im );
    vec2 impulsion = j * normal;


    a->ApplyImpulse(-impulsion,contactPoint);
    b->ApplyImpulse(impulsion,contactPoint);
}
void CollisionInfo::CorrectPositions()const
{

    if(a->im == 0 && b->im == 0)
        return;

    f32 threshold = 0.01f;
    const f32 pourcentage =1.0f;

    vec2 correction = std::max(penetration - threshold,0.0f) / (a->im + b->im) * pourcentage * normal;
    a->position = a->position -  (a->im * correction);
    b->position += b->im * correction;

}
