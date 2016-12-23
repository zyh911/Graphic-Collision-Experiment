/*
 * @file rigid_body_3d.h 
 * @3D rigid_body class.
 * @author Tianxiang Zhang
 * 
 * This file is part of Physika, a versatile physics simulation library.
 * Copyright (C) 2013 Physika Group.
 *
 * This Source Code Form is subject to the terms of the GNU General Public License v2.0. 
 * If a copy of the GPL was not distributed with this file, you can obtain one at:
 * http://www.gnu.org/licenses/gpl-2.0.html
 *
 */

#ifndef PHYSIKA_DYNAMICS_RIGID_BODY_RIGID_BODY_3D_H_
#define PHYSIKA_DYNAMICS_RIGID_BODY_RIGID_BODY_3D_H_

#include "Physika_Dynamics/Rigid_Body/rigid_body.h"
#include "Physika_Core/Transform/transform_3d.h"
#include "Physika_Dynamics/Rigid_Body/inertia_tensor.h"
#include "Physika_Dynamics/Collidable_Objects/mesh_based_collidable_object.h"

namespace Physika{

template <typename Scalar,int Dim> class Vector;
template <typename Scalar> class SurfaceMesh;

//We strongly recommend using copy construction function to construct a rigid body from another one with the same mesh, scale and density because inertia tensor will not be recalculated.
template <typename Scalar>
class RigidBody<Scalar, 3>
{
public:
    //constructors && deconstructors
    RigidBody();
    RigidBody(SurfaceMesh<Scalar>* mesh, Scalar density = 1);
    RigidBody(SurfaceMesh<Scalar>* mesh, const Transform<Scalar, 3>& transform, Scalar density = 1);
    RigidBody(const RigidBody<Scalar, 3>& rigid_body);
    virtual ~RigidBody();

    //get & set
    void copy(const RigidBody<Scalar, 3>& rigid_body);//Using this function for construction is strongly recommended because inertia tensor will not be recalculated for the same mesh.
    inline typename CollidableObjectInternal::ObjectType objectType() const {return object_type_;};
    inline SurfaceMesh<Scalar>* mesh() {return mesh_;};
    inline const Transform<Scalar, 3>& transform() const {return transform_;};
    inline Transform<Scalar, 3>& transform() {return transform_;};
    inline const Transform<Scalar, 3>* transformPtr() const {return &transform_;};
    inline Transform<Scalar, 3>* transformPtr() {return &transform_;};//WARNING! Don't use this to modify the transform of this rigid body. Use setTranslate(), setRotate() and setScale() instead.
    void setTranslation(const Vector<Scalar, 3>& translation);
    void setRotation(const Vector<Scalar, 3>& rotation);
    void setRotation(const Quaternion<Scalar>& rotation);
    void setRotation(const SquareMatrix<Scalar, 3>& rotation);
    void setScale(const Vector<Scalar, 3>& scale);
    void setProperty(SurfaceMesh<Scalar>* mesh, Scalar density = 1);//Inertia tensor will be recalculated
    void setProperty(SurfaceMesh<Scalar>* mesh, const Transform<Scalar, 3>& transform, Scalar density = 1);//Inertia tensor will be recalculated
    inline void setFixed(bool is_fixed) {is_fixed_ = is_fixed;};
	inline void setFlat(bool is_flat) {is_flat_ = is_flat;};
    inline bool isFixed() const {return is_fixed_;};
    inline void setCoeffRestitution(Scalar coeff_restitution) {coeff_restitution_ = coeff_restitution;};
    inline Scalar coeffRestitution() {return coeff_restitution_;};
    inline void setCoeffFriction(Scalar coeff_friction) {coeff_friction_ = coeff_friction;};
    inline Scalar coeffFriction() {return coeff_friction_;};
    inline const SquareMatrix<Scalar, 3> spatialInertiaTensor() const {return inertia_tensor_.spatialInertiaTensor();};
    inline const SquareMatrix<Scalar, 3> bodyInertiaTensor() const {return inertia_tensor_.bodyInertiaTensor();};
    inline const SquareMatrix<Scalar, 3> spatialInertiaTensorInverse() const {return inertia_tensor_.spatialInertiaTensorInverse();};
    inline const SquareMatrix<Scalar, 3> bodyInertiaTensorInverse() const {return inertia_tensor_.bodyInertiaTensorInverse();};
    inline Scalar density() const {return density_;};
    inline Scalar mass() const {return mass_;};
    inline Vector<Scalar, 3> localMassCenter() const {return local_mass_center_;};
    inline Vector<Scalar, 3> globalMassCenter() const {return transform_.rotation().rotate(local_mass_center_) + transform_.translation();};//Can't use transform_.transform() here because its scales local_mass_center_ unexpectedly
    inline Vector<Scalar, 3> globalTranslation() const {return global_translation_;};
    inline Quaternion<Scalar> globalRotation() const {return global_rotation_;};
    inline Vector<Scalar, 3> globalTranslationVelocity() const {return global_translation_velocity_;};
    inline Vector<Scalar, 3> globalAngularVelocity() const {return global_angular_velocity_;};
    inline void setGlobalTranslationVelocity(const Vector<Scalar, 3>& velocity) {global_translation_velocity_ = velocity;};
    inline void setGlobalAngularVelocity(const Vector<Scalar, 3>& velocity) {global_angular_velocity_ = velocity;};

    //dynamics
    void update(Scalar dt, bool is_force_update = false);//update its configuration and velocity. If is_force_update is true, velocity and configuration will be changed whether this body is fixed or not
    void addImpulse(Scalar magnitude, const Vector<Scalar, 3>& direction, const Vector<Scalar, 3>& global_position);//accumulate collision impulse to the rigid body. This will not change its velocity until velocityIntegral has been called
    void addTranslationImpulse(const Vector<Scalar, 3>& impulse);//This will not change its velocity until velocityIntegral has been called
    void addAngularImpulse(const Vector<Scalar, 3>& impulse);//This will not change its velocity until velocityIntegral has been called
    void performGravity(Scalar gravity, Scalar dt);//Attention! This will change its velocity

    Vector<Scalar, 3> globalVertexPosition(unsigned int vertex_idnex) const;//get the position of a vertex in global frame
    Vector<Scalar, 3> globalVertexVelocity(unsigned int vertex_index) const;//get the velocity of a vertex in global frame
    Vector<Scalar, 3> globalPointVelocity(const Vector<Scalar, 3>& global_point_position) const;//get the velocity of an arbitrary point on/inside the rigid body in global frame

protected:
    //basic properties of a rigid body

    //can be set by public functions
    typename CollidableObjectInternal::ObjectType object_type_;
    SurfaceMesh<Scalar>* mesh_;
    Transform<Scalar, 3> transform_;
    Scalar density_;
    bool is_fixed_;//if fixed is true, this body will be treated as having infinite mass and will not affected by gravity. Generally speaking, fixed body doesn't move. But by calling update with is_force_update = true you can still change its motion status
    bool is_flat_;
	Scalar coeff_restitution_;
    Scalar coeff_friction_;

    //obtained by internal computation
    InertiaTensor<Scalar> inertia_tensor_;
    Scalar mass_;
    Vector<Scalar, 3> local_mass_center_;//position of mass center in local frame (mesh frame)

    //configuration
    Vector<Scalar, 3> global_translation_;//translation of mass center in global frame (inertia frame). Different from translation in transform_ (which is the translation of local frame in global frame) 
    Quaternion<Scalar> global_rotation_;//rotation of rigid body in global frame (inertia frame). Same with rotation in transform_
    //velocity
    Vector<Scalar, 3> global_translation_velocity_;//velocity of mass center in global frame (inertia frame)
    Vector<Scalar, 3> global_angular_velocity_;//angular velocity of rigid body in global frame (inertia frame)
    //impulse
    Vector<Scalar, 3> global_translation_impulse_;//translation impulse accumulated during the collision. This will be used to update global_translation_velocity_
    Vector<Scalar, 3> global_angular_impulse_;///rotation impulse accumulated during the collision. This will be used to update global_angular_velocity_

    //Internal functions

    //dynamics
    void resetTemporaryVariables();//prepare for the new time step
    void velocityIntegral(Scalar dt, bool is_force_update);//if is_force_update is true, velocity will be changed whether this body is fixed or not
    void configurationIntegral(Scalar dt, bool is_force_update);//if is_force_update is true, configuration will be changed whether this body is fixed or not
    void updateInertiaTensor();
    void recalculateTransform();//recalculate transform_ from global_translation_ and global_rotation_
    void recalculatePosition();//recalculate global_translation_ and global_rotation_ from transform_

    //set
    void setMesh(SurfaceMesh<Scalar>* mesh);
    void setTransform(Transform<Scalar, 3>& transform);

};

} //end of namespace Physika

#endif //PHYSIKA_DYNAMICS_RIGID_BODY_RIGID_BODY_3D_H_