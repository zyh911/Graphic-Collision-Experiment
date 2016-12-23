/*
 * @file rigid_driver_plugin_motion.h 
 * @Customize the motion of objects
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

#ifndef PHYSIKA_DYNAMICS_RIGID_BODY_RIGID_DRIVER_PLUGIN_MOTION_H_
#define PHYSIKA_DYNAMICS_RIGID_BODY_RIGID_DRIVER_PLUGIN_MOTION_H_

#include <map>
#include <queue>

namespace Physika{

template <typename Scalar>
class RigidPluginMotionCustomization
{
public:
    RigidPluginMotionCustomization();
    ~RigidPluginMotionCustomization();

    void setRigidBody(RigidBody<Scalar, 3>* rigid_body);
    void setConstantTranslation(const Vector<Scalar, 3>& velocity); // translation move with constant velocity
    void setConstantRotation(const Vector<Scalar, 3>& velocity); // spin with constant angular velocity
    void setPeriodTranslation(const Vector<Scalar, 3>& velocity, Scalar period); // back and force
    void setPeriodRotation(const Vector<Scalar, 3>& velocity, Scalar period); // rotate clockwise and counterclockwise
	void setConstantTranslationAcceleration(const Vector<Scalar, 3> &acceleration); // translation move with constant acceleration
	void setConstantRotationAcceleration(const Vector<Scalar, 3> &acceleration); // spin with constant angular acceleration

    void update(Scalar dt, Scalar current_time);

protected:
    RigidBody<Scalar, 3>* rigid_body_;
    Vector<Scalar, 3> constant_translation_velocity_;
    Vector<Scalar, 3> constant_rotation_velocity_;
    Vector<Scalar, 3> period_translation_velocity_;
    Vector<Scalar, 3> period_rotation_velocity_;
	Vector<Scalar, 3> constant_translation_acceleration_;
	Vector<Scalar, 3> constant_rotation_acceleration_;
    Scalar translation_period_;
    Scalar rotation_period_;

};

template <typename Scalar>
class RigidBodyTimerTask
{
public:

	enum TimerTaskType{
		ADD_CONSTANT_TRANSLATION = 1,
		ADD_CONSTANT_ROTATION = 2,
		ADD_CONSTANT_TRANSLATION_ACCELERATION = 3,
		ADD_CONSTANT_ROTATION_ACCELERATION = 4,
		SET_FREE = 5}; // SET_FREE means after this task, the rigid body will be handled and updated by driver, instead of here

	RigidBodyTimerTask();
	RigidBodyTimerTask(RigidBody<Scalar, 3> *rigid_body, Scalar happen_time, TimerTaskType task_type, const Vector<Scalar, 3> &tempValue, unsigned int priority = 0);
	~RigidBodyTimerTask();

	void setHappenTime(Scalar happen_time);
	void setRigidBody(RigidBody<Scalar, 3>* rigid_body);
	void setValue(const Vector<Scalar, 3> &tempValue); // it is lucky that all tasks need values of Vector3d
	void setType(TimerTaskType task_type);
	void setPriority(unsigned int priority);

	RigidBody<Scalar, 3> *rigidBody() const;
	Scalar happenTime() const;
	Vector<Scalar, 3> getValue() const;
	TimerTaskType taskType() const;

	bool operator < (const RigidBodyTimerTask<Scalar> &another) const;

private:
	Scalar happen_time_;
	Vector<Scalar, 3> value_;
	RigidBody<Scalar, 3> *rigid_body_;
	TimerTaskType type_;
	unsigned int priority_; // only be useful if two tasks happen at the same time
};

template <typename Scalar>
class RigidDriverPluginMotion: public RigidDriverPlugin<Scalar, 3>
{
public:
    RigidDriverPluginMotion();
    ~RigidDriverPluginMotion();

    //functions called in driver
    void onBeginFrame(unsigned int frame);
    void onEndFrame(unsigned int frame);
    void onBeginTimeStep(Scalar time, Scalar dt);
    void onEndTimeStep(Scalar time, Scalar dt);

    void onBeginRigidStep(unsigned int step, Scalar dt);//replace the original onBeginTimeStep in rigid body simulation
    void onEndRigidStep(unsigned int step, Scalar dt);//replace the original onEndTimeStep in rigid body simulation

    void onAddRigidBody(RigidBody<Scalar, 3>* rigid_body);
    void onBeginCollisionDetection();
    void onEndCollisionDetection();

    void setConstantTranslation(RigidBody<Scalar, 3>* rigid_body, const Vector<Scalar, 3>& velocity);//WARNING! rigid_body will be set fixed after calling this function
    void setConstantRotation(RigidBody<Scalar, 3>* rigid_body, const Vector<Scalar, 3>& velocity);//WARNING! rigid_body will be set fixed after calling this function
    void setPeriodTranslation(RigidBody<Scalar, 3>* rigid_body, const Vector<Scalar, 3>& velocity, Scalar period);//WARNING! rigid_body will be set fixed after calling this function
    void setPeriodRotation(RigidBody<Scalar, 3>* rigid_body, const Vector<Scalar, 3>& velocity, Scalar period);//WARNING! rigid_body will be set fixed after calling this function
	void setConstantTranslationAcceleration(RigidBody<Scalar, 3> *rigid_body, const Vector<Scalar, 3> &acceleration);
	void setConstantRotationAcceleration(RigidBody<Scalar, 3> *rigid_body, const Vector<Scalar, 3> &acceleration);

	void addTimerTask(RigidBody<Scalar, 3> *rigid_body, Scalar happen_time, typename RigidBodyTimerTask<Scalar>::TimerTaskType task_type, const Vector<Scalar, 3> &tempValue, unsigned int priority = 0);

protected:
    Scalar time_;
    std::map<RigidBody<Scalar, 3>*, RigidPluginMotionCustomization<Scalar> > customized_motions_;
	
	std::priority_queue<RigidBodyTimerTask<Scalar> > tasks_;

};

} //end of namespace Physika

#endif //PHYSIKA_DYNAMICS_RIGID_BODY_RIGID_DRIVER_PLUGIN_MOTION_H_
