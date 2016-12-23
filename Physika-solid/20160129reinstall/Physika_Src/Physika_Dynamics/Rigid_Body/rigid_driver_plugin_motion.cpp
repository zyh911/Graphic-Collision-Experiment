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

#include "Physika_Dynamics/Rigid_Body/rigid_body.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_3d.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_driver.h"
#include "Physika_Dynamics/Rigid_Body/rigid_driver_plugin.h"
#include "Physika_Dynamics/Rigid_Body/rigid_driver_plugin_motion.h"
#include "Physika_Core/Utilities/math_utilities.h"

namespace Physika{


///////////////////////////////////////////////////////////////////////////////////////
//RigidPluginMotionCustomization
///////////////////////////////////////////////////////////////////////////////////////

template <typename Scalar>
RigidPluginMotionCustomization<Scalar>::RigidPluginMotionCustomization():
    rigid_body_(NULL),
    constant_translation_velocity_(0),
    constant_rotation_velocity_(0),
	constant_translation_acceleration_(0),
	constant_rotation_acceleration_(0),
    period_translation_velocity_(0),
    period_rotation_velocity_(0),
    translation_period_(0),
    rotation_period_(0)
{

}

template <typename Scalar>
RigidPluginMotionCustomization<Scalar>::~RigidPluginMotionCustomization()
{

}

template <typename Scalar>
void RigidPluginMotionCustomization<Scalar>::setRigidBody(RigidBody<Scalar, 3>* rigid_body)
{
    rigid_body_ = rigid_body;
}

template <typename Scalar>
void RigidPluginMotionCustomization<Scalar>::setConstantTranslation(const Vector<Scalar, 3>& velocity)
{
    if(rigid_body_ == NULL)
    {
        std::cerr<<"NULL rigid body in motion customization!"<<std::endl;
        return;
    }
    constant_translation_velocity_ = velocity;
}

template <typename Scalar>
void RigidPluginMotionCustomization<Scalar>::setConstantRotation(const Vector<Scalar, 3>& velocity)
{
    if(rigid_body_ == NULL)
    {
        std::cerr<<"NULL rigid body in motion customization!"<<std::endl;
        return;
    }
    constant_rotation_velocity_ = velocity;
}

template <typename Scalar>
void RigidPluginMotionCustomization<Scalar>::setPeriodTranslation(const Vector<Scalar, 3>& velocity, Scalar period)
{
    if(rigid_body_ == NULL)
    {
        std::cerr<<"NULL rigid body in motion customization!"<<std::endl;
        return;
    }
    if(period <= 0)
    {
        std::cerr<<"Period should be positive in motion customization!"<<std::endl;
        return;
    }
    period_translation_velocity_ = velocity;
    translation_period_ = period;
}

template <typename Scalar>
void RigidPluginMotionCustomization<Scalar>::setPeriodRotation(const Vector<Scalar, 3>& velocity, Scalar period)
{
    if(rigid_body_ == NULL)
    {
        std::cerr<<"NULL rigid body in motion customization!"<<std::endl;
        return;
    }
    if(period <= 0)
    {
        std::cerr<<"Period should be positive in motion customization!"<<std::endl;
        return;
    }
    period_rotation_velocity_ = velocity;
    rotation_period_ = period;
}

template <typename Scalar>
void RigidPluginMotionCustomization<Scalar>::setConstantTranslationAcceleration(const Vector<Scalar, 3> &acceleration)
{
	if(rigid_body_ == NULL)
	{
		std::cerr<<"NULL rigid body in motion customization!"<<std::endl;
		return;
	}
	constant_translation_acceleration_ = acceleration;
}

template <typename Scalar>
void RigidPluginMotionCustomization<Scalar>::setConstantRotationAcceleration(const Vector<Scalar, 3> &acceleration)
{
	if(rigid_body_ == NULL)
	{
		std::cerr<<"NULL rigid body in motion customization!"<<std::endl;
		return;
	}
	constant_rotation_acceleration_ = acceleration;
}

template <typename Scalar>
void RigidPluginMotionCustomization<Scalar>::update(Scalar dt, Scalar current_time)
{
    //constant motion & acceleration
    Vector<Scalar, 3> translation_velocity = constant_translation_velocity_;
	constant_translation_velocity_ += (constant_translation_acceleration_ * dt); // use acceleration to update the translation velocity
    Vector<Scalar, 3> rotation_velocity = constant_rotation_velocity_;
	constant_rotation_velocity_ += (constant_rotation_acceleration_ * dt);

    //period motion
    if(translation_period_ > 0)
    {
        Scalar phase = cos(current_time * 2 * PI / translation_period_);
        translation_velocity += period_translation_velocity_ * phase;
    }
    if(rotation_period_ > 0)
    {
        Scalar phase = cos(current_time * 2 * PI / rotation_period_);
        rotation_velocity += period_rotation_velocity_ * phase;
    }

    //update
    rigid_body_->setGlobalTranslationVelocity(translation_velocity);
    rigid_body_->setGlobalAngularVelocity(rotation_velocity);
    rigid_body_->update(dt, true);
}

///////////////////////////////////////////////////////////////////////////////////////
//RigidBodyTimerTask
///////////////////////////////////////////////////////////////////////////////////////

template <typename Scalar>
RigidBodyTimerTask<Scalar>::RigidBodyTimerTask():
	happen_time_(0),
	value_(0),
	priority_(0)
{
	
}

template <typename Scalar>
RigidBodyTimerTask<Scalar>::RigidBodyTimerTask(RigidBody<Scalar, 3> *rigid_body, Scalar happen_time, TimerTaskType task_type, const Vector<Scalar, 3> &tempValue, unsigned int priority = 0):
	happen_time_(happen_time),
	value_(tempValue),
	rigid_body_(rigid_body),
	type_(task_type),
	priority_(priority)
{

}

template <typename Scalar>
RigidBodyTimerTask<Scalar>::~RigidBodyTimerTask()
{

}

template <typename Scalar>
void RigidBodyTimerTask<Scalar>::setHappenTime(Scalar happen_time)
{
	if (happen_time_ < 0)
	{
		std::cerr<<"Task happen time should be positive in timer task!"<<std::endl;
		return;
	}
	happen_time_ = happen_time;
}

template <typename Scalar>
void RigidBodyTimerTask<Scalar>::setRigidBody(RigidBody<Scalar, 3>* rigid_body)
{
	rigid_body_ = rigid_body;
}

template <typename Scalar>
void RigidBodyTimerTask<Scalar>::setValue(const Vector<Scalar, 3> &tempValue)
{
	value_ = tempValue;
}

template <typename Scalar>
void RigidBodyTimerTask<Scalar>::setType(TimerTaskType task_type)
{
	type_ = task_type;
}

template <typename Scalar>
void RigidBodyTimerTask<Scalar>::setPriority(unsigned int priority)
{
	priority_ = priority;
}

template <typename Scalar>
RigidBody<Scalar, 3> *RigidBodyTimerTask<Scalar>::rigidBody() const
{
	return rigid_body_;
}

template <typename Scalar>
Scalar RigidBodyTimerTask<Scalar>::happenTime() const
{
	return happen_time_;
}

template <typename Scalar>
Vector<Scalar, 3> RigidBodyTimerTask<Scalar>::getValue() const
{
	return value_;
}

template <typename Scalar>
typename RigidBodyTimerTask<Scalar>::TimerTaskType RigidBodyTimerTask<Scalar>::taskType() const
{
	return type_;
}

template <typename Scalar>
bool RigidBodyTimerTask<Scalar>::operator < (const RigidBodyTimerTask<Scalar> &another) const
{
	if (happen_time_ < another.happen_time_) // happen first goes first
		return false;
	else if (happen_time_ > another.happen_time_)
		return true;
	else
	{
		if (priority_ > another.priority_) // same time, higher priority goes first
			return false;
		else if (priority_ < another.priority_)
			return true;
		else
		{
			if (type_ == RigidBodyTimerTask::SET_FREE) // same priority, SET_FREE goes later(the set free will be of no use if set free first and then set constant move immediately)
				return true;
			return false; // same time, same priority, neither SET_FREE, then the order doesn't matter
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////
//RigidDriverPluginMotion
///////////////////////////////////////////////////////////////////////////////////////

template <typename Scalar>
RigidDriverPluginMotion<Scalar>::RigidDriverPluginMotion():
    time_(0)
{

}

template <typename Scalar>
RigidDriverPluginMotion<Scalar>::~RigidDriverPluginMotion()
{

}

template <typename Scalar>
void RigidDriverPluginMotion<Scalar>::onBeginFrame(unsigned int frame)
{
	
}

template <typename Scalar>
void RigidDriverPluginMotion<Scalar>::onEndFrame(unsigned int frame)
{

}

template <typename Scalar>
void RigidDriverPluginMotion<Scalar>::onBeginTimeStep(Scalar time, Scalar dt)
{

}

template <typename Scalar>
void RigidDriverPluginMotion<Scalar>::onEndTimeStep(Scalar time, Scalar dt)
{

}

template <typename Scalar>
void RigidDriverPluginMotion<Scalar>::onBeginRigidStep(unsigned int step, Scalar dt)
{
	// scan the priority_queue task_ from head
	while ((! tasks_.empty()) && time_ >= tasks_.top().happenTime())
	{
		const RigidBodyTimerTask<Scalar> &tempTop = tasks_.top();
		RigidBodyTimerTask<Scalar>::TimerTaskType tempType = tempTop.taskType();
		// determine the task type
		if (tempType == RigidBodyTimerTask<Scalar>::ADD_CONSTANT_TRANSLATION)
		{
			setConstantTranslation(tempTop.rigidBody(), tempTop.getValue());
		}
		else if (tempType == RigidBodyTimerTask<Scalar>::ADD_CONSTANT_ROTATION)
		{
			setConstantRotation(tempTop.rigidBody(), tempTop.getValue());
		}
		else if (tempType == RigidBodyTimerTask<Scalar>::ADD_CONSTANT_TRANSLATION_ACCELERATION)
		{
			setConstantTranslationAcceleration(tempTop.rigidBody(), tempTop.getValue());
		}
		else if (tempType == RigidBodyTimerTask<Scalar>::ADD_CONSTANT_ROTATION_ACCELERATION)
		{
			setConstantRotationAcceleration(tempTop.rigidBody(), tempTop.getValue());
		}
		else if (tempType == RigidBodyTimerTask<Scalar>::SET_FREE) // unset fixed, and delete all motions of this rigid body from the map
		{
			tempTop.rigidBody()->setFixed(false);
			typename std::map<RigidBody<Scalar, 3>*, RigidPluginMotionCustomization<Scalar> >::iterator map_itr;
			map_itr = customized_motions_.find(tempTop.rigidBody());
			if (map_itr != customized_motions_.end())
				customized_motions_.erase(map_itr);
		}
		tasks_.pop();
	}
}

template <typename Scalar>
void RigidDriverPluginMotion<Scalar>::onEndRigidStep(unsigned int step, Scalar dt)
{
    typename std::map<RigidBody<Scalar, 3>*, RigidPluginMotionCustomization<Scalar> >::iterator map_itr;
    for(map_itr = customized_motions_.begin(); map_itr != customized_motions_.end(); map_itr++)
    {
		//RigidPluginMotionCustomization<Scalar> *custom = (map_itr->second);
        (map_itr->second).update(dt, time_);
    }
    time_ += dt;
}

template <typename Scalar>
void RigidDriverPluginMotion<Scalar>::onAddRigidBody(RigidBody<Scalar, 3>* rigid_body)
{

}

template <typename Scalar>
void RigidDriverPluginMotion<Scalar>::onBeginCollisionDetection()
{

}

template <typename Scalar>
void RigidDriverPluginMotion<Scalar>::onEndCollisionDetection()
{

}

template <typename Scalar>
void RigidDriverPluginMotion<Scalar>::setConstantTranslation(RigidBody<Scalar, 3>* rigid_body, const Vector<Scalar, 3>& velocity)
{
    typename std::map<RigidBody<Scalar, 3>*, RigidPluginMotionCustomization<Scalar> >::iterator map_itr;
    map_itr = customized_motions_.find(rigid_body);
    if(map_itr == customized_motions_.end()) // this rigid_body has not been set with motion
    {
        RigidPluginMotionCustomization<Scalar> customization;
        rigid_body->setFixed(true);
        customization.setRigidBody(rigid_body);
        customization.setConstantTranslation(velocity);
        customized_motions_.insert(std::pair<RigidBody<Scalar, 3>*, RigidPluginMotionCustomization<Scalar> >(rigid_body, customization));
    }
    else // this rigid_body has already been set earlier
    {
        customized_motions_[rigid_body].setConstantTranslation(velocity);
    }
}

template <typename Scalar>
void RigidDriverPluginMotion<Scalar>::setConstantRotation(RigidBody<Scalar, 3>* rigid_body, const Vector<Scalar, 3>& velocity)
{
    typename std::map<RigidBody<Scalar, 3>*, RigidPluginMotionCustomization<Scalar> >::iterator map_itr;
    map_itr = customized_motions_.find(rigid_body);
    if(map_itr == customized_motions_.end())
    {
        RigidPluginMotionCustomization<Scalar> customization;
        rigid_body->setFixed(true);
        customization.setRigidBody(rigid_body);
        customization.setConstantRotation(velocity);
        customized_motions_.insert(std::pair<RigidBody<Scalar, 3>*, RigidPluginMotionCustomization<Scalar> >(rigid_body, customization));
    }
    else
    {
        customized_motions_[rigid_body].setConstantRotation(velocity);
    }
}

template <typename Scalar>
void RigidDriverPluginMotion<Scalar>::setPeriodTranslation(RigidBody<Scalar, 3>* rigid_body, const Vector<Scalar, 3>& velocity, Scalar period)
{
    typename std::map<RigidBody<Scalar, 3>*, RigidPluginMotionCustomization<Scalar> >::iterator map_itr;
    map_itr = customized_motions_.find(rigid_body);
    if(map_itr == customized_motions_.end())
    {
        RigidPluginMotionCustomization<Scalar> customization;
        rigid_body->setFixed(true);
        customization.setRigidBody(rigid_body);
        customization.setPeriodTranslation(velocity, period);
        customized_motions_.insert(std::pair<RigidBody<Scalar, 3>*, RigidPluginMotionCustomization<Scalar> >(rigid_body, customization));
    }
    else
    {
        customized_motions_[rigid_body].setPeriodTranslation(velocity, period);
    }
}

template <typename Scalar>
void RigidDriverPluginMotion<Scalar>::setPeriodRotation(RigidBody<Scalar, 3>* rigid_body, const Vector<Scalar, 3>& velocity, Scalar period)
{
	typename std::map<RigidBody<Scalar, 3>*, RigidPluginMotionCustomization<Scalar> >::iterator map_itr;
	map_itr = customized_motions_.find(rigid_body);
	if(map_itr == customized_motions_.end())
	{
		RigidPluginMotionCustomization<Scalar> customization;
		rigid_body->setFixed(true);
		customization.setRigidBody(rigid_body);
		customization.setPeriodRotation(velocity, period);
		customized_motions_.insert(std::pair<RigidBody<Scalar, 3>*, RigidPluginMotionCustomization<Scalar> >(rigid_body, customization));
	}
	else
	{
		customized_motions_[rigid_body].setPeriodRotation(velocity, period);
	}
}

template <typename Scalar>
void RigidDriverPluginMotion<Scalar>::setConstantTranslationAcceleration(RigidBody<Scalar, 3> *rigid_body, const Vector<Scalar, 3> &acceleration)
{
	typename std::map<RigidBody<Scalar, 3>*, RigidPluginMotionCustomization<Scalar> >::iterator map_itr;
	map_itr = customized_motions_.find(rigid_body);
	if(map_itr == customized_motions_.end())
	{
		RigidPluginMotionCustomization<Scalar> customization;
		rigid_body->setFixed(true);
		customization.setRigidBody(rigid_body);
		customization.setConstantTranslationAcceleration(acceleration);
		customized_motions_.insert(std::pair<RigidBody<Scalar, 3>*, RigidPluginMotionCustomization<Scalar> >(rigid_body, customization));
	}
	else
	{
		customized_motions_[rigid_body].setConstantTranslationAcceleration(acceleration);
	}
}

template <typename Scalar>
void RigidDriverPluginMotion<Scalar>::setConstantRotationAcceleration(RigidBody<Scalar, 3> *rigid_body, const Vector<Scalar, 3> &acceleration)
{
	typename std::map<RigidBody<Scalar, 3>*, RigidPluginMotionCustomization<Scalar> >::iterator map_itr;
	map_itr = customized_motions_.find(rigid_body);
	if(map_itr == customized_motions_.end())
	{
		RigidPluginMotionCustomization<Scalar> customization;
		rigid_body->setFixed(true);
		customization.setRigidBody(rigid_body);
		customization.setConstantRotationAcceleration(acceleration);
		customized_motions_.insert(std::pair<RigidBody<Scalar, 3>*, RigidPluginMotionCustomization<Scalar> >(rigid_body, customization));
	}
	else
	{
		customized_motions_[rigid_body].setConstantRotationAcceleration(acceleration);
	}
}

template <typename Scalar>
void RigidDriverPluginMotion<Scalar>::addTimerTask(RigidBody<Scalar, 3> *rigid_body, Scalar happen_time, typename RigidBodyTimerTask<Scalar>::TimerTaskType task_type, const Vector<Scalar, 3> &tempValue, unsigned int priority = 0)
{
	if (happen_time < 0)
	{
		std::cerr<<"Task happen time should be positive in add timer task in plugin motion!"<<std::endl;
		return;
	}
	RigidBodyTimerTask<Scalar> timerTask(rigid_body, happen_time, task_type, tempValue, priority);
	tasks_.push(timerTask);
}

//explicit instantiation
template class RigidPluginMotionCustomization<float>;
template class RigidPluginMotionCustomization<double>;
template class RigidBodyTimerTask<float>;
template class RigidBodyTimerTask<double>;
template class RigidDriverPluginMotion<float>;
template class RigidDriverPluginMotion<double>;

}
