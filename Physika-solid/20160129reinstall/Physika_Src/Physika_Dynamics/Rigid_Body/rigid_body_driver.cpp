/*
 * @file rigid_body_driver.cpp
 * @Basic rigid body driver class.
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

#include <iostream>

#include <limits>
#include "Physika_Dynamics/Rigid_Body/rigid_body.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_2d.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_3d.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_driver.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_driver_utility.h"
#include "Physika_Dynamics/Collidable_Objects/mesh_based_collidable_object.h"
#include "Physika_Geometry/Boundary_Meshes/surface_mesh.h"
#include "Physika_Core/Utilities/math_utilities.h"
#include "Physika_Dynamics/Rigid_Body/rigid_driver_plugin.h"
#include "Physika_Dynamics/Collidable_Objects/collision_detection_method_DTBVH.h"
#include "Physika_Dynamics/Rigid_Body/rigid_response_method_BLCP.h"

namespace Physika{

///////////////////////////////////////////////////////////////////////////////////////
//RigidBodyArchive
///////////////////////////////////////////////////////////////////////////////////////

template <typename Scalar,int Dim>
RigidBodyArchive<Scalar, Dim>::RigidBodyArchive():
	index_(0),
	rigid_body_(NULL),
	collide_object_(NULL)
{

}

template <typename Scalar,int Dim>
RigidBodyArchive<Scalar, Dim>::RigidBodyArchive(RigidBody<Scalar, Dim>* rigid_body)
{
	setRigidBody(rigid_body);
}

template <typename Scalar,int Dim>
RigidBodyArchive<Scalar, Dim>::~RigidBodyArchive()
{
	delete collide_object_;
}

template <typename Scalar,int Dim>
void RigidBodyArchive<Scalar, Dim>::setRigidBody(RigidBody<Scalar, Dim>* rigid_body)
{
    setRigidBody(rigid_body, DimensionTrait<Dim>());
}

template <typename Scalar,int Dim>
unsigned int RigidBodyArchive<Scalar, Dim>::index() const
{
	return index_;
}

template <typename Scalar,int Dim>
void RigidBodyArchive<Scalar, Dim>::setIndex(unsigned int index)
{
	index_ = index;
}

template <typename Scalar,int Dim>
RigidBody<Scalar, Dim>* RigidBodyArchive<Scalar, Dim>::rigidBody()
{
	return rigid_body_;
}

template <typename Scalar,int Dim>
CollidableObject<Scalar, Dim>* RigidBodyArchive<Scalar, Dim>::collideObject()
{
	return collide_object_;
}

template <typename Scalar,int Dim>
void RigidBodyArchive<Scalar, Dim>::setRigidBody(RigidBody<Scalar, Dim>* rigid_body, DimensionTrait<2> trait)
{
    //to do
}

template <typename Scalar,int Dim>
void RigidBodyArchive<Scalar, Dim>::setRigidBody(RigidBody<Scalar, Dim>* rigid_body, DimensionTrait<3> trait)
{
    if(rigid_body == NULL)
        return;

    rigid_body_ = rigid_body;
    RigidBody<Scalar, 3>* rigid_body_3d = dynamic_cast<RigidBody<Scalar, 3>* >(rigid_body);

    switch(rigid_body->objectType())
    {
    case CollidableObjectInternal::MESH_BASED: collide_object_ = dynamic_cast<CollidableObject<Scalar, Dim>* >(new MeshBasedCollidableObject<Scalar>());break;
    default: std::cerr<<"Object type error!"<<std::endl; return;
    }
    MeshBasedCollidableObject<Scalar>* mesh_object = dynamic_cast<MeshBasedCollidableObject<Scalar>*>(collide_object_);
    mesh_object->setMesh(rigid_body_3d->mesh());
    mesh_object->setTransform(rigid_body_3d->transformPtr());
}

///////////////////////////////////////////////////////////////////////////////////////
//TimerRigidBody
///////////////////////////////////////////////////////////////////////////////////////

template <typename Scalar, int Dim>
TimerRigidBody<Scalar, Dim>::TimerRigidBody():
	add_time_(0)
{
}

template <typename Scalar, int Dim>
TimerRigidBody<Scalar, Dim>::TimerRigidBody(RigidBody<Scalar, Dim> *rigid_body, Scalar add_time):
	rigid_body_(rigid_body),
	add_time_(add_time)
{
}

template <typename Scalar, int Dim>
void TimerRigidBody<Scalar, Dim>::setRigidBody(RigidBody<Scalar, Dim> *rigid_body)
{
	rigid_body_ = rigid_body;
}

template <typename Scalar, int Dim>
void TimerRigidBody<Scalar, Dim>::setAddTime(Scalar add_time)
{
	add_time_ = add_time;
}

template <typename Scalar, int Dim>
RigidBody<Scalar, Dim> *TimerRigidBody<Scalar, Dim>::rigidBody() const
{
	return rigid_body_;
}

template <typename Scalar, int Dim>
Scalar TimerRigidBody<Scalar, Dim>::addTime() const
{
	return add_time_;
}

template <typename Scalar, int Dim>
bool TimerRigidBody<Scalar, Dim>::operator < (const TimerRigidBody<Scalar, Dim> &another) const
{
	return (add_time_ > another.add_time_);
}

///////////////////////////////////////////////////////////////////////////////////////
//RigidBodyDriver
///////////////////////////////////////////////////////////////////////////////////////


template <typename Scalar,int Dim>
RigidBodyDriver<Scalar, Dim>::RigidBodyDriver():
    collision_detection_method_(new CollisionDetectionMethodDTBVH<Scalar, Dim>()),
    collision_response_method_(new RigidResponseMethodBLCP<Scalar, Dim>()),
    is_default_detection_method_(true),
    is_default_response_method_(true),
    gravity_(9.81),
    step_(0),
	time_(0)
{
    this->dt_ = 0.01;
    collision_response_method_->setRigidDriver(this);
	//pluginPOV = new RigidDriverPluginPOV<Scalar, Dim>();
}

template <typename Scalar,int Dim>
RigidBodyDriver<Scalar, Dim>::~RigidBodyDriver()
{
	unsigned int num_rigid_body = numRigidBody();
	for(unsigned int i = 0; i < num_rigid_body; ++i)
	{
		delete rigid_body_archives_[i];
	}
	rigid_body_archives_.clear();
    if(is_default_detection_method_)
        delete collision_detection_method_;
    if(is_default_response_method_)
        delete collision_response_method_;
}

template <typename Scalar,int Dim>
void RigidBodyDriver<Scalar, Dim>::initConfiguration(const std::string &file_name)
{
//TO DO
}

template <typename Scalar,int Dim>
void RigidBodyDriver<Scalar, Dim>::printConfigFileFormat()
{
//TO DO
}

template <typename Scalar,int Dim>
void RigidBodyDriver<Scalar, Dim>::initSimulationData()
{
//TO DO
}


// ���������̣�ÿ��ִ��һ��ʱ�䲽

template <typename Scalar,int Dim>
void RigidBodyDriver<Scalar, Dim>::advanceStep(Scalar dt)
{
    //update step and time
    step_++;
	if(step_ > 5000)
		exit(1);
	time_ += dt;

	//plugin
	unsigned int plugin_num = static_cast<unsigned int>((this->plugins_).size());

	RigidDriverPlugin<Scalar, Dim>* plugin;

	for(unsigned int i = 0; i < plugin_num; ++i)
	{
		plugin = dynamic_cast<RigidDriverPlugin<Scalar, Dim>*>((this->plugins_)[i]);
		if(plugin != NULL)
			plugin->onBeginRigidStep(step_, dt);
	}

	// check timer rigid body, only need to check the head(top) of the queue ������Ƿ�����ʱ��ָ���ĸ������
	while ((! timerRigidBodies.empty()) && time_ >= timerRigidBodies.top().addTime())
	{
		addRigidBody(timerRigidBodies.top().rigidBody());
		timerRigidBodies.pop();
	}
    
    //simulation step
    performGravity(dt);
	collisionDetection();
    collisionResponse();
    updateRigidBody(dt);
	
    //plugin
    for(unsigned int i = 0; i < plugin_num; ++i)
    {
        plugin = dynamic_cast<RigidDriverPlugin<Scalar, Dim>*>((this->plugins_)[i]);
        if(plugin != NULL)
            plugin->onEndRigidStep(step_, dt);
    }
}

template <typename Scalar,int Dim>
Scalar RigidBodyDriver<Scalar, Dim>::computeTimeStep()
{
	return this->dt_;
}

template <typename Scalar,int Dim>
bool RigidBodyDriver<Scalar,Dim>::withRestartSupport() const
{
    return false;
}

template <typename Scalar,int Dim>
void RigidBodyDriver<Scalar, Dim>::write(const std::string &file_name)
{
//TO DO
}

template <typename Scalar,int Dim>
void RigidBodyDriver<Scalar, Dim>::read(const std::string &file_name)
{
//TO DO
}

template <typename Scalar,int Dim>
void RigidBodyDriver<Scalar, Dim>::addRigidBody(RigidBody<Scalar, Dim>* rigid_body)
{
	if(rigid_body == NULL)
		return;

	unsigned int old_num = numRigidBody();
	unsigned int new_num = old_num + 1;

    //add this rigid body
	RigidBodyArchive<Scalar, Dim>* archive = new RigidBodyArchive<Scalar, Dim>(rigid_body);
	archive->setIndex(numRigidBody());
    rigid_body_archives_.push_back(archive);
    collision_detection_method_->addCollidableObject(archive->collideObject());

    //plugin
	unsigned int plugin_num = static_cast<unsigned int>((this->plugins_).size());
	RigidDriverPlugin<Scalar, Dim>* plugin;
	for(unsigned int i = 0; i < plugin_num; ++i)
	{
		plugin = dynamic_cast<RigidDriverPlugin<Scalar, Dim>*>((this->plugins_)[i]);
		if(plugin != NULL)
			plugin->onAddRigidBody(rigid_body);
	}
}

template <typename Scalar,int Dim>
void RigidBodyDriver<Scalar, Dim>::addTimerRigidBody(RigidBody<Scalar, Dim> * rigid_body, Scalar add_time)
{
	TimerRigidBody<Scalar, Dim> temp(rigid_body, add_time);
	timerRigidBodies.push(temp);
}

template <typename Scalar,int Dim>
void RigidBodyDriver<Scalar, Dim>::setGravity(Scalar gravity)
{
    gravity_ = gravity;
}

template <typename Scalar,int Dim>
unsigned int RigidBodyDriver<Scalar, Dim>::numRigidBody() const
{
	return static_cast<unsigned int>(rigid_body_archives_.size());
}

template <typename Scalar,int Dim>
RigidBody<Scalar, Dim>* RigidBodyDriver<Scalar, Dim>::rigidBody(unsigned int index)
{
	if(index >= numRigidBody())
	{
		std::cerr<<"Rigid body index out of range!"<<std::endl;
		return NULL;
	}
	return rigid_body_archives_[index]->rigidBody();
}

template <typename Scalar,int Dim>
unsigned int RigidBodyDriver<Scalar, Dim>::numCollisionPair() const
{
    return collision_detection_method_->numCollisionPair();
}

template <typename Scalar,int Dim>
CollisionPairBase<Scalar, Dim>* RigidBodyDriver<Scalar, Dim>::collisionPair(unsigned int index)
{
    return collision_detection_method_->collisionPair(index);
}

template <typename Scalar,int Dim>
unsigned int RigidBodyDriver<Scalar, Dim>::numContactPoint() const
{
    return collision_detection_method_->numContactPoint();
}


template <typename Scalar,int Dim>
ContactPoint<Scalar, Dim>* RigidBodyDriver<Scalar, Dim>::contactPoint(unsigned int index)
{
    return collision_detection_method_->contactPoint(index);
}

template <typename Scalar,int Dim>
void RigidBodyDriver<Scalar, Dim>::addPlugin(DriverPluginBase<Scalar>* plugin)
{
    if(plugin == NULL)
    {
        std::cerr<<"Null plugin!"<<std::endl;
        return;
    }
    if(dynamic_cast<RigidDriverPlugin<Scalar, Dim>* >(plugin) == NULL)
    {
        std::cerr<<"Wrong plugin type!"<<std::endl;
        return;
    }
    plugin->setDriver(this);
    this->plugins_.push_back(plugin);
}

template <typename Scalar,int Dim>
void RigidBodyDriver<Scalar, Dim>::setCollisionDetectionMethod(CollisionDetectionMethod<Scalar, Dim>* collision_detection_method)
{
    if(collision_detection_method_ == NULL)
    {
        std::cerr<<"Null collision detection method!"<<std::endl;
        return;
    }
    if(is_default_detection_method_)
        delete collision_detection_method_;
    collision_detection_method_ = collision_detection_method;
    is_default_detection_method_ = false;
}

template <typename Scalar,int Dim>
void RigidBodyDriver<Scalar, Dim>::setCollisionResponseMethod(RigidResponseMethod<Scalar, Dim>* collision_response_method)
{
    if(collision_response_method_ == NULL)
    {
        std::cerr<<"Null collision response method!"<<std::endl;
        return;
    }
    if(is_default_response_method_)
        delete collision_response_method_;
    collision_response_method_ = collision_response_method;
    collision_response_method_->setRigidDriver(this);
    is_default_response_method_ = false;
}

template <typename Scalar,int Dim>
void RigidBodyDriver<Scalar, Dim>::performGravity(Scalar dt)
{
    unsigned int num_rigid_body = numRigidBody();
    RigidBody<Scalar, Dim>* rigid_body;
    for(unsigned int i = 0; i < num_rigid_body; i++)
    {
        rigid_body = rigid_body_archives_[i]->rigidBody();
        if(!rigid_body->isFixed())
        {
            rigid_body->performGravity(gravity_, dt);
        }
    }
}

template <typename Scalar,int Dim>
bool RigidBodyDriver<Scalar, Dim>::collisionDetection()
{
    //plugin
    unsigned int plugin_num = static_cast<unsigned int>((this->plugins_).size());
    RigidDriverPlugin<Scalar, Dim>* plugin;
    for(unsigned int i = 0; i < plugin_num; ++i)
    {
        plugin = dynamic_cast<RigidDriverPlugin<Scalar, Dim>*>((this->plugins_)[i]);
        if(plugin != NULL)
            plugin->onBeginCollisionDetection();
    }
    
    //clean
    collision_detection_method_->cleanResults();

    //update and collide
    collision_detection_method_->update();
    bool is_collide = collision_detection_method_->collisionDetection();

    //plugin
    plugin_num = static_cast<unsigned int>((this->plugins_).size());
    for(unsigned int i = 0; i < plugin_num; ++i)
    {
        plugin = dynamic_cast<RigidDriverPlugin<Scalar, Dim>*>((this->plugins_)[i]);
        if(plugin != NULL)
            plugin->onEndCollisionDetection();
    }
    return is_collide;
}

template <typename Scalar,int Dim>
void RigidBodyDriver<Scalar, Dim>::collisionResponse()
{
    collision_response_method_->collisionResponse();
}

template <typename Scalar,int Dim>
void RigidBodyDriver<Scalar, Dim>::updateRigidBody(Scalar dt)
{
    //update
    unsigned int num_rigid_body = numRigidBody();
    RigidBody<Scalar, Dim>* rigid_body;
    for(unsigned int i = 0; i < num_rigid_body; i++)
    {
        rigid_body = rigid_body_archives_[i]->rigidBody();
        if(!rigid_body->isFixed())
        {
            rigid_body->update(dt);
        }
    }
}

//explicit instantiation
//template class RigidBodyArchive<float, 2>;
//template class RigidBodyArchive<double, 2>;
template class RigidBodyArchive<float, 3>;
template class RigidBodyArchive<double, 3>;

//template class RigidBodyDriver<float, 2>;
//template class RigidBodyDriver<double, 2>;
template class RigidBodyDriver<float, 3>;
template class RigidBodyDriver<double, 3>;

}
