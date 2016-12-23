/*
 * @file mpm_solid_base.cpp 
 * @Brief base class of all MPM drivers for solid.
 * @author Fei Zhu
 * 
 * This file is part of Physika, a versatile physics simulation library.
 * Copyright (C) 2013 Physika Group.
 *
 * This Source Code Form is subject to the terms of the GNU General Public License v2.0. 
 * If a copy of the GPL was not distributed with this file, you can obtain one at:
 * http://www.gnu.org/licenses/gpl-2.0.html
 *
 */

#include <cstdlib>
#include <iostream>
#include <algorithm>
#include "Physika_Core/Utilities/physika_assert.h"
#include "Physika_Core/Utilities/math_utilities.h"
#include "Physika_Core/Vectors/vector_2d.h"
#include "Physika_Core/Vectors/vector_3d.h"
#include "Physika_Dynamics/Particles/solid_particle.h"
#include "Physika_Dynamics/MPM/mpm_solid_base.h"
#include "Physika_Dynamics/MPM/MPM_Step_Methods/mpm_solid_step_method_USL.h"

namespace Physika{

template <typename Scalar, int Dim>
MPMSolidBase<Scalar,Dim>::MPMSolidBase()
    :MPMBase<Scalar,Dim>(),integration_method_(FORWARD_EULER)
{
    this->template setStepMethod<MPMSolidStepMethodUSL<Scalar,Dim> >(); //default step method is USL
}

template <typename Scalar, int Dim>
MPMSolidBase<Scalar,Dim>::MPMSolidBase(unsigned int start_frame, unsigned int end_frame, Scalar frame_rate, Scalar max_dt, bool write_to_file)
    :MPMBase<Scalar,Dim>(start_frame,end_frame,frame_rate,max_dt,write_to_file),integration_method_(FORWARD_EULER)
{
    this->template setStepMethod<MPMSolidStepMethodUSL<Scalar,Dim> >(); //default step method is USL
}

template <typename Scalar, int Dim>
MPMSolidBase<Scalar,Dim>::~MPMSolidBase()
{
    for(unsigned int i = 0; i < particles_.size(); ++i)
        for(unsigned int j = 0; j < particles_[i].size(); ++j)
            if(particles_[i][j])
                delete particles_[i][j];
}

template <typename Scalar, int Dim>
unsigned int MPMSolidBase<Scalar,Dim>::totalParticleNum() const
{
    unsigned int total_particle_num = 0;
    for(unsigned int i = 0; i < objectNum(); ++i)
        total_particle_num += particleNumOfObject(i);
    return total_particle_num;
}

template <typename Scalar, int Dim>
unsigned int MPMSolidBase<Scalar,Dim>::particleNumOfObject(unsigned int object_idx) const
{
    if(object_idx>=objectNum())
    {
        std::cerr<<"Error: object index out of range, program abort!\n";
        std::exit(EXIT_FAILURE);
    }
    return particles_[object_idx].size();
}
 
template <typename Scalar, int Dim>
unsigned int MPMSolidBase<Scalar,Dim>::objectNum() const
{
    return particles_.size();
}
 
template <typename Scalar, int Dim>
void MPMSolidBase<Scalar,Dim>::addObject(const std::vector<SolidParticle<Scalar,Dim>*> &particles_of_object)
{
    std::vector<SolidParticle<Scalar,Dim>*> new_object;
    particles_.push_back(new_object);
    unsigned int new_object_idx = particles_.size() - 1;
    particles_[new_object_idx].resize(particles_of_object.size());
    for(unsigned int i = 0; i < particles_of_object.size(); ++i)
        particles_[new_object_idx][i] = particles_of_object[i]->clone();
    //allocate space and initialize data attached to each particle
    appendAllParticleRelatedDataOfLastObject();
}
 
template <typename Scalar, int Dim>
void MPMSolidBase<Scalar,Dim>::removeObject(unsigned int object_idx)
{
    if(object_idx>=objectNum())
    {
        std::cerr<<"Warning: object index out of range, operation ignored!\n";
        return;
    }
    for(unsigned int i = 0; i < particles_[object_idx].size(); ++i)
        if(particles_[object_idx][i])
            delete particles_[object_idx][i];
    typename std::vector<std::vector<SolidParticle<Scalar,Dim>*> >::iterator iter = particles_.begin() + object_idx;
    particles_.erase(iter);
    //delete other data related to the object
    deleteAllParticleRelatedDataOfObject(object_idx);
}
     
template <typename Scalar, int Dim>
void MPMSolidBase<Scalar,Dim>::addParticle(unsigned int object_idx, const SolidParticle<Scalar,Dim> &particle)
{
    if(object_idx>=objectNum())
    {
        std::cerr<<"Warning: object index out of range, operation ignored!\n";
        return;
    }
    SolidParticle<Scalar,Dim> *new_particle = particle.clone();
    particles_[object_idx].push_back(new_particle);
    //append space and initialize data related to the newly added particle
    appendLastParticleRelatedDataOfObject(object_idx);
}
 
template <typename Scalar, int Dim>
void MPMSolidBase<Scalar,Dim>::removeParticle(unsigned int object_idx, unsigned int particle_idx)
{
    if(object_idx>=objectNum())
    {
        std::cerr<<"Warning: object index out of range, operation ignored!\n";
        return;
    }
    if(particle_idx>=particleNumOfObject(object_idx))
    {
        std::cerr<<"Warning: particle index out of range, operation ignored!\n";
        return;
    }
    if(particles_[object_idx][particle_idx])
        delete particles_[object_idx][particle_idx];
    typename std::vector<SolidParticle<Scalar,Dim>*>::iterator iter = particles_[object_idx].begin() + particle_idx;
    particles_[object_idx].erase(iter);
    //delete other related data
    deleteOneParticleRelatedDataOfObject(object_idx,particle_idx);
}
 
template <typename Scalar, int Dim>
const SolidParticle<Scalar,Dim>& MPMSolidBase<Scalar,Dim>::particle(unsigned int object_idx, unsigned int particle_idx) const
{
    if(object_idx>=objectNum())
    {
        std::cerr<<"Error: object index out of range, program abort!\n";
        std::exit(EXIT_FAILURE);
    }
    if(particle_idx>=particleNumOfObject(object_idx))
    {
        std::cerr<<"Error: particle index out of range, program abort!\n";
        std::exit(EXIT_FAILURE);
    }
    return *particles_[object_idx][particle_idx];
}

template <typename Scalar, int Dim>
SolidParticle<Scalar,Dim>& MPMSolidBase<Scalar,Dim>::particle(unsigned int object_idx, unsigned int particle_idx)
{
    if(object_idx>=objectNum())
    {
        std::cerr<<"Error: object index out of range, program abort!\n";
        std::exit(EXIT_FAILURE);
    }
    if(particle_idx>=particleNumOfObject(object_idx))
    {
        std::cerr<<"Error: particle index out of range, program abort!\n";
        std::exit(EXIT_FAILURE);
    }
    return *particles_[object_idx][particle_idx];
}

template <typename Scalar, int Dim>
const std::vector<SolidParticle<Scalar,Dim>*>& MPMSolidBase<Scalar,Dim>::allParticlesOfObject(unsigned int object_idx) const
{
    if(object_idx>=objectNum())
    {
        std::cerr<<"Error: object index out of range, program abort!\n";
        std::exit(EXIT_FAILURE);
    }
    return particles_[object_idx];
}
    
template <typename Scalar, int Dim>
Vector<Scalar,Dim> MPMSolidBase<Scalar,Dim>::externalForceOnParticle(unsigned int object_idx, unsigned int particle_idx) const
{
    if(object_idx>=objectNum())
    {
        std::cerr<<"Error: object index out of range, program abort!\n";
        std::exit(EXIT_FAILURE);
    }
    if(particle_idx>=particleNumOfObject(object_idx))
    {
        std::cerr<<"Error: particle index out of range, program abort!\n";
        std::exit(EXIT_FAILURE);
    }
    return particle_external_force_[object_idx][particle_idx];
}

template <typename Scalar, int Dim>
void MPMSolidBase<Scalar,Dim>::setExternalForceOnParticle(unsigned int object_idx, unsigned int particle_idx, const Vector<Scalar,Dim> &force)
{
    if(object_idx>=objectNum())
    {
        std::cerr<<"Warning: object index out of range, operation ignored!\n";
        return;
    }
    if(particle_idx>=particleNumOfObject(object_idx))
    {
        std::cerr<<"Warning: particle index out of range, operation ignored!\n";
        return;
    }
    particle_external_force_[object_idx][particle_idx] = force;
}

template <typename Scalar, int Dim>
void MPMSolidBase<Scalar,Dim>::addDirichletParticle(unsigned int object_idx, unsigned int particle_idx)
{
    if(object_idx>=objectNum())
    {
        std::cerr<<"Warning: object index out of range, operation ignored!\n";
        return;
    }
    if(particle_idx>=particleNumOfObject(object_idx))
    {
        std::cerr<<"Warning: particle index out of range, operation ignored!\n";
        return;
    }
    is_dirichlet_particle_[object_idx][particle_idx] = 0x01;
}
    
template <typename Scalar, int Dim>
void MPMSolidBase<Scalar,Dim>::addDirichletParticles(unsigned int object_idx, const std::vector<unsigned int> &particle_idx)
{
    if(object_idx>=objectNum())
    {
        std::cerr<<"Warning: object index out of range, operation ignored!\n";
        return;
    }
    unsigned int invalid_particle = 0;
    for(unsigned int i = 0; i < particle_idx.size(); ++i)
    {
        if(particle_idx[i]>=particleNumOfObject(object_idx))
            ++invalid_particle;
        else
            is_dirichlet_particle_[object_idx][particle_idx[i]] = 0x01;
    }
    if(invalid_particle > 0)
        std::cerr<<"Warning: "<<invalid_particle<<" invalid particle index are ignored!\n";
}
    
template <typename Scalar, int Dim>
void MPMSolidBase<Scalar,Dim>::setTimeIntegrationMethod(const IntegrationMethod &method)
{
    integration_method_ = method;
}

template <typename Scalar, int Dim>
Scalar MPMSolidBase<Scalar,Dim>::maxParticleVelocityNorm() const
{
    if(particles_.empty())
        return 0;
    Scalar min_vel = (std::numeric_limits<Scalar>::max)();
    for(unsigned int i = 0; i < particles_.size(); ++i)
    {
        for(unsigned int j = 0; j < particles_[i].size(); ++j)
        {
            Scalar norm_sqr = (particles_[i][j]->velocity()).normSquared();
            min_vel = norm_sqr < min_vel ? norm_sqr : min_vel;
        }
    }
    return sqrt(min_vel);
}

template <typename Scalar, int Dim>
void MPMSolidBase<Scalar,Dim>::appendAllParticleRelatedDataOfLastObject()
{
    unsigned int last_object_idx = particles_.size() - 1;
    unsigned int particle_num_of_last_object = particles_[last_object_idx].size();
    is_dirichlet_particle_.push_back(std::vector<unsigned char>(particle_num_of_last_object));
    particle_initial_volume_.push_back(std::vector<Scalar>(particle_num_of_last_object));
    particle_external_force_.push_back(std::vector<Vector<Scalar,Dim> >(particle_num_of_last_object));
    for(unsigned int i = 0; i < particle_num_of_last_object; ++i)
    {
        is_dirichlet_particle_[last_object_idx][i] = 0;
        particle_initial_volume_[last_object_idx][i] = particles_[last_object_idx][i]->volume();
        particle_external_force_[last_object_idx][i] = Vector<Scalar,Dim>(0);
    }
}

template <typename Scalar, int Dim>
void MPMSolidBase<Scalar,Dim>::appendLastParticleRelatedDataOfObject(unsigned int object_idx)
{
    PHYSIKA_ASSERT(object_idx < objectNum());
    unsigned int last_particle_idx = particles_[object_idx].size() - 1;
    is_dirichlet_particle_[object_idx].push_back(0);
    particle_initial_volume_[object_idx].push_back(particles_[object_idx][last_particle_idx]->volume());
    particle_external_force_[object_idx].push_back(Vector<Scalar,Dim>(0));
}

template <typename Scalar, int Dim>
void MPMSolidBase<Scalar,Dim>::deleteAllParticleRelatedDataOfObject(unsigned int object_idx)
{
    PHYSIKA_ASSERT(object_idx < objectNum());
    typename std::vector<std::vector<unsigned char> >::iterator iter1 = is_dirichlet_particle_.begin() + object_idx;
    is_dirichlet_particle_.erase(iter1);
    typename std::vector<std::vector<Scalar> >::iterator iter2 = particle_initial_volume_.begin() + object_idx;
    particle_initial_volume_.erase(iter2);
    typename std::vector<std::vector<Vector<Scalar,Dim> > >::iterator iter3 = particle_external_force_.begin() + object_idx;
    particle_external_force_.erase(iter3);
}
    
template <typename Scalar, int Dim>
void MPMSolidBase<Scalar,Dim>::deleteOneParticleRelatedDataOfObject(unsigned int object_idx, unsigned int particle_idx)
{
    PHYSIKA_ASSERT(object_idx < objectNum());
    PHYSIKA_ASSERT(particle_idx < particleNumOfObject(object_idx));
    typename std::vector<unsigned char>::iterator iter1 = is_dirichlet_particle_[object_idx].begin() + particle_idx;
    is_dirichlet_particle_[object_idx].erase(iter1);
    typename std::vector<Scalar>::iterator iter2 = particle_initial_volume_[object_idx].begin() + particle_idx;
    particle_initial_volume_[object_idx].erase(iter2);
    typename std::vector<Vector<Scalar,Dim> >::iterator iter3 = particle_external_force_[object_idx].begin() + particle_idx;
    particle_external_force_[object_idx].erase(iter3);
}

//explicit instantiations
template class MPMSolidBase<float,2>;
template class MPMSolidBase<float,3>;
template class MPMSolidBase<double,2>;
template class MPMSolidBase<double,3>;

}  //end of namespace Physika
