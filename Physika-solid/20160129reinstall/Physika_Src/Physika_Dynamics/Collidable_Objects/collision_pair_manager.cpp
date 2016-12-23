/*
 * @file  collision_pair_manager.cpp
 * @results of collision detection
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

#include "Physika_Dynamics/Collidable_Objects/collision_pair.h"
#include "Physika_Dynamics/Collidable_Objects/collision_pair_manager.h"
#include "Physika_Dynamics/Collidable_Objects/mesh_based_collidable_object.h"

namespace Physika{

template <typename Scalar,int Dim>
CollisionPairManager<Scalar, Dim>::CollisionPairManager():
	number_pcs_(0),
	current_object_lhs_idx_(0),
	current_object_rhs_idx_(0)
{
}

template <typename Scalar,int Dim>
CollisionPairManager<Scalar, Dim>::~CollisionPairManager()
{
	cleanCollisionPairs();
}

template <typename Scalar,int Dim>
unsigned int CollisionPairManager<Scalar, Dim>::numberPCS() const
{
	return number_pcs_;
}

template <typename Scalar,int Dim>
unsigned int CollisionPairManager<Scalar, Dim>::numberCollision() const
{
	unsigned int number_collision = static_cast<unsigned int>(collision_pairs_.size());
	return number_collision;
}

template <typename Scalar,int Dim>
const std::vector<CollisionPairBase<Scalar, Dim>*>& CollisionPairManager<Scalar, Dim>::collisionPairs() const
{
	return collision_pairs_;
}

template <typename Scalar,int Dim>
std::vector<CollisionPairBase<Scalar, Dim>*>& CollisionPairManager<Scalar, Dim>::collisionPairs()
{
	return collision_pairs_;
}

template <typename Scalar,int Dim>
CollisionPairBase<Scalar, Dim>* CollisionPairManager<Scalar, Dim>::collisionPair(unsigned int index)
{
    if(index > numberCollision())
    {
        std::cerr<<"Collision index our of range!"<<std::endl;
        return NULL;
    }
    return collision_pairs_[index];
}

template <typename Scalar,int Dim>
void CollisionPairManager<Scalar, Dim>::setCurrentObjectIndex(unsigned int current_object_lhs_idx, unsigned int current_object_rhs_idx)
{
	current_object_lhs_idx_ = current_object_lhs_idx;
	current_object_rhs_idx_ = current_object_rhs_idx;
}

template <typename Scalar,int Dim>
void CollisionPairManager<Scalar, Dim>::addPCS()
{
	number_pcs_++;
}

template <typename Scalar,int Dim>
void CollisionPairManager<Scalar, Dim>::addCollisionPair(CollisionPairBase<Scalar, Dim>* collision_pair)
{
	collision_pairs_.push_back(collision_pair);
}

template <typename Scalar,int Dim>
void CollisionPairManager<Scalar, Dim>::cleanCollisionPairs()
{
    number_pcs_ = 0;
	unsigned int number_collision = static_cast<unsigned int>(collision_pairs_.size());
	for(unsigned int i = 0 ; i < number_collision; ++i)
	{
		if(collision_pairs_[i] != NULL)
		{
			delete collision_pairs_[i];
			collision_pairs_[i] = NULL;
		}
	}
	collision_pairs_.clear();
}

template <typename Scalar,int Dim>
void CollisionPairManager<Scalar, Dim>::addCollisionPair(MeshBasedCollidableObject<Scalar>* object_lhs, MeshBasedCollidableObject<Scalar>* object_rhs, unsigned int face_lhs_index, unsigned int face_rhs_index)
{
    if(Dim == 2)
    {
        std::cerr<<"Can't add a 3D collision pair to 2D results!"<<std::endl;
        return;
    }
	CollisionPairMeshToMesh<Scalar>* collision_pair = new CollisionPairMeshToMesh<Scalar>(current_object_lhs_idx_, current_object_rhs_idx_, object_lhs, object_rhs, face_lhs_index, face_rhs_index);
	collision_pairs_.push_back(dynamic_cast<CollisionPairBase<Scalar, Dim>*>(collision_pair));
}

template <typename Scalar, int Dim>
unsigned int CollisionPairManager<Scalar, Dim>::getCurrentObjectLhsIndex() const
{

	return current_object_lhs_idx_;

}

template <typename Scalar, int Dim>
unsigned int CollisionPairManager<Scalar, Dim>::getCurrentObjectRhsIndex() const
{

	return current_object_rhs_idx_;

}

template class CollisionPairManager<float, 2>;
template class CollisionPairManager<double, 2>;
template class CollisionPairManager<float, 3>;
template class CollisionPairManager<double, 3>;

}