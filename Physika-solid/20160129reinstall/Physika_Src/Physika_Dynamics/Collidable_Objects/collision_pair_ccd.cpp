
/*
 * @file  collision_pair.cpp
 * @pairs of colliding elementaries
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

#include "Physika_Dynamics/Collidable_Objects/collision_pair_ccd.h"
#include "Physika_Dynamics/Collidable_Objects/mesh_based_collidable_object.h"

namespace Physika{

using BoundaryMeshInternal::Vertex;
using SurfaceMeshInternal::Face;
using SurfaceMeshInternal::FaceGroup;

template <typename Scalar>
CollisionPairCCD<Scalar>::CollisionPairCCD(unsigned int object_lhs_index, unsigned int object_rhs_index, MeshBasedCollidableObject<Scalar>* object_lhs, MeshBasedCollidableObject<Scalar>* object_rhs, unsigned int face_lhs_index, unsigned int face_rhs_index, Vector<Scalar, 3> &contact_point, Vector<Scalar, 3> &contact_normal) :
	CollisionPairMeshToMesh(object_lhs_index, object_rhs_index, object_lhs, object_rhs, face_lhs_index, face_rhs_index),
	contact_point_(contact_point),
	contact_normal_(contact_normal)
{
	
}

template <typename Scalar>
CollisionPairCCD<Scalar>::~CollisionPairCCD()
{
	
}

template <typename Scalar>
Vector<Scalar, 3> CollisionPairCCD<Scalar>::getContactPoint()
{
	return contact_point_;
}

template <typename Scalar>
Vector<Scalar, 3> CollisionPairCCD<Scalar>::getContactNormal()
{
	return contact_normal_;
}

template class CollisionPairCCD<float>;
template class CollisionPairCCD<double>;

}