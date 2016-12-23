/*
 * @file  collision_pair.h
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

#ifndef PHYSIKA_DYNAMICS_COLLIDABLE_OBJECTS_COLLISION_PAIR_CCD_H_
#define PHYSIKA_DYNAMICS_COLLIDABLE_OBJECTS_COLLISION_PAIR_CCD_H_

#include "Physika_Geometry/Boundary_Meshes/surface_mesh.h"
#include "Physika_Dynamics/Collidable_Objects/collidable_object.h"
#include "Physika_Dynamics/Collidable_Objects/collision_pair.h"

namespace Physika{

template <typename Scalar>
class CollisionPairCCD : public CollisionPairMeshToMesh<Scalar>
{
	
public:
	
	CollisionPairCCD(unsigned int object_lhs_index, unsigned int object_rhs_index, MeshBasedCollidableObject<Scalar>* object_lhs, MeshBasedCollidableObject<Scalar>* object_rhs, unsigned int face_lhs_index, unsigned int face_rhs_index, Vector<Scalar, 3> &contact_point, Vector<Scalar, 3> &contact_normal);
	~CollisionPairCCD();
	// provide interfaces to acquire the contact point and normal from here
	Vector<Scalar, 3> getContactPoint();
	Vector<Scalar, 3> getContactNormal();
	
protected:
	// CCD store the contact point and normal here(do not need to re-compute in contact_point_manager)
	Vector<Scalar, 3> contact_point_;
	Vector<Scalar, 3> contact_normal_;
	
};

}  //end of namespace Physika

#endif  //PHYSIKA_DYNAMICS_COLLIDABLE_OBJECTS_COLLISION_PAIR_CCD_H_