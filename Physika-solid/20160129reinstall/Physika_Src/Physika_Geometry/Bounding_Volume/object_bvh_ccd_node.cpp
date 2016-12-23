/*
 * @file  object_bvh_node.cpp
 * @node of a collidable object's BVH
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

#include "Physika_Geometry/Bounding_Volume/bvh_node_base.h"
#include "Physika_Geometry/Bounding_Volume/object_bvh_ccd_node.h"
#include "Physika_Geometry/Bounding_Volume/bounding_volume_kdop18.h"
#include "Physika_Geometry/Bounding_Volume/bounding_volume_octagon.h"
#include "Physika_Core/Vectors/vector_3d.h"
#include "Physika_Core/Vectors/vector_2d.h"
#include "Physika_Geometry/Boundary_Meshes/surface_mesh.h"
#include "Physika_Dynamics/Collidable_Objects/mesh_based_collidable_object.h"
#include "Physika_Dynamics/Collidable_Objects/collision_pair_manager.h"
#include "Physika_Dynamics/Collidable_Objects/collision_detection_method_CCD.h"
#include "Physika_Dynamics/Collidable_Objects/collision_pair_ccd.h"

namespace Physika{

using BoundaryMeshInternal::Vertex;
using SurfaceMeshInternal::Face;
using SurfaceMeshInternal::FaceGroup;


template <typename Scalar,int Dim>
bool ObjectBVHCCDNode<Scalar, Dim>::elemTest(const BVHNodeBase<Scalar, Dim>* const target, CollisionPairManager<Scalar, Dim>& collision_result)
{
	if(target == NULL || !target->isObjectNode())
		return false;
	if(target->BVType() != this->bv_type_)
		return false;
	const ObjectBVHCCDNode<Scalar, Dim>* object_target = dynamic_cast<const ObjectBVHCCDNode<Scalar, Dim>*>(target);
	if(object_target == NULL)
		return false;
	if(object_type_ == CollidableObjectInternal::MESH_BASED && object_target->objectType() == CollidableObjectInternal::MESH_BASED)
	{
		MeshBasedCollidableObject<Scalar>* mesh_object_this = dynamic_cast<MeshBasedCollidableObject<Scalar>*>(object_);
		MeshBasedCollidableObject<Scalar>* mesh_object_target = dynamic_cast<MeshBasedCollidableObject<Scalar>*>(object_target->object_);
		if(mesh_object_this == NULL || mesh_object_target == NULL)
			return false;
		if(!has_face_ || !object_target->has_face_)
			return false;
		//bool is_collide = mesh_object_this->collideWithMesh(mesh_object_target, face_index_, object_target->face_index_);
		Vector<Scalar, 3> contact_point, contact_normal;
		bool is_collide = CollisionDetectionMethodCCD<Scalar, Dim>::collide(mesh_object_this, mesh_object_target, face_index_, object_target->face_index_, contact_point, contact_normal);
		collision_result.addPCS();
		if(is_collide)
		{
			unsigned int current_object_lhs_index = collision_result.getCurrentObjectLhsIndex();
			unsigned int current_object_rhs_index = collision_result.getCurrentObjectRhsIndex();
			
			CollisionPairCCD<Scalar> *pair = new CollisionPairCCD<Scalar>(current_object_lhs_index, current_object_rhs_index, mesh_object_this, mesh_object_target, face_index_, object_target -> face_index_, contact_point, contact_normal);
			collision_result.addCollisionPair(dynamic_cast<CollisionPairBase<Scalar, Dim> *>(pair));
		}
	}
	return false;
}

template <typename Scalar, int Dim>
void ObjectBVHCCDNode<Scalar, Dim>::buildFromFace()
{

	//std::cout << "in ObjectBVHCCDNode -> buildFromFace !\n";

    if(Dim == 2)
    {
        std::cerr<<"Can't build a 2D BVH from a 3D mesh!"<<std::endl;
        return;
    }
	this->is_leaf_ = true;
	if(!has_face_)
		return;
	if(object_->objectType() != CollidableObjectInternal::MESH_BASED)
		return;
	if(this->bounding_volume_ == NULL)
	{
        this->bounding_volume_ = BoundingVolumeInternal::createBoundingVolume<Scalar, Dim>(this->bv_type_);
	}
	this->bounding_volume_->setEmpty();
	MeshBasedCollidableObject<Scalar>* object = dynamic_cast<MeshBasedCollidableObject<Scalar>*>(object_);
	//MeshBasedCollidableObject<Scalar>* object_v = object;
	if(object == NULL)
		return;
	const Face<Scalar>& face = object->mesh()->face(face_index_);
	unsigned int point_num = face.numVertices();
	for(unsigned int i = 0; i < point_num; ++i)
	{
        Vector<Scalar,3> vertex_pos = object->vertexPosition(face.vertex(i).positionIndex());
		this->bounding_volume_->unionWith(*dynamic_cast<Vector<Scalar, Dim>* >(&vertex_pos));
	}
	//std::vector<Vector<Scalar, 3> > previous_vertex_position = CollisionDetectionMethodCCD::previous_position[object][face_index_];
	
	//std::cout << "before union with previous position \n";

	if (! CollisionDetectionMethodCCD<Scalar, Dim>::hasKey(object))
		return;

	for (unsigned int i = 0; i <= point_num; ++ i)
	{
		
		//std::cout << "\ti = " << i << '\n';	

		Vector<Scalar,3> vertex_pos = CollisionDetectionMethodCCD<Scalar, Dim>::getPosition(object, face_index_, i);
		
		//std::cout << "\tafter get position ()\n";

		this->bounding_volume_->unionWith(*dynamic_cast<Vector<Scalar, Dim>* >(&vertex_pos));

		//std::cout << "\tafter union with ()\n";

	}

	//std::cout << "end of build from face\n";
	
}


//template class ObjectBVHCCDNode<float, 2>;
//template class ObjectBVHCCDNode<double, 2>;
template class ObjectBVHCCDNode<float, 3>;
template class ObjectBVHCCDNode<double, 3>;

}
