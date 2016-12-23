/*
 * @file  object_bvh.cpp
 * @bounding volume hierarchy (BVH) of a collidable object, second level of DT-BVH [Tang et al. 2009]
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

#include "Physika_Geometry/Bounding_Volume/object_bvh_ccd.h"
#include "Physika_Geometry/Bounding_Volume/object_bvh_ccd_node.h"
#include "Physika_Geometry/Bounding_Volume/bvh_base.h"
#include "Physika_Geometry/Bounding_Volume/bvh_node_base.h"
#include "Physika_Dynamics/Collidable_Objects/mesh_based_collidable_object.h"
#include "Physika_Geometry/Bounding_Volume/bounding_volume_kdop18.h"
#include "Physika_Core/Vectors/vector_3d.h"

namespace Physika{

using BoundaryMeshInternal::Vertex;
using SurfaceMeshInternal::Face;
using SurfaceMeshInternal::FaceGroup;

template <typename Scalar, int Dim>
void ObjectBVHCCD<Scalar, Dim>::buildFromMeshObject(MeshBasedCollidableObject<Scalar>* collidable_object)
{
    if(Dim == 2)
    {
        std::cerr<<"Can't build a 2D BVH from a 3D mesh!"<<std::endl;
        return;
    }
	if(!this->isEmpty())
		BVHBase<Scalar, Dim>::clean();
	if(collidable_object == NULL)
    {
        std::cerr<<"Null object when building a BVH from mesh!"<<std::endl;
		return;
    }
	SurfaceMesh<Scalar>* mesh = collidable_object->mesh();
	if(mesh == NULL)
    {
        std::cerr<<"Null mesh when building a BVH from mesh!"<<std::endl;
		return;
    }
	ObjectBVHNode<Scalar, Dim>* node = NULL;

	// update collidable Object vert_pos_vec_;
	updateCollidableObjVertPosVec();
	unsigned int face_num = mesh->numFaces();          
	for (unsigned int face_idx = 0; face_idx < face_num; face_idx++)
	{
		node = new ObjectBVHCCDNode<Scalar, Dim>();
		node->setLeaf(true);
		node->setBVType(this->bv_type_);
		node->setObject(dynamic_cast<CollidableObject<Scalar, Dim>* >(collidable_object));
		node->setFaceIndex(face_idx);
		this->addNode(node);
	}

	this->root_node_ = BVHBase<Scalar, Dim>::buildFromLeafList(0, this->numLeaf());
}

//template class ObjectBVHCCD<float, 2>;
//template class ObjectBVHCCD<double, 2>;
template class ObjectBVHCCD<float, 3>;
template class ObjectBVHCCD<double, 3>;

}
