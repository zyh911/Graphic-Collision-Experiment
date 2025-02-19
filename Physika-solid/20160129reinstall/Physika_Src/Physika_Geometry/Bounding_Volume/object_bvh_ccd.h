/*
 * @file  object_bvh.h
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

#ifndef PHYSIKA_GEOMETRY_BOUNDING_VOLUME_OBJECT_BVH_CCD_H_
#define PHYSIKA_GEOMETRY_BOUNDING_VOLUME_OBJECT_BVH_CCD_H_

#include "Physika_Geometry/Bounding_Volume/object_bvh.h"

namespace Physika{

template <typename Scalar,int Dim> class Vector;
template <typename Scalar,int Dim> class BVHBase;
template <typename Scalar,int Dim> class CollidableObject;
template <typename Scalar> class MeshBasedCollidableObject;

template <typename Scalar,int Dim>
class ObjectBVHCCD : public ObjectBVH<Scalar, Dim>
{
public:
	//constructors && deconstructors
	//ObjectBVHCCD();
	//~ObjectBVHCCD();
	
protected:
	//structure maintain
	void buildFromMeshObject(MeshBasedCollidableObject<Scalar>* collidable_object);
};

}  //end of namespace Physika

#endif  //PHYSIKA_GEOMETRY_BOUNDING_VOLUME_OBJECT_BVH_H_