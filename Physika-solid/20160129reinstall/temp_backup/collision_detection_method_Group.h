/*
 * @file  collision_detection_method_Group.h
 * @collision detection using clustering
 * @author Ke Quan
 * 
 * This file is part of Physika, a versatile physics simulation library.
 * Copyright (C) 2016 Physika Group.
 *
 * This Source Code Form is subject to the terms of the GNU General Public License v2.0. 
 * If a copy of the GPL was not distributed with this file, you can obtain one at:
 * http://www.gnu.org/licenses/gpl-2.0.html
 *
 */

#ifndef PHYSIKA_DYNAMICS_COLLIDABLE_OBJECTS_COLLISION_DETECTION_METHOD_GROUP_H_
#define PHYSIKA_DYNAMICS_COLLIDABLE_OBJECTS_COLLISION_DETECTION_METHOD_GROUP_H_

#include <vector>

#include "Physika_Dynamics/Collidable_Objects/collision_detection_method.h"
#include "Physika_Dynamics/Collidable_Objects/collision_pair.h"
#include "Physika_Geometry/Bounding_Volume/bvh_base.h"
#include "Physika_Geometry/Bounding_Volume/scene_bvh.h"
#include "Physika_Geometry/Bounding_Volume/object_bvh.h"
#include "Physika_Core/Vectors/vector.h"

namespace Physika{

template <typename Scalar, int Dim>
class Segment
{
public:
	Segment();
	Segment(const Vector<Scalar, Dim> &point1, const Vector<Scalar, Dim> &point2);
	~Segment();
	bool common(const Segment<Scalar, Dim> &segment) const;
	Vector<Scalar, Dim> point1() const;
	Vector<Scalar, Dim> point2() const;
private:
	Vector<Scalar, Dim> point1_, point2_;
};

template <typename Scalar, int Dim>
class CollisionDetectionMethodGroup : public CollisionDetectionMethod<Scalar, Dim>
{
public:
	CollisionDetectionMethodGroup();
	~CollisionDetectionMethodGroup();
	void update();
	void addCollidableObject(CollidableObject<Scalar, Dim> *object);
	bool collisionDetection();
protected:
	SceneBVH<Scalar, Dim> scene_bvh_;
	std::vector<Segment<Scalar, Dim> *> segments_;
	std::vector<unsigned int> object_lhs_index_, object_rhs_index_;
	std::vector<Vector<Scalar, Dim> > object_lhs_face_normal_;
	// still need to save object_l(r)hs_index
	// in order to add ContactPoint<Scalar, Dim>
	std::vector<int> belong_;
	unsigned int num_loops_;
	void getSegment(CollisionPairBase<Scalar, Dim> *pair);
	void getLoop();
	void setNormal();
	static Scalar det3(Scalar a11, Scalar a12, Scalar a13, Scalar a21, Scalar a22, Scalar a23, Scalar a31, Scalar a32, Scalar a33);
	static bool solve31Equation(Scalar a11, Scalar a12, Scalar a13, Scalar a21, Scalar a22, Scalar a23, Scalar a31, Scalar a32, Scalar a33, Scalar b1, Scalar b2, Scalar b3, Scalar &ans_x, Scalar &ans_y, Scalar &ans_z);
};

}

#endif //PHYSIKA_DYNAMICS_COLLIDABLE_OBJECTS_COLLISION_DETECTION_METHOD_GROUP_H_
