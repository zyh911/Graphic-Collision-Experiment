/*
 * @file  collision_detection_method_CCD.h
 * @continuous collision detection using ICCD in 
 * "ICCD: Interactive Continuous Collision Detection between Deformable Models using Connectivity-Based Culling"
 * Tang et al. 2007
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

#ifndef PHYSIKA_DYNAMICS_COLLIDABLE_OBJECTS_COLLISION_DETECTION_METHOD_CCD_H_
#define PHYSIKA_DYNAMICS_COLLIDABLE_OBJECTS_COLLISION_DETECTION_METHOD_CCD_H_

#include "Physika_Dynamics/Collidable_Objects/collision_detection_method.h"
#include "Physika_Dynamics/Collidable_Objects/mesh_based_collidable_object.h"
#include "Physika_Geometry/Bounding_Volume/bvh_base.h"
#include "Physika_Geometry/Bounding_Volume/scene_bvh.h"
#include "Physika_Geometry/Bounding_Volume/object_bvh_ccd.h"
#include "Physika_Geometry/Boundary_Meshes/face.h"
#include "Physika_Core/Vectors/vector.h"

#include <vector>
#include <map>

namespace Physika{

template <typename Scalar,int Dim>
class CollisionDetectionMethodCCD : public CollisionDetectionMethod<Scalar, Dim>
{
public:
    CollisionDetectionMethodCCD();
    ~CollisionDetectionMethodCCD();
    void update();
    void addCollidableObject(CollidableObject<Scalar, Dim>* object);
	//void addCollidableObject(MeshBasedCollidableObject<Scalar>* object);
    bool collisionDetection();

	static bool hasKey(MeshBasedCollidableObject<Scalar> *object);
	static Vector<Scalar, 3> getPosition(MeshBasedCollidableObject<Scalar> *object, unsigned int face_index, unsigned int vertex_index);

	static bool collide(MeshBasedCollidableObject<Scalar> *object_lhs, MeshBasedCollidableObject<Scalar> *object_rhs, unsigned int face_index_lhs, unsigned int face_index_rhs, Vector<Scalar, 3> &point, Vector<Scalar, 3> &normal);


//protected:

    void continuousSampling();

    SceneBVH<Scalar, Dim> scene_bvh_;
    // a map to store the previous-frame positions of all objects, key is the object pointer
	static std::map<MeshBasedCollidableObject<Scalar> *, std::vector<Vector<Scalar, 3> > > previous_position_;//record the status of bodies in the previous step. This will be used in continuous collision detection.
	
//private:
	// vertec-face test, and edge-edge test
	static bool VFTest(Vector<Scalar, 3> &vertex, Vector<Scalar, 3> &vertex_previous, Vector<Scalar, 3> *face_vertices, Vector<Scalar, 3> *face_vertices_previous, Vector<Scalar, 3> &point, Vector<Scalar, 3> &normal, Vector<Scalar, 3> &face_normal, int tag);
	static bool EETest(Vector<Scalar, 3> *edge1_vertices, Vector<Scalar, 3> *edge1_vertices_previous, Vector<Scalar, 3> *edge2_vertices, Vector<Scalar, 3> *edge2_vertices_previous, Vector<Scalar, 3> &point, Vector<Scalar, 3> &normal, Vector<Scalar, 3> &face_lhs_normal, Vector<Scalar, 3> &face_rhs_normal);
	// both the VF and EE problem can be reduced to the same format vector equation
	static Scalar solveVectorEquation(Vector<Scalar, 3> *vertices, Vector<Scalar, 3> *velocities);//01 dot (23 cross 45) = 0
	static Scalar solveCubicEquation(Scalar a, Scalar b, Scalar c, Scalar d, unsigned int iteration);//ax^3+bx^2+cx+d=0
	// solve the cubic equation use bisection
	static Scalar solveCubicEquationDevide(Scalar a, Scalar b, Scalar c, Scalar d, Scalar minRange, Scalar maxRange, bool direction, unsigned int depth);//true:leftup->rightdown

	static Scalar cubicValue(Scalar a, Scalar b, Scalar c, Scalar d, Scalar x);
	// add the previous position for one object
	static void addPreviousPosition(MeshBasedCollidableObject<Scalar>* object);

};

}

#endif //PHYSIKA_DYNAMICS_COLLIDABLE_OBJECTS_COLLISION_DETECTION_METHOD_CCD_H_