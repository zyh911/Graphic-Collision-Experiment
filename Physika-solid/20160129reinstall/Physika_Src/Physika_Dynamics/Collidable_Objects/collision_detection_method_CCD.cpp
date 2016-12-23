/*
 * @file  collision_detection_method_CCD.cpp
 * @continuous collision detection using ICCD in 
 * "ICCD: Interactive Continuous Collision Detection between Deformable Models using Connectivity-Based Culling"
 * Tang et al. 2007
 * @author Ke Quan
 * 
 * This file is part of Physika, a versatile physics simulation library.
 * Copyright (C) 2013 Physika Group.
 *
 * This Source Code Form is subject to the terms of the GNU General Public License v2.0. 
 * If a copy of the GPL was not distributed with this file, you can obtain one at:
 * http://www.gnu.org/licenses/gpl-2.0.html
 *
 */

#include "Physika_Dynamics/Collidable_Objects/collision_detection_method_CCD.h"

namespace Physika{

using BoundaryMeshInternal::Vertex;
using SurfaceMeshInternal::Face;
using SurfaceMeshInternal::FaceGroup;

template<typename Scalar, int Dim>
std::map<MeshBasedCollidableObject<Scalar> *, std::vector<Vector<Scalar, 3> > > CollisionDetectionMethodCCD<Scalar, Dim>::previous_position_;


template<typename Scalar, int Dim>
CollisionDetectionMethodCCD<Scalar, Dim>::CollisionDetectionMethodCCD()
{

}

template<typename Scalar, int Dim>
CollisionDetectionMethodCCD<Scalar, Dim>::~CollisionDetectionMethodCCD()
{

}

template<typename Scalar, int Dim>
void CollisionDetectionMethodCCD<Scalar, Dim>::update()
{
    scene_bvh_.updateSceneBVH();
}

template<typename Scalar, int Dim>
void CollisionDetectionMethodCCD<Scalar, Dim>::addPreviousPosition(MeshBasedCollidableObject<Scalar>* object)
{

	unsigned int face_number = object->mesh()->numFaces();
	
	std::vector<Vector<Scalar, 3> > object_positions; // to store this object's vertex positions

	for (unsigned int f = 0; f < face_number; ++ f)
	{

		Face<Scalar> &temp_face = object->mesh()->face(f);
		unsigned int vertex_number = temp_face.numVertices();
		for (unsigned int v = 0; v < vertex_number; ++ v)
			object_positions.push_back(object->vertexPosition(temp_face.vertex(v).positionIndex()));

	}
	// put into the map
	previous_position_.insert(std::pair<MeshBasedCollidableObject<Scalar> *, std::vector<Vector<Scalar, 3> > >(object, object_positions));

}

template<typename Scalar, int Dim>
void CollisionDetectionMethodCCD<Scalar, Dim>::addCollidableObject(CollidableObject<Scalar, Dim>* object)
{
    ObjectBVHCCD<Scalar, Dim>* object_bvh = new ObjectBVHCCD<Scalar, Dim>();
    object_bvh->setCollidableObject(object);
    scene_bvh_.addObjectBVH(object_bvh);
    
	// create previous_positon_ entry for the new object
	addPreviousPosition(dynamic_cast<MeshBasedCollidableObject<Scalar> *>(object));

}

template<typename Scalar, int Dim>
bool CollisionDetectionMethodCCD<Scalar, Dim>::collisionDetection()
{

    bool is_collide = scene_bvh_.selfCollide(this->collision_pairs_);
    (this->contact_points_).setCollisionResult(this->collision_pairs_);
	// have to temporarily store all object pointers
	std::vector<MeshBasedCollidableObject<Scalar> *> objectPointers;
	for (std::map<MeshBasedCollidableObject<Scalar> *, std::vector<Vector<Scalar, 3> > >::iterator iter = previous_position_.begin(); iter != previous_position_.end(); ++ iter)
		objectPointers.push_back(iter->first);

	previous_position_.clear();
	// update the previous positions for each object
	for (unsigned int i = 0; i < objectPointers.size(); ++ i)
		addPreviousPosition(objectPointers[i]);

    return is_collide;

}

template<typename Scalar, int Dim>
bool CollisionDetectionMethodCCD<Scalar, Dim>::hasKey(MeshBasedCollidableObject<Scalar> *object)
{

	return (previous_position_.find(object) == previous_position_.end());

}

template<typename Scalar, int Dim>
Vector<Scalar, 3> CollisionDetectionMethodCCD<Scalar, Dim>::getPosition(MeshBasedCollidableObject<Scalar>* object, unsigned int face_index, unsigned int vertex_index)
{

	if (face_index * 3 + vertex_index >= previous_position_[object].size())
		return Vector<Scalar, 3>((Scalar)0.0, (Scalar)0.0, (Scalar)0.0);
	// this is according to how the positions are stored in previous_position_
	return previous_position_[object][face_index * 3 + vertex_index];

}

template<typename Scalar, int Dim>
bool CollisionDetectionMethodCCD<Scalar, Dim>::collide(MeshBasedCollidableObject<Scalar> *object_lhs, MeshBasedCollidableObject<Scalar>*object_rhs, unsigned int face_index_lhs, unsigned int face_index_rhs, Vector<Scalar, 3> &point, Vector<Scalar, 3> &normal)
{
	
	if (previous_position_.size() == 0)
		return false;
	// declare all needed position variables
	Face<Scalar>& face_lhs = object_lhs->mesh()->face(face_index_lhs);
	Vector<Scalar, 3> face_lhs_normal = face_lhs.faceNormal();
	Face<Scalar>& face_rhs = object_rhs->mesh()->face(face_index_rhs);
	Vector<Scalar, 3> face_rhs_normal = face_rhs.faceNormal();
	unsigned int num_vertex_lhs = face_lhs.numVertices();
	unsigned int num_vertex_rhs = face_rhs.numVertices();
	Vector<Scalar, 3>* vertex_lhs = new Vector<Scalar, 3>[num_vertex_lhs];
	Vector<Scalar, 3>* vertex_rhs = new Vector<Scalar, 3>[num_vertex_rhs];
	Vector<Scalar, 3>* previous_vertex_lhs = new Vector<Scalar, 3>[num_vertex_lhs];
	Vector<Scalar, 3>* previous_vertex_rhs = new Vector<Scalar, 3>[num_vertex_rhs];
	// get the values of all needed positions
	for (unsigned int i = 0; i < num_vertex_lhs; ++ i)
	{
		vertex_lhs[i] = object_lhs->vertexPosition(face_lhs.vertex(i).positionIndex());
	}
	for (unsigned int i = 0; i < num_vertex_rhs; ++ i)
	{
		vertex_rhs[i] = object_rhs->vertexPosition(face_rhs.vertex(i).positionIndex());
	}
	for (unsigned int i = 0; i < num_vertex_lhs; ++ i)
	{
		previous_vertex_lhs[i] = getPosition(object_lhs, face_index_lhs, i);
	}
	for (unsigned int i = 0; i < num_vertex_rhs; ++ i)
	{
		previous_vertex_rhs[i] = getPosition(object_rhs, face_index_rhs, i);
	}

	//VF test
	for (unsigned int i = 0; i < num_vertex_lhs; ++ i)
		if (VFTest(vertex_lhs[i], previous_vertex_lhs[i], vertex_rhs, previous_vertex_rhs, point, normal, face_rhs_normal, 0))
			return true;

	for (unsigned int i = 0; i < num_vertex_rhs; ++ i)
		if (VFTest(vertex_rhs[i], previous_vertex_rhs[i], vertex_lhs, previous_vertex_lhs, point, normal, face_lhs_normal, 1))
			return true;

	//EE test
	Vector<Scalar, 3> edge_lhs[2], edge_rhs[2], previous_edge_lhs[2], previous_edge_rhs[2];
	for (unsigned int i = 0; i < num_vertex_lhs; ++ i)
		for (unsigned int j = 0; j < num_vertex_rhs; ++ j)
		{

			unsigned int i1 = i, i2 = (i + 1) % num_vertex_lhs;
			unsigned int j1 = j, j2 = (j + 1) % num_vertex_rhs;

			edge_lhs[0] = vertex_lhs[i1];
			edge_lhs[1] = vertex_lhs[i2];
			edge_rhs[0] = vertex_rhs[j1];
			edge_rhs[1] = vertex_rhs[j2];
			previous_edge_lhs[0] = previous_vertex_lhs[i1];
			previous_edge_lhs[1] = previous_vertex_lhs[i2];
			previous_edge_rhs[0] = previous_vertex_rhs[j1];
			previous_edge_rhs[1] = previous_vertex_rhs[j2];

			if (EETest(edge_lhs, previous_edge_lhs, edge_rhs, previous_edge_rhs, point, normal, face_lhs_normal, face_rhs_normal))
				return true;
		}
	// if no collision detected :
	return false;

}

template<typename Scalar, int Dim>
bool CollisionDetectionMethodCCD<Scalar, Dim>::VFTest(Vector<Scalar, 3> &vertex, Vector<Scalar, 3> &vertex_previous, Vector<Scalar, 3> *face_vertices, Vector<Scalar, 3> *face_vertices_previous, Vector<Scalar, 3> &point, Vector<Scalar, 3> &normal, Vector<Scalar, 3> &face_normal, int tag)
{
	
	Vector<Scalar, 3> face_vertex_velocity[3];//velocities of the 3 vertices of the face (A, B, C)
	for (unsigned int i = 0; i < 3; ++ i)
		face_vertex_velocity[i] = face_vertices[i] - face_vertices_previous[i];
	Vector<Scalar, 3> vertex_velocity = vertex - vertex_previous;//velocity of the single vertex (P)

	Vector<Scalar, 3> vertices[6];//parameter for the vector equation
	Vector<Scalar, 3> velocities[6];//parameter for the vector equation

	vertices[0] = face_vertices_previous[0];
	vertices[1] = vertex_previous;
	vertices[2] = face_vertices_previous[0];
	vertices[3] = face_vertices_previous[1];
	vertices[4] = face_vertices_previous[0];
	vertices[5] = face_vertices_previous[2];

	velocities[0] = face_vertex_velocity[0];
	velocities[1] = vertex_velocity;
	velocities[2] = face_vertex_velocity[0];
	velocities[3] = face_vertex_velocity[1];
	velocities[4] = face_vertex_velocity[0];
	velocities[5] = face_vertex_velocity[2];
	// compute the collision time t
	Scalar t = solveVectorEquation(vertices, velocities);

	// check whether the collision time is within this frame
	if (t < 0.0 || t > 1.0)
		return false;


	Vector<Scalar, 3> A[3];
	for (unsigned int i = 0; i < 3; ++ i)
		A[i] = face_vertices_previous[i] + face_vertex_velocity[i] * t;
	Vector<Scalar, 3> P = vertex_previous + vertex_velocity * t; // P is the collision point
	// face ABC, base vectors are AB and AC
	Vector<Scalar, 3> AB = A[1] - A[0], AC = A[2] - A[0], AP = P - A[0];
	Vector<Scalar, 3> n = AB.cross(AC);
	Vector<Scalar, 3> n1 = n.cross(AB), n2 = n.cross(AC);
	Scalar u = AP.dot(n2) / AB.dot(n2), v = AP.dot(n1) / AC.dot(n1);
	// check whether the collision point is within the face
	if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0 || u + v > 1.0) // AP = u * AB + v * AC
		return false;

	//set point and normal
	point = P;
	normal = face_normal.normalize();
	if (tag == 0)
		normal = -normal;

	return true;

}

template<typename Scalar, int Dim>
bool CollisionDetectionMethodCCD<Scalar, Dim>::EETest(Vector<Scalar, 3> *edge1_vertices, Vector<Scalar, 3> *edge1_vertices_previous, Vector<Scalar, 3> *edge2_vertices, Vector<Scalar, 3> *edge2_vertices_previous, Vector<Scalar, 3> &point, Vector<Scalar, 3> &normal, Vector<Scalar, 3> &face_lhs_normal, Vector<Scalar, 3> &face_rhs_normal)
{
	
	Vector<Scalar, 3> edge1_velocity[2], edge2_velocity[2];
	for (unsigned int i = 0; i < 2; ++ i)
	{
		edge1_velocity[i] = edge1_vertices[i] - edge1_vertices_previous[i];
		edge2_velocity[i] = edge2_vertices[i] - edge2_vertices_previous[i];
	}

	Vector<Scalar, 3> vertices[6];
	Vector<Scalar, 3> velocities[6];

	vertices[0] = edge1_vertices_previous[0];
	vertices[1] = edge2_vertices_previous[0];
	vertices[2] = edge1_vertices_previous[0];
	vertices[3] = edge1_vertices_previous[1];
	vertices[4] = edge2_vertices_previous[0];
	vertices[5] = edge2_vertices_previous[1];

	velocities[0] = edge1_velocity[0];
	velocities[1] = edge2_velocity[0];
	velocities[2] = edge1_velocity[0];
	velocities[3] = edge1_velocity[1];
	velocities[4] = edge2_velocity[0];
	velocities[5] = edge2_velocity[1];
	// compute the collision time t
	Scalar t = solveVectorEquation(vertices, velocities);
	// check whether the collision time is within this frame
	if (t < 0.0 || t > 1.0)
		return false;
	// edge AB and edge CD
	Vector<Scalar, 3> A = edge1_vertices_previous[0] + edge1_velocity[0] * t;
	Vector<Scalar, 3> B = edge1_vertices_previous[1] + edge1_velocity[1] * t;
	Vector<Scalar, 3> C = edge2_vertices_previous[0] + edge2_velocity[0] * t;
	Vector<Scalar, 3> D = edge2_vertices_previous[1] + edge2_velocity[1] * t;
	Vector<Scalar, 3> AB = B - A, DC = C - D, AC = C - A;
	Vector<Scalar, 3> n = AB.cross(DC);
	Vector<Scalar, 3> n1 = n.cross(AB), n2 = n.cross(DC);
	Scalar u = AC.dot(n2) / AB.dot(n2), v = AC.dot(n1) / DC.dot(n1);
	// check whether the collision point is within the 2 edges
	if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0) // u is the ratio of collision point on AB, v the ratio of collision point on CD
		return false;
	if (n.norm() == 0.0)
		return false;

	point = AB * u + A;

	if (n.dot(face_lhs_normal) < 0.0)
		n = -n;
	normal = n.normalize();

	return true;

}

template<typename Scalar, int Dim>
Scalar CollisionDetectionMethodCCD<Scalar, Dim>::solveVectorEquation(Vector<Scalar, 3> *vertices, Vector<Scalar, 3> *velocities)
{

	Vector<Scalar, 3> s1 = vertices[3] - vertices[2];
	Vector<Scalar, 3> s2 = vertices[5] - vertices[4];
	Vector<Scalar, 3> v1 = velocities[3] - velocities[2];
	Vector<Scalar, 3> v2 = velocities[5] - velocities[4];

	Vector<Scalar, 3> j0 = s1.cross(s2);
	Vector<Scalar, 3> j1 = s1.cross(v2) + v1.cross(s2);
	Vector<Scalar, 3> j2 = v1.cross(v2);

	Vector<Scalar, 3> i0 = vertices[1] - vertices[0];
	Vector<Scalar, 3> i1 = velocities[1] - velocities[0];

	Scalar r0 = i0.dot(j0);
	Scalar r1 = i0.dot(j1) + i1.dot(j0);
	Scalar r2 = i0.dot(j2) + i1.dot(j1);
	Scalar r3 = i1.dot(j2);

	return solveCubicEquation(r3, r2, r1, r0, 100);

}

template<typename Scalar, int Dim>
Scalar CollisionDetectionMethodCCD<Scalar, Dim>::solveCubicEquation(Scalar a, Scalar b, Scalar c, Scalar d, unsigned int iteration)
{

	Scalar value_0 = cubicValue(a, b, c, d, 0.0);
	Scalar value_1 = cubicValue(a, b, c, d, 1.0);

	if (value_0 * value_1 > 0.0) // no solution within this frame
		return -1.0;

	bool direction = (value_0 > value_1); // whether go on looking for the left half or the right half during bisection

	return solveCubicEquationDevide(a, b, c, d, 0.0, 1.0, direction, 0);

}

template<typename Scalar, int Dim>
Scalar CollisionDetectionMethodCCD<Scalar, Dim>::solveCubicEquationDevide(Scalar a, Scalar b, Scalar c, Scalar d, Scalar minRange, Scalar maxRange, bool direction, unsigned int depth)
{

	Scalar x = (minRange + maxRange) / 2.0; // bisection

	if (depth > 100)
		return x;

	Scalar f = cubicValue(a, b, c, d, x);

	if (f == 0.0)
		return x;
	if (f > 0.0)
	{
		if (direction)
			return solveCubicEquationDevide(a, b, c, d, x, maxRange, direction, depth + 1);
		else
			return solveCubicEquationDevide(a, b, c, d, minRange, x, direction, depth + 1);
	}
	else
	{
		if (direction)
			return solveCubicEquationDevide(a, b, c, d, minRange, x, direction, depth + 1);
		else
			return solveCubicEquationDevide(a, b, c, d, x, maxRange, direction, depth + 1);
	}

}


template<typename Scalar, int Dim>
Scalar CollisionDetectionMethodCCD<Scalar, Dim>::cubicValue(Scalar a, Scalar b, Scalar c, Scalar d, Scalar x)
{
	return a * x * x * x + b * x * x + c * x + d;
}

//template class CollisionDetectionMethodCCD<float, 2>;
//template class CollisionDetectionMethodCCD<double, 2>;
template class CollisionDetectionMethodCCD<float, 3>;
template class CollisionDetectionMethodCCD<double, 3>;


}