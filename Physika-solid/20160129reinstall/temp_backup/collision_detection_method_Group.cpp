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

#include <iostream>

#include "Physika_Dynamics/Collidable_Objects/collision_detection_method_Group.h"
#include "Physika_Dynamics/Collidable_Objects/mesh_based_collidable_object.h"
#include "Physika_Dynamics/Collidable_Objects/contact_point.h"
#include "Physika_Dynamics/Collidable_Objects/contact_point_manager.h"

namespace Physika{

using BoundaryMeshInternal::Vertex;
using SurfaceMeshInternal::Face;
using SurfaceMeshInternal::FaceGroup;

template <typename Scalar, int Dim>
Segment<Scalar, Dim>::Segment()
{

}

template <typename Scalar, int Dim>
Segment<Scalar, Dim>::Segment(const Vector<Scalar, Dim> &point1, const Vector<Scalar, Dim> &point2)
{
	point1_ = point1;
	point2_ = point2;
}

template <typename Scalar, int Dim>
Segment<Scalar, Dim>::~Segment()
{

}

template <typename Scalar, int Dim>
bool Segment<Scalar, Dim>::common(const Segment<Scalar, Dim> &otherSegment) const
{
	//Scalar epsl = std::numeric_limits<Scalar>::epsilon();
	Scalar epsl = 0.0001;
	if ((point1_ - otherSegment.point1_).norm() < epsl)
		return true;
	if ((point1_ - otherSegment.point2_).norm() < epsl)
		return true;
	if ((point2_ - otherSegment.point1_).norm() < epsl)
		return true;
	if ((point2_ - otherSegment.point2_).norm() < epsl)
		return true;
	return false;
}

template <typename Scalar, int Dim>
Vector<Scalar, Dim> Segment<Scalar, Dim>::point1() const
{
	return point1_;
}

template <typename Scalar, int Dim>
Vector<Scalar, Dim> Segment<Scalar, Dim>::point2() const
{
	return point2_;
}

template <typename Scalar, int Dim>
CollisionDetectionMethodGroup<Scalar, Dim>::CollisionDetectionMethodGroup()
{

}

template <typename Scalar, int Dim>
CollisionDetectionMethodGroup<Scalar, Dim>::~CollisionDetectionMethodGroup()
{

}

template <typename Scalar, int Dim>
void CollisionDetectionMethodGroup<Scalar, Dim>::update()
{
	scene_bvh_.updateSceneBVH();
}

template <typename Scalar, int Dim>
void CollisionDetectionMethodGroup<Scalar, Dim>::addCollidableObject(CollidableObject<Scalar, Dim> *object)
{
	ObjectBVH<Scalar, Dim> *object_bvh = new ObjectBVH<Scalar, Dim>();
	object_bvh->setCollidableObject(object);
	scene_bvh_.addObjectBVH(object_bvh);
}

template <typename Scalar, int Dim>
bool CollisionDetectionMethodGroup<Scalar, Dim>::collisionDetection()
{
	segments_.clear();
	object_lhs_index_.clear();
	object_rhs_index_.clear();
	object_lhs_face_normal_.clear();
	bool is_collide = scene_bvh_.selfCollide(this->collision_pairs_);
	/*
	if (! is_collide)
	{
		std::cout << "is_collide = false" << '\n';
		std::cout << collision_pairs_.numberCollision() << '\n';
		getchar();
		return false;
	}
	*/
	// get loop from contactPairManager
	for (unsigned int i = 0; i < collision_pairs_.numberCollision(); ++ i)
		getSegment(collision_pairs_.collisionPairs()[i]);
	if (segments_.size() > 0)
	{
		getLoop();
		setNormal();
		return true;
	}
	return false;
}

template <typename Scalar, int Dim>
void CollisionDetectionMethodGroup<Scalar, Dim>::getSegment(CollisionPairBase<Scalar, Dim> *pair)
{
	if (pair->objectTypeLhs() == CollidableObjectInternal::MESH_BASED && pair->objectTypeRhs() == CollidableObjectInternal::MESH_BASED)
	{
		CollisionPairMeshToMesh<Scalar> *collision_pair = dynamic_cast<CollisionPairMeshToMesh<Scalar>*>(pair);
		Face<Scalar>* face_lhs = collision_pair->faceLhsPtr();
		Face<Scalar>* face_rhs = collision_pair->faceRhsPtr();
		unsigned int num_vertex_lhs = face_lhs->numVertices();
		unsigned int num_vertex_rhs = face_rhs->numVertices();
		Vector<Scalar, 3>* vertex_lhs = new Vector<Scalar, 3>[num_vertex_lhs];
		Vector<Scalar, 3>* vertex_rhs = new Vector<Scalar, 3>[num_vertex_rhs];
		Vector<Scalar, 3> mesh_rhs_face_normal = collision_pair->meshObjectRhs()->faceNormal(collision_pair->faceRhsIdx());
		Vector<Scalar, 3> mesh_lhs_face_normal = collision_pair->meshObjectLhs()->faceNormal(collision_pair->faceLhsIdx());
		for (unsigned int i = 0; i < num_vertex_lhs; ++ i)
			vertex_lhs[i] = collision_pair->meshObjectLhs()->vertexPosition(face_lhs->vertex(i).positionIndex());
		for (unsigned int i = 0; i < num_vertex_rhs; ++ i)
			vertex_rhs[i] = collision_pair->meshObjectRhs()->vertexPosition(face_rhs->vertex(i).positionIndex());
		Vector<Scalar, 3> overlap_points[2];
		int overlap_count = 0;
		// lhs's edge xxxx rhs face
		for (unsigned int i = 0; i < num_vertex_lhs; ++ i)
			if (MeshBasedCollidableObject<Scalar>::overlapEdgeTriangle(vertex_lhs[i], vertex_lhs[(i + 1) % num_vertex_lhs], vertex_rhs[0], vertex_rhs[1], vertex_rhs[2], mesh_rhs_face_normal, overlap_points[overlap_count]))
			{
				overlap_count += 1;
			}
		// rhs's edge xxxx lhs's face
		for (unsigned int i = 0; i < num_vertex_rhs; ++ i)
			if (MeshBasedCollidableObject<Scalar>::overlapEdgeTriangle(vertex_rhs[i], vertex_rhs[(i + 1) % num_vertex_rhs], vertex_lhs[0], vertex_lhs[1], vertex_lhs[2], mesh_lhs_face_normal, overlap_points[overlap_count]))
			{
				overlap_count += 1;
			}
		if (overlap_count != 2)
		{
			std::cout << "2 triangles overlap not 2 points !\n";
			return;
		}
		Segment<Scalar, Dim> *temp_segment = new Segment<Scalar, Dim>(overlap_points[0], overlap_points[1]);
		segments_.push_back(temp_segment);
		object_lhs_index_.push_back(collision_pair->objectLhsIdx());
		object_rhs_index_.push_back(collision_pair->objectRhsIdx());
		object_lhs_face_normal_.push_back(mesh_lhs_face_normal);
	}
}

template <typename Scalar, int Dim>
void CollisionDetectionMethodGroup<Scalar, Dim>::getLoop()
{
	const unsigned int num_segments = segments_.size();
	std::vector<std::vector<bool> > adjacent(num_segments);
	for (unsigned int i = 0; i < num_segments; ++ i)
		adjacent[i].resize(num_segments);
	belong_.clear();
	belong_.resize(num_segments);
	for (unsigned int i = 0; i < num_segments; ++ i)
		for (unsigned int j = i + 1; j < num_segments; ++ j)
		{
			adjacent[i][j] = segments_[i]->common(*segments_[j]);
			adjacent[j][i] = adjacent[i][j];
		}
	/*
	std::cout << segments_[0]->common(*segments_[1]) << ' ' << segments_[0]->point1() << ' ' << segments_[1]->point2() << ' ' << (segments_[0]->point1() - segments_[1]->point2()).norm() << '\n';
	for (unsigned int i = 0; i < num_segments; ++ i)
	{
		for (unsigned int j = 0; j < num_segments; ++ j)
			std::cout << adjacent[i][j] << ' ';
		std::cout << '\n';
	}
	*/
	// naive algorithm
	num_loops_ = 0;
	for (unsigned int i = 0; i < num_segments; ++ i)
		belong_[i] = -1;
	for (unsigned int i = 0; i < num_segments; ++ i)
	{
		if (belong_[i] != -1)
			continue;
		std::vector<unsigned int> segment_queue;
		std::vector<bool> in_queue(num_segments, false);
		segment_queue.push_back(i);
		in_queue[i] = true;
		for (unsigned int p = 0; p < segment_queue.size(); ++ p)
		{
			for (unsigned int temp = 0; temp < num_segments; ++ temp)
				if ((! in_queue[temp]) && adjacent[segment_queue[p]][temp])
				{
					segment_queue.push_back(temp);
					in_queue[temp] = true;
				}
			//in_queue[segment_queue[p]] = false;
		}
		for (unsigned int p = 0; p < segment_queue.size(); ++ p)
			belong_[segment_queue[p]] = num_loops_;
		num_loops_ += 1;
	}
	/*
	for (unsigned int i = 0; i < num_segments; ++ i)
		std::cout << i << ' ' << segments_[i]->point1() << ' ' << segments_[i]->point2() << ' ' << belong_[i] << '\n';
	getchar();
	*/
}

template <typename Scalar, int Dim>
Scalar CollisionDetectionMethodGroup<Scalar, Dim>::det3(Scalar a11, Scalar a12, Scalar a13, Scalar a21, Scalar a22, Scalar a23, Scalar a31, Scalar a32, Scalar a33)
{
	return (a11 * a22 * a33 + a12 * a23 * a31 + a13 * a21 * a32 - a11 * a23 * a32 - a12 * a21 * a33 - a13 * a22 * a31);
}

template <typename Scalar, int Dim>
bool CollisionDetectionMethodGroup<Scalar, Dim>::solve31Equation(Scalar a11, Scalar a12, Scalar a13, Scalar a21, Scalar a22, Scalar a23, Scalar a31, Scalar a32, Scalar a33, Scalar b1, Scalar b2, Scalar b3, Scalar &ans_x, Scalar &ans_y, Scalar &ans_z)
{
	Scalar D = det3(a11, a12, a13, a21, a22, a23, a31, a32, a33);
	if (abs(D) <= std::numeric_limits<Scalar>::epsilon())
		return false;
	Scalar X = det3(b1, a12, a13, b2, a22, a23, b3, a32, a33);
	Scalar Y = det3(a11, b1, a13, a21, b2, a23, a31, b3, a33);
	Scalar Z = det3(a11, a12, b1, a21, a22, b2, a31, a32, b3);
	ans_x = X / D;
	ans_y = Y / D;
	ans_z = Z / D;
	return true;
}

template <typename Scalar, int Dim>
void CollisionDetectionMethodGroup<Scalar, Dim>::setNormal()
{
	std::vector<Scalar> x_sum(num_loops_), y_sum(num_loops_), z_sum(num_loops_), x2_sum(num_loops_), y2_sum(num_loops_), xy_sum(num_loops_), xz_sum(num_loops_), yz_sum(num_loops_);
	std::vector<int> loop_segment_count(num_loops_, 0);
	for (unsigned int i = 0; i < segments_.size(); ++ i)
	{
		int temp_belong = belong_[i];
		Vector<Scalar, Dim> temp_points[2];
		temp_points[0] = segments_[i]->point1();
		temp_points[0] = segments_[i]->point2();
		for (unsigned int j = 0; j < 2; ++ j)
		{
			loop_segment_count[temp_belong] += 1;
			Scalar temp_x, temp_y, temp_z;
			temp_x = temp_points[j][0];
			temp_y = temp_points[j][1];
			temp_z = temp_points[j][2];
			x_sum[temp_belong] += temp_x;
			y_sum[temp_belong] += temp_y;
			z_sum[temp_belong] += temp_z;
			x2_sum[temp_belong] += (temp_x * temp_x);
			y2_sum[temp_belong] += (temp_y * temp_y);
			xy_sum[temp_belong] += (temp_x * temp_y);
			xz_sum[temp_belong] += (temp_x * temp_z);
			yz_sum[temp_belong] += (temp_y * temp_z);
		}
	}
	std::vector<Vector<Scalar, Dim> > loop_normals;
	for (unsigned int i = 0; i < num_loops_; ++ i)
	{
		Vector<Scalar, Dim> temp_normal;
		// z = A * x + B * y + C
		Scalar A, B, C;
		if (! solve31Equation(x2_sum[i], xy_sum[i], x_sum[i], xy_sum[i], y2_sum[i], y_sum[i], x_sum[i], y_sum[i], (Scalar)(loop_segment_count[i]), xz_sum[i], yz_sum[i], z_sum[i], A, B, C))
		{
			temp_normal = Vector<Scalar, Dim>(0, 1, 0);
		}
		else
		{
			temp_normal = Vector<Scalar, Dim>(A, B, -1);
			temp_normal = temp_normal.normalize();
		}
		loop_normals.push_back(temp_normal);
	}
	for (unsigned int i = 0; i < segments_.size(); ++ i)
	{
		Vector<Scalar, Dim> point1 = segments_[i]->point1();
		Vector<Scalar, Dim> point2 = segments_[i]->point2();
		Vector<Scalar, Dim> mid((point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2, (point1[2] + point2[2]) / 2);
		Vector<Scalar, Dim> temp_normal = loop_normals[belong_[i]];
		//std::cout << "lhs face normal = " << object_lhs_face_normal_[i] << '\n';
		if (temp_normal.dot(object_lhs_face_normal_[i]) < 0)
			temp_normal = -temp_normal;
		ContactPoint<Scalar, Dim> *temp_contact_point = new ContactPoint<Scalar, Dim>(contact_points_.numContactPoint(), object_lhs_index_[i], object_rhs_index_[i], mid, temp_normal);
		//std::cout << object_lhs_index_[i] << ' ' << object_rhs_index_[i] << ' ' << point1 << ' ' << point2 << ' ' << belong_[i] << '\n';
		//std::cout << temp_normal << '\n';
		contact_points_.addContactPoint(temp_contact_point);
	}
	//getchar();
}

//template class CollisionDetectionMethodGroup<float, 2>;
//template class CollisionDetectionMethodGroup<double, 2>;
template class CollisionDetectionMethodGroup<float, 3>;
template class CollisionDetectionMethodGroup<double, 3>;

//template class Segment<float, 2>;
template class Segment<float, 3>;
//template class Segment<double, 2>;
template class Segment<double, 3>;

}
