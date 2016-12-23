/*
 * @file  contact_point_manager.cpp
 * @manager of contact points in rigid body simulation
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

#include <iostream>
#include <limits>
#include "Physika_Dynamics/Collidable_Objects/mesh_based_collidable_object.h"
#include "Physika_Dynamics/Collidable_Objects/collision_pair.h"
#include "Physika_Dynamics/Collidable_Objects/collision_pair_manager.h"
#include "Physika_Dynamics/Collidable_Objects/contact_point.h"
#include "Physika_Dynamics/Collidable_Objects/contact_point_manager.h"
#include "Physika_Core/Transform/transform_3d.h"

namespace Physika{

using BoundaryMeshInternal::Vertex;
using SurfaceMeshInternal::Face;
using SurfaceMeshInternal::FaceGroup;

template <typename Scalar,int Dim>
ContactPointManager<Scalar, Dim>::ContactPointManager()
{

}

template <typename Scalar,int Dim>
ContactPointManager<Scalar, Dim>::~ContactPointManager()
{
    cleanContactPoints();
}

template <typename Scalar,int Dim>
void ContactPointManager<Scalar, Dim>::setCollisionResult(CollisionPairManager<Scalar, Dim>& collision_result)
{
    unsigned int num_collision = collision_result.numberCollision();
    CollisionPairBase<Scalar, Dim>* collision_pair;
    for(unsigned int i = 0; i < num_collision; ++i)
    {
        collision_pair = collision_result.collisionPairs()[i];
        if(collision_pair->objectTypeLhs() == CollidableObjectInternal::MESH_BASED && collision_pair->objectTypeRhs() == CollidableObjectInternal::MESH_BASED)
        {
            getMeshContactPoint(dynamic_cast<CollisionPairMeshToMesh<Scalar>*>(collision_pair));
        }
        else
        {
            std::cerr<<"Wrong object type of collision pair: should be MESH_BASED!"<<std::endl;
            return;
        }
    }
}

template <typename Scalar,int Dim>
unsigned int ContactPointManager<Scalar, Dim>::numContactPoint() const
{
    return static_cast<unsigned int>(contact_points_.size());
}

template <typename Scalar,int Dim>
ContactPoint<Scalar, Dim>* ContactPointManager<Scalar, Dim>::contactPoint(unsigned int contact_index)
{
    if(contact_index >= numContactPoint())
    {
        std::cerr<<"Contact index our of range!"<<std::endl;
        return NULL;
    }
    return contact_points_[contact_index];
}

template <typename Scalar,int Dim>
const std::vector<ContactPoint<Scalar, Dim>* >& ContactPointManager<Scalar, Dim>::contactPoints() const
{
    return contact_points_;
}

template <typename Scalar,int Dim>
std::vector<ContactPoint<Scalar, Dim>* >& ContactPointManager<Scalar, Dim>::contactPoints()
{
    return contact_points_;
}

template <typename Scalar,int Dim>
ContactPoint<Scalar, Dim>* ContactPointManager<Scalar, Dim>::operator[] (unsigned int contact_index)
{
    if(contact_index >= numContactPoint())
    {
        std::cerr<<"Contact index our of range!"<<std::endl;
        return NULL;
    }
    return contact_points_[contact_index];
}

template <typename Scalar,int Dim>
void ContactPointManager<Scalar, Dim>::addContactPoint(ContactPoint<Scalar, Dim> *point_)
{
	contact_points_.push_back(point_);
}

template <typename Scalar,int Dim>
void ContactPointManager<Scalar, Dim>::cleanContactPoints()
{
    unsigned int num_contact = numContactPoint();
    for(unsigned int i = 0; i < num_contact; ++i)
    {
        delete contact_points_[i];
    }
    contact_points_.clear();
}

template <typename Scalar>
Scalar min(Scalar x1, Scalar x2)
{
	if (x1 < x2)
		return x1;
	return x2;
}

template <typename Scalar, int Dim>
Scalar bend(const Vector<Scalar, Dim> &overlap_point, const Vector<Scalar, Dim> &face_normal, Vector<Scalar, Dim> *vertex_positions, Vector<Scalar, Dim> *vertex_normals)
{
	Scalar weights[Dim], weights_sum = 0;
	Scalar answer = 0;
	for (unsigned int i = 0; i < Dim; ++ i)
	{
		Vector<Scalar, Dim> temp = overlap_point - vertex_positions[i];
		weights[i] = 1 / (temp.norm() + 1);
		weights_sum += weights[i];
	}
	for (unsigned int i = 0; i < Dim; ++ i)
	{
		answer += (face_normal.dot(vertex_normals[i]) * weights[i] / weights_sum);
	}
	Vector<Scalar, Dim> temp = (vertex_positions[1] - vertex_positions[0]).cross(vertex_positions[2] - vertex_positions[0]);
	answer *= temp.norm();
	return answer;
}

template <typename Scalar,int Dim>
void ContactPointManager<Scalar, Dim>::getMeshContactPoint(CollisionPairMeshToMesh<Scalar>* collision_pair)
{
    if(Dim == 2)
    {
        std::cerr<<"Can't convert 2D collision!"<<std::endl;
        return;
    }

    if(collision_pair == NULL)
    {
        std::cerr<<"Null collision pair!"<<std::endl;
        return;
    }

	unsigned int object_index_lhs = collision_pair->objectLhsIdx();
	unsigned int object_index_rhs = collision_pair->objectRhsIdx();

    Face<Scalar>* face_lhs = collision_pair->faceLhsPtr();
    Face<Scalar>* face_rhs = collision_pair->faceRhsPtr();
    unsigned int num_vertex_lhs = face_lhs->numVertices();
    unsigned int num_vertex_rhs = face_rhs->numVertices();
    Vector<Scalar, 3>* vertex_lhs = new Vector<Scalar, 3>[num_vertex_lhs];
    Vector<Scalar, 3>* vertex_rhs = new Vector<Scalar, 3>[num_vertex_rhs];
	Vector<Scalar,3> mesh_rhs_face_normal = collision_pair->meshObjectRhs()->faceNormal(collision_pair->faceRhsIdx());
	Vector<Scalar,3> mesh_lhs_face_normal = collision_pair->meshObjectLhs()->faceNormal(collision_pair->faceLhsIdx());
	Vector<Scalar, 3> *vertex_normal_lhs = new Vector<Scalar, 3>[num_vertex_lhs];
	Vector<Scalar, 3> *vertex_normal_rhs = new Vector<Scalar, 3>[num_vertex_rhs];
	Scalar bend_lhs = 0, bend_rhs = 0; // curvature. large value -> flat
	Transform<Scalar, 3>* tran_lhs = collision_pair->meshObjectLhs()->transform();
	Transform<Scalar, 3>* tran_rhs = collision_pair->meshObjectRhs()->transform();
	
    for(unsigned int i = 0; i < num_vertex_lhs; i++)
    {
        vertex_lhs[i] = collision_pair->meshObjectLhs()->vertexPosition(face_lhs->vertex(i).positionIndex());
		unsigned int tempNormalIndex = collision_pair->faceLhsPtr()->vertexPtr(i)->normalIndex();
		vertex_normal_lhs[i] = tran_lhs->rotate(collision_pair->meshObjectLhs()->mesh()->vertexNormal(tempNormalIndex));
    }
	bend_lhs /= Scalar(num_vertex_lhs);
    for(unsigned int i = 0; i < num_vertex_rhs; i++)
    {
        vertex_rhs[i] = collision_pair->meshObjectRhs()->vertexPosition(face_rhs->vertex(i).positionIndex());
		unsigned int tempNormalIndex = collision_pair->faceRhsPtr()->vertexPtr(i)->normalIndex();
		vertex_normal_rhs[i] = tran_rhs->rotate(collision_pair->meshObjectRhs()->mesh()->vertexNormal(tempNormalIndex));
	}
	bend_rhs /= Scalar(num_vertex_rhs);
	/*
	if ((bend_lhs > bend_rhs) == (object_index_lhs == 1))
	{
		std::cout << object_index_lhs << ' ' << object_index_rhs << ' ' << bend_lhs << ' ' << bend_rhs << '\n';
		getchar();
	}
	*/
	
    unsigned int num_overlap = 0;
    bool is_lhs_tri = (num_vertex_lhs == 3);
    bool is_rhs_tri = (num_vertex_rhs == 3);
    bool is_lhs_quad = (num_vertex_lhs == 4);
    bool is_rhs_quad = (num_vertex_rhs == 4);
    Vector<Scalar, 3> overlap_point(0);
    Vector<Scalar, 3> temp_overlap_point(0);
    Vector<Scalar, 3> contact_normal_lhs(0);

    //test each edge of lhs with the face of rhs
	if (object_index_lhs != 0){
    for(unsigned int i = 0; i < num_vertex_lhs; i++)
    {
        if(is_rhs_tri)//triangle
        {
			bool tempInside = inside_relation_[object_index_rhs][object_index_lhs];
			if(tempInside != true && MeshBasedCollidableObject<Scalar>::overlapEdgeTriangle(vertex_lhs[i], vertex_lhs[(i + 1)%num_vertex_lhs], vertex_rhs[0], vertex_rhs[1], vertex_rhs[2], mesh_rhs_face_normal, temp_overlap_point))
            {
				Vector<Scalar, 3> tempNormal = mesh_rhs_face_normal * (-1);
				if (object_index_rhs == 0)
					tempNormal = Vector<Scalar, 3>(0, 1, 0);
				bend_lhs = bend(temp_overlap_point, mesh_lhs_face_normal, vertex_lhs, vertex_normal_lhs);
				bend_rhs = bend(temp_overlap_point, mesh_rhs_face_normal, vertex_rhs, vertex_normal_rhs);
				//if (bend_lhs >= bend_rhs)
				//	tempNormal = mesh_lhs_face_normal;
				//std::cout << "Ledge x Rface, " << object_index_lhs << ' ' << object_index_rhs << ' ' << bend_lhs << ' ' << bend_rhs << ' ' << mesh_lhs_face_normal << ' ' << mesh_rhs_face_normal << ' ' << tempNormal << '\n';
				ContactPoint<Scalar, Dim>* contact_point = new ContactPoint<Scalar, Dim>(numContactPoint(), collision_pair->objectLhsIdx(), collision_pair->objectRhsIdx(),
                    *dynamic_cast<Vector<Scalar, Dim>*>(&temp_overlap_point), 
                    *dynamic_cast<Vector<Scalar, Dim>*>(&tempNormal));
                contact_points_.push_back(contact_point);
                //num_overlap++;
                //overlap_point += temp_overlap_point;
            }
        }
        if(is_rhs_quad)//quad
        {
            if(MeshBasedCollidableObject<Scalar>::overlapEdgeQuad(vertex_lhs[i], vertex_lhs[(i + 1)%num_vertex_lhs], vertex_rhs[0], vertex_rhs[1], vertex_rhs[2], vertex_rhs[3], temp_overlap_point))
            {
                //num_overlap++;
                //overlap_point += temp_overlap_point;
            }
        }
	}}

    //test each edge of rhs with the face of lhs
	if (object_index_rhs != 0) {
    for(unsigned int i = 0; i < num_vertex_rhs; i++)
    {
        if(is_lhs_tri)//triangle
        {
			bool tempInside = inside_relation_[object_index_lhs][object_index_rhs];
			if(tempInside != true && MeshBasedCollidableObject<Scalar>::overlapEdgeTriangle(vertex_rhs[i], vertex_rhs[(i + 1)%num_vertex_rhs], vertex_lhs[0], vertex_lhs[1], vertex_lhs[2], mesh_lhs_face_normal, temp_overlap_point))
            {
				Vector<Scalar, 3> tempNormal = mesh_lhs_face_normal * (-1);
				if (object_index_lhs == 0)
					tempNormal = Vector<Scalar, 3>(0, -1, 0);
				bend_lhs = bend(temp_overlap_point, mesh_lhs_face_normal, vertex_lhs, vertex_normal_lhs);
				bend_rhs = bend(temp_overlap_point, mesh_rhs_face_normal, vertex_rhs, vertex_normal_rhs);
				//if (bend_rhs >= bend_lhs)
				//	tempNormal = mesh_rhs_face_normal;
				//std::cout << "Redge x Lface, " << object_index_lhs << ' ' << object_index_rhs << ' ' << bend_lhs << ' ' << bend_rhs << ' ' << tempNormal << '\n';
				ContactPoint<Scalar, Dim>* contact_point = new ContactPoint<Scalar, Dim>(numContactPoint(), collision_pair->objectRhsIdx(), collision_pair->objectLhsIdx(),
                    *dynamic_cast<Vector<Scalar, Dim>*>(&temp_overlap_point), 
                    *dynamic_cast<Vector<Scalar, Dim>*>(&tempNormal));
                contact_points_.push_back(contact_point);
				
                //num_overlap++;
                //overlap_point += temp_overlap_point;
            }
        }
        if(is_lhs_quad)//quad
        {
            if(MeshBasedCollidableObject<Scalar>::overlapEdgeQuad(vertex_rhs[i], vertex_rhs[(i + 1)%num_vertex_rhs], vertex_lhs[0], vertex_lhs[1], vertex_lhs[2], vertex_lhs[3], temp_overlap_point))
            {
                //num_overlap++;
                //overlap_point += temp_overlap_point;
            }
        }
	}}

	//getchar();

	/*
    //generate position and normal
    if(num_overlap > 0)
    {
        overlap_point /= static_cast<Scalar>(num_overlap);
        //contact_normal_lhs = (collision_pair->meshObjectLhs()->faceNormal(collision_pair->faceLhsIdx()) - 
        //    collision_pair->meshObjectRhs()->faceNormal(collision_pair->faceRhsIdx())).normalize();
        contact_normal_lhs = -collision_pair->meshObjectRhs()->faceNormal(collision_pair->faceRhsIdx());
        if(contact_normal_lhs.norm() > std::numeric_limits<Scalar>::epsilon())
        {
            contact_normal_lhs.normalize();
            ContactPoint<Scalar, Dim>* contact_point = new ContactPoint<Scalar, Dim>(numContactPoint(), collision_pair->objectLhsIdx(), collision_pair->objectRhsIdx(),
                *dynamic_cast<Vector<Scalar, Dim>*>(&overlap_point), 
                *dynamic_cast<Vector<Scalar, Dim>*>(&contact_normal_lhs));
            contact_points_.push_back(contact_point);
        }
    }
	*/

    delete[] vertex_lhs;
    delete[] vertex_rhs;
}


template class ContactPointManager<float, 2>;
template class ContactPointManager<double, 2>;
template class ContactPointManager<float, 3>;
template class ContactPointManager<double, 3>;

}
