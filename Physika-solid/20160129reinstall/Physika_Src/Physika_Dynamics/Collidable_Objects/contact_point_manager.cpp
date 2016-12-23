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

#include <limits>
#include "Physika_Dynamics/Collidable_Objects/mesh_based_collidable_object.h"
#include "Physika_Dynamics/Collidable_Objects/collision_pair.h"
#include "Physika_Dynamics/Collidable_Objects/collision_pair_ccd.h"
#include "Physika_Dynamics/Collidable_Objects/collision_pair_manager.h"
#include "Physika_Dynamics/Collidable_Objects/contact_point.h"
#include "Physika_Dynamics/Collidable_Objects/contact_point_manager.h"

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
            // use cast to determine the collision method is CCD or DTBVH
			CollisionPairMeshToMesh<Scalar> *collision_pair_mesh_to_mesh = dynamic_cast<CollisionPairMeshToMesh<Scalar> *>(collision_pair);
			CollisionPairCCD<Scalar> *collision_pair_ccd = dynamic_cast<CollisionPairCCD<Scalar> *>(collision_pair);
			// different way to handle according to different collision detection methods
			if (collision_pair_ccd != NULL) // if using CCD, then directly acquire collision point and normal from the collision pair
			{
				Vector<Scalar, Dim> collision_point = collision_pair_ccd->getContactPoint();
				Vector<Scalar, Dim> collision_normal = collision_pair_ccd->getContactNormal();
				ContactPoint<Scalar, Dim>* contact_point = new ContactPoint<Scalar, Dim>(numContactPoint(), collision_pair_ccd->objectLhsIdx(), collision_pair_ccd->objectRhsIdx(), *dynamic_cast<Vector<Scalar, Dim>*>(&collision_point), *dynamic_cast<Vector<Scalar, Dim>*>(&collision_normal));
				contact_points_.push_back(contact_point);
			}
			else if (collision_pair_mesh_to_mesh != NULL) // if using DTBVH, then compute them now
			{
				getMeshContactPoint(collision_pair_mesh_to_mesh);
			}
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
void ContactPointManager<Scalar, Dim>::cleanContactPoints()
{
    unsigned int num_contact = numContactPoint();
    for(unsigned int i = 0; i < num_contact; ++i)
    {
        delete contact_points_[i];
    }
    contact_points_.clear();
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
	// declare the needed position variables
    Face<Scalar>* face_lhs = collision_pair->faceLhsPtr();
    Face<Scalar>* face_rhs = collision_pair->faceRhsPtr();
    unsigned int num_vertex_lhs = face_lhs->numVertices();
    unsigned int num_vertex_rhs = face_rhs->numVertices();
    Vector<Scalar, 3>* vertex_lhs = new Vector<Scalar, 3>[num_vertex_lhs];
    Vector<Scalar, 3>* vertex_rhs = new Vector<Scalar, 3>[num_vertex_rhs];
	// get values for the needed positions
    for(unsigned int i = 0; i < num_vertex_lhs; i++)
    {
        vertex_lhs[i] = collision_pair->meshObjectLhs()->vertexPosition(face_lhs->vertex(i).positionIndex());
    }
    for(unsigned int i = 0; i < num_vertex_rhs; i++)
    {
        vertex_rhs[i] = collision_pair->meshObjectRhs()->vertexPosition(face_rhs->vertex(i).positionIndex());
    }

    unsigned int num_overlap = 0;
    bool is_lhs_tri = (num_vertex_lhs == 3);
    bool is_rhs_tri = (num_vertex_rhs == 3);
    bool is_lhs_quad = (num_vertex_lhs == 4);
    bool is_rhs_quad = (num_vertex_rhs == 4);
    Vector<Scalar, 3> overlap_point(0);
    Vector<Scalar, 3> temp_overlap_point(0);
    Vector<Scalar, 3> contact_normal_lhs(0);

    //test each edge of lhs with the face of rhs
	Vector<Scalar,3> mesh_rhs_face_normal = collision_pair->meshObjectRhs()->faceNormal(collision_pair->faceRhsIdx());
    for(unsigned int i = 0; i < num_vertex_lhs; i++)
    {
        if(is_rhs_tri)//triangle
        {
            if(MeshBasedCollidableObject<Scalar>::overlapEdgeTriangle(vertex_lhs[i], vertex_lhs[(i + 1)%num_vertex_lhs], vertex_rhs[0], vertex_rhs[1], vertex_rhs[2], mesh_rhs_face_normal, temp_overlap_point)) // decide whether overlap and compute temp_overlap_point
            {
                ContactPoint<Scalar, Dim>* contact_point = new ContactPoint<Scalar, Dim>(numContactPoint(), collision_pair->objectLhsIdx(), collision_pair->objectRhsIdx(),
                    *dynamic_cast<Vector<Scalar, Dim>*>(&temp_overlap_point), 
                    *dynamic_cast<Vector<Scalar, Dim>*>(&mesh_rhs_face_normal) * (-1));
                contact_points_.push_back(contact_point);
                //num_overlap++;
                //overlap_point += temp_overlap_point;
            }
        }
        if(is_rhs_quad)//quad
        { // do not support yet
            if(MeshBasedCollidableObject<Scalar>::overlapEdgeQuad(vertex_lhs[i], vertex_lhs[(i + 1)%num_vertex_lhs], vertex_rhs[0], vertex_rhs[1], vertex_rhs[2], vertex_rhs[3], temp_overlap_point))
            {
                //num_overlap++;
                //overlap_point += temp_overlap_point;
            }
        }
    }

    //test each edge of rhs with the face of lhs
	Vector<Scalar,3> mesh_lhs_face_normal = collision_pair->meshObjectLhs()->faceNormal(collision_pair->faceLhsIdx());
    for(unsigned int i = 0; i < num_vertex_rhs; i++)
    {
        if(is_lhs_tri)//triangle
        {
            if(MeshBasedCollidableObject<Scalar>::overlapEdgeTriangle(vertex_rhs[i], vertex_rhs[(i + 1)%num_vertex_rhs], vertex_lhs[0], vertex_lhs[1], vertex_lhs[2], mesh_lhs_face_normal, temp_overlap_point))
            {
                ContactPoint<Scalar, Dim>* contact_point = new ContactPoint<Scalar, Dim>(numContactPoint(), collision_pair->objectRhsIdx(), collision_pair->objectLhsIdx(),
                    *dynamic_cast<Vector<Scalar, Dim>*>(&temp_overlap_point), 
                    *dynamic_cast<Vector<Scalar, Dim>*>(&mesh_lhs_face_normal) * (-1));
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
    }
	// tried to use the average position of several neighbor contact points, but did not work well
    //generate position and normal
    //if(num_overlap > 0)
    //{
    //    overlap_point /= static_cast<Scalar>(num_overlap);
    //    //contact_normal_lhs = (collision_pair->meshObjectLhs()->faceNormal(collision_pair->faceLhsIdx()) - 
    //    //    collision_pair->meshObjectRhs()->faceNormal(collision_pair->faceRhsIdx())).normalize();
    //    contact_normal_lhs = -collision_pair->meshObjectRhs()->faceNormal(collision_pair->faceRhsIdx());
    //    if(contact_normal_lhs.norm() > std::numeric_limits<Scalar>::epsilon())
    //    {
    //        contact_normal_lhs.normalize();
    //        ContactPoint<Scalar, Dim>* contact_point = new ContactPoint<Scalar, Dim>(numContactPoint(), collision_pair->objectLhsIdx(), collision_pair->objectRhsIdx(),
    //            *dynamic_cast<Vector<Scalar, Dim>*>(&overlap_point), 
    //            *dynamic_cast<Vector<Scalar, Dim>*>(&contact_normal_lhs));
    //        contact_points_.push_back(contact_point);
    //    }
    //}

    delete[] vertex_lhs;
    delete[] vertex_rhs;
}


//template class ContactPointManager<float, 2>;
//template class ContactPointManager<double, 2>;
template class ContactPointManager<float, 3>;
template class ContactPointManager<double, 3>;

}
