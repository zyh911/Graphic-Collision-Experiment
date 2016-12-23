/*
 * @file  tri_mesh.cpp
 * @brief Counterpart of Tetrahedral mesh in 2D.
 * @author Fei Zhu
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
#include "Physika_Core/Utilities/physika_assert.h"
#include "Physika_Core/Utilities/math_utilities.h"
#include "Physika_Core/Vectors/vector_2d.h"
#include "Physika_Core/Matrices/matrix_3x3.h"
#include "Physika_Core/Arrays/array.h"
#include "Physika_Geometry/Volumetric_Meshes/tri_mesh.h"

namespace Physika{

template <typename Scalar>
TriMesh<Scalar>::TriMesh():VolumetricMesh<Scalar,2>()
{
    (this->vert_per_ele_).clear();
    (this->vert_per_ele_).push_back(3);
}

template <typename Scalar>
TriMesh<Scalar>::TriMesh(unsigned int vert_num, const Scalar *vertices, unsigned int ele_num, const unsigned int *elements)
	:VolumetricMesh<Scalar,2>(vert_num, vertices, ele_num, elements, 3)
{
}

template <typename Scalar>
TriMesh<Scalar>::TriMesh(const TriMesh<Scalar> &tri_mesh)
    :VolumetricMesh<Scalar,2>(tri_mesh)
{
}

template <typename Scalar>
TriMesh<Scalar>::~TriMesh()
{
}

template <typename Scalar>
TriMesh<Scalar>& TriMesh<Scalar>::operator= (const TriMesh<Scalar> &tri_mesh)
{
    VolumetricMesh<Scalar,2>::operator= (tri_mesh);
    return *this;
}

template <typename Scalar>
TriMesh<Scalar>* TriMesh<Scalar>::clone() const
{
    return new TriMesh<Scalar>(*this);
}

template <typename Scalar>
void TriMesh<Scalar>::printInfo() const
{
    std::cout<<"2D Triangle Mesh."<<std::endl;
}

template <typename Scalar>
VolumetricMeshInternal::ElementType TriMesh<Scalar>::elementType() const
{
    return VolumetricMeshInternal::TRI;
}

template <typename Scalar>
unsigned int TriMesh<Scalar>::eleVertNum() const
{
    return 3;
}

template <typename Scalar>
Scalar TriMesh<Scalar>::eleVolume(unsigned int ele_idx) const
{
    if(ele_idx>=this->ele_num_)
    {
        std::cerr<<"TriMesh element index out of range!\n";
        std::exit(EXIT_FAILURE);
    }
    Array< Vector<Scalar,2> > ele_vertices(3);
    for(unsigned int i = 0; i < 3; ++i)
        ele_vertices[i] = this->eleVertPos(ele_idx,i);
    Vector<Scalar,2> b_minus_a = ele_vertices[1] - ele_vertices[0];
    Vector<Scalar,2> c_minus_a = ele_vertices[2] - ele_vertices[0]; 
    return abs(b_minus_a.cross(c_minus_a))/2.0;
}

template <typename Scalar>
bool TriMesh<Scalar>::containsVertex(unsigned int ele_idx, const Vector<Scalar,2> &pos) const
{
    if((ele_idx<0) || (ele_idx>=this->ele_num_))
    {
        std::cerr<<"TriMesh element index out of range!\n";
        std::exit(EXIT_FAILURE);
    }
    std::vector<Scalar> weights;
    interpolationWeights(ele_idx,pos,weights);
    bool vert_in_ele = (weights[0]>=0)&&(weights[1]>=0)&&(weights[2]>=0);
    return vert_in_ele;
}

template <typename Scalar>
void TriMesh<Scalar>::interpolationWeights(unsigned int ele_idx, const Vector<Scalar,2> &pos, std::vector<Scalar> &weights) const
{
/*
  w0    |x1 y1 z1|  -1       |pos1|
  w1  = |x2 y2 z2|          *|pos2|
  w2    |1  1  1 |           | 1  |


*/
    if(ele_idx>=this->ele_num_)
    {
        std::cerr<<"TriMesh element index out of range!\n";
        std::exit(EXIT_FAILURE);
    }
    Array< Vector<Scalar,2> > ele_vertices(3);
    for(unsigned int i = 0; i < 3; ++i)
        ele_vertices[i] = this->eleVertPos(ele_idx,i);
    SquareMatrix<Scalar, 3> m0(Vector<Scalar, 3>(ele_vertices[0][0] , ele_vertices[1][0], ele_vertices[2][0]),
                               Vector<Scalar, 3>(ele_vertices[0][1], ele_vertices[1][1], ele_vertices[2][1]),
                               Vector<Scalar, 3>(1, 1, 1));
    Vector<Scalar, 3> result;
    result = (m0.inverse())*Vector<Scalar, 3>(pos[0], pos[1], 1);
    weights.clear();
    for(unsigned int i = 0; i <3; ++i)
        weights.push_back(result[i]);
}

//explicit instantitation
template class TriMesh<float>;
template class TriMesh<double>;

}  //end of namespace Physika
