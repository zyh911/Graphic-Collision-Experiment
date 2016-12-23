/*
 * @file rigid_body_2d.cpp
 * @2D rigid_body class.
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

#include "Physika_Dynamics/Rigid_Body/rigid_body.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_2d.h"
#include "Physika_Geometry/Boundary_Meshes/polygon.h"

namespace Physika{

template <typename Scalar>
RigidBody<Scalar, 2>::RigidBody()
{

}

template <typename Scalar>
RigidBody<Scalar, 2>::~RigidBody()
{

}

template <typename Scalar>
void RigidBody<Scalar, 2>::update(Scalar dt)
{
    //to do
}

template <typename Scalar>
void RigidBody<Scalar, 2>::performGravity(Scalar gravity, Scalar dt)
{
    //to do
}

//explicit instantiation
template class RigidBody<float, 2>;
template class RigidBody<double, 2>;

}
