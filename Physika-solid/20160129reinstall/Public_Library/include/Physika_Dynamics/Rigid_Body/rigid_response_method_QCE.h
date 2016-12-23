/*
 * @file rigid_response_method_QCE.h 
 * @Rigid-body collision response using QCE
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

#ifndef PHYSIKA_DYNAMICS_RIGID_BODY_RIGID_RESPONSE_METHOD_QCE_H_
#define PHYSIKA_DYNAMICS_RIGID_BODY_RIGID_RESPONSE_METHOD_QCE_H_

#include "Physika_Dynamics/Rigid_Body/rigid_response_method.h"
#include "Physika_Dynamics/Rigid_Body/compressed_matrix.h"
#include "Physika_Core/Utilities/math_utilities.h"
#include <map>

namespace Physika{

template <typename Scalar, int Dim>
class RigidResponseMethodQCE : public RigidResponseMethod<Scalar, Dim>
{
public:
    //constructor
    RigidResponseMethodQCE();
    virtual ~RigidResponseMethodQCE();

    //dynamic function used in a driver
    void collisionResponse();

protected:

    void QCEStep();//Use QCE to resolve this system
    void QCEInitFactor();//initialize variables, e.g. A, B, C used in following steps
    void QCEInitSolution();//compute the initial solution for each contact point
    void QCEInnerIteration();//repeatedly select the min value from solutions, end its contact process and modify the solutions of other points effected by it
    void QCEApplyImpulse();//apply the impulse onto generalized velocities
    bool QCECheckTerminate(unsigned int iteration_count);//check whether all contact points are resolved

    //normal simulation variables
    unsigned int m;//m: number of contact points
    unsigned int n;//n: number of rigid bodies
    unsigned int fric_sample_count;//count of friction sample directions
    unsigned int s;//s: number of friction sample. Here a square sample is adopted
    unsigned int dof;//DoF(Degree of Freedom) of a rigid-body system
    CompressedJacobianMatrix<Scalar, Dim> J;//Jacobian matrix
    CompressedJacobianMatrix<Scalar, Dim> D;//Jacobian matrix of friction
    CompressedInertiaMatrix<Scalar, Dim> M;//inertia matrix
    CompressedInertiaMatrix<Scalar, Dim> M_inv;//inversed inertia matrix
    CompressedJacobianMatrix<Scalar, Dim> MJ;
    CompressedJacobianMatrix<Scalar, Dim> MD;
    VectorND<Scalar> v;//generalized velocity of the system
    VectorND<Scalar> Jv;//normal relative velocity of each contact point (for normal contact impulse calculation)
    VectorND<Scalar> Dv;//tangent relative velocity of each contact point (for frictional contact impulse calculation)
    VectorND<Scalar> CoR;//coefficient of restitution (for normal contact impulse calculation)
    VectorND<Scalar> CoF;//coefficient of friction (for frictional contact impulse calculation)
    VectorND<Scalar> z_norm;//normal contact impulse. The key of collision response
    VectorND<Scalar> z_fric;//frictional contact impulse. The key of collision response

    //variables for QCE
    VectorND<Scalar> solutions_;//solutions of each contact point's energy equation
    std::multimap<Scalar, unsigned int> ordered_solutions_;//map for <solution, point_index>, used to find the min value of solutions in O(1) time and modify an arbitrary solution in O(lgn)
    VectorND<Scalar> MJF;
    VectorND<Scalar> F;
    VectorND<Scalar> A;
    VectorND<Scalar> B;
    VectorND<Scalar> C;
    VectorND<Scalar> P;

    VectorND<Scalar> QCE_v;//generalized velocity modified in QCE steps instead of the real generalized velocity v
    VectorND<Scalar> QCE_Jv;//relative velocities modified in QCE steps instead of the real relative velocities Jv
};

}

#endif //PHYSIKA_DYNAMICS_RIGID_BODY_RIGID_RESPONSE_METHOD_QCE_H_