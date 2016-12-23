/*
 * @file rigid_response_method_QCE.cpp 
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

#include <stdio.h>
#include "rigid_response_method_QCE.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_driver.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_driver_utility.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_3d.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_2d.h"
#include "Physika_Core/Matrices/sparse_matrix_iterator.h"
#include "Physika_Core/Timer/timer.h"
#include <map>

namespace Physika{

template <typename Scalar, int Dim>
RigidResponseMethodQCE<Scalar, Dim>::RigidResponseMethodQCE()
{

}

template <typename Scalar, int Dim>
RigidResponseMethodQCE<Scalar, Dim>::~RigidResponseMethodQCE()
{

}

template <typename Scalar, int Dim>
void RigidResponseMethodQCE<Scalar, Dim>::collisionResponse()
{
    //initialize
    m = this->rigid_driver_->numContactPoint();//m: number of contact points
    n = this->rigid_driver_->numRigidBody();//n: number of rigid bodies
    if(m == 0 || n == 0)//no collision or no rigid body
        return;

    dof = n * (Dim + RotationDof<Dim>::degree);//DoF(Degree of Freedom) of a rigid-body system
    fric_sample_count = 2;//count of friction sample directions
    s = m * fric_sample_count;//s: number of friction sample. Here a square sample is adopted
    J.resize(m, n);//Jacobian matrix
    D.resize(s, n);//Jacobian matrix of friction
    M.resize(n);//inertia matrix
    M_inv.resize(n);//inversed inertia matrix
    MJ.resize(m, n);
    MJ.transpose();
    MD.resize(s, n);
    MD.transpose();
    v.resize(dof);//generalized velocity of the system
    v *= 0;
    Jv.resize(m);//normal relative velocity of each contact point (for normal contact impulse calculation)
    Jv *= 0;
    Dv.resize(s);//tangent relative velocity of each contact point (for frictional contact impulse calculation)
    Dv *= 0;
    CoR.resize(m);//coefficient of restitution (for normal contact impulse calculation)
    CoR *= 0;
    CoF.resize(s);//coefficient of friction (for frictional contact impulse calculation)
    CoF *= 0;
    z_norm.resize(m);//normal contact impulse. The key of collision response
    z_norm *= 0;
    z_fric.resize(s);//frictional contact impulse. The key of collision response
    z_fric *= 0;

    //compute the matrix of dynamics
    RigidBodyDriverUtility<Scalar, Dim>::computeMassMatrix(this->rigid_driver_, M, M_inv);
    RigidBodyDriverUtility<Scalar, Dim>::computeJacobianMatrix(this->rigid_driver_, J);
    RigidBodyDriverUtility<Scalar, Dim>::computeFricJacobianMatrix(this->rigid_driver_, D);
    RigidBodyDriverUtility<Scalar, Dim>::computeGeneralizedVelocity(this->rigid_driver_, v);

    //compute other matrix in need
    CompressedJacobianMatrix<Scalar, Dim> J_T = J;
    J_T = J.transpose();
    CompressedJacobianMatrix<Scalar, Dim> D_T = D;
    D_T = D.transpose();
    MJ = M_inv * J_T;
    MD = M_inv * D_T;
    Jv = J * v;
    Dv = D * v;
    
    J.buildRelationTable();
    D.buildRelationTable();

    //double before_energy, after_energy;

    //update CoR and CoF
    RigidBodyDriverUtility<Scalar, Dim>::computeCoefficient(this->rigid_driver_, CoR, CoF);
    //before_energy = v.dot(M * v);
    //Timer time;
    //time.startTimer();
    //QCE step
    QCEStep();
    //time.stopTimer();
    //std::cout<<time.getElapsedTime()<<std::endl;
    //after_energy = QCE_v.dot(M * QCE_v);
    //compute expected post-impact velocity
    VectorND<Scalar> post_Jv(m, 0);
    for(unsigned int i = 0; i < m; ++i)
        post_Jv[i] = QCE_Jv[i] * CoR[i];

    //std::cout<<"energy: "<<QCE_v.dot(M * QCE_v)<<std::endl;

    //time.startTimer();
    //solve BLCP with PGS. z_norm and z_fric are the unknown variables
    RigidBodyDriverUtility<Scalar, Dim>::solveBLCPWithPGS(this->rigid_driver_, J, D, MJ, MD, Jv, post_Jv, Dv, z_norm, z_fric, CoR, CoF, 50);
    //time.stopTimer();
    //std::cout<<time.getElapsedTime()<<std::endl;

    //double potential_energy = 0;
    //for(unsigned int i = 0; i < n; ++i)
    //{
    //    if(this->rigid_driver_->rigidBody(i)->isFixed())
    //        continue;
    //    potential_energy += this->rigid_driver_->rigidBody(i)->mass() * this->rigid_driver_->rigidBody(i)->globalTranslation()[1];
    //}

    //apply impulse
    RigidBodyDriverUtility<Scalar, Dim>::applyImpulse(this->rigid_driver_, z_norm, z_fric, J_T, D_T);

    
    //if(before_energy != 0)
    //{
    //    std::ofstream reslog;
    //    std::cout<<"error: "<<100 * abs(before_energy - after_energy) / before_energy<<std::endl;
    //    reslog.open("result.log", std::ios::app);
    //    reslog<<abs(before_energy - after_energy) / before_energy<<std::endl;
    //    reslog.close();
    //}
    //std::cout<<"potential energy: "<<potential_energy<<std::endl;
    //std::cout<<"total energy !!!!!!!!!!!!!!!!!!!: "<<v.dot(M * v) + potential_energy<<std::endl;
}

template <typename Scalar, int Dim>
void RigidResponseMethodQCE<Scalar, Dim>::QCEStep()
{
    MJF.resize(dof * n);
    F.resize(m);
    A.resize(m);
    B.resize(m);
    C.resize(m);
    P.resize(m);

    QCE_v = v;
    QCE_Jv = Jv;
    
    unsigned int iteration_count = 0;
    while(true)
    {
        iteration_count++;
        QCEInitFactor();
        QCEInitSolution();
        QCEInnerIteration();
        if(QCECheckTerminate(iteration_count))
            break;
        QCEApplyImpulse();
    }
}

template <typename Scalar, int Dim>
void RigidResponseMethodQCE<Scalar, Dim>::QCEInitFactor()
{
    MJF *= 0;
    F *= 0;
    A *= 0;
    B *= 0;
    C *= 0;
    P *= 0;

    for(unsigned int i = 0; i < m; ++i)
    {
        if(QCE_Jv[i] < 0)
            F[i] = -QCE_Jv[i];
    }
    
    MJF = MJ * F;
    A = J * MJF;
    A *= 0.5;
    for(unsigned int i = 0; i < m; i++)
        B[i] = F[i] * QCE_Jv[i];
    C *= 0;
}

template <typename Scalar, int Dim>
void RigidResponseMethodQCE<Scalar, Dim>::QCEInitSolution()
{
    solutions_.resize(m);
    Scalar root_lhs, root_rhs;

    //for each contact point, solve its energy equation
    for(unsigned int i = 0; i < m; ++i)
    {
        Scalar delta = B[i] * B[i] - 4 * A[i] * C[i];
        
        if(delta < 0 || A[i] <= 0 || F[i] <= 0)
        {
            solutions_[i] = -1;
            continue;
        }
        delta = sqrt(delta);
        root_lhs = (-B[i] - delta)/(2 * A[i]);
        root_rhs = (-B[i] + delta)/(2 * A[i]);
        root_lhs /= F[i];
        root_rhs /= F[i];
        if(root_lhs <= 0 && root_rhs <= 0)
        {
            P[i] = 0;
            solutions_[i] = -1;
        }
        else
        {
            solutions_[i] = max<Scalar>(root_lhs, root_rhs);
            ordered_solutions_.insert(std::make_pair<Scalar, unsigned int>(solutions_[i], i));
        }
    }
}

template <typename Scalar, int Dim>
void RigidResponseMethodQCE<Scalar, Dim>::QCEInnerIteration()
{
    Timer time;
    double time1 = 0;
    double time2 = 0;
    double time3 = 0;
    double time4 = 0;

    while(!ordered_solutions_.empty())
    {
        //contact points that end their contact process nearly in the same time
        std::vector<unsigned int> simultaneous;

        //select the min value from all solutions
        std::multimap<Scalar, unsigned int>::iterator pair = ordered_solutions_.begin();
        Scalar s = pair->first;//calculate s
        P[pair->second] = s * F[pair->second];//calculate its impulse
        simultaneous.push_back(pair->second);//record it in the simultaneous list
        ordered_solutions_.erase(ordered_solutions_.begin());//delete it from solutions

        //find contact points that end their contact process nearly in the same time with the selected one
        if(!ordered_solutions_.empty())
        {
            pair = ordered_solutions_.begin();
            while((pair->first - s)/s < 0.000001)//finish time in the tolerance
            {
                P[pair->second] = pair->first * F[pair->second];//calculate its impulse
                simultaneous.push_back(pair->second);//record it in the simultaneous list
                ordered_solutions_.erase(ordered_solutions_.begin());//delete it from solutions
                if(ordered_solutions_.empty())
                    break;
                pair = ordered_solutions_.begin();
            }
        }

        //modify the solutions of contact points effected by the termination
        for(unsigned int index = 0; index < simultaneous.size(); ++index)
        {
            
            unsigned int point_index = simultaneous[index];//terminated contact points

            std::vector<unsigned int> related_contact_points;
            J.relatedRows(point_index, related_contact_points);
            
            for(std::vector<unsigned int>::iterator it = related_contact_points.begin(); it != related_contact_points.end(); ++it)
            //for(SparseMatrixIterator<Scalar> it(JMJ, point_index); it; ++it)//contact points effect by its termination
            {
                //Scalar value = it.value();
                //unsigned int neighbor = it.row();
                unsigned int neighbor = *it;
                if(P[neighbor] != 0)
                    continue;
                if(F[neighbor] == 0)
                    continue;
                if(solutions_[neighbor] == -1)
                    continue;
                Scalar value = productValue(J, MJ, point_index, neighbor);

                //record its original factors of energy equation
                Scalar original_a, original_b;
                original_a = A[neighbor];
                original_b = B[neighbor];

                //modify its factors of energy equation
                A[neighbor] -= 0.5 * value * F[point_index];
                B[neighbor] += value * F[neighbor] * P[point_index];
                Scalar current_P = F[neighbor] * s;
                C[neighbor] = original_a * current_P * current_P + original_b * current_P + C[neighbor] - (A[neighbor] * current_P * current_P + B[neighbor] * current_P);
            }
        }
        /*
        Scalar energy = (QCE_v + MJ * P).dot(M * (QCE_v + MJ * P));
        std::cout<<ordered_solutions_.size()<<" : "<<std::endl;
        std::cout<<(QCE_v + MJ * old_p).dot(M * (QCE_v + MJ * old_p))<<" "<<energy<<" ";
        old_p = P;
        Scalar potential = 0;
        int count = 0;
        for(int i = 0; i < C.dims(); i++)
        {
            if(P[i] == 0 && F[i] != 0 && solutions_[i] != -1 && A[i] > 0)
            {
                potential -= (A[i] * s * F[i] * s * F[i] + B[i] * s * F[i] + C[i])/F[i];
                count++;
            }
        }
        std::cout<<energy + potential<<" "<<potential<<" "<<count<<std::endl;*/

        //re-calculate the solutions for contact points effected by the termination
        Scalar root_lhs, root_rhs;
        for(unsigned int index = 0; index < simultaneous.size(); ++index)
        {
            unsigned int point_index = simultaneous[index];//terminated contact points

            std::vector<unsigned int> related_contact_points;
            J.relatedRows(point_index, related_contact_points);

            for(std::vector<unsigned int>::iterator it = related_contact_points.begin(); it != related_contact_points.end(); ++it)
            //for (SparseMatrixIterator<Scalar> it(JMJ, point_index); it; ++it)//contact points effect by its termination
            {
                //Scalar value = it.value();
                //unsigned int neighbor = it.row();
                unsigned int neighbor = *it;
                if(P[neighbor] != 0)
                    continue;
                if(F[neighbor] == 0)
                    continue;
                if(solutions_[neighbor] == -1)
                    continue;

                Scalar original_solution = solutions_[neighbor];

                //solve the new equation
                Scalar delta = B[neighbor] * B[neighbor] - 4 * A[neighbor] * C[neighbor];
                if(A[neighbor] < 0 && delta < 0)
                {
                    solutions_[neighbor] = -1;
                    P[neighbor] = s * F[neighbor];

                    //find the original solution in ordered_solutions_
                    std::multimap<Scalar, unsigned int>::iterator position = ordered_solutions_.find(original_solution);
                    while(position->second != neighbor)
                    {
                        position++;
                        if(position->first > original_solution)
                        {
                            std::cerr<<"Can't find a solution!"<<std::endl;
                            return;
                        }
                    }

                    //delete or modify the original solution
                    ordered_solutions_.erase(position);
                    if(solutions_[neighbor] != -1)
                        ordered_solutions_.insert(std::make_pair<Scalar, unsigned int>(solutions_[neighbor], neighbor));

                    continue;
                }   
                if(delta < 0 || A[neighbor] == 0 || F[neighbor] <= 0)
                {
                    P[neighbor] = 0;
                    solutions_[neighbor] = -1;
                    continue;
                }
                delta = sqrt(delta);
                root_lhs = (-B[neighbor] - delta)/(2 * A[neighbor]);
                root_rhs = (-B[neighbor] + delta)/(2 * A[neighbor]);
                root_lhs /= F[neighbor];
                root_rhs /= F[neighbor];
                
                if(root_lhs <= 0 && root_rhs <= 0)
                {
                    P[neighbor] = 0;
                    solutions_[neighbor] = -1;
                }
                else
                {
                    if(A[neighbor] > 0)
                        solutions_[neighbor] = max<Scalar>(root_lhs, root_rhs);
                    else
                        //solutions_[neighbor] = 10000000000;
                        solutions_[neighbor] = min<Scalar>(root_lhs, root_rhs);
                    if(solutions_[neighbor] < s)
                    {
                        std::cout<<"here"<<std::endl;
                        std::cout<<A[neighbor]<<" "<<solutions_[neighbor]<<" "<<s<<std::endl;
                        system("pause");
                        solutions_[neighbor] = -1;
                        P[neighbor] = s * F[neighbor];
                        //solutions_[neighbor] = 100000000;
                    }
                }

                //find the original solution in ordered_solutions_
                std::multimap<Scalar, unsigned int>::iterator position = ordered_solutions_.find(original_solution);
                while(position->second != neighbor)
                {
                    position++;
                    if(position->first > original_solution)
                    {
                        std::cerr<<"Can't find a solution!"<<std::endl;
                        return;
                    }
                }

                //delete or modify the original solution
                ordered_solutions_.erase(position);
                if(solutions_[neighbor] != -1)
                    ordered_solutions_.insert(std::make_pair<Scalar, unsigned int>(solutions_[neighbor], neighbor));
            }
        }
    }
}

template <typename Scalar, int Dim>
void RigidResponseMethodQCE<Scalar, Dim>::QCEApplyImpulse()
{
    z_norm += P;
    QCE_v += MJ * P;
    QCE_Jv = J * QCE_v;
}

template <typename Scalar, int Dim>
bool RigidResponseMethodQCE<Scalar, Dim>::QCECheckTerminate(unsigned int iteration_count)
{
    if(iteration_count >= 50)
        return true;
    bool is_terminate = true;
    for(unsigned int i = 0; i < m; ++i)
    {
        if(abs(P[i]) > 0)
            is_terminate = false;
    }
    return is_terminate;
}

//template class RigidResponseMethodQCE<float, 2>;
template class RigidResponseMethodQCE<float, 3>;
//template class RigidResponseMethodQCE<double, 2>;
template class RigidResponseMethodQCE<double, 3>;

}
