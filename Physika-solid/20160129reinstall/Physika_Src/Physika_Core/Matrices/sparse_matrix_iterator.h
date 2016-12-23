/*
* @file sparse_matrix_iterator.h
* @brief  iterator of SparseMatrix class.
* @author Liyou Xu
*
* This file is part of Physika, a versatile physics simulation library.
* Copyright (C) 2013 Physika Group.
*
* This Source Code Form is subject to the terms of the GNU General Public License v2.0.
* If a copy of the GPL was not distributed with this file, you can obtain one at:
* http://www.gnu.org/licenses/gpl-2.0.html
*
*/
#ifndef PHYSIKA_CORE_MATRICES_SPARSE_MATRIX_ITERATOR_H_
#define PHYSIKA_CORE_MATRICES_SPARSE_MATRIX_ITERATOR_H_
#include "Physika_Dependency/Eigen/Eigen"
#include "Physika_Core/Matrices/sparse_matrix.h"
#include <iostream>

namespace Physika{
    
    template <typename Scalar>
    class SparseMatrixIterator
    {
    public:
        SparseMatrixIterator(SparseMatrix<Scalar> & mat, unsigned int i);
        SparseMatrixIterator<Scalar>& operator++();
        unsigned int row();
        unsigned int col();
        Scalar value();
        operator bool ();
    protected:
#ifdef PHYSIKA_USE_BUILT_IN_SPARSE_MATRIX
        unsigned int first_ele_ ,last_ele_;
        SparseMatrix<Scalar> *ptr_matrix_;
#elif defined(PHYSIKA_USE_EIGEN_SPARSE_MATRIX)
        typename Eigen::SparseMatrix<Scalar>::InnerIterator it;
#endif
    };

}  //end of namespace Physika

//implementation

#endif //PHYSIKA_CORE_MATRICES_SPARSE_MATRIX_ITERATOR_H_