/*
 * @file matrix_3x3.cpp 
 * @brief 3x3 matrix.
 * @author Sheng Yang, Fei Zhu
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
#include <cstdlib>
#include "Physika_Core/Utilities/math_utilities.h"
#include "Physika_Core/Vectors/vector_3d.h"
#include "Physika_Core/Matrices/matrix_3x3.h"

namespace Physika{

template <typename Scalar>
SquareMatrix<Scalar,3>::SquareMatrix()
{
}

template <typename Scalar>
SquareMatrix<Scalar,3>::SquareMatrix(Scalar value)
{
#ifdef PHYSIKA_USE_EIGEN_MATRIX
    eigen_matrix_3x3_(0,0) = value;
    eigen_matrix_3x3_(0,1) = value;
    eigen_matrix_3x3_(0,2) = value;
    eigen_matrix_3x3_(1,0) = value;
    eigen_matrix_3x3_(1,1) = value;
    eigen_matrix_3x3_(1,2) = value;
    eigen_matrix_3x3_(2,0) = value;
    eigen_matrix_3x3_(2,1) = value;
    eigen_matrix_3x3_(2,2) = value;
#elif defined(PHYSIKA_USE_BUILT_IN_MATRIX)
    data_[0][0] = value;
    data_[0][1] = value;
    data_[0][2] = value;
    data_[1][0] = value;
    data_[1][1] = value;
    data_[1][2] = value;
    data_[2][0] = value;
    data_[2][1] = value;
    data_[2][2] = value;
#endif
}

template <typename Scalar>
SquareMatrix<Scalar,3>::SquareMatrix(Scalar x00, Scalar x01, Scalar x02, Scalar x10, Scalar x11, Scalar x12, Scalar x20, Scalar x21, Scalar x22)
{
#ifdef PHYSIKA_USE_EIGEN_MATRIX
    eigen_matrix_3x3_(0,0) = x00;
    eigen_matrix_3x3_(0,1) = x01;
    eigen_matrix_3x3_(0,2) = x02;
    eigen_matrix_3x3_(1,0) = x10;
    eigen_matrix_3x3_(1,1) = x11;
    eigen_matrix_3x3_(1,2) = x12;
    eigen_matrix_3x3_(2,0) = x20;
    eigen_matrix_3x3_(2,1) = x21;
    eigen_matrix_3x3_(2,2) = x22;
#elif defined(PHYSIKA_USE_BUILT_IN_MATRIX)
    data_[0][0] = x00;
    data_[0][1] = x01;
    data_[0][2] = x02;
    data_[1][0] = x10;
    data_[1][1] = x11;
    data_[1][2] = x12;
    data_[2][0] = x20;
    data_[2][1] = x21;
    data_[2][2] = x22;
#endif
}

template <typename Scalar>
SquareMatrix<Scalar,3>::SquareMatrix(const Vector<Scalar,3> &row1, const Vector<Scalar,3> &row2, const Vector<Scalar,3> &row3)
{
#ifdef PHYSIKA_USE_EIGEN_MATRIX
    for(unsigned int col = 0; col < 3; ++col)
    {
        eigen_matrix_3x3_(0,col) = row1[col];
        eigen_matrix_3x3_(1,col) = row2[col];
        eigen_matrix_3x3_(2,col) = row3[col];
    }
#elif defined(PHYSIKA_USE_BUILT_IN_MATRIX)
    for(unsigned int col = 0; col < 3; ++col)
    {
        data_[0][col] = row1[col];
        data_[1][col] = row2[col];
        data_[2][col] = row3[col];
    }
#endif
}

template <typename Scalar>
SquareMatrix<Scalar,3>::SquareMatrix(const SquareMatrix<Scalar,3> &mat2)
{
    *this = mat2;
}

template <typename Scalar>
SquareMatrix<Scalar,3>::~SquareMatrix()
{
}

template <typename Scalar>
Scalar& SquareMatrix<Scalar,3>::operator() (unsigned int i, unsigned int j)
{
    bool index_valid = (i<3)&&(j<3);
    if(!index_valid)
    {
        std::cerr<<"Matrix index out of range!\n";
        std::exit(EXIT_FAILURE);
    }
#ifdef PHYSIKA_USE_EIGEN_MATRIX
    return eigen_matrix_3x3_(i,j);
#elif defined(PHYSIKA_USE_BUILT_IN_MATRIX)
    return data_[i][j];
#endif
}

template <typename Scalar>
const Scalar& SquareMatrix<Scalar,3>::operator() (unsigned int i, unsigned int j) const
{
    bool index_valid = (i<3)&&(j<3);
    if(!index_valid)
    {
        std::cerr<<"Matrix index out of range!\n";
        std::exit(EXIT_FAILURE);
    }
#ifdef PHYSIKA_USE_EIGEN_MATRIX
    return eigen_matrix_3x3_(i,j);
#elif defined(PHYSIKA_USE_BUILT_IN_MATRIX)
    return data_[i][j];
#endif
}

template <typename Scalar>
SquareMatrix<Scalar,3> SquareMatrix<Scalar,3>::operator+ (const SquareMatrix<Scalar,3> &mat3) const
{
    Scalar result[9];
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            result[i*3+j] = (*this)(i,j) + mat3(i,j);
    return SquareMatrix<Scalar,3>(result[0], result[1], result[2], result[3] , result[4], result[5], result[6], result[7], result[8]);
}

template <typename Scalar>
SquareMatrix<Scalar,3>& SquareMatrix<Scalar,3>::operator+= (const SquareMatrix<Scalar,3> &mat3)
{
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            (*this)(i,j) = (*this)(i,j) + mat3(i,j);
    return *this;
}

template <typename Scalar>
SquareMatrix<Scalar,3> SquareMatrix<Scalar,3>::operator- (const SquareMatrix<Scalar,3> &mat3) const
{
    Scalar result[9];
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            result[i*3+j] = (*this)(i,j) - mat3(i,j);
    return SquareMatrix<Scalar,3>(result[0], result[1], result[2], result[3] , result[4], result[5], result[6], result[7], result[8]);
}

template <typename Scalar>
SquareMatrix<Scalar,3>& SquareMatrix<Scalar,3>::operator-= (const SquareMatrix<Scalar,3> &mat3)
{
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            (*this)(i,j) = (*this)(i,j) - mat3(i,j);
    return *this;
}

template <typename Scalar>
SquareMatrix<Scalar,3>& SquareMatrix<Scalar,3>::operator= (const SquareMatrix<Scalar,3> &mat3)
{
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            (*this)(i,j) = mat3(i,j);
    return *this;
}

template <typename Scalar>
bool SquareMatrix<Scalar,3>::operator== (const SquareMatrix<Scalar,3> &mat3) const
{
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            if((*this)(i,j) != mat3(i,j))
                return false;
    return true;
}

template <typename Scalar>
bool SquareMatrix<Scalar,3>::operator!= (const SquareMatrix<Scalar,3> &mat3) const
{
    return !((*this)==mat3);
}

template <typename Scalar>
SquareMatrix<Scalar,3> SquareMatrix<Scalar,3>::operator* (Scalar scale) const
{
    Scalar result[9];
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            result[i*3+j] = (*this)(i,j) * scale;
    return SquareMatrix<Scalar,3>(result[0], result[1], result[2], result[3] , result[4], result[5], result[6], result[7], result[8]);
}

template <typename Scalar>
SquareMatrix<Scalar,3>& SquareMatrix<Scalar,3>::operator*= (Scalar scale)
{
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            (*this)(i,j) = (*this)(i,j) * scale;
    return *this;
}

template <typename Scalar>
Vector<Scalar,3> SquareMatrix<Scalar,3>::operator* (const Vector<Scalar,3> &vec) const
{
    Vector<Scalar,3> result(0);
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            result[i] += (*this)(i,j)*vec[j];
    return result;
}

template <typename Scalar>
SquareMatrix<Scalar,3> SquareMatrix<Scalar,3>::operator* (const SquareMatrix<Scalar,3> &mat2) const
{
    SquareMatrix<Scalar,3> result(0,0,0,0,0,0,0,0,0);
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            for(unsigned int k = 0; k < 3; ++k)
                result(i,j) += (*this)(i,k) * mat2(k,j);
    return result;
}

template <typename Scalar>
SquareMatrix<Scalar,3>& SquareMatrix<Scalar,3>::operator*= (const SquareMatrix<Scalar,3> &mat2)
{
    SquareMatrix<Scalar,3> result(0,0,0,0,0,0,0,0,0);
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            for(unsigned int k = 0; k < 3; ++k)
                result(i,j) += (*this)(i,k) * mat2(k,j);
    *this = result;
    return *this;
}
    
template <typename Scalar>
SquareMatrix<Scalar,3> SquareMatrix<Scalar,3>::operator/ (Scalar scale) const
{
    if(abs(scale)<std::numeric_limits<Scalar>::epsilon())
    {
        std::cerr<<"Matrix Divide by zero error!\n";
        std::exit(EXIT_FAILURE);
    }
    Scalar result[9];
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            result[i*3+j] = (*this)(i,j) / scale;
    return SquareMatrix<Scalar,3>(result[0], result[1], result[2], result[3] , result[4], result[5], result[6], result[7], result[8]);
}

template <typename Scalar>
SquareMatrix<Scalar,3>& SquareMatrix<Scalar,3>::operator/= (Scalar scale)
{
    if(abs(scale)<std::numeric_limits<Scalar>::epsilon())
    {
        std::cerr<<"Matrix Divide by zero error!\n";
        std::exit(EXIT_FAILURE);
    }
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            (*this)(i,j) = (*this)(i,j) / scale;
    return *this;
}

template <typename Scalar>
SquareMatrix<Scalar,3> SquareMatrix<Scalar,3>::transpose() const
{
    Scalar result[9];
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j< 3; ++j)
            result[i*3+j] = (*this)(j,i);
    return SquareMatrix<Scalar,3>(result[0], result[1], result[2], result[3] , result[4], result[5], result[6], result[7], result[8]);
}

template <typename Scalar>
SquareMatrix<Scalar,3> SquareMatrix<Scalar,3>::inverse() const
{
    Scalar det = determinant();
    if(isEqual(det,static_cast<Scalar>(0)))
    {
        std::cerr<<"Matrix not invertible!\n";
        std::exit(EXIT_FAILURE);
    }
    return SquareMatrix<Scalar,3>((-(*this)(1,2) * (*this)(2,1) + (*this)(1,1) * (*this)(2,2))/det, ((*this)(0,2) * (*this)(2,1) - (*this)(0,1) * (*this)(2,2))/det, 
                                  (-(*this)(0,2) * (*this)(1,1) + (*this)(0,1) * (*this)(1,2))/det, ((*this)(1,2) * (*this)(2,0) - (*this)(1,0) * (*this)(2,2))/det,
                                  (-(*this)(0,2) * (*this)(2,0) + (*this)(0,0) * (*this)(2,2))/det, ((*this)(0,2) * (*this)(1,0) - (*this)(0,0) * (*this)(1,2))/det,
                                  (-(*this)(1,1) * (*this)(2,0) + (*this)(1,0) * (*this)(2,1))/det, ((*this)(0,1) * (*this)(2,0) - (*this)(0,0) * (*this)(2,1))/det,
                                  (-(*this)(0,1) * (*this)(1,0) + (*this)(0,0) * (*this)(1,1))/det);
}

template <typename Scalar>
Scalar SquareMatrix<Scalar,3>::determinant() const
{
#ifdef PHYSIKA_USE_EIGEN_MATRIX
    return eigen_matrix_3x3_.determinant();
#elif defined(PHYSIKA_USE_BUILT_IN_MATRIX)
    return (data_[0][0]*data_[1][1]*data_[2][2] + data_[0][1]*data_[1][2]*data_[2][0] + data_[0][2]*data_[1][0]*data_[2][1])
        - (data_[0][2]*data_[1][1]*data_[2][0] + data_[0][1]*data_[1][0]*data_[2][2] + data_[0][0]*data_[1][2]*data_[2][1]); 
#endif
}

template <typename Scalar>
Scalar SquareMatrix<Scalar,3>::trace() const
{
    return (*this)(0,0) + (*this)(1,1) + (*this)(2,2);
}

template <typename Scalar>
SquareMatrix<Scalar,3> SquareMatrix<Scalar,3>::identityMatrix()
{
    return SquareMatrix<Scalar,3>(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0);
}

template <typename Scalar>
Scalar SquareMatrix<Scalar,3>::doubleContraction(const SquareMatrix<Scalar,3> &mat2) const
{
    Scalar result = 0;
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            result += (*this)(i,j)*mat2(i,j);
    return result;
}

template <typename Scalar>
void SquareMatrix<Scalar,3>::singularValueDecomposition(SquareMatrix<Scalar,3> &left_singular_vectors,
                                                        Vector<Scalar,3> &singular_values,
                                                        SquareMatrix<Scalar,3> &right_singular_vectors) const
{
#ifdef PHYSIKA_USE_EIGEN_MATRIX
    //hack: Eigen::SVD does not support integer types, hence we cast Scalar to long double for decomposition
    Eigen::Matrix<long double,3,3> temp_matrix;
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)          
            temp_matrix(i,j) = static_cast<long double>(eigen_matrix_3x3_(i,j));
    Eigen::JacobiSVD<Eigen::Matrix<long double,3,3> > svd(temp_matrix,Eigen::ComputeThinU|Eigen::ComputeThinV);
    const Eigen::Matrix<long double,3,3> &left = svd.matrixU(), &right = svd.matrixV();
    const Eigen::Matrix<long double,3,1> &values = svd.singularValues();
    for(unsigned int i = 0; i < 3; ++i)
    {
        singular_values[i] = static_cast<Scalar>(values(i,0));
        for(unsigned int j = 0; j < 3; ++j)
        {
            left_singular_vectors(i,j) = static_cast<Scalar>(left(i,j));
            right_singular_vectors(i,j) = static_cast<Scalar>(right(i,j));
        }
    }
#elif defined(PHYSIKA_USE_BUILT_IN_MATRIX)
    std::cerr<<"SVD not implemeted for built in matrix!\n";
    std::exit(EXIT_FAILURE);
#endif
}

template <typename Scalar>
void SquareMatrix<Scalar,3>::eigenDecomposition(Vector<Scalar,3> &eigen_values_real, Vector<Scalar,3> &eigen_values_imag,
                                                SquareMatrix<Scalar,3> &eigen_vectors_real, SquareMatrix<Scalar,3> &eigen_vectors_imag)
{
#ifdef PHYSIKA_USE_EIGEN_MATRIX
    //hack: Eigen::EigenSolver does not support integer types, hence we cast Scalar to long double for decomposition
    Eigen::Matrix<long double,3,3> temp_matrix;
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)          
            temp_matrix(i,j) = static_cast<long double>(eigen_matrix_3x3_(i,j));
    Eigen::EigenSolver<Eigen::Matrix<long double,3,3> > eigen(temp_matrix);
    Eigen::Matrix<std::complex<long double>,3,3> vectors = eigen.eigenvectors();
    const Eigen::Matrix<std::complex<long double>,3,1> &values = eigen.eigenvalues();
    for(unsigned int i = 0; i < 3; ++i)
    {
        eigen_values_real[i] = static_cast<Scalar>(values(i,0).real());
        eigen_values_imag[i] = static_cast<Scalar>(values(i,0).imag());
        for(unsigned int j = 0; j < 3; ++j)
        {
            eigen_vectors_real(i,j) = static_cast<Scalar>(vectors(i,j).real());
            eigen_vectors_imag(i,j) = static_cast<Scalar>(vectors(i,j).imag());
        }
    }
#elif defined(PHYSIKA_USE_BUILT_IN_MATRIX)
    std::cerr<<"Eigen decomposition not implemeted for built in matrix!\n";
    std::exit(EXIT_FAILURE);
#endif
}

//explicit instantiation of template so that it could be compiled into a lib
template class SquareMatrix<unsigned char,3>;
template class SquareMatrix<unsigned short,3>;
template class SquareMatrix<unsigned int,3>;
template class SquareMatrix<unsigned long,3>;
template class SquareMatrix<unsigned long long,3>;
template class SquareMatrix<signed char,3>;
template class SquareMatrix<short,3>;
template class SquareMatrix<int,3>;
template class SquareMatrix<long,3>;
template class SquareMatrix<long long,3>;
template class SquareMatrix<float,3>;
template class SquareMatrix<double,3>;
template class SquareMatrix<long double,3>;

}  //end of namespace Physika
