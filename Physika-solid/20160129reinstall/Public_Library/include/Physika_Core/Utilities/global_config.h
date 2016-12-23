/*
 * @file global_config.h 
 * @brief This file is used to globally configurate Physika.
 * @author FeiZhu
 * 
 * This file is part of Physika, a versatile physics simulation library.
 * Copyright (C) 2013 Physika Group.
 *
 * This Source Code Form is subject to the terms of the GNU General Public License v2.0. 
 * If a copy of the GPL was not distributed with this file, you can obtain one at:
 * http://www.gnu.org/licenses/gpl-2.0.html
 *
 */

#ifndef PHYSIKA_CORE_UTILITIES_GLOBAL_CONFIG_H_
#define PHYSIKA_CORE_UTILITIES_GLOBAL_CONFIG_H_

//global macros to configurate Physika

/***********Matrix&Vector***********/

#define PHYSIKA_USE_EIGEN_MATRIX
//#define PHYSIKA_USE_BUILT_IN_MATRIX

//#define PHYSIKA_USE_EIGEN_VECTOR
#define PHYSIKA_USE_BUILT_IN_VECTOR

//#define PHYSIKA_USE_EIGEN_SPARSE_MATRIX
#define PHYSIKA_USE_BUILT_IN_SPARSE_MATRIX

////////////////////////////////////////////

//include necessary header files with different configurations
#if defined(PHYSIKA_USE_EIGEN_MATRIX)||defined(PHYSIKA_USE_EIGEN_VECTOR)||defined(PHYSIKA_USE_EIGEN_SPARSE_MATRIX)
#include "Physika_Dependency/Eigen/Eigen"
#endif

#endif //PHYSIKA_CORE_UTILITIES_GLOBAL_CONFIG_H_
