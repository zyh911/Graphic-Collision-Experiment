/*
 * @file physika_assert.h 
 * @brief Customized assert macro for Physika.
 *        All these asserts are for debug purpose, they do nothing in Relase build
 *        Exception: PHYSIKA_STATIC_ASSERT also works in Release build
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

#ifndef PHYSIKA_CORE_UTILITIES_PHYSIKA_ASSERT_H_
#define PHYSIKA_CORE_UTILITIES_PHYSIKA_ASSERT_H_

#include <iostream>
#include <cassert>
#include "Physika_Core/Utilities/cxx11_support.h"

//assert from standard library
#define PHYSIKA_ASSERT(x) assert(x)

//assert with message
#ifndef NDEBUG
#   define PHYSIKA_MESSAGE_ASSERT(condition,message) \
    do \
    {\
        if(!(condition)) std::cerr<<message<<std::endl; \
        assert((condition)); \
    }while(false)
#else
#   define PHYSIKA_MESSAGE_ASSERT(condition,message) do{}while(false)
#endif

//only error message
#ifndef NDEBUG
#    define PHYSIKA_ERROR(message) \
     do \
     { \
         std::cerr<<message<<std::endl; \
         std::cerr<<"Assertion failed: function "<<__FUNCTION__<<", file "<<__FILE__<<", line "<<__LINE__<<"."<<std::endl; \
         std::exit(EXIT_FAILURE);                                       \
     }while(false)
#else
#    define PHYSIKA_ERROR(message) do{}while(false)
#endif

//compile-time assert, only supported in C++0x or later
//works in Release build as well
#ifndef SUPPORT_STATIC_ASSERT
#    define PHYSIKA_STATIC_ASSERT(condition,message) do{}while(false) 
#else
#    define PHYSIKA_STATIC_ASSERT(condition,message) static_assert(condition,message)
#endif

#endif//PHYSIKA_CORE_UTILITIES_PHYSIKA_ASSERT_H_
