/*
 * @file grid_weight_function_creator.h 
 * @brief grid-based weight function creator. 
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

#ifndef PHYSIKA_CORE_GRID_WEIGHT_FUNCTIONS_GRID_WEIGHT_FUNCTION_CREATOR_H_
#define PHYSIKA_CORE_GRID_WEIGHT_FUNCTIONS_GRID_WEIGHT_FUNCTION_CREATOR_H_

namespace Physika{

template <typename Scalar, int Dim> class GridWeightFunction;

template <typename GridWeightFunctionTypeName>
class GridWeightFunctionCreator
{
public:
    static GridWeightFunction<typename GridWeightFunctionTypeName::ScalarType,GridWeightFunctionTypeName::DimSize>* createGridWeightFunction()
    {
        return new GridWeightFunctionTypeName();
    }
};

}  //end of namespace Physika

#endif
