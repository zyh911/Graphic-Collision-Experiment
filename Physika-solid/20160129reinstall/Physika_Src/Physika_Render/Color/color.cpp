/*
 * @file color.cpp 
 * @Brief the color for rendering.
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

#include "Physika_Render/Color/color.h"

namespace Physika{

template <typename Scalar>
Color<Scalar>::Color()
{
    rgba_[0] = rgba_[1] = rgba_[2] = 0;
    rgba_[3] = std::numeric_limits<Scalar>::max();
}

template <typename Scalar>
Color<Scalar>::Color(Scalar red, Scalar green, Scalar blue)
{
    rgba_[0] = red;
    rgba_[1] = green;
    rgba_[2] = blue;
    rgba_[3] = std::numeric_limits<Scalar>::max();
}

template <typename Scalar>
Color<Scalar>::Color(Scalar red, Scalar green, Scalar blue, Scalar alpha)
{
    rgba_[0] = red;
    rgba_[1] = green;
    rgba_[2] = blue;
    rgba_[3] = alpha;
}

template <typename Scalar>
Color<Scalar>::Color(const Color<Scalar> &color)
{
    for(unsigned int i = 0; i < 4; ++i)
        rgba_[i] = color.rgba_[i];
}

template <typename Scalar>
Color<Scalar>& Color<Scalar>::operator= (const Color<Scalar> &color)
{
    for(unsigned int i = 0; i < 4; ++i)
        rgba_[i] = color.rgba_[i];
    return *this;
}

template <typename Scalar>
bool Color<Scalar>::operator== (const Color<Scalar> &color)
{
    return (rgba_[0]==color.rgba_[0]) && (rgba_[1]==color.rgba_[1])
        && (rgba_[2]==color.rgba_[2]) && (rgba_[3]==color.rgba_[3]);
}

template <typename Scalar>
bool Color<Scalar>::operator!= (const Color<Scalar> &color)
{
    return !((*this)==color);
}

template <typename Scalar>
Scalar Color<Scalar>::redChannel() const
{
    return rgba_[0];
}

template <typename Scalar>
Scalar Color<Scalar>::greenChannel() const
{
    return rgba_[1];
}

template <typename Scalar>
Scalar Color<Scalar>::blueChannel() const
{
    return rgba_[2];
}

template <typename Scalar>
Scalar Color<Scalar>::alphaChannel() const
{
    return rgba_[3];
}

template <typename Scalar>
void Color<Scalar>::setRedChannel(Scalar red)
{
    rgba_[0] = red;
}

template <typename Scalar>
void Color<Scalar>::setGreenChannel(Scalar green)
{
    rgba_[1] = green;
}

template <typename Scalar>
void Color<Scalar>::setBlueChannel(Scalar blue)
{
    rgba_[2] = blue;
}

template <typename Scalar>
void Color<Scalar>::setAlphaChannel(Scalar alpha)
{
    rgba_[3] = alpha;
}

template <typename Scalar>
Color<Scalar> Color<Scalar>::Red()
{
    Scalar red = std::numeric_limits<Scalar>::max();
    Scalar green = 0, blue = 0;
    return Color<Scalar>(red,green,blue);
}

template <typename Scalar>
Color<Scalar> Color<Scalar>::Green()
{
    Scalar green = std::numeric_limits<Scalar>::max();
    Scalar red = 0, blue = 0;
    return Color<Scalar>(red,green,blue);
}

template <typename Scalar>
Color<Scalar> Color<Scalar>::Blue()
{
    Scalar blue = std::numeric_limits<Scalar>::max();
    Scalar red = 0, green = 0;
    return Color<Scalar>(red,green,blue);
}

template <typename Scalar>
Color<Scalar> Color<Scalar>::White()
{
    Scalar red = std::numeric_limits<Scalar>::max();
    Scalar green = std::numeric_limits<Scalar>::max();
    Scalar blue = std::numeric_limits<Scalar>::max();
    return Color<Scalar>(red,green,blue);
}

template <typename Scalar>
Color<Scalar> Color<Scalar>::Black()
{
    return Color<Scalar>(0,0,0);
}

template <typename Scalar>
Color<Scalar> Color<Scalar>::Gray()
{
    Scalar red = static_cast<Scalar>(std::numeric_limits<Scalar>::max()/2.0);
    Scalar green = static_cast<Scalar>(std::numeric_limits<Scalar>::max()/2.0);
    Scalar blue = static_cast<Scalar>(std::numeric_limits<Scalar>::max()/2.0);
    return Color<Scalar>(red,green,blue);
}

template <typename Scalar>
Color<Scalar> Color<Scalar>::Yellow()
{
    Scalar red = std::numeric_limits<Scalar>::max();
    Scalar green = std::numeric_limits<Scalar>::max();
    Scalar blue = 0;
    return Color<Scalar>(red,green,blue);
}

template <typename Scalar>
Color<Scalar> Color<Scalar>::Purple()
{
    Scalar red = static_cast<Scalar>(std::numeric_limits<Scalar>::max()/2.0);
    Scalar blue = static_cast<Scalar>(std::numeric_limits<Scalar>::max()/2.0);
    Scalar green = 0;
    return Color<Scalar>(red,green,blue);
}

template <typename Scalar>
Color<Scalar> Color<Scalar>::Cyan()
{
    Scalar red = 0;
    Scalar green = static_cast<Scalar>(std::numeric_limits<Scalar>::max()/2.0);
    Scalar blue = static_cast<Scalar>(std::numeric_limits<Scalar>::max()/2.0);
    return Color<Scalar>(red,green,blue);
}

//explicit instantions
template class Color<signed char>;
template class Color<short>;
template class Color<int>;
template class Color<float>;
template class Color<double>;
template class Color<unsigned char>;
template class Color<unsigned short>;
template class Color<unsigned int>;

}  //end of namespace Physika
