/*
 * @file  bounding_volume_kdop18.h
 * @18-DOP bounding volume of a collidable object
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

#ifndef PHYSIKA_GEOMETRY_BOUNDING_VOLUME_BOUNDING_VOLUME_KDOP18_H_
#define PHYSIKA_GEOMETRY_BOUNDING_VOLUME_BOUNDING_VOLUME_KDOP18_H_

#include "Physika_Geometry/Bounding_Volume/bounding_volume.h"

namespace Physika{

template <typename Scalar, int Dim> class Vector;

template <typename Scalar>
class BoundingVolumeKDOP18 : public BoundingVolume<Scalar, 3>
{
public:
	//constructors && deconstructors
	BoundingVolumeKDOP18();
	~BoundingVolumeKDOP18();

	//set
	void setBoundingVolume(const BoundingVolume<Scalar, 3>* const bounding_volume);
	void setBoundingVolume(const Vector<Scalar, 3>& point);
	void setBoundingVolume(const Vector<Scalar, 3>& point_a, const Vector<Scalar, 3>& point_b);
	typename BoundingVolumeInternal::BVType bvType() const;

	//basic operation
	bool isOverlap(const BoundingVolume<Scalar, 3>* const bounding_volume) const;
	bool isOverlap(const BoundingVolume<Scalar, 3>* const bounding_volume, BoundingVolume<Scalar, 3>* return_volume) const;
	bool isInside(const Vector<Scalar, 3> &point) const;
	Vector<Scalar, 3> center() const;
	Scalar width() const;
	Scalar height() const;
	Scalar depth() const;
	void setEmpty();

	//union operation
	void unionWith(const Vector<Scalar, 3> &point);
	void unionWith(const BoundingVolume<Scalar, 3>* const bounding_volume);
	void obtainUnion(const BoundingVolume<Scalar, 3>* const bounding_volume_lhs, const BoundingVolume<Scalar, 3>* const bounding_volume_rhs);

protected:
	//faces of a 18-DOP
	Scalar dist_[18];

	//internal functions
	void getDistances(const Vector<Scalar, 3>& point, Scalar& d3, Scalar& d4, Scalar& d5, Scalar& d6, Scalar& d7, Scalar& d8) const;
	void getDistances(const Vector<Scalar, 3>& point, Scalar d[]) const;
};

}  //end of namespace Physika

#endif  //PHYSIKA_GEOMETRY_BOUNDING_VOLUME_BOUNDING_VOLUME_KDOP18_H_
