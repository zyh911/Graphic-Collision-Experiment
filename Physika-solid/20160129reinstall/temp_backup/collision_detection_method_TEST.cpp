#include "Physika_Dynamics/Collidable_Objects/collision_detection_method_TEST.h"

#include <iostream>

namespace Physika {

template <typename Scalar, int Dim>
CollisionDetectionMethodTEST<Scalar, Dim>::CollisionDetectionMethodTEST()
{
}

template <typename Scalar, int Dim>
CollisionDetectionMethodTEST<Scalar, Dim>::CollisionDetectionMethodTEST(RigidBodyDriver<Scalar, Dim> *driver)
{
	driver_ = driver;
}

template <typename Scalar, int Dim>
CollisionDetectionMethodTEST<Scalar, Dim>::~CollisionDetectionMethodTEST()
{
}

template <typename Scalar, int Dim>
void CollisionDetectionMethodTEST<Scalar, Dim>::update()
{
}

template <typename Scalar, int Dim>
void CollisionDetectionMethodTEST<Scalar, Dim>::addCollidableObject(CollidableObject<Scalar, Dim>* object)
{

}

template <typename Scalar, int Dim>
bool CollisionDetectionMethodTEST<Scalar, Dim>::collisionDetection()
{

	RigidBody<Scalar, 3> *chess = dynamic_cast<RigidBody<Scalar, 3> *>(driver_->rigidBody(1));
	RigidBody<Scalar, 3> *car = dynamic_cast<RigidBody<Scalar, 3> *>(driver_->rigidBody(0));

	Vector<Scalar, 3> *chessBottomPoint = new Vector<Scalar, 3>[4];
	chessBottomPoint[0] = chess->globalVertexPosition(0);
	chessBottomPoint[1] = chess->globalVertexPosition(1);
	chessBottomPoint[2] = chess->globalVertexPosition(6);
	chessBottomPoint[3] = chess->globalVertexPosition(7);
	std::cout << "block bottom 4 vertices :\n";
	for (unsigned int i = 0; i < 4; ++ i)
		std::cout << i + 1 << '\t' << chessBottomPoint[i] << '\n';
	//getchar();
	
	Vector<Scalar, 3> normalDirection(0, 1, 0);

	bool existCollide = false;
	for (unsigned int i = 0; i < 4; ++ i)
		if (chessBottomPoint[i][1] <= 0.0)
		{
			ContactPoint<Scalar, Dim> *p = new ContactPoint<Scalar, Dim>(i, 0, 1, chessBottomPoint[i], normalDirection);
			contact_points_.addContactPoint(p);
			existCollide = true;
		}
	
	return existCollide;

}

template class CollisionDetectionMethodTEST<float, 3>;
template class CollisionDetectionMethodTEST<double, 3>;

}