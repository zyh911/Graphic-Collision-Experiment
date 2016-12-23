
#ifndef PHYSIKA_DYNAMICS_COLLIDABLE_OBJECTS_COLLISION_DETECTION_METHOD_TEST_H_
#define PHYSIKA_DYNAMICS_COLLIDABLE_OBJECTS_COLLISION_DETECTION_METHOD_TEST_H_

#include "Physika_Dynamics/Collidable_Objects/collision_detection_method.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_3d.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_driver.h"
#include <vector>

namespace Physika{

template <typename Scalar,int Dim>
class CollisionDetectionMethodTEST : public CollisionDetectionMethod<Scalar, Dim>
{
public:
    CollisionDetectionMethodTEST();
	CollisionDetectionMethodTEST(RigidBodyDriver<Scalar, Dim> *diver);
    ~CollisionDetectionMethodTEST();
    void update();
    void addCollidableObject(CollidableObject<Scalar, Dim>* object);
    bool collisionDetection();

	void addRigidBody(RigidBody<Scalar, Dim> *rigid_body);

	//vector<CollidableObject<Scalar, Dim> *> collidable_object_;
	//vector<RigidBody<Scalar, Dim> *> rigid_body_;
	RigidBodyDriver<Scalar, Dim> *driver_;

};

}

#endif //PHYSIKA_DYNAMICS_COLLIDABLE_OBJECTS_COLLISION_DETECTION_METHOD_TEST_H_