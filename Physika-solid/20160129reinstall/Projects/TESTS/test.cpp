#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <map>
#include <vector>

#include "Physika_IO/Surface_Mesh_IO/obj_mesh_io.h"
#include "Physika_Dynamics/Collidable_Objects/mesh_based_collidable_object.h"
#include "Physika_Geometry/Boundary_Meshes/surface_mesh.h"
#include "Physika_Geometry/Bounding_Volume/bvh_base.h"
#include "Physika_Geometry/Bounding_Volume/bvh_node_base.h"
#include "Physika_Geometry/Bounding_Volume/object_bvh.h"
#include "Physika_Geometry/Bounding_Volume/object_bvh_node.h"
#include "Physika_Geometry/Bounding_Volume/scene_bvh.h"
#include "Physika_Geometry/Bounding_Volume/scene_bvh_node.h"
#include "Physika_Core/Vectors/vector_2d.h"
#include "Physika_Core/Vectors/vector_3d.h"
#include "Physika_Geometry/Bounding_Volume/bounding_volume_kdop18.h"
#include "Physika_Geometry/Bounding_Volume/bounding_volume.h"
#include "Physika_Dynamics/Collidable_Objects/collision_pair_manager.h"
#include "Physika_Core/Utilities/math_utilities.h"
#include "Physika_Core/Timer/timer.h"
#include "Physika_Dynamics/Collidable_Objects/collision_pair.h"

#include <GL/freeglut.h>
#include <GL/glui.h>
#include "Physika_Render/OpenGL_Primitives/opengl_primitives.h"
#include "Physika_GUI/Glut_Window/glut_window.h"
#include "Physika_GUI/Glui_Window/glui_window.h"
#include "Physika_Render/Surface_Mesh_Render/surface_mesh_render.h"

#include "Physika_Dynamics/Rigid_Body/rigid_body.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_2d.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_3d.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_driver.h"
#include "Physika_Dynamics/Rigid_Body/rigid_driver_plugin.h"
#include "Physika_Dynamics/Rigid_Body/rigid_driver_plugin_render.h"
#include "Physika_Dynamics/Rigid_Body/rigid_driver_plugin_print.h"
#include "Physika_Dynamics/Rigid_Body/rigid_driver_plugin_motion.h"
#include "Physika_Dynamics/Rigid_Body/rigid_driver_plugin_POV.h"
#include "Physika_Dynamics/Collidable_Objects/collision_detection_method_DTBVH.h"
#include "Physika_Dynamics/Collidable_Objects/collision_detection_method_CCD.h"
#include "Physika_Dynamics/Rigid_Body/compressed_matrix.h"

#include "Physika_Dynamics/Rigid_Body/rigid_response_method_QCE.h"

using namespace std;
using namespace Physika;

double randD()
{
	return double(rand()) / double(RAND_MAX);
}

void testBlock(Vector<double, 3> _angularPosition)
{

	SurfaceMesh<double> mesh_box;
	if(!ObjMeshIO<double>::load(string("Models/box.obj"), &mesh_box))
		exit(1);
	mesh_box.computeAllFaceNormals();

	RigidBody<double,3> block(&mesh_box);
	block.setScale(Vector<double, 3>(1.0, 1.0, 1.0));
	block.setCoeffRestitution(1.0);
	block.setCoeffFriction(0.0);

	RigidBody<double,3> floor(&mesh_box);
	floor.setScale(Vector<double, 3>(2000, 1, 2000));
	floor.setTranslation(Vector<double, 3>(100, -1, 0));
	floor.setCoeffRestitution(1.0);
	floor.setCoeffFriction(0.0);
	floor.setFixed(true);

	RigidBodyDriver<double, 3> driver;
	driver.setGravity(3.0);
	CollisionDetectionMethodCCD<double, 3> *ccd = new CollisionDetectionMethodCCD<double, 3>;   // 默认碰撞检测是离散DTBVH，此处设置为连续CCD
	driver.setCollisionDetectionMethod(ccd);
	RigidResponseMethodQCE<double, 3> *qce = new RigidResponseMethodQCE<double, 3>();  //默认碰撞响应是BLCP，此处设置为QCE
	driver.setCollisionResponseMethod(qce);

	RigidBody<double, 3>* object;

	object = new RigidBody<double,3>(floor);
	driver.addRigidBody(object);

	object = new RigidBody<double, 3>(block);
	object -> setTranslation(Vector<double, 3>(0.0, 1.0, 0.0));
	object -> setRotation(_angularPosition);
	//object -> setGlobalAngularVelocity(_angularPosition);
	driver.addRigidBody(object);

	driver.setDt(0.02);

	GlutWindow glut_window;
	std::cout<<"Window name: "<<glut_window.name()<<"\n";
	std::cout<<"Window size: "<<glut_window.width()<<"x"<<glut_window.height()<<"\n";
	glut_window.setCameraPosition(Vector<double, 3>(4.0, 0.02, 0.0));
	glut_window.setCameraFocusPosition(Vector<double, 3>(0.0, 0.02, 0.0));
	//glut_window.setCameraPosition(Vector<double, 3>(0, 0, 5));
	//glut_window.setCameraFocusPosition(Vector<double, 3>(0, 0, 0));
	glut_window.setCameraFarClip(10000);
	glut_window.setCameraNearClip(1.0);
	glut_window.setCameraFOV(40);

	RigidDriverPluginRender<double, 3>* plugin = new RigidDriverPluginRender<double, 3>();
	plugin->setWindow(&glut_window);
	driver.addPlugin(plugin);
	//plugin->disableRenderSolidAll();
	//plugin->enableRenderWireframeAll();
	//plugin->enableRenderContactFaceAll();
	//plugin->enableRenderContactNormalAll();

	glut_window.createWindow();

}

void testCylinder()
{
	// load meshes
	SurfaceMesh<double> mesh_box_high;
	if(!ObjMeshIO<double>::load(string("Models/box_high.obj"), &mesh_box_high))
		exit(1);
	mesh_box_high.computeAllFaceNormals();
	mesh_box_high.computeAllVertexNormals(SurfaceMesh<double>::WEIGHTED_FACE_NORMAL);

	SurfaceMesh<double> mesh_cylinder;
	if(!ObjMeshIO<double>::load(string("Models/cylinder2.obj"), &mesh_cylinder))
		exit(1);
	mesh_cylinder.computeAllFaceNormals();
	mesh_cylinder.computeAllVertexNormals(SurfaceMesh<double>::WEIGHTED_FACE_NORMAL);

	SurfaceMesh<double> mesh_slope;
	if(!ObjMeshIO<double>::load(string("Models/box_0811.obj"), &mesh_slope))
		exit(1);
	mesh_slope.computeAllFaceNormals();
	mesh_slope.computeAllVertexNormals(SurfaceMesh<double>::WEIGHTED_FACE_NORMAL);

	RigidBodyDriver<double, 3> driver;  //刚体的一个管理对象
	driver.setGravity(10.0);
	driver.setDt(0.02) ; // 时间步长

	RigidBody<double, 3>* object; // a general object for each rigid body
	RigidBody<double, 3> *cylinderPointer;
	Vector<double, 3> spinSpeed(0, 0, 10), littleSpeed(0, -0.1, 0);
	
	//cylinder
	object = new RigidBody<double,3>(&mesh_cylinder);    // <*,*> 数据类型（精度）， 维数
	object->setScale(Vector<double, 3>(0.5, 0.5, 0.5));
	object->setTranslation(Vector<double, 3>(-3, 9, -10));
	object->setProperty(&mesh_cylinder, 1000);
	object->setRotation(Vector<double, 3>(0, 3.14159265 / 2, 0));
	object->setCoeffRestitution(1.0);
	object->setCoeffFriction(10);
	cylinderPointer = object;
	driver.addRigidBody(object);
	
	
	//floor
	object = new RigidBody<double, 3>(&mesh_box_high);
	object->setScale(Vector<double, 3>(30, 0.05, 30));
	object->setTranslation(Vector<double, 3>(0, -2.25, 0));
	object->setProperty(&mesh_box_high, 1000);
	object->setCoeffRestitution(0.00001);
	object->setCoeffFriction(0.000001);
	object->setFixed(true);
	driver.addRigidBody(object);
	
	
	//slope
	object = new RigidBody<double, 3>(&mesh_slope);
	object->setScale(Vector<double, 3>(0.3, 0.5, 0.4));
	object->setTranslation(Vector<double, 3>(0, 0, 5));
	object->setProperty(&mesh_slope, 0.1);
	object->setRotation(Vector<double, 3>(0, 0, 3.14159265 / 2 * 3));
	object->setGlobalTranslationVelocity(Vector<double, 3>(0, 0, 0));
	object->setCoeffRestitution(0.0001);
	object->setCoeffFriction(0.000001);
	driver.addRigidBody(object);
	
	
	GlutWindow glut_window;
	std::cout<<"Window name: "<<glut_window.name()<<"\n";
	std::cout<<"Window size: "<<glut_window.width()<<"x"<<glut_window.height()<<"\n";
	glut_window.setCameraPosition(Vector<double, 3>(0, 10, 15));
	glut_window.setCameraFocusPosition(Vector<double, 3>(4, 2, 0));
	glut_window.setCameraFarClip(10000);
	glut_window.setCameraNearClip(1.0);
	glut_window.setCameraFOV(80);

	RigidDriverPluginRender<double, 3>* pluginRender = new RigidDriverPluginRender<double, 3>();   // 显示插件
	pluginRender->setWindow(&glut_window);
	driver.addPlugin(pluginRender);
	//pluginRender->disableRenderSolidAll();
	//pluginRender->enableRenderWireframeAll();
	//pluginRender->enableRenderContactFaceAll();
	//pluginRender->enableRenderContactNormalAll();

	RigidDriverPluginMotion<double> *pluginMotion = new RigidDriverPluginMotion<double>();    // 设置运动方式，初始运动为匀速运动，0.3开始运动,4.8秒停止
	pluginMotion->setConstantRotation(cylinderPointer, spinSpeed);
	//pluginMotion->addTimerTask(cylinderPointer, 0.1, RigidBodyTimerTask<double>::ADD_CONSTANT_TRANSLATION, littleSpeed, 1);
	pluginMotion->setConstantTranslation(cylinderPointer, littleSpeed);
	driver.addPlugin(pluginMotion);

	//RigidDriverPluginPOV
	//RigidDriverPluginPOV<double, 3> *pluginPOV = new RigidDriverPluginPOV<double, 3>();
	//driver.addPlugin(pluginPOV);

	glut_window.createWindow();
}
void testCar()
{
	// load meshes
	SurfaceMesh<double> mesh_box_high;
	if(!ObjMeshIO<double>::load(string("Models/box_high.obj"), &mesh_box_high))
		exit(1);
	mesh_box_high.computeAllFaceNormals();
	mesh_box_high.computeAllVertexNormals(SurfaceMesh<double>::WEIGHTED_FACE_NORMAL);

	SurfaceMesh<double> mesh_box;
	if(!ObjMeshIO<double>::load(string("Models/box.obj"), &mesh_box))
		exit(1);
	mesh_box.computeAllFaceNormals();
	mesh_box.computeAllVertexNormals(SurfaceMesh<double>::WEIGHTED_FACE_NORMAL);

	SurfaceMesh<double> mesh_pawn;
	if(!ObjMeshIO<double>::load(string("Models/centeredPawn.obj"), &mesh_pawn))
		exit(1);
	mesh_pawn.computeAllFaceNormals();
	mesh_pawn.computeAllVertexNormals(SurfaceMesh<double>::WEIGHTED_FACE_NORMAL);

	SurfaceMesh<double> mesh_car;
	if(!ObjMeshIO<double>::load(string("Models/car1_new0.obj"), &mesh_car))
		exit(1);
	mesh_car.computeAllFaceNormals();
	mesh_car.computeAllVertexNormals(SurfaceMesh<double>::WEIGHTED_FACE_NORMAL);

	RigidBodyDriver<double, 3> driver;  //刚体的一个管理对象
	driver.setGravity(5.0);
	driver.setDt(0.015) ; // 时间步长
	
	RigidBody<double, 3>* object; // a general object for each rigid body
	RigidBody<double, 3> *carPointer; // specially record the car, for later setting timer motion
	Vector<double, 3> constantSpeed(2.5, 0, 0), stay(0, 0, 0);

	
	//car
	object = new RigidBody<double,3>(&mesh_car);    // <*,*> 数据类型（精度）， 维数
	object->setScale(Vector<double, 3>(0.15, 0.15, 0.15));
	object->setTranslation(Vector<double, 3>(0, -6, 0));
	//object->setTranslation(Vector<double, 3>(0, -1, 0));
	object->setCoeffRestitution(1.0);
	object->setCoeffFriction(0.0);
	carPointer = object;
	driver.addRigidBody(object);
	
	/*
	//carnew
	object = new RigidBody<double,3>(&mesh_car);    // <*,*> 数据类型（精度）， 维数
	object->setScale(Vector<double, 3>(1, 1, 1));
	object->setTranslation(Vector<double, 3>(0, 0, 0));
	//object->setTranslation(Vector<double, 3>(0, -1, 0));
	object->setCoeffRestitution(1.0);
	object->setCoeffFriction(0.0);
	carPointer = object;
	driver.addRigidBody(object);
	*/
	
	// floor
	object = new RigidBody<double, 3>(&mesh_box_high);
	object->setScale(Vector<double, 3>(20, 0.05, 20));
	object->setTranslation(Vector<double, 3>(0, -8, 0));
	object->setCoeffRestitution(0.1);
	object->setCoeffFriction(0.1);
	object->setFixed(true);
	driver.addRigidBody(object);
	
	// pawn0
	object = new RigidBody<double, 3>(&mesh_pawn);
	object->setScale(Vector<double, 3>(1.0, 1.0, 1.0));
	object->setTranslation(Vector<double, 3>(-5, 0.54, 0.1));
	object->setRotation(Vector<double, 3>(0, 3.14, 0));
	object->setCoeffRestitution(0.05);
	object->setCoeffFriction(0.1);
	driver.addRigidBody(object);

	// pawn1
	object = new RigidBody<double, 3>(&mesh_pawn);
	object->setScale(Vector<double, 3>(1.0, 1.0, 1.0));
	object->setTranslation(Vector<double, 3>(-3, 0.54, 0.1));
	object->setCoeffRestitution(0.05);
	object->setCoeffFriction(0.1);
	driver.addRigidBody(object);

	// pawn2
	object = new RigidBody<double, 3>(&mesh_pawn);
	object->setScale(Vector<double, 3>(1.0, 1.0, 1.0));
	object->setTranslation(Vector<double, 3>(-0.5, 0.51, -1.0));
	//object->setRotation(Quaternion<double>(0.0821714, -0.402524, 0.777342, 0.476405));
	object->setRotation(Quaternion<double>(0.0821714, 0.402524, 0.777342, 0.476405));
	//object->setRotation(Vector<double, 3>(-0.2, 2.0, 3.0));
	//object->setGlobalAngularVelocity(Vector<double, 3>(5, 0, -5));
	object->setCoeffRestitution(0.05);
	object->setCoeffFriction(0.2);     // 两个物体碰撞时，摩擦系数取较大的，恢复系数取较小
	driver.addRigidBody(object);
	
	GlutWindow glut_window;
	std::cout<<"Window name: "<<glut_window.name()<<"\n";
	std::cout<<"Window size: "<<glut_window.width()<<"x"<<glut_window.height()<<"\n";
	glut_window.setCameraPosition(Vector<double, 3>(-2, 10, 15));
	glut_window.setCameraFocusPosition(Vector<double, 3>(4, 2, 0));
	glut_window.setCameraFarClip(10000);
	glut_window.setCameraNearClip(1.0);
	glut_window.setCameraFOV(80);

	RigidDriverPluginRender<double, 3>* pluginRender = new RigidDriverPluginRender<double, 3>();   // 显示插件
	pluginRender->setWindow(&glut_window);
	driver.addPlugin(pluginRender);
	//pluginRender->disableRenderSolidAll();
	//pluginRender->enableRenderWireframeAll();
	//pluginRender->enableRenderContactFaceAll();
	//pluginRender->enableRenderContactNormalAll();

	RigidDriverPluginMotion<double> *pluginMotion = new RigidDriverPluginMotion<double>();    // 设置运动方式，初始运动为匀速运动，0.3开始运动,4.8秒停止
	pluginMotion->setConstantTranslation(carPointer, stay); // set the car stay put from the beginning
	pluginMotion->addTimerTask(carPointer, 0.3, RigidBodyTimerTask<double>::ADD_CONSTANT_TRANSLATION, constantSpeed, 1); // set the car to move constantSpped from 0.3s
	pluginMotion->addTimerTask(carPointer, 4.8, RigidBodyTimerTask<double>::ADD_CONSTANT_TRANSLATION, stay, 1); // set the cat to stay put from 4.8s
	driver.addPlugin(pluginMotion);

	//RigidDriverPluginPOV
	//RigidDriverPluginPOV<double, 3> *pluginPOV = new RigidDriverPluginPOV<double, 3>();
	//driver.addPlugin(pluginPOV);

	glut_window.createWindow();

}

void testBowl()
{
	SurfaceMesh<double> mesh_armadillo;
	if(!ObjMeshIO<double>::load(string("Models/armadillo.obj"), &mesh_armadillo))
		exit(1);
	mesh_armadillo.computeAllFaceNormals();
	mesh_armadillo.computeAllVertexNormals(SurfaceMesh<double>::WEIGHTED_FACE_NORMAL);

	SurfaceMesh<double> mesh_elephant;
	if(!ObjMeshIO<double>::load(string("Models/elephant1.obj"), &mesh_elephant))
		exit(1);
	mesh_elephant.computeAllFaceNormals();
	mesh_elephant.computeAllVertexNormals(SurfaceMesh<double>::WEIGHTED_FACE_NORMAL);

	SurfaceMesh<double> mesh_pawn1;
	if(!ObjMeshIO<double>::load(string("Models/MyPawn1.obj"), &mesh_pawn1))
		exit(1);
	mesh_pawn1.computeAllFaceNormals();
	mesh_pawn1.computeAllVertexNormals(SurfaceMesh<double>::WEIGHTED_FACE_NORMAL);

	SurfaceMesh<double> mesh_pawn2;
	if(!ObjMeshIO<double>::load(string("Models/MyPawn2.obj"), &mesh_pawn2))
		exit(1);
	mesh_pawn2.computeAllFaceNormals();
	mesh_pawn2.computeAllVertexNormals(SurfaceMesh<double>::WEIGHTED_FACE_NORMAL);
	
	SurfaceMesh<double> mesh_bunny;
	if(!ObjMeshIO<double>::load(string("Models/bunny3.obj"), &mesh_bunny))
		exit(1);
	mesh_bunny.computeAllFaceNormals();
	mesh_bunny.computeAllVertexNormals(SurfaceMesh<double>::WEIGHTED_FACE_NORMAL);

	SurfaceMesh<double> mesh_ball;
	if(!ObjMeshIO<double>::load(string("Models/ball1.obj"), &mesh_ball))
		exit(1);
	mesh_ball.computeAllFaceNormals();
	mesh_ball.computeAllVertexNormals(SurfaceMesh<double>::WEIGHTED_FACE_NORMAL);

	/*
	SurfaceMesh<double> mesh_box;
	if(!ObjMeshIO<double>::load(string("Models/box.obj"), &mesh_box))
		exit(1);
	mesh_box.computeAllFaceNormals();
	mesh_box.computeAllVertexNormals(SurfaceMesh<double>::WEIGHTED_FACE_NORMAL);
	*/

	SurfaceMesh<double> mesh_bowl;
	if(!ObjMeshIO<double>::load(string("Models/bowl2.obj"), &mesh_bowl))
		exit(1);
	mesh_bowl.computeAllFaceNormals();
	mesh_bowl.computeAllVertexNormals(SurfaceMesh<double>::WEIGHTED_FACE_NORMAL);

	RigidBodyDriver<double, 3> driver;
	driver.setGravity(7.0);
	driver.setDt(0.002);
	//driver.setCollisionDetectionMethod(new CollisionDetectionMethodGroup<double, 3>());

	RigidBody<double, 3> *object;
	RigidBody<double, 3> *bowlPointer;
	Vector<double, 3> zero(0, 0, 0), constantSpin(0, 2, 0), constantAcceleration(0, 0.5, 0);

	// bowl
	object = new RigidBody<double, 3>(&mesh_bowl);
	
	//properties when using bowl2.obj
	object->setTranslation(Vector<double, 3>(0.0, -3.0, -28));
	object->setProperty(&mesh_bowl, 1000); 
	object->setScale(Vector<double, 3>(1.3, 1.3, 1.3));
	
	/*
	object->setTranslation(Vector<double, 3>(0.0, -3.0, 0));
	object->setProperty(&mesh_bowl, 1000); 
	object->setScale(Vector<double, 3>(0.4, 0.4, 0.4));
	*/
	object->setCoeffFriction(0.0);
	object->setCoeffRestitution(1.0);
	object->setFixed(true);
	bowlPointer = object;
	driver.addRigidBody(object);     // addRigidBody 添加自始至终存在的物体
	//driver.addTimerRigidBody(object, 0);
	
	for (unsigned int layer = 0; layer < 5; ++ layer)
	{
		double y = 10.0;
		
		if (layer % 2 == 0)
		{
			for (double x = -6; x <= 3; x += 3)
			{
				for (double z = -6; z <= 3; z += 3)
				{
					object = new RigidBody<double, 3>(&mesh_pawn1);
					object->setScale(Vector<double, 3>(1, 1, 1));
					object->setTranslation(Vector<double, 3>(x + 6, y, z - 9));
					object->setGlobalTranslationVelocity(Vector<double, 3>(0, -5, 0));
					object->setCoeffFriction(0.4);
					object->setCoeffRestitution(0.2);
					driver.addTimerRigidBody(object, layer * 0.5 + 0.5);   // TimerRigidBody 添加中间时刻加入的物体，由第二个参数指定加入时间
				}
			}
		}
		else
		{
			for (double x = -6; x <= 3; x += 3)
			{
				for (double z = -6; z <= 3; z += 3)
				{
					object = new RigidBody<double, 3>(&mesh_pawn2);
					object->setScale(Vector<double, 3>(1, 1, 1));
					object->setTranslation(Vector<double, 3>(x + 6, y, z - 9));
					object->setGlobalTranslationVelocity(Vector<double, 3>(0, -5, 0));
					object->setCoeffFriction(0.4);
					object->setCoeffRestitution(0.2);
					driver.addTimerRigidBody(object, layer * 0.5 + 0.5);   // TimerRigidBody 添加中间时刻加入的物体，由第二个参数指定加入时间
				}
			}
		}
	}
	
	
	for(double i = 2.5; i <= 2.5; i += 0.5)//upper bound should be 4
	{
		object = new RigidBody<double, 3>(&mesh_armadillo);
		object->setScale(Vector<double, 3>(0.02, 0.02, 0.02));
		object->setProperty(&mesh_armadillo, 0.1);
		object->setTranslation(Vector<double, 3>(10, 16, -10));
		object->setGlobalTranslationVelocity(Vector<double, 3>(-3, -2, 3));
		object->setGlobalAngularVelocity(Vector<double, 3>(0, 2, 0.3));
		object->setCoeffFriction(0.4);
		object->setCoeffRestitution(0.2);
		driver.addTimerRigidBody(object, i);
	}

	
	for(double i = 2; i <= 2; i += 0.5)//upper bound should be 3.5
	{
		object = new RigidBody<double, 3>(&mesh_elephant);
		object->setScale(Vector<double, 3>(0.1, 0.1, 0.1));
		object->setProperty(&mesh_elephant, 0.1);
		object->setTranslation(Vector<double, 3>(0, 25, 15));
		object->setGlobalTranslationVelocity(Vector<double, 3>(0, -5, -5));
		object->setGlobalAngularVelocity(Vector<double, 3>(0, 2, 0.3));
		object->setCoeffFriction(0.4);
		object->setCoeffRestitution(0.2);
		driver.addTimerRigidBody(object, i);
	}
	
	
	for(double i = 2; i <= 3.5; i += 0.3)
	{
		object = new RigidBody<double, 3>(&mesh_ball);
		object->setScale(Vector<double, 3>(0.03, 0.03, 0.03));
		object->setProperty(&mesh_ball, 0.1);
		object->setTranslation(Vector<double, 3>(-22, 20, 0));
		object->setGlobalTranslationVelocity(Vector<double, 3>(8, -2, 0));
		object->setCoeffFriction(0.4);
		object->setCoeffRestitution(0.2);
		driver.addTimerRigidBody(object, i);
	}
	
	/*
	for(double i = 1.5; i <= 3; i += 0.3)
	{
		object = new RigidBody<double, 3>(&mesh_bunny);
		object->setScale(Vector<double, 3>(0.0015, 0.0015, 0.0015));
		object->setProperty(&mesh_bunny, 1);
		object->setTranslation(Vector<double, 3>(-5, 30, 17));
		object->setGlobalTranslationVelocity(Vector<double, 3>(2, -4, -5));
		object->setGlobalAngularVelocity(Vector<double, 3>(0, -4, 5));
		object->setCoeffFriction(0.4);
		object->setCoeffRestitution(0.2);
		driver.addTimerRigidBody(object, i);
	}
	*/
	
	for(double i = 1.6; i <= 2.8; i += 0.4)
	{
		object = new RigidBody<double, 3>(&mesh_bunny);
		object->setScale(Vector<double, 3>(0.0015, 0.0015, 0.0015));
		object->setProperty(&mesh_bunny, 1);
		object->setTranslation(Vector<double, 3>(-4, 30, -10));
		object->setGlobalTranslationVelocity(Vector<double, 3>(4, -4, 4));
		object->setGlobalAngularVelocity(Vector<double, 3>(0, -4, 5));
		object->setCoeffFriction(0.4);
		object->setCoeffRestitution(0.2);
		driver.addTimerRigidBody(object, i);
	}
	

	GlutWindow glut_window;
	std::cout<<"Window name: "<<glut_window.name()<<"\n";
	std::cout<<"Window size: "<<glut_window.width()<<"x"<<glut_window.height()<<"\n";
	glut_window.setCameraPosition(Vector<double, 3>(15, 18, 0));
	glut_window.setCameraFocusPosition(Vector<double, 3>(0, 0, 0));
	glut_window.setCameraFarClip(10000);
	glut_window.setCameraNearClip(1.0);
	glut_window.setCameraFOV(80);

	RigidDriverPluginRender<double, 3>* pluginRender = new RigidDriverPluginRender<double, 3>();
	pluginRender->setWindow(&glut_window);
	driver.addPlugin(pluginRender);
	pluginRender->disableRenderSolidAll();
	pluginRender->enableRenderWireframeAll();
	//pluginRender->enableRenderContactFaceAll();
	pluginRender->enableRenderContactNormalAll();

	// RigidDriverPluginMotion用于设定指定物体的指定运动方式的，不受物理规律改变
	RigidDriverPluginMotion<double> *pluginMotion = new RigidDriverPluginMotion<double>();
	pluginMotion->setConstantRotation(bowlPointer, constantSpin);
	pluginMotion->setConstantRotationAcceleration(bowlPointer, constantAcceleration);
	pluginMotion->addTimerTask(bowlPointer, 10.0, RigidBodyTimerTask<double>::ADD_CONSTANT_ROTATION_ACCELERATION, zero);
//	pluginMotion->addTimerTask(bowlPointer, 15.0, RigidBodyTimerTask<double>::SET_FREE, zero);
	driver.addPlugin(pluginMotion);

	//RigidDriverPluginPOV
	RigidDriverPluginPOV<double, 3> *pluginPOV = new RigidDriverPluginPOV<double, 3>();
	driver.addPlugin(pluginPOV);


	glut_window.createWindow();

}

int main()
{
	//testCylinder();
	testCar();
	//testBowl();
	//testBlock(Vector<double, 3>(1.1, 1.4, -2.7));

	return 0;
}
