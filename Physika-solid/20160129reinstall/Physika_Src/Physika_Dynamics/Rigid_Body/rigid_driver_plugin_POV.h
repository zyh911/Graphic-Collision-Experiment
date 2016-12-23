/*
 * @file rigid_driver_plugin_POV.h 
 * @POV plugin of rigid body driver. output scene to pov script
 * @author Wei Chen
 * 
 * This file is part of Physika, a versatile physics simulation library.
 * Copyright (C) 2013 Physika Group.
 *
 * This Source Code Form is subject to the terms of the GNU General Public License v2.0. 
 * If a copy of the GPL was not distributed with this file, you can obtain one at:
 * http://www.gnu.org/licenses/gpl-2.0.html
 *
 */

#ifndef PHYSIKA_DYNAMICS_RIGID_BODY_RIGID_DRIVER_PLUGIN_POV
#define PHYSIKA_DYNAMICS_RIGID_BODY_RIGID_DRIVER_PLUGIN_POV

#include <string>
#include <fstream>
#include "Physika_Dynamics/Rigid_Body/rigid_driver_plugin.h"
#include "Physika_Core/Transform/transform_3d.h"
#include "Physika_Geometry/Boundary_Meshes/surface_mesh.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_3d.h"

namespace Physika
{
template <typename Scalar,int Dim>
class RigidDriverPluginPOV : public RigidDriverPlugin<Scalar, Dim>
{
public:
    RigidDriverPluginPOV();
    ~RigidDriverPluginPOV();

    //functions called in driver
    void onBeginFrame(unsigned int frame);
    void onEndFrame(unsigned int frame);
    void onBeginTimeStep(Scalar time, Scalar dt);
    void onEndTimeStep(Scalar time, Scalar dt);

    void onBeginRigidStep(unsigned int step, Scalar dt);//replace the original onBeginTimeStep in rigid body simulation
    void onEndRigidStep(unsigned int step, Scalar dt);//replace the original onEndTimeStep in rigid body simulation

    void onAddRigidBody(RigidBody<Scalar, Dim>* rigid_body);
    void onBeginCollisionDetection();
    void onEndCollisionDetection();

    void setOutputFilenameBase(const std::string & filename_base);
    inline void setOutputInterval(int interval){output_interval_ = interval;}
    inline void setMaxOutputStep(int step){max_output_step_ = step;}

protected:
    void saveMeshToFile(std::fstream & fileout, RigidBody<Scalar, 3>* rigid_body);//,const SurfaceMesh<Scalar> *mesh, Transform<Scalar, Dim> * transform);
    void saveSceneConfigToFile(std::fstream & fileout);
    int output_interval_;
    int max_output_step_;
    std::string output_filename_base_; //format: output_filename_base+stepid+'.pov', default: demo

};

} // end of namespace Physika

#endif // PHYSIKA_DYNAMICS_RIGID_BODY_RIGID_DRIVER_PLUGIN_POV