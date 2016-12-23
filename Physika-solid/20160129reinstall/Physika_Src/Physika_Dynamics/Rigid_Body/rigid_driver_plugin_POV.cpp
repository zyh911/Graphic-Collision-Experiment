/*
 * @file rigid_driver_plugin_POV.cpp
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

#include <sstream>
#include <string>
#include "rigid_driver_plugin_POV.h"
#include "Physika_Dynamics/Rigid_Body/rigid_driver_plugin.h"
#include "Physika_Dynamics/Rigid_Body/rigid_body_driver.h"
#include "Physika_Core/Utilities/File_Utilities/file_path_utilities.h"
#include "Physika_Core/Utilities/File_Utilities/parse_line.h"


namespace Physika{

template <typename Scalar, int Dim>
RigidDriverPluginPOV<Scalar,Dim>::RigidDriverPluginPOV()
    :output_filename_base_("pov/demo"),
    output_interval_(1),
    max_output_step_(0)
{

}

template <typename Scalar, int Dim>
RigidDriverPluginPOV<Scalar,Dim>::~RigidDriverPluginPOV()
{

}

template <typename Scalar, int Dim>
void RigidDriverPluginPOV<Scalar, Dim>::onBeginFrame(unsigned int frame)
{

}

template <typename Scalar, int Dim>
void RigidDriverPluginPOV<Scalar, Dim>::onEndFrame(unsigned int frame)
{

}

template <typename Scalar, int Dim>
void RigidDriverPluginPOV<Scalar, Dim>::onBeginTimeStep(Scalar time, Scalar dt)
{

}

template <typename Scalar, int Dim>
void RigidDriverPluginPOV<Scalar, Dim>::onEndTimeStep(Scalar time, Scalar dt)
{

}

template <typename Scalar, int Dim>
void RigidDriverPluginPOV<Scalar, Dim>::onAddRigidBody(RigidBody<Scalar, Dim>* rigid_body)
{

}

template <typename Scalar, int Dim>
void RigidDriverPluginPOV<Scalar, Dim>::onBeginCollisionDetection()
{

}

template <typename Scalar, int Dim>
void RigidDriverPluginPOV<Scalar, Dim>::onEndCollisionDetection()
{

}

template <typename Scalar, int Dim>
void RigidDriverPluginPOV<Scalar, Dim>::onBeginRigidStep(unsigned int step, Scalar dt)
{

}

template <typename Scalar, int Dim>
void RigidDriverPluginPOV<Scalar, Dim>::onEndRigidStep(unsigned int step, Scalar dt)
{
    /*
    int begin = 600;
    int gap = 70;
    int max_id = 6 * 6 * 4;
    if(step >= begin && step < begin + 16 * gap)
    {
        this->rigid_driver_->rigidBody(1 + 6 * 6 * 4 + (step - begin) / gap)->setFixed(false);
        max_id = 1 + 6 * 6 * 4 + (step - begin) / gap;
    }
    if(step >= begin + 16 * gap)
        max_id = this->rigid_driver_->numRigidBody() - 1;*/

    if(step % output_interval_ != 0)
        return;
    if(max_output_step_ != 0 && step > max_output_step_)
        return;
    
    step /= output_interval_;
    srand(0);
    
    std::stringstream sstream;
    std::string stepid;
    sstream<<step;
    sstream>>stepid;

    std::string filename = output_filename_base_+stepid+".pov";

    std::fstream fileout(filename.c_str(),std::ios::out|std::ios::trunc);
    if(!fileout.is_open())
    {
        std::cerr<<"error in opening file!"<<std::endl;
        std::exit(EXIT_FAILURE);
    }

    this->saveSceneConfigToFile(fileout);

    //for (unsigned int rigid_body_idx = 0; rigid_body_idx<=max_id; rigid_body_idx++)
    for (unsigned int rigid_body_idx = 0; rigid_body_idx<this->rigid_driver_->numRigidBody(); rigid_body_idx++)
    {
        RigidBody<Scalar, 3> * rigid_body = dynamic_cast<RigidBody<Scalar,3>*>(this->rigid_driver_->rigidBody(rigid_body_idx));
        this->saveMeshToFile(fileout, rigid_body);//rigid_body->mesh(), &rigid_body->transform());
    }
    fileout.close();
}

template <typename Scalar, int Dim>
void RigidDriverPluginPOV<Scalar, Dim>::setOutputFilenameBase(const std::string & filename_base)
{
    this->output_filename_base_ = filename_base;
}

template <typename Scalar, int Dim>
void RigidDriverPluginPOV<Scalar, Dim>::saveSceneConfigToFile(std::fstream & fileout)
{
    //output included inc 
    fileout<<"#include \"colors.inc\""<<std::endl;
    fileout<<"#include \"textures.inc\""<<std::endl;
    fileout<<"#include \"shapes.inc\""<<std::endl;
    fileout<<"#include \"metals.inc\""<<std::endl;
    fileout<<"#include \"woods.inc\""<<std::endl;
    fileout<<"#include \"stones.inc\""<<std::endl;

    //output global settings
    fileout<<"global_settings {max_trace_level 5}"<<std::endl;

    //output camera
    fileout<<"camera \n{"<<std::endl;
	fileout<<"   location <-2, 15, -18>"<<std::endl;
    fileout<<"   look_at <2, 0, -4> \n}"<<std::endl;
    //fileout<<"   location <4, 15, -18>"<<std::endl;
    //fileout<<"   look_at <-1, 0, 5> \n}"<<std::endl;
    //fileout<<"   location <3.5, 20, -25>"<<std::endl;
    //fileout<<"   look_at <3.5, 0, 0> \n}"<<std::endl;

    fileout<<"light_source \n{"<<std::endl;
    fileout<<"   < -20, 30, -150>  "<<std::endl;
	//fileout<<"   < 100, 1000, -150>  "<<std::endl;
    fileout<<"   color White "<<std::endl;
    fileout<<"}"<<std::endl;

    fileout<<"light_source \n{"<<std::endl;
    fileout<<"   < 100, 1000, -150>  "<<std::endl;
    fileout<<"   color Gray30 cylinder radius 800 falloff 300 point_at <100, 0, 0>"<<std::endl;
    fileout<<"}"<<std::endl;

    //output sky_sphere
    fileout<<"sky_sphere \n{"<<std::endl;
    fileout<<"   pigment \n   {"<<std::endl;
    fileout<<"       gradient y"<<std::endl;
    fileout<<"       color_map {[0, 1  color Gray80 color Gray80]}"<<std::endl;
    fileout<<"   } \n}"<<std::endl;
}

template <typename Scalar, int Dim>
void RigidDriverPluginPOV<Scalar, Dim>::saveMeshToFile(std::fstream & fileout, RigidBody<Scalar, 3>* rigid_body)//const SurfaceMesh<Scalar> *mesh, Transform<Scalar, Dim> * transform)
{
    SurfaceMesh<Scalar> *mesh = rigid_body->mesh();
    Transform<Scalar, Dim> *transform = rigid_body->transformPtr();
    fileout<<"mesh2 {\n";

    //vertex_vectors
    unsigned int vert_num = mesh->numVertices();
    if(vert_num > 0)
    {
        fileout<<"   vertex_vectors {\n";
        fileout<<"      "<<vert_num<<",\n";
        for (unsigned int idx = 0; idx < vert_num; idx++)
        {
            Vector<Scalar,3> pos = mesh->vertexPosition(idx);
            pos = transform->transform(pos);
            fileout<<"      <"<<pos[0]<<","<<pos[1]<<","<<-pos[2]<<">"; //negative axis-z due to different coordinate system with OpenGL
            if (idx != vert_num - 1)
                fileout<<",\n";
            else
                fileout<<"\n";
        }
        fileout<<"   }\n"; //end of vertex_vertors
    }

    // normal_vectors
    unsigned int normal_num = mesh->numNormals();
    if(normal_num > 0)
    {
        fileout<<"   normal_vectors {\n";
        fileout<<"      "<<normal_num<<",\n";
        for (unsigned int idx = 0; idx < normal_num; idx++)
        {
            Vector<Scalar,3> normal = mesh->vertexNormal(idx);
            normal = transform->rotate(normal);
            //normal.normalize();
            fileout<<"      <"<<normal[0]<<","<<normal[1]<<","<<-normal[2]<<">";//negative axis-z due to different coordinate system with OpenGL
            if (idx != normal_num-1)
                fileout<<",\n";
            else
                fileout<<"\n";
        }
        fileout<<"   }\n"; //end of normal_vertors
    }

    // uv_vectors
    unsigned int tex_coord_num = mesh->numTextureCoordinates();
    if(tex_coord_num > 0)
    {
        fileout<<"   uv_vectors {\n";
        fileout<<"      "<<tex_coord_num<<",\n";
        for (unsigned int idx = 0; idx < tex_coord_num; idx++)
        {
            Vector<Scalar,2> tex_coord = mesh->vertexTextureCoordinate(idx);
            fileout<<"      <"<<tex_coord[0]<<","<<tex_coord[1]<<">";
            if (idx != tex_coord_num - 1)
                fileout<<",\n";
            else
                fileout<<"\n";
        }
        fileout<<"   }\n"; //end of uv_vectors
    }

    // texture_list
    unsigned int tex_num = mesh->numTextures();
    if(tex_num > 0)
    {
        fileout<<"   texture_list {\n";
        fileout<<"      "<<tex_num<<",\n";
        for (unsigned int idx = 0; idx<mesh->numMaterials(); idx++)
        {
            if(mesh->material(idx).hasTexture())
            {
                std::string texture_file_name = mesh->material(idx).textureFileName();
                std::cerr<<texture_file_name<<std::endl;
                std::string file_extension = FileUtilities::fileExtension(texture_file_name);
                fileout<<"      "<<"texture{ pigment{ image_map{ "<<"png"/*file_extension*/<<" \""<<texture_file_name<<"\"}}}\n";
            }
        }
        fileout<<"   }\n"; // end of texture_list
    }

    // face_indices
    unsigned int face_num = mesh->numFaces();
    unsigned int tex_idx = 0;
    if(face_num > 0 && vert_num > 0)
    {
        fileout<<"   face_indices {\n";
        fileout<<"      "<<face_num<<",\n";
        for(unsigned int group_idx = 0; group_idx < mesh->numGroups(); ++group_idx)
        {
            const SurfaceMeshInternal::FaceGroup<Scalar> &group = mesh->group(group_idx);
            const BoundaryMeshInternal::Material<Scalar> &material = mesh->material(group.materialIndex());
            for(unsigned int face_idx = 0; face_idx < group.numFaces(); ++face_idx)
            {
                const SurfaceMeshInternal::Face<Scalar> &face = group.face(face_idx);
                const Vertex<Scalar> & ver0 = face.vertex(0);
                const Vertex<Scalar> & ver1 = face.vertex(1);
                const Vertex<Scalar> & ver2 = face.vertex(2);
                fileout<<"      <"<<ver0.positionIndex()<<","<<ver1.positionIndex()<<","<<ver2.positionIndex()<<">";
                if(material.hasTexture())
                    fileout<<","<<tex_idx;
                if (face_idx != group.numFaces() - 1 || group_idx != mesh->numGroups() - 1)
                    fileout<<",\n";
                else
                    fileout<<"\n";
            }
            if(material.hasTexture())
                ++ tex_idx;
        }
        fileout<<"   }\n"; //end of face_indices
    }

    // normal_indices
    if(face_num > 0 && normal_num > 0)
    {
        fileout<<"   normal_indices {\n";
        fileout<<"      "<<face_num<<",\n";
        for (unsigned int idx = 0; idx < face_num; idx++)
        {
            const SurfaceMeshInternal::Face<Scalar> & face = mesh->face(idx);
            const Vertex<Scalar> & ver0 = face.vertex(0);
            const Vertex<Scalar> & ver1 = face.vertex(1);
            const Vertex<Scalar> & ver2 = face.vertex(2);
            fileout<<"      <"<<ver0.normalIndex()<<","<<ver1.normalIndex()<<","<<ver2.normalIndex()<<">";
            if (idx != face_num - 1)
                fileout<<",\n";
            else
                fileout<<"\n";
        }
        fileout<<"   }\n"; //end of normal_indices
    }

    // uv_indices
    if(face_num > 0 && tex_coord_num > 0)
    {
        fileout<<"   uv_indices {\n";
        fileout<<"      "<<face_num<<",\n";
        for (unsigned int idx = 0; idx < face_num; idx++)
        {
            const SurfaceMeshInternal::Face<Scalar> & face = mesh->face(idx);
            const Vertex<Scalar> & ver0 = face.vertex(0);
            const Vertex<Scalar> & ver1 = face.vertex(1);
            const Vertex<Scalar> & ver2 = face.vertex(2);
            fileout<<"      <"<<ver0.textureCoordinateIndex()<<","<<ver1.textureCoordinateIndex()<<","<<ver2.textureCoordinateIndex()<<">";
            if (idx != face_num - 1)
                fileout<<",\n";
            else
                fileout<<"\n";
        }
        fileout<<"   }\n"; //end of uv_indices
    }
    if(rigid_body->isFixed())
    {
        if(rigid_body->mesh()->numFaces() < 20)
        {
            fileout<<"  texture { pigment {color rgb <0.6, 0.6, 0.6>} finish {metallic reflection {0.3, 0.5}}}"<<std::endl;
            fileout<<"}\n"; //end
            return;
        }
        else
        {
            fileout<<"  texture {uv_mapping pigment {image_map{"<<'"'<<"wood.jpg"<<'"'<<"}} finish {phong 1/*metallic reflection {0.05, 0.15}*/} }"<<std::endl;
            fileout<<"}\n"; //end
            return;
        }
    }
    if(rigid_body->mesh()->numVertices() > 400)
    {
    fileout<<"  texture {uv_mapping pigment {image_map{"<<'"'<<"stone.jpg"<<'"'<<"}} finish {metallic reflection {0.03, 0.075}} }"<<std::endl;
    fileout<<"}\n"<<std::endl;
    return;
    }

    int color_index = rand() % 3;

    //if(color_index == 0)
    //    fileout<<"  texture { pigment {color rgb <0.75, 0.75, 0.75>} finish {metallic reflection {0.15, 0.3}}}"<<std::endl;

    if(color_index == 0)
        fileout<<"  texture { pigment {color rgb <0.7, 0.4, 0.4>}finish {metallic reflection {0.03, 0.075}} }"<<std::endl;

    if(color_index == 1)
        fileout<<"  texture { pigment {color rgb <0.4, 0.7, 0.4>}finish {metallic reflection {0.03, 0.075}}  }"<<std::endl;

    if(color_index == 2)
        fileout<<"  texture { pigment {color rgb <0.4, 0.4, 0.7>}finish {metallic reflection {0.03, 0.075}}  }"<<std::endl;


    fileout<<"}\n"; //end
}

//explicit instantiation
template class RigidDriverPluginPOV<float, 3>;
template class RigidDriverPluginPOV<double, 3>;


}// end of namespace Physika
