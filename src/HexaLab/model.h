#pragma once

#include "common.h"
#include "mesh.h"

namespace HexaLab {
    using namespace std;
    
    // GPU geometry buffers container 

    struct Model {
        vector<glm::vec3> surface_vert_pos;
        vector<glm::vec3> surface_vert_norm;
        vector<glm::vec4> surface_vert_color;
        vector<Index>    surface_ibuffer;
        vector<glm::vec3> wireframe_vert_pos;
        vector<glm::vec4> wireframe_vert_color;
        vector<glm::vec3> mesh_vert_pos;
        vector<glm::vec3> mesh_vert_norm;
        vector<glm::vec4> mesh_vert_color;
        vector<Index>    mesh_ibuffer;


        void clear() {
            surface_vert_pos.clear();
            surface_vert_norm.clear();
            surface_vert_color.clear();
            surface_ibuffer.clear();
            wireframe_vert_pos.clear();
            wireframe_vert_color.clear();
            mesh_vert_pos.clear();
            mesh_vert_norm.clear();
            mesh_vert_color.clear();
            mesh_ibuffer.clear();
        }

        void add_surf_vert(glm::vec3 pos, glm::vec4 col){
            surface_vert_pos.push_back(pos);
            surface_vert_color.push_back(col);
        }

        void add_wire_vert(glm::vec3 pos, glm::vec4 col){
            wireframe_vert_pos.push_back(pos);
            wireframe_vert_color.push_back(col);
        }

    };
}
