#include "app.h"

#include <limits>

#define D2R (3.141592653589793f / 180.f)

namespace HexaLab {

    // PUBLIC

    bool App::import_mesh ( const vector<glm::vec3>& verts, const vector<HexaLab::Index>& indices ) {
        delete mesh;
        mesh = new Mesh();

        // Build
        HL_LOG ( "Building...\n" );
        Builder::build ( *mesh, verts, indices );
        // Validate
        HL_LOG ( "Validating...\n" );
        Builder::validate ( *mesh );
        // Update stats
        float max = std::numeric_limits<float>::lowest();
        float min = std::numeric_limits<float>::max();
        float avg = 0;

        for ( size_t i = 0; i < mesh->edges.size(); ++i ) {
            MeshNavigator nav = mesh->navigate ( mesh->edges[i] );
            glm::vec3 edge = nav.vert().position - nav.flip_vert().vert().position;
            float len = glm::length(edge);

            if ( len < min ) {
                min = len;
            }

            if ( len > max ) {
                max = len;
            }

            avg += len;
        }

        avg /= mesh->edges.size();
        mesh_stats.min_edge_len = min;
        mesh_stats.max_edge_len = max;
        mesh_stats.avg_edge_len = avg;
        float avg_v = 0;
        glm::vec3 v[8];

        for ( size_t i = 0; i < mesh->cells.size(); ++i ) {
            int j = 0;
            MeshNavigator nav = mesh->navigate ( mesh->cells[i] );
            Vert& a = nav.vert();

            do {
                v[j++] = nav.vert().position;
                nav = nav.rotate_on_face();
            } while ( nav.vert() != a );

            nav = nav.rotate_on_cell().rotate_on_cell().flip_vert();
            Vert& b = nav.vert();

            do {
                v[j++] = nav.vert().position;
                nav = nav.rotate_on_face();
            } while ( nav.vert() != b );

            avg_v += QualityMeasureFun::volume ( v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], nullptr );
        }

        avg_v /= mesh->cells.size();
        mesh_stats.avg_volume = avg_v;
        mesh_stats.vert_count = mesh->verts.size();
        mesh_stats.hexa_count = mesh->cells.size();
        mesh_stats.aabb = mesh->aabb;
        mesh_stats.quality_min = 0;
        mesh_stats.quality_max = 0;
        mesh_stats.quality_avg = 0;
        mesh_stats.quality_var = 0;
        // build quality buffers
        this->compute_hexa_quality();

        // build buffers
        this->flag_models_as_dirty();
        //this->update_models();
        //this->build_singularity_models();
        //this->build_full_model(); // TODO: Uncommented, as not needed in HexVolumeRenderer at the moment.
        return true;
    }

    /*void App::enable_quality_color_mapping ( ColorMap::Palette e ) {
        this->color_map = ColorMap ( e );
        this->quality_color_mapping_enabled = true;
        this->flag_models_as_dirty();   // TODO update color only
    }*/
    void App::disable_quality_color_mapping() {
        this->quality_color_mapping_enabled = false;
        this->flag_models_as_dirty();   // TODO update color only
    }

    void App::set_quality_measure ( QualityMeasureEnum e ) {
        this->quality_measure = e;
        if(this->mesh == nullptr) return; 
        this->compute_hexa_quality();

        if ( this->is_quality_color_mapping_enabled() ) {
            this->flag_models_as_dirty();   // TODO update color only
        }
    }

    void App::set_geometry_mode ( GeometryMode mode ) {
        this->geometry_mode = mode;
        this->flag_models_as_dirty();
    }

    void App::set_default_outside_attribute ( float val ) {
        this->default_outside_attribute = val;

        if ( !this->is_quality_color_mapping_enabled() ) {
            this->flag_models_as_dirty();
        }
    }

    void App::set_default_inside_attribute ( float val ) {
        this->default_inside_attribute = val;

        if ( !this->is_quality_color_mapping_enabled() ) {
            this->flag_models_as_dirty();
        }
    }

    bool App::update_models() {
        this->build_surface_models();

        return true;
    }


    void App::show_boundary_singularity ( bool do_show ) {
        this->do_show_boundary_singularity = do_show;
        this->flag_models_as_dirty();
    }
    void App::show_boundary_creases ( bool do_show ) {
        this->do_show_boundary_creases = do_show;
        this->flag_models_as_dirty();
    }

    void App::set_crack_size ( float size ) {
        this->crack_size = size;
        this->flag_models_as_dirty();
    }
    void App::set_rounding_radius ( float rad ) {
        this->rounding_radius = rad;
        this->flag_models_as_dirty();
    }

    void App::set_regularize_str ( size_t  level ) {
        this->regularize_str = level;
        this->flag_models_as_dirty();
    }

    void App::show_visible_wireframe_singularity ( bool show ) {
        this->do_show_visible_wireframe_singularity = show;
        this->flag_models_as_dirty();
    }

    // PRIVATE

    void App::compute_hexa_quality() {
        quality_measure_fun* qualityFunctor = get_quality_measure_fun ( this->quality_measure );
        void* arg = nullptr;

        switch ( this->quality_measure ) {
            case QualityMeasureEnum::RSS:
            case QualityMeasureEnum::SHAS:
            case QualityMeasureEnum::SHES:
                arg = &this->mesh_stats.avg_volume;
                break;

            default:
                break;
        }

        float minQ =  std::numeric_limits<float>::max();
        float maxQ = -std::numeric_limits<float>::max();
        float sum = 0;
        float sum2 = 0;
        mesh->hexa_quality.resize ( mesh->cells.size() );
        mesh->normalized_hexa_quality.resize ( mesh->cells.size() );
        glm::vec3 v[8];

        for ( size_t i = 0; i < mesh->cells.size(); ++i ) {
            int j = 0;
            MeshNavigator nav = mesh->navigate ( mesh->cells[i] );
            Vert& a = nav.vert();

            do {
                v[j++] = nav.vert().position;
                nav = nav.rotate_on_face();
            } while ( nav.vert() != a );

            nav = nav.rotate_on_cell().rotate_on_cell().flip_vert();
            Vert& b = nav.vert();

            do {
                v[j++] = nav.vert().position;
                nav = nav.rotate_on_face();
            } while ( nav.vert() != b );

            float q = qualityFunctor ( v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], arg );
            mesh->hexa_quality[i] = q;
            minQ = min(q,minQ);
            maxQ = max(q,maxQ);
            sum += q;
            sum2 += q * q;
        }

        this->mesh_stats.quality_min = minQ;
        this->mesh_stats.quality_max = maxQ;
        this->mesh_stats.quality_avg = sum / mesh->cells.size();
        this->mesh_stats.quality_var = sum2 / mesh->cells.size() - this->mesh_stats.quality_avg * this->mesh_stats.quality_avg;
        minQ = std::numeric_limits<float>::max();
        maxQ = -std::numeric_limits<float>::max();

        for ( size_t i = 0; i < mesh->cells.size(); ++i ) {
            float q = normalize_quality_measure ( this->quality_measure, mesh->hexa_quality[i], this->mesh_stats.quality_min, this->mesh_stats.quality_max );
            minQ = min(q,minQ);
            maxQ = max(q,maxQ);
            mesh->normalized_hexa_quality[i] = q;
        }

        this->mesh_stats.normalized_quality_min = minQ;
        this->mesh_stats.normalized_quality_max = maxQ;
        HL_LOG ( "[QUALITY: %s] %f %f - norm %f %f.\n", get_quality_name(this->quality_measure), mesh_stats.quality_min,mesh_stats.quality_max,mesh_stats.normalized_quality_min,mesh_stats.normalized_quality_max );
    }

    float App::get_normalized_hexa_quality_cell(uint32_t cell_id) {
        return mesh->normalized_hexa_quality.at(cell_id);
    }

    /*void App::build_singularity_model(
            std::vector<glm::vec3>& lineVertices,
            std::vector<glm::vec4>& lineColors,
            std::vector<glm::vec3>& pointVertices,
            std::vector<glm::vec4>& pointColors) {
        // Find all irregular edges
        for (size_t i = 0; i < mesh->edges.size(); ++i) {
            MeshNavigator nav = mesh->navigate(mesh->edges[i]);
            int edgeValence = nav.edge().valence;
            bool isBoundary = nav.edge().is_surface;
            if ((edgeValence != 2 && isBoundary) || (edgeValence != 4 && !isBoundary)) {
                lineVertices.push_back(nav.vert().position);
                lineVertices.push_back(nav.flip_vert().vert().position);
                //glm::vec4 color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
                glm::vec4 color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
                if (edgeValence == 1) {
                    color = glm::vec4(0.0f, 1.0f, 0.0f, 1.0f);
                }
                if (edgeValence == 2) {
                    color = glm::vec4(0.0f, 0.0f, 1.0f, 1.0f);
                }
                if (edgeValence == 3) {
                    color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
                }
                if (edgeValence == 4) {
                    color = glm::vec4(1.0f, 0.0f, 1.0f, 1.0f);
                }
                if (edgeValence == 6) {
                    color = glm::vec4(0.5f, 0.0f, 0.5f, 1.0f);
                }
                lineColors.push_back(color);
                lineColors.push_back(color);
            }
        }

        // Find all irregular vertices
        for (size_t i = 0; i < mesh->verts.size(); ++i) {
            MeshNavigator nav = mesh->navigate(mesh->verts[i]);
            int vertValence = nav.vert().valence;//nav.incident_cell_on_vertex_num();
            bool isBoundary = nav.vert().is_surface;
            if ((vertValence != 4 && isBoundary) || (vertValence != 8 && !isBoundary)) {
                pointVertices.push_back(nav.vert().position);
                pointColors.push_back(glm::vec4(0.4f, 0.0f, 0.0f, 1.0f));
            }
        }
    }

    void App::build_singularity_models() {
        line_singularity_model.clear();
        spined_singularity_model.clear();
        full_singularity_model.clear();

        // boundary_singularity_model.clear();
        // boundary_creases_model.clear();
        for ( size_t i = 0; i < mesh->edges.size(); ++i ) {
            MeshNavigator nav = mesh->navigate ( mesh->edges[i] );
            // -- Boundary check --
            // {
            //     bool boundary = false;
            //     Face& begin = nav.face();
            //     do {
            //         if (nav.is_face_boundary()) {
            //             boundary = true;
            //             break;
            //         }
            //         nav = nav.rotate_on_edge();
            //     } while(nav.face() != begin);
            //     if (boundary) {
            //         // Boundary singularity
            //         if (nav.incident_face_on_edge_num() != 2) {
            //             for (int n = 0; n < 2; ++n) {     // 2 verts that make the edge
            //                 boundary_singularity_model.wireframe_vert_pos.push_back(nav.vert().position);
            //                 boundary_singularity_model.wireframe_vert_color.push_back(glm::vec3(0, 0, 1));
            //                 nav = nav.flip_vert();
            //             }
            //         }
            //         // Boundary Crease
            //         MeshNavigator nav2 = nav.flip_face();
            //         if (!nav2.is_face_boundary()) {
            //             nav2 = nav2.flip_cell().flip_face();
            //             float dot = nav.face().normal.dot(nav2.face().normal);
            //             if (std::acos(dot) > 30 * D2R) {
            //                 for (int n = 0; n < 2; ++n) {     // 2 verts that make the edge
            //                     boundary_creases_model.wireframe_vert_pos.push_back(nav.vert().position);
            //                     boundary_creases_model.wireframe_vert_color.push_back(glm::vec3(1, 0, 0));
            //                     nav = nav.flip_vert();
            //                 }
            //             }
            //         }
            //     }
            // }
            // -- Singularity check
            int face_count = nav.incident_face_on_edge_num();

            if ( face_count == 4 ) {
                continue;
            }

            if ( nav.edge().is_surface ) {
                continue;
            }

            glm::vec3 colWI; // wireframe Interior
            glm::vec3 colWE; // wireframe Exterior
            glm::vec3 colSI; // surface Interior
            glm::vec3 colSE; // surface Exterior
            glm::vec3 colSW; // surface Wire

            switch ( face_count ) {
                case  3:
                    colWI = glm::vec3( 0.8f, 0.30f, 0.30f );
                    colWE = glm::vec3( 1.0f, 0.80f, 0.80f );
                    colSE = glm::vec3( 1.0f, 0.30f, 0.30f );
                    colSI = colSE * 0.6f;
                    colSW = colSI *0.75f;
                    break;

                case  5:
                    colWI = glm::vec3( 0.1f, 0.70f , 0.1f );
                    colWE = glm::vec3( 0.5f, 0.90f , 0.5f );
                    colSE = glm::vec3( 0.3f, 1.00f , 0.3f );;
                    colSI = colSE * 0.5f;
                    colSW = colSI * 0.75f;
              break;

                default:
                    colWI  = glm::vec3( 0.2f, 0.2f, 1 );
                    colWE = glm::vec3( 0.6f, 0.6f, 1 );
                    colSI = colWI * 0.5f;
                    colSE = colWI;
                    colSW = colSI * 0.75f;
              break;
            }

            glm::vec3 black = glm::vec3(0,0,0);
            glm::vec3 white = glm::vec3(1,1,1);

            glm::vec3 v0, v1;

            v0 = mesh->verts[nav.dart().vert].position;
            v1 = mesh->verts[nav.flip_vert().dart().vert].position;


            // add adjacent faces/edges
            Face& begin = nav.face();

            do {
                glm::vec3 v2, v3;
                v2 = mesh->verts[nav.rotate_on_face().flip_vert().dart().vert].position;
                v3 = mesh->verts[nav.flip_vert().rotate_on_face().flip_vert().dart().vert].position;
                v2 = v2*0.45f + v1 *0.55f;
                v3 = v3*0.45f + v0 *0.55f;

                spined_singularity_model.add_wire_vert( v0, glm::vec4(colWI, 1.0f) );
                spined_singularity_model.add_wire_vert( v3, glm::vec4(colWE, 1.0f) );

                spined_singularity_model.add_wire_vert( v1, glm::vec4(colWI, 1.0f) );
                spined_singularity_model.add_wire_vert( v2, glm::vec4(colWE, 1.0f) );

                full_singularity_model.add_wire_vert( v0, glm::vec4(colSW, 1.0f) );
                full_singularity_model.add_wire_vert( v3, glm::vec4(colSE, 1.0f) );

                full_singularity_model.add_wire_vert( v1, glm::vec4(colSW, 1.0f) );
                full_singularity_model.add_wire_vert( v2, glm::vec4(colSE, 1.0f) );

                // add two tris
                full_singularity_model.add_surf_vert( v0, glm::vec4(colSI, 1.0f) );
                full_singularity_model.add_surf_vert( v1, glm::vec4(colSI, 1.0f) );
                full_singularity_model.add_surf_vert( v2, glm::vec4(colSE, 1.0f) );
                full_singularity_model.add_surf_vert( v2, glm::vec4(colSE, 1.0f) );
                full_singularity_model.add_surf_vert( v3, glm::vec4(colSE, 1.0f) );
                full_singularity_model.add_surf_vert( v0, glm::vec4(colSI, 1.0f) );

                nav = nav.rotate_on_edge();
            } while ( nav.face() != begin );

            line_singularity_model.add_wire_vert( v0 , glm::vec4(colWI, 1.0f));
            spined_singularity_model.add_wire_vert( v0 , glm::vec4(colWI, 1.0f));
            full_singularity_model.add_wire_vert( v0 , glm::vec4(black, 1.0f));

            line_singularity_model.add_wire_vert( v1 , glm::vec4(colWI, 1.0f));
            spined_singularity_model.add_wire_vert( v1 , glm::vec4(colWI, 1.0f));
            full_singularity_model.add_wire_vert( v1 , glm::vec4(black, 1.0f));


        }
    }*/

    void App::add_visible_vert ( Dart& dart, float normal_sign, float cellAttribute ) {
        MeshNavigator nav = mesh->navigate ( dart );
        visible_model.surface_vert_pos.push_back ( nav.vert().position );
        visible_model.surface_vert_norm.push_back ( nav.face().normal * normal_sign );
        visible_model.surface_vert_attribute.push_back ( cellAttribute );
        HL_ASSERT ( visible_model.surface_vert_pos.size() == visible_model.surface_vert_norm.size() &&
                    visible_model.surface_vert_pos.size() == visible_model.surface_vert_attribute.size() );
        Index idx = visible_model.surface_vert_pos.size() - 1;
        visible_model.surface_ibuffer.push_back ( idx );
    }

    size_t App::add_vertex ( glm::vec3 pos, glm::vec3 norm, float cellAttribute ) {
        size_t i = visible_model.surface_vert_pos.size();
        visible_model.surface_vert_pos.push_back ( pos );
        visible_model.surface_vert_norm.push_back ( norm );
        visible_model.surface_vert_attribute.push_back ( cellAttribute );
        return i;
    }

    size_t App::add_full_vertex ( glm::vec3 pos, glm::vec3 norm, float cellAttribute ) {
        size_t i = full_model.surface_vert_pos.size();
        full_model.surface_vert_pos.push_back ( pos );
        full_model.surface_vert_norm.push_back ( norm );
        full_model.surface_vert_attribute.push_back ( cellAttribute );
        return i;
    }

    size_t App::add_mesh_vertex ( glm::vec3 pos, glm::vec3 norm, float cellAttribute ) {
        size_t i = visible_model.mesh_vert_pos.size();
        visible_model.mesh_vert_pos.push_back ( pos );
        visible_model.mesh_vert_norm.push_back ( norm );
        visible_model.mesh_vert_attribute.push_back ( cellAttribute );
        return i;
    }

    size_t App::add_mesh_vertex_volume ( glm::vec3 pos, float cellAttribute ) {
        size_t i = visible_model.mesh_vert_pos.size();
        visible_model.mesh_vert_pos.push_back ( pos );
        visible_model.mesh_vert_attribute.push_back ( cellAttribute );
        return i;
    }

    void App::add_triangle ( size_t i1, size_t i2, size_t i3 ) {
        visible_model.surface_ibuffer.push_back ( i1 );
        visible_model.surface_ibuffer.push_back ( i2 );
        visible_model.surface_ibuffer.push_back ( i3 );
    }

    void App::add_full_triangle ( size_t i1, size_t i2, size_t i3 ) {
        full_model.surface_ibuffer.push_back ( i1 );
        full_model.surface_ibuffer.push_back ( i2 );
        full_model.surface_ibuffer.push_back ( i3 );
    }

    void App::add_mesh_triangle ( size_t i1, size_t i2, size_t i3 ) {
        visible_model.mesh_ibuffer.push_back ( i1 );
        visible_model.mesh_ibuffer.push_back ( i2 );
        visible_model.mesh_ibuffer.push_back ( i3 );
    }

    void App::add_mesh_triangle_volume ( size_t i1, size_t i2, size_t i3 ) {
        visible_model.mesh_ibuffer.push_back ( i1 );
        visible_model.mesh_ibuffer.push_back ( i2 );
        visible_model.mesh_ibuffer.push_back ( i3 );
    }

    void App::add_quad ( size_t i1, size_t i2, size_t i3, size_t i4 ) {
        add_triangle ( i1, i2, i3 );
        add_triangle ( i3, i4, i1 );
    }

    // dart: first dart in the face to render
    void App::add_visible_face ( Dart& dart, float normal_sign ) {
        MeshNavigator nav = mesh->navigate ( dart );

        // Faces are normally shared between two cells, but their data structure references only one of them, the 'main' (the first encountered when parsing the mesh file).
        // If the normal sign is -1, it means that the cell we want to render is not the main.
        // Therefore a flip cell is performed, along with a flip edge to maintain the winding.
        if ( normal_sign == -1 ) {
            nav = nav.flip_cell().flip_edge();
        }

        // If cell quality display is enabled, fetch the appropriate quality color.
        // Otherwise use the default coloration (white for outer faces, yellow for everything else)
        float cellAttribute;

        if ( is_quality_color_mapping_enabled() ) {
            //color = color_map.get ( mesh->normalized_hexa_quality[nav.cell_index()] );
            cellAttribute = 1.0f - mesh->normalized_hexa_quality[nav.cell_index()];
        } else {
            cellAttribute = nav.is_face_boundary() ? this->default_outside_attribute : this->default_inside_attribute;
        }

        for ( int i = 0; i < 2; ++i ) {
            for ( int j = 0; j < 2; ++j ) {
                add_visible_wireframe ( nav.dart() );
                nav = nav.rotate_on_face();
            }
        }

        nav = mesh->navigate ( dart );
        // nav = mesh->navigate(nav.cell());   // TODO remove this

        if ( normal_sign == -1 ) {
            nav = nav.flip_vert();    // TODO flip cell/edge instead? same thing?
        }

        Vert& vert = nav.vert();
        Index idx = visible_model.surface_vert_pos.size();

        do {
            add_vertex ( nav.vert().position, nav.face().normal * normal_sign, cellAttribute );
            nav = nav.rotate_on_face();
        } while ( nav.vert() != vert );

        add_triangle ( idx + 2, idx + 1, idx + 0 );
        add_triangle ( idx + 0, idx + 3, idx + 2 );
    }

    void App::add_visible_wireframe ( Dart& dart ) {
        MeshNavigator nav = mesh->navigate ( dart );
        //if (nav.edge().mark != mesh->mark) {
        //            nav.edge().mark = mesh->mark;
        MeshNavigator edge_nav = nav;
        bool boundary_singularity = false;
        bool boundary_crease = false;

        if ( this->do_show_boundary_singularity ) {
            if ( nav.incident_face_on_edge_num() != 2 ) {
                boundary_singularity = true;
            }
        }

        // if (this->do_show_boundary_creases) {
        //     Face& begin = nav.face();
        //     bool prev_face_is_boundary = false;
        //     glm::vec3 prev_face_normal;
        //     do {
        //         if (!prev_face_is_boundary) {
        //             if (nav.is_face_boundary()) {
        //                 prev_face_is_boundary = true;
        //                 prev_face_normal = nav.face().normal;
        //             }
        //         } else {
        //             if (nav.is_face_boundary()) {
        //                 // float dot = nav.face().normal.dot(prev_face_normal);
        //                 // if (std::acos(std::abs(dot)) > 1 * D2R) {
        //                     boundary_crease = true;
        //                     break;      // Ends in a different dart from the starting one!
        //                 // }
        //             } else {
        //                 prev_face_is_boundary = false;
        //             }
        //         }
        //         nav = nav.rotate_on_edge();
        //     } while (nav.face() != begin);
        // }

        float alpha;
        if (this->do_show_visible_wireframe_singularity) {
            size_t unfiltered_cells = 0;
            Cell& h = edge_nav.cell();
            do {
                if ( !mesh->is_marked( edge_nav.cell() ) ) {
                    ++unfiltered_cells;
                }
                edge_nav = edge_nav.rotate_on_edge();
            } while ( edge_nav.cell() != h );

            switch ( unfiltered_cells ) {
            case 0:             // should never hit
                alpha = 0;
            case 1:
                alpha = 0.1;
                break;
            case 2:
                alpha = 0.5;
            case 3:
                alpha = 0.7;
            default:
                alpha = 0.9;
            }
        } else {
            alpha = 1;
        }

        for ( int v = 0; v < 2; ++v ) {
            visible_model.wireframe_vert_pos.push_back ( edge_nav.vert().position );

            // if (this->do_show_boundary_singularity && boundary_singularity
            // && this->do_show_boundary_creases && boundary_crease) {
            // visible_model.wireframe_vert_color.push_back(glm::vec3(0, 1, 1));
            // } else
            if ( this->do_show_boundary_singularity && boundary_singularity ) {
                visible_model.wireframe_vert_color.push_back ( glm::vec4 ( 0, 0, 1, 1 ) );
                // } else if (this->do_show_boundary_creases && boundary_crease) {
                // visible_model.wireframe_vert_color.push_back(glm::vec3(0, 1, 0));
            } else {
                visible_model.wireframe_vert_color.push_back ( glm::vec4 ( 0, 0, 0, alpha ) );
            }

            edge_nav = edge_nav.flip_vert();
        }

        //}
    }

    void App::add_filtered_face ( Dart& dart ) {
        MeshNavigator nav = mesh->navigate ( dart );

        for ( int i = 0; i < 2; ++i ) {
            for ( int j = 0; j < 2; ++j ) {
                filtered_model.surface_vert_pos.push_back ( mesh->verts[nav.dart().vert].position );
                add_filtered_wireframe ( nav.dart() );
                nav = nav.rotate_on_face();
            }

            filtered_model.surface_vert_pos.push_back ( mesh->verts[nav.dart().vert].position );
            glm::vec3 normal = nav.face().normal;
            filtered_model.surface_vert_norm.push_back ( normal );
            filtered_model.surface_vert_norm.push_back ( normal );
            filtered_model.surface_vert_norm.push_back ( normal );
        }
    }

    void App::add_filtered_wireframe ( Dart& dart ) {
        MeshNavigator nav = mesh->navigate ( dart );
        //if (nav.edge().mark != mesh->mark) {
        //            nav.edge().mark = mesh->mark;
        MeshNavigator edge_nav = nav;

        for ( int v = 0; v < 2; ++v ) {
            filtered_model.wireframe_vert_pos.push_back ( mesh->verts[edge_nav.dart().vert].position );
            edge_nav = edge_nav.flip_vert();
        }

        //}
    }

    void App::add_full_face ( Dart& dart ) {
        MeshNavigator nav = mesh->navigate ( dart );
        Vert& vert = nav.vert();
        Index idx = full_model.surface_vert_pos.size();

        do {
            add_full_vertex ( nav.vert().position, nav.face().normal, 0.0f );
            nav = nav.rotate_on_face();
        } while ( nav.vert() != vert );

        add_full_triangle ( idx + 2, idx + 1, idx + 0 );
        add_full_triangle ( idx + 0, idx + 3, idx + 2 );
    }

    void App::add_mesh_face ( Dart& dart, float normal_sign ) {
        MeshNavigator nav = mesh->navigate ( dart );

        // Faces are normally shared between two cells, but their data structure references only one of them, the 'main' (the first encountered when parsing the mesh file).
        // If the normal sign is -1, it means that the cell we want to render is not the main.
        // Therefore a flip cell is performed, along with a flip edge to maintain the winding.
        bool flip_winding = false;
        if ( normal_sign == -1 ) {
            if (nav.dart().cell_neighbor == -1) {
                flip_winding = true;
            } else {
                nav = nav.flip_cell().flip_edge();
            }
        }

        // If cell quality display is enabled, fetch the appropriate quality color.
        // Otherwise use the defautl coloration (white for outer faces, yellow for everything else)
        float cellAttribute;

        if ( is_quality_color_mapping_enabled() ) {
            //color = color_map.get ( mesh->normalized_hexa_quality[nav.cell_index()] );
            cellAttribute = 1.0f - mesh->normalized_hexa_quality[nav.cell_index()];
        } else {
            cellAttribute = nav.is_face_boundary() ? this->default_outside_attribute : this->default_inside_attribute;
        }

        nav = mesh->navigate ( dart );

        if ( normal_sign == -1 ) {
            nav = nav.flip_vert();    // TODO flip cell/edge instead? same thing?
        }

        Vert& vert = nav.vert();
        Index idx = visible_model.mesh_vert_pos.size();

        do {
            add_mesh_vertex ( nav.vert().position, nav.face().normal * normal_sign, cellAttribute );
            nav = nav.rotate_on_face();
        } while ( nav.vert() != vert );

        add_mesh_triangle ( idx + 2, idx + 1, idx + 0 );
        add_mesh_triangle ( idx + 0, idx + 3, idx + 2 );
    }

    void App::add_mesh_face_volume ( Dart& dart, bool addPositiveNormal, bool addNegativeNormal ) {
        MeshNavigator nav = mesh->navigate ( dart );

        // Faces are normally shared between two cells, but their data structure references only one of them, the 'main' (the first encountered when parsing the mesh file).
        // If the normal sign is -1, it means that the cell we want to render is not the main.
        // Therefore a flip cell is performed, along with a flip edge to maintain the winding.
        //if ( normal_sign == -1 ) {
        //    nav = nav.flip_cell().flip_edge();
        //}

        // If cell quality display is enabled, fetch the appropriate quality color.
        // Otherwise use the defautl coloration (white for outer faces, yellow for everything else)
        float pos_attribute, neg_attribute;

        if ( is_quality_color_mapping_enabled() ) {
            //color = color_map.get ( mesh->normalized_hexa_quality[nav.cell_index()] );
            if (addPositiveNormal) {
                pos_attribute = 1.0f - mesh->normalized_hexa_quality[nav.cell_index()];
            }
            if (addNegativeNormal) {
                neg_attribute = 1.0f - mesh->normalized_hexa_quality[nav.flip_cell().flip_edge().cell_index()];
            }
        } else {
            pos_attribute = nav.is_face_boundary() ? this->default_outside_attribute : this->default_inside_attribute;
            neg_attribute = pos_attribute;
        }

        if (addPositiveNormal) {
            nav = mesh->navigate ( dart );

            //if ( normal_sign == -1 ) {
            //    nav = nav.flip_vert();    // TODO flip cell/edge instead? same thing?
            //}

            Vert& vert = nav.vert();
            Index idx = visible_model.mesh_vert_pos.size();

            do {
                add_mesh_vertex_volume ( nav.vert().position, pos_attribute );
                nav = nav.rotate_on_face();
            } while ( nav.vert() != vert );

            add_mesh_triangle_volume ( idx + 2, idx + 1, idx + 0 );
            add_mesh_triangle_volume ( idx + 0, idx + 3, idx + 2 );
        }

        if (addNegativeNormal) {
            nav = mesh->navigate ( dart );

            //if ( normal_sign == -1 ) {
            //    nav = nav.flip_vert();    // TODO flip cell/edge instead? same thing?
            //}

            Vert& vert = nav.vert();
            Index idx = visible_model.mesh_vert_pos.size();

            do {
                add_mesh_vertex_volume ( nav.vert().position, neg_attribute );
                nav = nav.rotate_on_face();
            } while ( nav.vert() != vert );

            add_mesh_triangle_volume ( idx + 0, idx + 1, idx + 2 );
            add_mesh_triangle_volume ( idx + 2, idx + 3, idx + 0 );
        }
    }


    void App::prepare_geometry() {
        visible_model.mesh_vert_pos.clear();
        visible_model.mesh_vert_norm.clear();
        visible_model.mesh_vert_attribute.clear();
        visible_model.mesh_ibuffer.clear();

        for ( size_t i = 0; i < mesh->faces.size(); ++i ) {
            MeshNavigator nav = mesh->navigate ( mesh->faces[i] );

            // cell a visible, cell b not existing or not visible
            if ( !mesh->is_marked ( nav.cell() ) && ( nav.dart().cell_neighbor == -1 || mesh->is_marked ( nav.flip_cell().cell() ) ) ) {
                this->add_visible_face ( nav.dart(), 1 );
                // cell a invisible, cell b existing and visible
            } else if ( mesh->is_marked ( nav.cell() ) && nav.dart().cell_neighbor != -1 && !mesh->is_marked ( nav.flip_cell().cell() ) ) {
                this->add_visible_face ( nav.dart(), -1 );
                // add_filtered_face(nav.dart());
                // face was culled by the plane, is surface
            } else if ( mesh->is_marked ( nav.cell() ) && nav.dart().cell_neighbor == -1 ) {
                this->add_filtered_face ( nav.flip_edge().dart() );
            }

            // Add all front faces to the full mesh.
            if ( !mesh->is_marked ( nav.cell() ) ) {
                this->add_mesh_face( nav.dart(), 1 );
            }
            if ( nav.dart().cell_neighbor != -1 && !mesh->is_marked ( nav.flip_cell().cell() ) ) {
                this->add_mesh_face( nav.dart(), -1 );
            }
        }
    }

    void App::get_volume_geometry_faces() {
        visible_model.mesh_vert_pos.clear();
        visible_model.mesh_vert_norm.clear();
        visible_model.mesh_vert_attribute.clear();
        visible_model.mesh_ibuffer.clear();

        for ( size_t i = 0; i < mesh->faces.size(); ++i ) {
            MeshNavigator nav = mesh->navigate ( mesh->faces[i] );

            // Add all unfiltered cells to the full mesh (backfaces only for boundary surface).
            if ( !mesh->is_marked ( nav.cell() ) ) {
                this->add_mesh_face( nav.dart(), 1 );
            }
            if ( !mesh->is_marked ( nav.cell() ) || (mesh->is_marked ( nav.cell() ) && nav.dart().cell_neighbor != -1
                                                     && !mesh->is_marked ( nav.flip_cell().cell() ))) {
                this->add_mesh_face( nav.dart(), -1 );
            }
        }
    }

    void App::get_volume_geometry_volume() {
        visible_model.mesh_vert_pos.clear();
        visible_model.mesh_vert_norm.clear();
        visible_model.mesh_vert_attribute.clear();
        visible_model.mesh_ibuffer.clear();
        TriangleSet triangle_set;

        for ( size_t i = 0; i < mesh->faces.size(); ++i ) {
            MeshNavigator nav = mesh->navigate ( mesh->faces[i] );

            bool addPositiveNormal = false, addNegativeNormal = false;

            // Add all front faces to the full mesh.
            if ( !mesh->is_marked ( nav.cell() ) ) {
                addPositiveNormal = true;
            }
            if ( nav.dart().cell_neighbor != -1 && !mesh->is_marked ( nav.flip_cell().cell() ) ) {
                addNegativeNormal = true;
            }
            if (addPositiveNormal || addNegativeNormal) {
                this->add_mesh_face_volume( nav.dart(), addPositiveNormal, addNegativeNormal );
            }
        }
    }

    void App::build_gap_hexa ( const glm::vec3 pp[8], const glm::vec3 nn[6], const bool vv[8], const float ww[6] ) {
        if ( !vv[0] && !vv[1] && !vv[2] && !vv[3] && !vv[4] && !vv[5] && !vv[6] && !vv[7] ) {
            return;
        }

        float gap = this->max_crack_size * crack_size;
        glm::vec3 bari ( 0, 0, 0 );

        for ( int i = 0; i < 8; i++ ) {
            bari += pp[i];
        }

        bari *= ( gap / 8 );
        auto addSide = [&] ( int v0, int v1, int v2, int v3, int fi ) {
            if ( vv[v0] || vv[v1] || vv[v2] || vv[v3] ) add_quad (
                    add_vertex ( ( !vv[v0] ) ? pp[v0] : ( pp[v0] * ( 1 - gap ) + bari ), nn[fi], ww[fi] ),
                    add_vertex ( ( !vv[v3] ) ? pp[v3] : ( pp[v3] * ( 1 - gap ) + bari ), nn[fi], ww[fi] ),
                    add_vertex ( ( !vv[v2] ) ? pp[v2] : ( pp[v2] * ( 1 - gap ) + bari ), nn[fi], ww[fi] ),
                    add_vertex ( ( !vv[v1] ) ? pp[v1] : ( pp[v1] * ( 1 - gap ) + bari ), nn[fi], ww[fi] )
                );
        };
        //    P6------P7
        //   / |     / |
        //  P2------P3 |
        //  |  |    |  |
        //  | P4----|--P5
        //  | /     | /
        //  P0------P1
        //
        addSide ( 0 + 0, 2 + 0, 6 + 0, 4 + 0, 0 );
        addSide ( 2 + 1, 0 + 1, 4 + 1, 6 + 1, 1 );
        addSide ( 0 + 0, 1 + 0, 3 + 0, 2 + 0, 4 );
        addSide ( 1 + 4, 0 + 4, 2 + 4, 3 + 4, 5 );
        addSide ( 0 + 0, 4 + 0, 5 + 0, 1 + 0, 2 );
        addSide ( 4 + 2, 0 + 2, 1 + 2, 5 + 2, 3 );
        //addSide ( 0 + 0, 2 + 0, 6 + 0, 4 + 0, 1 );
        //addSide ( 2 + 1, 0 + 1, 4 + 1, 6 + 1, 0 );
        //addSide ( 0 + 0, 1 + 0, 3 + 0, 2 + 0, 5 );
        //addSide ( 1 + 4, 0 + 4, 2 + 4, 3 + 4, 4 );
        //addSide ( 0 + 0, 4 + 0, 5 + 0, 1 + 0, 3 );
        //addSide ( 4 + 2, 0 + 2, 1 + 2, 5 + 2, 2 );
        //{ 0, 1, 2, 3 },   // Front
        //{ 5, 4, 7, 6 },   // Back
        //{ 1, 5, 6, 2 },   // Left
        //{ 4, 0, 3, 7 },   // Right
        //{ 6, 7, 3, 2 },   // Bottom
        //{ 4, 5, 1, 0 },   // Top
    }
    /*
    float len(vec3 p){
    return p[0]*p[0]+p[1]*p[1]+p[2]*p[2];
    }*/
    // 8 [pp]ositions, 6 [nn]ormals, 8 [vv]isible 6 [ww]hite_or_not
    void App::build_smooth_hexa ( const glm::vec3 pp[8], const glm::vec3 nn[6], const bool vv[8], const bool ww[6], Index cell_idx ) {
        if ( !vv[0] && !vv[1] && !vv[2] && !vv[3] && !vv[4] && !vv[5] && !vv[6] && !vv[7] ) {
            return;
        }

        static glm::vec3 p[4][4][4];
        static glm::vec3 n[4][4][4];
        static bool     w[4][4][4]; // TODO
        static int     iv[4][4][4]; // indices
        auto addSide = [&] ( int v0, int v1, int v2, int v3, bool side ) {
            if ( ( v0 != -1 ) && ( v1 != -1 ) && ( v2 != -1 ) && ( v3 != -1 ) ) {
                add_quad ( v0, v1, v2, v3 );
            } else if ( side ) {
                if ( ( v0 != -1 ) && ( v1 != -1 ) && ( v2 != -1 ) ) {
                    add_triangle ( v0, v1, v2 );
                } else if ( ( v0 != -1 ) && ( v1 != -1 ) && ( v3 != -1 ) ) {
                    add_triangle ( v0, v1, v3 );
                } else if ( ( v0 != -1 ) && ( v2 != -1 ) && ( v3 != -1 ) ) {
                    add_triangle ( v0, v2, v3 );
                } else if ( ( v1 != -1 ) && ( v2 != -1 ) && ( v3 != -1 ) ) {
                    add_triangle ( v1, v2, v3 );
                }
            }
        };

        // compute normals / whites (from per face to per vertex)
        for ( int z = 0; z < 4; z++ ) {
            for ( int y = 0; y < 4; y++ ) {
                for ( int x = 0; x < 4; x++ ) {
                    n[x][y][z] = glm::vec3 ( 0, 0, 0 ); // todo: memfill or something - ?
                    w[x][y][z] = false;
                }
            }
        }

        for ( int z = 0; z < 4; z++ ) {
            for ( int y = 0; y < 4; y++ ) {
                n[0][y][z] += nn[0];
                n[3][y][z] += nn[1];
                n[y][0][z] += nn[2];
                n[y][3][z] += nn[3];
                n[y][z][0] += nn[4];
                n[y][z][3] += nn[5];
                w[0][y][z] |= ww[0];    // TODO
                w[3][y][z] |= ww[1];
                w[y][0][z] |= ww[2];
                w[y][3][z] |= ww[3];
                w[y][z][0] |= ww[4];
                w[y][z][3] |= ww[5];
            }
        }

        for ( int i = 0; i < 4; ++i ) {
            for ( int j = 0; j < 4; ++j ) {
                for ( int k = 0; k < 4; ++k ) {
                    n[i][j][k] = glm::normalize(n[i][j][k]);
                }
            }
        }

        for ( int z = 0; z < 4; z++ ) {
            for ( int y = 0; y < 4; y++ ) {
                for ( int x = 0; x < 4; x++ ) {
                    int ii = ( x / 2 ) + ( y / 2 ) * 2 + ( z / 2 ) * 4;
                    p[x][y][z] = pp[ii];
                    iv[x][y][z] = -1;
                }
            }
        }

        float smooth = this->max_rounding_radius * this->rounding_radius;

        for ( int z = 0; z < 4; z += 3 ) {
            for ( int y = 0; y < 4; y += 3 ) {
                for ( int x = 0; x < 4; x += 3 ) {
                    //std::cout << " ";
                    for ( int D = 0; D < 3; D++ ) {
                        int dA[3] = { 0, 0, 0 };
                        int dB[3] = { 0, 0, 0 };
                        int dC[3] = { 0, 0, 0 };
                        int s[3];
                        dA[D] = 1;
                        dB[ ( D + 1 ) % 3] = 1;
                        dC[ ( D + 2 ) % 3] = 1;
                        s[0] = ( x > 1 ) ? -1 : +1;
                        s[1] = ( y > 1 ) ? -1 : +1;
                        s[2] = ( z > 1 ) ? -1 : +1;
                        int ii = ( x / 2 ) + ( y / 2 ) * 2 + ( z / 2 ) * 4;

                        if ( vv[ii] ) {
                            int jj = ( ii ^ ( 1 << D ) );
                            glm::vec3 edge0 = ( pp[jj] - pp[ii] ) * smooth;
                            glm::vec3 edge1 = ( pp[jj] - pp[ii] ) * ( smooth * ( 1 - ( sqrt ( 2.0f ) / 2 ) ) );
                            glm::vec3 edge2 = ( pp[jj] - pp[ii] ) * ( smooth * ( 1 - ( sqrt ( 3.0f ) / 3 ) ) );
                            p[x + s[0] * ( dA[0] )][y + s[1] * ( dA[1] )][z + s[2] * ( dA[2] )] += edge0;
                            p[x + s[0] * ( dB[0] + dA[0] )][y + s[1] * ( dB[1] + dA[1] )][z + s[2] * ( dB[2] + dA[2] )] += edge0;
                            p[x + s[0] * ( dC[0] + dA[0] )][y + s[1] * ( dC[1] + dA[1] )][z + s[2] * ( dC[2] + dA[2] )] += edge0;
                            p[x + s[0] * ( dB[0] )][y + s[1] * ( dB[1] )][z + s[2] * ( dB[2] )] += edge1;
                            p[x + s[0] * ( dC[0] )][y + s[1] * ( dC[1] )][z + s[2] * ( dC[2] )] += edge1;
                            p[x][y][z] += edge2;
                        }
                    }
                }
            }
        }

        for ( int z = 0; z < 4; z++ ) {
            for ( int y = 0; y < 4; y++ ) {
                for ( int x = 0; x < 4; x++ ) {
                    int i = ( x / 2 ) + ( y / 2 ) * 2 + ( z / 2 ) * 4;
                    bool mx = ( ( x > 0 ) && ( x < 3 ) ); // mid
                    bool my = ( ( y > 0 ) && ( y < 3 ) );
                    bool mz = ( ( z > 0 ) && ( z < 3 ) );
                    int category = 0;

                    if ( mx ) {
                        category++;
                    }

                    if ( my ) {
                        category++;
                    }

                    if ( mz ) {
                        category++;
                    }

                    int j = i;

                    if ( category == 1 ) {
                        if ( mx ) {
                            j ^= 1;
                        }

                        if ( my ) {
                            j ^= 2;
                        }

                        if ( mz ) {
                            j ^= 4;
                        }
                    }

                    /*
                    v[i][j][k] =    (category==3)  // midpoint: never show
                    || (category==0 && !vv[i])  // corner: show only if corner visible
                    || (category==1 && !vv[i] && !vv[j])  // mid edge: show only if corner visible
                    || (category==2 && !vv[i]); // mid face: show only if corner visible*/

                    if ( category == 0 && !vv[i] ) {
                        continue;    // corner: show only if corner visible
                    }

                    if ( category == 1 && !vv[i] && !vv[j] ) {
                        continue;    // mid edge: show only if corner visible
                    }

                    if ( category == 2 && !vv[i] ) {
                        continue;    // mid face: show only if corner visible*/
                    }

                    if ( category == 3 ) {
                        continue;    // midpoint: never show
                    }

                    float attr;

                    if ( is_quality_color_mapping_enabled() ) {
                        //c = color_map.get ( mesh->normalized_hexa_quality[cell_idx] );
                        attr = 1.0f - mesh->normalized_hexa_quality[cell_idx];
                    } else {
                        attr = w[x][y][z] ? this->default_outside_attribute : this->default_inside_attribute;
                    }

                    iv[x][y][z] = add_vertex ( p[x][y][z], n[x][y][z], attr );
                    //std::cout<<"Range = "<<len(p[x][y][z]-vec3(1,1,1))<<"\n"; // test: smooth = 0.5 --> perfect sphere
                }
            }
        }

        for ( int x = 0; x < 3; x++ ) {
            for ( int y = 0; y < 3; y++ ) {
                bool mx = ( x == 1 ); // mid
                bool my = ( y == 1 );
                int category = 0;

                if ( mx ) {
                    category++;
                }

                if ( my ) {
                    category++;
                }

                addSide ( iv[x][y][0], iv[x][y + 1][0], iv[x + 1][y + 1][0], iv[x + 1][y][0], category == 1 );
                addSide ( iv[x][y][3], iv[x + 1][y][3], iv[x + 1][y + 1][3], iv[x][y + 1][3], category == 1 );
                addSide ( iv[x][0][y], iv[x + 1][0][y], iv[x + 1][0][y + 1], iv[x][0][y + 1], category == 1 );
                addSide ( iv[x][3][y], iv[x][3][y + 1], iv[x + 1][3][y + 1], iv[x + 1][3][y], category == 1 );
                addSide ( iv[0][x][y], iv[0][x][y + 1], iv[0][x + 1][y + 1], iv[0][x + 1][y], category == 1 );
                addSide ( iv[3][x][y], iv[3][x + 1][y], iv[3][x + 1][y + 1], iv[3][x][y + 1], category == 1 );
            }
        }
    }
    void App::prepare_cracked_geometry() {
        for ( size_t i = 0; i < this->mesh->verts.size(); ++i ) {
            this->mesh->unmark ( this->mesh->verts[i] );
        }

        auto mark_face_as_visible = [] ( Mesh * mesh, Dart & dart ) {
            MeshNavigator nav = mesh->navigate ( dart );
            Vert& vert = nav.vert();

            do {
                mesh->mark ( nav.vert() );
                nav = nav.rotate_on_face();
            } while ( nav.vert() != vert );
        };

        for ( size_t i = 0; i < mesh->faces.size(); ++i ) {
            MeshNavigator nav = mesh->navigate ( mesh->faces[i] );

            if ( !mesh->is_marked ( nav.cell() ) && ( nav.dart().cell_neighbor == -1 || mesh->is_marked ( nav.flip_cell().cell() ) ) ) {
                mark_face_as_visible ( this->mesh, nav.dart() );
            } else if ( mesh->is_marked ( nav.cell() ) && nav.dart().cell_neighbor != -1 && !mesh->is_marked ( nav.flip_cell().cell() ) ) {
                mark_face_as_visible ( this->mesh, nav.dart() );
            }
        }

        for ( size_t i = 0; i < mesh->faces.size(); ++i ) {
            MeshNavigator nav = mesh->navigate ( mesh->faces[i] );

            if ( mesh->is_marked ( nav.cell() ) && nav.dart().cell_neighbor == -1 ) {
                this->add_filtered_face ( nav.flip_edge().dart() );
            }
        }

        for ( size_t i = 0; i < mesh->cells.size(); ++i ) {
            if ( mesh->is_marked ( mesh->cells[i] ) ) {
                continue;
            }

            // ******
            //    P6------P7
            //   / |     / |
            //  P2------P3 |
            //  |  |    |  |
            //  | P4----|--P5
            //  | /     | /
            //  P0------P1
            glm::vec3 verts_pos[8];
            bool      verts_vis[8];
            float     faces_attributes[6];
            glm::vec3 faces_norms[6];
            // ******
            // Extract face normals
            MeshNavigator nav = this->mesh->navigate ( mesh->cells[i] );
            Face& face = nav.face();
            glm::vec3    norms_buffer[6];
            float        attributes_buffer[6];

            for ( size_t f = 0; f < 6; ++f ) {
                MeshNavigator n2 = this->mesh->navigate ( nav.face() );
                float normal_sign;

                if ( n2.cell() == nav.cell() ) {
                    normal_sign = 1;
                } else {
                    normal_sign = -1;
                    n2 = n2.flip_cell().flip_edge();
                }

                norms_buffer[f] = n2.face().normal * normal_sign;

                // color
                if ( is_quality_color_mapping_enabled() ) {
                    //colors_buffer[f] = color_map.get ( mesh->normalized_hexa_quality[nav.cell_index()] );
                    attributes_buffer[f] = 1.0f - mesh->normalized_hexa_quality[nav.cell_index()];
                } else {
                    attributes_buffer[f] =
                            nav.is_face_boundary() ? this->default_outside_attribute : this->default_inside_attribute;
                }

                nav = nav.next_cell_face();
            }

            faces_norms[0] = norms_buffer[4];
            faces_norms[1] = norms_buffer[1];
            faces_norms[2] = norms_buffer[5];
            faces_norms[3] = norms_buffer[2];
            faces_norms[4] = norms_buffer[0];
            faces_norms[5] = norms_buffer[3];
            faces_attributes[0] = attributes_buffer[4];
            faces_attributes[1] = attributes_buffer[1];
            faces_attributes[2] = attributes_buffer[5];
            faces_attributes[3] = attributes_buffer[2];
            faces_attributes[4] = attributes_buffer[0];
            faces_attributes[5] = attributes_buffer[3];
            // Extract vertices
            nav = mesh->navigate ( mesh->cells[i] );
            auto store_vert = [&] ( size_t i ) {
                verts_pos[i] = nav.vert().position;
                verts_vis[i] = mesh->is_marked ( nav.vert() );
            };
            nav = mesh->navigate ( mesh->cells[i] ).flip_vert();
            store_vert ( 1 );
            nav = mesh->navigate ( mesh->cells[i] );
            store_vert ( 0 );
            nav = mesh->navigate ( mesh->cells[i] ).flip_side().flip_vert();
            store_vert ( 5 );
            nav = mesh->navigate ( mesh->cells[i] ).flip_side();
            store_vert ( 4 );
            nav = mesh->navigate ( mesh->cells[i] ).flip_vert().flip_edge().flip_vert();
            store_vert ( 3 );
            nav = mesh->navigate ( mesh->cells[i] ).flip_edge().flip_vert();
            store_vert ( 2 );
            nav = mesh->navigate ( mesh->cells[i] ).flip_side().flip_vert().flip_edge().flip_vert();
            store_vert ( 7 );
            nav = mesh->navigate ( mesh->cells[i] ).flip_side().flip_edge().flip_vert();
            store_vert ( 6 );
            build_gap_hexa ( verts_pos, faces_norms, verts_vis, faces_attributes );
        }
    }
    void App::prepare_smooth_geometry() {
        for ( size_t i = 0; i < this->mesh->verts.size(); ++i ) {
            this->mesh->unmark ( this->mesh->verts[i] );
        }

        auto mark_face_as_visible = [] ( Mesh * mesh, Dart & dart ) {
            MeshNavigator nav = mesh->navigate ( dart );
            Vert& vert = nav.vert();

            do {
                mesh->mark ( nav.vert() );
                nav = nav.rotate_on_face();
            } while ( nav.vert() != vert );
        };

        for ( size_t i = 0; i < mesh->faces.size(); ++i ) {
            MeshNavigator nav = mesh->navigate ( mesh->faces[i] );

            if ( !mesh->is_marked ( nav.cell() ) && ( nav.dart().cell_neighbor == -1 || mesh->is_marked ( nav.flip_cell().cell() ) ) ) {
                mark_face_as_visible ( this->mesh, nav.dart() );
            } else if ( mesh->is_marked ( nav.cell() ) && nav.dart().cell_neighbor != -1 && !mesh->is_marked ( nav.flip_cell().cell() ) ) {
                mark_face_as_visible ( this->mesh, nav.dart() );
            }
        }

        for ( size_t i = 0; i < mesh->faces.size(); ++i ) {
            MeshNavigator nav = mesh->navigate ( mesh->faces[i] );

            if ( mesh->is_marked ( nav.cell() ) && nav.dart().cell_neighbor == -1 ) {
                this->add_filtered_face ( nav.flip_edge().dart() );
            }
        }

        for ( size_t i = 0; i < mesh->cells.size(); ++i ) {
            if ( mesh->is_marked ( mesh->cells[i] ) ) {
                continue;
            }

            // ******
            //    P6------P7
            //   / |     / |
            //  P4------P5 |
            //  |  |    |  |
            //  | P2----|--P3
            //  | /     | /
            //  P0------P1
            glm::vec3    verts_pos[8];
            bool        verts_vis[8];
            bool        faces_vis[6]; // true: external, false: internal
            glm::vec3    faces_norms[6];
            // ******
            // Extract face normals
            MeshNavigator nav = this->mesh->navigate ( mesh->cells[i] );
            Face& face = nav.face();
            glm::vec3    norms_buffer[6];
            bool        vis_buffer[6];

            for ( size_t f = 0; f < 6; ++f ) {
                MeshNavigator n2 = this->mesh->navigate ( nav.face() );
                float normal_sign;

                if ( n2.cell() == nav.cell() ) {
                    normal_sign = 1;
                } else {
                    normal_sign = -1;
                    n2 = n2.flip_cell().flip_edge();
                }

                norms_buffer[f] = n2.face().normal * normal_sign;
                vis_buffer[f] = n2.dart().cell_neighbor == -1;
                nav = nav.next_cell_face();
            }

            faces_norms[0] = norms_buffer[4];
            faces_norms[1] = norms_buffer[1];
            faces_norms[2] = norms_buffer[5];
            faces_norms[3] = norms_buffer[2];
            faces_norms[4] = norms_buffer[0];
            faces_norms[5] = norms_buffer[3];
            faces_vis[0] = vis_buffer[4];
            faces_vis[1] = vis_buffer[1];
            faces_vis[2] = vis_buffer[5];
            faces_vis[3] = vis_buffer[2];
            faces_vis[4] = vis_buffer[0];
            faces_vis[5] = vis_buffer[3];
            // Extract vertices
            nav = mesh->navigate ( mesh->cells[i] );
            auto store_vert = [&] ( size_t i ) {
                verts_pos[i] = nav.vert().position;
                verts_vis[i] = mesh->is_marked ( nav.vert() );
            };
            nav = mesh->navigate ( mesh->cells[i] ).flip_vert();
            store_vert ( 1 );
            nav = mesh->navigate ( mesh->cells[i] );
            store_vert ( 0 );
            nav = mesh->navigate ( mesh->cells[i] ).flip_side().flip_vert();
            store_vert ( 5 );
            nav = mesh->navigate ( mesh->cells[i] ).flip_side();
            store_vert ( 4 );
            nav = mesh->navigate ( mesh->cells[i] ).flip_vert().flip_edge().flip_vert();
            store_vert ( 3 );
            nav = mesh->navigate ( mesh->cells[i] ).flip_edge().flip_vert();
            store_vert ( 2 );
            nav = mesh->navigate ( mesh->cells[i] ).flip_side().flip_vert().flip_edge().flip_vert();
            store_vert ( 7 );
            nav = mesh->navigate ( mesh->cells[i] ).flip_side().flip_edge().flip_vert();
            store_vert ( 6 );
            build_smooth_hexa ( verts_pos, faces_norms, verts_vis, faces_vis, nav.cell_index() );
        }
    }
    void App::build_surface_models() {
        if ( mesh == nullptr ) {
            return;
        }

        visible_model.clear();
        filtered_model.clear();

        switch ( this->geometry_mode ) {
            case GeometryMode::Default:
                this->prepare_geometry();
                break;

            case GeometryMode::Cracked:
                this->prepare_cracked_geometry();
                break;

            case GeometryMode::Smooth:
                this->prepare_smooth_geometry();
                break;
        }
    }

    void App::build_full_model() {
        if ( mesh == nullptr ) {
            return;
        }

        this->full_model.clear();

        for ( size_t i = 0; i < mesh->faces.size(); ++i ) {
            MeshNavigator nav = mesh->navigate ( mesh->faces[i] );

            // cell a visible, cell b not existing or not visible
            if ( nav.dart().cell_neighbor == -1 ) {
                this->add_full_face ( nav.dart() );
            }
        }
    }

    void App::erode_dilate( int str ) {
        for ( int i=0; i<str; i++ ) erode();
        for ( int i=0; i<str; i++ ) dilate();
    }


    // filter -> mark vertices -> inc mark -> manual mark update -> re-mark vertices -> ...
    void App::erode() {
        for ( size_t i = 0; i < this->mesh->verts.size(); ++i ) {
            this->mesh->unmark ( this->mesh->verts[i] );
        }

        auto mark_face_as_visible = [] ( Mesh * mesh, Dart & dart ) {
            MeshNavigator nav = mesh->navigate ( dart );
            Vert& vert = nav.vert();

            do {
                mesh->mark ( nav.vert() );
                nav = nav.rotate_on_face();
            } while ( nav.vert() != vert );
        };

        for ( size_t i = 0; i < mesh->faces.size(); ++i ) {
            MeshNavigator nav = mesh->navigate ( mesh->faces[i] );

            if ( !mesh->is_marked ( nav.cell() ) && ( nav.dart().cell_neighbor == -1 || mesh->is_marked ( nav.flip_cell().cell() ) ) ) {
                mark_face_as_visible ( this->mesh, nav.dart() );
            } else if ( mesh->is_marked ( nav.cell() ) && nav.dart().cell_neighbor != -1 && !mesh->is_marked ( nav.flip_cell().cell() ) ) {
                mark_face_as_visible ( this->mesh, nav.dart() );
            }
        }

        for ( size_t i = 0; i < mesh->cells.size(); ++i ) {
            if ( mesh->is_marked ( mesh->cells[i] ) ) {
                continue;
            }

            MeshNavigator nav = mesh->navigate ( mesh->cells[i] );
            bool exit = false;

            for ( size_t j = 0; j < 4; ++j ) {
                if ( mesh->is_marked ( nav.vert() ) && !nav.vert().is_surface ) {
                    mesh->mark ( nav.cell() );
                    exit = true;
                    break;
                }

                nav = nav.rotate_on_face();
            }

            if ( exit ) {
                continue;
            }

            nav = nav.rotate_on_cell().rotate_on_cell();

            for ( size_t j = 0; j < 4; ++j ) {
                if ( mesh->is_marked ( nav.vert() ) && !nav.vert().is_surface ) {
                    mesh->mark ( nav.cell() );
                    break;
                }

                nav = nav.rotate_on_face();
            }
        }
    }

    void App::dilate() {
        for ( size_t i = 0; i < this->mesh->verts.size(); ++i ) {
            this->mesh->unmark ( this->mesh->verts[i] );
        }

        auto mark_face_as_visible = [] ( Mesh * mesh, Dart & dart ) {
            MeshNavigator nav = mesh->navigate ( dart );
            Vert& vert = nav.vert();

            do {
                mesh->mark ( nav.vert() );
                nav = nav.rotate_on_face();
            } while ( nav.vert() != vert );
        };

        for ( size_t i = 0; i < mesh->faces.size(); ++i ) {
            MeshNavigator nav = mesh->navigate ( mesh->faces[i] );

            if ( !mesh->is_marked ( nav.cell() ) && ( nav.dart().cell_neighbor == -1 || mesh->is_marked ( nav.flip_cell().cell() ) ) ) {
                mark_face_as_visible ( this->mesh, nav.dart() );
            } else if ( mesh->is_marked ( nav.cell() ) && nav.dart().cell_neighbor != -1 && !mesh->is_marked ( nav.flip_cell().cell() ) ) {
                mark_face_as_visible ( this->mesh, nav.dart() );
            }
        }

        for ( size_t i = 0; i < mesh->cells.size(); ++i ) {
            if ( !mesh->is_marked ( mesh->cells[i] ) ) {
                continue;
            }

            MeshNavigator nav = mesh->navigate ( mesh->cells[i] );
            bool exit = false;

            for ( size_t j = 0; j < 4; ++j ) {
                if ( mesh->is_marked ( nav.vert() ) && !nav.vert().is_surface ) {
                    mesh->unmark ( nav.cell() );
                    exit = true;
                    break;
                }

                nav = nav.rotate_on_face();
            }

            if ( exit ) {
                continue;
            }

            nav = nav.rotate_on_cell().rotate_on_cell();

            for ( size_t j = 0; j < 4; ++j ) {
                if ( mesh->is_marked ( nav.vert() ) && !nav.vert().is_surface ) {
                    mesh->unmark ( nav.cell() );
                    break;
                }

                nav = nav.rotate_on_face();
            }
        }
    }
}
