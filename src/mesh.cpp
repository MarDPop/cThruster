#include "mesh.h"

double interp(const std::array<std::vector<double>,2>& profile, double x)
{
    if(x < profile[0].front())
    {
        return 0;
    }
    if(x > profile[0].back())
    {
        return 0;
    }

    int idx = 0;
    for(double x_profile : profile[0])
    {
        if(x_profile > x)
        {
            break;
        }
        idx++;
    }

    double drdx = (profile[1][idx] - profile[1][idx - 1])/(profile[0][idx] - profile[0][idx - 1]);
    double dx = x - profile[0][idx];
    return profile[1][idx] + dx*drdx;
}

Eigen::Vector2d get_offset_vector(Vertex_Axisymmtric* v1, Vertex_Axisymmtric* v2)
{
    auto face_direction = v2->point - v1->point;
    Eigen::Vector2d perpendicular_inside(face_direction[1],-face_direction[0]);
    return perpendicular_inside*(offset_distance/perpendicular_inside.norm());
}

void AxisymmetricMesh::merge_ghosts(  std::vector<GhostFace_Axisymmetric> ghost_faces,
                        std::vector<GhostCell_Axisymmetric> ghost_cells)
{

}

void add_wall_ghosts(  std::vector<GhostFace_Axisymmetric>& ghost_faces,
                            std::vector<GhostCell_Axisymmetric>& ghost_cells,
                            const std::array<std::vector<double>,2>& motor_profile)
{
    unsigned nPoints = motor_profile.size();
    unsigned nFaces = nPoints - 1;
    current_vertex_ids.resize(nFaces);
    for(unsigned vertex_idx = 0; vertex_idx < nPoints; vertex_idx++)
    {
        this->vertices.emplace_back(motor_profile[vertex_idx][0],motor_profile[vertex_idx][1]);
    }

    double offset_distance = options.wall_layer_first_height;
    for(int wall_layer = 0; wall_layer < options.prism_layers; wall_layer++)
    {
        int start_id = prism_layer*nPoints;
        auto* vertex_back = &this->vertices[start_id];
        auto* vertex = vertex_back;
        auto* vertex_front = vertex_back+1;
        auto offset = get_offset_vector(vertex_back, vertex_front);
        this->vertices.emplace_back(vertex->point + offset);

        vertex++;
        vertex_front++;
        for(unsigned idx = 1; idx < nPoints - 1; idx++)
        {
            offset = get_offset_vector(vertex_back, vertex_front);
            this->vertices.emplace_back(vertex->point + offset);

            vertex_back++;
            vertex++;
            vertex_front++;
        }
        vertex_front--;

        offset = get_offset_vector(vertex_back, vertex_front);
        this->vertices.emplace_back(vertex->point + offset);

        unsigned top_left_corner_idx = start_id;
        for(unsigned idx = 0; idx < nPoints - 1; idx++)
        {
            unsigned ghost_face_start_id = ghost_faces.size();

            unsigned top_right_corner_idx = top_left_corner_idx + 1;
            unsigned bottom_left_corner_idx = top_left_corner_idx + nPoints;
            unsigned bottom_right_corner_idx = bottom_left_corner_idx + 1;

            // add faces clockwise (ever cell center is rhs)
            ghost_faces.emplace_back(top_left_corner_idx, top_right_corner_idx); // top
            ghost_faces.emplace_back(top_right_corner_idx, bottom_right_corner_idx); // right
            ghost_faces.emplace_back(bottom_right_corner_idx, bottom_left_corner_idx); // bottom
            ghost_faces.emplace_back(bottom_left_corner_idx, top_left_corner_idx); // bottom

            ghost_cells.push_back();
            auto& new_cell = ghost_cells.back();
            new_cell.ghost_face_id.push_back(ghost_face_start_id);
            new_cell.ghost_face_id.push_back(ghost_face_start_id+1);
            new_cell.ghost_face_id.push_back(ghost_face_start_id+2);
            new_cell.ghost_face_id.push_back(ghost_face_start_id+3);

            top_left_corner_idx++;
        }
        offset_distance*= options.wall_geometric_fractor;
    }
}

void add_interior_ghosts(  std::vector<GhostFace_Axisymmetric>& ghost_faces,
                            std::vector<GhostCell_Axisymmetric>& ghost_cells,
                            const std::array<std::vector<double>,2>& motor_profile)
{

}

void AxisymmetricMesh::generate_from_profile(const std::array<std::vector<double>,2>& motor_profile, Generation_Options& options)
{
    std::vector<GhostFace_Axisymmetric> ghost_faces;
    std::vector<GhostCell_Axisymmetric> ghost_cells;



    std::vector<int> current_vertex_ids;
    std::vector<int> next_vertex_ids;


}
