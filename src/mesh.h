#pragma once

#include "structures.h"
#include "shape.h"

class AxisymmetricMesh
{
friend class Simulation;

    std::vector<Vertex_Axisymmtric> vertices;
    std::vector<Face_Axisymmetric> faces;
    std::vector<Cell_Axisymmetric> cells;

    void merge_ghosts(  const std::vector<GhostFace_Axisymmetric>& ghost_faces,
                        const std::vector<GhostCell_Axisymmetric>& ghost_cells);

    void add_wall_ghosts(  std::vector<GhostFace_Axisymmetric>& ghost_faces,
                            std::vector<GhostCell_Axisymmetric>& ghost_cells,
                            const std::array<std::vector<double>,2>& motor_profile);

    void add_interior_ghosts(  std::vector<GhostFace_Axisymmetric>& ghost_faces,
                                std::vector<GhostCell_Axisymmetric>& ghost_cells,
                                const std::array<std::vector<double>,2>& motor_profile);

public:

    struct Generation_Options
    {
        int wall_layers = 0;
        double wall_geometric_fractor = 1.2;
        double wall_layer_first_height = 0.0001;
        double axi_geometric_factor = 1.1;
    };

    AxisymmetricMesh();
    ~AxisymmetricMesh();

    void generate_from_profile(const std::array<std::vector<double>,2>& motor_profile, Generation_Options& options);

};
