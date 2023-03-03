#include "structures.h"

Vertex_Axisymmtric::Vertex_Axisymmtric(){}

Vertex_Axisymmtric::Vertex_Axisymmtric(double x, double r) : point(x,r) {}

Vertex_Axisymmtric::Vertex_Axisymmtric(Eigen::Vector2d _point) : point(_point) {}

Fluid_Axisymmetric::Fluid_Axisymmetric(Vertex_Axisymmtric* p1, Vertex_Axisymmtric* p2) : vertex{p1,p2}
{

}

Cell_Axisymmetric::Cell_Axisymmetric(std::vector<Vertex_Axisymmtric*>& faces)
{

}


Cell_Axisymmetric::Cell_Axisymmetric(std::vector<Face_Axisymmetric*>& faces)
{

}
