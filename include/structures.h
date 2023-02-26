#pragma once

#include "boundary.h"
#include "fluid.h"

#include "../../../lib/Eigen"

struct Cell_Axisymmetric;

struct Face_Axisymmetric
{
    Fluid_Axisymmetric fluid;

    Eigen::Vector2d vertex[2];
    Eigen::Vector2d normal;
    double area;
    Cell_Axisymmetric cells[2];
    BoundaryCondition

};

struct Cell_Axisymmetric
{
    Face_Axisymmetric faces[4];

};
