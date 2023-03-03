#include "shape.h"

#include <fstream>
#include <vector>
#include <array>

void print(std::array<std::vector<double>,2>& curve, std::string fn)
{
    std::ofstream file(fn);

    int nPoint = curve[0].size();
    for(int idx = 0; idx < nPoint; idx++)
    {
        file << curve[0][i] << " " << curve[1][i] << std::endl;
    }
}

void test()
{
    MotorProfile shape;

    double combustor_radius = 0.15;
    double combustor_inlet_radius = 0.1;
    double throat_radius = 0.05;
    double combustor_angle = 0.4;
    double chamber_fillet_radius = 0.025;
    double throat_fillet_radius = 0.05;
    double length = 0.25;
    double throat_length = 0;

    shape.set_chamber(  combustor_radius,
                        combustor_inlet_radius,
                        throat_radius,
                        combustor_angle,
                        chamber_fillet_radius,
                        throat_fillet_radius,
                        length,
                        throat_length);


    double exit_radius = 0.1;
    int type = 2;
    if(type == 1)
    {
        shape.set_bell_nozzle(exit_radius,exit_radius*2);
    }
    else if (type == 2)
    {
        shape.set_parabolic_nozzle(exit_radius,25*0.017453292519943);
    }
    else
    {
        double cone_angle = 0.2;
        shape.set_conical_nozzle(  exit_radius, cone_angle );
    }


    auto curve = shape.generate();

    print(curve,"curve.txt");
}

int main(int argc, char** argv)
{
    test();

    return 0;
}
