#include "shape.h"

#include <fstream>
#include <vector>
#include <array>

void print(std::vector<std::array<double,2>>& curve, std::string fn)
{
    std::ofstream file(fn);

    for(auto point : curve)
    {
        file << point[0] << " " << point[1] << std::endl;
    }
}

void test()
{
    MotorProfile shape;

    double combustor_radius = 0.15;
    double combustor_inlet_radius = 0.1;
    double throat_radius = 0.05;
    double combustor_angle = 0.2;
    double chamber_fillet_radius = 0.005;
    double throat_fillet_radius = 0.005;
    double length = 0.5;
    double throat_length = 0;

    shape.set_chamber(  combustor_radius,
                        combustor_inlet_radius,
                        throat_radius,
                        combustor_angle,
                        chamber_fillet_radius,
                        throat_fillet_radius,
                        length,
                        throat_length);

    auto curve = shape.generate();

    print(curve,"curve.txt");
}

int main(int argc, char** argv)
{
    test();

    return 0;
}
