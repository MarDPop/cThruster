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
    RocketProfile shape;

    auto curve = shape.generate();

    print(curve,"curve.txt");
}

int main(int argc, char** argv)
{
    test();

    return 0;
}
