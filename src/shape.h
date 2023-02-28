#pragma once

#include <vector>
#include <array>

struct SegmentedCurve
{
    std::vector<std::array<double,2>> points;

    double get(double x);
};

class RocketProfileShape
{
    RocketProfileShape* next = nullptr;
    const RocketProfileShape* prev;

    const std::array<double,3> start; // x, r, s
    const std::array<double,3> finish; // x, r, s

public:

    RocketProfileShape(std::array<double,2> _start,std::array<double,2> _finish);
    RocketProfileShape(RocketProfileShape* _prev, std::array<double,2> _finish);
    virtual ~RocketProfileShape();

    inline bool in_bounds(double x) const
    {
        return x < start[0] || x > finish[0];
    }

    virtual double get_r_from_x(double x) const = 0;

    virtual double get_r_from_s(double s) const = 0;
};

class Line : public virtual RocketProfileShape
{

public:
    Line(std::array<double,2> _start, std::array<double,2> _finish);
    Line(RocketProfileShape* _prev, std::array<double,2> _finish);

    double get_r_from_x(double x) const, override;
};

class Arc : public virtual RocketProfileShape
{
    std::array<double,2> center;
    double radius;

public:

    static constexpr double START_TOP = M_PI/2.0;
    static constexpr double START_BOTTOM = -M_PI/2.0;

    Arc(RocketProfileShape* _prev, std::array<double,2> _finish, std::array<double,2> _center);

    static Arc* create(RocketProfileShape* _prev, double radius, double start_angle, double finish_angle);

    double get_r_from_x(double x) const, override;
};

class Parabola : public virtual RocketProfileShape
{
    std::array<double,2> coef;

public:

    Parabola(RocketProfileShape* _prev, std::array<double,2> _finish, std::array<double,2> _coef);

    static Parabola* create(RocketProfileShape* _prev, double drdx_start, double r_finish);

    double get_r_from_x(double x) const, override;
};

class RocketProfile

    RocketProfileShape* start = nullptr;
    RocketProfileShape* finish = nullptr;

    double dx_ratio = 0.1; // delta_x is

public:

    RocketProfile();
    ~RocketProfile();

    inline void set_shape_parameters(double dx_ratio)
    {
        this->dx_ratio = dx_ratio;
    }

    void set_blended_chamber(double combustor_radius,
                             double throat_radius,
                             double combustor_angle,
                             double fillet_radius,
                             double length) = 0;

    void set_chamber(   double combustor_radius,
                        double combustor_inlet_radius,
                        double throat_radius,
                        double combustor_angle,
                        double chamber_fillet_radius,
                        double throat_fillet_radius,
                        double length,
                        double throat_length);

    void set_conical_nozzle(double nozzle_fillet,
                            double cone_angle,
                            double exit_radius);

    void set_bell_nozzle(double nozzle_fillet,
                         double cone_angle,
                         double exit_radius);

    void set_parabolic_nozzle(double nozzle_fillet,
                              double cone_angle,
                              double exit_radius);

    SegmentedCurve generate();
};
