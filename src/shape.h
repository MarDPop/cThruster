#pragma once

#include <vector>
#include <array>

#define M_PI 3.1415926535897932384626433832795

class MotorProfileShape
{
public:

    MotorProfileShape* next = nullptr;
    const MotorProfileShape* const prev;

    const std::array<double,3> start; // x, r, s
    const std::array<double,3> finish; // x, r, s

    MotorProfileShape(std::array<double,3> _start,std::array<double,3> _finish);
    MotorProfileShape(MotorProfileShape* _prev, std::array<double,3> _finish);
    virtual ~MotorProfileShape();

    inline bool s_in_bounds(double s) const
    {
        return s > start[2] && s < finish[2];
    }

    inline bool x_in_bounds(double x) const
    {
        return x > start[0] && x < finish[0];
    }

    virtual double get_r_from_x(double x) const = 0;

    virtual std::array<double,2> get_point_from_s(double s) const = 0;
};

class Line : public virtual MotorProfileShape
{

public:
    Line(std::array<double,3> _start, std::array<double,3> _finish);
    Line(MotorProfileShape* _prev, std::array<double,3> _finish);

    double get_r_from_x(double x) const override;

    std::array<double,2> get_point_from_s(double s) const override;
};

class Arc : public virtual MotorProfileShape
{
    std::array<double,2> center;
    double radius;

public:

    static constexpr double START_TOP = M_PI/2.0;
    static constexpr double START_BOTTOM = M_PI*1.5;

    Arc(MotorProfileShape* _prev, std::array<double,3> _finish, std::array<double,2> _center);

    static Arc* create(MotorProfileShape* _prev, double radius, double start_angle, double finish_angle);

    double get_r_from_x(double x) const override;

    std::array<double,2> get_point_from_s(double s) const override;
};

class Parabola : public virtual MotorProfileShape
{
    std::array<double,2> coef;

public:

    Parabola(MotorProfileShape* _prev, std::array<double,3> _finish, std::array<double,2> _coef);

    static Parabola* create(MotorProfileShape* _prev, double drdx_start, double r_finish);

    double get_r_from_x(double x) const override;

    std::array<double,2> get_point_from_s(double x) const override;
};

class MotorProfile
{
    MotorProfileShape* first_shape = nullptr;
    MotorProfileShape* last_shape = nullptr;

    double dx_ratio = 0.05; // delta_x is

public:

    MotorProfile();
    ~MotorProfile();

    inline void set_shape_parameters(double dx_ratio)
    {
        this->dx_ratio = dx_ratio;
    }


    void set_blended_chamber(double combustor_radius,
                             double throat_radius,
                             double combustor_angle,
                             double fillet_radius,
                             double length);

    void set_chamber(   double combustor_radius,
                        double combustor_inlet_radius,
                        double throat_radius,
                        double combustor_angle,
                        double chamber_fillet_radius,
                        double throat_fillet_radius,
                        double length,
                        double throat_length);

    void set_conical_nozzle(double exit_radius,
                            double cone_angle,
                            double initial_turn_radius_fraction = 0.382);

    void set_bell_nozzle(double exit_radius,
                        double bell_radius,
                        double initial_turn_radius_fraction = 0.382);

    void set_parabolic_nozzle(double exit_radius,
                             double initial_turn_angle,
                             double initial_turn_radius_fraction = 0.382);

    std::array<std::vector<double>,2> generate();
};
