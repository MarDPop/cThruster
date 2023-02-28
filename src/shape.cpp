#include "shape.h"

#include <cmath>

RocketProfileShape::RocketProfileShape(std::array<double,3> _start,std::array<double,3> _finish) : prev(nullptr), start(_start), finish(_finish)
{}

RocketProfileShape::RocketProfileShape(RocketProfileShape* _prev, std::array<double,3> _finish) : prev(_prev), start(prev->finish), finish(_finish)

RocketProfileShape::~RocketProfileShape()
{
    delete next;
}

Line::Line(std::array<double,3> _start, std::array<double,3> _finish) : RocketProfileShape(_start,_finish) {}

Line::Line(RocketProfileShape* _prev, std::array<double,3> _finish) : RocketProfileShape(_prev,_finish) {}

double Line::get_r_from_x(double x) const
{
    double dx = x - start[0];
    double dydx = (finish[1] - start[1])/(finish[0] - start[0]);
    return dx*dydx + start[1];
}

std::array<double,2> Line::get_point_from_s(double s) const
{
    double deltaS = s - start[2];
    double dx = (finish[0] - start[0]);
    double dy = (finish[1] - start[1]);
    double ds = 1.0/sqrt(dx*dx + dy*dy);
    double dxds = dx*ds;
    double dyds = dy*ds;
    return {start[0] + deltaS*dxds, start[1] + deltaS*dyds};
}


Arc::Arc(RocketProfileShape* _prev, std::array<double,3> _finish, std::array<double,2> _center) : RocketProfileShape(_prev,_finish), center(_center)
{
    double dx = center[0] - start[0];
    double dy = center[1] - start[1];
    this->radius = sqrt(dx*dx + dy*dy);
}

Arc* Arc::from_radius(RocketProfileShape* _prev, double radius, double start_angle, double finish_angle)
{
    std::array<double,2> center;
    center[0] = prev->finish[0] - cos(start_angle)*radius;
    center[1] = prev->finish[0] - sin(start_angle)*radius;
    std::array<double,3> finish;
    finish[0] = center[0] + cos(finish_angle)*radius;
    finish[1] = center[1] + sin(finish_angle)*radius;
    finish[2] = prev->finish[0] + radius*fabs(finish_angle - start_angle);
    return new Arc(_prev, finish, center);
}

double Arc::get_r_from_x(double x) const
{
    double dx = x - center[0];
    double dr = sqrt(this->radius*this->radius - dx*dx);
    if(start[1] < center[1])
    {
        return center[1] - dr;
    }
    else
    {
        return center[1] + dr;
    }
}

std::array<double,2> Arc::get_point_from_s(double s) const
{
    double deltaS = s - this->start[2];
    double angle2subtend = deltaS / this->radius;

    std::array<double,2> start_vec = {this->start[0] - this->center[0], this->start[1] - this->center[1]};
    std::array<double,2> finish_vec = {this->finish[0] - this->center[0], this->finish[1] - this->center[1]};

    // slerp
    double omega = acos((start_vec[0]*finish_vec[0] + start_vec[1]*finish_vec[1])/(this->radius*this->radius));
    double t = angle2subtend / omega;

    double sOmega = 1.0/sin(omega);
    double h1 = sin(angle2subtend);
    double h2 = sin(omega - angle2subtend);

    std::array<double,2> output;
    output[0] = (h2*start_vec[0] + h1*finish_vec[0])*sOmega + this->start[0];
    output[1] = (h2*start_vec[1] + h1*finish_vec[1])*sOmega + this->start[1];
    return output;
}

Parabola::Parabola(RocketProfileShape* _prev, std::array<double,2> _finish, std::array<double,2> _coef) : RocketProfileShape(_prev,_finish), coef(_coef)

Parabola* Parabola::create(RocketProfileShape* _prev, double drdx_start, double r_finish)
{
    std::array<double,2> coef;
    coef[1] = _prev->finish[1]*_prev->finish[1];
    coef[0] = drdx_start*2*_prev->finish[1];
    std::array<double,2> finish;
    finish[0] = (r_finish*r_finish - coef[1])/coef[0];
    finish[1] = r_finish;
    return new Parabola(_prev,finish,coef);

}

double Parabola::get_r_from_x(double x) const
{
    double dx = x - start[0];
    return sqrt(coef[0]*dx + coef[1]);
}

std::array<double,2> Parabola::get_point_from_s(double s) const
{
    double deltaS = s - this->start[2];
    // ... you don't want to know the integral for this... just do it numerically

    std::array<double,2> output;
    return output;
}

RocketProfile(){}

~RocketProfile()
{
    delete start;
}

void RocketProfile::set_chamber(double combustor_radius,
                        double combustor_inlet_radius,
                        double throat_radius,
                        double combustor_angle,
                        double chamber_fillet_radius,
                        double throat_fillet_radius,
                        double length,
                        double throat_length)
{
    delete start;
    std::array<double,2> point = {0,combustor_radius};
    std::array<double,2> next = {length,combustor_radius};
    this->start = new Line(point,next);
    this->finish = this->start;

    next[0] = length;
    next[1] = combustor_inlet_radius;
    this->finish.next = new Line(this->finish,finish);
    this->finish = this->finish.next;
    this->finish.next = Arc::create(this->finish, chamber_fillet_radius, M_PI, Arc::START_BOTTOM - chamber_fillet_radius);
    this->finish = this->finish.next;

    next[1] = throat_radius + sin(throat_fillet_radius);
    double dr = next[1] - this->finish->finish[1];
    double drdx = tan(-chamber_fillet_radius);
    double dx = dr/drdx;
    next[0] = this->finish->finish[0] + dx;
    this->finish.next = new Line(this->finish,finish);
    this->finish = this->finish.next;

    dx = sin(throat_fillet_radius);
    point[0] = this->finish->finish[0] + dx;
    point[1] = throat_radius + throat_fillet_radius;
    next[0] = point[0];
    next[1] = throat_radius;
    this->finish.next = Arc::create(this->finish,next,point);
    this->finish = this->finish.next;

    next[0] += throat_length;
    next[1] = throat_radius;
    this->finish.next = new Line(this->finish,next);
    this->finish = this->finish.next;
}

void RocketProfile::set_conical_nozzle(double nozzle_fillet,
                               double cone_angle,
                               double exit_radius)
{

}

void RocketProfile::set_bell_nozzle(double nozzle_fillet,
                            double cone_angle,
                            double exit_radius)
{

}

void RocketProfile::set_parabolic_nozzle(double nozzle_fillet,
                                 double cone_angle,
                                 double exit_radius);

std::vector<std::array<double,2>> RocketProfile::generate()
{
    std::vector<std::array<double,2>>
}
