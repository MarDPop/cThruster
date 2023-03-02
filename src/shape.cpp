#include "shape.h"

#include <cmath>

MotorProfileShape::MotorProfileShape(std::array<double,3> _start,std::array<double,3> _finish) : prev(nullptr), start(_start), finish(_finish)
{}

MotorProfileShape::MotorProfileShape(MotorProfileShape* _prev, std::array<double,3> _finish) : prev(_prev), start(prev->finish), finish(_finish) {}

MotorProfileShape::~MotorProfileShape()
{
    delete next;
}

Line::Line(std::array<double,3> _start, std::array<double,3> _finish) : MotorProfileShape(_start,_finish) {}

Line::Line(MotorProfileShape* _prev, std::array<double,3> _finish) : MotorProfileShape(_prev,_finish) {}

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
    double ds = sqrt(dx*dx + dy*dy);
    if(ds == 0)
    {
        return {start[0], start[1]};
    }
    double dxds = dx/ds;
    double dyds = dy/ds;
    return {start[0] + deltaS*dxds, start[1] + deltaS*dyds};
}


Arc::Arc(MotorProfileShape* _prev, std::array<double,3> _finish, std::array<double,2> _center) : MotorProfileShape(_prev,_finish), center(_center)
{
    double dx = center[0] - start[0];
    double dy = center[1] - start[1];
    this->radius = sqrt(dx*dx + dy*dy);
}

Arc* Arc::create(MotorProfileShape* _prev, double radius, double start_angle, double finish_angle)
{
    std::array<double,2> center;
    center[0] = _prev->finish[0] - cos(start_angle)*radius;
    center[1] = _prev->finish[0] - sin(start_angle)*radius;
    std::array<double,3> finish;
    finish[0] = center[0] + cos(finish_angle)*radius;
    finish[1] = center[1] + sin(finish_angle)*radius;
    finish[2] = _prev->finish[0] + radius*fabs(finish_angle - start_angle);
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

Parabola::Parabola(MotorProfileShape* _prev, std::array<double,3> _finish, std::array<double,2> _coef) : MotorProfileShape(_prev,_finish), coef(_coef) {}

Parabola* Parabola::create(MotorProfileShape* _prev, double drdx_start, double r_finish)
{
    if(r_finish < _prev->finish[1])
    {
        return nullptr;
    }
    std::array<double,2> coef;
    coef[1] = _prev->finish[1]*_prev->finish[1];
    coef[0] = drdx_start*2*_prev->finish[1];
    std::array<double,3> finish;
    finish[0] = (r_finish*r_finish - coef[1])/coef[0];
    finish[1] = r_finish;
    int nIntegrationSteps = 1000;
    double dx = finish[0]/nIntegrationSteps;
    double sFinish = 0;
    for(int i = 0; i < nIntegrationSteps;i++)
    {
        double x = i*dx;
        double drdx = coef[0]/(2*sqrt(coef[0]*x + coef[1]));
        double ds = sqrt(1 + drdx*drdx);
        sFinish += ds;
    }
    finish[2] = _prev->finish[2]+ sFinish*dx;
    finish[0] += _prev->finish[0];

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
    // integral for y
    int nIntegrationSteps = 1000;
    double dx = (this->finish[0] - this->start[0])/nIntegrationSteps;
    double C = coef[0]*coef[0]*0.25;
    double sIntegrated = 0;
    double x = 0;
    while(sIntegrated < s)
    {
        double dsdx = sqrt(1 + C/(coef[0]*x + coef[1])); // sqrt(1 + (dr/dx)^2)
        double ds = dsdx*dx;
        sIntegrated += ds;
        x += dx;
    }
    // back track overshoot
    double ds = sIntegrated - s;
    double dsdx = sqrt(1 + C/(coef[0]*x + coef[1]));
    dx = ds/dsdx;
    x += dx;
    std::array<double,2> output = {x , this->get_r_from_x(x)};
    return output;
}

MotorProfile::MotorProfile(){}

MotorProfile::~MotorProfile()
{
    delete this->first_shape;
}

void MotorProfile::set_blended_chamber(double combustor_radius,
                             double throat_radius,
                             double combustor_angle,
                             double fillet_radius,
                             double length)
{
    delete this->first_shape;
    std::array<double,3> point = {0,combustor_radius,0};
    std::array<double,3> next = {length,combustor_radius,length};
    this->first_shape = new Line(point,next);
    this->last_shape = this->first_shape;
}

void MotorProfile::set_chamber(double combustor_radius,
                        double combustor_inlet_radius,
                        double throat_radius,
                        double combustor_angle,
                        double chamber_fillet_radius,
                        double throat_fillet_radius,
                        double length,
                        double throat_length)
{
    delete this->first_shape;
    std::array<double,3> point = {0,combustor_radius,0};
    std::array<double,3> next = {length,combustor_radius,length};
    this->first_shape = new Line(point,next);
    this->last_shape = this->first_shape;

    if(combustor_inlet_radius > combustor_radius)
    {
        combustor_inlet_radius = combustor_radius;
    }

    next[0] = length;
    next[1] = combustor_inlet_radius;
    next[2] = this->last_shape->finish[2] + (combustor_radius - combustor_inlet_radius);
    this->last_shape->next = new Line(this->last_shape,next);
    this->last_shape = this->last_shape->next;
    this->last_shape->next = Arc::create(this->last_shape, chamber_fillet_radius, M_PI, Arc::START_BOTTOM - combustor_angle);
    this->last_shape = this->last_shape->next;

    next[1] = throat_radius + sin(throat_fillet_radius);
    double dr = next[1] - this->last_shape->finish[1];
    double drdx = tan(-combustor_angle);
    double dx = dr/drdx;
    next[0] = this->last_shape->finish[0] + dx;
    next[2] = this->last_shape->finish[2] + sqrt(dr*dr + dx*dx);
    this->last_shape->next = new Line(this->last_shape,next);
    this->last_shape = this->last_shape->next;

    dx = sin(throat_fillet_radius);
    std::array<double,2> center;
    center[0] = this->last_shape->finish[0] + dx;
    center[1] = throat_radius + throat_fillet_radius;
    next[0] = center[0];
    next[1] = throat_radius;
    next[2] = this->last_shape->finish[2] + throat_fillet_radius*combustor_angle;
    this->last_shape->next = new Arc(this->last_shape,next,center);
    this->last_shape = this->last_shape->next;

    next[0] += throat_length;
    next[1] = throat_radius;
    next[2] = this->last_shape->finish[2] + throat_length;
    this->last_shape->next = new Line(this->last_shape,next);
    this->last_shape = this->last_shape->next;
}

void MotorProfile::set_conical_nozzle(double nozzle_fillet,
                               double cone_angle,
                               double exit_radius,
                               double initial_turn_radius_fraction)
{
    double throat_exit_radius = initial_turn_radius_fraction*this->last_shape->finish[1];

    this->last_shape->next = Arc::create(this->last_shape, throat_exit_radius, Arc::START_BOTTOM, Arc::START_BOTTOM + cone_angle);
    this->last_shape = this->last_shape->next;

    std::array<double,3> next;
    next[1] = exit_radius;
    next[0] = (exit_radius - this->last_shape->finish[1]) / tan(cone_angle);
    double dx = next[0] - this->last_shape->finish[0];
    double dr = next[1] - this->last_shape->finish[1];
    next[2] = this->last_shape->finish[2] + sqrt(dx*dx + dr*dr);
    this->last_shape->next = new Line(this->last_shape, next);
    this->last_shape = this->last_shape->next;
}

void MotorProfile::set_bell_nozzle(double nozzle_fillet,
                            double cone_angle,
                            double exit_radius,
                            double bell_radius,
                            double initial_turn_radius_fraction)
{
    if(bell_radius < exit_radius)
    {
        bell_radius = exit_radius;
    }

    double throat_exit_radius = initial_turn_radius_fraction*this->last_shape->finish[1];
    double projection = (exit_radius - this->last_shape->finish[1])/(bell_radius - throat_exit_radius);
    double turn_angle = acos(1 - projection);

    this->last_shape->next = Arc::create(this->last_shape, throat_exit_radius, Arc::START_BOTTOM, Arc::START_BOTTOM + turn_angle);
    this->last_shape = this->last_shape->next;

    this->last_shape->next = Arc::create(this->last_shape, bell_radius, Arc::START_TOP - turn_angle, Arc::START_TOP);
    this->last_shape = this->last_shape->next;

}

void MotorProfile::set_parabolic_nozzle(double nozzle_fillet,
                                 double cone_angle,
                                 double exit_radius,
                                 double initial_turn_angle,
                                 double initial_turn_radius_fraction)
{
    double throat_exit_radius = initial_turn_radius_fraction*this->last_shape->finish[1];

    this->last_shape->next = Arc::create(this->last_shape, throat_exit_radius, Arc::START_BOTTOM, Arc::START_BOTTOM + initial_turn_angle);
    this->last_shape = this->last_shape->next;

}

std::vector<std::array<double,2>> MotorProfile::generate()
{
    std::vector<std::array<double,2>> output;

    auto* shape = this->first_shape;

    double s = 0;
    while(shape)
    {
        auto point = shape->get_point_from_s(s);
        output.push_back(point);
        double ds = point[1]*this->dx_ratio;
        s += ds;

        while(!shape->s_in_bounds(s))
        {
            shape = shape->next;
            if(shape == nullptr)
            {
                break;
            }
        }
    }

    return output;
}
