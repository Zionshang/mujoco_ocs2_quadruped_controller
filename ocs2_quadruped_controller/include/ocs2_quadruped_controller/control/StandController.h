#pragma once
#include <vector>
#include <cmath>
#include <iostream>

class StandController
{
public:
    StandController(const std::vector<double> &middle_position,
                    const std::vector<double> &final_position);

    void setInitPosition(const std::vector<double> &init_position) { init_position_ = init_position; };

    double calcTargetPosition(int id_leg, double current_time, double finish_time);

    double TSpline_S_V_A(double p0, double v0, double a0, double t0,
                         double p1, double t1,
                         double p2, double v2, double a2, double t2,
                         double cont_T);

private:
    std::vector<double> init_position_;
    std::vector<double> middle_position_;
    std::vector<double> final_position_;
};
