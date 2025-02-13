#include "ocs2_quadruped_controller/control/StandController.h"

StandController::StandController(const std::vector<double> &middle_position,
                                 const std::vector<double> &final_position)
    : middle_position_(middle_position), final_position_(final_position) {}

double StandController::calcTargetPosition(int i, double current_time, double finish_time)
{
    double target_position;
    double percent = current_time / finish_time;

    if (percent < 0.5)
        target_position = TSpline_S_V_A(init_position_[i], 0, 0, 0,
                                        (middle_position_[i] + init_position_[i]) / 2, finish_time / 2,
                                        middle_position_[i], 0, 0, finish_time,
                                        10 * percent);
    else if (percent > 0.5)
        target_position = TSpline_S_V_A(middle_position_[i], 0, 0, 0,
                                        (final_position_[i] + middle_position_[i]) / 2, finish_time / 2,
                                        final_position_[i], 0, 0, finish_time,
                                        10 * (percent - 0.5));
    else
        target_position = final_position_[i];

    return target_position;
}

double StandController::TSpline_S_V_A(double p0, double v0, double a0, double t0,
                                      double p1, double t1,
                                      double p2, double v2, double a2, double t2,
                                      double cont_T)
{
    double P[10];
    double p, v, a;
    P[0] = (a2 * (pow(t0, 3.0) * pow(t1, 2.0) - t2 * pow(t0, 3.0) * t1)) /
               (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) + t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 -
                     t2 * pow(t1, 2.0))) -
           (v0 * (2 * pow(t0, 3.0) * pow(t1, 2.0) + t2 * pow(t0, 3.0) * t1 - pow(t0, 2.0) * pow(t1, 3.0) -
                  3 * t2 * pow(t0, 2.0) * pow(t1, 2.0) + t2 * t0 * pow(t1, 3.0))) /
               (pow(t0, 4.0) - 3 * pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) + 3 * pow(t0, 2.0) * pow(t1, 2.0) +
                3 * t2 * pow(t0, 2.0) * t1 - t0 * pow(t1, 3.0) - 3 * t2 * t0 * pow(t1, 2.0) + t2 * pow(t1, 3.0)) -
           (p0 * (4 * pow(t0, 3.0) * pow(t1, 2.0) + 2 * t2 * pow(t0, 3.0) * t1 - 4 * pow(t0, 2.0) * pow(t1, 3.0) -
                  6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) + t0 * pow(t1, 4.0) + 4 * t2 * t0 * pow(t1, 3.0) -
                  t2 * pow(t1, 4.0))) /
               (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) - 6 * pow(t0, 3.0) * pow(t1, 2.0) -
                4 * t2 * pow(t0, 3.0) * t1 + 4 * pow(t0, 2.0) * pow(t1, 3.0) + 6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) -
                t0 * pow(t1, 4.0) - 4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0)) +
           (a0 * (2 * pow(t0, 3.0) * pow(t1, 2.0) + t2 * pow(t0, 3.0) * t1 - 3 * t2 * pow(t0, 2.0) * pow(t1, 2.0))) /
               (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) + t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 -
                     t2 * pow(t1, 2.0))) -
           (p1 * (pow(t0, 5.0) - 4 * pow(t0, 4.0) * t1 - t2 * pow(t0, 4.0) + 2 * pow(t0, 3.0) * pow(t1, 2.0) +
                  2 * t2 * pow(t0, 3.0) * t1)) /
               (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) - 6 * pow(t0, 3.0) * pow(t1, 2.0) -
                4 * t2 * pow(t0, 3.0) * t1 + 4 * pow(t0, 2.0) * pow(t1, 3.0) + 6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) -
                t0 * pow(t1, 4.0) - 4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0)) +
           (pow(t0, 3.0) * t1 * v2) / (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) + t0 * pow(t1, 2.0) +
                                       2 * t2 * t0 * t1 - t2 * pow(t1, 2.0)) -
           (2 * p1 * pow(t0, 3.0) * t1) /
               (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 - 2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
                pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
                pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) +
           (2 * p2 * pow(t0, 3.0) * t1) /
               (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 - 2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
                pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
                pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0));
    P[1] =
        (2 * p1 * (pow(t0, 3.0) + 3 * t1 * pow(t0, 2.0))) /
            (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 - 2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
             pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
             pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) -
        (2 * p2 * (pow(t0, 3.0) + 3 * t1 * pow(t0, 2.0))) /
            (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 - 2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
             pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
             pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) -
        (v0 * (-5 * pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) + 3 * t2 * pow(t0, 2.0) * t1 + t0 * pow(t1, 3.0) +
               3 * t2 * t0 * pow(t1, 2.0) - t2 * pow(t1, 3.0))) /
            (pow(t0, 4.0) - 3 * pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) + 3 * pow(t0, 2.0) * pow(t1, 2.0) +
             3 * t2 * pow(t0, 2.0) * t1 - t0 * pow(t1, 3.0) - 3 * t2 * t0 * pow(t1, 2.0) + t2 * pow(t1, 3.0)) -
        (a2 * (pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) + 3 * pow(t0, 2.0) * pow(t1, 2.0) - 3 * t2 * pow(t0, 2.0) * t1)) /
            (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) + t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 -
                  t2 * pow(t1, 2.0))) -
        (a0 * (5 * pow(t0, 3.0) * t1 + t2 * pow(t0, 3.0) + 3 * pow(t0, 2.0) * pow(t1, 2.0) -
               3 * t2 * pow(t0, 2.0) * t1 - 6 * t2 * t0 * pow(t1, 2.0))) /
            (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) + t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 -
                  t2 * pow(t1, 2.0))) -
        (v2 * (pow(t0, 3.0) + 3 * t1 * pow(t0, 2.0))) / (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
                                                         t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0)) -
        (2 * p0 *
         (-5 * pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) + 3 * pow(t0, 2.0) * pow(t1, 2.0) + 3 * t2 * pow(t0, 2.0) * t1)) /
            (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) - 6 * pow(t0, 3.0) * pow(t1, 2.0) -
             4 * t2 * pow(t0, 3.0) * t1 + 4 * pow(t0, 2.0) * pow(t1, 3.0) + 6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) -
             t0 * pow(t1, 4.0) - 4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0)) +
        (2 * p1 *
         (-5 * pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) + 3 * pow(t0, 2.0) * pow(t1, 2.0) + 3 * t2 * pow(t0, 2.0) * t1)) /
            (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) - 6 * pow(t0, 3.0) * pow(t1, 2.0) -
             4 * t2 * pow(t0, 3.0) * t1 + 4 * pow(t0, 2.0) * pow(t1, 3.0) + 6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) -
             t0 * pow(t1, 4.0) - 4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0));
    P[2] = (6 * p2 * (pow(t0, 2.0) + t1 * t0)) /
               (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 - 2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
                pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
                pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) -
           (6 * p1 * (pow(t0, 2.0) + t1 * t0)) /
               (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 - 2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
                pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
                pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) +
           (3 * v2 * (pow(t0, 2.0) + t1 * t0)) / (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
                                                  t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0)) +
           (6 * p0 * (-pow(t0, 3.0) - pow(t0, 2.0) * t1 + t0 * pow(t1, 2.0) + t2 * t0 * t1)) /
               (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) - 6 * pow(t0, 3.0) * pow(t1, 2.0) -
                4 * t2 * pow(t0, 3.0) * t1 + 4 * pow(t0, 2.0) * pow(t1, 3.0) + 6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) -
                t0 * pow(t1, 4.0) - 4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0)) -
           (6 * p1 * (-pow(t0, 3.0) - pow(t0, 2.0) * t1 + t0 * pow(t1, 2.0) + t2 * t0 * t1)) /
               (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) - 6 * pow(t0, 3.0) * pow(t1, 2.0) -
                4 * t2 * pow(t0, 3.0) * t1 + 4 * pow(t0, 2.0) * pow(t1, 3.0) + 6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) -
                t0 * pow(t1, 4.0) - 4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0)) +
           (a2 * (pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) + t0 * pow(t1, 2.0) - t2 * t0 * t1)) /
               (2 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) + t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 -
                     t2 * pow(t1, 2.0))) +
           (3 * v0 * (-pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 + t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1)) /
               (pow(t0, 4.0) - 3 * pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) + 3 * pow(t0, 2.0) * pow(t1, 2.0) +
                3 * t2 * pow(t0, 2.0) * t1 - t0 * pow(t1, 3.0) - 3 * t2 * t0 * pow(t1, 2.0) + t2 * pow(t1, 3.0)) +
           (a0 * (pow(t0, 3.0) + 3 * pow(t0, 2.0) * t1 - 3 * t2 * t0 * t1 - t2 * pow(t1, 2.0))) /
               (2 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) + t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 -
                     t2 * pow(t1, 2.0)));
    P[3] = (2 * p1 * (3 * t0 + t1)) /
               (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 - 2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
                pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
                pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) -
           (v0 * (-5 * pow(t0, 2.0) + 2 * t2 * t0 + pow(t1, 2.0) + 2 * t2 * t1)) /
               (pow(t0, 4.0) - 3 * pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) + 3 * pow(t0, 2.0) * pow(t1, 2.0) +
                3 * t2 * pow(t0, 2.0) * t1 - t0 * pow(t1, 3.0) - 3 * t2 * t0 * pow(t1, 2.0) + t2 * pow(t1, 3.0)) -
           (2 * p2 * (3 * t0 + t1)) /
               (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 - 2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
                pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
                pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) -
           (2 * p0 * (-4 * pow(t0, 2.0) + t0 * t1 + t2 * t0 + pow(t1, 2.0) + t2 * t1)) /
               (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) - 6 * pow(t0, 3.0) * pow(t1, 2.0) -
                4 * t2 * pow(t0, 3.0) * t1 + 4 * pow(t0, 2.0) * pow(t1, 3.0) + 6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) -
                t0 * pow(t1, 4.0) - 4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0)) +
           (2 * p1 * (-4 * pow(t0, 2.0) + t0 * t1 + t2 * t0 + pow(t1, 2.0) + t2 * t1)) /
               (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) - 6 * pow(t0, 3.0) * pow(t1, 2.0) -
                4 * t2 * pow(t0, 3.0) * t1 + 4 * pow(t0, 2.0) * pow(t1, 3.0) + 6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) -
                t0 * pow(t1, 4.0) - 4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0)) -
           (v2 * (3 * t0 + t1)) / (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) + t0 * pow(t1, 2.0) +
                                   2 * t2 * t0 * t1 - t2 * pow(t1, 2.0)) -
           (a2 * (3 * t0 * t1 - 3 * t0 * t2 - t1 * t2 + pow(t1, 2.0))) /
               (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) + t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 -
                     t2 * pow(t1, 2.0))) +
           (a0 * (-6 * pow(t0, 2.0) - 3 * t0 * t1 + 3 * t2 * t0 + pow(t1, 2.0) + 5 * t2 * t1)) /
               (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) + t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 -
                     t2 * pow(t1, 2.0)));
    P[4] =
        (2 * p2) / (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 - 2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
                    pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 -
                    2 * t0 * t1 * pow(t2, 2.0) - pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) -
        (2 * p1) / (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 - 2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
                    pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 -
                    2 * t0 * t1 * pow(t2, 2.0) - pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) +
        v2 / (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) + t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 -
              t2 * pow(t1, 2.0)) -
        (a0 * (t1 - 3 * t0 + 2 * t2)) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
                                              t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0))) +
        (a2 * (t1 - t2)) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) + t0 * pow(t1, 2.0) +
                                 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0))) +
        (v0 * (t1 - 2 * t0 + t2)) /
            (pow(t0, 4.0) - 3 * pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) + 3 * pow(t0, 2.0) * pow(t1, 2.0) +
             3 * t2 * pow(t0, 2.0) * t1 - t0 * pow(t1, 3.0) - 3 * t2 * t0 * pow(t1, 2.0) + t2 * pow(t1, 3.0)) +
        (p0 * (2 * t1 - 3 * t0 + t2)) /
            (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) - 6 * pow(t0, 3.0) * pow(t1, 2.0) -
             4 * t2 * pow(t0, 3.0) * t1 + 4 * pow(t0, 2.0) * pow(t1, 3.0) + 6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) -
             t0 * pow(t1, 4.0) - 4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0)) -
        (p1 * (2 * t1 - 3 * t0 + t2)) /
            (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) - 6 * pow(t0, 3.0) * pow(t1, 2.0) -
             4 * t2 * pow(t0, 3.0) * t1 + 4 * pow(t0, 2.0) * pow(t1, 3.0) + 6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) -
             t0 * pow(t1, 4.0) - 4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0));
    P[5] =
        (p2 * (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) - 4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
               4 * pow(t1, 2.0) * pow(t2, 3.0) - 6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) + 2 * t0 * t1 * pow(t2, 3.0))) /
            (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) - 4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
             6 * pow(t1, 2.0) * pow(t2, 3.0) - 6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
             4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0)) -
        (a2 * (2 * pow(t1, 2.0) * pow(t2, 3.0) - 3 * t0 * pow(t1, 2.0) * pow(t2, 2.0) + t0 * t1 * pow(t2, 3.0))) /
            (6 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) - 2 * t0 * t1 * t2 - pow(t2, 3.0) +
                  t0 * pow(t2, 2.0))) -
        (v2 * (-pow(t1, 3.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) * t2 + 2 * pow(t1, 2.0) * pow(t2, 3.0) -
               3 * t0 * pow(t1, 2.0) * pow(t2, 2.0) + t0 * t1 * pow(t2, 3.0))) /
            (-pow(t1, 3.0) * t2 + t0 * pow(t1, 3.0) + 3 * pow(t1, 2.0) * pow(t2, 2.0) - 3 * t0 * pow(t1, 2.0) * t2 -
             3 * t1 * pow(t2, 3.0) + 3 * t0 * t1 * pow(t2, 2.0) + pow(t2, 4.0) - t0 * pow(t2, 3.0)) +
        (p1 * (2 * pow(t1, 2.0) * pow(t2, 3.0) - 4 * t1 * pow(t2, 4.0) + 2 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) -
               t0 * pow(t2, 4.0))) /
            (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) - 4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
             6 * pow(t1, 2.0) * pow(t2, 3.0) - 6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
             4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0)) -
        (a0 * (pow(t1, 2.0) * pow(t2, 3.0) - t0 * t1 * pow(t2, 3.0))) /
            (6 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) - 2 * t0 * t1 * t2 - pow(t2, 3.0) +
                  t0 * pow(t2, 2.0))) -
        (t1 * pow(t2, 3.0) * v0) / (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) - 2 * t0 * t1 * t2 -
                                    pow(t2, 3.0) + t0 * pow(t2, 2.0)) +
        (2 * p0 * t1 * pow(t2, 3.0)) /
            (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 + pow(t0, 2.0) * pow(t2, 2.0) -
             t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
             pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) + t1 * pow(t2, 3.0)) -
        (2 * p1 * t1 * pow(t2, 3.0)) /
            (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 + pow(t0, 2.0) * pow(t2, 2.0) -
             t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
             pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) + t1 * pow(t2, 3.0));
    P[6] =
        (2 * p1 * (pow(t2, 3.0) + 3 * t1 * pow(t2, 2.0))) /
            (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 + pow(t0, 2.0) * pow(t2, 2.0) -
             t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
             pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) + t1 * pow(t2, 3.0)) -
        (2 * p0 * (pow(t2, 3.0) + 3 * t1 * pow(t2, 2.0))) /
            (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 + pow(t0, 2.0) * pow(t2, 2.0) -
             t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
             pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) + t1 * pow(t2, 3.0)) +
        (v2 * (-pow(t1, 3.0) * t2 + t0 * pow(t1, 3.0) - 3 * t0 * pow(t1, 2.0) * t2 + 5 * t1 * pow(t2, 3.0) -
               3 * t0 * t1 * pow(t2, 2.0) + t0 * pow(t2, 3.0))) /
            (-pow(t1, 3.0) * t2 + t0 * pow(t1, 3.0) + 3 * pow(t1, 2.0) * pow(t2, 2.0) - 3 * t0 * pow(t1, 2.0) * t2 -
             3 * t1 * pow(t2, 3.0) + 3 * t0 * t1 * pow(t2, 2.0) + pow(t2, 4.0) - t0 * pow(t2, 3.0)) -
        (2 * p1 *
         (3 * pow(t1, 2.0) * pow(t2, 2.0) - 5 * t1 * pow(t2, 3.0) + 3 * t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0))) /
            (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) - 4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
             6 * pow(t1, 2.0) * pow(t2, 3.0) - 6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
             4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0)) +
        (2 * p2 *
         (3 * pow(t1, 2.0) * pow(t2, 2.0) - 5 * t1 * pow(t2, 3.0) + 3 * t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0))) /
            (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) - 4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
             6 * pow(t1, 2.0) * pow(t2, 3.0) - 6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
             4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0)) +
        (a0 * (3 * pow(t1, 2.0) * pow(t2, 2.0) + t1 * pow(t2, 3.0) - 3 * t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0))) /
            (6 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) - 2 * t0 * t1 * t2 - pow(t2, 3.0) +
                  t0 * pow(t2, 2.0))) +
        (a2 * (3 * pow(t1, 2.0) * pow(t2, 2.0) - 6 * t0 * pow(t1, 2.0) * t2 + 5 * t1 * pow(t2, 3.0) -
               3 * t0 * t1 * pow(t2, 2.0) + t0 * pow(t2, 3.0))) /
            (6 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) - 2 * t0 * t1 * t2 - pow(t2, 3.0) +
                  t0 * pow(t2, 2.0))) +
        (v0 * (pow(t2, 3.0) + 3 * t1 * pow(t2, 2.0))) /
            (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) - 2 * t0 * t1 * t2 - pow(t2, 3.0) +
             t0 * pow(t2, 2.0));
    P[7] = (6 * p0 * (pow(t2, 2.0) + t1 * t2)) /
               (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 + pow(t0, 2.0) * pow(t2, 2.0) -
                t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
                pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) + t1 * pow(t2, 3.0)) -
           (6 * p1 * (pow(t2, 2.0) + t1 * t2)) /
               (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 + pow(t0, 2.0) * pow(t2, 2.0) -
                t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
                pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) + t1 * pow(t2, 3.0)) +
           (a0 * (-pow(t1, 2.0) * t2 - t1 * pow(t2, 2.0) + t0 * t1 * t2 + t0 * pow(t2, 2.0))) /
               (2 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) - 2 * t0 * t1 * t2 - pow(t2, 3.0) +
                     t0 * pow(t2, 2.0))) -
           (3 * v2 * (-pow(t1, 2.0) * t2 + 2 * t1 * pow(t2, 2.0) - 2 * t0 * t1 * t2 + pow(t2, 3.0))) /
               (-pow(t1, 3.0) * t2 + t0 * pow(t1, 3.0) + 3 * pow(t1, 2.0) * pow(t2, 2.0) - 3 * t0 * pow(t1, 2.0) * t2 -
                3 * t1 * pow(t2, 3.0) + 3 * t0 * t1 * pow(t2, 2.0) + pow(t2, 4.0) - t0 * pow(t2, 3.0)) -
           (3 * v0 * (pow(t2, 2.0) + t1 * t2)) / (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) -
                                                  2 * t0 * t1 * t2 - pow(t2, 3.0) + t0 * pow(t2, 2.0)) +
           (a2 * (t0 * pow(t1, 2.0) - 3 * t1 * pow(t2, 2.0) + 3 * t0 * t1 * t2 - pow(t2, 3.0))) /
               (2 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) - 2 * t0 * t1 * t2 - pow(t2, 3.0) +
                     t0 * pow(t2, 2.0))) -
           (6 * p1 * (-pow(t1, 2.0) * t2 + t1 * pow(t2, 2.0) - t0 * t1 * t2 + pow(t2, 3.0))) /
               (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) - 4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
                6 * pow(t1, 2.0) * pow(t2, 3.0) - 6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
                4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0)) +
           (6 * p2 * (-pow(t1, 2.0) * t2 + t1 * pow(t2, 2.0) - t0 * t1 * t2 + pow(t2, 3.0))) /
               (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) - 4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
                6 * pow(t1, 2.0) * pow(t2, 3.0) - 6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
                4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0));
    P[8] = (2 * p1 * (t1 + 3 * t2)) /
               (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 + pow(t0, 2.0) * pow(t2, 2.0) -
                t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
                pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) + t1 * pow(t2, 3.0)) -
           (2 * p0 * (t1 + 3 * t2)) /
               (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 + pow(t0, 2.0) * pow(t2, 2.0) -
                t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
                pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) + t1 * pow(t2, 3.0)) -
           (v2 * (pow(t1, 2.0) + 2 * t0 * t1 - 5 * pow(t2, 2.0) + 2 * t0 * t2)) /
               (-pow(t1, 3.0) * t2 + t0 * pow(t1, 3.0) + 3 * pow(t1, 2.0) * pow(t2, 2.0) - 3 * t0 * pow(t1, 2.0) * t2 -
                3 * t1 * pow(t2, 3.0) + 3 * t0 * t1 * pow(t2, 2.0) + pow(t2, 4.0) - t0 * pow(t2, 3.0)) -
           (a2 * (pow(t1, 2.0) - 3 * t1 * t2 + 5 * t0 * t1 - 6 * pow(t2, 2.0) + 3 * t0 * t2)) /
               (6 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) - 2 * t0 * t1 * t2 - pow(t2, 3.0) +
                     t0 * pow(t2, 2.0))) -
           (a0 * (t0 * t1 + 3 * t0 * t2 - 3 * t1 * t2 - pow(t1, 2.0))) /
               (6 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) - 2 * t0 * t1 * t2 - pow(t2, 3.0) +
                     t0 * pow(t2, 2.0))) +
           (v0 * (t1 + 3 * t2)) / (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) - 2 * t0 * t1 * t2 -
                                   pow(t2, 3.0) + t0 * pow(t2, 2.0)) -
           (2 * p1 * (pow(t1, 2.0) + t1 * t2 + t0 * t1 - 4 * pow(t2, 2.0) + t0 * t2)) /
               (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) - 4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
                6 * pow(t1, 2.0) * pow(t2, 3.0) - 6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
                4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0)) +
           (2 * p2 * (pow(t1, 2.0) + t1 * t2 + t0 * t1 - 4 * pow(t2, 2.0) + t0 * t2)) /
               (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) - 4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
                6 * pow(t1, 2.0) * pow(t2, 3.0) - 6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
                4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0));
    P[9] = (2 * p0) / (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 + pow(t0, 2.0) * pow(t2, 2.0) -
                       t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
                       pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) + t1 * pow(t2, 3.0)) -
           v0 / (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) - 2 * t0 * t1 * t2 - pow(t2, 3.0) +
                 t0 * pow(t2, 2.0)) -
           (2 * p1) / (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 + pow(t0, 2.0) * pow(t2, 2.0) -
                       t0 * pow(t1, 3.0) + t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
                       pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) + t1 * pow(t2, 3.0)) +
           (p1 * (t0 + 2 * t1 - 3 * t2)) /
               (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) - 4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
                6 * pow(t1, 2.0) * pow(t2, 3.0) - 6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
                4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0)) -
           (p2 * (t0 + 2 * t1 - 3 * t2)) /
               (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) - 4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
                6 * pow(t1, 2.0) * pow(t2, 3.0) - 6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
                4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0)) +
           (v2 * (t0 + t1 - 2 * t2)) /
               (-pow(t1, 3.0) * t2 + t0 * pow(t1, 3.0) + 3 * pow(t1, 2.0) * pow(t2, 2.0) - 3 * t0 * pow(t1, 2.0) * t2 -
                3 * t1 * pow(t2, 3.0) + 3 * t0 * t1 * pow(t2, 2.0) + pow(t2, 4.0) - t0 * pow(t2, 3.0)) +
           (a2 * (2 * t0 + t1 - 3 * t2)) / (6 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) -
                                                 2 * t0 * t1 * t2 - pow(t2, 3.0) + t0 * pow(t2, 2.0))) +
           (a0 * (t0 - t1)) / (6 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) - 2 * t0 * t1 * t2 -
                                    pow(t2, 3.0) + t0 * pow(t2, 2.0)));

    if (cont_T <= t1)
    {
        p = P[0] + P[1] * cont_T + P[2] * cont_T * cont_T + P[3] * cont_T * cont_T * cont_T +
            P[4] * cont_T * cont_T * cont_T * cont_T;
        v = P[1] + 2 * P[2] * cont_T + 3 * P[3] * cont_T * cont_T + 4 * P[4] * cont_T * cont_T * cont_T;
        a = 2 * P[2] + 6 * P[3] * cont_T + 12 * P[4] * cont_T * cont_T;
    }

    if ((cont_T > t1) && (cont_T <= t2))
    {
        p = P[5] + P[6] * cont_T + P[7] * cont_T * cont_T + P[8] * cont_T * cont_T * cont_T +
            P[9] * cont_T * cont_T * cont_T * cont_T;
        v = P[6] + 2 * P[7] * cont_T + 3 * P[8] * cont_T * cont_T + 4 * P[9] * cont_T * cont_T * cont_T;
        a = 2 * P[7] + 6 * P[8] * cont_T + 12 * P[9] * cont_T * cont_T;
    }

    if (cont_T > t2)
    {
        p = p2;
        v = v2;
        a = a2;
    }

    return p;
}