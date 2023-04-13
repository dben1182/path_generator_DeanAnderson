#ifndef OBSTACLECONSTRAINTS_HPP
#define OBSTACLECONSTRAINTS_HPP
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "CBindingHelper.hpp"
#include "SphereCollisionEvaluator.hpp"

template <int D>
class ObstacleConstraints
{
    public:
        ObstacleConstraints();
        double* getObstacleConstraintsForIntervals(double cont_pts[], int num_control_points,
                                double obstacle_radius, double obstacle_center[]);
        double getObstacleConstraintForSpline(double cont_pts[], int num_control_points,
                                double obstacle_radius, double obstacle_center[]);
    private:
        CBindingHelper<D> helper{};
        SphereCollisionEvaluator<D> colision_eval{};
};

extern "C"
{
    ObstacleConstraints<2>* ObstacleConstraints_2(){return new ObstacleConstraints<2>();}
    double* getObstacleConstraintsForIntervals_2(ObstacleConstraints<2>* obj, double cont_pts[], int num_control_points,
        double obstacle_radius, double obstacle_center[]){return obj->getObstacleConstraintsForIntervals(
        cont_pts, num_control_points, obstacle_radius, obstacle_center);}
    double getObstacleConstraintForSpline_2(ObstacleConstraints<2>* obj, double cont_pts[], int num_control_points,
        double obstacle_radius, double obstacle_center[]){return obj->getObstacleConstraintForSpline(
        cont_pts, num_control_points, obstacle_radius, obstacle_center);}

    ObstacleConstraints<3>* ObstacleConstraints_3(){return new ObstacleConstraints<3>();}
    double* getObstacleConstraintsForIntervals_3(ObstacleConstraints<3>* obj, double cont_pts[], int num_control_points,
        double obstacle_radius, double obstacle_center[]){return obj->getObstacleConstraintsForIntervals(
        cont_pts, num_control_points, obstacle_radius, obstacle_center);}
    double getObstacleConstraintForSpline_3(ObstacleConstraints<3>* obj, double cont_pts[], int num_control_points,
        double obstacle_radius, double obstacle_center[]){return obj->getObstacleConstraintForSpline(
        cont_pts, num_control_points, obstacle_radius, obstacle_center);}
}

#endif