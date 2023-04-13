#include "ObstacleConstraints.hpp"
#include <iostream>

template <int D>
ObstacleConstraints<D>::ObstacleConstraints()
{

}

template <int D>
double* ObstacleConstraints<D>::getObstacleConstraintsForIntervals(double cont_pts[], int num_control_points,
    double obstacle_radius, double obstacle_center[])
{
    double* distances_to_obstacle = colision_eval.getAllSplineIntervalDistancesToSphere(obstacle_center, 
        obstacle_radius, cont_pts, num_control_points);
    return distances_to_obstacle;
}

template <int D>
double ObstacleConstraints<D>::getObstacleConstraintForSpline(double cont_pts[], 
    int num_control_points, double obstacle_radius, double obstacle_center[])
{
    double distance_to_obstacle = colision_eval.getSplineDistanceToSphere(obstacle_center, 
        obstacle_radius, cont_pts, num_control_points);
    return distance_to_obstacle;
}


// explicit instantiation
template class ObstacleConstraints<2>;
template class ObstacleConstraints<3>;
