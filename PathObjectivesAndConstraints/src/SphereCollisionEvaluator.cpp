#include "SphereCollisionEvaluator.hpp"
#include <math.h>  
#include <iostream>

template <int D>
SphereCollisionEvaluator<D>::SphereCollisionEvaluator()
{

}

template <int D>
double SphereCollisionEvaluator<D>::getSplineDistanceToSphere(double obstacle_center[], 
            double &obstacle_radius, double cont_pts[], int &num_cont_points)
{
    int order = 3;
    int num_intervals = num_cont_points - order;
    double min_distance = std::numeric_limits<double>::max();
    Eigen::Matrix<double,D,1> obstacle_center_ = getObstacleCenterFromArray(obstacle_center);
    for (unsigned int i = 0; i < num_intervals; i++)
    {
        Eigen::Matrix<double,D,4> interval_control_points = cbind_help.array_section_to_eigen(cont_pts, num_cont_points, i);
        Eigen::Matrix<double,D,4> minvo_control_points = cp_converter.convert_3rd_order_spline(interval_control_points);
        double distance = getPointsDistanceToSphere(obstacle_center_, obstacle_radius, minvo_control_points);
        if (min_distance > distance)
        {
            min_distance = distance;
        }
    }
    return min_distance;
}

template <int D>
double*  SphereCollisionEvaluator<D>::getAllSplineIntervalDistancesToSphere(double obstacle_center[], 
    double &obstacle_radius, double cont_pts[], int &num_cont_points)
{
    int order = 3;
    int num_intervals = num_cont_points - order;
    double* distances_to_obstacle = new double[num_intervals];
    Eigen::Matrix<double,D,1> obstacle_center_ = getObstacleCenterFromArray(obstacle_center);
    for (unsigned int i = 0; i < num_intervals; i++)
    {
        Eigen::Matrix<double,D,4> interval_control_points = cbind_help.array_section_to_eigen(cont_pts, num_cont_points, i);
        Eigen::Matrix<double,D,4> minvo_control_points = cp_converter.convert_3rd_order_spline(interval_control_points);
        distances_to_obstacle[i] = getPointsDistanceToSphere(obstacle_center_, obstacle_radius, minvo_control_points);
    }
    return distances_to_obstacle;
}

template <int D>
double SphereCollisionEvaluator<D>::getPointsDistanceToSphere(Eigen::Matrix<double,D,1> &obstacle_center, 
    double &obstacle_radius, Eigen::Matrix<double,D,4> &cont_pts)
{
    Eigen::Matrix<double,D,D> rotation = getRotationForSphereToIntervalVector(obstacle_center, cont_pts);
    Eigen::Matrix<double,D,4> rotated_points = rotation*cont_pts;
    double points_min_x = rotated_points.block(0,0,1,4).minCoeff();
    Eigen::Matrix<double,D,1> rotated_sphere_center = rotation*obstacle_center;
    double sphere_max_x = rotated_sphere_center.coeff(0) + obstacle_radius;
    double distance = points_min_x - sphere_max_x;
    return distance;
}

template <int D>
Eigen::Matrix<double,D,D> SphereCollisionEvaluator<D>::getRotationForSphereToIntervalVector(Eigen::Matrix<double,D,1> &obstacle_center,
    Eigen::Matrix<double,D,4> &cont_pts)
{
    Eigen::Matrix<double, D, 1> mean_pts = cont_pts.rowwise().mean();
    Eigen::Matrix<double, D, 1> vector = mean_pts - obstacle_center;
    Eigen::Matrix<double, D, D> rotation;
    if (D == 2)
    {
        if (vector(0) == 0 && vector(1) == 0)
        {
            rotation << 1 , 0, 
                        0 , 1;
        }
        else
        {
            rotation = get2DVectorToXDirRotation(vector);
        }
    }
    else //(D == 3)
    {
        if (vector(0) == 0 && vector(1) == 0 && vector(2) == 0)
        {
            rotation << 1 , 0 , 0, 
                        0, 1, 0, 
                        0, 0, 1;
        }
        else
        {
            rotation = get3DVectorToXDirRotation(vector);
        }
    }
    return rotation;
}

template <int D>
Eigen::Matrix<double,D,D> SphereCollisionEvaluator<D>::get2DVectorToXDirRotation(Eigen::Matrix<double, D, 1> vector)
{
    double dx = vector.coeff(0);
    double dy = vector.coeff(1);
    double psi = -atan2(dy,dx);
    double c_psi = cos(psi);
    double s_psi = sin(psi);
    Eigen::Matrix<double,D,D> rotation;
    rotation << c_psi, -s_psi, 
                s_psi,  c_psi;
    return rotation;
}

template <int D>
Eigen::Matrix<double,D,D> SphereCollisionEvaluator<D>::get3DVectorToXDirRotation(Eigen::Matrix<double, D, 1> vector)
{
    double dx_1 = vector.coeff(0);
    double dz_1 = vector.coeff(2);
    double theta = atan2(dz_1,dx_1);
    double c_theta = cos(theta);
    double s_theta = sin(theta);
    Eigen::Matrix<double,D,D> Ry;
    Ry << c_theta, 0, s_theta,
                0, 1,       0,
         -s_theta, 0, c_theta;
    Eigen::Matrix<double,D,1> vector_2 = Ry * vector;
    double dx_2 = vector_2.coeff(0);
    double dy_2 = vector_2.coeff(1);
    double psi = atan2(dy_2,dx_2);
    double c_psi = cos(psi);
    double s_psi = sin(psi);
    Eigen::Matrix<double,D,D> Rz;
    Rz << c_psi, -s_psi, 0,
          s_psi,  c_psi, 0,
              0,      0, 1;
    Eigen::Matrix<double,D,D> rotation = Rz.transpose() * Ry;
    return rotation;
}

template <int D>
Eigen::Matrix<double,D,1> SphereCollisionEvaluator<D>::getObstacleCenterFromArray(double obstacle_center[])
{
    Eigen::Matrix<double,D,1> obstacle_center_;
    if (D == 2)
    {
        obstacle_center_ << obstacle_center[0], obstacle_center[1];
    }
    else //D == 3
    {
        obstacle_center_ << obstacle_center[0], obstacle_center[1], obstacle_center[2];
    }
    return obstacle_center_;
}

// explicit instantiation
template class SphereCollisionEvaluator<2>;
template class SphereCollisionEvaluator<3>;