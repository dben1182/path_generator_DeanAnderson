#ifndef WAYPOINTCONSTRAINTS_HPP
#define WAYPOINTCONSTRAINTS_HPP
#include "CBindHelperFunctions.hpp"
#include "RotationHelperFunctions.hpp"
template<int D>
class WaypointConstraints
{
    public:
        WaypointConstraints();
        double* velocity_constriaint(double cont_pts[], int num_control_points, 
            double scale_factor, double desired_velocity[], bool isStartPoint);
        double* direction_constraint(double cont_pts[], int num_control_points, 
            double desired_angles[], bool isStartPoint);
        double* continuity_constraint(double cont_pts[], int num_control_points, 
            double desired_unscaled_vel[], double desired_unscaled_accel[], bool isStartPoint);

    private:
        CBindHelperFunctions<D> cbind_help{};
        RotationHelperFunctions<D> rot_help{};
        Eigen::Matrix<double,D,1> get_initial_velocity(double cont_pts[], int &num_control_points, double &scale_factor);
        Eigen::Matrix<double,D,1> get_final_velocity(double cont_pts[], int &num_control_points, double &scale_factor);
        Eigen::Matrix<double,D,1> get_initial_acceleration(double cont_pts[], int &num_control_points, double &scale_factor);
        Eigen::Matrix<double,D,1> get_final_acceleration(double cont_pts[], int &num_control_points, double &scale_factor);
};

#endif