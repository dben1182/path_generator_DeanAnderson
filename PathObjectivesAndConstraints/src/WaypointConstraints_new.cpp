#include "WaypointConstraints_new.hpp"
#include <iostream>

template<int D>
WaypointConstraints<D>::WaypointConstraints()
{

}

template<int D>
double* WaypointConstraints<D>::velocity_constriaint(double cont_pts[], int num_control_points, 
    double scale_factor, double desired_velocity[], bool isStartPoint)
{
    int order = 3;
    Eigen::Matrix<double,D,1> velocity_vector;
    double* constraints = new double[D];
    if (isStartPoint)
    {
        Eigen::Matrix<double,D,1> velocity_vector = get_initial_velocity(cont_pts, num_control_points, scale_factor);
    }
    else // is endpoint
    {
        Eigen::Matrix<double,D,1> velocity_vector = get_final_velocity(cont_pts, num_control_points, scale_factor);
    }
    for(unsigned int i = 0; i < D; i++)
    {
            constraints[i] = velocity_vector.coeff(i) - desired_velocity[i];
    }
    return constraints;
}

template<int D>
double* WaypointConstraints<D>::direction_constraint(double cont_pts[], int num_control_points, 
    double desired_angles[], bool isStartPoint)
{
    int order = 3;
    double* constraints = new double[D-1];
    Eigen::Matrix<double,D,1> velocity_vector;
    double scale_factor = 1;
    if (isStartPoint)
    {
        velocity_vector = get_initial_velocity(cont_pts, num_control_points, scale_factor);
    }
    else // is end point
    {
        velocity_vector = get_final_velocity(cont_pts, num_control_points, scale_factor);
    }

    if (D == 2):
    {
        double psi = rot_help.get2DXDirToVectorAngles(velocity_vector);
        constraints[0] = psi - desired_angles[0]
    }
    else (D == 3):
    {
        Eigen::Matrix<double,2,1> angles = rot_help.get3DXDirToVectorAngles(velocity_vector);
        constraints[0] = angles.coeff(0) - desired_angles[0]
        constraints[1] = angles.coeff(1) - desired_angles[1]
    }
    return constraints;
}

template<int D>
double* WaypointConstraints<D>::continuity_constraint(double cont_pts[], int num_control_points, 
    double desired_unscaled_vel[], double desired_unscaled_accel[], bool isStartPoint)
{
    int order = 3;
    double* constraints = new double[D*2];
    Eigen::Matrix<double,D,1> unscaled_vel_vector;
    Eigen::Matrix<double,D,1> unscaled_accel_vector;
    if (isStartPoint)
    {
        unscaled_vel_vector = get_initial_velocity(cont_pts, num_control_points, scale_factor);
        unscaled_accel_vector = get_initial_acceleration(cont_pts, num_control_points, scale_factor);
    }
    else // is end point
    {
        unscaled_vel_vector = get_final_velocity(cont_pts, num_control_points, scale_factor);
        unscaled_accel_vector = get_final_acceleration(cont_pts, num_control_points, scale_factor);
    }
    for(unsigned int i = 0; i < D; i++)
    {
            constraints[i] = unscaled_vel_vector.coeff(i) - desired_unscaled_vel[i];
            constraints[D+i] = unscaled_accel_vector.coeff(i) - desired_unscaled_accel[i];
    }
    return constraints;
}

template<int D>
Eigen::Matrix<double,D,1> WaypointConstraints<D>::get_initial_velocity(double cont_pts[], int &num_control_points, double &scale_factor)
{
    Eigen::Matrix<double,D,4> initial_control_points = cbind_help.array_section_to_eigen(cont_pts, num_cont_pts, start_index);
    Eigen::Matrix<double,D,1> p1 = initial_control_points.block(0,0,D,1);
    Eigen::Matrix<double,D,1> p3 = initial_control_points.block(0,2,D,1);
    Eigen::Matrix<double,D,1> velocity_vector_0 = (p3 - p1)/(2*scale_factor);
}

template<int D>
Eigen::Matrix<double,D,1> WaypointConstraints<D>::get_final_velocity(double cont_pts[], int &num_control_points, double &scale_factor)
{
    Eigen::Matrix<double,D,4> final_control_points = cbind_help.array_section_to_eigen(cont_pts, num_cont_pts, end_index);
    Eigen::Matrix<double,D,1> p2 = final_control_points.block(0,1,D,1);
    Eigen::Matrix<double,D,1> p4 = final_control_points.block(0,3,D,1);
    Eigen::Matrix<double,D,1> velocity_vector_f = (p4 - p2)/(2*scale_factor);
}

template<int D>
Eigen::Matrix<double,D,1> WaypointConstraints<D>::get_initial_acceleration(double cont_pts[], int &num_control_points, double &scale_factor)
{
    Eigen::Matrix<double,D,4> initial_control_points = cbind_help.array_section_to_eigen(cont_pts, num_cont_pts, start_index);
    Eigen::Matrix<double,D,1> p1 = initial_control_points.block(0,0,D,1);
    Eigen::Matrix<double,D,1> p2 = initial_control_points.block(0,1,D,1);
    Eigen::Matrix<double,D,1> p3 = initial_control_points.block(0,2,D,1);
    Eigen::Matrix<double,D,1> acceleration_vector_0 = (p1 - 2*p2 + p3)/(scale_factor * scale_factor);
}

template<int D>
Eigen::Matrix<double,D,1> WaypointConstraints<D>::get_final_acceleration(double cont_pts[], int &num_control_points, double &scale_factor)
{
    Eigen::Matrix<double,D,4> final_control_points = cbind_help.array_section_to_eigen(cont_pts, num_cont_pts, end_index);
    Eigen::Matrix<double,D,1> p2 = final_control_points.block(0,1,D,1);
    Eigen::Matrix<double,D,1> p3 = final_control_points.block(0,2,D,1);
    Eigen::Matrix<double,D,1> p4 = final_control_points.block(0,3,D,1);
    Eigen::Matrix<double,D,1> acceleration_vector_f = (p2 - 2*p3 + p4)/(scale_factor * scale_factor);
}


//explicit instantiation
template class WaypointConstraints<2>;
template class WaypointConstraints<3>;
