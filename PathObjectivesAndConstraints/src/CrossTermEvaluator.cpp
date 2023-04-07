#include "CrossTermEvaluator.hpp"
#include "CubicEquationSolver.hpp"
#include <iostream>
#include <stdexcept>

template <int D>
CrossTermEvaluator<D>::CrossTermEvaluator()
{

}

template <int D>
double CrossTermEvaluator<D>::calculate_cross_term_magnitude(double &t, Eigen::Matrix<double,D,4> &control_points, double &scale_factor)
{
    Eigen::Matrix<double,D,1> velocity_vector = d_dt_eval.calculate_velocity_vector(t, control_points, scale_factor);
    double cross_term_magnitude = calculate_cross_term_magnitude(t, control_points, scale_factor, velocity_vector);
    return cross_term_magnitude;
}

template <int D>
double CrossTermEvaluator<D>::calculate_curvature(double &t, Eigen::Matrix<double,D,4> &control_points, double &scale_factor)
{
    Eigen::Matrix<double,D,1> velocity_vector = d_dt_eval.calculate_velocity_vector(t, control_points, scale_factor);
    // double vel_cubed_mag = pow((velocity_vector.norm()),3);
    double vel_mag = (velocity_vector.norm());
    double cross_term_mag = calculate_cross_term_magnitude(t, control_points, scale_factor, velocity_vector);
    double curvature = 0;
    if (vel_mag < 0.00000001)
    {
        if (check_if_not_ascending_colinear(t, control_points, scale_factor))
        {
            curvature = std::numeric_limits<double>::max();
        }
        else
        {
            curvature = 0;
        }
    }
    else
    {
        curvature = cross_term_mag / pow(vel_mag,3);
    }
    return curvature;
}

template <int D>
bool CrossTermEvaluator<D>::check_if_not_ascending_colinear(double &t, Eigen::Matrix<double,D,4> &control_points, double &scale_factor)
{
    return false;
}

template <int D>
double CrossTermEvaluator<D>::calculate_cross_term_magnitude(double &t, Eigen::Matrix<double,D,4> &control_points, double &scale_factor,
        Eigen::Matrix<double,D,1> &velocity_vector)
{
    Eigen::Matrix<double,D,1> acceleration_vector = d_dt_eval.calculate_acceleration_vector(t, control_points, scale_factor);
    double cross_term_magnitude = 0;
    if (D == 2)
    { 
        cross_term_magnitude = abs(velocity_vector(0)*acceleration_vector(1) - velocity_vector(1)*acceleration_vector(0));
    }
    else
    {
        double x = velocity_vector(1)*acceleration_vector(2) - velocity_vector(2)*acceleration_vector(1);
        double y = velocity_vector(2)*acceleration_vector(0) - velocity_vector(0)*acceleration_vector(2);
        double z = velocity_vector(0)*acceleration_vector(1) - velocity_vector(1)*acceleration_vector(0);
        cross_term_magnitude = sqrt(x*x + y*y + z*z);
    }
    return cross_term_magnitude;
}

    // def get_angular_rate_at_time_t(self, time):
    //     '''
    //     This function evaluates the angular rate at time t
    //     '''
    //     dimension = get_dimension(self._control_points)
    //     velocity = self.get_derivative_at_time_t(time,1)
    //     if dimension > 3:
    //         raise Exception("Curvature cannot be evaluated for higher than 3 dimensions")
    //     if dimension == 1:
    //         velocity_magnitude = np.linalg.norm(np.array([1 , velocity]))
    //     else:
    //         velocity_magnitude = np.linalg.norm(velocity)
    //     centripetal_acceleration = self.get_centripetal_acceleration_at_time_t(time)
    //     if velocity_magnitude < 0.1:
    //         num_intervals = self._num_control_points - self._order
    //         time_step = (self._end_time - self._start_time)/(100*num_intervals)
    //         if time == self._start_time:
    //             point_a = self.get_spline_at_time_t(time)
    //             point_b = self.get_spline_at_time_t(time+time_step)
    //             point_c = self.get_spline_at_time_t(time+2*time_step)
    //         elif time == self._end_time:
    //             point_a = self.get_spline_at_time_t(time-time_step*2)
    //             point_b = self.get_spline_at_time_t(time-time_step)
    //             point_c = self.get_spline_at_time_t(time)
    //         else:
    //             point_a = self.get_spline_at_time_t(time-time_step)
    //             point_b = self.get_spline_at_time_t(time)
    //             point_c = self.get_spline_at_time_t(time+time_step)
    //         result = self.check_if_points_are_ascending_colinear(point_a, point_b, point_c)
    //         if result == "colinear_ascending":
    //             angular_rate = 0
    //         elif result == "colinear_unordered":
    //             angular_rate = sys.maxsize
    //         elif velocity_magnitude < 1e-10:
    //             angular_rate = 0
    //         else:
    //             angular_rate = centripetal_acceleration/velocity_magnitude
    //     else:
    //         angular_rate = centripetal_acceleration/velocity_magnitude
    //     return angular_rate

//Explicit template instantiations
template class CrossTermEvaluator<2>;
template class CrossTermEvaluator<3>;