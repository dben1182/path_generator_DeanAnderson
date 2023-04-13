#include "gtest/gtest.h"
#include "ObstacleConstraints.hpp"

TEST(ObstacleConstraintTests, AllIntervalsNotColliding3D)
{
    const int D{3};
    ObstacleConstraints<D> obstacle_const{};
    double obstacle_center[] = {7, 9, 12};
    double obstacle_radius = 2; 
    double cont_pts[] = {1, 3,  -1,  2, 4,
                        2, 5, 6.9, -4, 7,
                        3, 4, 8, 2, 8.5};
    int num_control_points = 5;
    double* distances = obstacle_const.getObstacleConstraintsForIntervals(cont_pts, num_control_points,
        obstacle_radius, obstacle_center);
    double true_distance_1 = 7.1033445550584702;
    double true_distance_2 = 7.1877666609532103;
    double tolerance = 0.0000000001;
    EXPECT_NEAR(true_distance_1, distances[0], tolerance);
    EXPECT_NEAR(true_distance_2, distances[1], tolerance);
}

TEST(ObstacleConstraintTests, WholeSplineNotColliding3D)
{
    const int D{3};
    ObstacleConstraints<D> obstacle_const{};
    double obstacle_center[] = {7, 9, 12};
    double obstacle_radius = 2; 
    double cont_pts[] = {1, 3,  -1,  2, 4,
                        2, 5, 6.9, -4, 7,
                        3, 4, 8, 2, 8.5};
    int num_control_points = 5;
    double distance = obstacle_const.getObstacleConstraintForSpline(cont_pts, num_control_points,
        obstacle_radius, obstacle_center);
    double true_distance = 7.1033445550584702;
    double tolerance = 0.0000000001;
    EXPECT_NEAR(true_distance, distance, tolerance);
}