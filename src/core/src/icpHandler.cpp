#define _USE_MATH_DEFINES

#include <math.h>

#include "icpHandler.h"

IcpHandler::IcpHandler()
{
}

TransformationComponents IcpHandler::extractTransformation(const Eigen::Matrix3d &transformationMatrix)
{
    TransformationComponents components;

    // Extract translation vector with only x and y as double
    components.translation_vector << static_cast<int>(transformationMatrix(0, 2)),
        static_cast<int>(transformationMatrix(1, 2));

    // Calculate rotation angle from the rotation matrix part of T
    // Assuming the rotation is around the Z-axis
    double theta_rad = std::atan2(transformationMatrix(1, 0), transformationMatrix(0, 0)); // atan2(sin, cos)
    components.rotation_angle = theta_rad;                                                 // Store in radians

    return components;
}

Eigen::MatrixX2d IcpHandler::polar_to_cartesian_from_matrix(Eigen::MatrixX2d points)
{
    Eigen::Matrix<double, -1, 2, 1> cartesian_coords(points.rows(), 2);
    for (int i = 0; i < points.rows(); i++)
    {
        double x = points.row(i)[1] * cos(points.row(i)[0]);
        double y = points.row(i)[1] * sin(points.row(i)[0]);
        cartesian_coords.row(i) << x, y; // Setting x, y
    }
    return cartesian_coords;
}

TransformationComponents IcpHandler::execute_icp(Eigen::MatrixXd initial_matrix, Eigen::MatrixXd transformed_matrix)
{
    ICP_OUT result = icp(initial_matrix, transformed_matrix, 100, 0);

    // std::cout << "Transformation Matrix" << std::endl;
    // std::cout << result.trans << std::endl;

    // std::cout << "Iter:" << std::endl;
    // std::cout << result.iter << std::endl;

    std::cout << "Extracted:" << std::endl;
    TransformationComponents trans_comps = extractTransformation(result.trans);

    std::cout << "X: " << trans_comps.translation_vector[0] << " "
              << "Y: " << trans_comps.translation_vector[1]
              << std::endl;

    std::cout << "Theta: " << trans_comps.rotation_angle << std::endl;

    // Convert the rotation angle to degrees for display purposes
    double rotation_angle_deg = trans_comps.rotation_angle * (180.0 / M_PI);
    std::cout << "Angle: " << rotation_angle_deg << "deg\n"
              << std::endl;

    return trans_comps;
}

TransformationComponents IcpHandler::call_icp(Eigen::MatrixX2d scan_one, Eigen::MatrixX2d scan_two)
{
    Eigen::MatrixX2d initial_mattrix = get_matrix_from_points(scan_one);
    Eigen::MatrixX2d transformed_matirx = get_matrix_from_points(scan_two);

    return execute_icp(initial_mattrix, transformed_matirx);
}

Eigen::MatrixX2d IcpHandler::get_matrix_from_points(Eigen::MatrixX2d points)
{
    Eigen::MatrixX2d nPoints = get_n_closest_points(points, 30);
    return polar_to_cartesian_from_matrix(nPoints);
}

Eigen::MatrixX2d IcpHandler::get_n_closest_points(Eigen::MatrixX2d points, int n)
{
    std::vector<Eigen::RowVector2d> pointVectors;
    Eigen::MatrixX2d sortedPoints(points.rows(), 2);
    Eigen::MatrixX2d nPoints(n, 2);
    
    int count = 0;

    for (int i = 0; i < points.rows(); i++)
    {
        if (points.row(i)[1] != 0)
        {
            pointVectors.push_back(points.row(i));
        }
    }

    std::sort(pointVectors.begin(), pointVectors.end(),
              [](Eigen::Vector2d const &t1, Eigen::Vector2d const &t2)
              { return t1(1) < t2(1); });

    for (int i = 0; i < points.rows(); i++)
    {
        sortedPoints.row(i) = pointVectors[i];
    }

    nPoints = sortedPoints.block(0, 0, n, 2);

    return nPoints;
}
