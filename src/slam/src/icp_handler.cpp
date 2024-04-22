#define _USE_MATH_DEFINES

#include <math.h>

#include "../include/icp_handler.h"


IcpHandler::IcpHandler()
{
}

TransformationComponents IcpHandler::extractTransformation(const Eigen::Matrix4d &transformationMatrix)
{
    TransformationComponents components;

    // Extract translation vector directly
    components.translation_vector = transformationMatrix.block<3, 1>(0, 3);

    // Calculate rotation angle from the rotation matrix part of T
    // Assuming the rotation is around the Z-axis
    double theta_rad = std::atan2(transformationMatrix(0, 1), transformationMatrix(0, 0)); // atan2(sin, cos)
    components.rotation_angle_deg = theta_rad * (180.0 / M_PI);                            // Convert to degrees

    return components;
}

TransformationComponents IcpHandler::call_icp(Eigen::MatrixXd initial_mattrix, Eigen::MatrixXd transformed_matirx)
{

    ICP_OUT result = icp(initial_mattrix, transformed_matirx, 50, 0.000000001);

    std::cout << "Transformation Matrix" << std::endl;
    std::cout << result.trans << std::endl;

    std::cout << "Iter:" << std::endl;
    std::cout << result.iter << std::endl;

    std::cout << "extracted:" << std::endl;
    TransformationComponents trans_comps = extractTransformation(result.trans);
    std::cout << trans_comps.translation_vector << std::endl;
    std::cout << trans_comps.rotation_angle_deg << std::endl;
    return trans_comps;
}