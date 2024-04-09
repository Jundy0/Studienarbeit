#include <iostream>
#include <vector>
#include <cmath>


#include "../icp/Eigen/Dense"
#include "../icp/icp.h"
#include "./load_vectors.cpp"

// Function to convert polar coordinates to Cartesian coordinates using Eigen library
Eigen::MatrixXd polar_to_cartesian(const std::vector<double>& distances, const std::vector<double>& angles) {
    Eigen::MatrixXd cartesian_coords(3, distances.size());
    for (int i = 0; i < distances.size(); i++) {
        double x = distances[i] * cos(angles[i] * M_PI / 180);
        double y = distances[i] * sin(angles[i] * M_PI / 180);
        cartesian_coords.col(i) << x, y, 0.0; // Setting x, y, z = 0
    }
    std::cout << cartesian_coords.rows() << "\n" << cartesian_coords.cols() << std::endl;
    return cartesian_coords.transpose();
}

// Function to translate and rotate the matrix using Eigen library

Eigen::MatrixXd translate_rotate_matrix(const Eigen::MatrixXd& cartesian_coords, const std::vector<double>& translation_vector, double rotation_angle) {
    // Convert rotation angle to radians
    double theta = rotation_angle * M_PI / 180.0;
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);

    // Define rotation matrix
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << cos_theta, -sin_theta, 0,
                       sin_theta, cos_theta, 0,
                       0, 0, 1;

    // Create a copy of the input matrix for manipulation
    Eigen::MatrixXd rotated_translated_matrix = cartesian_coords;


    // Apply rotation to each point
    rotated_translated_matrix = rotated_translated_matrix* rotation_matrix;  

    // Apply translation matrix to the points
    for (int i = 0; i < rotated_translated_matrix.rows(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rotated_translated_matrix.row(i)[j] += translation_vector[j];
        }
    } 

    return rotated_translated_matrix;
}

struct TransformationComponents {
    Eigen::Vector3d translation_vector;
    double rotation_angle_deg; // Rotation angle in degrees
};

TransformationComponents extractTransformation(const Eigen::Matrix4d& T) {
    TransformationComponents components;

    // Extract translation vector directly
    components.translation_vector = T.block<3,1>(0,3);

    // Calculate rotation angle from the rotation matrix part of T
    // Assuming the rotation is around the Z-axis
    double theta_rad = std::atan2(T(0,1), T(0, 0)); // atan2(sin, cos)
    components.rotation_angle_deg = theta_rad * (180.0 / M_PI); // Convert to degrees

    return components;
}


int main() {
    // Example usage
    std::vector<double> distances, angles; // Example distances


    readPolarCoordinates(distances, angles);
    // Convert polar coordinates to Cartesian coordinates
    Eigen::MatrixXd cartesian_coords = polar_to_cartesian(distances, angles);
    // Perform translation and rotation
    std::vector<double> translation_vector = {8.0, 1.0, 0.0}; // Example translation
    double rotation_angle = 20.0; // Example rotation angle
    Eigen::MatrixXd rotated_translated_matrix = translate_rotate_matrix(cartesian_coords, translation_vector, rotation_angle);
    // Output the rotated and translated matrix
    std::cout << rotated_translated_matrix.rows() << "\n" << rotated_translated_matrix.cols() << std::endl;
    std::cout << cartesian_coords.rows() << "\n" << cartesian_coords.cols() << std::endl;
    ICP_OUT result = icp(cartesian_coords, rotated_translated_matrix, 50, 0.000000001);

    std::cout << "Distances" << std::endl;
    for (int i = 0; i < result.distances.size(); i++) {
            std::cout << result.distances[i] << " ";
        }

    std::cout << std::endl;
    std::cout << "Transformation Matrix" << std::endl;
    std::cout << result.trans << std::endl;

    std::cout<< "Iter:"<<  std::endl;
    std::cout << result.iter << std::endl;

    std::cout << "extracted:" << std::endl;
    TransformationComponents trans_comps = extractTransformation(result.trans);
    std::cout << trans_comps.translation_vector << std::endl;
    std::cout<< trans_comps.rotation_angle_deg << std::endl;
    return 0;
}
