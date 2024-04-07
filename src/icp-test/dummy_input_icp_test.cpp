#include <iostream>
#include <vector>
#include <cmath>


#include "../icp/Eigen/Dense"
#include "../icp/icp.h"

// Function to convert polar coordinates to Cartesian coordinates using Eigen library
Eigen::MatrixXd polar_to_cartesian(const std::vector<double>& distances, const std::vector<double>& angles) {
    Eigen::MatrixXd cartesian_coords(3, distances.size());
    for (int i = 0; i < distances.size(); ++i) {
        double x = distances[i] * cos(angles[i] * M_PI / 180);
        double y = distances[i] * sin(angles[i] * M_PI / 180);
        cartesian_coords.col(i) << x, y, 0.0; // Setting x, y, z = 0
    }
    return cartesian_coords;
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

    // Apply translation
    for (int i = 0; i < rotated_translated_matrix.rows(); ++i) {
        rotated_translated_matrix.row(i) += Eigen::Vector3d(translation_vector[0], translation_vector[1], translation_vector[2]);
    }

    // Apply rotation to each point
    for (int i = 0; i < rotated_translated_matrix.rows(); ++i) {
        rotated_translated_matrix.row(i) = (rotation_matrix * rotated_translated_matrix.row(i).transpose()).transpose();
    }

    return rotated_translated_matrix;
}


int main() {
    // Example usage
    std::vector<double> distances = {1, 2, 3}; // Example distances
    std::vector<double> angles = {30, 45, 60}; // Example angles

    // Convert polar coordinates to Cartesian coordinates
    Eigen::MatrixXd cartesian_coords = polar_to_cartesian(distances, angles);

    // Perform translation and rotation
    std::vector<double> translation_vector = {1.0, 2.0, 0.0}; // Example translation
    double rotation_angle = 45.0; // Example rotation angle
    Eigen::MatrixXd rotated_translated_matrix = translate_rotate_matrix(cartesian_coords, translation_vector, rotation_angle);

    // Output the rotated and translated matrix
    std::cout << "Rotated and translated matrix:" << std::endl;
    std::cout << "Matrix A:" << "\n" << cartesian_coords<< std::endl;
    std::cout << "Matrix B:" << "\n" << rotated_translated_matrix << std::endl;
    ICP_OUT result = icp(cartesian_coords, rotated_translated_matrix);
    for (int i = 0; i < result.distances.size(); i++) {
            std::cout << result.distances[i] << " ";
        }
    std::cout << std::endl;
    return 0;
}
