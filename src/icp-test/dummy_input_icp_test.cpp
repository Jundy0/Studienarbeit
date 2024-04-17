#include "./dummy_input_icp_test.h"

// Function to convert polar coordinates to Cartesian coordinates using Eigen library
Eigen::MatrixXd polar_to_cartesian(const std::vector<double>& distances, const std::vector<double>& angles) {
    Eigen::MatrixXd initial_coordinates(3, distances.size());
    for (int i = 0; i < distances.size(); i++) {
        double x = distances[i] * cos(angles[i] * M_PI / 180);
        double y = distances[i] * sin(angles[i] * M_PI / 180);
        initial_coordinates.col(i) << x, y, 0.0; // Setting x, y, z = 0
    }
    std::cout << initial_coordinates.rows() << "\n" << initial_coordinates.cols() << std::endl;
    return initial_coordinates.transpose();
}

Eigen::MatrixXd polar_to_cartesian_from_matrix(Eigen::MatrixX2d points)
{
    Eigen::MatrixXd cartesian_coords(3, points.rows());
    for (int i = 0; i < points.rows(); i++) {
        double x = points.row(i)[0]* cos(points.row(i)[1] * M_PI / 180);
        double y = points.row(i)[0] * sin(points.row(i)[1] * M_PI / 180);
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

Eigen::MatrixX2d load_points_from_file()
{
    return getDataFromFile();
}

Eigen::MatrixXd get_matrix_from_points(Eigen::MatrixX2d points = load_points_from_file())
{
    return polar_to_cartesian_from_matrix(points);
}

Eigen::MatrixXd apply_transformation(Eigen::MatrixXd start_matrix)
{
    std::vector<double> translation_vector = {4.0, 1.0, 0.0}; // Example translation
    double rotation_angle = 20.0; // Example rotation angle
    translate_rotate_matrix(start_matrix, translation_vector, rotation_angle);
}

void call_icp()
{
    // Convert polar coordinates to Cartesian coordinates
    Eigen::MatrixXd initial_coordinates = get_matrix_from_points();
    // Perform translation and rotation
    Eigen::MatrixXd rotated_translated_matrix = apply_transformation(initial_coordinates);
    ICP_OUT result = icp(initial_coordinates, rotated_translated_matrix, 50, 0.000000001);

    std::cout << "Transformation Matrix" << std::endl;
    std::cout << result.trans << std::endl;

    std::cout<< "Iter:"<<  std::endl;
    std::cout << result.iter << std::endl;

    std::cout << "extracted:" << std::endl;
    TransformationComponents trans_comps = extractTransformation(result.trans);
    std::cout << trans_comps.translation_vector << std::endl;
    std::cout<< trans_comps.rotation_angle_deg << std::endl;
}

int main() {
    call_icp();
    return 0;
}
