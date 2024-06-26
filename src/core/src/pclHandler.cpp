#include "pclHandler.h"

std::string convCrit[] =
{
    "not_converged",
    "iterations",
    "transformation_epsilon",
    "absolute_mse",
    "relative_mse",
    "nor_correspondence",
    "failure_after_max_iterations"
};

PclHandler::PclHandler()
{
}

TransformationComponents PclHandler::computeTransformation(const Eigen::MatrixX2d &firstScan, const Eigen::MatrixX2d &secondScan, double currentRotation)
{
    // Create Point Clouds to write data to
    PointCloud::Ptr sourceCloudPtr = PointCloud().makeShared();
    PointCloud::Ptr targetCloudPtr = PointCloud().makeShared();

    // Write data from matrix to point clouds
    *sourceCloudPtr = matrixToPointCloud(firstScan, currentRotation);
    *targetCloudPtr = matrixToPointCloud(secondScan, currentRotation);

    // Run ICP
    Eigen::Matrix4f tfMatrix = Eigen::Matrix4f::Identity();
    tfMatrix = computeAlignment(sourceCloudPtr, targetCloudPtr);

    // Return extracted TransformationComponents
    return extractTransformationComponents(tfMatrix);
}

Eigen::Matrix4f PclHandler::computeAlignment(const PointCloud::Ptr &sourcePoints, const PointCloud::Ptr &targetPoints)
{
    // Create ICP instance
    pcl::IterativeClosestPoint<PointT, PointT> icp;

    // Set registration output to the pointer of the source cloud
    // This overwrites the source cloud with the result of the icp so it can be used for another iteration
    PointCloud::Ptr registrationOutput = sourcePoints;

    // Create initial transformation matrix that does nothing
    Eigen::Matrix4f tfMatrix;
    tfMatrix << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    int iterations = 0;

    // Loop ICP until fitness score is under a threshhold or other criteria is met
    do
    {
        // Set ICP parameter
        icp.setMaximumIterations(MAX_ITERATIONS_ICP);
        icp.setMaxCorrespondenceDistance(MAX_CORRESPONDENCE_DISTANCE_ICP - (iterations * 20));
        icp.setRANSACOutlierRejectionThreshold(OUTLIER_REJECTION_THRESHOLD_ICP);
        icp.setTransformationEpsilon(TRANSFORMATION_EPSILON_ICP);
        icp.setTransformationRotationEpsilon(ROTATION_EPSILON_ICP);
        icp.setEuclideanFitnessEpsilon(EUCLIDEAN_FITNESS_EPSILON_ICP);

        // Set ICP source and target point cloud
        icp.setInputSource(registrationOutput);
        icp.setInputTarget(targetPoints);

        // Run ICP
        icp.align(*registrationOutput);

        // Get transformation matrix and store it to tfMatrix
        // If icp is not run for the first time the result gets added onto the results from previous iterations
        tfMatrix = icp.getFinalTransformation() * tfMatrix;

        iterations++; 

    } while (icp.getFitnessScore() > ICP_FITNESS_THRESHOLD && iterations < ICP_MAX_NR_CORRECTIONS);

    // Output of ICP results
    std::cout << "Converged: " << (bool)icp.hasConverged() << std::endl;
    std::cout << "Fitness: " << icp.getFitnessScore() << std::endl;
    std::cout << "Reason: " << convCrit[icp.getConvergeCriteria()->getConvergenceState()] << std::endl;

    return tfMatrix;
}

PointCloud PclHandler::matrixToPointCloud(const Eigen::MatrixX2d &matrix, double currentRotation)
{
    // Create PointCloud and push all points above a certain threshhold onto it
    PointCloud pointCloud;
    Eigen::RowVector3f pointVector;
    PointT point;

    for (int i = 0; i < matrix.rows(); i++)
    {
        if (matrix.row(i)[1] > MIN_ICP_DISTANCE || matrix.row(i)[1] > MAX_ICP_DISTANCE)
        {
            pointVector = polarToCartesianXYZ(matrix.row(i), currentRotation);
            point.x = pointVector[0];
            point.y = pointVector[1];
            point.z = pointVector[2];
            pointCloud.push_back(point);
        }
    }

    return pointCloud;
}

Eigen::RowVector3f PclHandler::polarToCartesianXYZ(const Eigen::RowVector2d &polarPoint, double currentRotation)
{
    Eigen::RowVector3f cart_point;

    double theta = polarPoint[0];
    double r = polarPoint[1];

    // Adds rotation to theta to align point cloud with global map axis
    cart_point[0] = (r)*cos(theta + currentRotation);
    cart_point[1] = (r)*sin(theta + currentRotation);
    cart_point[2] = 0;

    return cart_point;
}

TransformationComponents PclHandler::extractTransformationComponents(const Eigen::Matrix4f &transformationMatrix)
{
    TransformationComponents components;

    // Extract translation vector with only x and y as double
    components.translation_vector << static_cast<double>(-transformationMatrix(0, 3)),
        static_cast<double>(-transformationMatrix(1, 3));

    // Calculate rotation angle from the rotation matrix part of T
    // Assuming the rotation is around the Z-axis
    double thetaRad = std::atan2(transformationMatrix(1, 0), transformationMatrix(0, 0)); // atan2(sin, cos)
    components.rotation_angle = -thetaRad;                                                // Store in radians

    return components;
}
