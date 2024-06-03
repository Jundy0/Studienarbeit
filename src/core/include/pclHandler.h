#ifndef __PCL_HANDLER_H__
#define __PCL_HANDLER_H__

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <pcl/common/transforms.h>

////////////////////
// ICP parameters //
////////////////////
/// @brief Max Iterations before the algorithm will be considered to have converged
const int MAX_ITERATIONS_ICP = 25;
/// @brief Mximum distance threshold between two correspondent points in source <-> target.
const float MAX_CORRESPONDENCE_DISTANCE_ICP = 150.0f; // 150mmm
/// @brief Inlier distance threshold for the internal RANSAC outlier rejection loop
const float OUTLIER_REJECTION_THRESHOLD_ICP = 10.0f; // 10mm
/// @brief Maximum allowable translation squared difference between two consecutive transformations in order for an optimization to be considered as having converged.
const float TRANSFORMATION_EPSILON_ICP = 1e-12 * 1e-12;
/// @brief Maximum allowable rotation difference between two consecutive transformations in order for an optimization to be considered as having converged.
const float ROTATION_EPSILON_ICP = 1e-5; // 0.029deg
/// @brief Maximum allowed distance error before the algorithm will be considered to have converged
const float EUCLIDEAN_FITNESS_EPSILON_ICP = 1e-10;

/// @brief The transformation components extracted from the transformation matrix got using icp
typedef struct
{
    /// @brief A vector composed of global x and y values
    Eigen::RowVector2d translation_vector;
    /// @brief A rotation angle in deg
    double rotation_angle;
} TransformationComponents;

/// @brief Point definition used during icp computation.
typedef pcl::PointXYZ PointT;
/// @brief PointCloud definition used during icp computation.
typedef pcl::PointCloud<PointT> PointCloud;

/// @brief Interface between particle and the pcl library.
class PclHandler
{
public:
    /// @brief Empty Constructor.
    PclHandler();

    /// @brief Empty Deconstructor.
    ~PclHandler() {}

    /// @brief Computes the transformation matrix between two sets of scan data and parses them into TransformationComponents.
    /// @param firstScan First set of scan data composed of polar coordinates.
    /// @param secondScan Second set of scan data composed of polar coordinates.
    /// @param currentRotation The rotation angle of the particle at the time of the first scan.
    //                          Used to align local axis of the scan data with the global axis of the map.
    /// @return TransformationComponents got from the calculated transformation matrix.
    TransformationComponents computeTransformation(const Eigen::MatrixX2d &firstScan, const Eigen::MatrixX2d &secondScan, double currentRotation);

private:
    /// @brief Use IterativeClosestPoint to find a precise alignment from the source cloud to the target cloud.
    /// @param sourcePoints The "source" points, i.e., the points that must be transformed to align with the target point cloud.
    /// @param targetPoints The "target" points, i.e., the points to which the source point cloud will be aligned.
    /// @return A transformation matrix that will precisely align the points in source to the points in target.
    Eigen::Matrix4f computeAlignment(const PointCloud::Ptr &sourcePoints, const PointCloud::Ptr &targetPoints);

    /// @brief Converts the matrix to a point cloud aligned with the global x and y axis.
    /// @param matrix Set of scan data composed of polar coordinates.
    /// @param currentRotation The rotation angle of the particle at the time of the first scan.
    //                          Used to align local axis of the scan data with the global axis of the map.
    /// @return The PointCloud composed of all the points within the given matrix.
    PointCloud matrixToPointCloud(const Eigen::MatrixX2d &matrix, double currentRotation);

    /// @brief Aligns polar point to the global axis and parses it into x, y and z coordinates with z always being zero.
    /// @param polarPoint The polar point.
    /// @param currentRotation The rotation angle of the particle at the time of the first scan.
    /// @return Cartesian representation of the polar point (X,Y,0)
    Eigen::RowVector3f polarToCartesianXYZ(const Eigen::RowVector2d &polarPoint, double currentRotation);

    /// @brief Extracts the TransformationComponents from the transformation matrix computed by the icp algorithm.
    /// @param transformationMatrix The transformation matrix. Result of the icp algorithm.
    /// @return TransformationComponents composed of a vector with the x and y delta and the rotation delta between the two sets of scan data.
    TransformationComponents extractTransformationComponents(const Eigen::Matrix4f &transformationMatrix);
};

#endif //__PCL_HANDLER_H__
