
#ifndef CONVERT_H_
#define CONVERT_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#define LANLON_COEFFICIENT 0.01112f

namespace xag_chassis
{
namespace common
{

class Converter
{
public:
    //Eigen::Matrix4d SuperXState_to_transpose(FCToXrvioPoseData& _state);

    Eigen::Matrix4d poseToTransformMatrix(const Eigen::Vector3d& _ned, Eigen::Vector3d& _euler_angle, const Eigen::Vector3d& _offset_angle, const Eigen::Vector3d& _offset_position);

    void llsToNed(const Eigen::Vector3i& _lls_takeoff, const Eigen::Vector3i& _lls, Eigen::Vector3d& _ned);
    void nedToLls(const Eigen::Vector3i& _lls_takeoff, const Eigen::Vector3d& _ned, Eigen::Vector3i& _lls);
    void eulerToQuaternion(const Eigen::Vector3d& _euler, Eigen::Quaterniond& _qua);
};

}
}

#endif
