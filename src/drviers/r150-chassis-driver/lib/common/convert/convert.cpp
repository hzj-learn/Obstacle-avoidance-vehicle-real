#include "common/convert/convert.h"

#include <iostream>
#include "math.h"
#include <Eigen/Geometry>

namespace xag_chassis 
{
namespace common
{
// Eigen::Matrix4d Converter::SuperXState_to_transpose(FCToXrvioPoseData& _state)
// {
// // 	-------------------------------普通转换方式---------------------------------------
// // 		Eigen::Matrix4d transpose_;
// // 		transpose_.setZero(4,4);
// // 		Eigen::Matrix3d rotation;
// // 		Eigen::Vector3d translate;
// // 		translate[0] = _state.PosX;
// // 		translate[1] = _state.PosY;
// // 		translate[2] = _state.PosZ;
// // 		double roRoll  = _state.Roll*0.0174532922f;
// // 		double roPitch = _state.Pitch*0.0174532922f;
// // 		double roYaw   = _state.Yaw*0.0174532922f;
// // 		Eigen::Matrix3d rotate1;
// // 		rotate1 << 1,0,0,0,cos(roRoll),-sin(roRoll),0,sin(roRoll),cos(roRoll);
// // 		Eigen::Matrix3d rotate2;
// // 		rotate2 << cos(roPitch),0,sin(roPitch),0,1,0,-sin(roPitch),0,cos(roPitch);
// // 		Eigen::Matrix3d rotate3;
// // 		rotate3 << cos(roYaw),-sin(roYaw),0,sin(roYaw),cos(roYaw),0,0,0,1;
// //
// // 		rotation = rotate3*rotate2*rotate1;
// //
// // 		transpose_.block<3,3>(0,0) = rotation;
// // 		transpose_.block<3,1>(0,3) = translate;
// // 		transpose_(3,3)            = 1;
// // 	----------------------------------------------------------------------------------
// 		// Eigen::AngleAxis<double> angle_roll(_state.estimator_attitude.eularangles[0]*0.0174532922f, Eigen::Vector3d(1,0,0));
// 		// Eigen::AngleAxis<double> angle_pitch(_state.body_attitude.eularangles[1]*0.0174532922f,Eigen::Vector3d(0,1,0));
// 		// Eigen::AngleAxis<double> angle_yaw(_state.body_attitude.eularangles[2]*0.0174532922f,  Eigen::Vector3d(0,0,1));
// 		// Eigen::Translation3d translation3(_state.position_Relative_TakeOff..PosX,_state.takeoff_cor_position.PosY,_state.takeoff_cor_position.PosZ);

// 		// Eigen::Matrix4d transform = Eigen::Isometry3d(translation3*angle_yaw*angle_pitch*angle_roll).matrix();
// 		// return transform;
// }

Eigen::Matrix4d Converter::poseToTransformMatrix(const Eigen::Vector3d& _ned, Eigen::Vector3d& _euler_angle, const Eigen::Vector3d& _offset_angle, const Eigen::Vector3d& _offset_position)
{
    // Eigen::AngleAxis<float> angle_roll(pose.attitude.eularangles[0]*0.0174532922f, Eigen::Vector3f(1,0,0));
    // Eigen::AngleAxis<float> angle_pitch(pose.attitude.eularangles[1]*0.0174532922f,Eigen::Vector3f(0,1,0));
    // Eigen::AngleAxis<float> angle_yaw(pose.attitude.eularangles[2]*0.0174532922f,  Eigen::Vector3f(0,0,1));
    Eigen::Quaterniond _offset_qua;
    eulerToQuaternion(_offset_angle, _offset_qua);
    Eigen::Quaterniond _qua;
    // _euler_angle += _visualization;
    eulerToQuaternion(_euler_angle, _qua);
    // Eigen::Quaterniond _visualization_qua;
    // eulerToQuaternion(_visualization, _visualization_qua);
    Eigen::Translation3d _offset_translation(_offset_position(0), _offset_position(1), _offset_position(2));

    Eigen::Translation3d translation3(_ned[0], _ned[1], _ned[2]);
    Eigen::Matrix4d transform = Eigen::Isometry3d(translation3 * _qua * _offset_translation * _offset_qua).matrix();//angle_yaw*angle_pitch*angle_roll
    return transform;
}

// 纬 经 海拔(飞控返回原始数据) --> 北x 东y 地z(单位:m)
void Converter::llsToNed(const Eigen::Vector3i& _lls_takeoff, const Eigen::Vector3i& _lls, Eigen::Vector3d& _ned)
{
    double temp = static_cast<double>(_lls[1] - _lls_takeoff[1])*LANLON_COEFFICIENT;
    temp *= cos(static_cast<double>(_lls[0])/pow(10,7)*0.0174532925f);
    _ned[1] = temp;
    temp = static_cast<double>(_lls[0] - _lls_takeoff[0])*LANLON_COEFFICIENT;
    _ned[0] = temp;
    _ned[2] = -static_cast<double>(_lls[2] - _lls_takeoff[2]) / 100.0f;
}

void Converter::nedToLls(const Eigen::Vector3i& _lls_takeoff, const Eigen::Vector3d& _ned, Eigen::Vector3i& _lls)
{
    _lls[0] = static_cast<int32_t>(static_cast<double>(_ned[0])/LANLON_COEFFICIENT) + _lls_takeoff[0];
	_lls[1] = static_cast<int32_t>((static_cast<double>(_ned[1])/cos(static_cast<double>(_lls[0])/pow(10,7)*0.0174532925f))/LANLON_COEFFICIENT) 
			  + _lls_takeoff[1];
    _lls[2] = _lls_takeoff[2] - _ned[2] * 100.0f;
}

void Converter::eulerToQuaternion(const Eigen::Vector3d& _euler, Eigen::Quaterniond& _qua)
{
    Eigen::AngleAxis<double> angle_roll(_euler[0]*0.0174532922f, Eigen::Vector3d(1,0,0));
    Eigen::AngleAxis<double> angle_pitch(_euler[1]*0.0174532922f,Eigen::Vector3d(0,1,0));
    Eigen::AngleAxis<double> angle_yaw(_euler[2]*0.0174532922f,  Eigen::Vector3d(0,0,1));
    _qua = Eigen::Quaterniond(angle_yaw*angle_pitch*angle_roll);
}

}
}
