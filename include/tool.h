//
// Created by user on 24-7-10.
//

#ifndef TOOL_H
#define TOOL_H
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>


class SuperTool {
public:
	static void external_process();
	static void generate_mask();

private:
	static Eigen::Matrix3d RotateMatrix3d(int degree, const Eigen::Vector3d& axis);

	static void STPrint(const Eigen::Matrix3d& R);
	static void STPrint(const Eigen::Vector3d& t);

};



#endif //TOOL_H
