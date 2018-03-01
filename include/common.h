#ifndef COMMON_INCLUDE_H_
#define COMMON_INCLUDE_H_

//define the commonly included file to avoid a long include list
//for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

//for Sophus
#include <sophus/se3.h>
using Sophus::SE3;
using Sophus::SO3;

//for opencv
#include <opencv2/core/core.hpp>
using cv::Mat;

//std STL
#include <vector>
#include <list>
#include <memory>	//std::shared_ptr, std::unique_ptr
#include <string>
#include <iostream>	//cout, endl, cerr
#include <set>		//按照一定顺序排列的非重复元素的集合
#include <unordered_map>
#include <map>
using namespace std;
#endif