#pragma once
#include <opencv2\opencv.hpp>
#include "AssimpGLSkin.h"
#include "cv_skeleton.h"

//function for saving input frames
bool save_input_frame(
	const std::string& filename,
	const double& time,
	const cv::Mat& camera_pose,
	const float& win_width,
	const float& win_height,
	const float& fovy,
	const SkeletonNodeHard& snh,
	const cv::Mat& color,
	const cv::Mat& depth);

//function for loading input frames
bool load_input_frame(
	const std::string& filename,
	double& time,
	cv::Mat& camera_pose,
	cv::Mat& camera_matrix,
	SkeletonNodeHard& snh,
	cv::Mat& color,
	cv::Mat& depth);

SkeletonNodeHard hard_skeleton(SkeletonNode * node);

//converts from openGL depth (-1, 1) to true depth
cv::Mat depth_to_z(cv::Mat& depth, const cv::Mat& OPENGL_PROJECTION_MATRIX);


cv::Mat depth_to_HSV(const cv::Mat& depth);

//cv::Point project2D(cv::Vec4f pt, const cv::Mat& camera_matrix);
//cv::Vec4f get_origin(const cv::Mat& pt_m);

cv::Vec4f vertex(const cv::Mat& transform, const cv::Vec4f& local_vertex);

void cv_draw_bodypart_cylinder(const BodyPartDefinition& bpd, cv::Mat& image, const cv::Mat& camera_pose, const cv::Mat& camera_matrix, const SkeletonNodeHardMap& snhMap);
void cv_draw_volume(const cv::Scalar& color, const cv::Mat& volume_transform, const float& Y_SIDE_LENGTH, float X_SIDE_LENGTH, float Z_SIDE_LENGTH, cv::Mat& image, const cv::Mat& external_parameters, const cv::Mat& camera_matrix);