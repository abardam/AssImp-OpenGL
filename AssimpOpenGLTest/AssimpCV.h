#pragma once
#include <opencv2\opencv.hpp>
#include "AssimpGLSkin.h"
#include "cv_skeleton.h"

//also converts from AssImp order (parent * child) to GHOST order (child * parent)
SkeletonNodeHard hard_skeleton(SkeletonNode * node, const cv::Mat& parent_transform_absolute);

//converts from openGL depth (-1, 1) to true depth
cv::Mat depth_to_z(cv::Mat& depth, const cv::Mat& OPENGL_PROJECTION_MATRIX);


cv::Mat depth_to_HSV(const cv::Mat& depth);

//cv::Point project2D(cv::Vec4f pt, const cv::Mat& camera_matrix);
//cv::Vec4f get_origin(const cv::Mat& pt_m);

cv::Vec4f vertex(const cv::Mat& transform, const cv::Vec4f& local_vertex);

void cv_draw_bodypart_cylinder(const BodyPartDefinition& bpd, cv::Mat& image, const cv::Mat& camera_pose, const cv::Mat& camera_matrix, const SkeletonNodeHardMap& snhMap);
void cv_draw_volume(const cv::Scalar& color, const cv::Mat& volume_transform, const float& Y_SIDE_LENGTH, float X_SIDE_LENGTH, float Z_SIDE_LENGTH, cv::Mat& image, const cv::Mat& external_parameters, const cv::Mat& camera_matrix);