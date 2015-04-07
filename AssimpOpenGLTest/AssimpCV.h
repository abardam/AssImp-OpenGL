#pragma once
#include <opencv2\opencv.hpp>
#include "AssimpGLSkin.h"

//pointerless version of skeleton node for saving/loading
struct SkeletonNodeHard{
	std::vector<SkeletonNodeHard> mChildren;
	cv::Mat mTransformation;
	cv::Mat mTempTransformation; //temporary GLOBAL transformation
	std::string mName;
	std::string mParentName;

	std::string toString(std::string prefix) const;

	SkeletonNodeHard(){};
	SkeletonNodeHard(const SkeletonNodeHard& c){
		mChildren = c.mChildren;
		mTransformation = c.mTransformation.clone();
		mTempTransformation = c.mTempTransformation.clone();
		mName = c.mName;
		mParentName = c.mParentName;
	};
};
typedef std::map<const std::string, SkeletonNodeHard*> SkeletonNodeHardMap;
typedef std::pair<const std::string, SkeletonNodeHard*> SkeletonNodeHardEntry;

std::ostream& operator<<(std::ostream& os, const SkeletonNodeHard& snh);

//opencv serialization for skeleton node hard
void write(cv::FileStorage& fs, const std::string&, const SkeletonNodeHard& n);
void read(const cv::FileNode& node, SkeletonNodeHard& n, const SkeletonNodeHard& default_value = SkeletonNodeHard());

//opencv serialization for body parts
void write(cv::FileStorage& fs, const std::string&, const BodyPartDefinition& n);
void read(const cv::FileNode& node, BodyPartDefinition& n, const BodyPartDefinition& default_value = BodyPartDefinition());

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

cv::Point project2D(cv::Vec4f pt, const cv::Mat& camera_matrix);

cv::Vec4f get_origin(const cv::Mat& pt_m);

cv::Vec4f vertex(const cv::Mat& transform, const cv::Vec4f& local_vertex);

//retrieves the global transform for the specified body part. you need to run cv_draw_and_build_skeleton first in order to set the snh transformations. also returns the length of the body part, if you want.
cv::Mat get_bodypart_transform(const BodyPartDefinition& bpd, const SkeletonNodeHardMap& snhMap, const cv::Mat * external_parameters = 0, float * length = 0);

void cv_draw_and_build_skeleton(SkeletonNodeHard * node, const cv::Mat& parent_transform, const cv::Mat& camera_matrix, SkeletonNodeHardMap * snhMap = NULL, cv::Mat& image = cv::Mat());

void cv_draw_bodypart_cylinder(const BodyPartDefinition& bpd, cv::Mat& image, const cv::Mat& camera_pose, const cv::Mat& camera_matrix, const SkeletonNodeHardMap& snhMap);
void cv_draw_volume(const cv::Scalar& color, const cv::Mat& volume_transform, const float& Y_SIDE_LENGTH, float X_SIDE_LENGTH, float Z_SIDE_LENGTH, cv::Mat& image, const cv::Mat& external_parameters, const cv::Mat& camera_matrix);