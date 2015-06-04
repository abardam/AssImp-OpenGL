#include "AssimpCV.h"
#include <cv_draw_cylinder.h>

SkeletonNodeHard hard_skeleton(SkeletonNode * node, const cv::Mat& parent_transform_absolute){
	SkeletonNodeHard hard_node;
	//hard_node.mTransformation = cv::Mat(4, 4, CV_32F, &(node->mNode->mTransformation.a1));
	cv::Mat node_transform = cv::Mat(4, 4, CV_32F, &(node->mNode->mTransformation.a1));
	hard_node.mTransformation = parent_transform_absolute * node_transform * parent_transform_absolute.inv();
	hard_node.mName = node->mNode->mName.C_Str();
	hard_node.mParentName = node->mParent ? node->mParent->mNode->mName.C_Str() : "";

	for (auto it = node->mChildren.begin(); it != node->mChildren.end(); ++it){
		hard_node.mChildren.push_back(hard_skeleton(*it, hard_node.mTransformation * parent_transform_absolute));
	}

	return hard_node;
}




cv::Mat depth_to_HSV(const cv::Mat& depth){
	cv::Mat red(depth.rows, depth.cols, CV_8UC3, cv::Scalar(255, 0, 0));
	cv::Mat hsv_red;
	cv::cvtColor(red, hsv_red, CV_BGR2HSV);
	int numpix = depth.rows*depth.cols;
	cv::Mat depth_norm;
	cv::normalize(depth, depth_norm, 0, 1, CV_MINMAX, CV_32F);
	float * depth_norm_p = depth_norm.ptr<float>();
	cv::Vec3b * hsv_red_p = hsv_red.ptr<cv::Vec3b>();
	for (int i = 0; i < numpix; ++i){
		(*hsv_red_p)(0) = 255. * *depth_norm_p;
		++hsv_red_p;
		++depth_norm_p;
	}
	return hsv_red;
}

//cv::Point project2D(cv::Vec4f pt, const cv::Mat& camera_matrix){
//	pt[0] /= pt[2];
//	pt[1] /= pt[2];
//	pt[2] = 1;
//	pt[3] = 1;
//	cv::Mat pt_m = camera_matrix * cv::Mat(pt);
//	int x = pt_m.ptr<float>(0)[0];
//	int y = pt_m.ptr<float>(1)[0];
//
//	return cv::Point(x, y);
//}
//
//cv::Vec4f get_origin(const cv::Mat& pt_m){
//	cv::Vec4f vert;
//
//	vert(0) = pt_m.ptr<float>(0)[3];
//	vert(1) = pt_m.ptr<float>(1)[3];
//	vert(2) = pt_m.ptr<float>(2)[3];
//	vert(3) = 1;
//
//	return vert;
//}

cv::Vec4f vertex(const cv::Mat& transform, const cv::Vec4f& local_vertex){
	//first make a translation matrix out of the vertex
	cv::Mat translation = cv::Mat::eye(4, 4, CV_32F);
	translation.ptr<float>(0)[3] = local_vertex(0);
	translation.ptr<float>(1)[3] = local_vertex(1);
	translation.ptr<float>(2)[3] = local_vertex(2);

	//next transform
	return get_origin(transform * translation);
}

void cv_draw_bodypart_cylinder(const BodyPartDefinition& bpd, cv::Mat& image, const cv::Mat& camera_pose, const cv::Mat& camera_matrix, const SkeletonNodeHardMap& snhMap){

	float X_SIDE_LENGTH = 1;
	float Z_SIDE_LENGTH = 1;

	unsigned char color_r = bpd.mColor[0] * 255;
	unsigned char color_g = bpd.mColor[1] * 255;
	unsigned char color_b = bpd.mColor[2] * 255;

	cv::Vec3b color(color_b, color_g, color_r);

	float length;
	cv::Mat volume_transform = get_bodypart_transform(bpd, snhMap, camera_pose, &length);

	cv_draw_cylinder(image, X_SIDE_LENGTH, Z_SIDE_LENGTH, length, 8, volume_transform, camera_matrix, color);
}

void cv_draw_volume(const cv::Scalar& color, const cv::Mat& volume_transform, const float& length, float X_SIDE_LENGTH, float Z_SIDE_LENGTH, cv::Mat& image, const cv::Mat& external_parameters, const cv::Mat& camera_matrix, bool draw_axis){

	X_SIDE_LENGTH /= 2;
	Z_SIDE_LENGTH /= 2;
	
	cv::line(image,	project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
					project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),	color);
	cv::line(image,	project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
					project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),	color);
	cv::line(image,	project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
					project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),	color);
	cv::line(image,	project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
					project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),	color);

	//cv::line(image,
	//	project2D(vertex(child_transform, cv::Vec4f(X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
	//	project2D(vertex(child_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
	//	color);
	//cv::line(image,
	//	project2D(vertex(child_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
	//	project2D(vertex(child_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
	//	color);
	//cv::line(image,
	//	project2D(vertex(child_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
	//	project2D(vertex(child_transform, cv::Vec4f(X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
	//	color);
	//cv::line(image,
	//	project2D(vertex(child_transform, cv::Vec4f(X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
	//	project2D(vertex(child_transform, cv::Vec4f(X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
	//	color);
	//
	//cv::line(image,
	//	project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
	//	project2D(vertex(child_transform, cv::Vec4f(X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
	//	color);
	//cv::line(image,
	//	project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
	//	project2D(vertex(child_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
	//	color);
	//cv::line(image,
	//	project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
	//	project2D(vertex(child_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
	//	color);
	//cv::line(image,
	//	project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
	//	project2D(vertex(child_transform, cv::Vec4f(X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
	//	color);

	cv::line(image,	project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, length, Z_SIDE_LENGTH)), camera_matrix),
					project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, length, Z_SIDE_LENGTH)), camera_matrix),	color);
	cv::line(image,	project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, length, Z_SIDE_LENGTH)), camera_matrix),
					project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, length, -Z_SIDE_LENGTH)), camera_matrix),	color);
	cv::line(image,	project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, length, -Z_SIDE_LENGTH)), camera_matrix),
					project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, length, -Z_SIDE_LENGTH)), camera_matrix),	color);
	cv::line(image,	project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, length, -Z_SIDE_LENGTH)), camera_matrix),
					project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, length, Z_SIDE_LENGTH)), camera_matrix),	color);
	cv::line(image,	project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
					project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, length, Z_SIDE_LENGTH)), camera_matrix),	color);
	cv::line(image,	project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
					project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, length, Z_SIDE_LENGTH)), camera_matrix),	color);
	cv::line(image,	project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
					project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, length, -Z_SIDE_LENGTH)), camera_matrix),	color);
	cv::line(image,	project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
					project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, length, -Z_SIDE_LENGTH)), camera_matrix),	color);

	if (draw_axis){
		cv::line(image, project2D(vertex(volume_transform, cv::Vec4f(0, 0, 0)), camera_matrix),
						project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH/2, 0, 0)), camera_matrix), cv::Scalar(0,0,0xff));
		cv::line(image, project2D(vertex(volume_transform, cv::Vec4f(0, 0, 0)), camera_matrix),
						project2D(vertex(volume_transform, cv::Vec4f(0, length/2, 0)), camera_matrix), cv::Scalar(0,0xff,0));
		cv::line(image, project2D(vertex(volume_transform, cv::Vec4f(0, 0, 0)), camera_matrix),
						project2D(vertex(volume_transform, cv::Vec4f(0, 0, Z_SIDE_LENGTH/2)), camera_matrix), cv::Scalar(0xff,0,0));
	}
}