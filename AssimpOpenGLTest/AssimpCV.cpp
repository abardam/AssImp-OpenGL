#include "AssimpCV.h"
#include <cv_draw_cylinder.h>

bool save_input_frame(
	const std::string& filename, 
	const double& time, 
	const cv::Mat& camera_pose, 
	const float& win_width, 
	const float& win_height, 
	const float& fovy, 
	const SkeletonNodeHard& snh, 
	const cv::Mat& color, 
	const cv::Mat& depth){

	cv::FileStorage fs(filename, cv::FileStorage::WRITE);

	if (!fs.isOpened()) return false;

	fs << "time" << time;
	fs << "camera_extrinsic" << camera_pose;
	//fs << "camera_intrinsic" << camera_intrinsic;
	fs << "camera_intrinsic" << "{"
		<< "width" << win_width << "height" << win_height << "fovy" << fovy
		<< "}";
	fs << "skeleton" << snh;
	fs << "color" << color;
	fs << "depth" << depth;
	fs.release();

	return true;
}

bool load_input_frame(
	const std::string& filename,
	double& time,
	cv::Mat& camera_pose,
	cv::Mat& camera_matrix,
	SkeletonNodeHard& snh,
	cv::Mat& color,
	cv::Mat& depth){

	float win_width;
	float win_height;
	float fovy;

	cv::FileStorage fs(filename, cv::FileStorage::READ);

	if (!fs.isOpened()) return false;

	fs["time"] >> time;
	fs["camera_extrinsic"] >> camera_pose;
	fs["camera_intrinsic"]["width"] >> win_width;
	fs["camera_intrinsic"]["height"] >> win_height;
	fs["camera_intrinsic"]["fovy"] >> fovy;
	fs["skeleton"] >> snh;
	fs["color"] >> color;
	fs["depth"] >> depth;

	fs.release();

	camera_matrix = cv::Mat::eye(4, 4, CV_32F);
	camera_matrix.ptr<float>(0)[0] = -win_width / (2 * tan(AI_DEG_TO_RAD((fovy * (win_width / win_height) / 2.)))); //for some strange reason this is inaccurate for non-square aspect ratios
	camera_matrix.ptr<float>(1)[1] = win_height / (2 * tan(AI_DEG_TO_RAD(fovy / 2.)));
	camera_matrix.ptr<float>(0)[2] = win_width / 2 + 0.5;
	camera_matrix.ptr<float>(1)[2] = win_height / 2 + 0.5;
	//camera_intrinsic.ptr<float>(2)[2] = -1;

	return true;
}

SkeletonNodeHard hard_skeleton(SkeletonNode * node){
	SkeletonNodeHard hard_node;
	hard_node.mTransformation = cv::Mat(4, 4, CV_32F, &(node->mNode->mTransformation.a1));
	hard_node.mName = node->mNode->mName.C_Str();
	hard_node.mParentName = node->mParent ? node->mParent->mNode->mName.C_Str() : "";

	for (auto it = node->mChildren.begin(); it != node->mChildren.end(); ++it){
		hard_node.mChildren.push_back(hard_skeleton(*it));
	}

	return hard_node;
}


cv::Mat depth_to_z(cv::Mat& depth, const cv::Mat& projection){
	float A = projection.ptr<float>(2)[2];
	float B = projection.ptr<float>(2)[3];
	int numpix = depth.rows * depth.cols;
	cv::Mat out(depth.rows, depth.cols, CV_32F);

	float * depth_ptr = depth.ptr<float>();
	float * out_ptr = out.ptr<float>();

	float zNear = -B / (1.0 - A);
	float zFar = B / (1.0 + A);

	for (int i = 0; i < numpix; ++i){
		float d = *depth_ptr;

		if (d == 1) //nothing
		{
			*out_ptr = 0;
		}
		else{

			//*out_ptr = 0.5 * (-A*d + B) + 0.5;
			//*out_ptr = -2 * B / (d + 1 + 2 * A);
			float z_b = d;
			float z_n = 2.0 * z_b - 1.0;
			float z_e = -2.0 * zNear * zFar / (zFar + zNear - z_n * (zFar - zNear));
			*out_ptr = z_e;

		}
		++depth_ptr;
		++out_ptr;
	}

	return out;
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
	cv::Mat volume_transform = get_bodypart_transform(bpd, snhMap, &camera_pose, &length);

	cv_draw_cylinder(image, X_SIDE_LENGTH, Z_SIDE_LENGTH, length, 8, volume_transform, camera_matrix, color);
}

void cv_draw_volume(const cv::Scalar& color, const cv::Mat& volume_transform, const float& length, float X_SIDE_LENGTH, float Z_SIDE_LENGTH, cv::Mat& image, const cv::Mat& external_parameters, const cv::Mat& camera_matrix){

	X_SIDE_LENGTH /= 2;
	Z_SIDE_LENGTH /= 2;
	
	cv::line(image,
		project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
		project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
		color);
	cv::line(image,
		project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
		project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
		color);
	cv::line(image,
		project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
		project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
		color);
	cv::line(image,
		project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
		project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
		color);

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

	cv::line(image,
		project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, length, Z_SIDE_LENGTH)), camera_matrix),
		project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, length, Z_SIDE_LENGTH)), camera_matrix),
		color);
	cv::line(image,
		project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, length, Z_SIDE_LENGTH)), camera_matrix),
		project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, length, -Z_SIDE_LENGTH)), camera_matrix),
		color);
	cv::line(image,
		project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, length, -Z_SIDE_LENGTH)), camera_matrix),
		project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, length, -Z_SIDE_LENGTH)), camera_matrix),
		color);
	cv::line(image,
		project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, length, -Z_SIDE_LENGTH)), camera_matrix),
		project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, length, Z_SIDE_LENGTH)), camera_matrix),
		color);

	cv::line(image,
		project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
		project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, length, Z_SIDE_LENGTH)), camera_matrix),
		color);
	cv::line(image,
		project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, Z_SIDE_LENGTH)), camera_matrix),
		project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, length, Z_SIDE_LENGTH)), camera_matrix),
		color);
	cv::line(image,
		project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
		project2D(vertex(volume_transform, cv::Vec4f(-X_SIDE_LENGTH, length, -Z_SIDE_LENGTH)), camera_matrix),
		color);
	cv::line(image,
		project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH)), camera_matrix),
		project2D(vertex(volume_transform, cv::Vec4f(X_SIDE_LENGTH, length, -Z_SIDE_LENGTH)), camera_matrix),
		color);
}