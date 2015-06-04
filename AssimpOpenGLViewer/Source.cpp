#include <opencv2\opencv.hpp>
#include "AssimpCV.h"



int main(int argc, char * argv[]){
	if (argc <= 1){
		std::cout << "Please enter directory\n";
		return 0;
	}

	std::string video_directory(argv[1]);
	int i = 0;
	std::stringstream filenameSS;
	cv::FileStorage fs;

	filenameSS << video_directory << "/bodypartdefinitions.xml.gz";

	fs.open(filenameSS.str(), cv::FileStorage::READ);
	BodyPartDefinitionVector bpdv;
	for (auto it = fs["bodypartdefinitions"].begin();
		it != fs["bodypartdefinitions"].end();
		++it){
		BodyPartDefinition bpd;
		read(*it, bpd);
		bpdv.push_back(bpd);
	}
	fs.release();

	cv::Mat colorMat, camera_extrinsic, camera_intrinsic;
	SkeletonNodeHardMap snhMap;
	while (true){
		filenameSS.str("");
		filenameSS << video_directory << "/" << i << ".xml.gz";
		fs.open(filenameSS.str(), cv::FileStorage::READ);

		double time;
		int facing;
		SkeletonNodeHard snh;
		cv::Mat depthMat;

		if (!load_input_frame(filenameSS.str(), time, camera_extrinsic, camera_intrinsic, snh, colorMat, depthMat, facing)) {
			break;
		}

		cv_draw_and_build_skeleton(&snh, cv::Mat::eye(4,4,CV_32F), camera_intrinsic, camera_extrinsic, &snhMap, colorMat);
		for (auto it = bpdv.begin(); it != bpdv.end(); ++it){
			cv::Scalar color(it->mColor[2] * 255, it->mColor[1] * 255, it->mColor[0] * 255);
			float length;
			cv::Mat volume_transform = get_bodypart_transform(*it, snhMap, camera_extrinsic, &length);
			cv_draw_volume(color, volume_transform, length, 1, 1, colorMat, camera_extrinsic, camera_intrinsic);
		}
		//cv_draw_volume(bpdv[3], colorMat, camera_intrinsic, snhMap);
		snhMap.clear();

		cv::imshow("color", colorMat);

		//cv::Mat depthMat;
		//fs["depth"] >> depthMat;
		//
		//cv::Mat depthHSV = depth_to_HSV(depthMat);
		//
		//cv::imshow("depth", depthHSV);

		cv::waitKey(10);
		++i;
	}
}