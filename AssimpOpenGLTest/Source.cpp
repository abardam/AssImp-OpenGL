/* ----------------------------------------------------------------------------
// Simple sample to prove that Assimp is easy to use with OpenGL.
// It takes a file name as command line parameter, loads it using standard
// settings and displays it.
//
// If you intend to _use_ this code sample in your app, do yourself a favour
// and replace immediate mode calls with VBOs ...
//
// The vc8 solution links against assimp-release-dll_win32 - be sure to
// have this configuration built.
// ----------------------------------------------------------------------------
*/

#include <stdlib.h>
#include <stdio.h>

#include <GL/glut.h>

#include <map>
#include <vector>

#include "AssimpGLSkin.h"
#include "AssimpGLDraw.h"

/* assimp include files. These three are usually needed. */
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/color4.h>

//opencv
#include <opencv2\opencv.hpp>

#include "AssimpCV.h"
#include <glcv.h>
#include <cv_draw_common.h>

//create idrectory
#include <Windows.h>

/* the global Assimp scene object */
const  aiScene* scene = NULL;
GLuint scene_list = 0;
aiVector3D scene_min, scene_max, scene_center;


//global texture map
TextureIDMap gTextureIDMap;

//global node map
NodeMap gNodeMap;

//global necessity map
NecessityMap gNecessityMap;

//global mesh map
MeshMap gMeshMap;

//global skeleton root
SkeletonNode * gSkeletonRoot;

//global inverse root transform
aiMatrix4x4 gInverseRootTransform;

//global bone info
BoneMap gBoneMap;
std::vector<BoneInfo> gBoneInfoVector;
std::vector<VertexBoneDataVector> gVertexBoneVector; //index = mesh ID

//body parts
BodyPartDefinitionVector gBodyPartDefinitionVector;

/* current rotation angle */
static float angle = 0.f;

float zNear = 1.0, zFar = 10.0;

//manual rotation angle
float angle_x = 0.f;
float angle_y = 0.f;

//manual zoom
float zoom = 1.f;

//mouse
int mouse_x, mouse_y;
bool mouse_down = false;
bool auto_rotate = true;

float current_matrix[16];
cv::Mat modelview_matrix, modelview_matrix_rotpt;

#define ZOOM_VALUE 0.01
#define BIG_ZOOM_VALUE 0.1
#define ROTATE_VALUE 0.001

//saving

double time = 0;
double maxtime = 0;
double elapsed_time = 0;
double current_time;
double prev_time;

bool save_playback = false;
bool show_skeleton = false;

//play/pause
bool playing = true;

//whether to use the actual time for rendering animations, or to use a fixed time per frame (for slow rendering)
bool real_time = false;
#define FIXED_TIME_INTERVAL 0.1

cv::Mat camera_intrinsic;

int file_no = 0;

#define aisgl_min(x,y) (x<y?x:y)
#define aisgl_max(x,y) (y>x?y:x)

//window dimensions
int win_width, win_height;
//window name
int window1;

float fovy = 45.;

std::string savepath;


void releaseSkeleton(SkeletonNode * skeletonRoot){
	for (auto it = skeletonRoot->mChildren.begin(); it != skeletonRoot->mChildren.end(); ++it){
		releaseSkeleton(*it);
	}
	delete skeletonRoot;
}
/* ---------------------------------------------------------------------------- */
void reshape(int width, int height)
{
	const double aspectRatio = (float)width / height, fieldOfView = fovy;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fieldOfView, aspectRatio,
		zNear, zFar);  /* Znear and Zfar */
	glViewport(0, 0, width, height);
	win_width = width;
	win_height = height;

	camera_intrinsic.create(4, 4, CV_32F);
	glGetFloatv(GL_PROJECTION_MATRIX, (GLfloat*)camera_intrinsic.data);
	camera_intrinsic = camera_intrinsic.t();
}

/* ---------------------------------------------------------------------------- */
void get_bounding_box_for_node(const aiNode* nd,
	aiVector3D* min,
	aiVector3D* max,
	aiMatrix4x4* trafo
	){
	aiMatrix4x4 prev;
	unsigned int n = 0, t;

	prev = *trafo;
	aiMultiplyMatrix4(trafo, &nd->mTransformation);

	for (; n < nd->mNumMeshes; ++n) {
		const aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];
		for (t = 0; t < mesh->mNumVertices; ++t) {

			aiVector3D tmp = mesh->mVertices[t];
			aiTransformVecByMatrix4(&tmp, trafo);

			min->x = aisgl_min(min->x, tmp.x);
			min->y = aisgl_min(min->y, tmp.y);
			min->z = aisgl_min(min->z, tmp.z);

			max->x = aisgl_max(max->x, tmp.x);
			max->y = aisgl_max(max->y, tmp.y);
			max->z = aisgl_max(max->z, tmp.z);
		}
	}

	for (n = 0; n < nd->mNumChildren; ++n) {
		get_bounding_box_for_node(nd->mChildren[n], min, max, trafo);
	}
	*trafo = prev;
}

/* ---------------------------------------------------------------------------- */
void get_bounding_box(aiVector3D* min, aiVector3D* max)
{
	aiMatrix4x4 trafo;
	aiIdentityMatrix4(&trafo);

	min->x = min->y = min->z = 1e10f;
	max->x = max->y = max->z = -1e10f;
	get_bounding_box_for_node(scene->mRootNode, min, max, &trafo);
}



/* ---------------------------------------------------------------------------- */
void do_motion(void)
{
	static GLint prev_time = 0;
	static GLint prev_fps_time = 0;
	static int frames = 0;

	int time = glutGet(GLUT_ELAPSED_TIME);
	angle += (time - prev_time)*0.01;
	prev_time = time;

	frames += 1;
	if ((time - prev_fps_time) > 1000) /* update every seconds */
	{
		int current_fps = frames * 1000 / (time - prev_fps_time);
		printf("%d fps\n", current_fps);
		frames = 0;
		prev_fps_time = time;
	}


	glutPostRedisplay();
}

void mouseFunc(int button, int state, int x, int y){
	switch (button){
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN){
			mouse_x = x;
			mouse_y = y;
			mouse_down = true;
			modelview_matrix_rotpt = modelview_matrix.clone();
		}
		else if (state == GLUT_UP){
			mouse_down = false;
			glGetFloatv(GL_MODELVIEW_MATRIX, current_matrix);
			//modelview_matrix = modelview_matrix_rotpt.clone();
			angle_x = 0;
			angle_y = 0;
		}
		break;
	case 3: //scroll up?
		zoom += ZOOM_VALUE;
		if (state == GLUT_UP) return;
		break;
	case 4: //scroll down?
		if (state == GLUT_UP) return;
		zoom -= ZOOM_VALUE;
		break;
	}
}

void mouseMoveFunc(int x, int y){
	if (mouse_down){
		angle_x = (x - mouse_x) * ROTATE_VALUE;
		angle_y = (y - mouse_y) * ROTATE_VALUE;
		auto_rotate = false;

		cv::Mat rot_tmp = cv::Mat::eye(4,4,CV_32F);
		cv::Rodrigues(cv::Vec3f(angle_y, angle_x, 0), rot_tmp(cv::Range(0,3),cv::Range(0,3)));

		modelview_matrix = rot_tmp * modelview_matrix_rotpt;

	}
}

void keyboardFunc(unsigned char key, int x, int y){
	cv::Mat trans_tmp = cv::Mat::eye(4, 4, CV_32F);
	switch (key){
	case 'K':
	case 'k':
		//show skeleton
		show_skeleton = !show_skeleton;
		break;
	case 'R':
	case 'r':
		//save
		save_playback = !save_playback;
		time = 0;
		file_no = 0;
		break;
	case 'T':
	case 't':
		real_time = !real_time;
		break;
	case 'P':
	case 'p':
		playing = !playing;
		break;
	//case 'A':
	//case 'a':
	//	zoom += ZOOM_VALUE;
	//	break;
	//case 'Z':
	//case 'z':
	//	zoom -= ZOOM_VALUE;
	//	if (zoom < ZOOM_VALUE) zoom = ZOOM_VALUE;
	//	break;
	case 'W':
		trans_tmp.ptr<float>(2)[3] = BIG_ZOOM_VALUE;
		modelview_matrix = trans_tmp * modelview_matrix;
		break;
	case 'w':
		trans_tmp.ptr<float>(2)[3] = ZOOM_VALUE;
		modelview_matrix = trans_tmp * modelview_matrix;
		break;
	case 'S':
		trans_tmp.ptr<float>(2)[3] = -BIG_ZOOM_VALUE;
		modelview_matrix = trans_tmp * modelview_matrix;
		break;
	case 's':
		trans_tmp.ptr<float>(2)[3] = -ZOOM_VALUE;
		modelview_matrix = trans_tmp * modelview_matrix;
		break;
	case 'A':
		trans_tmp.ptr<float>(0)[3] = BIG_ZOOM_VALUE;
		modelview_matrix = trans_tmp * modelview_matrix;
		break;
	case 'a':
		trans_tmp.ptr<float>(0)[3] = ZOOM_VALUE;
		modelview_matrix = trans_tmp * modelview_matrix;
		break;
	case 'D':
		trans_tmp.ptr<float>(0)[3] = -BIG_ZOOM_VALUE;
		modelview_matrix = trans_tmp * modelview_matrix;
		break;
	case 'd':
		trans_tmp.ptr<float>(0)[3] = -ZOOM_VALUE;
		modelview_matrix = trans_tmp * modelview_matrix;
		break;
	case 'Q':
		trans_tmp.ptr<float>(1)[3] = -BIG_ZOOM_VALUE;
		modelview_matrix = trans_tmp * modelview_matrix;
		break;
	case 'q':
		trans_tmp.ptr<float>(1)[3] = -ZOOM_VALUE;
		modelview_matrix = trans_tmp * modelview_matrix;		
		break;
	case 'E':
		trans_tmp.ptr<float>(1)[3] = BIG_ZOOM_VALUE;
		modelview_matrix = trans_tmp * modelview_matrix;
		break;
	case 'e':
		trans_tmp.ptr<float>(1)[3] = ZOOM_VALUE;
		modelview_matrix = trans_tmp * modelview_matrix;
		break;
	}
}


/* ---------------------------------------------------------------------------- */
void display(void)
{
	glutSetWindow(window1);
	current_time = ((glutGet(GLUT_ELAPSED_TIME) + 0.0) / 1000.0);
	elapsed_time = real_time ? current_time - prev_time : FIXED_TIME_INTERVAL;
	prev_time = current_time;

	if (playing)
		time += elapsed_time;

	if (time > maxtime){
		time -= maxtime;
		save_playback = false;
	}
	
	animate(time, scene, scene->mRootNode, aiMatrix4x4(), &gBoneMap, &gBoneInfoVector, gInverseRootTransform);

	float tmp;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//if (auto_rotate){
	//	gluLookAt(0.f, 0.f, 3.f, 0.f, 0.f, -5.f, 0.f, 1.f, 0.f);
	//	/* rotate it around the y axis */
	//	glRotatef(angle, 0.f, 1.f, 0.f);
	//	/* scale the whole asset to fit into our view frustum */
	//	tmp = scene_max.x - scene_min.x;
	//	tmp = aisgl_max(scene_max.y - scene_min.y, tmp);
	//	tmp = aisgl_max(scene_max.z - scene_min.z, tmp);
	//	tmp = 1.f / tmp;
	//	glScalef(tmp, tmp, tmp);
	//	/* center the model */
	//	glTranslatef(-scene_center.x, -scene_center.y, -scene_center.z);
	//
	//	glGetFloatv(GL_MODELVIEW_MATRIX, current_matrix);
	//}
	//else{
	//	glMultMatrixf(current_matrix);
	//
	//	glRotatef(angle_x, 0.f, 1.f, 0.f);
	//	glRotatef(angle_y, 1.f, 0.f, 0.f);
	//
	//	glScalef(zoom, zoom, zoom);
	//
	//}
	cv::Mat modelview_matrix_t = modelview_matrix.t();
	glMultMatrixf(modelview_matrix_t.ptr<float>());

	SkeletonNodeHard snh = hard_skeleton(gSkeletonRoot, cv::Mat::eye(4,4,CV_32F));

	recursive_render(scene, scene->mRootNode, &gMeshMap, &gTextureIDMap, &gVertexBoneVector, &gBoneInfoVector);
	
	if (show_skeleton){
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);
		glColor3b(127, -127, -127);
		draw_skeleton(scene, gSkeletonRoot);
		glEnable(GL_LIGHTING);
		glEnable(GL_DEPTH_TEST);
	}

	cv::Mat depthMat(win_height, win_width, CV_32F);
	glReadPixels(0, 0, win_width, win_height, GL_DEPTH_COMPONENT, GL_FLOAT, depthMat.data);
	cv::Mat depthMatFlipped;
	cv::flip(depth_to_z(depthMat, camera_intrinsic), depthMatFlipped, 0);

	if (save_playback){
		cv::Mat camera_extrinsic(4, 4, CV_32F);
		glGetFloatv(GL_MODELVIEW_MATRIX, (GLfloat*)(camera_extrinsic.data));
		camera_extrinsic = camera_extrinsic.t();

		cv::Mat colorMat(win_height, win_width, CV_8UC3);
		glReadPixels(0, 0, win_width, win_height, GL_BGR_EXT, GL_UNSIGNED_BYTE, colorMat.data);
		cv::Mat colorMatFlipped;
		cv::flip(colorMat, colorMatFlipped, 0);

		std::stringstream filename;


		filename.str("");
		filename<< savepath << file_no << ".xml.gz";

		save_input_frame(filename.str(), time, camera_extrinsic, win_width, win_height, fovy, snh, colorMatFlipped, depthMatFlipped);

		++file_no;

		//cv::imshow("color", colorMatFlipped);
	}

	//cv::imshow("depth", depthMatInverse);


	//cv::waitKey(10);

	glutSwapBuffers();

	do_motion();


}

/* ---------------------------------------------------------------------------- */
int loadasset(const char* path)
{
	/* we are taking one of the postprocessing presets to avoid
	spelling out 20+ single postprocessing flags here. */
	scene = aiImportFile(path, aiProcessPreset_TargetRealtime_MaxQuality);

	if (scene) {
		get_bounding_box(&scene_min, &scene_max);
		scene_center.x = (scene_min.x + scene_max.x) / 2.0f;
		scene_center.y = (scene_min.y + scene_max.y) / 2.0f;
		scene_center.z = (scene_min.z + scene_max.z) / 2.0f;
		return 0;
	}
	return 1;
}

int loadtextures(const aiScene * scene){
	for (int i = 0; i < scene->mNumMaterials; ++i){
		int texIndex = 0;
		aiString path;
		aiReturn texFound = scene->mMaterials[i]->GetTexture(aiTextureType_DIFFUSE, texIndex, &path);

		while (texFound == AI_SUCCESS){
			gTextureIDMap[path.data] = 0;
			++texIndex;
			texFound = scene->mMaterials[i]->GetTexture(aiTextureType_DIFFUSE, texIndex, &path);
		}
	}

	int numTextures = gTextureIDMap.size();

	GLuint * textureIDs = new GLuint[numTextures];
	glGenTextures(numTextures, textureIDs);

	auto it = gTextureIDMap.begin();
	for (int i = 0; it != gTextureIDMap.end(); ++i, ++it){
		std::string filename = it->first;
		it->second = textureIDs[i];

		cv::Mat im = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
		if (!im.empty()){
			glBindTexture(GL_TEXTURE_2D, textureIDs[i]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, im.cols, im.rows, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, im.data);

		}
		else{
			std::cout << "Couldn't load image: " << filename << std::endl;
		}
	}

	delete[] textureIDs;
	return 1;
}


/* ---------------------------------------------------------------------------- */
int main(int argc, char **argv)
{
	if (argc < 3) return 0;
	savepath = argv[2];

	CreateDirectory(savepath.c_str(), NULL);

	aiLogStream stream;

	win_width = 600;
	win_height = 600;

	glutInitWindowSize(win_width, win_height);
	glutInitWindowPosition(100, 100);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInit(&argc, argv);

	window1 = glutCreateWindow("Assimp - Very simple OpenGL sample");
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);

	glutMouseFunc(mouseFunc);
	glutMotionFunc(mouseMoveFunc);
	glutKeyboardUpFunc(keyboardFunc);

	/* get a handle to the predefined STDOUT log stream and attach
	it to the logging system. It remains active for all further
	calls to aiImportFile(Ex) and aiApplyPostProcessing. */
	stream = aiGetPredefinedLogStream(aiDefaultLogStream_STDOUT, NULL);
	aiAttachLogStream(&stream);

	/* ... same procedure, but this stream now writes the
	log messages to assimp_log.txt */
	stream = aiGetPredefinedLogStream(aiDefaultLogStream_FILE, "assimp_log.txt");
	aiAttachLogStream(&stream);

	/* the model name can be specified on the command line. If none
	is specified, we try to locate one of the more expressive test
	models from the repository (/models-nonbsd may be missing in
	some distributions so we need a fallback from /models!). */
	if (0 != loadasset(argc >= 2 ? argv[1] : "../../test/models-nonbsd/X/dwarf.x")) {
		if (argc != 1 || (0 != loadasset("../../../../test/models-nonbsd/X/dwarf.x") && 0 != loadasset("../../test/models/X/Testwuson.X"))) {
			return -1;
		}
	}

	loadtextures(scene);

	gSkeletonRoot = new SkeletonNode();

	{
		//init body part definitions
		cv::Mat head_offset = create_translation_mat(cv::Vec3f( 0, 0, 0.5 ));
		cv::Mat head_offset2 = create_translation_mat(cv::Vec3f( 0, 2, 0.5 ));
		cv::Mat abs_offset = create_translation_mat(cv::Vec3f( 0, -1, 0 ));
		cv::Mat hand_offset = create_translation_mat(cv::Vec3f( 0, 0.5, 0 ));
		cv::Mat foot_offset = create_translation_mat(cv::Vec3f( 0, 0.5, 0 ));
		cv::Mat extend_offset1 = create_translation_mat(cv::Vec3f( 0, -1, 0 ));
		cv::Mat extend_offset2 = create_translation_mat(cv::Vec3f( 0, 1, 0 ));
		cv::Mat extend_offset_small1 = create_translation_mat(cv::Vec3f( 0, -0.5, 0 ));
		cv::Mat extend_offset_small2 = create_translation_mat(cv::Vec3f( 0, 0.5, 0 ));

		gBodyPartDefinitionVector.push_back(BodyPartDefinition("ABS", "hips", "chest", 0.2, 0.2, 0.8, abs_offset));
		gBodyPartDefinitionVector.push_back(BodyPartDefinition("CHEST", "chest", "neck", 0.2, 0.8, 0.2));
		gBodyPartDefinitionVector.push_back(BodyPartDefinition("HEAD", "neck", "head", 0.8, 0.2, 0.2, head_offset, head_offset2));


		gBodyPartDefinitionVector.push_back(BodyPartDefinition("UPPER ARM LEFT", "upper_arm.L", "forearm.L", 0.8, 0.8, 0.2, extend_offset1));
		gBodyPartDefinitionVector.push_back(BodyPartDefinition("UPPER ARM RIGHT", "upper_arm.R", "forearm.R", 0.2, 0.8, 0.8, extend_offset1));
		gBodyPartDefinitionVector.push_back(BodyPartDefinition("LOWER ARM LEFT",  "forearm.L", "hand.L", 0.8, 0.8, 0.5));
		gBodyPartDefinitionVector.push_back(BodyPartDefinition("LOWER ARM RIGHT", "forearm.R", "hand.R", 0.5, 0.8, 0.8));
		
		//gBodyPartDefinitionVector.push_back(BodyPartDefinition("HAND LEFT",  "hand.L", "f_middle.03.L", 0.7, 0.7, 0.1, 0, hand_offset));
		//gBodyPartDefinitionVector.push_back(BodyPartDefinition("HAND RIGHT", "hand.R", "f_middle.03.R", 0.1, 0.7, 0.7, 0, hand_offset));

		gBodyPartDefinitionVector.push_back(BodyPartDefinition("UPPER LEG LEFT",  "thigh.L", "shin.L", 0.5, 0.8, 0.2));
		gBodyPartDefinitionVector.push_back(BodyPartDefinition("UPPER LEG RIGHT", "thigh.R", "shin.R", 0.2, 0.8, 0.5));
		gBodyPartDefinitionVector.push_back(BodyPartDefinition("LOWER LEG LEFT",  "shin.L", "foot.L", 0.8, 0.5, 0.2, extend_offset_small1));
		gBodyPartDefinitionVector.push_back(BodyPartDefinition("LOWER LEG RIGHT", "shin.R", "foot.R", 0.2, 0.5, 0.8, extend_offset_small1));

		//gBodyPartDefinitionVector.push_back(BodyPartDefinition("FOOT LEFT",  "foot.L", "toe.L", 0.9, 0.6, 0.3, 0, foot_offset));
		//gBodyPartDefinitionVector.push_back(BodyPartDefinition("FOOT RIGHT", "foot.R", "toe.R", 0.3, 0.6, 0.9, 0, foot_offset));
	}

	std::stringstream bpdSS;
	bpdSS << savepath << "/bodypartdefinitions.xml.gz";

	cv::FileStorage fs;
	fs.open(bpdSS.str(), cv::FileStorage::WRITE);
	fs << "bodypartdefinitions" << "[";
	for (auto it = gBodyPartDefinitionVector.begin(); it != gBodyPartDefinitionVector.end(); ++it){
		fs << (*it);
	}
	fs << "]";
	fs.release();

	init_node_map(scene, scene->mRootNode, &gNodeMap);
	init_model(scene, gSkeletonRoot, &gNecessityMap, &gNodeMap, &gMeshMap, &gBoneMap, &gBoneInfoVector, &gVertexBoneVector, &gBodyPartDefinitionVector);
	gInverseRootTransform = scene->mRootNode->mTransformation;
	gInverseRootTransform.Inverse();
	if (scene->HasAnimations()){
		maxtime = scene->mAnimations[0]->mDuration / scene->mAnimations[0]->mTicksPerSecond;
	}


	glClearColor(0.1f, 0.1f, 0.1f, 1.f);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);    /* Uses default lighting parameters */

	glEnable(GL_DEPTH_TEST);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glEnable(GL_NORMALIZE);

	/* XXX docs say all polygons are emitted CCW, but tests show that some aren't. */
	if (getenv("MODEL_IS_BROKEN"))
		glFrontFace(GL_CW);

	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);


	modelview_matrix = cv::Mat::eye(4, 4, CV_32F);
	glutGet(GLUT_ELAPSED_TIME);
	glutMainLoop();

	/* cleanup - calling 'aiReleaseImport' is important, as the library
	keeps internal resources until the scene is freed again. Not
	doing so can cause severe resource leaking. */
	aiReleaseImport(scene);

	releaseSkeleton(gSkeletonRoot);

	/* We added a log stream to the library, it's our job to disable it
	again. This will definitely release the last resources allocated
	by Assimp.*/
	aiDetachAllLogStreams();
	return 0;
}
