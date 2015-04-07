#pragma once
#include <map>
#include <vector>
#include <assimp/scene.h>

typedef std::map<std::string, aiNode*> NodeMap;
typedef std::pair<std::string, aiNode*> NodeEntry;

typedef std::map<std::string, bool> NecessityMap;
typedef std::pair<std::string, bool> NecessityEntry;

typedef std::map<std::string, unsigned int> BoneMap;
typedef std::pair<std::string, unsigned int> BoneEntry;

struct BodyPart{
	std::string mName;
	aiNode * mNode1;
	aiNode * mNode2;
	float mColor[3];
	bool empty;

	BodyPart() :empty(true){}

	BodyPart(aiNode * n1, aiNode * n2) :
		mNode1(n1), mNode2(n2), empty(false){}
};

struct SkeletonNode{
	SkeletonNode * mParent;
	std::vector<SkeletonNode*> mChildren;
	BodyPart mBodyPart; //whether this node forms a body part with an ancestor
	aiMatrix4x4 mTempTransform; //temporary; changes every frame
	aiNode * mNode;

	SkeletonNode():
		mParent(NULL),
		mNode(NULL){}
};

struct BoneInfo{
	aiMatrix4x4 mTransformMatrix;
	aiMatrix4x4 mOffsetMatrix;
};


struct BodyPartDefinition{
	std::string mNode1Name;
	std::string mNode2Name;

	float mNode1Offset[3];
	float mNode2Offset[3];

	std::string mBodyPartName;
	float mColor[3];
	
	BodyPartDefinition(std::string name, std::string n1, std::string n2, float * color, float * n1o=0, float * n2o=0) :
		mBodyPartName(name),
		mNode1Name(n1),
		mNode2Name(n2){
		mColor[0] = color[0];
		mColor[1] = color[1];
		mColor[2] = color[2];

		if (n1o == 0){
			mNode1Offset[0] = 0;
			mNode1Offset[1] = 0;
			mNode1Offset[2] = 0;
		}
		else{
			mNode1Offset[0] = n1o[0];
			mNode1Offset[1] = n1o[1];
			mNode1Offset[2] = n1o[2];
		}

		if (n2o == 0){
			mNode2Offset[0] = 0;
			mNode2Offset[1] = 0;
			mNode2Offset[2] = 0;
		}
		else{
			mNode2Offset[0] = n2o[0];
			mNode2Offset[1] = n2o[1];
			mNode2Offset[2] = n2o[2];
		}
	}

	BodyPartDefinition(std::string name, std::string n1, std::string n2, float r, float g, float b, float * n1o=0, float * n2o=0) :
		mBodyPartName(name),
		mNode1Name(n1),
		mNode2Name(n2){
		mColor[0] = r;
		mColor[1] = g;
		mColor[2] = b;

		if (n1o == 0){
			mNode1Offset[0] = 0;
			mNode1Offset[1] = 0;
			mNode1Offset[2] = 0;
		}
		else{
			mNode1Offset[0] = n1o[0];
			mNode1Offset[1] = n1o[1];
			mNode1Offset[2] = n1o[2];
		}

		if (n2o == 0){
			mNode2Offset[0] = 0;
			mNode2Offset[1] = 0;
			mNode2Offset[2] = 0;
		}
		else{
			mNode2Offset[0] = n2o[0];
			mNode2Offset[1] = n2o[1];
			mNode2Offset[2] = n2o[2];
		}
	}

	BodyPartDefinition(std::string name, std::string n1, std::string n2) :
		mBodyPartName(name),
		mNode1Name(n1),
		mNode2Name(n2){
		mColor[0] = 1;
		mColor[1] = 1;
		mColor[2] = 1;

		mNode1Offset[0] = 0;
		mNode1Offset[1] = 0;
		mNode1Offset[2] = 0;
		mNode2Offset[0] = 0;
		mNode2Offset[1] = 0;
		mNode2Offset[2] = 0;
	}

	BodyPartDefinition(){
		mColor[0] = 1;
		mColor[1] = 1;
		mColor[2] = 1;

		mNode1Offset[0] = 0;
		mNode1Offset[1] = 0;
		mNode1Offset[2] = 0;
		mNode2Offset[0] = 0;
		mNode2Offset[1] = 0;
		mNode2Offset[2] = 0;
	}
};

typedef std::vector<BodyPart> BodyPartVector;
typedef std::vector<BodyPartDefinition> BodyPartDefinitionVector;

void necessityParent(aiNode * node, NecessityMap * nmap, std::string target, std::string target2);

void build_skeleton(const aiScene * sc, aiNode * node, SkeletonNode * sNode, NecessityMap * nmap, BodyPartVector * bpv);
void build_body_parts(NodeMap * nodeMap, BodyPartDefinitionVector * bpdv, BodyPartVector * bpv);