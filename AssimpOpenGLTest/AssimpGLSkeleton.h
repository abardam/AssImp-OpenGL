#pragma once
#include <map>
#include <vector>
#include <assimp/scene.h>
#include <cv_skeleton.h>

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


typedef std::vector<BodyPart> BodyPartVector;

void necessityParent(aiNode * node, NecessityMap * nmap, std::string target, std::string target2);

void build_skeleton(const aiScene * sc, aiNode * node, SkeletonNode * sNode, NecessityMap * nmap, BodyPartVector * bpv);
void build_body_parts(NodeMap * nodeMap, BodyPartDefinitionVector * bpdv, BodyPartVector * bpv);