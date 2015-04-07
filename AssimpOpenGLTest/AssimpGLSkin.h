#pragma once

#include "AssimpGLSkeleton.h"
#include <assimp/scene.h>
#include <vector>
#include <map>

typedef std::map<std::string, unsigned int> MeshMap;
typedef std::pair<std::string, unsigned int> MeshEntry;

typedef std::map<std::string, int> TextureIDMap;
typedef std::pair<std::string, int> TextureIDEntry;


struct VertexBoneData{
	std::vector<unsigned int> ID;
	std::vector<float> Weight;
};

typedef std::vector <VertexBoneData>  VertexBoneDataVector;

void animate(double time, const  aiScene *sc, aiNode * node, aiMatrix4x4 parentTransform, BoneMap * boneMap, std::vector<BoneInfo> * boneInfoVector, const aiMatrix4x4& inverseRootTransform);

void init_model(const aiScene * sc,
	SkeletonNode * sRoot, 
	NecessityMap * nmap, 
	NodeMap * nodeMap,
	MeshMap * meshMap, 
	BoneMap * boneMap,
	std::vector<BoneInfo> * boneInfoVector,
	std::vector<VertexBoneDataVector> * vertexBoneVector,
	BodyPartDefinitionVector * bodyPartDefinitionVector);

void load_bones_from_mesh(const aiMesh * mesh, BoneMap * boneMap, std::vector<BoneInfo> * boneInfoVector, std::vector<VertexBoneData> * vertexBoneVector);

void init_node_map(const aiScene * sc, aiNode * node, NodeMap * nodeMap);
