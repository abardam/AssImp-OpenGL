#pragma once

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/color4.h>
#include "AssimpGLSkin.h"

#include <stack>


void recursive_render(const  aiScene *sc, const  aiNode* nd, MeshMap * meshMap, TextureIDMap * textureIDMap, std::vector<VertexBoneDataVector> * vertexBoneVector, std::vector<BoneInfo> * boneInfoVector);

void draw_skeleton(const aiScene * sc, SkeletonNode * node, aiMatrix4x4 currentTransform = aiMatrix4x4());
void debug_draw_volume(SkeletonNode * node1, SkeletonNode * node2);