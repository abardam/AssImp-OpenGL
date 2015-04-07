#include "AssimpGLSkin.h"
#include "AssimpGLUtil.h"



void animate(double time, const  aiScene *sc, aiNode * node, aiMatrix4x4 parentTransform, BoneMap * boneMap, std::vector<BoneInfo> * boneInfoVector, const aiMatrix4x4& inverseRootTransform){


	int numChannels = sc->mAnimations[0]->mNumChannels;
	for (int i = 0; i < numChannels; ++i){
		aiString nodeName = sc->mAnimations[0]->mChannels[i]->mNodeName;
		if (nodeName != node->mName) continue;

		aiQuatKey * rotationKey = &sc->mAnimations[0]->mChannels[i]->mRotationKeys[0];
		for (int frame = 0; frame < sc->mAnimations[0]->mChannels[i]->mNumRotationKeys; ++frame){
			if (sc->mAnimations[0]->mChannels[i]->mRotationKeys[frame].mTime < time)
				rotationKey = &sc->mAnimations[0]->mChannels[i]->mRotationKeys[frame];
			else break;
		}

		aiVectorKey * positionKey = &sc->mAnimations[0]->mChannels[i]->mPositionKeys[0];
		for (int frame = 0; frame < sc->mAnimations[0]->mChannels[i]->mNumPositionKeys; ++frame){
			if (sc->mAnimations[0]->mChannels[i]->mPositionKeys[frame].mTime < time)
				positionKey = &sc->mAnimations[0]->mChannels[i]->mPositionKeys[frame];
			else break;
		}

		aiVectorKey * scalingKey = &sc->mAnimations[0]->mChannels[i]->mScalingKeys[0];
		for (int frame = 0; frame < sc->mAnimations[0]->mChannels[i]->mNumScalingKeys; ++frame){
			if (sc->mAnimations[0]->mChannels[i]->mScalingKeys[frame].mTime < time)
				scalingKey = &sc->mAnimations[0]->mChannels[i]->mScalingKeys[frame];
			else break;
		}


		aiMatrix4x4 aiRotationMat = aiMatrix4x4(rotationKey->mValue.GetMatrix());

		aiMatrix4x4 aiPositionMat;
		if (positionKey) aiMatrix4x4::Translation(positionKey->mValue, aiPositionMat);

		aiMatrix4x4 aiScalingMat;
		//if (scalingKey) aiMatrix4x4::Scaling(scalingKey->mValue, aiScalingMat);

		node->mTransformation = aiPositionMat * aiRotationMat * aiScalingMat;

		auto it = boneMap->find(node->mName.C_Str());
		if (it != boneMap->end()){
			(*boneInfoVector)[it->second].mTransformMatrix = inverseRootTransform * parentTransform * node->mTransformation * (*boneInfoVector)[it->second].mOffsetMatrix;
			//gBoneInfoVector[it->second].mTransformMatrix = aiMatrix4x4();
		}
	}

	for (int i = 0; i < node->mNumChildren; ++i){
		animate(time, sc, node->mChildren[i], parentTransform * node->mTransformation, boneMap, boneInfoVector, inverseRootTransform);
	}
}


void load_bones_from_mesh(const aiMesh * mesh, BoneMap * boneMap, std::vector<BoneInfo> * boneInfoVector, std::vector<VertexBoneData> * vertexBoneVector){

	for (int i = 0; i < mesh->mNumBones; ++i){
		int boneID;
		aiBone * bone = mesh->mBones[i];
		auto it = boneMap->find(bone->mName.C_Str());
		if (it == boneMap->end()){
			boneID = boneMap->size();
			boneMap->insert(BoneEntry(bone->mName.C_Str(), boneID));
		}
		else{
			boneID = it->second;
		}

		if (boneInfoVector->size() <= boneID){
			boneInfoVector->resize(boneID + 1);
		}
		(*boneInfoVector)[boneID].mOffsetMatrix = bone->mOffsetMatrix;

		for (int j = 0; j < bone->mNumWeights; ++j){
			int vertexID = bone->mWeights[j].mVertexId;
			if (vertexBoneVector->size() <= vertexID){
				vertexBoneVector->resize(vertexID + 1);
			}

			(*vertexBoneVector)[vertexID].ID.push_back(boneID);
			(*vertexBoneVector)[vertexID].Weight.push_back(bone->mWeights[j].mWeight);

		}
	}
}

void init_model(
	const aiScene * sc, 
	SkeletonNode * sRoot, 
	NecessityMap * nmap, 
	NodeMap * nodeMap, 
	MeshMap * meshMap, 
	BoneMap * boneMap, 
	std::vector<BoneInfo> * boneInfoVector, 
	std::vector<VertexBoneDataVector> * vertexBoneVector,
	BodyPartDefinitionVector * bodyPartDefinitionVector){
	for (int i = 0; i < sc->mNumMeshes; ++i){
		for (int j = 0; j < sc->mMeshes[i]->mNumBones; ++j){

			std::string boneName = sc->mMeshes[i]->mBones[j]->mName.C_Str();
			aiNode * node = (*nodeMap)[boneName];

			necessityParent(node, nmap, sc->mMeshes[i]->mName.C_Str(), (*nodeMap)[sc->mMeshes[i]->mName.C_Str()]->mName.C_Str());
		}

		auto it = meshMap->find(sc->mMeshes[i]->mName.C_Str());
		int meshID;
		if (it == meshMap->end()){
			meshID = i;
			meshMap->insert(MeshEntry(sc->mMeshes[i]->mName.C_Str(), meshID));
		}
		else{
			meshID = it->second;
		}

		if (vertexBoneVector->size() <= meshID){
			vertexBoneVector->resize(meshID + 1);
		}

		load_bones_from_mesh(sc->mMeshes[i], boneMap, boneInfoVector, &(*vertexBoneVector)[meshID]);
	}

	BodyPartVector bpv;

	build_body_parts(nodeMap, bodyPartDefinitionVector, &bpv);
	build_skeleton(sc, sc->mRootNode, sRoot, nmap, &bpv);
}


void init_node_map(const aiScene * sc, aiNode * node, NodeMap * nodeMap){
	nodeMap->insert(NodeEntry(node->mName.C_Str(), node));

	for (int i = 0; i < node->mNumChildren; ++i){
		init_node_map(sc, node->mChildren[i], nodeMap);
	}
}
