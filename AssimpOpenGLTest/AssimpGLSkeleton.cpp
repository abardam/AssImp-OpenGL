#include "AssimpGLSkeleton.h"



void necessityParent(aiNode * node, NecessityMap * nmap, std::string target, std::string target2){
	nmap->insert(NecessityEntry(node->mName.C_Str(), true));
	if (node->mName.C_Str() != target &&
		node->mName.C_Str() != target2 &&
		node->mParent){
		necessityParent(node->mParent, nmap, target, target2);
	}
}

void build_skeleton(const aiScene * sc, aiNode * node, SkeletonNode * sNode, NecessityMap * nmap, BodyPartVector * bpv){
	sNode->mNode = node;

	for (auto it = bpv->begin(); it != bpv->end(); ++it){
		if (it->mNode2->mName == node->mName){
			sNode->mBodyPart = *it;
			break;
		}
	}

#if 1 //for debug
	for (int i = 0; i < node->mNumChildren; ++i){
		if ((*nmap)[node->mChildren[i]->mName.C_Str()]){
			printf("%s---%s\n", node->mName.C_Str(), node->mChildren[i]->mName.C_Str());
		}
	}
#endif

	for (int i = 0; i < node->mNumChildren; ++i){
		if ((*nmap)[node->mChildren[i]->mName.C_Str()]){
			SkeletonNode * child = new SkeletonNode;
			child->mParent = sNode;
			sNode->mChildren.push_back(child);

			build_skeleton(sc, node->mChildren[i], child, nmap, bpv);

		}

	}
}

void build_body_parts(NodeMap * nodeMap, BodyPartDefinitionVector * bpdv, BodyPartVector * bpv){
	for (auto it = bpdv->begin(); it != bpdv->end(); ++it){
		BodyPart bp(nodeMap->find(it->mNode1Name)->second, nodeMap->find(it->mNode2Name)->second);

		bp.mName = it->mBodyPartName;
		bp.mColor[0] = it->mColor[0];
		bp.mColor[1] = it->mColor[1];
		bp.mColor[2] = it->mColor[2];

		bpv->push_back(bp);
	}
}