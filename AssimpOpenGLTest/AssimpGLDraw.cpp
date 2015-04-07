#include "AssimpGLDraw.h"
#include "AssimpGLUtil.h"
#include <GL\glut.h>


/* ---------------------------------------------------------------------------- */
void apply_material(const  aiMaterial *mtl)
{
	float c[4];

	GLenum fill_mode;
	int ret1, ret2;
	aiColor4D diffuse;
	aiColor4D specular;
	aiColor4D ambient;
	aiColor4D emission;
	float shininess, strength;
	int two_sided;
	int wireframe;
	unsigned int max;

	set_float4(c, 0.8f, 0.8f, 0.8f, 1.0f);
	if (AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
		color4_to_float4(&diffuse, c);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, c);

	set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
	if (AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular))
		color4_to_float4(&specular, c);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);

	set_float4(c, 0.2f, 0.2f, 0.2f, 1.0f);
	if (AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient))
		color4_to_float4(&ambient, c);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, c);

	set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
	if (AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission))
		color4_to_float4(&emission, c);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);

	max = 1;
	ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
	if (ret1 == AI_SUCCESS) {
		max = 1;
		ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
		if (ret2 == AI_SUCCESS)
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess * strength);
		else
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
	}
	else {
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
		set_float4(c, 0.0f, 0.0f, 0.0f, 0.0f);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
	}

	max = 1;
	if (AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_ENABLE_WIREFRAME, &wireframe, &max))
		fill_mode = wireframe ? GL_LINE : GL_FILL;
	else
		fill_mode = GL_FILL;
	glPolygonMode(GL_FRONT_AND_BACK, fill_mode);

	max = 1;
	if ((AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED, &two_sided, &max)) && two_sided)
		glDisable(GL_CULL_FACE);
	else
		glEnable(GL_CULL_FACE);
}

/* ---------------------------------------------------------------------------- */
void recursive_render(const  aiScene *sc, const  aiNode* nd, MeshMap * meshMap, TextureIDMap * textureIDMap, std::vector<VertexBoneDataVector> * vertexBoneVector, std::vector<BoneInfo> * boneInfoVector)
{
	unsigned int i;
	unsigned int n = 0, t;
	aiMatrix4x4 m = nd->mTransformation;

	/* update transform */
	aiTransposeMatrix4(&m);
	glPushMatrix();
	glMultMatrixf((float*)&m);

	/* draw all meshes assigned to this node */
	for (; n < nd->mNumMeshes; ++n) {
		const  aiMesh* mesh = sc->mMeshes[nd->mMeshes[n]];

		apply_material(sc->mMaterials[mesh->mMaterialIndex]);

		if (mesh->mNormals == NULL) {
			glDisable(GL_LIGHTING);
		}
		else {
			glEnable(GL_LIGHTING);
			glEnable(GL_NORMALIZE);
		}

		//texture
		aiString texPath;
		if (AI_SUCCESS == sc->mMaterials[mesh->mMaterialIndex]->GetTexture(aiTextureType_DIFFUSE, 0, &texPath)){
			unsigned int texID = (*textureIDMap)[texPath.data];
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, texID);
		}
		else{
			glDisable(GL_TEXTURE_2D);
		}

		for (t = 0; t < mesh->mNumFaces; ++t) {
			const  aiFace* face = &mesh->mFaces[t];
			GLenum face_mode;
			bool depthTest = false;

			switch (face->mNumIndices) {
			case 1: face_mode = GL_POINTS; break;
			case 2: face_mode = GL_LINES; break;
			case 3: face_mode = GL_TRIANGLES; depthTest = true;  break;
			default: face_mode = GL_POLYGON; depthTest = true;  break;
			}

			if (depthTest){
				glEnable(GL_DEPTH_TEST);
			}
			else{
				//actually dont even render it
				continue;

				glDisable(GL_DEPTH_TEST);
			}

			glBegin(face_mode);

			for (i = 0; i < face->mNumIndices; i++) {
				int index = face->mIndices[i];
				if (mesh->mColors[0] != NULL)
					glColor4fv((GLfloat*)&mesh->mColors[0][index]);

				
				

				//apply bone transformation
				aiVector3D meshVertex = mesh->mVertices[index];

				aiMatrix4x4 transform;
				int meshID = (*meshMap)[mesh->mName.C_Str()];
				if ((*vertexBoneVector)[meshID].size() > index)
				{
					if ((*vertexBoneVector)[meshID][index].ID.size() > 0){
						transform.a1 = 0;
						transform.b2 = 0;
						transform.c3 = 0;
					}
					for (int j = 0; j < (*vertexBoneVector)[meshID][index].ID.size(); ++j){
						transform = transform + (*boneInfoVector)[(*vertexBoneVector)[meshID][index].ID[j]].mTransformMatrix * (*vertexBoneVector)[meshID][index].Weight[j];
					}
				}
				if (mesh->mNormals != NULL){
					aiVector3D meshNormal = mesh->mNormals[index];
					aiMatrix4x4 normalTransform = transform;
					normalTransform.Transpose();
					normalTransform.Inverse();
					meshNormal = normalTransform * meshNormal;
					glNormal3fv(&meshNormal.x);
				}
				if (mesh->HasTextureCoords(0)){
					glTexCoord2d(mesh->mTextureCoords[0][index].x, mesh->mTextureCoords[0][index].y);
				}

				
				meshVertex = transform * meshVertex;
				glVertex3fv(&meshVertex.x);

			}

			glEnd();

			glEnable(GL_DEPTH_TEST);
		}

	}

	/* draw all children */
	for (n = 0; n < nd->mNumChildren; ++n) {
		recursive_render(sc, nd->mChildren[n], meshMap, textureIDMap, vertexBoneVector, boneInfoVector);
	}

	glPopMatrix();
}

void draw_skeleton(const aiScene * sc, SkeletonNode * node, aiMatrix4x4 currentTransform){
	aiMatrix4x4 m = node->mNode->mTransformation;
	currentTransform = currentTransform * m;
	node->mTempTransform = currentTransform;

	if (node->mParent){
		//draw to parent

		if (node->mBodyPart.empty){
			glColor3f(1, 0, 0);
		}
		else{
			float node_x = currentTransform.a4;
			float node_y = currentTransform.b4;
			float node_z = currentTransform.c4;

			SkeletonNode * itNode = node;
			aiMatrix4x4 parentTransform;
			while (true){
				itNode = itNode->mParent;
				if (itNode->mNode->mName == node->mBodyPart.mNode1->mName){
					parentTransform = itNode->mTempTransform;
					break;
				}
			}

			float parent_x = parentTransform.a4;
			float parent_y = parentTransform.b4;
			float parent_z = parentTransform.c4;

			glColor3fv(node->mBodyPart.mColor);
			glBegin(GL_LINES);
			glVertex3f(parent_x, parent_y, parent_z);
			glVertex3f(node_x, node_y, node_z);
			glEnd();

			debug_draw_volume(itNode, node);
		}

	}


	for (auto it = node->mChildren.begin(); it != node->mChildren.end(); ++it){

		draw_skeleton(sc, *it, currentTransform);
	}

}

void debug_draw_volume(SkeletonNode * node1, SkeletonNode * node2){

	float X_SIDE_LENGTH = 0.5;
	float Z_SIDE_LENGTH = 0.5;

	aiMatrix4x4 parentTransform;
	parentTransform = node2->mParent->mTempTransform;
	parentTransform.a4 = node1->mTempTransform.a4;
	parentTransform.b4 = node1->mTempTransform.b4;
	parentTransform.c4 = node1->mTempTransform.c4;
	parentTransform.Transpose();

	aiMatrix4x4 transformDiff = node2->mTempTransform - node1->mTempTransform;

	float length = sqrtf(
		transformDiff.a4*transformDiff.a4 +
		transformDiff.b4*transformDiff.b4 +
		transformDiff.c4*transformDiff.c4);

	glPushMatrix();
	glMultMatrixf(&parentTransform.a1);

	glBegin(GL_LINES);

	//parent rect
	glVertex3f(X_SIDE_LENGTH, 0, Z_SIDE_LENGTH);
	glVertex3f(-X_SIDE_LENGTH, 0, Z_SIDE_LENGTH);

	glVertex3f(-X_SIDE_LENGTH, 0, Z_SIDE_LENGTH);
	glVertex3f(-X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH);

	glVertex3f(-X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH);
	glVertex3f(X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH);

	glVertex3f(X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH);
	glVertex3f(X_SIDE_LENGTH, 0, Z_SIDE_LENGTH);

	//child rect
	glVertex3f(X_SIDE_LENGTH, length, Z_SIDE_LENGTH);
	glVertex3f(-X_SIDE_LENGTH, length, Z_SIDE_LENGTH);

	glVertex3f(-X_SIDE_LENGTH, length, Z_SIDE_LENGTH);
	glVertex3f(-X_SIDE_LENGTH, length, -Z_SIDE_LENGTH);

	glVertex3f(-X_SIDE_LENGTH, length, -Z_SIDE_LENGTH);
	glVertex3f(X_SIDE_LENGTH, length, -Z_SIDE_LENGTH);

	glVertex3f(X_SIDE_LENGTH, length, -Z_SIDE_LENGTH);
	glVertex3f(X_SIDE_LENGTH, length, Z_SIDE_LENGTH);

	//sides
	glVertex3f(X_SIDE_LENGTH, 0, Z_SIDE_LENGTH);
	glVertex3f(X_SIDE_LENGTH, length, Z_SIDE_LENGTH);

	glVertex3f(-X_SIDE_LENGTH, 0, Z_SIDE_LENGTH);
	glVertex3f(-X_SIDE_LENGTH, length, Z_SIDE_LENGTH);

	glVertex3f(-X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH);
	glVertex3f(-X_SIDE_LENGTH, length, -Z_SIDE_LENGTH);

	glVertex3f(X_SIDE_LENGTH, 0, -Z_SIDE_LENGTH);
	glVertex3f(X_SIDE_LENGTH, length, -Z_SIDE_LENGTH);


	glEnd();

	glPopMatrix();
}