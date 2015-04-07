#pragma once

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/color4.h>


/* ---------------------------------------------------------------------------- */
void color4_to_float4(const aiColor4D *c, float f[4]);

/* ---------------------------------------------------------------------------- */
void set_float4(float f[4], float a, float b, float c, float d);


//overload aiMatrix4x4 add
aiMatrix4x4 operator+(const aiMatrix4x4& left, const aiMatrix4x4& right);

aiMatrix4x4 operator-(const aiMatrix4x4& left, const aiMatrix4x4& right);

//overload aiMatrix4x4 mutliply
aiMatrix4x4 operator*(const aiMatrix4x4& left, const float right);
