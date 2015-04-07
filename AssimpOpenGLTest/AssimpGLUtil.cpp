#include "AssimpGLUtil.h"

/* ---------------------------------------------------------------------------- */
void color4_to_float4(const aiColor4D *c, float f[4])
{
	f[0] = c->r;
	f[1] = c->g;
	f[2] = c->b;
	f[3] = c->a;
}

/* ---------------------------------------------------------------------------- */
void set_float4(float f[4], float a, float b, float c, float d)
{
	f[0] = a;
	f[1] = b;
	f[2] = c;
	f[3] = d;
}


//overload aiMatrix4x4 add
aiMatrix4x4 operator+(const aiMatrix4x4& left, const aiMatrix4x4& right){
	aiMatrix4x4 sum;
	for (int i = 0; i < 16; ++i){
		(&sum.a1)[i] = (&left.a1)[i] + (&right.a1)[i];
	}
	sum.d4 = 1;
	return sum;
}

aiMatrix4x4 operator-(const aiMatrix4x4& left, const aiMatrix4x4& right){
	aiMatrix4x4 sum;
	for (int i = 0; i < 16; ++i){
		(&sum.a1)[i] = (&left.a1)[i] - (&right.a1)[i];
	}
	sum.d4 = 1;
	return sum;
}

//overload aiMatrix4x4 mutliply
aiMatrix4x4 operator*(const aiMatrix4x4& left, const float right){
	aiMatrix4x4 product;
	for (int i = 0; i < 16; ++i){
		(&product.a1)[i] = (&left.a1)[i] * right;
	}
	product.d4 = 1;
	return product;
}
