#ifndef LINEAR_INLINE_H
#define LINEAR_INLINE_H

#ifdef __cplusplus
extern "C" {
#endif

#define INLINE 

INLINE
void
m4_mul(mat4_t *r, const mat4_t *a, const mat4_t *b);


INLINE
void
m4_mul_mi(mat4_t *r, const mat4_t *a,
          double b00, double b01, double b02, double b03,
          double b10, double b11, double b12, double b13,
          double b20, double b21, double b22, double b23,
          double b30, double b31, double b32, double b33);
INLINE
void
m4_rotate(mat4_t *m, const mat4_t *t, double a, double x, double y, double z);

INLINE
void
m4_translate(mat4_t *r, const mat4_t *t, double x, double y, double z);

/*
 * 4x4 Matrix and homogeneous vector mulitplication.  The input
 * coordinates are specified in immediate parameters (not wrapped in a
 * vector).
 */
INLINE
void
m4_transform_i(vec4_t *r, const mat4_t *m, double x, double y, double z, double w);

/*
 * Like m4_transform_i, with only x,y, and z specified.  w is set to
 * 1.0, and used to perform a homogeneous divide to convert to a 3-D
 * vector.
 */
INLINE
void
m4_transform_i3(vec3_t *r, const mat4_t *m, double x, double y, double z);

INLINE
void
m4_transform_v(vec4_t *r, const mat4_t *m, const vec4_t *v);

INLINE
vec3_t *
v3_sub(vec3_t *r, const vec3_t *a, const vec3_t *b);

INLINE
void
v3_add(vec3_t *r, const vec3_t *a, const vec3_t *b);

INLINE
void
v3_scale(vec3_t *r, const vec3_t *v, double s);

INLINE
double
v3_dot(const vec3_t *a, const vec3_t *b);

INLINE
double
v3_len2(const vec3_t *v);

INLINE
double
v3_len(const vec3_t *v);

INLINE
double
v3_dist2(const vec3_t *a, const vec3_t *b);

INLINE
double
v3_dist(const vec3_t *a, const vec3_t *b);

INLINE
void
v3_norm(vec3_t *r, const vec3_t *a);

INLINE
vec2_t *
v2_sub(vec2_t *r, const vec2_t *a, const vec2_t *b);

INLINE
double
v2_dot(const vec2_t *a, const vec2_t *b);

INLINE
double
v2_len2(const vec2_t *a);

INLINE
double
v2_dist2(const vec2_t *a, const vec2_t *b);

INLINE
double
v2_dist(const vec2_t *a, const vec2_t *b);

INLINE
void
m4_extract_translation(vec4_t *r, const mat4_t *m);

#undef INLINE

#ifdef __cplusplus
}
#endif

#endif
