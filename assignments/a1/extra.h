#ifndef EXTRA_H
#define EXTRA_H

#ifdef WIN32
#include <windows.h>
#endif

#ifdef __APPLE__

#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
/* Just in case we need these later
// References:
// http://alumni.cs.ucsb.edu/~wombatty/tutorials/opengl_mac_osx.html
// # include <OpenGL/gl.h>
// # include <OpenGL/glu.h>
*/
#else
#include <GL/gl.h>
#endif

#include <vecmath.h>

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

// Inline functions to help with drawing
inline void glVertex(const Vector3f &a) {
  glVertex3fv(a.getElements());
}

inline void glNormal(const Vector3f &a) {
  glNormal3fv(a.getElements());
}

inline void glLoadMatrix(const Matrix4f &m) {
  glLoadMatrixf(m.getElements());
}

inline void glMultMatrix(const Matrix4f &m) {
  glMultMatrixf(m.getElements());
}

#endif
