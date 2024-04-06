#ifdef __APPLE__
#include <OpenGL/gl.h>
/* Just in case we need these later
// References:
// http://alumni.cs.ucsb.edu/~wombatty/tutorials/opengl_mac_osx.html
// # include <OpenGL/gl.h>
// # include <OpenGL/glu.h>
*/
#else
#include <GL/gl.h>
#endif

#include "curve.h"
#include "extra.h"
#ifdef WIN32
#include <windows.h>
#endif

using namespace std;

namespace {
// Approximately equal to.  We don't want to use == because of
// precision issues with floating point.
inline bool approx(const Vector3f &lhs, const Vector3f &rhs) {
  const float eps = 1e-8f;
  return (lhs - rhs).absSquared() < eps;
}

// Spline matrix for bezier curves (Bernstein polynomials)
Matrix4f bezier(1, -3, 3, -1, 0, 3, -6, 3, 0, 0, 3, -3, 0, 0, 0, 1);
// Derivatives of Bernstein polynomials
Matrix4f bezier_d(-3, 6, -3, 0, 3, -12, 9, 0, 0, 6, -9, 0, 0, 0, 3, 0);

} // namespace

Curve evalBezier(const vector<Vector3f> &P, unsigned steps) {
  // Check
  if (P.size() < 4 || P.size() % 3 != 1) {
    cerr << "evalBezier must be called with 3n+1 control points." << endl;
    exit(0);
  }

  unsigned segments = (P.size() - 1) / 3;       // # of curve segments
  unsigned points = (steps - 1) * segments + 2; // # of points in approximation
  Curve curve;
  curve.reserve(points); // pre-allocate

  cerr << segments << " segments, " << points << " points" << endl;

  for (unsigned s = 0; s < segments; s++) {
    int i = s * 3;
    Vector3f p0 = P[i], p1 = P[i + 1], p2 = P[i + 2], p3 = P[i + 3];

    Vector3f lastB(0, 0, 1); // Arbitrary starting bitangent
    for (unsigned j = 0; j <= (s == segments - 1 ? steps : steps - 1); j++) {
      float t = (float)j / steps;
      float t2 = t * t;
      float t3 = t2 * t;
      Vector4f basis(1, t, t2, t3);

      // Coefficients for V = q(t) (Bernstein polynomials)
      Vector4f b = bezier * basis;

      // Coefficients for T = q'(t) (derivatives of Bernstein polynomials)
      Vector4f d = bezier_d * basis;

      // Calculate point and tangent vector
      // We do this part explicitly because I don't want to write a 3x4 matrix
      // class just for this. Lazy!
      CurvePoint point;
      point.V = b[0] * p0 + b[1] * p1 + b[2] * p2 + b[3] * p3;
      point.T = (d[0] * p0 + d[1] * p1 + d[2] * p2 + d[3] * p3).normalized();

      // Calculate normal and bitangent vector
      // If "seed" vector is parallel to normal change it to perpendicular
      if (j == 0 && approx(Vector3f::dot(lastB, point.T), 1.0))
        lastB = Vector3f(0, 1, 0);

      point.N = Vector3f::cross(lastB, point.T).normalized();
      lastB = point.B = Vector3f::cross(point.T, point.N).normalized();

      // Add point to the curve
      curve.push_back(point);
    }
  }

  cerr << curve.size() << " actual points" << endl;

  return curve;
}

Curve evalBspline(const vector<Vector3f> &P, unsigned steps) {
  // Check
  if (P.size() < 4) {
    cerr << "evalBspline must be called with 4 or more control points." << endl;
    exit(0);
  }

  vector<Vector3f> P_bezier;
  P_bezier.reserve(4 + 3 * (P.size() - 4));

  // First four points can be converted directly
  // "Unrolled" basis conversion, easier than building a matrix out of all the
  // vectors
  P_bezier.push_back((1.0 / 6.0) * (P[0] + 4 * P[1] + P[2]));
  P_bezier.push_back((1.0 / 6.0) * (4 * P[1] + 2 * P[2]));
  P_bezier.push_back((1.0 / 6.0) * (2 * P[1] + 4 * P[2]));
  P_bezier.push_back((1.0 / 6.0) * (P[1] + 4 * P[2] + P[3]));

  // After that, for each point P[i], i >= 4 we need to push an entire segment
  // of the last four points P[i-3]..P[i]
  for (unsigned i = 4; i < P.size(); i++) {
    P_bezier.push_back((1.0 / 6.0) * (P[i - 3] + 4 * P[i - 2] + P[i - 1]));
    P_bezier.push_back((1.0 / 6.0) * (4 * P[i - 2] + 2 * P[i - 1]));
    P_bezier.push_back((1.0 / 6.0) * (2 * P[i - 2] + 4 * P[i - 1]));
    P_bezier.push_back((1.0 / 6.0) * (P[i - 2] + 4 * P[i - 1] + P[i]));
  }

  return evalBezier(P_bezier, steps);
}

Curve evalCircle(float radius, unsigned steps) {
  // This is a sample function on how to properly initialize a Curve
  // (which is a vector< CurvePoint >).

  // Preallocate a curve with steps+1 CurvePoints
  Curve R(steps + 1);

  // Fill it in counterclockwise
  for (unsigned i = 0; i <= steps; ++i) {
    // step from 0 to 2pi
    float t = 2.0f * M_PI * float(i) / steps;

    // Initialize position
    // We're pivoting counterclockwise around the y-axis
    R[i].V = radius * Vector3f(cos(t), sin(t), 0);

    // Tangent vector is first derivative
    R[i].T = Vector3f(-sin(t), cos(t), 0);

    // Normal vector is second derivative
    R[i].N = Vector3f(-cos(t), -sin(t), 0);

    // Finally, binormal is facing up.
    R[i].B = Vector3f(0, 0, 1);
  }

  return R;
}

void drawCurve(const Curve &curve, float framesize) {
  // Save current state of OpenGL
  glPushAttrib(GL_ALL_ATTRIB_BITS);

  // Setup for line drawing
  glDisable(GL_LIGHTING);
  glColor4f(1, 1, 1, 1);
  glLineWidth(1);

  // Draw curve
  glBegin(GL_LINE_STRIP);
  for (unsigned i = 0; i < curve.size(); ++i) {
    glVertex(curve[i].V);
  }
  glEnd();

  glLineWidth(1);

  // Draw coordinate frames if framesize nonzero
  if (framesize != 0.0f) {
    Matrix4f M;

    for (unsigned i = 0; i < curve.size(); ++i) {
      M.setCol(0, Vector4f(curve[i].N, 0));
      M.setCol(1, Vector4f(curve[i].B, 0));
      M.setCol(2, Vector4f(curve[i].T, 0));
      M.setCol(3, Vector4f(curve[i].V, 1));

      glPushMatrix();
      glMultMatrixf(M.getElements());
      glScaled(framesize, framesize, framesize);
      glBegin(GL_LINES);
      glColor3f(1, 0, 0);
      glVertex3d(0, 0, 0);
      glVertex3d(1, 0, 0);
      glColor3f(0, 1, 0);
      glVertex3d(0, 0, 0);
      glVertex3d(0, 1, 0);
      glColor3f(0, 0, 1);
      glVertex3d(0, 0, 0);
      glVertex3d(0, 0, 1);
      glEnd();
      glPopMatrix();
    }
  }

  // Pop state
  glPopAttrib();
}
