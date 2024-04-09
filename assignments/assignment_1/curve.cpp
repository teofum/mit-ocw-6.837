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

CurvePoint getBezierPoint(float t, const Vector3f *P, const CurvePoint *last) {
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
  point.V = b[0] * P[0] + b[1] * P[1] + b[2] * P[2] + b[3] * P[3];
  point.T =
      (d[0] * P[0] + d[1] * P[1] + d[2] * P[2] + d[3] * P[3]).normalized();

  if (last == nullptr) {
    // Calculate the Frenet frame for the first point p0
    Vector3f V(0, 0, 1);
    if (approx(Vector3f::dot(V, point.T), 1))
      V = Vector3f(0, 1, 0);

    point.N = Vector3f::cross(V, point.T).normalized();
    point.B = Vector3f::cross(point.T, point.N);
  } else {
    // Approximate RMF by the double reflection method
    // Computation of Rotation Minimizing Frames, Wang, W. et al., 2008
    Vector3f v1 = point.V - last->V; // Reflection vector 1
    float c1 = v1.absSquared();
    Vector3f Nl = last->N - (2.0f / c1) * Vector3f::dot(v1, last->N) * v1;
    Vector3f Tl = last->T - (2.0f / c1) * Vector3f::dot(v1, last->T) * v1;

    Vector3f v2 = point.T - Tl; // Reflection vector 2
    float c2 = v2.absSquared();
    point.N = Nl - (2.0f / c2) * Vector3f::dot(v2, Nl) * v2;
    point.B = Vector3f::cross(point.T, point.N);
  }

  return point;
}

bool isClosed(const Curve &curve) {
  unsigned last = curve.size() - 1;
  return approx((curve[0].V - curve[last].V).absSquared(), 0) &&
         approx(Vector3f::dot(curve[0].T, curve[last].T), 1);
}

void fixClosedCurveError(Curve &curve) {
  if (!isClosed(curve))
    return;

  unsigned last = curve.size() - 1;
  float cosTheta = Vector3f::dot(curve[0].N, curve[last].N);
  if (approx(cosTheta, 1))
    return; // Normals match at beginning and end

  // Because the tangents match, normals and bitangents are all coplanar, so
  // we can get the sine this way
  float sinTheta = Vector3f::dot(curve[0].N, curve[last].B);
  // Then use sin, cos to get angle in the axis aligned with the tangent
  float theta = atan2(sinTheta, cosTheta);
  if (theta > M_PI)
    theta -= 2.0 * M_PI;

  // Spread the error across the entire curve, this is like "twisting" the curve
  for (unsigned i = last; i > 0; i--) {
    float t = (float)i / last;

    float c = cos(t * theta), s = sin(t * theta);
    float x = curve[i].T.x(), y = curve[i].T.y(), z = curve[i].T.z();

    // Rotate (t * theta) radians along the axis of the tangent of curve[i]
    Matrix3f rotation(
        c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s,
        y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s,
        z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)
    );

    curve[i].N = rotation * curve[i].N;
    curve[i].B = rotation * curve[i].B;
  }
}

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
    const Vector3f *firstPoint = &P[s * 3];
    unsigned k = s == segments - 1 ? steps : steps - 1;

    for (unsigned j = 0; j <= k; j++) {
      float t = (float)j / steps;

      CurvePoint *last = (s + j == 0) ? nullptr : &curve[s * steps + j - 1];
      CurvePoint point = getBezierPoint(t, firstPoint, last);
      curve.push_back(point);
    }
  }

  fixClosedCurveError(curve);

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
    P_bezier.push_back((1.0 / 6.0) * (4 * P[i - 2] + 2 * P[i - 1]));
    P_bezier.push_back((1.0 / 6.0) * (2 * P[i - 2] + 4 * P[i - 1]));
    P_bezier.push_back((1.0 / 6.0) * (P[i - 2] + 4 * P[i - 1] + P[i]));
  }

  return evalBezier(P_bezier, steps);
}

Curve evalCatmullRom(const std::vector<Vector3f> &P, unsigned steps, float r) {
  // Check
  if (P.size() < 4) {
    cerr << "evalCatmullRom must be called with 4 or more control points."
         << endl;
    exit(0);
  }

  vector<Vector3f> P_bezier;
  P_bezier.reserve(4 + 3 * (P.size() - 4));

  // First four points can be converted directly
  // "Unrolled" basis conversion, easier than building a matrix out of all the
  // vectors
  P_bezier.push_back(P[1]);
  P_bezier.push_back((-r / 3) * P[0] + P[1] + (r / 3) * P[2]);
  P_bezier.push_back((r / 3) * P[1] + P[2] + (-r / 3) * P[3]);
  P_bezier.push_back(P[2]);

  // After that, for each point P[i], i >= 4 we need to push an entire segment
  // of the last four points P[i-3]..P[i]
  for (unsigned i = 4; i < P.size(); i++) {
    P_bezier.push_back((-r / 3) * P[i - 3] + P[i - 2] + (r / 3) * P[i - 1]);
    P_bezier.push_back((r / 3) * P[i - 2] + P[i - 1] + (-r / 3) * P[i]);
    P_bezier.push_back(P[i - 1]);
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

Curve evalTrefoil(float radius, unsigned steps) {
  Curve curve(steps + 1);

  for (unsigned i = 0; i <= steps; i++) {
    float t = 2.0f * M_PI * float(i) / steps;

    // Position, trefoil knot is a parametric curve in 3D
    // See https://en.wikipedia.org/wiki/Trefoil_knot
    float x = sin(t) + 2 * sin(2 * t);
    float y = cos(t) - 2 * cos(2 * t);
    float z = -sin(3 * t);
    curve[i].V = Vector3f(x, y, z) * radius;

    // Tangent is the derivative
    float x1 = cos(t) + 4 * cos(2 * t);
    float y1 = -sin(t) + 4 * sin(2 * t);
    float z1 = -3 * cos(3 * t);
    curve[i].T = Vector3f(x1, y1, z1).normalized();

    // Normal is the second derivative
    float x2 = -sin(t) - 8 * sin(2 * t);
    float y2 = -cos(t) + 8 * cos(2 * t);
    float z2 = 9 * sin(3 * t);
    curve[i].N = Vector3f(x2, y2, z2).normalized();

    // Bitangent is T x N
    curve[i].B = Vector3f::cross(curve[i].T, curve[i].N);
  }

  return curve;
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
