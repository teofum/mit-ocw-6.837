#include "surf.h"
#include "extra.h"

using namespace std;

#define RAD(deg) ((deg) * M_PI / 180)

namespace {
// Approximately equal to.  We don't want to use == because of
// precision issues with floating point.
inline bool approx(float lhs, float rhs) {
  const float eps = 1e-8f;
  return abs(lhs - rhs) < eps;
}

// Spline matrix for bezier curves (Bernstein polynomials)
Matrix4f bezier(1, -3, 3, -1, 0, 3, -6, 3, 0, 0, 3, -3, 0, 0, 0, 1);
// Derivatives of Bernstein polynomials
Matrix4f bezier_d(-3, 6, -3, 0, 3, -12, 9, 0, 0, 6, -9, 0, 0, 0, 3, 0);

// We're only implenting swept surfaces where the profile curve is
// flat on the xy-plane.  This is a check function.
bool checkFlat(const Curve &profile) {
  for (auto point: profile)
    if (!approx(point.V[2], 0) || !approx(point.T[2], 0) ||
        !approx(point.N[2], 0))
      return false;

  return true;
}

void createFaces(
  Surface &surface,
  unsigned profileLength,
  unsigned steps,
  bool loop = true
) {
  surface.VF.reserve((profileLength - 1) * steps * 2);

  unsigned last = loop ? steps + 1 : steps;
  for (unsigned i = 1; i < last; i++) {
    unsigned iLastProfile = (i - 1) * profileLength;
    unsigned iProfile = (i % steps) * profileLength;

    // Create a quad (two triangles)
    for (unsigned j = 1; j < profileLength; j++) {
      unsigned v00 = iLastProfile + j - 1, v01 = iLastProfile + j,
        v10 = iProfile + j - 1, v11 = iProfile + j;

      surface.VF.emplace_back(v00, v11, v10);
      surface.VF.emplace_back(v00, v01, v11);
    }
  }
}

constexpr float cornerThreshold = RAD(30);

bool isCorner(const CurvePoint &point, const CurvePoint *last) {
  return Vector3f::dot(point.T, last->T) < cos(cornerThreshold);
}

void appendSegment(
  Surface &surface,
  const Curve &profile,
  const Vector3f &pos,
  const CurvePoint &frame,
  const Vector2f &pScale
) {
  Matrix4f transform = Matrix4f(
    Vector4f(frame.N, 0), Vector4f(frame.B, 0), Vector4f(frame.T, 0),
    Vector4f(pos, 1), true
  );

  Matrix4f scale = Matrix4f::scaling(pScale.x(), pScale.y(), 1.0f);

  for (auto &point: profile) {
    // Move the point and rotate it to match the sweep coordinate frame
    Vector3f V = (transform * scale * Vector4f(point.V, 1)).xyz();

    // Rotate the normal to match sweep coordinate frame
    // We can use the 3x3 matrix, and because NTB is an othonormal basis,
    // once again the transpose of the inverse is the same matrix
    Vector3f N = transform.getSubmatrix3x3(0, 0) * -point.N;

    surface.VV.push_back(V);
    surface.VN.push_back(N);
  }
}

void appendCorner(
  Surface &surface,
  const Curve &profile,
  const CurvePoint &frame,
  const CurvePoint &last,
  const Vector2f &pScale
) {
  Matrix4f transform = Matrix4f(
    Vector4f(frame.N, 0), Vector4f(frame.B, 0), Vector4f(frame.T, 0),
    Vector4f(frame.V, 1), true
  );
  Matrix4f transformLast = Matrix4f(
    Vector4f(last.N, 0), Vector4f(last.B, 0), Vector4f(last.T, 0),
    Vector4f(frame.V, 1), true
  );

  Matrix4f scale = Matrix4f::scaling(pScale.x(), pScale.y(), 1.0f);

  for (auto &point: profile) {
    Vector3f P0 = (transformLast * scale).getSubmatrix3x3(0, 0) * point.V;
    Vector3f P1 = (transform * scale).getSubmatrix3x3(0, 0) * point.V;

    Vector3f N0 = transformLast.getSubmatrix3x3(0, 0) * -point.N;
    Vector3f N1 = transform.getSubmatrix3x3(0, 0) * -point.N;

    float t = (P1 - P0).abs() / (last.T + frame.T).abs();
    float sign = Vector3f::dot(P0, last.N) > 0 ? 1 : -1;
    if (Vector3f::dot(last.N, frame.T) > 0.5)
      sign = -sign;

    Vector3f P = P0 + t * sign * last.T + frame.V;
    Vector3f N = (N0 + N1).normalized();

    surface.VV.push_back(P);
    surface.VN.push_back(N);
  }
}

// Assume both the sweep and scale curves are uniform
// Supporting non-uniform curves is far more complicated and this works
// well enough
Vector2f getScale(const Curve &scale, float t) {
  float st = t * (float) (scale.size() - 1);
  int i = (int) st;
  float localT = st - (float) i;
  if (i == scale.size() - 1) {
    i--;
    localT = 1;
  };

  return Vector2f::lerp(scale[i].V.xy(), scale[i + 1].V.xy(), localT);
}

void fillPointMatrix(
  Matrix4f &points,
  const vector<Vector3f> &P,
  unsigned n,
  unsigned sn,
  unsigned sm,
  int d
) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      points(i, j) = P[(sn + i) * n + sm + j][d];
    }
  }
}

unsigned vertexIdx(unsigned i, unsigned j, unsigned n) {
  return i * n + j;
}

void addBezierPatch(
  Surface &surface,
  const vector<Vector3f> &P,
  unsigned n,
  unsigned sn,
  unsigned sm,
  unsigned steps
) {
  Matrix4f points[3];
  for (int d = 0; d < 3; d++) {
    fillPointMatrix(points[d], P, n, sn, sm, d);
  }

  for (unsigned i = 0; i <= steps; i++) {
    for (unsigned j = 0; j <= steps; j++) {
      float u = (float) i / (float) steps;
      float v = (float) j / (float) steps;

      float u2 = u * u, v2 = v * v;
      float u3 = u2 * u, v3 = v2 * v;
      Vector4f uBasis(1, u, u2, u3), vBasis(1, v, v2, v3);
      Vector4f U = bezier * uBasis, V = bezier * vBasis;
      Vector4f dU = bezier_d * uBasis, dV = bezier_d * vBasis;

      Vector3f point, dPdU, dPdV;
      for (int d = 0; d < 3; d++) {
        point[d] = Vector4f::dot(U, points[d] * V);
        dPdU[d] = Vector4f::dot(dU, points[d] * V);
        dPdV[d] = Vector4f::dot(U, points[d] * dV);
      }

      surface.VV.push_back(point);
      surface.VN.push_back(-Vector3f::cross(dPdU, dPdV).normalized());

      if (i > 0 && j > 0) {
        unsigned ii = i + sn, jj = j + sm;
        surface.VF.emplace_back(
          vertexIdx(ii - 1, jj - 1, steps + 1),
          vertexIdx(ii, jj, steps + 1),
          vertexIdx(ii, jj - 1, steps + 1)
        );
        surface.VF.emplace_back(
          vertexIdx(ii - 1, jj - 1, steps + 1),
          vertexIdx(ii - 1, jj, steps + 1),
          vertexIdx(ii, jj, steps + 1)
        );
      }
    }
  }
}

} // namespace

Surface makeSurfRev(const Curve &profile, unsigned steps) {
  Surface surface;

  if (!checkFlat(profile)) {
    cerr << "surfRev profile curve must be flat on xy plane." << endl;
    exit(0);
  }

  // Reserve capacity for surface vectors
  unsigned profileLength = profile.size();
  surface.VV.reserve(profileLength * steps);
  surface.VN.reserve(profileLength * steps);

  // Create vertices and normals
  for (unsigned i = 0; i < steps; i++) {
    float phi = i * 2 * M_PI / steps;
    Matrix3f rotation = Matrix3f::rotateY(phi);

    for (auto &point: profile) {
      // Rotate the point phi radians around the y-axis
      // We don't need translation so we can use a 3x3 matrix
      Vector3f V = rotation * point.V;

      // Rotate the normal
      // Actually the transpose of the inverse, happy coincidence it's the same
      // matrix for rotation only
      Vector3f N = rotation * -point.N; // Normals are flipped for some reason

      surface.VV.push_back(V);
      surface.VN.push_back(N);
    }
  }

  createFaces(surface, profileLength, steps);

  return surface;
}

Surface makeGenCyl(
  const Curve &profile,
  const Curve &sweep,
  const Curve &scale,
  bool useScaleCurve
) {
  Surface surface;

  if (!checkFlat(profile)) {
    cerr << "genCyl profile curve must be flat on xy plane." << endl;
    exit(0);
  }

  // Reserve capacity for surface vectors
  unsigned profileLength = profile.size();
  unsigned steps = sweep.size();
  surface.VV.reserve(profileLength * steps);
  surface.VN.reserve(profileLength * steps);

  // Create vertices and normals
  const CurvePoint *last = nullptr;
  int i = 0;
  Vector2f vScale(1.0f, 1.0f);
  for (auto &center: sweep) {
    if (useScaleCurve) {
      float t = (float) i / (float) (sweep.size() - 1);
      vScale = getScale(scale, t);
    }

    if (last != nullptr && isCorner(center, last)) {
      appendSegment(surface, profile, center.V, *last, vScale);
      appendCorner(surface, profile, center, *last, vScale);
    }

    appendSegment(surface, profile, center.V, center, vScale);
    last = &center;
    i++;
  }

  createFaces(surface, profileLength, steps);

  return surface;
}

Surface makeBirail(
  const Curve &profile,
  const Curve &sweep,
  const Curve &sweep2
) {
  Surface surface;

  if (!checkFlat(profile)) {
    cerr << "birail profile curve must be flat on xy plane." << endl;
    exit(0);
  }

  if (sweep.size() != sweep2.size()) {
    cerr << "birail sweep curves must have the same number of control points."
         << endl;
    exit(0);
  }

  // Reserve capacity for surface vectors
  unsigned profileLength = profile.size();
  unsigned steps = sweep.size();
  surface.VV.reserve(profileLength * steps);
  surface.VN.reserve(profileLength * steps);

  // We need to transform the profile curve so its start and endpoints match
  // the ith point of either sweep curve
  // First, get a frame for the (pn - p0) vector
  CurvePoint frame;
  frame.T = (profile[profileLength - 1].V - profile[0].V).normalized();
  Vector3f X(0, 0, 1);
  if (abs(Vector3f::dot(X, frame.T) - 1) < 1e-8)
    X = Vector3f(0, 1, 0);

  frame.N = Vector3f::cross(X, frame.T).normalized();
  frame.B = Vector3f::cross(frame.T, frame.N);
  frame.V = profile[0].V;

  // Then, calculate the inverse transform (P -> canonical basis)
  Matrix4f pInverse = Matrix4f(
    Vector4f(frame.N, 0), Vector4f(frame.B, 0), Vector4f(frame.T, 0),
    Vector4f(frame.V, 1), true
  ).inverse();

  float baseScale = (profile[profileLength - 1].V - profile[0].V).abs();

  // Create vertices and normals
  for (unsigned i = 0; i < steps; i++) {
    auto &from = sweep[i];
    auto &to = sweep2[i];

    float scale = (to.V - from.V).abs() / baseScale;
    Matrix4f mScale = Matrix4f::uniformScaling(scale);
    CurvePoint sweepFrame;
    sweepFrame.T = (to.V - from.V).normalized();
    sweepFrame.N = -from.B;
    sweepFrame.B = Vector3f::cross(sweepFrame.T, sweepFrame.N);
    sweepFrame.V = profile[0].V;
    sweepFrame.V = from.V;
    Matrix4f sTransform = Matrix4f(
      Vector4f(sweepFrame.N, 0), Vector4f(sweepFrame.B, 0),
      Vector4f(sweepFrame.T, 0), Vector4f(sweepFrame.V, 1), true
    );

    // profile -> sweep
    Matrix4f transform = sTransform * pInverse;
    Matrix3f transformNormal = transform.inverse().transposed()
                                        .getSubmatrix3x3(0, 0);

    for (auto &point: profile) {
      Vector3f V = (transform * mScale * Vector4f(point.V, 1)).xyz();
      Vector3f N =
        transformNormal * -point.N; // Normals are flipped for some reason

      surface.VV.push_back(V);
      surface.VN.push_back(N);
    }
  }

  createFaces(surface, profileLength, steps, false);

  return surface;
}

Surface makeBezierSurf(
  const std::vector<Vector3f> &P,
  unsigned n,
  unsigned steps
) {
  unsigned m = P.size() / n;
  if (n < 4 || n % 3 != 1 || P.size() % n != 0 || m < 4 || m % 3 != 1) {
    cerr << "makeBezierSurf must be called with 3n+1 control points." << endl;
    exit(0);
  }

  unsigned nSegments = (n - 1) / 3;
  unsigned mSegments = (m - 1) / 3;
  Surface surface;

  for (unsigned sn = 0; sn < nSegments; sn++) {
    for (unsigned sm = 0; sm < mSegments; sm++) {
      addBezierPatch(surface, P, n, sn, sm, steps);
    }
  }

  return surface;
}

void drawSurface(const Surface &surface, bool shaded) {
  // Save current state of OpenGL
  glPushAttrib(GL_ALL_ATTRIB_BITS);

  if (shaded) {
    // This will use the current material color and light
    // positions.  Just set these in drawScene();
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // This tells openGL to *not* draw backwards-facing triangles.
    // This is more efficient, and in addition it will help you
    // make sure that your triangles are drawn in the right order.
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
  } else {
    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glColor4f(0.4f, 0.4f, 0.4f, 1.f);
    glLineWidth(1);
  }

  glBegin(GL_TRIANGLES);
  for (unsigned i = 0; i < surface.VF.size(); i++) {
    glNormal(surface.VN[surface.VF[i][0]]);
    glVertex(surface.VV[surface.VF[i][0]]);
    glNormal(surface.VN[surface.VF[i][1]]);
    glVertex(surface.VV[surface.VF[i][1]]);
    glNormal(surface.VN[surface.VF[i][2]]);
    glVertex(surface.VV[surface.VF[i][2]]);
  }
  glEnd();

  glPopAttrib();
}

void drawNormals(const Surface &surface, float len) {
  // Save current state of OpenGL
  glPushAttrib(GL_ALL_ATTRIB_BITS);

  glDisable(GL_LIGHTING);
  glColor4f(0, 1, 1, 1);
  glLineWidth(1);

  glBegin(GL_LINES);
  for (unsigned i = 0; i < surface.VV.size(); i++) {
    glVertex(surface.VV[i]);
    glVertex(surface.VV[i] + surface.VN[i] * len);
  }
  glEnd();

  glPopAttrib();
}

void outputObjFile(ostream &out, const Surface &surface) {

  for (unsigned i = 0; i < surface.VV.size(); i++)
    out << "v  " << surface.VV[i][0] << " " << surface.VV[i][1] << " "
        << surface.VV[i][2] << endl;

  for (unsigned i = 0; i < surface.VN.size(); i++)
    out << "vn " << surface.VN[i][0] << " " << surface.VN[i][1] << " "
        << surface.VN[i][2] << endl;

  out << "vt  0 0 0" << endl;

  for (unsigned i = 0; i < surface.VF.size(); i++) {
    out << "f  ";
    for (unsigned j = 0; j < 3; j++) {
      unsigned a = surface.VF[i][j] + 1;
      out << a << "/"
          << "1"
          << "/" << a << " ";
    }
    out << endl;
  }
}
