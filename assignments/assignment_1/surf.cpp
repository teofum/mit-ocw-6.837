#include "surf.h"
#include "extra.h"

#define PI 3.141529f

using namespace std;

namespace {

// We're only implenting swept surfaces where the profile curve is
// flat on the xy-plane.  This is a check function.
static bool checkFlat(const Curve &profile) {
  for (unsigned i = 0; i < profile.size(); i++)
    if (profile[i].V[2] != 0.0 || profile[i].T[2] != 0.0 ||
        profile[i].N[2] != 0.0)
      return false;

  return true;
}

void createFaces(Surface &surface, unsigned profileLength, unsigned steps) {
  surface.VF.reserve((profileLength - 1) * steps * 2);

  for (unsigned i = 1; i <= steps; i++) {
    unsigned iLastProfile = (i - 1) * profileLength;
    unsigned iProfile = (i % steps) * profileLength;

    // Create a quad (two triangles)
    for (unsigned j = 1; j < profileLength; j++) {
      unsigned v00 = iLastProfile + j - 1, v01 = iLastProfile + j,
               v10 = iProfile + j - 1, v11 = iProfile + j;

      surface.VF.push_back(Tup3u(v00, v11, v10));
      surface.VF.push_back(Tup3u(v00, v01, v11));
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
    float phi = i * 2 * PI / (steps + 1);
    Matrix3f rotation = Matrix3f::rotateY(phi);

    for (auto &point : profile) {
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

Surface makeGenCyl(const Curve &profile, const Curve &sweep) {
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
  for (auto &center : sweep) {
    Matrix4f transform = Matrix4f(
        Vector4f(center.N, 0), Vector4f(center.B, 0), Vector4f(center.T, 0),
        Vector4f(center.V, 1), true
    );

    for (auto &point : profile) {
      // Move the point and rotate it to match the sweep coordinate frame
      Vector3f V = (transform * Vector4f(point.V, 1)).xyz();

      // Rotate the normal to match sweep coordinate frame
      // We can use the 3x3 matrix, and because NTB is an othonormal basis, once
      // again the transpose of the inverse is the same matrix
      Vector3f N = transform.getSubmatrix3x3(0, 0) * -point.N;

      surface.VV.push_back(V);
      surface.VN.push_back(N);
    }
  }

  createFaces(surface, profileLength, steps);

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
