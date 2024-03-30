#include "simplify.h"

void simplify(
    vector<Vector3f> &vertices,
    vector<Vector3f> &normals,
    vector<vector<unsigned>> &faces,
    vector<unsigned> &edges
) {
  // Quadric error matrix for each vertex
  vector<Matrix4f> vertQ(vertices.size());

  // Initialize each Q to the null matrix
  std::fill(vertQ.begin(), vertQ.end(), Matrix4f());

  // First, compute the error quadric for each vertex

  for (int i = 0; i < faces.size(); i++) {
    auto face = faces[i];
    // Calculate the equation of the plane (ax + by + cz + d = 0)
    // Build the matrix Ki with the coefficients
    Matrix4f faceK; // TODO values

    for (int j = 0; j < face.size(); j += VERT_SIZE) {
      unsigned vi = face[j];
      vertQ[vi] += faceK;
    }
  }
}