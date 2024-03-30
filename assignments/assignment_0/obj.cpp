#include <iostream>
#include <sstream>

#include "obj.h"

#define BUF_SIZE 256
#define VERT_SIZE 3 // Number of items in a vertex definition (pos, uv, normal)

void readObj(
    vector<Vector3f> &vertices,
    vector<Vector3f> &normals,
    vector<vector<unsigned>> &faces,
    vector<unsigned> &edges // Vertex adjacency matrix, for mesh simplification
) {
  char buffer[BUF_SIZE];
  string type;
  bool isNormal;

  while (cin.getline(buffer, BUF_SIZE)) {
    stringstream ss(buffer);
    ss >> type;

    if (type == "v" || (isNormal = type == "vn")) {
      double x, y, z;
      ss >> x >> y >> z;

      Vector3f vec(x, y, z);
      if (isNormal) {
        normals.push_back(vec);
      } else {
        vertices.push_back(vec);
      }
    } else if (type == "f") {
      string vertexStr;
      vector<unsigned> vertexData;
      while (ss >> vertexStr) {
        stringstream vss(vertexStr);
        string s;
        while (getline(vss, s, '/')) {
          // Subtract one because OBJ indices are one-indexed
          vertexData.push_back((unsigned)stoi(s) - 1);
        }
      }
      faces.push_back(vertexData);
    }
  }

  int vertexCount = vertices.size();
  edges.reserve(vertexCount * vertexCount);
  edges.resize(edges.capacity(), 0); // Fill with 0

  for (int i = 0; i < faces.size(); i++) {
    auto face = faces[i];
    int faceEdges = face.size();
    for (int j = 0; j < faceEdges; j += VERT_SIZE) {
      int next = (j + VERT_SIZE) % faceEdges;

      // Each pair of consecutive vertices in a face is connected by an edge
      int v1 = face[j], v2 = face[next];
      edges[v1 * vertexCount + v2]++;
    }
  }
}
