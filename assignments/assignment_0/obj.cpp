#include <iostream>
#include <sstream>

#include "obj.h"

#define BUF_SIZE 256

void readObj(
    vector<Vector3f> &vertices,
    vector<Vector3f> &normals,
    vector<vector<unsigned>> &faces
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
}