#ifndef __OBJ_H__
#define __OBJ_H__

#include <vecmath.h>
#include <vector>

using namespace std;

/**
 * Read an OBJ file from stdin
 * Stores vertex, normal and face data
 */
void readObj(
    vector<Vector3f> &vertices,
    vector<Vector3f> &normals,
    vector<vector<unsigned>> &faces,
    vector<unsigned> &edges
);

#endif
