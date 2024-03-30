#ifndef __SIMPLIFY_H__
#define __SIMPLIFY_H__

#include <vecmath.h>
#include <vector>

// TODO move this somewhere else so it's not defined twice
#define VERT_SIZE 3 // Number of items in a vertex definition (pos, uv, normal)

using namespace std;

/**
 * Simplify a mesh
 */
void simplify(
    vector<Vector3f> &vertices,
    vector<Vector3f> &normals,
    vector<vector<unsigned>> &faces,
    vector<unsigned> &edges
);

#endif
