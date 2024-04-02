#include "simplify.h"
#include <iostream>

#define T 0.02 // Arbitrary?
#define TARGET_SIZE 2000

using namespace std;

/**
 * Calculate initial error quadrics
 */
static void quadrics(
    const vector<Vector3f> &vertices,
    const vector<vector<unsigned>> &faces,
    vector<Matrix4f> &vertQ
) {
  // Compute the error quadric for each vertex
  // As described in the paper, S5 Deriving Error Quadrics
  for (auto face : faces) {
    // The algorithm assumes the faces are triangles, which is the case for
    // the models we get with the assignment. The program as a whole still works
    // with n-gon faces, but simplification won't work properly in that case.

    Vector3f edge1 = vertices[face[VERT_SIZE]] - vertices[face[0]];
    Vector3f edge2 = vertices[face[VERT_SIZE * 2]] - vertices[face[0]];
    Vector3f normal = Vector3f::cross(edge1, edge2).normalized();

    // Let normal = (a, b, c), then the plane equation is (ax + by + cz + d = 0)
    // To find d we can use any point in the plane, we'll use the first vertex
    // Note that dot(normal, v0) = ax + by + cz = -d
    float d = -1.0f * Vector3f::dot(normal, vertices[face[0]]);
    float a = normal[0], b = normal[1], c = normal[2];

    // Build the matrix Ki with the coefficients
    Matrix4f faceK(
        a * a, a * b, a * c, a * d, //
        b * a, b * b, b * c, b * d, //
        c * a, c * b, c * c, c * d, //
        d * a, d * b, d * c, d * d
    );

    for (int j = 0; j < face.size(); j += VERT_SIZE) {
      unsigned vi = face[j];
      vertQ[vi] += faceK;
    }
  }
}

static VertexPair makePair(
    unsigned i,
    unsigned j,
    const vector<Vector3f> &vertices,
    const vector<Matrix4f> &vertQ
) {
  // Calculate error quadric for pair
  Matrix4f pairQ = vertQ[i] + vertQ[j];
  Matrix4f qq = pairQ;
  qq(0, 3) = 0;
  qq(1, 3) = 0;
  qq(2, 3) = 0;
  qq(3, 3) = 1;

  Vector3f target;
  if (fabsf(qq.determinant()) >= __FLT_EPSILON__) {
    Matrix4f invQ = qq.inverse();
    target[0] = invQ(3, 0);
    target[1] = invQ(3, 1);
    target[2] = invQ(3, 2);
  } else {
    target = vertices[i]; // TODO fall back on interpolating v1, v2
  }

  Vector4f t4(target, 1.0f);
  float cost = Vector3f::dot(target, (pairQ * t4).xyz());

  return VertexPair{i, j, target, cost};
}

static void vertexPairs(
    const vector<Vector3f> &vertices,
    const vector<unsigned> &edges,
    const vector<Matrix4f> &vertQ,
    vector<VertexPair> &pairs
) {
  int vc = vertices.size();
  for (unsigned i = 0; i < vc; i++) {
    for (unsigned j = i + 1; j < vc; j++) {
      if (edges[i * vc + j] || // Vertices are adjacent, OR
          (vertices[i] - vertices[j]).absSquared() < T * T // Distance is < t
      ) {
        pairs.push_back(makePair(i, j, vertices, vertQ));
      }
    }
  }
  make_heap(pairs.begin(), pairs.end(), [](VertexPair &lhs, VertexPair &rhs) {
    return lhs.cost > rhs.cost;
  });
}

void simplify(
    vector<Vector3f> &vertices,
    vector<Vector3f> &normals,
    vector<vector<unsigned>> &faces,
    const vector<unsigned> &edges
) {
  // Quadric error matrix for each vertex
  vector<Matrix4f> vertQ(vertices.size());

  // Initialize each Q to the null matrix
  fill(vertQ.begin(), vertQ.end(), Matrix4f());
  quadrics(vertices, faces, vertQ);

  // Find valid vertex pairs, calculate optimal target and cost and make heap
  vector<VertexPair> pairs;
  vertexPairs(vertices, edges, vertQ, pairs);

  // TODO remove pairs
  while (faces.size() > TARGET_SIZE && pairs.size() > 0) {
    if (faces.size() % 100 == 0)
      cout << faces.size() << " faces" << endl;

    VertexPair pair = pairs[0];
    pop_heap(pairs.begin(), pairs.end(), [](VertexPair &lhs, VertexPair &rhs) {
      return lhs.cost > rhs.cost;
    });
    pairs.pop_back();

    // Replace v1 with new vertex v
    vertices[pair.v1] = pair.target;

    // In each face replace v2 with v1 (now v)
    for (auto &face : faces) {
      char contains = 0x0;
      for (int i = 0; i < face.size(); i += VERT_SIZE) {
        if (face[i] == pair.v1) {
          contains |= 0x1;
        } else if (face[i] == pair.v2) {
          contains |= 0x2;
          face[i] = pair.v1;
        }
      }

      // If the face had both vertices in the pair it's now degenerate
      if (contains == 0x3) {
        face[0] = -1; // Flag for removal
      }
    }

    // Remove any degenerate faces
    faces.erase(
        remove_if(
            faces.begin(), faces.end(),
            [](const vector<unsigned> &face) { return face[0] == -1; }
        ),
        faces.end()
    );

    // TODO: remove any duplicate faces

    // For each pair
    vector<unsigned> pairsToDelete;
    for (int i = 0; i < pairs.size(); i++) {
      VertexPair other = pairs[i];

      // If the pair contains v2 replace with v1
      if (other.v1 == pair.v2 || other.v2 == pair.v2) {
        unsigned u = (other.v1 == pair.v2) ? other.v2 : other.v1;

        if (u == pair.v1) {
          pairsToDelete.push_back(i); // Mark pair for deletion
        } else {
          other.v1 = pair.v1;
          other.v2 = u;

          VertexPair newPair = makePair(other.v1, other.v2, vertices, vertQ);
          other.target = newPair.target;
          other.cost = newPair.cost;
        }
      }
    }

    for (unsigned i : pairsToDelete) {
      pairs[i].cost = 1e20; // Arbitrary big number
      // Float it to the top of the heap
      push_heap(
          pairs.begin(), pairs.begin() + i + 1,
          [](VertexPair &lhs, VertexPair &rhs) { return lhs.cost > rhs.cost; }
      );
      pop_heap(
          pairs.begin(), pairs.end(),
          [](VertexPair &lhs, VertexPair &rhs) { return lhs.cost > rhs.cost; }
      );
      pairs.pop_back();
    }
  }
}
