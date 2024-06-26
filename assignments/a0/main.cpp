#include <GLUT/glut.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <vecmath.h>
#include <vector>

#include "obj.h"
#include "simplify.h"

#define PI 3.141529

using namespace std;

// Globals

// List of points (3D vectors)
vector<Vector3f> vertices;

// List of normals (also 3D vectors)
vector<Vector3f> normals;

// List of faces (indices into vertices and normals)
vector<vector<unsigned>> faces;

// Vertex adjacency matrix, used for mesh simplification
vector<unsigned> edges;

// Color change
constexpr unsigned int colorCount = 4;
constexpr float colorChangeSpeed = 1.0f / 10;
unsigned int colorIdx = 0, nextColorIdx = 0;
float colorLerp = 0.0f;

GLfloat colors[4][4] = {
    {0.5, 0.5, 0.9, 1.0},
    {0.9, 0.5, 0.5, 1.0},
    {0.5, 0.9, 0.3, 1.0},
    {0.3, 0.8, 0.9, 1.0}
};

// Light position
GLfloat Lt0pos[] = {1.0f, 1.0f, 5.0f, 1.0f};

// Rotation flag and amount
constexpr float rotationSpeed = PI * 2 / 10;
bool rotating = false;
float rotation = 0.0f;

// Mouse position
int lastMousePos[2] = {-1, -1};
Matrix4f rotationMatrix = Matrix4f::identity();
constexpr float zoomSpeed = 0.25f;
float cameraDistance = 5.0f;

// These are convenience functions which allow us to call OpenGL
// methods on Vec3d objects
inline void glVertex(const Vector3f &a) { glVertex3fv(a.getElements()); }

inline void glNormal(const Vector3f &a) { glNormal3fv(a.getElements()); }

// This function is called whenever a "Normal" key press is received.
void keyboardFunc(unsigned char key, int x, int y) {
  switch (key) {
  case 27: // Escape key
    exit(0);
    break;
  case 'c':
    nextColorIdx = (colorIdx + 1) % colorCount;
    colorLerp += colorChangeSpeed;
    break;
  case 'r':
    rotating = !rotating;
    break;
  case '+':
    cameraDistance = fmaxf(cameraDistance - zoomSpeed, 1.0);
    break;
  case '-':
    cameraDistance += zoomSpeed;
    break;
  default:
    cout << "Unhandled key press " << key << "." << endl;
  }

  // this will refresh the screen so that the user sees the color change
  glutPostRedisplay();
}

// This function is called whenever a "Special" key press is received.
// Right now, it's handling the arrow keys.
void specialFunc(int key, int x, int y) {
  switch (key) {
  case GLUT_KEY_UP:
    Lt0pos[1] += 0.5f;
    break;
  case GLUT_KEY_DOWN:
    Lt0pos[1] -= 0.5f;
    break;
  case GLUT_KEY_LEFT:
    Lt0pos[0] -= 0.5f;
    break;
  case GLUT_KEY_RIGHT:
    Lt0pos[0] += 0.5f;
    break;
  }

  // this will refresh the screen so that the user sees the light position
  glutPostRedisplay();
}

// This function is called whenever the mouse is moved with a button pressed
void motionFunc(int x, int y) {
  if (lastMousePos[0] != -1 && lastMousePos[1] != -1) {
    float dx = x - lastMousePos[0];
    float dy = y - lastMousePos[1];
    if (dx != 0.0 || dy != 0.0) {
      float len = sqrtf(dx * dx + dy * dy);
      float angle = len * 0.01;

      Vector2f axis(dy, dx);
      axis.normalize();
      float x = axis.x(), y = axis.y();
      float c = cosf(angle), s = sinf(angle);
      Matrix4f localRotation(
          x * x * (1 - c) + c, x * y * (1 - c), y * s, 0.0,  //
          y * x * (1 - c), y * y * (1 - c) + c, -x * s, 0.0, //
          -y * s, x * s, c, 0.0,                             //
          0.0, 0.0, 0.0, 1.0
      );
      rotationMatrix = localRotation * rotationMatrix;
    }

    // this will refresh the screen
    glutPostRedisplay();
  }

  lastMousePos[0] = x;
  lastMousePos[1] = y;
}

void mouseFunc(int button, int state, int x, int y) {
  if ((button == 3 || button == 4) && state == GLUT_DOWN) {
    int direction = button == 3 ? -1 : 1;
    cameraDistance += direction * zoomSpeed / 10;

    // this will refresh the screen
    glutPostRedisplay();
  } else if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
    lastMousePos[0] = -1;
    lastMousePos[1] = -1;
  }
}

void redraw(int _) { glutPostRedisplay(); }

// This function is responsible for displaying the object.
void drawScene(void) {
  // Clear the rendering window
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Rotate the image
  glMatrixMode(GL_MODELVIEW); // Current matrix affects objects positions
  glLoadIdentity();           // Initialize to the identity

  // Position the camera at [0,0,5], looking at [0,0,0],
  // with [0,1,0] as the up direction.
  gluLookAt(0.0, 0.0, cameraDistance, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

  // Set material properties of object

  GLfloat matColor[4];
  for (int i = 0; i < 4; i++) {
    matColor[i] = (1.0f - colorLerp) * colors[colorIdx][i] +
                  colorLerp * colors[nextColorIdx][i];
  }

  // Here we use the first color entry as the diffuse color
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, matColor);

  // Define specular color and shininess
  GLfloat specColor[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat shininess[] = {100.0};

  // Note that the specular color and shininess can stay constant
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specColor);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);

  // Set light properties

  // Light color (RGBA)
  GLfloat Lt0diff[] = {1.0, 1.0, 1.0, 1.0};

  glLightfv(GL_LIGHT0, GL_DIFFUSE, Lt0diff);
  glLightfv(GL_LIGHT0, GL_POSITION, Lt0pos);

  // Draw object vertices
  glMultMatrixf(rotationMatrix.getElements());
  glRotatef(rotation, 0.0f, 1.0f, 0.0f);
  glCallList(1);

  // Dump the image to the screen.
  glutSwapBuffers();

  if (rotating) {
    rotation += rotationSpeed;
  }
  if (colorLerp != 0.0f) {
    colorLerp += colorChangeSpeed;
    if (colorLerp >= 1.0f) {
      colorLerp = 0.0f;
      colorIdx = nextColorIdx;
    }
  }
  if (rotating || colorLerp != 0.0f) {
    glutTimerFunc(16, redraw, 0);
  }
}

// Initialize OpenGL's rendering modes
void initRendering() {
  glEnable(GL_DEPTH_TEST); // Depth testing must be turned on
  glEnable(GL_LIGHTING);   // Enable lighting calculations
  glEnable(GL_LIGHT0);     // Turn on light #0.
}

// Called when the window is resized
// w, h - width and height of the window in pixels.
void reshapeFunc(int w, int h) {
  // Always use the largest square viewport possible
  if (w > h) {
    glViewport((w - h) / 2, 0, h, h);
  } else {
    glViewport(0, (h - w) / 2, w, w);
  }

  // Set up a perspective view, with square aspect ratio
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // 50 degree fov, uniform aspect ratio, near = 1, far = 100
  gluPerspective(50.0, 1.0, 1.0, 100.0);
}

void loadInput() {
  readObj(vertices, normals, faces, edges);
  simplify(vertices, normals, faces, edges);

  glNewList(1, GL_COMPILE);
  glBegin(GL_TRIANGLES);
  for (int i = 0; i < faces.size(); i++) {
    auto face = faces[i];
    for (int j = 0; j < face.size(); j += 3) {
      Vector3f vertex = vertices[face[j]];
      Vector3f normal = normals[face[2 + j]];
      glNormal3f(normal[0], normal[1], normal[2]);
      glVertex3f(vertex[0], vertex[1], vertex[2]);
    }
  }
  glEnd();
  glEndList();
}

// Main routine.
// Set up OpenGL, define the callbacks and start the main loop
int main(int argc, char **argv) {

  glutInit(&argc, argv);

  // We're going to animate it, so double buffer
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  // Initial parameters for window position and size
  glutInitWindowPosition(60, 60);
  glutInitWindowSize(360, 360);
  glutCreateWindow("Assignment 0");

  // Initialize OpenGL parameters.
  initRendering();

  loadInput();
  // Set up callback functions for key presses
  glutKeyboardFunc(keyboardFunc); // Handles "normal" ascii symbols
  glutSpecialFunc(specialFunc);   // Handles "special" keyboard keys
  glutMotionFunc(motionFunc);     // Handles mouse movement
  glutMouseFunc(mouseFunc);       // Handles mouse up

  // Set up the callback function for resizing windows
  glutReshapeFunc(reshapeFunc);

  // Call this whenever window needs redrawing
  glutDisplayFunc(drawScene);

  // Start the main loop.  glutMainLoop never returns.
  glutMainLoop();

  return 0; // This line is never reached.
}
