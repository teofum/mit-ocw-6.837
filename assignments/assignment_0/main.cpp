#include <GLUT/glut.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <vecmath.h>
#include <vector>

#include "obj.h"

#define PI 3.141529

using namespace std;

// Globals

// This is the list of points (3D vectors)
vector<Vector3f> vertices;

// This is the list of normals (also 3D vectors)
vector<Vector3f> normals;

// This is the list of faces (indices into vertices and normals)
vector<vector<unsigned>> faces;

// Color change
constexpr unsigned int colorCount = 4;
unsigned int colorIdx = 0;

// Light position
GLfloat Lt0pos[] = {1.0f, 1.0f, 5.0f, 1.0f};

// Rotation flag and amount
bool rotating = false;
float rotation = 0.0f;
constexpr float rotation_speed = PI * 2 / 10;

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
    colorIdx = (colorIdx + 1) % colorCount;
    break;
  case 'r':
    rotating = !rotating;
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

void redraw(int _) { glutPostRedisplay(); }

// This function is responsible for displaying the object.
void drawScene(void) {
  int i;

  // Clear the rendering window
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Rotate the image
  glMatrixMode(GL_MODELVIEW); // Current matrix affects objects positions
  glLoadIdentity();           // Initialize to the identity

  // Position the camera at [0,0,5], looking at [0,0,0],
  // with [0,1,0] as the up direction.
  gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

  // Set material properties of object

  // Here are some colors you might use - feel free to add more
  GLfloat diffColors[4][4] = {
      {0.5, 0.5, 0.9, 1.0},
      {0.9, 0.5, 0.5, 1.0},
      {0.5, 0.9, 0.3, 1.0},
      {0.3, 0.8, 0.9, 1.0}
  };

  // Here we use the first color entry as the diffuse color
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffColors[colorIdx]);

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
  glRotatef(rotation, 0.0f, 1.0f, 0.0f);
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

  // Dump the image to the screen.
  glutSwapBuffers();

  if (rotating) {
    rotation += rotation_speed;
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

void loadInput() { readObj(vertices, normals, faces); }

// Main routine.
// Set up OpenGL, define the callbacks and start the main loop
int main(int argc, char **argv) {
  loadInput();

  glutInit(&argc, argv);

  // We're going to animate it, so double buffer
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  // Initial parameters for window position and size
  glutInitWindowPosition(60, 60);
  glutInitWindowSize(360, 360);
  glutCreateWindow("Assignment 0");

  // Initialize OpenGL parameters.
  initRendering();

  // Set up callback functions for key presses
  glutKeyboardFunc(keyboardFunc); // Handles "normal" ascii symbols
  glutSpecialFunc(specialFunc);   // Handles "special" keyboard keys

  // Set up the callback function for resizing windows
  glutReshapeFunc(reshapeFunc);

  // Call this whenever window needs redrawing
  glutDisplayFunc(drawScene);

  // Start the main loop.  glutMainLoop never returns.
  glutMainLoop();

  return 0; // This line is never reached.
}
