#include "renderer_main.h"

using namespace std;

const int WIDTH = 2000;
const int HEIGHT = 1000;
const double ZOOM_CONST = 1.9;

double dataWidth;
double dataHeight;
Point dataCenter;

/**
 * Calculates the dataWidth, dataHeight and dataCenter based on the edges.
 */
// void calculateDataBounds(std::vector<EdgeNode> &edges) {
//     double right = 0, top = 0, left = 0, bottom = 0;
//     for (auto &e : edges) {
//         Point &p1 = *e.getS();
//         Point &p2 = *e.getT();
//         left = p1.x < left ? p1.x : left;
//         right = p2.x > right ? p2.x : right;
//         bottom = p1.y < bottom ? p1.y : bottom;
//         bottom = p2.y < bottom ? p2.y : bottom;
//         top = p1.y > top ? p1.y : top;
//         top = p2.y > top ? p2.y : top;
//     }
//     dataWidth = right - left;
//     dataHeight = top - bottom;
//     dataCenter.x = (float) (dataWidth / 2 + left);
//     dataCenter.y = (float) (dataHeight / 2 + bottom);
// }

void drawLine(const Point *p1, const Point *p2, const int weight) {
    int w = (int)sqrt(weight) / 2;
    if (w < 1) w = 1;
    float alpha = weight / 5.0f;
    if (alpha > 1.0) alpha = 1.0f;
    glLineWidth(w);
    glColor4f(0.5f, 0.5f, 1.0f, alpha);
    glBegin(GL_LINES);
        glVertex2d((p1->x - dataCenter.x) / dataWidth * ZOOM_CONST, (p1->y - dataCenter.y) / dataHeight * ZOOM_CONST);
        glVertex2d((p2->x - dataCenter.x) / dataWidth * ZOOM_CONST, (p2->y - dataCenter.y) / dataHeight * ZOOM_CONST);
    glEnd();
}

void drawBezier(const Point *start, const Point *ctrl1, const Point *ctrl2, const Point *end, const int weight) {
    GLdouble controlPoints[4][3] = {{(start->x - dataCenter.x) / dataWidth * ZOOM_CONST, (start->y - dataCenter.y) / dataHeight * ZOOM_CONST, 0},
                                   {(ctrl1->x - dataCenter.x) / dataWidth * ZOOM_CONST, (ctrl1->y - dataCenter.y) / dataHeight * ZOOM_CONST, 0},
                                   {(ctrl2->x - dataCenter.x) / dataWidth * ZOOM_CONST, (ctrl2->y - dataCenter.y) / dataHeight * ZOOM_CONST, 0},
                                   {(end->x - dataCenter.x) / dataWidth * ZOOM_CONST, (end->y - dataCenter.y) / dataHeight * ZOOM_CONST, 0}};
    float alpha = weight / 5.0f;
    if (alpha > 1.0) alpha = 1.0f;
    glColor4f(0.5f, 0.5f, 1.0f, alpha);
    glMap1d(GL_MAP1_VERTEX_3, 0.0, 1.0, 3, 4, &controlPoints[0][0]);
    glEnable(GL_MAP1_VERTEX_3);
    glLineWidth((weight <= 10.0f) ? 1.0f : (weight * 0.1f));
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i <= 30; i++) {
        glEvalCoord1d((GLdouble) i / 30);
    }
    glEnd();
}

void reshape(int w, int h) {
    glViewport(0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (w <= h)
        glOrtho(-1.0, 1.0, -1.0*(GLfloat)h/(GLfloat)w,
                1.0*(GLfloat)h/(GLfloat)w, -1.0, 1.0);
    else
        glOrtho(-1.0*(GLfloat)w/(GLfloat)h,
                1.0*(GLfloat)w/(GLfloat)h, -1.0, 1.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void renderBundler() {
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // EdgeBundleIterator *iter = bundler->getTree().iterator();
    // while (true) {
    //     BaseNode *n = iter->next();
    //     if (n == nullptr) break;

    //     printf("connecting %3f,%3f to %3f,%3f\n", n->getS()->x, n->getS()->y, n->getT()->x, n->getT()->y);
    //     drawLine(n->getS(), n->getT(), 1);
    // }
    // delete iter;
    glFlush();
}

int main(int argc, char *argv[]) {
  	glutInit(&argc, argv);
  	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA);
  	glutInitWindowSize(WIDTH, HEIGHT);
  	glutCreateWindow("fast-mingle");
  	glutDisplayFunc(renderBundler);
  	glutReshapeFunc(reshape);
  	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  	glutMainLoop();
}