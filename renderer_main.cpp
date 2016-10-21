#include "renderer_main.h"
#include <unordered_map>
#include <cassert>
#include <vector>

using namespace std;

const int WIDTH = 2000;
const int HEIGHT = 1000;
const double ZOOM_CONST = 1.9;

typedef uint32_t PointId;

struct Point {
	PointId id;
	double x, y;
};

struct Edge {
  PointId point1;
  PointId point2;
  int weight;
};

double dataWidth;
double dataHeight;
Point dataCenter;

std::unordered_map<PointId, Point> pointMap;
std::vector<Edge> edges;

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
    for (auto edge : edges) {
        Point s = pointMap[edge.point1];
        Point t = pointMap[edge.point2];
        drawLine(&s, &t, edge.weight);
    }
    glFlush();
}

/**
 * Reads the nodes from the provided path and calculates the bounds for the data.
 */
void readNodes(char *nodesPath) {
    FILE *nodesFilePointer = fopen(nodesPath, "r");
    assert(nodesFilePointer != NULL);

    Point point;
    double right = 0, top = 0, left = 0, bottom = 0;
    while(fscanf(nodesFilePointer, "%d %f %f", &point.id, &point.x, &point.y) != EOF) {
        pointMap[point.id] = point;
        if (left > point.x) {
            left = point.x;
        }
        if (right < point.x) {
            right = point.x;
        }
        if (top < point.y) {
            top = point.y;
        }
        if (bottom > point.y) {
            bottom = point.y;
        }
    }
    dataWidth = right - left;
    dataHeight = top - bottom;
    dataCenter.x = (float) (dataWidth / 2 + left);
    dataCenter.y = (float) (dataHeight / 2 + bottom);
}

void readEdges(char *edgesPath) {
    FILE *edgesFilePointer = fopen(edgesPath, "r");
    assert(edgesFilePointer != NULL);
    Edge edge;
    while(fscanf(edgesFilePointer, "%u:%u:%u", &edge.point1, &edge.point2, &edge.weight) != EOF) {
        edges.push_back(edge);
    }
}

int main(int argc, char *argv[]) {
	if (argc != 2 && argc != 3) {
        fprintf(stderr, "usage: %s edges.txt nodes.txt\n", *argv);
        return 1;
    }
    readNodes(argv[2]);
    readEdges(argv[1]);

  	glutInit(&argc, argv);
  	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA);
  	glutInitWindowSize(WIDTH, HEIGHT);
  	glutCreateWindow("fast-mingle");
  	glutDisplayFunc(renderBundler);
  	glutReshapeFunc(reshape);
  	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  	glutMainLoop();
}
