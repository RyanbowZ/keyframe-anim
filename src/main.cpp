#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <vector>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>

#include "Camera.h"
#include "GLSL.h"
#include "Program.h"
#include "MatrixStack.h"
#include "Shape.h"
#include "Helicopter.h"
#include "Keyframe.h"

using namespace std;

GLFWwindow* window; // Main application window
string RESOURCE_DIR = ""; // Where the resources are loaded from

int keyPresses[256] = { 0 }; // only for English keyboards!
int ncps = 0; // count the size of cps
float tmax = 5.0f;
float smax = 0;

enum SplineType
{
    CATMULL_ROM = 0,
    BASIS,
    SPLINE_TYPE_COUNT
};

SplineType type = CATMULL_ROM;

glm::mat4 Bcr, Bb, B;

shared_ptr<Program> prog;
shared_ptr<Camera> camera;
shared_ptr<Helicopter> helicopter;
shared_ptr<Keyframe> keyframe;

vector<pair<glm::vec3, glm::quat> > cps;
vector<vector<float> > px, py, pz;
vector<pair<float, float> > usTable;

static void error_callback(int error, const char* description)
{
    cerr << description << endl;
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }
}

static void char_callback(GLFWwindow* window, unsigned int key)
{
    keyPresses[key]++;
}

static void cursor_position_callback(GLFWwindow* window, double xmouse, double ymouse)
{
    int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    if (state == GLFW_PRESS) {
        camera->mouseMoved(xmouse, ymouse);
    }
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    // Get the current mouse position.
    double xmouse, ymouse;
    glfwGetCursorPos(window, &xmouse, &ymouse);
    // Get current window size.
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    if (action == GLFW_PRESS) {
        bool shift = mods & GLFW_MOD_SHIFT;
        bool ctrl = mods & GLFW_MOD_CONTROL;
        bool alt = mods & GLFW_MOD_ALT;
        camera->mouseClicked(xmouse, ymouse, shift, ctrl, alt);
    }
}


void buildTable()
{
    float vs = 0, vu = 0;
    usTable.push_back(make_pair(vu, vs));
    glm::vec3 P(0);
    glm::vec3 Pt(0);
    for (int k = 0; k < ncps - 3; ++k) {
        glm::mat4 Gk;
        for (int i = 0; i < 4; ++i) {
            Gk[i] = glm::vec4(cps[k + i].first, 0.0f);
        }
        int n = 21; // curve discretization

        for (int i = 0; i < n; ++i) {
            // u goes from 0 to 1 within this segment
            float u = i / (n - 1.0f);
            // Compute spline point at u
            glm::vec4 uVec(1.0f, u, u * u, u * u * u);
            Pt = glm::vec3(Gk * (B * uVec));
            if (i != 0) {
                vs += glm::length(Pt - P);
                vu += 1.0f / (n - 1.0f);
                usTable.push_back(make_pair(vu, vs));
            }
            P = Pt;
        }
    }
}

void buildGaussianTable()
{
    vector<double> xi = { -sqrt(0.6),0,sqrt(0.6) };
    vector<double> wi = { 5.0f / 9.0f,8.0f / 9.0f,5.0f / 9.0f };
    float vs = 0, vu = 0;
    usTable.push_back(make_pair(vu, vs));
    for (int k = 0; k < ncps - 3; ++k) {
        glm::mat4 Gk;
        for (int i = 0; i < 4; ++i) {
            Gk[i] = glm::vec4(cps[k + i].first, 0.0f);
        }

        int n = 21; // curve discretization

        for (int i = 0; i < n; ++i) {
            // u goes from 0 to 1 within this segment
            float u = i / (n - 1.0f);
            float du = 1.0f / (n - 1.0f);
            if (i != 0) {
                for (int j = 0; j < 3; j++) {
                    float pu = (du / 2.0f) * (xi[j] - 1.0f) + u;
                    glm::vec4 uVec(0.0f, 1.0f, 2.0 * pu, 3.0 * pu * pu);
                    vs += (du / 2.0f) * (wi[j] * glm::length(Gk * (B * uVec)));
                }
                vu += du;
                usTable.push_back(make_pair(vu, vs));
            }
        }
    }
}

static float s2u(float s)
{
    int left = 0, right = (int)usTable.size() - 1;
    // Use binary search to decrese time complexity
    while (left < right - 1) {
        int mid = left + (right - left) / 2;
        float mids = usTable[mid].second;
        if (mids == s) {
            return mids;
        }
        else if (mids < s) {
            left = mid;
        }
        else right = mid;
    }
    float s0 = usTable[left].second, s1 = usTable[right].second;
    float u0 = usTable[left].first, u1 = usTable[right].first;
    float alpha = (s - s0) / (s1 - s0);

    return (1 - alpha) * u0 + alpha * u1;
}

float selectFunc(float tNorm) {
    float ualpha = 0;

    if (keyPresses[(unsigned)'s'] % 8 == 2) {
        // uAlpha is the interpolation parameter using Arc-Length Parameterization
        float sNorm = tNorm;
        float s = smax * sNorm;
        ualpha = s2u(s);
    }
    else if (keyPresses[(unsigned)'s'] % 8 == 3) {
        // uAlpha is the interpolation parameter using ease in/out Parameterization
        float sNorm = -2.0f * tNorm * tNorm * tNorm + 3 * tNorm * tNorm;
        float s = smax * sNorm;
        ualpha = s2u(s);
    }
    else if (keyPresses[(unsigned)'s'] % 8 == 4) {
        // uAlpha is the interpolation parameter making the helicoter to stop in middle
        float sNorm = sqrt(tNorm);
        if (sNorm > 0.5f)sNorm = 0.5f;
        float s = smax * sNorm;
        ualpha = s2u(s);
    }
    else if (keyPresses[(unsigned)'s'] % 8 == 5) {
        // uAlpha is the interpolation parameter making the helicoter to go backwards
        float sNorm = -3.0f * pow((tNorm - 0.5f), 2) + 0.75f;
        float s = smax * sNorm;
        ualpha = s2u(s);
    }
    else if (keyPresses[(unsigned)'s'] % 8 == 6) {
        // uAlpha is the interpolation parameter making the helicoter to disappear for 1s and then transport to another place.
        float sNorm = 0.0f;
        if (tNorm <= 0.4f) {
            sNorm = 1.8f * tNorm;
        }
        else if (tNorm > 0.4f && tNorm < 0.6f) {
            sNorm = 0.0f;
        }
        else if (tNorm >= 0.6f) {
            sNorm = 1.3f * tNorm - 0.3f;
        }

        float s = smax * sNorm;
        ualpha = s2u(s);
    }
    // solving a 4x4 linear system
    else if (keyPresses[(unsigned)'s'] % 8 == 7) {
        glm::mat4 A;
        glm::vec4 b(0, 0.68f, 0.45f, 1.0f);
        vector<float> tp = { 0,0.4f,0.65f,1.0f };
        // Fill A
        A[(int)tp.size() - 1] = glm::vec4(1, 1, 1, 1);
        for (int i = (int)tp.size() - 2; i >= 0; i--) {
            for (int k = 0; k < 4; k++) {
                A[i][k] = A[i + 1][k] * tp[k];
            }
        }
        // Solve for x
        glm::vec4 x = glm::inverse(A) * b;
        // Use x solution to compute sNorm
        float sNorm = glm::dot(x, glm::vec4(pow(tNorm, 3), pow(tNorm, 2), tNorm, 1.0f));
        float s = smax * sNorm;
        ualpha = s2u(s);
    }
    // multiple cubics using eigen
    else {
        Eigen::MatrixXf A(8, 8), b(8, 1);
        A << 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0,
            0.064, 0.16, 0.4, 1, -0.064, -0.16, -0.4, -1,
            0.48, 0.8, 1, 0, -0.48, -0.8, -1, 0,
            0, 0, 0, 0, 0.125, 0.25, 0.5, 1,
            0, 0, 0, 0, 0.75, 1, 1, 0,
            0, 0, 0, 0, 1, 1, 1, 1,
            0, 0, 0, 0, 3, 2, 1, 0;
        b << 0,
            0,
            0,
            0,
            0.2,
            0,
            1,
            0;
        Eigen::VectorXf c = A.colPivHouseholderQr().solve(b);
        Eigen::Vector4f coeffs0 = c.segment<4>(0);
        Eigen::Vector4f coeffs1 = c.segment<4>(4);
        float t3 = pow(tNorm, 3), tmid = 0.4f, sNorm;
        if (tNorm < tmid) {
            sNorm = 0;
            for (int i = 0; i < 4; i++) {
                sNorm += coeffs0(i) * t3;
                t3 /= tNorm;
            }
        }
        else {
            sNorm = 0;
            for (int i = 0; i < 4; i++) {
                sNorm += coeffs1(i) * t3;
                t3 /= tNorm;
            }
        }
        float s = smax * sNorm;
        ualpha = s2u(s);
    }
    return ualpha;
}

glm::mat4 computeE(float uhat) {
    int k = floor(uhat);
    float u = uhat - k;
    glm::vec4 uVec(1.0f, u, u * u, u * u * u);
    glm::mat4 Gk;
    // Compute position
    for (int i = 0; i < 4; ++i) {
        Gk[i] = glm::vec4(cps[k + i].first, 0.0f);
    }
    glm::vec3 pVec = Gk * (B * uVec);

    // Compute rotation
    // handle the rotation to be along the short route
    for (int i = 0; i < ncps - 1; i++) {
        if (glm::dot(cps[i].second, cps[i + 1].second) < 0) {
            cps[i + 1].second *= -1;
        }
    }

    for (int i = 0; i < 4; ++i) {
        auto t = cps[k + i].second;
        Gk[i] = glm::vec4(t.x, t.y, t.z, t.w);
    }
    glm::vec4 qVec = Gk * (B * uVec);
    glm::quat q(qVec[3], qVec[0], qVec[1], qVec[2]);
    glm::mat4 E = glm::mat4_cast(glm::normalize(q));
    E[3] = glm::vec4(pVec, 1.0f);
    return E;
}

void drawEqualPoints() {
    if (!usTable.empty()) {
        float ds = 1.0f;
        glColor3f(0.3f, 0.9f, 0.3f);
        glPointSize(7.5f);
        glBegin(GL_POINTS);
        for (float s = 0.0f; s < smax; s += ds) {
            // Convert from s to (concatenated) u
            float uu = s2u(s);
            // Convert from concatenated u to the usual u between 0 and 1.
            float kfloat;
            float u = std::modf(uu, &kfloat);
            // k is the index of the starting control point
            int k = (int)std::floor(kfloat);
            // Compute spline point at u
            glm::mat4 Gk;
            for (int i = 0; i < 4; ++i) {
                Gk[i] = glm::vec4(cps[k + i].first, 0.0f);
            }
            glm::vec4 uVec(1.0f, u, u * u, u * u * u);
            glm::vec3 P(Gk * (B * uVec));
            glVertex3fv(&P[0]);
        }
        glEnd();
    }
}

void drawFrame() {
    glLineWidth(2);
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(1, 0, 0);
    glColor3f(0, 1, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 1, 0);
    glColor3f(0, 0, 1);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 1);
    glEnd();
}

void drawGrid() {
    float gridSizeHalf = 20.0f;
    int gridNx = 40;
    int gridNz = 40;
    glLineWidth(1);
    glColor3f(0.8f, 0.8f, 0.8f);
    glBegin(GL_LINES);
    for (int i = 0; i < gridNx + 1; ++i) {
        float alpha = i / (float)gridNx;
        float x = (1.0f - alpha) * (-gridSizeHalf) + alpha * gridSizeHalf;
        glVertex3f(x, 0, -gridSizeHalf);
        glVertex3f(x, 0, gridSizeHalf);
    }
    for (int i = 0; i < gridNz + 1; ++i) {
        float alpha = i / (float)gridNz;
        float z = (1.0f - alpha) * (-gridSizeHalf) + alpha * gridSizeHalf;
        glVertex3f(-gridSizeHalf, 0, z);
        glVertex3f(gridSizeHalf, 0, z);
    }
    glEnd();
}

void drawSurface(int m, int n) {
    // Draw control points of the surface
    glPointSize(15.0f);
    glColor3f(0.9f, 0.1f, 0.1f);
    glBegin(GL_POINTS);
    for (int c = 0; c < px.size(); c++) {
        for (int r = 0; r < px[0].size(); r++) {
            glVertex3f(px[c][r], py[c][r], pz[c][r]);
        }
    }
    glEnd();

    for (int k = 0; k < px.size() - 3; ++k) {
        glm::mat4 Gkx, Gky, Gkz;
        for (int l = 0; l < px[0].size() - 3; ++l) {

            // initialize the block of GK from 3 dimensions: x, y, z
            for (int i = 0; i < 4; ++i) {
                /* another way to initialize block by another direction:
                Gkx[i] = glm::vec4(px[l][i+k],px[l+1][i+k],px[l+2][i+k],px[l+3][i+k]);
                Gky[i] = glm::vec4(py[l][i+k],py[l+1][i+k],py[l+2][i+k],py[l+3][i+k]);
                Gkz[i] = glm::vec4(pz[l][i+k],pz[l+1][i+k],pz[l+2][i+k],pz[l+3][i+k]);*/
                Gkx[i] = glm::vec4(px[l + i][k], px[l + i][k + 1], px[l + i][k + 2], px[l + i][k + 3]);
                Gky[i] = glm::vec4(py[l + i][k], py[l + i][k + 1], py[l + i][k + 2], py[l + i][k + 3]);
                Gkz[i] = glm::vec4(pz[l + i][k], pz[l + i][k + 1], pz[l + i][k + 2], pz[l + i][k + 3]);
            }

            // render spline surfaces by spline curves with fixed u & v
            // meanwhile draw the relevant normal vectors at each point
            // to reduce time complexity, optimize performance, compute u once and use 3 loops of v to respectively draw.
            for (int p = 0; p < m; ++p) {
                glLineWidth(6.0f);
                glColor3f(0.2, 0.5, 0.2);
                float u = p / (m - 1.0f);
                glm::vec4 uVec(1.0f, u, u * u, u * u * u);
                glm::vec4 up(0.0f, 1.0f, 2.0f * u, 3.0f * u * u);
                glBegin(GL_LINE_STRIP);
                for (int j = 0; j < n; ++j) {
                    float v = j / (n - 1.0f);
                    glm::vec4 vVec(1.0f, v, v * v, v * v * v);
                    glm::vec3 pos(glm::dot(vVec, glm::transpose(B) * (Gkx * (B * uVec))),
                        glm::dot(vVec, glm::transpose(B) * (Gky * (B * uVec))),
                        glm::dot(vVec, glm::transpose(B) * (Gkz * (B * uVec)))
                    );
                    glVertex3fv(&pos[0]);
                }
                glEnd();
                glBegin(GL_LINE_STRIP);
                for (int j = 0; j < n; ++j) {
                    float v = j / (n - 1.0f);
                    glm::vec4 vVec(1.0f, v, v * v, v * v * v);
                    glm::vec3 pos(glm::dot(uVec, glm::transpose(B) * (Gkx * (B * vVec))),
                        glm::dot(uVec, glm::transpose(B) * (Gky * (B * vVec))),
                        glm::dot(uVec, glm::transpose(B) * (Gkz * (B * vVec)))
                    );
                    glVertex3fv(&pos[0]);
                }
                glEnd();
                glLineWidth(3.5f);
                glColor3f(0.78f, 0.8, 0);
                glBegin(GL_LINES);
                for (int j = 0; j < n; ++j) {
                    float v = j / (n - 1.0f);
                    glm::vec4 vVec(1.0f, v, v * v, v * v * v);
                    glm::vec4 vp(0.0f, 1.0f, 2.0f * v, 3.0f * v * v);
                    glm::vec3 pos(glm::dot(vVec, glm::transpose(B) * (Gkx * (B * uVec))),
                        glm::dot(vVec, glm::transpose(B) * (Gky * (B * uVec))),
                        glm::dot(vVec, glm::transpose(B) * (Gkz * (B * uVec)))
                    );
                    glm::vec3 dpu(glm::dot(vVec, glm::transpose(B) * (Gkx * (B * up))),
                        glm::dot(vVec, glm::transpose(B) * (Gky * (B * up))),
                        glm::dot(vVec, glm::transpose(B) * (Gkz * (B * up)))
                    );
                    glm::vec3 dpv(glm::dot(vp, glm::transpose(B) * (Gkx * (B * uVec))),
                        glm::dot(vp, glm::transpose(B) * (Gky * (B * uVec))),
                        glm::dot(vp, glm::transpose(B) * (Gkz * (B * uVec)))
                    );
                    glm::vec3 normal(glm::normalize(glm::cross(dpu, dpv)));
                    float nscale = 0.5f;
                    glVertex3fv(&pos[0]);
                    glVertex3f(pos[0] + normal[0] * nscale, pos[1] + normal[1] * nscale, pos[2] + normal[2] * nscale);
                }
                glEnd();
            }
        }
    }
}


static void init()
{
    GLSL::checkVersion();

    // Set background color
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    // Enable z-buffer test
    glEnable(GL_DEPTH_TEST);

    // Set key inspection
    keyPresses[(unsigned)'c'] = 1;
    keyPresses[(unsigned)'k'] = 1;
    keyPresses[(unsigned)'s'] = 1;
    keyPresses[(unsigned)' '] = 1;
    keyPresses[(unsigned)'b'] = 0;

    prog = make_shared<Program>();
    prog->setShaderNames(RESOURCE_DIR + "phong_vert.glsl", RESOURCE_DIR + "phong_frag.glsl");
    prog->setVerbose(true);
    prog->init();
    prog->addUniform("P");
    prog->addUniform("MV");
    prog->addUniform("lightPos");
    prog->addUniform("ka");
    prog->addUniform("kd");
    prog->addUniform("ks");
    prog->addUniform("s");
    prog->addAttribute("aPos");
    prog->addAttribute("aNor");
    prog->setVerbose(false);

    camera = make_shared<Camera>();
    helicopter = make_shared<Helicopter>();
    helicopter->init(RESOURCE_DIR);
    keyframe = make_shared<Keyframe>();
    keyframe->init(RESOURCE_DIR, 5);

    // Initialize control points.
    cps = {
        make_pair(glm::vec3(0,0,0),glm::angleAxis(float(25.5f / 180.0f * M_PI), glm::normalize(glm::vec3(0,0,-1)))),
        make_pair(glm::vec3(0,0,0),glm::angleAxis(float(25.5f / 180.0f * M_PI), glm::normalize(glm::vec3(0,0,-1)))),
        make_pair(glm::vec3(-3.8f,-0.7f,1.8f),glm::angleAxis(float(25.5f / 180.0f * M_PI), glm::normalize(glm::vec3(.5f,.8f,0)))),
        make_pair(glm::vec3(-4.6f,2.4f,-0.8f),glm::angleAxis(float(145.3f / 180.0f * M_PI), glm::normalize(glm::vec3(.1f,-0.2f,.9f)))),
        make_pair(glm::vec3(2.5f,3.0f,1.2f),glm::angleAxis(float(230.2f / 180.0f * M_PI), glm::normalize(glm::vec3(-3,2,9)))),
        make_pair(glm::vec3(3.2f,-1.9f,-1.7f),glm::angleAxis(float(200.2f / 180.0f * M_PI), glm::normalize(glm::vec3(.1f,.2f,0.0f)))),
        make_pair(glm::vec3(0,0,0),glm::angleAxis(float(25.5f / 180.0f * M_PI), glm::normalize(glm::vec3(0,0,-1)))),
        make_pair(glm::vec3(0,0,0),glm::angleAxis(float(25.5f / 180.0f * M_PI), glm::normalize(glm::vec3(0,0,-1))))
    };
    ncps = (int)cps.size();

    // Initialize spline surface control points.
    px = {
        {-20.0f,-15.0f,-10.0f,-5.0f,0.0f,5.0f,10.0f,15.0f,20.0f},
        {-20.0f,-15.0f,-10.0f,-5.0f,0.0f,5.0f,10.0f,15.0f,20.0f},
        {-20.0f,-15.0f,-10.0f,-5.0f,0.0f,5.0f,10.0f,15.0f,20.0f},
        {-20.0f,-15.0f,-10.0f,-5.0f,0.0f,5.0f,10.0f,15.0f,20.0f},
        {-20.0f,-15.0f,-10.0f,-5.0f,0.0f,5.0f,10.0f,15.0f,20.0f},
        {-20.0f,-15.0f,-10.0f,-5.0f,0.0f,5.0f,10.0f,15.0f,20.0f},
        {-20.0f,-15.0f,-10.0f,-5.0f,0.0f,5.0f,10.0f,15.0f,20.0f},
        {-20.0f,-15.0f,-10.0f,-5.0f,0.0f,5.0f,10.0f,15.0f,20.0f},
        {-20.0f,-15.0f,-10.0f,-5.0f,0.0f,5.0f,10.0f,15.0f,20.0f}
    };
    py = {
        {-3.0f,-4.0f,-6.0f,-6.0f,-6.0f,-6.0f,-6.0f,-3.0f,-3.0f},
        {-2.0f,-2.0f,-6.0f,-6.0f,-6.0f,-8.0f,-7.0f,-6.0f,-3.0f},
        {-2.0f,-3.0f,-4.0f,-6.0f,-6.0f,-6.0f,-9.0f,-10.0f,-3.0f},
        {-2.0f,-3.0f,-6.0f,-2.0f,-6.0f,-6.0f,-7.0f,-9.0f,-3.0f},
        {-2.0f,-4.0f,-6.0f,-6.0f,-4.0f,-6.0f,-6.0f,-7.0f,-3.0f},
        {-2.0f,-4.0f,-6.0f,-6.0f,-6.0f,-2.0f,-6.0f,-3.0f,-3.0f},
        {-2.0f,-5.0f,-6.0f,-1.0f,-6.0f,-6.0f,-5.0f,-3.0f,-3.0f},
        {-2.0f,-6.0f,-6.0f,-6.0f,-6.0f,-6.0f,-6.0f,-3.0f,-3.0f},
        {-2.0f,-6.0f,-6.0f,-6.0f,-6.0f,-6.0f,-6.0f,-3.0f,-3.0f}
    };
    pz = {
        {-20.0f,-20.0f,-20.0f,-20.0f,-20.0f,-20.0f,-20.0f,-20.0f,-20.0f},
        {-15.0f,-15.0f,-15.0f,-15.0f,-15.0f,-15.0f,-15.0f,-15.0f,-15.0f},
        {-10.0f,-10.0f,-10.0f,-10.0f,-10.0f,-10.0f,-10.0f,-10.0f,-10.0f},
        {-5.0f,-5.0f,-5.0f,-5.0f,-5.0f,-5.0f,-5.0f,-5.0f,-5.0f},
        {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
        {5.0f,5.0f,5.0f,5.0f,5.0f,5.0f,5.0f,5.0f,5.0f},
        {10.0f,10.0f,10.0f,10.0f,10.0f,10.0f,10.0f,10.0f,10.0f},
        {15.0f,15.0f,15.0f,15.0f,15.0f,15.0f,15.0f,15.0f,15.0f},
        {20.0f,20.0f,20.0f,20.0f,20.0f,20.0f,20.0f,20.0f,20.0f}
    };

    // Provides two types of B matrix
    Bcr[0] = glm::vec4(0.0f, 2.0f, 0.0f, 0.0f);
    Bcr[1] = glm::vec4(-1.0f, 0.0f, 1.0f, 0.0f);
    Bcr[2] = glm::vec4(2.0f, -5.0f, 4.0f, -1.0f);
    Bcr[3] = glm::vec4(-1.0f, 3.0f, -3.0f, 1.0f);
    Bcr *= 0.5;

    Bb[0] = glm::vec4(1.0f, 4.0f, 1.0f, 0.0f);
    Bb[1] = glm::vec4(-3.0f, 0.0f, 3.0f, 0.0f);
    Bb[2] = glm::vec4(3.0f, -6.0f, 3.0f, 0.0f);
    Bb[3] = glm::vec4(-1.0f, 3.0f, -3.0f, 1.0f);
    Bb /= 6.0f;

    B = (type == CATMULL_ROM ? Bcr : Bb);

    buildGaussianTable();
    //    buildTable();
    smax = usTable.back().second;

    // Initialize time.
    glfwSetTime(0.0);

    // If there were any OpenGL errors, this will print something.
    // You can intersperse this line in your code to find the exact location
    // of your OpenGL error.
    GLSL::checkError(GET_FILE_LINE);
}

void render()
{
    // Update time.
    double t = glfwGetTime();
    // Alpha is the linear interpolation parameter between 0 and 1
    float alpha = std::fmod(0.5f * t, 1.0f);
    float tNorm = std::fmod(t, tmax) / tmax;
    // Choose Spline Curve Mode
    type = (SplineType)((keyPresses[(unsigned)'b']) % SPLINE_TYPE_COUNT);
    B = (type == CATMULL_ROM ? Bcr : Bb);

    // Get current frame buffer size.
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);

    // Use the window size for camera.
    glfwGetWindowSize(window, &width, &height);
    camera->setAspect((float)width / (float)height);
    camera->setInitDistance(0.0f);

    // Clear buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (keyPresses[(unsigned)'c'] % 2) {
        glEnable(GL_CULL_FACE);
    }
    else {
        glDisable(GL_CULL_FACE);
    }
    if (keyPresses[(unsigned)'z'] % 2) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }
    else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    auto P = make_shared<MatrixStack>();
    auto MV = make_shared<MatrixStack>();

    // Apply camera transforms
    P->pushMatrix();
    camera->applyProjectionMatrix(P);
    MV->pushMatrix();

    // Compute the helicopter matrix E first, then handle the camera
    float uhat = keyPresses[(unsigned)'s'] % 8 == 1 ? std::fmod(t, ncps - 3) : selectFunc(tNorm);
    glm::mat4 E = computeE(uhat);

    // Toggle the Camera Mode
    if (keyPresses[(unsigned)' '] % 2) {
        // mannually control the distance from camera to obj
        camera->setInitDistance(10.5f);
        camera->applyViewMatrix(MV);
    }
    else {
        camera->followHelicopter(MV, E, 7.5f);
    }

    // Draw the helicopter animation
    helicopter->draw(prog, P, MV, E, alpha);

    // Show keyframe
    if (keyPresses[(unsigned)'k'] % 2) {
        // Draw the helicopter of keyframes
        keyframe->drawHelicopter(prog, P, MV, cps, alpha);
        // Draw spline curve that connect the keyframes
        keyframe->drawCurve(prog, P, MV, cps, B, alpha);
    }

    // Draw the frame, grid, equal s points, and spline surface with OpenGL 1.x (no GLSL)
    // Setup the projection matrix
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadMatrixf(glm::value_ptr(P->topMatrix()));

    // Setup the modelview matrix
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadMatrixf(glm::value_ptr(MV->topMatrix()));

    // Draw equally spaced points on the spline curve
    drawEqualPoints();

    // Draw frame
    drawFrame();

    // Draw grid
    drawGrid();

    // Draw Spline Surface
    drawSurface(11, 11);

    // Pop modelview matrix
    glPopMatrix();

    // Pop projection matrix
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    // Pop stacks
    MV->popMatrix();
    P->popMatrix();

    GLSL::checkError(GET_FILE_LINE);
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please specify the resource directory." << endl;
        return 0;
    }
    RESOURCE_DIR = argv[1] + string("/");

    // Set error callback.
    glfwSetErrorCallback(error_callback);
    // Initialize the library.
    if (!glfwInit()) {
        return -1;
    }
    // Create a windowed mode window and its OpenGL context.
    window = glfwCreateWindow(640, 480, "YOUR NAME", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    // Make the window's context current.
    glfwMakeContextCurrent(window);
    // Initialize GLEW.
    glewExperimental = true;
    if (glewInit() != GLEW_OK) {
        cerr << "Failed to initialize GLEW" << endl;
        return -1;
    }
    glGetError(); // A bug in glewInit() causes an error that we can safely ignore.
    cout << "OpenGL version: " << glGetString(GL_VERSION) << endl;
    cout << "GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
    // Set vsync.
    glfwSwapInterval(1);
    // Set keyboard callback.
    glfwSetKeyCallback(window, key_callback);
    // Set char callback.
    glfwSetCharCallback(window, char_callback);
    // Set cursor position callback.
    glfwSetCursorPosCallback(window, cursor_position_callback);
    // Set mouse button callback.
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    // Initialize scene.
    init();
    // Loop until the user closes the window.
    while (!glfwWindowShouldClose(window)) {
        if (!glfwGetWindowAttrib(window, GLFW_ICONIFIED)) {
            // Render scene.
            render();
            // Swap front and back buffers.
            glfwSwapBuffers(window);
        }
        // Poll for and process events.
        glfwPollEvents();
    }
    // Quit program.
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
