#define _USE_MATH_DEFINES
#include <cmath>
#include "Keyframe.h"
#include "Helicopter.h"
#include "Shape.h"
#include "MatrixStack.h"
#include <algorithm>
#include <iostream>

#include "GLSL.h"
#include "Program.h"

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

using namespace std;

Keyframe::Keyframe() {}
Keyframe::~Keyframe() {}

void Keyframe::init(const string& RESOURCE_DIR, int nums) {
    while (nums--) {
        helicopter_frame.push_back(make_shared<Helicopter>());
        helicopter_frame.back()->init(RESOURCE_DIR);
    }
}

void Keyframe::drawHelicopter(const std::shared_ptr<Program> prog, std::shared_ptr<MatrixStack> P, std::shared_ptr<MatrixStack> MV, std::vector<std::pair<glm::vec3, glm::quat> > cps, float alpha) const {
    for (int i = 0; i < helicopter_frame.size(); i++) {
        glm::mat4 E = glm::mat4_cast(glm::normalize(cps[i + 1].second));
        E[3] = glm::vec4(cps[i + 1].first, 1.0f);
        helicopter_frame[i]->draw(prog, P, MV, E, alpha);
    }
}

void Keyframe::drawCurve(const std::shared_ptr<Program> prog, std::shared_ptr<MatrixStack> P, std::shared_ptr<MatrixStack> MV, std::vector<std::pair<glm::vec3, glm::quat> > cps, glm::mat4 B, float alpha) const {
    // Setup the projection matrix
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadMatrixf(glm::value_ptr(P->topMatrix()));

    // Setup the modelview matrix
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadMatrixf(glm::value_ptr(MV->topMatrix()));

    glLineWidth(2.0f);
    for (int k = 0; k < (int)(cps.size()) - 3; ++k) {
        glm::mat4 Gk;
        for (int i = 0; i < 4; ++i) {
            Gk[i] = glm::vec4(cps[k + i].first, 0.0f);
        }
        int n = 31; // curve discretization

        glColor3f(0.8, 0.5, 1);
        glBegin(GL_LINE_STRIP);
        for (int i = 0; i < n; ++i) {
            // u goes from 0 to 1 within this segment
            float u = i / (n - 1.0f);
            // Compute spline point at u
            glm::vec4 uVec(1.0f, u, u * u, u * u * u);
            glVertex3fv(&glm::vec3(Gk * (B * uVec))[0]);
        }
        glEnd();
    }

    // Pop modelview matrix
    glPopMatrix();

    // Pop projection matrix
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
}



