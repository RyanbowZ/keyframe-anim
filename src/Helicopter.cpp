#define _USE_MATH_DEFINES
#include <cmath>
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

Helicopter::Helicopter(){}
Helicopter::~Helicopter() {}

void Helicopter::init(const string& RESOURCE_DIR) {
	shape = make_shared<Shape>();
	shape2 = make_shared<Shape>();
	shape3 = make_shared<Shape>();
	shape4 = make_shared<Shape>();
	shape->loadMesh(RESOURCE_DIR + "helicopter_body1.obj");
	shape2->loadMesh(RESOURCE_DIR + "helicopter_body2.obj");
	shape3->loadMesh(RESOURCE_DIR + "helicopter_prop1.obj");
	shape4->loadMesh(RESOURCE_DIR + "helicopter_prop2.obj");
	shape->init();
	shape2->init();
	shape3->init();
	shape4->init();
}

void Helicopter::draw(const shared_ptr<Program> prog, shared_ptr<MatrixStack> P, shared_ptr<MatrixStack> MV, glm::mat4 E, float alpha) const {
    
    prog->bind();
    
    MV->pushMatrix();
    MV->multMatrix(E);
    glUniformMatrix4fv(prog->getUniform("P"), 1, GL_FALSE, glm::value_ptr(P->topMatrix()));
    glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	glUniform3f(prog->getUniform("kd"), 1.0f, 0.412f, 0.706f);
	shape->draw(prog);
	glUniform3f(prog->getUniform("kd"), 0.0f, 0.9f, 0.9f);
	shape2->draw(prog);
	glUniform3f(prog->getUniform("kd"), 0.5f, 0.5f, 0.5f);

    MV->pushMatrix();
    MV->translate(0.0, 0.4819, 0.0);
    MV->rotate(2 * M_PI * alpha, glm::vec3(0.0f, 1.0f, 0.0f));
    MV->translate(0.0, -0.4819, 0.0);
    glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
    MV->popMatrix();
    shape3->draw(prog);
    
    MV->pushMatrix();
    MV->translate(0.6228, 0.1179, 0.1365);
    MV->rotate(2*M_PI *alpha, glm::vec3(0.0f, 0.0f, 1.0f));
    MV->translate(-0.6228, -0.1179, -0.1365);
    glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
    MV->popMatrix();
    shape4->draw(prog);

    MV->popMatrix();
    glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
    prog->unbind();
}

