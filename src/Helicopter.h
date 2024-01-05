#pragma once
#ifndef Helicopter_H
#define Helicopter_H
#include <string>
#include <vector>
#include <memory>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

class Shape;
class Program;
class MatrixStack;

class Helicopter {
public:
	Helicopter();
	virtual ~Helicopter();
	void init(const std::string &RESOURCE_DIR);
    void draw(const std::shared_ptr<Program> prog, std::shared_ptr<MatrixStack> P, std::shared_ptr<MatrixStack> MV, glm::mat4 E, float alpha) const;
	
private:
	std::shared_ptr<Shape> shape;
	std::shared_ptr<Shape> shape2;
	std::shared_ptr<Shape> shape3;
	std::shared_ptr<Shape> shape4;
};


#endif
