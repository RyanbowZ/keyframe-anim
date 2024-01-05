#pragma once
#ifndef Keyframe_H
#define Keyframe_H
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
class Helicopter;

class Keyframe {
public:
	Keyframe();
	virtual ~Keyframe();
    void init(const std::string &RESOURCE_DIR, int nums);
    void drawHelicopter(const std::shared_ptr<Program> prog, std::shared_ptr<MatrixStack> P, std::shared_ptr<MatrixStack> MV, std::vector<std::pair<glm::vec3, glm::quat> > cps, float alpha) const;
    void drawCurve(const std::shared_ptr<Program> prog, std::shared_ptr<MatrixStack> P, std::shared_ptr<MatrixStack> MV, std::vector<std::pair<glm::vec3, glm::quat> > cps, glm::mat4 B, float alpha) const;
    
    
private:
    std::vector< std::shared_ptr<Helicopter> > helicopter_frame;
};


#endif
