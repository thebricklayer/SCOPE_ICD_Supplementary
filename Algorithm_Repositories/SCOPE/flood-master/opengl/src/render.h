#ifndef RENDER_H
#define RENDER_H

#include "glad/glad.h"
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <atomic>
#include <pthread.h>

class Render {
public:
	Render(float *sa, std::atomic<bool> *d, pthread_mutex_t *lock, std::atomic<bool> *e);
	int run();

private:
	// Functions
	static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
	static void mouse_callback(GLFWwindow* window, double xpos, double ypos);
	static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
	static void processInput(GLFWwindow *window);
	float *shared_array;
	std::atomic<bool> *done, *exit;
	pthread_mutex_t *sa_lock;
};

#endif