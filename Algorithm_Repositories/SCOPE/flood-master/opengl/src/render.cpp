
#include "render.h"


#include <learnopengl/shader.h>
#include <learnopengl/camera.h>
#include <learnopengl/model.h>
#include <learnopengl/text.h>

#include <iostream>

#define PI 3.14159

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 0.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

Render::Render(float *sa, std::atomic<bool> *d, pthread_mutex_t *lock, std::atomic<bool> *e) {
    shared_array = sa;
    done = d;
    sa_lock = lock;
    exit = e;
}


int Render::run()
{
    //Define euler strings
    std::string yaw, pitch, roll, x, y, z;
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);


    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Predicted Orientation", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, this->framebuffer_size_callback);
    glfwSetCursorPosCallback(window, this->mouse_callback);
    glfwSetScrollCallback(window, this->scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);
    // initialize text renderer
    Shader textshader("shaders/text.vs", "shaders/text.fs");
    Text text(textshader);

    // build and compile shaders
    // -------------------------
    Shader ourShader("shaders/1.model_loading.vs", "shaders/1.model_loading.fs");

    // load models
    // -----------
    Model ourModel("models/target/target_test.obj");
    
    // draw in wireframe
    // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glm::quat q;
    glm::vec3 translation, textColor(0.5, 0.8f, 0.2f);
    // render loop
    // -----------
    while (!glfwWindowShouldClose(window))
    {
        // per-frame time logic
        // --------------------
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(window);

        // render
        // ------
        glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // don't forget to enable shader before setting uniforms
        ourShader.use();
        // view/projection transformations
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();
        ourShader.setMat4("projection", projection);
        ourShader.setMat4("view", view);

        // render the loaded model
        if(*done) {
            pthread_mutex_lock(sa_lock);
            q = glm::quat(shared_array[0], shared_array[1], shared_array[2], shared_array[3]);
            translation = glm::vec3(-shared_array[4], -shared_array[5], -shared_array[6]);
            *done = false;
            pthread_mutex_unlock(sa_lock);
            glm::vec3 euler = glm::eulerAngles(q);
            // q = glm::inverse(q);
            yaw = "Yaw: " + std::to_string(-euler.z*180/PI);
            pitch = "Pitch: " + std::to_string(-euler.y*180/PI);
            roll = "Roll: " + std::to_string(-euler.x*180/PI);
            x = "X: " + std::to_string(translation.x);
            y = "Y: " + std::to_string(translation.y);
            z = "Z: " + std::to_string(translation.z);
        }
        glm::mat4 model, rotation, trans, init_rot;
        trans = glm::translate(trans, translation);
        rotation = glm::toMat4(q);
        init_rot = glm::rotate(init_rot, glm::radians(90.0f), glm::vec3(1.0, 0.0, 0.0));
        model = trans * rotation * init_rot;
        ourShader.setMat4("model", model);
        ourModel.Draw(ourShader);

        text.RenderText(textshader, yaw.c_str(), 25.0f, 75.0f, 0.5f, textColor);
        text.RenderText(textshader, pitch.c_str(), 25.0f, 50.0f, 0.5f, textColor);
        text.RenderText(textshader, roll.c_str(), 25.0f, 25.0f, 0.5f, textColor);
        text.RenderText(textshader, x.c_str(), 600.0f, 75.0f, 0.5f, textColor);
        text.RenderText(textshader, y.c_str(), 600.0f, 50.0f, 0.5f, textColor);
        text.RenderText(textshader, z.c_str(), 600.0f, 25.0f, 0.5f, textColor);
        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    *exit = true;
    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void Render::processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void Render::framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void Render::mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void Render::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}
