#include "CommonInterfaces/CommonExampleInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "ExampleBrowser/OpenGLGuiHelper.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "Utils/b3Clock.h"
#include "SharedMemory/SharedMemoryPublic.h"

#include "Gripper.h"

#include <iostream>

class GripperSim : public CommonExampleInterface {
    CommonGraphicsApp* m_app;
    GUIHelperInterface* m_guiHelper;
    btDiscreteDynamicsWorld* m_dynamicsWorld;
    btBroadphaseInterface* m_broadphase;
    btCollisionDispatcher* m_dispatcher;
    btConstraintSolver* m_solver;
    btDefaultCollisionConfiguration* m_collisionConfiguration;
    
    Gripper* m_gripper;
    btRigidBody* m_targetObject;
    
    btScalar m_time;

public:
    GripperSim(GUIHelperInterface* helper) : m_guiHelper(helper), m_time(0) {
        m_app = helper->getAppInterface();
    }
    
    virtual ~GripperSim() {
        delete m_gripper;
        exitPhysics();
    }
    
    void initPhysics() override {
        m_guiHelper->setUpAxis(1); // Y is up
        
        m_collisionConfiguration = new btDefaultCollisionConfiguration();
        m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
        m_broadphase = new btDbvtBroadphase();
        m_solver = new btSequentialImpulseConstraintSolver();
        
        m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
        m_dynamicsWorld->setGravity(btVector3(0, -9.8, 0));
        
        m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
        
        // Create Ground
        {
            btCollisionShape* groundShape = new btBoxShape(btVector3(50, 1, 50));
            btTransform groundTransform;
            groundTransform.setIdentity();
            groundTransform.setOrigin(btVector3(0, -2, 0));
            
            btScalar mass = 0;
            btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, btVector3(0, 0, 0));
            btRigidBody* body = new btRigidBody(rbInfo);
            m_dynamicsWorld->addRigidBody(body);
            
            m_guiHelper->createCollisionShapeGraphicsObject(groundShape);
            m_guiHelper->createRigidBodyGraphicsObject(body, btVector4(0.8, 0.8, 0.8, 1));
        }
        
        // Create Gripper
        m_gripper = new Gripper(m_dynamicsWorld, m_guiHelper, btVector3(0, 2, 0));
        
        // Create Target Object
        {
            btCollisionShape* shape = new btSphereShape(0.15);
            btTransform transform;
            transform.setIdentity();
            transform.setOrigin(btVector3(0, 2.3, 0)); // Between fingers
            
            btScalar mass = 1.0;
            btVector3 localInertia(0, 0, 0);
            shape->calculateLocalInertia(mass, localInertia);
            
            btDefaultMotionState* motionState = new btDefaultMotionState(transform);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, shape, localInertia);
            m_targetObject = new btRigidBody(rbInfo);
            
            m_dynamicsWorld->addRigidBody(m_targetObject);
            
            m_guiHelper->createCollisionShapeGraphicsObject(shape);
            m_guiHelper->createRigidBodyGraphicsObject(m_targetObject, btVector4(0, 0, 1, 1));
        }
    }
    
    void exitPhysics() override {
        delete m_dynamicsWorld;
        delete m_solver;
        delete m_broadphase;
        delete m_dispatcher;
        delete m_collisionConfiguration;
    }
    
    void stepSimulation(float deltaTime) override {
        m_dynamicsWorld->stepSimulation(deltaTime);
        m_time += deltaTime;
        
        // Log data every 60 steps (approx 1 sec if 60Hz)
        static int counter = 0;
        if (counter++ % 60 == 0) {
            m_gripper->printTactileData();
        }
    }
    
    void renderScene() override {
        m_dynamicsWorld->debugDrawWorld();
    }
    
    void physicsDebugDraw(int debugFlags) override {
        m_dynamicsWorld->debugDrawWorld();
    }
    
    bool mouseMoveCallback(float x, float y) override { return false; }
    bool mouseButtonCallback(int button, int state, float x, float y) override { return false; }
    bool keyboardCallback(int key, int state) override { return false; }
    
    void resetCamera() override {
        float dist = 4;
        float pitch = -30;
        float yaw = 0;
        float targetPos[3] = {0, 2, 0};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }
};

int main(int argc, char* argv[])
{
    SimpleOpenGL3App* app = new SimpleOpenGL3App("Gripper Tactile Simulator", 1024, 768, true);

    OpenGLGuiHelper gui(app, false);
    gui.setVisualizerFlag(COV_ENABLE_GUI, false);

    CommonExampleOptions options(&gui);
    GripperSim* example = new GripperSim(&gui);

    example->initPhysics();
    example->resetCamera();

    b3Clock clock;

    do
    {
        app->m_instancingRenderer->init();
        app->m_instancingRenderer->updateCamera(app->getUpAxis());

        btScalar dtSec = btScalar(clock.getTimeInSeconds());
        if (dtSec < 0.1)
            dtSec = 0.1;

        example->stepSimulation(dtSec);
        clock.reset();

        example->renderScene();
        
        app->drawGrid();
        app->swapBuffer();
    } while (!app->m_window->requestedExit());

    example->exitPhysics();
    delete example;
    delete app;
    return 0;
}
