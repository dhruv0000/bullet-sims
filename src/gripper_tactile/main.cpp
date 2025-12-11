#include "CommonInterfaces/CommonRigidBodyBase.h"
#include "Utils/b3Clock.h"
#include "Gripper.h"
#include <iostream>

enum State {
    IDLE,
    DESCEND,
    GRASP,
    LIFT,
    HOLD,
    RELEASE
};

class GripperTactileExample : public CommonRigidBodyBase
{
public:
    GripperTactileExample(struct GUIHelperInterface* helper)
        : CommonRigidBodyBase(helper), m_gripper(nullptr), m_state(IDLE), m_stateTime(0)
    {
    }
    virtual ~GripperTactileExample() {
        delete m_gripper;
    }

    virtual void initPhysics()
    {
        m_guiHelper->setUpAxis(1);

        createEmptyDynamicsWorld();
        m_dynamicsWorld->setGravity(btVector3(0, -9.8, 0));
        m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

        if (m_dynamicsWorld->getDebugDrawer())
            m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

        // Ground
        btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
        m_collisionShapes.push_back(groundShape);
        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -50, 0));
        createRigidBody(0, groundTransform, groundShape, btVector4(0.5, 0.5, 0.5, 1));

        // Object to pickup
        btBoxShape* objShape = new btBoxShape(btVector3(0.05, 0.05, 0.05));
        m_collisionShapes.push_back(objShape);
        btTransform objTrans;
        objTrans.setIdentity();
        objTrans.setOrigin(btVector3(0, 0.05, 0)); // On ground
        btScalar mass(0.1f);
        btVector3 localInertia(0, 0, 0);
        objShape->calculateLocalInertia(mass, localInertia);
        createRigidBody(mass, objTrans, objShape, btVector4(1, 0, 0, 1));

        // Gripper
        // Start high above
        m_gripperPos = btVector3(0, 0.5, 0);
        m_gripper = new Gripper(m_dynamicsWorld, m_gripperPos);

        m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
    }

    virtual void stepSimulation(float dt)
    {
        CommonRigidBodyBase::stepSimulation(dt);
        
        m_stateTime += dt;
        m_gripper->update(dt);

        // State Machine
        switch (m_state) {
            case IDLE:
                if (m_stateTime > 1.0f) {
                    m_state = DESCEND;
                    m_stateTime = 0;
                    printf("State: DESCEND\n");
                }
                break;
            case DESCEND:
                m_gripperPos.setY(m_gripperPos.y() - 0.2f * dt);
                if (m_gripperPos.y() < 0.23f) { // Target height
                    m_state = GRASP;
                    m_stateTime = 0;
                    printf("State: GRASP\n");
                }
                m_gripper->setTarget(m_gripperPos);
                break;
            case GRASP:
                m_gripper->setGrasp(1.0f); // Close
                if (m_stateTime > 1.0f) { // Wait for grasp
                    m_state = LIFT;
                    m_stateTime = 0;
                    printf("State: LIFT\n");
                }
                break;
            case LIFT:
                m_gripperPos.setY(m_gripperPos.y() + 0.2f * dt);
                if (m_gripperPos.y() > 0.5f) {
                    m_state = HOLD;
                    m_stateTime = 0;
                    printf("State: HOLD\n");
                }
                m_gripper->setTarget(m_gripperPos);
                break;
            case HOLD:
                if (m_stateTime > 2.0f) {
                    m_state = RELEASE;
                    m_stateTime = 0;
                    printf("State: RELEASE\n");
                }
                break;
            case RELEASE:
                m_gripper->setGrasp(0.0f); // Open
                if (m_stateTime > 1.0f) {
                    m_state = IDLE;
                    m_stateTime = 0;
                    printf("State: IDLE\n");
                }
                break;
        }

        // Print Tactile Data occasionally
        static float printTimer = 0;
        printTimer += dt;
        if (printTimer > 0.5f) {
            printTimer = 0;
            TactileData data = m_gripper->getFinger(0)->getTactileData(2); // Distal joint of finger 0
            printf("F0 Distal Force: %.2f, %.2f, %.2f\n", data.force.x(), data.force.y(), data.force.z());
        }
    }

private:
    Gripper* m_gripper;
    State m_state;
    float m_stateTime;
    btVector3 m_gripperPos;
};

#include "CommonInterfaces/CommonExampleInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "SharedMemory/SharedMemoryPublic.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "ExampleBrowser/OpenGLGuiHelper.h"

class GripperTactileExampleWithCamera : public GripperTactileExample {
public:
    GripperTactileExampleWithCamera(struct GUIHelperInterface* helper) : GripperTactileExample(helper) {}
    virtual void resetCamera()
    {
        float dist = 0.8f;
        float pitch = -20;
        float yaw = 0; // Rotated 90 degrees right from 90
        float targetPos[3] = {0, 0.2, 0};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }
};

int main(int argc, char* argv[])
{
    SimpleOpenGL3App* app = new SimpleOpenGL3App("Gripper Tactile Simulator", 1024, 768, true);

    OpenGLGuiHelper gui(app, false);
    gui.setVisualizerFlag(COV_ENABLE_GUI, false);

    CommonExampleOptions options(&gui);
    GripperTactileExampleWithCamera* example = new GripperTactileExampleWithCamera(&gui);

    example->initPhysics();
    example->resetCamera();

    b3Clock clock;

    do
    {
        app->m_instancingRenderer->init();
        app->m_instancingRenderer->updateCamera(app->getUpAxis());

        btScalar dtSec = btScalar(clock.getTimeInSeconds());
        if (dtSec < 0.001f) dtSec = 0.001f; // Min step
        if (dtSec > 0.1f) dtSec = 0.1f; // Max step

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
