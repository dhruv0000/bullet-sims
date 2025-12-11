#include "CommonInterfaces/CommonRigidBodyBase.h"
#include "Utils/b3Clock.h"

class BasicExample : public CommonRigidBodyBase
{
public:
	BasicExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~BasicExample() {}

	virtual void initPhysics()
	{
		m_guiHelper->setUpAxis(1);

		createEmptyDynamicsWorld();
		m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

		if (m_dynamicsWorld->getDebugDrawer())
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

		///create a few basic rigid bodies
		btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
		m_collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, -50, 0));

		{
			btScalar mass(0.);
			createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
		}

		{
			//create a few dynamic rigidbodies
			// Re-using the same collision is better for memory usage and performance
			btBoxShape* colShape = new btBoxShape(btVector3(1, 1, 1));
			m_collisionShapes.push_back(colShape);

			/// Create Dynamic Objects
			btTransform startTransform;
			startTransform.setIdentity();

			btScalar mass(1.f);

			//rigidbody is dynamic if and only if mass is non zero, otherwise static
			bool isDynamic = (mass != 0.f);

			btVector3 localInertia(0, 0, 0);
			if (isDynamic)
				colShape->calculateLocalInertia(mass, localInertia);

			startTransform.setOrigin(btVector3(2, 10, 0));

			createRigidBody(mass, startTransform, colShape);
		}

		m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	}

	virtual void renderScene()
	{
		CommonRigidBodyBase::renderScene();
	}
};

#include "CommonInterfaces/CommonExampleInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "SharedMemory/SharedMemoryPublic.h"
#include "Utils/b3Clock.h"

#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "ExampleBrowser/OpenGLGuiHelper.h"

int main(int argc, char* argv[])
{
	SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Basic Example", 1024, 768, true);

	OpenGLGuiHelper gui(app, false);
	gui.setVisualizerFlag(COV_ENABLE_GUI, false); // Disable default GUI for cleaner view

	CommonExampleOptions options(&gui);
	BasicExample* example = new BasicExample(&gui);

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
