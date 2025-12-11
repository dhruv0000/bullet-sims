#include "CommonInterfaces/CommonRigidBodyBase.h"
#include "Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "Importers/ImportURDFDemo/URDF2Bullet.h"
#include "Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "Utils/b3ResourcePath.h"
#include "Utils/b3Clock.h"

class UrdfExample : public CommonRigidBodyBase
{
public:
	UrdfExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~UrdfExample() {}

	virtual void createEmptyDynamicsWorld()
	{
		m_collisionConfiguration = new btDefaultCollisionConfiguration();
		m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
		m_broadphase = new btDbvtBroadphase();
		m_solver = new btMultiBodyConstraintSolver;
		m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, (btMultiBodyConstraintSolver*)m_solver, m_collisionConfiguration);
		m_dynamicsWorld->setGravity(btVector3(0, 0, -10));
	}

	virtual void initPhysics()
	{
		m_guiHelper->setUpAxis(2); // Z-up for URDFs usually

		createEmptyDynamicsWorld();
		m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

		// Ground
		{
			btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(0.1)));
			m_collisionShapes.push_back(groundShape);
			btTransform groundTransform;
			groundTransform.setIdentity();
			groundTransform.setOrigin(btVector3(0, 0, 0));
			createRigidBody(0, groundTransform, groundShape);
		}

		// Load URDF
		BulletURDFImporter u2b(m_guiHelper, 0);
		bool loadOk = u2b.loadURDF("r2d2.urdf"); 
		if (loadOk)
		{
			btTransform rootTrans;
			rootTrans.setIdentity();
			rootTrans.setOrigin(btVector3(0, 0, 1));
			
			// Convert URDF to Bullet Rigid Bodies
			MyMultiBodyCreator creationCallback(m_guiHelper);
			ConvertURDF2Bullet(u2b, creationCallback, rootTrans, (btMultiBodyDynamicsWorld*)m_dynamicsWorld, true, u2b.getPathPrefix());
		}
		else
		{
			b3Printf("Could not load URDF!");
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
#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "ExampleBrowser/OpenGLGuiHelper.h"

int main(int argc, char* argv[])
{
	SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet URDF Example", 1024, 768, true);

	OpenGLGuiHelper gui(app, false);
	gui.setVisualizerFlag(COV_ENABLE_GUI, false);

	CommonExampleOptions options(&gui);
	UrdfExample* example = new UrdfExample(&gui);

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
