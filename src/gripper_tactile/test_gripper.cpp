// Test code to verify Gripper finger positions and grasping behavior
#include "Gripper.h"
#include <iostream>
#include <cmath>

int main() {
    // Setup Bullet World
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
    
    dynamicsWorld->setGravity(btVector3(0, -10, 0));

    // Create Gripper
    btVector3 position(0, 2, 0);
    Gripper* gripper = new Gripper(dynamicsWorld, position);

    // Check initial positions
    std::cout << "Initial Positions of Distal Phalanxes:" << std::endl;
    for (int i = 0; i < 3; ++i) {
        btRigidBody* distal = gripper->getFinger(i)->getPhalanx(2);
        btTransform trans;
        distal->getMotionState()->getWorldTransform(trans);
        btVector3 pos = trans.getOrigin();
        std::cout << "Finger " << i << ": " << pos.x() << ", " << pos.y() << ", " << pos.z() << std::endl;
    }

    // Check distances between tips (should be equilateral)
    auto getPos = [&](int i) {
        btTransform trans;
        gripper->getFinger(i)->getPhalanx(2)->getMotionState()->getWorldTransform(trans);
        return trans.getOrigin();
    };

    float d01 = getPos(0).distance(getPos(1));
    float d12 = getPos(1).distance(getPos(2));
    float d20 = getPos(2).distance(getPos(0));

    std::cout << "Distances: " << d01 << ", " << d12 << ", " << d20 << std::endl;

    // Close Gripper
    std::cout << "Closing Gripper..." << std::endl;
    gripper->setGrasp(1.0f);

    // Step simulation
    for (int i = 0; i < 100; ++i) {
        dynamicsWorld->stepSimulation(1.0f / 60.0f, 10);
    }

    // Check final positions
    std::cout << "Final Positions:" << std::endl;
    for (int i = 0; i < 3; ++i) {
        btVector3 pos = getPos(i);
        std::cout << "Finger " << i << ": " << pos.x() << ", " << pos.y() << ", " << pos.z() << std::endl;
    }
    
    // Check if they moved closer to center (0, y, 0) in X-Z plane
    for (int i = 0; i < 3; ++i) {
        btVector3 pos = getPos(i);
        float distToCenter = sqrt(pos.x()*pos.x() + pos.z()*pos.z());
        std::cout << "Finger " << i << " distance to center: " << distToCenter << std::endl;
    }

    // Cleanup
    delete gripper;
    delete dynamicsWorld;
    delete solver;
    delete overlappingPairCache;
    delete dispatcher;
    delete collisionConfiguration;

    return 0;
}
