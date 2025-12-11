#include "Gripper.h"

Gripper::Gripper(btDiscreteDynamicsWorld* world, GUIHelperInterface* helper, const btVector3& position)
    : m_dynamicsWorld(world), m_guiHelper(helper) {
    
    createPalm(position);
    
    // Create two fingers
    // Finger 1 (Left)
    btVector3 f1Pos(-0.15, 0, 0); // Relative to palm center
    btQuaternion f1Rot(btVector3(0,0,1), 0); // Let's assume Y is "out" of palm
    // Actually, let's align fingers along Y axis, separated by X.
    
    // Let's rotate fingers so they point towards each other slightly? 
    // For now, parallel fingers pointing in Y direction.
    
    m_fingers.push_back(new Finger(world, helper, btVector3(-0.1, 0.1, 0), btQuaternion(0,0,0,1), 0));
    m_fingers.push_back(new Finger(world, helper, btVector3(0.1, 0.1, 0), btQuaternion(0,0,0,1), 1));
    
    for (Finger* f : m_fingers) {
        f->buildFinger(m_palm);
    }
}

Gripper::~Gripper() {
    for (Finger* f : m_fingers) {
        delete f;
    }
    // Palm is managed by world
}

void Gripper::createPalm(const btVector3& position) {
    btVector3 halfExtents(0.2, 0.05, 0.1);
    btCollisionShape* shape = new btBoxShape(halfExtents);
    
    btScalar mass = 10.0; // Heavy base
    btVector3 localInertia(0,0,0);
    shape->calculateLocalInertia(mass, localInertia);
    
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(position);
    
    btDefaultMotionState* motionState = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, shape, localInertia);
    m_palm = new btRigidBody(rbInfo);
    
    // Make the palm kinematic so we can control it easily, or just heavy dynamic?
    // Let's make it Kinematic for absolute control of position if needed, 
    // OR Dynamic with constraints. 
    // For simplicity, let's make it a static/kinematic base for now, or just a heavy floating body.
    // Let's make it KINEMATIC so it stays put unless we move it.
    m_palm->setCollisionFlags(m_palm->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    m_palm->setActivationState(DISABLE_DEACTIVATION);
    
    m_dynamicsWorld->addRigidBody(m_palm);
    
    m_guiHelper->createCollisionShapeGraphicsObject(shape);
    m_guiHelper->createRigidBodyGraphicsObject(m_palm, btVector4(0.5, 0.5, 0.5, 1));
}

void Gripper::update(btScalar timeStep) {
    // Here we could implement grasping logic, e.g. moving fingers closer
    // Since fingers are attached via fixed constraints to palm, we can't move them relative to palm easily
    // UNLESS the base constraint in Finger was a slider/motor.
    // 
    // In Finger.cpp, we used a Generic6DofConstraint locked at 0.
    // To actuate, we should have used a slider or updated the constraint frames.
    
    // For this simple example, let's just move the whole palm down onto an object, 
    // or assume the fingers are pre-positioned.
    
    // TODO: Implement actuation if needed. For now, static grasp test.
}

void Gripper::close(btScalar speed) {
    // To close, we would need to actuate the finger base joints.
    // Current implementation has fixed base joints.
}

void Gripper::open(btScalar speed) {
}

void Gripper::printTactileData() {
    std::cout << "--- Tactile Data ---" << std::endl;
    for (int i=0; i<m_fingers.size(); ++i) {
        btVector3 f = m_fingers[i]->getForces();
        btVector3 t = m_fingers[i]->getTorques();
        printf("Finger %d: F(%.2f, %.2f, %.2f) T(%.2f, %.2f, %.2f)\n", 
               i, f.x(), f.y(), f.z(), t.x(), t.y(), t.z());
    }
}
