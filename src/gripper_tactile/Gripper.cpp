#include "Gripper.h"

Gripper::Gripper(btDiscreteDynamicsWorld* dynamicsWorld, const btVector3& position)
    : m_dynamicsWorld(dynamicsWorld), m_graspAmount(0.0f), m_targetPosition(position)
{
    // Create Palm
    btVector3 palmSize(0.2f, 0.05f, 0.1f);
    btCollisionShape* shape = new btBoxShape(palmSize * 0.5f);
    
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(position);

    btDefaultMotionState* motionState = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(0.0f, motionState, shape, btVector3(0,0,0)); // Mass 0 for kinematic
    m_palm = new btRigidBody(rbInfo);
    
    m_palm->setCollisionFlags(m_palm->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    m_palm->setActivationState(DISABLE_DEACTIVATION);

    m_dynamicsWorld->addRigidBody(m_palm);

    // Create Fingers
    // Finger 1 (Left)
    btVector3 offset1(-0.08f, -0.025f, 0); // Relative to palm center
    m_fingers.push_back(new Finger(0, m_dynamicsWorld, m_palm, position, offset1));

    // Finger 2 (Right)
    btVector3 offset2(0.08f, -0.025f, 0);
    m_fingers.push_back(new Finger(1, m_dynamicsWorld, m_palm, position, offset2));
}

Gripper::~Gripper() {
    for (auto finger : m_fingers) {
        delete finger;
    }
    m_dynamicsWorld->removeRigidBody(m_palm);
    delete m_palm->getMotionState();
    delete m_palm->getCollisionShape();
    delete m_palm;
}

void Gripper::update(float dt) {
    // Move Palm to target
    btTransform trans;
    m_palm->getMotionState()->getWorldTransform(trans);
    
    // Simple interpolation or direct set for kinematic
    // For kinematic, we should update the motion state.
    // Let's just set it directly for now, or interpolate if we want smooth movement.
    // The physics engine handles velocity if we set it via motion state in stepSimulation? 
    // Actually for kinematic bodies, we usually update the transform in the motion state before the step.
    
    trans.setOrigin(m_targetPosition);
    m_palm->getMotionState()->setWorldTransform(trans);
    
    // Update Fingers
    for (auto finger : m_fingers) {
        finger->update(dt);
    }
}

void Gripper::setTarget(const btVector3& position) {
    m_targetPosition = position;
}

void Gripper::setGrasp(float amount) {
    m_graspAmount = amount;
    // Map amount (0..1) to joint angles
    // 0 -> Open (0 degrees or slightly negative)
    // 1 -> Closed (90 degrees or so)
    
    float targetAngle = amount * (SIMD_PI / 2.0f);
    
    for (auto finger : m_fingers) {
        // Finger 0 (Left, at -X) needs to rotate CCW (+Z) to move tip +X
        // Finger 1 (Right, at +X) needs to rotate CW (-Z) to move tip -X
        
        // We can check ID or index. Since we iterate, let's assume index 0 is left.
        // Or better, pass signed angle.
        
        float sign = (finger == m_fingers[0]) ? -1.0f : 1.0f;
        float angle = targetAngle * sign;

        finger->setJointTarget(0, angle);
        finger->setJointTarget(1, angle);
        finger->setJointTarget(2, angle);
    }
}
