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
    float radius = 0.08f;
    for (int i = 0; i < 3; ++i) {
        float angle = i * (2.0f * SIMD_PI / 3.0f); // 0, 120, 240 degrees
        
        // Calculate offset from palm center
        // We want the fingers to be arranged in a circle.
        // Let's say 0 degrees is at +X.
        // x = r * cos(angle)
        // z = r * sin(angle)
        // y = -0.025f (slightly below palm center)
        
        float x = radius * cos(angle);
        float z = radius * sin(angle);
        btVector3 offset(x, -0.025f, z);
        
        // Inside Finger, we rotate around Y by 'yaw'.
        // If yaw=0, finger is in default orientation.
        // Default: extends down (-Y). Hinge axis Z. Rotates in X-Y plane.
        // If we want it to close towards center, we need to think about the hinge rotation.
        // If hinge rotates +angle, tip moves one way.
        
        
        // Finger 0 (at +X): angle=0. Default orientation.
        // Finger 1 (at 120 deg): rotated 120 deg around Y.
        // Finger 2 (at 240 deg): rotated 240 deg around Y.
        
        m_fingers.push_back(new Finger(i, m_dynamicsWorld, m_palm, position, offset, -angle));
    }
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
        // Since fingers are radially symmetric and rotated by their position angle,
        // a positive joint angle should move them all "inwards" (or outwards depending on definition).
        // In Finger.cpp, limits are -45 to +45.
        // Default (0) is straight down.
        // We want them to curl inwards.
        // For Finger 0 (at +X, yaw=0), "inwards" is towards -X.
        // Hinge axis is Z. Rotation around Z.
        
        // Positive rotation around Z (CCW) moves tip towards +X (Right).
        // Negative rotation around Z (CW) moves tip towards -X (Left/Center).
        // Positive angle should move them inwards (based on test results)
        
        float angle = targetAngle;

        finger->setJointTarget(0, angle);
        finger->setJointTarget(1, angle);
        finger->setJointTarget(2, angle);
    }
}
