#include "Finger.h"
#include <iostream>

Finger::Finger(int id, btDiscreteDynamicsWorld* dynamicsWorld, btRigidBody* palm, const btVector3& palmPosition, const btVector3& offset, float yaw)
    : m_id(id), m_dynamicsWorld(dynamicsWorld)
{
    btVector3 startPos = palmPosition + offset;
    btQuaternion rotation(btVector3(0, 1, 0), yaw); // Rotation around Y axis

    // Helper to rotate a vector
    auto rotateVector = [&](const btVector3& v) {
        return quatRotate(rotation, v);
    };
    
    // Create Phalanxes
    // Proximal
    btVector3 proximalOffset(0, -m_phalanxLengths[0]/2.0, 0);
    
    btVector3 proximalPos = startPos + proximalOffset; 
    // The position is just offset from the start pos (which is already placed in the circle).
    // The orientation of the body needs to be rotated.
    
    btRigidBody* proximal = createPhalanx(btVector3(m_phalanxWidth, m_phalanxLengths[0], m_phalanxHeight), m_phalanxMass, proximalPos, rotation);
    m_phalanxes.push_back(proximal);

    // Intermediate
    btVector3 intermediatePos = proximalPos + btVector3(0, -m_phalanxLengths[0]/2.0 - m_phalanxLengths[1]/2.0, 0);
    btRigidBody* intermediate = createPhalanx(btVector3(m_phalanxWidth, m_phalanxLengths[1], m_phalanxHeight), m_phalanxMass, intermediatePos, rotation);
    m_phalanxes.push_back(intermediate);

    // Distal
    btVector3 distalPos = intermediatePos + btVector3(0, -m_phalanxLengths[1]/2.0 - m_phalanxLengths[2]/2.0, 0);
    btRigidBody* distal = createPhalanx(btVector3(m_phalanxWidth, m_phalanxLengths[2], m_phalanxHeight), m_phalanxMass, distalPos, rotation);
    m_phalanxes.push_back(distal);

    // Create Joints
    // We need to define frames for the joints to ensure consistent reference axes (angle 0).
    // The hinge axis is Z (0,0,1).
    // For MCP, Body A (Palm) is in World Frame. Body B (Proximal) is in Rotated Frame.
    // We want the frames to align when angle is 0.
    
    // MCP (Palm <-> Proximal)
    btTransform localA, localB;
    localA.setIdentity(); localB.setIdentity();
    
    localA.setOrigin(offset);
    localA.setRotation(rotation); // Rotate frame A to match finger orientation
    
    localB.setOrigin(btVector3(0, m_phalanxLengths[0]/2.0, 0));
    // localB rotation is Identity because Body B is already rotated.
    
    createJoint(0, palm, proximal, localA, localB);

    // PIP (Proximal <-> Intermediate)
    // Both bodies are in Rotated Frame. Relative rotation is Identity.
    localA.setIdentity(); localB.setIdentity();
    localA.setOrigin(btVector3(0, -m_phalanxLengths[0]/2.0, 0));
    localB.setOrigin(btVector3(0, m_phalanxLengths[1]/2.0, 0));
    
    createJoint(1, proximal, intermediate, localA, localB);

    // DIP (Intermediate <-> Distal)
    localA.setIdentity(); localB.setIdentity();
    localA.setOrigin(btVector3(0, -m_phalanxLengths[1]/2.0, 0));
    localB.setOrigin(btVector3(0, m_phalanxLengths[2]/2.0, 0));
    
    createJoint(2, intermediate, distal, localA, localB);
}

Finger::~Finger() {
    for (auto feedback : m_feedbacks) {
        delete feedback;
    }
    for (auto constraint : m_joints) {
        m_dynamicsWorld->removeConstraint(constraint);
        delete constraint;
    }
    for (auto body : m_phalanxes) {
        m_dynamicsWorld->removeRigidBody(body);
        delete body->getMotionState();
        delete body->getCollisionShape();
        delete body;
    }
}

void Finger::update(float dt) {
    // Update logic if needed
}

void Finger::setJointTarget(int jointIndex, float angle) {
    if (jointIndex >= 0 && jointIndex < m_joints.size()) {
        btHingeConstraint* hinge = m_joints[jointIndex];
        // Simple P-control for servo behavior
        float currentAngle = hinge->getHingeAngle();
        float error = angle - currentAngle;
        float targetVelocity = error * 20.0f; // Gain
        float maxImpulse = 1000.0f; // Increased strength
        hinge->enableAngularMotor(true, targetVelocity, maxImpulse);
    }
}

TactileData Finger::getTactileData(int jointIndex) {
    TactileData data;
    data.force = btVector3(0, 0, 0);
    data.torque = btVector3(0, 0, 0);

    if (jointIndex >= 0 && jointIndex < m_feedbacks.size()) {
        btJointFeedback* fb = m_feedbacks[jointIndex];
        // Force applied on Body B (the phalanx) by Body A (parent)
        data.force = fb->m_appliedForceBodyB;
        data.torque = fb->m_appliedTorqueBodyB;
    }
    return data;
}

btRigidBody* Finger::createPhalanx(const btVector3& size, float mass, const btVector3& position, const btQuaternion& rotation) {
    btCollisionShape* shape = new btBoxShape(size * 0.5f); // Box half extents
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(position);
    transform.setRotation(rotation);

    btVector3 localInertia(0, 0, 0);
    shape->calculateLocalInertia(mass, localInertia);

    btDefaultMotionState* motionState = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, shape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    body->setFriction(10.0f);

    m_dynamicsWorld->addRigidBody(body);
    return body;
}

void Finger::createJoint(int index, btRigidBody* bodyA, btRigidBody* bodyB, const btTransform& localA, const btTransform& localB) {
    btHingeConstraint* hinge = new btHingeConstraint(*bodyA, *bodyB, localA, localB);
    
    // Limits
    hinge->setLimit(-SIMD_PI * 0.25f, SIMD_PI * 0.25f); // Increased limits to 90 degrees

    // Feedback
    btJointFeedback* feedback = new btJointFeedback();
    m_feedbacks.push_back(feedback);
    hinge->setJointFeedback(feedback);

    m_dynamicsWorld->addConstraint(hinge, true);
    m_joints.push_back(hinge);
}
