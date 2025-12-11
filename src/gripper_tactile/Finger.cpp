#include "Finger.h"
#include <iostream>

Finger::Finger(int id, btDiscreteDynamicsWorld* dynamicsWorld, btRigidBody* palm, const btVector3& palmPosition, const btVector3& offset)
    : m_id(id), m_dynamicsWorld(dynamicsWorld)
{
    btVector3 startPos = palmPosition + offset;
    
    // Create Phalanxes
    // Proximal
    btVector3 proximalPos = startPos + btVector3(0, -m_phalanxLengths[0]/2.0, 0); // Extending down
    btRigidBody* proximal = createPhalanx(btVector3(m_phalanxWidth, m_phalanxLengths[0], m_phalanxHeight), m_phalanxMass, proximalPos);
    m_phalanxes.push_back(proximal);

    // Intermediate
    btVector3 intermediatePos = proximalPos + btVector3(0, -m_phalanxLengths[0]/2.0 - m_phalanxLengths[1]/2.0, 0);
    btRigidBody* intermediate = createPhalanx(btVector3(m_phalanxWidth, m_phalanxLengths[1], m_phalanxHeight), m_phalanxMass, intermediatePos);
    m_phalanxes.push_back(intermediate);

    // Distal
    btVector3 distalPos = intermediatePos + btVector3(0, -m_phalanxLengths[1]/2.0 - m_phalanxLengths[2]/2.0, 0);
    btRigidBody* distal = createPhalanx(btVector3(m_phalanxWidth, m_phalanxLengths[2], m_phalanxHeight), m_phalanxMass, distalPos);
    m_phalanxes.push_back(distal);

    // Create Joints
    btVector3 axis(0, 0, 1); // Hinge axis Z (Rotate in X-Y plane)

    // MCP (Palm <-> Proximal)
    // Pivot in Palm: offset
    // Pivot in Proximal: top center (0, length/2, 0)
    createJoint(0, palm, proximal, offset, btVector3(0, m_phalanxLengths[0]/2.0, 0), axis);

    // PIP (Proximal <-> Intermediate)
    // Pivot in Proximal: bottom center (0, -length/2, 0)
    // Pivot in Intermediate: top center (0, length/2, 0)
    createJoint(1, proximal, intermediate, btVector3(0, -m_phalanxLengths[0]/2.0, 0), btVector3(0, m_phalanxLengths[1]/2.0, 0), axis);

    // DIP (Intermediate <-> Distal)
    createJoint(2, intermediate, distal, btVector3(0, -m_phalanxLengths[1]/2.0, 0), btVector3(0, m_phalanxLengths[2]/2.0, 0), axis);
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
        float targetVelocity = error * 10.0f; // Gain
        float maxImpulse = 10.0f; // Compliance/Strength
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

btRigidBody* Finger::createPhalanx(const btVector3& size, float mass, const btVector3& position) {
    btCollisionShape* shape = new btBoxShape(size * 0.5f); // Box half extents
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(position);

    btVector3 localInertia(0, 0, 0);
    shape->calculateLocalInertia(mass, localInertia);

    btDefaultMotionState* motionState = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, shape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);

    m_dynamicsWorld->addRigidBody(body);
    return body;
}

void Finger::createJoint(int index, btRigidBody* bodyA, btRigidBody* bodyB, const btVector3& pivotInA, const btVector3& pivotInB, const btVector3& axis) {
    btHingeConstraint* hinge = new btHingeConstraint(*bodyA, *bodyB, pivotInA, pivotInB, axis, axis);
    
    // Limits
    hinge->setLimit(-SIMD_PI * 0.25f, SIMD_PI * 0.25f); // Example limits

    // Feedback
    btJointFeedback* feedback = new btJointFeedback();
    m_feedbacks.push_back(feedback);
    hinge->setJointFeedback(feedback);

    m_dynamicsWorld->addConstraint(hinge, true);
    m_joints.push_back(hinge);
}
