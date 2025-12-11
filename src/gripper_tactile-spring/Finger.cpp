#include "Finger.h"

Finger::Finger(btDiscreteDynamicsWorld* world, GUIHelperInterface* helper, 
               const btVector3& basePos, const btQuaternion& baseRot, int fingerIndex)
    : m_dynamicsWorld(world), m_guiHelper(helper), 
      m_basePos(basePos), m_baseRot(baseRot), m_fingerIndex(fingerIndex) {
}

Finger::~Finger() {
    // Cleanup is handled by the world usually, but good practice to have destructors
}

void Finger::buildFinger(btRigidBody* parentBody) {
    // 1. Create the base link (the sensor point)
    // We attach this to the parent body (the gripper palm) via a fixed constraint 
    // that acts as our force sensor.
    
    btTransform parentTrans = parentBody->getWorldTransform();
    btTransform localTrans(m_baseRot, m_basePos);
    btTransform startTrans = parentTrans * localTrans;
    
    btVector3 halfExtents(m_linkRadius, m_linkLength/2.0, m_linkRadius);
    
    // Create the first link (Base)
    m_base = createLink(startTrans, halfExtents);
    m_links.push_back(m_base);
    
    // Create the sensor constraint (Fixed constraint between parent and base)
    // We use a 6DOF constraint to measure forces, but lock all axes.
    btTransform frameInParent = localTrans;
    btTransform frameInBase = btTransform::getIdentity();
    
    m_baseConstraint = new btGeneric6DofConstraint(*parentBody, *m_base, frameInParent, frameInBase, true);
    
    // Lock all linear and angular motion
    m_baseConstraint->setLinearLowerLimit(btVector3(0,0,0));
    m_baseConstraint->setLinearUpperLimit(btVector3(0,0,0));
    m_baseConstraint->setAngularLowerLimit(btVector3(0,0,0));
    m_baseConstraint->setAngularUpperLimit(btVector3(0,0,0));
    
    // Enable feedback
    m_baseConstraint->setJointFeedback(new btJointFeedback());
    m_dynamicsWorld->addConstraint(m_baseConstraint, true);
    m_constraints.push_back(m_baseConstraint);
    
    // 2. Create the rest of the chain
    btRigidBody* prevBody = m_base;
    btTransform prevTrans = startTrans;
    
    for (int i = 1; i < m_numLinks; ++i) {
        // Next link position relative to previous
        // Shift along Y axis (assuming finger points along Y)
        btTransform offsetTrans;
        offsetTrans.setIdentity();
        offsetTrans.setOrigin(btVector3(0, m_linkLength + 0.02, 0)); // Small gap
        
        btTransform currentTrans = prevTrans * offsetTrans;
        
        btRigidBody* link = createLink(currentTrans, halfExtents);
        m_links.push_back(link);
        
        // Spring constraint between links
        btTransform frameInPrev; 
        frameInPrev.setIdentity();
        frameInPrev.setOrigin(btVector3(0, m_linkLength/2.0 + 0.01, 0));
        
        btTransform frameInCurr;
        frameInCurr.setIdentity();
        frameInCurr.setOrigin(btVector3(0, -m_linkLength/2.0 - 0.01, 0));
        
        btGeneric6DofSpringConstraint* spring = new btGeneric6DofSpringConstraint(
            *prevBody, *link, frameInPrev, frameInCurr, true);
            
        // Lock linear motion
        spring->setLinearLowerLimit(btVector3(0,0,0));
        spring->setLinearUpperLimit(btVector3(0,0,0));
        
        // Allow some angular motion with springs
        spring->setAngularLowerLimit(btVector3(-SIMD_PI/4, -SIMD_PI/4, -SIMD_PI/4));
        spring->setAngularUpperLimit(btVector3(SIMD_PI/4, SIMD_PI/4, SIMD_PI/4));
        
        // Enable springs on angular axes (3, 4, 5)
        for(int axis=3; axis<6; ++axis) {
            spring->enableSpring(axis, true);
            spring->setStiffness(axis, m_stiffness);
            spring->setDamping(axis, m_damping);
            spring->setEquilibriumPoint(axis, 0);
        }
        
        m_dynamicsWorld->addConstraint(spring, true);
        m_constraints.push_back(spring);
        
        prevBody = link;
        prevTrans = currentTrans;
    }
}

btRigidBody* Finger::createLink(const btTransform& transform, const btVector3& halfExtents) {
    btCollisionShape* shape = new btBoxShape(halfExtents);
    // Alternatively use Capsule for smoother contact
    // btCollisionShape* shape = new btCapsuleShape(halfExtents.x(), halfExtents.y()*2);
    
    btScalar mass = m_mass;
    btVector3 localInertia(0,0,0);
    shape->calculateLocalInertia(mass, localInertia);
    
    btDefaultMotionState* motionState = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, shape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    
    body->setFriction(1.0); // High friction for gripping
    
    m_dynamicsWorld->addRigidBody(body);
    
    // Visuals
    int colorIndex = m_fingerIndex % 4;
    btVector4 color;
    if (colorIndex == 0) color = btVector4(1,0,0,1);
    else if (colorIndex == 1) color = btVector4(0,1,0,1);
    else if (colorIndex == 2) color = btVector4(0,0,1,1);
    else color = btVector4(1,1,0,1);
    
    m_guiHelper->createCollisionShapeGraphicsObject(shape);
    m_guiHelper->createRigidBodyGraphicsObject(body, color);
    
    return body;
}

btVector3 Finger::getForces() {
    if (m_baseConstraint && m_baseConstraint->getJointFeedback()) {
        return m_baseConstraint->getJointFeedback()->m_appliedForceBodyA;
    }
    return btVector3(0,0,0);
}

btVector3 Finger::getTorques() {
    if (m_baseConstraint && m_baseConstraint->getJointFeedback()) {
        return m_baseConstraint->getJointFeedback()->m_appliedTorqueBodyA;
    }
    return btVector3(0,0,0);
}
