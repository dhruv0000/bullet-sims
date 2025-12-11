#ifndef FINGER_H
#define FINGER_H

#include "btBulletDynamicsCommon.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "btBulletDynamicsCommon.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"

#include <vector>
#include <iostream>

class Finger {
public:
    Finger(btDiscreteDynamicsWorld* world, GUIHelperInterface* helper, 
           const btVector3& basePos, const btQuaternion& baseRot, int fingerIndex);
    virtual ~Finger();

    void buildFinger(btRigidBody* parentBody);
    
    // Tactile data extraction
    btVector3 getForces();
    btVector3 getTorques();
    
    // Accessors
    btRigidBody* getBase() { return m_base; }

private:
    btDiscreteDynamicsWorld* m_dynamicsWorld;
    GUIHelperInterface* m_guiHelper;
    
    int m_fingerIndex;
    btVector3 m_basePos; // Relative to parent
    btQuaternion m_baseRot; // Relative to parent
    
    btRigidBody* m_base;
    btGeneric6DofConstraint* m_baseConstraint; // The sensor constraint
    
    btAlignedObjectArray<btRigidBody*> m_links;
    btAlignedObjectArray<btTypedConstraint*> m_constraints;
    
    // Configuration
    int m_numLinks = 5;
    btScalar m_linkLength = 0.2;
    btScalar m_linkRadius = 0.05;
    btScalar m_stiffness = 100.0;
    btScalar m_damping = 5.0;
    btScalar m_mass = 0.1;
    
    // Helper to create a link body
    btRigidBody* createLink(const btTransform& transform, const btVector3& halfExtents);
};

#endif // FINGER_H
