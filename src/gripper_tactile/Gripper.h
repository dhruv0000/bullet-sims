#ifndef GRIPPER_H
#define GRIPPER_H

#include "Finger.h"

class Gripper {
public:
    Gripper(btDiscreteDynamicsWorld* world, GUIHelperInterface* helper, const btVector3& position);
    virtual ~Gripper();

    void update(btScalar timeStep);
    
    // Control
    void close(btScalar speed);
    void open(btScalar speed);
    
    // Data
    void printTactileData();

private:
    btDiscreteDynamicsWorld* m_dynamicsWorld;
    GUIHelperInterface* m_guiHelper;
    
    btRigidBody* m_palm;
    std::vector<Finger*> m_fingers;
    
    btScalar m_currentGraspWidth;
    
    // Helper to create the palm
    void createPalm(const btVector3& position);
};

#endif // GRIPPER_H
