#ifndef GRIPPER_H
#define GRIPPER_H

#include "Finger.h"
#include "btBulletDynamicsCommon.h"
#include <vector>

class Gripper {
public:
    Gripper(btDiscreteDynamicsWorld* dynamicsWorld, const btVector3& position);
    ~Gripper();

    void update(float dt);
    void setTarget(const btVector3& position);
    void setGrasp(float amount); // 0.0 (open) to 1.0 (closed)

    Finger* getFinger(int index) { return m_fingers[index]; }
    btRigidBody* getPalm() { return m_palm; }

private:
    btDiscreteDynamicsWorld* m_dynamicsWorld;
    btRigidBody* m_palm;
    std::vector<Finger*> m_fingers;

    // State
    float m_graspAmount;
    btVector3 m_targetPosition;
};

#endif // GRIPPER_H
