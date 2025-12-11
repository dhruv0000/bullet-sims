#ifndef FINGER_H
#define FINGER_H

#include "btBulletDynamicsCommon.h"
#include <vector>

struct TactileData {
    btVector3 force;
    btVector3 torque;
};

class Finger {
public:
    Finger(int id, btDiscreteDynamicsWorld* dynamicsWorld, btRigidBody* palm, const btVector3& palmPosition, const btVector3& offset);
    ~Finger();

    void update(float dt);
    void setJointTarget(int jointIndex, float angle);
    TactileData getTactileData(int jointIndex);

    btRigidBody* getPhalanx(int index) { return m_phalanxes[index]; }

private:
    btRigidBody* createPhalanx(const btVector3& size, float mass, const btVector3& position);
    void createJoint(int index, btRigidBody* bodyA, btRigidBody* bodyB, const btVector3& pivotInA, const btVector3& pivotInB, const btVector3& axis);

    int m_id;
    btDiscreteDynamicsWorld* m_dynamicsWorld;
    
    std::vector<btRigidBody*> m_phalanxes; // 0: Proximal, 1: Intermediate, 2: Distal
    std::vector<btHingeConstraint*> m_joints; // 0: MCP, 1: PIP, 2: DIP
    std::vector<btJointFeedback*> m_feedbacks;

    // Configuration
    float m_phalanxLengths[3] = {0.08f, 0.06f, 0.04f};
    float m_phalanxWidth = 0.03f;
    float m_phalanxHeight = 0.03f;
    float m_phalanxMass = 0.05f;
};

#endif // FINGER_H
