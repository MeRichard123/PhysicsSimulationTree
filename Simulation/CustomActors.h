
#include "BasicActors.h"
#include "PhysicsEngine.h"
#include "vector"

namespace PhysicsEngine {
	class Character : public DynamicActor {
	public:
        Box* torso;
        Box* leftArm;
        Box* rightArm;
        Capsule* head;
        Box* leftLeg;
        Box* rightLeg;
        Box* leftLegLower;
        Box* rightLegLower;

        PxVec3 pos;

        SphericalJoint* leftArmJoint;
        SphericalJoint* rightArmJoint;
        FixedJoint* headJoint;
        RevoluteJoint* leftLegJoint;
        RevoluteJoint* rightLegJoint;
        RevoluteJoint* leftLegKneeJoint;
        RevoluteJoint* rightLegKneeJoint;

        vector<Box*> parts;

        Character(PxReal height = PxReal(1.75f), const PxVec3& position = PxVec3(0, 0, 0),
            const PxTransform& pose = PxTransform(PxIdentity))
            : DynamicActor(pose)
        {
            ComputeProportions(height);
            pos = position;

            // Create torso (main body)
            CreateShape(PxBoxGeometry(PxVec3(height/2, height, height/2)), 1.0f);
            SetKinematic(true);

            // Create arms, head, and legs
            CreateBoxs();

            // Create joints to connect everything
            CreateJoints();
        }

        vector<Box*> GetParts() {
            return parts;
        }
        Capsule* GetHead() {
            return head;
        }
    private:
        PxReal headSize;      // Base unit for proportions
        PxReal torsoWidth;    // Width of torso
        PxReal torsoHeight;   // Height of torso
        PxReal limbWidth;     // Width of arms/legs
        PxReal armLength;     // Length of arms
        PxReal legLength;     // Length of legs

        void ComputeProportions(PxReal height) {
            // Standard proportions based on total height
            headSize = height / 8.0f;              // Head is 1/8 of total height
            torsoHeight = headSize * 3.0f;         // Torso is 3 head lengths
            torsoWidth = headSize * 1.5f;          // Torso width is 1.5 head lengths
            limbWidth = headSize * 0.3f;           // Limb width is 0.3 head lengths
            armLength = headSize * 3.0f;           // Arm length is 3 head lengths
            legLength = headSize * 4.0f;           // Leg length is 4 head lengths
        }


        void CreateBoxs() {

            // Create torso
            torso = new Box(
                PxTransform(
                    PxVec3(pos.x, legLength + (torsoHeight/2) + 0.01f, pos.z)
                ),
                PxVec3(torsoHeight/4, torsoHeight/2, torsoHeight/8), 1.0f
            );
            torso->SetKinematic(true);
            parts.push_back(torso);

            // Create arms
            leftArm = new Box(
                PxTransform(
                    PxVec3(PxIdentity)
                ),
                PxVec3(torsoHeight/16, armLength/2, torsoHeight/16), 0.5f
            );
            parts.push_back(leftArm);

            rightArm = new Box(
                PxTransform(
                    PxVec3(PxIdentity)
                ),
                PxVec3(torsoHeight/16, armLength/2, torsoHeight/16), 0.5f
            );
            parts.push_back(rightArm);

            // Create head
            head = new Capsule(
                PxTransform(
                    PxVec3(PxIdentity), 
                    PxQuat(PxPi/2, PxVec3(0, 0, 1))
                ),
                PxVec2(headSize/1.5f, headSize/3.5f)
            ); 

            // Create legs
            leftLeg = new Box(
                PxTransform(
                    PxVec3(PxIdentity)
                ),
                PxVec3(torsoHeight/9, legLength/4, torsoHeight/9), 0.8f
            );
            parts.push_back(leftLeg);

            rightLeg = new Box(
                PxTransform(
                    PxVec3(PxIdentity)
                ),
                PxVec3(torsoHeight/9, legLength/4, torsoHeight/9), 0.8f
            );
            parts.push_back(rightLeg);

            // Create legs
            leftLegLower = new Box(
                PxTransform(
                    PxVec3(PxIdentity)
                ),
                PxVec3(torsoHeight / 9, legLength / 4, torsoHeight / 9), 0.8f
            );
            parts.push_back(leftLegLower);

            rightLegLower = new Box(
                PxTransform(
                    PxVec3(PxIdentity)
                ),
                PxVec3(torsoHeight / 9, legLength / 4, torsoHeight / 9), 0.8f
            );
            parts.push_back(rightLegLower);
        }

        void CreateJoints() {
            PxReal offset = (0.05f / torsoHeight);
            // Arm joints (spherical)
            leftArmJoint = new SphericalJoint(
                torso,
                PxTransform(
                    PxVec3((torsoHeight/4) + offset, (torsoHeight/2) - offset, 0.0f)
                ),
                leftArm,
                PxTransform(
                    PxVec3(torsoHeight/16, (armLength/2), 0.0f)
                )
            );

            rightArmJoint = new SphericalJoint(
                torso,
                PxTransform(
                    PxVec3(-(torsoHeight/4) - offset, (torsoHeight/2) - offset, 0.0f)
                ),
                rightArm,
                PxTransform(
                    PxVec3(-(torsoHeight/16), (armLength/2), 0.0f)
                )
            );

            // Head joint (revolute)
            headJoint = new FixedJoint(
                torso,
                PxTransform(
                    PxVec3(0.0f, (torsoHeight/2), 0.0f)
                ),
                head,
                PxTransform(
                    PxVec3(headSize, 0.0f, 0.0f), PxQuat(PxPi/2, PxVec3(0, 0, 1))
                )
            );

            // Leg joints (fixed)
            leftLegJoint = new RevoluteJoint(
                torso,
                PxTransform(
                    PxVec3(torsoHeight/8, -(torsoHeight/2), 0.0f)
                ),
                leftLeg,
                PxTransform(
                    PxVec3(0.0f, legLength/4, 0.0f)
                )
            );

            rightLegJoint = new RevoluteJoint(
                torso,
                PxTransform(
                    PxVec3(-(torsoHeight/8), -(torsoHeight/2), 0.0f)
                ),
                rightLeg,
                PxTransform(
                    PxVec3(0.0f, legLength/4, 0.0f)
                )
            );

            leftLegKneeJoint = new RevoluteJoint(
                leftLeg,
                PxTransform(
                    PxVec3(0.0f, -(legLength/4), 0.0f)
                ),
                leftLegLower,
                PxTransform(
                    PxVec3(0.0f, legLength/4, 0.0f)
                )
            );

            rightLegKneeJoint = new RevoluteJoint(
                rightLeg,
                PxTransform(
                    PxVec3(0.0f, -(legLength/4), 0.0f)
                ),
                rightLegLower,
                PxTransform(
                    PxVec3(0.0f, legLength/4, 0.0f)
                )
            );

            // Set joint limits
            PxJointLimitCone limit(PxPi/2, PxPi/4, 0.01f);
            leftArmJoint->SetLimits(limit);
            rightArmJoint->SetLimits(limit);
            leftLegJoint->SetLimits(-PxPi/2, PxPi/2, 0.01f);
            rightLegJoint->SetLimits(-PxPi/2, PxPi/2, 0.01f);
            rightLegKneeJoint->SetLimits(-PxPi/2, 0, 0.01f);
            leftLegKneeJoint->SetLimits(-PxPi/2, 0, 0.01f);
        }
	};
}