
#include "BasicActors.h"
#include "PhysicsEngine.h"
#include "vector"
#include "SZ_Cylinder.h"
#include <iomanip>
#include <random>

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
            CreateShape(PxBoxGeometry(PxVec3(height / 2, height, height / 2)), 1.0f);
            SetKinematic(true);

            // Create arms, head, and legs
            CreateBoxes();

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


        void CreateBoxes() {

            // Create torso
            torso = new Box(
                PxTransform(
                    PxVec3(pos.x, legLength + (torsoHeight / 2) + 0.01f, pos.z)
                ),
                PxVec3(torsoHeight / 4, torsoHeight / 2, torsoHeight / 8), 1.0f
            );
            torso->SetKinematic(true);
            parts.push_back(torso);

            // Create arms
            leftArm = new Box(
                PxTransform(
                    PxVec3(PxIdentity)
                ),
                PxVec3(torsoHeight / 16, armLength / 2, torsoHeight / 16), 0.5f
            );
            parts.push_back(leftArm);

            rightArm = new Box(
                PxTransform(
                    PxVec3(PxIdentity)
                ),
                PxVec3(torsoHeight / 16, armLength / 2, torsoHeight / 16), 0.5f
            );
            parts.push_back(rightArm);

            // Create head
            head = new Capsule(
                PxTransform(
                    PxVec3(PxIdentity),
                    PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                ),
                PxVec2(headSize / 1.5f, headSize / 3.5f)
            );

            // Create legs
            leftLeg = new Box(
                PxTransform(
                    PxVec3(PxIdentity)
                ),
                PxVec3(torsoHeight / 9, legLength / 4, torsoHeight / 9), 0.8f
            );
            parts.push_back(leftLeg);

            rightLeg = new Box(
                PxTransform(
                    PxVec3(PxIdentity)
                ),
                PxVec3(torsoHeight / 9, legLength / 4, torsoHeight / 9), 0.8f
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
                    PxVec3((torsoHeight / 4) + offset, (torsoHeight / 2) - offset, 0.0f)
                ),
                leftArm,
                PxTransform(
                    PxVec3(torsoHeight / 16, (armLength / 2), 0.0f)
                )
            );

            rightArmJoint = new SphericalJoint(
                torso,
                PxTransform(
                    PxVec3(-(torsoHeight / 4) - offset, (torsoHeight / 2) - offset, 0.0f)
                ),
                rightArm,
                PxTransform(
                    PxVec3(-(torsoHeight / 16), (armLength / 2), 0.0f)
                )
            );

            // Head joint (revolute)
            headJoint = new FixedJoint(
                torso,
                PxTransform(
                    PxVec3(0.0f, (torsoHeight / 2), 0.0f)
                ),
                head,
                PxTransform(
                    PxVec3(headSize, 0.0f, 0.0f), PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            // Leg joints (fixed)
            leftLegJoint = new RevoluteJoint(
                torso,
                PxTransform(
                    PxVec3(torsoHeight / 8, -(torsoHeight / 2), 0.0f)
                ),
                leftLeg,
                PxTransform(
                    PxVec3(0.0f, legLength / 4, 0.0f)
                )
            );

            rightLegJoint = new RevoluteJoint(
                torso,
                PxTransform(
                    PxVec3(-(torsoHeight / 8), -(torsoHeight / 2), 0.0f)
                ),
                rightLeg,
                PxTransform(
                    PxVec3(0.0f, legLength / 4, 0.0f)
                )
            );

            leftLegKneeJoint = new RevoluteJoint(
                leftLeg,
                PxTransform(
                    PxVec3(0.0f, -(legLength / 4), 0.0f)
                ),
                leftLegLower,
                PxTransform(
                    PxVec3(0.0f, legLength / 4, 0.0f)
                )
            );

            rightLegKneeJoint = new RevoluteJoint(
                rightLeg,
                PxTransform(
                    PxVec3(0.0f, -(legLength / 4), 0.0f)
                ),
                rightLegLower,
                PxTransform(
                    PxVec3(0.0f, legLength / 4, 0.0f)
                )
            );

            // Set joint limits
            PxJointLimitCone limit(PxPi / 2, PxPi / 4, 0.01f);
            leftArmJoint->SetLimits(limit);
            rightArmJoint->SetLimits(limit);
            leftLegJoint->SetLimits(-PxPi / 2, PxPi / 2, 0.01f);
            rightLegJoint->SetLimits(-PxPi / 2, PxPi / 2, 0.01f);
            rightLegKneeJoint->SetLimits(-PxPi / 2, 0, 0.01f);
            leftLegKneeJoint->SetLimits(-PxPi / 2, 0, 0.01f);
        }
    };

    class StaticTreePart : public TriangleMesh
    {
    public:
        StaticTreePart(PxTransform pose = PxTransform(PxIdentity), PxReal baseRadius = 2.f, PxReal topRadius = 1.f, PxReal height = 1.f)
            : TriangleMesh(GenerateTaperedCylinderVertices(baseRadius, topRadius, height, 16),
                GenerateCylinderIndices(16), pose)
        {
        }
    private:

        std::vector<PxVec3> GenerateTaperedCylinderVertices(PxReal baseRadius, PxReal topRadius, PxReal height, int numSides)
        {
            std::vector<PxVec3> verts;
            float angleStep = PxTwoPi / numSides;
            float halfHeight = height / 2;

            // Bottom circle (wider base)
            for (int i = 0; i < numSides; i++)
            {
                float angle = i * angleStep;
                float x = baseRadius * cos(angle);
                float z = baseRadius * sin(angle);
                verts.push_back(PxVec3(x, -halfHeight, z));
            }

            // Top circle (narrower top)
            for (int i = 0; i < numSides; i++)
            {
                float angle = i * angleStep;
                float x = topRadius * cos(angle);
                float z = topRadius * sin(angle);
                verts.push_back(PxVec3(x, halfHeight, z));
            }

            verts.push_back(PxVec3(0, -halfHeight, 0));
            verts.push_back(PxVec3(0, halfHeight, 0));

            return verts;
        }

        std::vector<PxU32> GenerateCylinderIndices(int numSides)
        {
            std::vector<PxU32> indices;
            int bottomCenter = numSides * 2;
            int topCenter = bottomCenter + 1;

            // Connecting top and bottom vertices with triangles
            for (int i = 0; i < numSides; i++)
            {
                int next = (i + 1) % numSides;

                // Side triangles
                indices.push_back(i);
                indices.push_back(next);
                indices.push_back(numSides + i);

                indices.push_back(numSides + i);
                indices.push_back(next);
                indices.push_back(numSides + next);
            }

            //Add a Bottom Lid
            for (int i = 0; i < numSides; i++)
            {
                int next = (i + 1) % numSides;
                indices.push_back(i);
                indices.push_back(bottomCenter);
                indices.push_back(next);
            }

            // Top lid (connect each top vertex to the center)
            for (int i = 0; i < numSides; i++)
            {
                int next = (i + 1) % numSides;
                indices.push_back(numSides + i);
                indices.push_back(numSides + next);
                indices.push_back(topCenter);
            }
            return indices;
        }
    };

    class Tree : public DynamicActor
    {
    public:
        StaticTreePart* treeBase;
        float bottomRad = 0.8;
        float topRad = bottomRad /= 1.2;
        float posY = 0.5;

        float height = 8.0f;
        vector<StaticTreePart*> trunkParts = { 0 };
        vector<Box*> parts;
        Box* core;

        Tree(PxTransform pose = PxTransform(PxIdentity))
            : DynamicActor(pose)
        {
            CreateTrunk();

            core = new Box(PxTransform(PxVec3(18.f, height + (topRad * 2), 0.f)), PxVec3(topRad, topRad, topRad));
            PxFilterData coreFilterData;
            coreFilterData.word0 = 1;
            core->GetShape()->setSimulationFilterData(coreFilterData);
            parts.push_back(core);

            AddJoints();
            AddBranches();
        }

        vector<Box*> getParts()
        {
            return parts;
        }

        vector<StaticTreePart*> getTrunkParts()
        {
            return trunkParts;
        }

        void CreateBranch(Box* branch, int depth = 0, int side = -1) {
            random_device rd;
            mt19937 gen(rd());

            if (depth == 2) return;

            //for (int i = 0; i < 2; ++i) {

                //std::uniform_real_distribution<float> angleDist(PxPi / 8, PxPi / 2);
                //std::uniform_real_distribution<float> axisDist(-1.0f, 1.0f);

            PxVec3 dims = branch->GetShape()->getGeometry().box().halfExtents;
            PxTransform pos = branch->GetShape()->getLocalPose();

            //PxVec3 randomAxis(axisDist(gen), axisDist(gen), axisDist(gen));
            //randomAxis.normalize();

            Box* b = new Box(pos, PxVec3(dims.x / 2, dims.y / 2, dims.z / 2));
            b->Color(PxVec3(105 / 255.0f, 75 / 255.0f, 55 / 255.0f));

            std::uniform_real_distribution<float> posFactorDist(-dims.y + 0.01, dims.y);

            //float randomAngle = angleDist(gen);

            int posParentBranch = posFactorDist(gen);
            float outwardOffset = dims.x * 1.5f;

            PxReal angle = (PxPi) / 4;

            PxQuat rotation = PxQuat(side * angle, PxVec3(1, 0, 0));
            PxQuat flip = PxQuat(PxPi, PxVec3(1, 0, 0));

            FixedJoint* j = new FixedJoint(
                branch,
                PxTransform(PxVec3(outwardOffset, posParentBranch, dims.z), rotation),
                b,
                PxTransform(PxVec3(dims.x / 2, dims.y / 2, dims.z), flip)
            );


            parts.push_back(b);

            depth++;
            side *= -1;

            CreateBranch(b, depth, side);
            CreateBranch(b, depth, side);
            //}
        }

        void CreateTrunk()
        {
            while (posY < height)
            {
                treeBase = new StaticTreePart(PxTransform(PxVec3(5.f, posY, 0.f)), bottomRad, topRad, 1.25f);
                treeBase->Color(PxVec3(105 / 255.0f, 75 / 255.0f, 55 / 255.0f));  // Brown color (105,75,55)
                bottomRad = topRad;
                topRad /= 1.2;
                posY += 1.25f;
                trunkParts.push_back(treeBase);
            }
        }

        void AddJoints()
        {
            FixedJoint* tree_core = new FixedJoint(
                trunkParts[trunkParts.size() - 1],
                PxTransform(PxVec3(0.0f, 1.25f / 2.0f, 0.0f)), // Local offset from the sphere
                core,
                PxTransform(PxVec3(0.0f, -topRad, 0.0f))
            );
        }

        void AddBranches()
        {
            float branchAngleStep = (2 * PxPi) / 4; // 90-degrees
            PxReal angle = 0 * branchAngleStep;
            topRad += 0.01f;

            PxRigidActor* coreActor = static_cast<PxRigidActor*>(core->Get());
            PxTransform coreTransform = coreActor->getGlobalPose();

            PxVec3 brown(105 / 255.0f, 75 / 255.0f, 55 / 255.0f);
            PxFilterData branchFilterData;
            branchFilterData.word0 = 2;

            // Define branch rotation angles
            PxReal branchAngles[] = {
                PxPi / 2,
                (3 * PxPi) / 2,
                (3 * PxPi) / 2,
                PxPi / 2
            };

            // Define branch positions
            PxVec3 branchPositions[] = {
                PxVec3(topRad, 0.0f, 0.0f),
                PxVec3(-topRad, 0.0f, 0.0f),
                PxVec3(0.0f, 0.0f, topRad),
                PxVec3(0.0f, 0.0f, -topRad)
            };

            for (int i = 0; i < 4; i++) {
                Box* branch = new Box(
                    coreTransform,
                    PxVec3(topRad / 3, height / 2.5, topRad / 3));
                branch->Color(brown);
                PxMaterial* wood = CreateMaterial(PxReal(0.4), PxReal(0.6), PxReal(0.7));
                branch->Material(wood);

                PxQuat horizontalRotation = PxQuat(branchAngles[i], PxVec3((i < 2) ? 0 : 1, 0, (i < 2) ? 1 : 0));
                PxQuat yRotation = PxQuat(angle, PxVec3(0, 1, 0));
                PxQuat finalRotation = yRotation * horizontalRotation;

                FixedJoint* branch_joint = new FixedJoint(
                    core,
                    PxTransform(branchPositions[i], finalRotation),
                    branch,
                    PxTransform(PxVec3(0.0f, (height / 2.5f), 0.0f))
                );

                branch->GetShape()->setSimulationFilterData(branchFilterData);
                CreateBranch(branch);
                parts.push_back(branch);
            }
        }
    };

    class House : public DynamicActor {
    public:
        vector<Box*> planks = {0};
        House(PxTransform pose = PxTransform(PxIdentity))
            : DynamicActor(pose)
        {
            PxVec3 PlankDims = PxVec3(0.2f, 2.0f, 0.05f);
            //CreateWall(PxVec3(0.2f, 2.0f, 0.05f), 10, 0.05f, PxQuat(PxPi / 2, PxVec3(0, 0, 1)));
            //CreateWall(PxVec3(0.2f, 2.0f, 0.05f), 10, 0.05f, PxQuat(PxPi / 2, PxVec3(0, 0,1)));
            
            vector<Box*> beamsA = CreateWall(
                PxVec3(00.0f, 0.0f, 0.0f),
                PlankDims,
                10, 0.05f,
                PxQuat(0, PxVec3(0, 0, 1))
            );


            vector<Box*> beamsB = CreateWall(
                PxVec3(10.0f, 0.0f, 00.0f),
                PlankDims,
                10, 0.05f,
                PxQuat(PxPi/2, PxVec3(0,0,1))
            );


            PxBoxGeometry boxAGeometry1 = GetBoxGeometry(beamsA[0]);
            PxBoxGeometry boxBGeometry1 = GetBoxGeometry(beamsB[0]); 
            PxBoxGeometry boxAGeometry2 = GetBoxGeometry(beamsA[1]);
            PxBoxGeometry boxBGeometry2 = GetBoxGeometry(beamsB[1]);

            FixedJoint* joint = new FixedJoint(
                beamsA[0],
                PxTransform(
                    PxVec3(boxAGeometry1.halfExtents.x ,boxAGeometry1.halfExtents.y, boxAGeometry1.halfExtents.z),
                    PxQuat(PxPi / 2, PxVec3(0,1, 0))
                ),
                beamsB[0],
                PxTransform(
                    PxVec3(boxBGeometry2.halfExtents.x, 0, boxBGeometry2.halfExtents.z)
                )
            );

            FixedJoint* joint2 = new FixedJoint(
                beamsA[1],
                PxTransform(
                    PxVec3(boxAGeometry2.halfExtents.x, boxAGeometry2.halfExtents.y, boxAGeometry2.halfExtents.z),
                    PxQuat(PxPi / 2, PxVec3(0, 1, 0)) 
                ),
                beamsB[1],
                PxTransform(
                    PxVec3(0, 0, boxBGeometry1.halfExtents.z)
                )
            );

        }

        vector<Box*> getPlanks()
        {
            return planks;
        }

        PxBoxGeometry GetBoxGeometry(Box* box)
        {
            return box->GetShape(0)->getGeometry().box();
        }

        vector<Box*> CreateWall(PxVec3 pose ,PxVec3 plankSize, int numPlanks, PxReal spacing, PxQuat rot)
        {
            PxReal totalWidth = numPlanks * (plankSize.x + spacing) - spacing;
            PxReal halfWidth = totalWidth * 0.5f;
            PxReal halfHeight = plankSize.y * 0.5f;
            int offset = 1.00f;
            vector<Box*> endBeams = vector<Box*>();

            // Create the top and bottom horizontal beams
            PxVec3 beamSize = PxVec3(halfWidth, plankSize.z, plankSize.z);
            PxTransform topPose(PxVec3(pose.x, plankSize.y + offset, pose.z), rot);
            PxTransform bottomPose(PxVec3(pose.x, 0 + offset, pose.z), rot);

            /// rest = 0, frict = 1, statfric = 1
            PxMaterial* wood = CreateMaterial(PxReal(1.0f), PxReal(1.0f), PxReal(0.0f));
            
            Box* topBeam = new Box(topPose, PxVec3(beamSize.x, beamSize.y / 2, beamSize.z / 2));
            Box* bottomBeam = new Box(bottomPose, PxVec3(beamSize.x, beamSize.y / 2, beamSize.z / 2));
            topBeam->Material(wood);
            bottomBeam->Material(wood);

            topBeam->Color(PxVec3(105 / 255.0f, 75 / 255.0f, 55 / 255.0f));
            bottomBeam->Color(PxVec3(105 / 255.0f, 75 / 255.0f, 55 / 255.0f));
            planks.push_back(topBeam);
            planks.push_back(bottomBeam);

            for (int i = 0; i < 2; i++) //numplanks
            {
                PxReal xPos = -halfWidth + i * (plankSize.x + spacing) + plankSize.x * 0.5f;
                PxTransform plankPose(PxVec3(xPos, halfHeight + offset, 0), rot);
                Box* plank = new Box(plankPose, PxVec3(plankSize.x / 2, plankSize.y / 2, plankSize.z / 2));
                plank->Material(wood);
                plank->Color(PxVec3(105 / 255.0f, 75 / 255.0f, 55 / 255.0f));

                // Attach to beams with FixedJoint
                FixedJoint* jointTop = new FixedJoint(
                    topBeam,
                    PxTransform(PxVec3(xPos + 0.01f, 0, 0), rot),
                    plank,
                    PxTransform(PxVec3(plankSize.x / 2, plankSize.y / 2 * 0.75, plankSize.z), rot)
                );

                FixedJoint* jointBottom = new FixedJoint(
                    bottomBeam,
                    PxTransform(PxVec3(xPos + 0.01f, 0, 0), rot),
                    plank,
                    PxTransform(PxVec3(plankSize.x / 2, -(plankSize.y / 2 * 0.75), plankSize.z), rot)
                );

                //uniplanks.push_back(plank);
            }

            endBeams.push_back(topBeam);
            endBeams.push_back(bottomBeam);

            return endBeams;
        }
    };

    class Cabin : public DynamicActor
    {
    public:
        vector<SZ_Cylinder*> logs;
        Box* floor;
        
        Cabin(PxTransform pose = PxTransform(PxIdentity))
            : DynamicActor(pose)
        {
            int numLayers = 9;      
            float logLength = 3.0f; 
            float logRadius = 0.1f; 
            float spacing = logRadius * 2.f;
            float logGap = 0.01f;   
            float windowWidth = .2f;
            float windowHeight = .5f; 
            float logDiameter = logRadius * 2.f;
            float floorThickness = 0.01f; 

            PxVec3 floorLocalPos(0, floorThickness / 2.f, 0); 
            PxTransform floorTransform = pose * PxTransform(floorLocalPos); 
            floor = new Box(floorTransform, PxVec3(logLength / 2, floorThickness / 2, logLength / 2));
  
            PxVec3 positions[4] = {
                PxVec3(0, logRadius, -logLength / 2), // Bottom-left corner
                PxVec3(logLength / 2, logRadius, 0),  // Bottom-right corner
                PxVec3(0, logRadius, logLength / 2),  // Top-left corner
                PxVec3(-logLength / 2, logRadius, 0)    // Top-right corner
            };

            std::vector<std::vector<SZ_Cylinder*>> layeredLogs(numLayers);
    
            for (int layer = 0; layer < numLayers; ++layer)
            {
                float y = layer * (logRadius * 2 + logGap);
                   
                for (int i = 0; i < 4; ++i)
                {
                    PxVec3 pos = positions[i];
                    pos.y = y;
                    PxTransform t;
                    float currentLogLength = logLength; // Default length
                    bool createLog = true; // Flag to skip log if needed

                    if (i % 2 == 0)
                    {
                        t = PxTransform(pos, PxQuat(PxPi / 2, PxVec3(0, 0, 1))); // Y-up to X
                    }
                    else
                    {
                        t = PxTransform(pos, PxQuat(PxPi / 2, PxVec3(1, 0, 0))); // Y-up to Z
                    }

                    if (i == 0) // Layers below window height
                    {
                        if (layer > 2 && layer < numLayers - 2) // Base layer: Split into two short logs
                        {
                            PxVec3 leftPos(-1.1f, y, -logLength / 2);
                            PxTransform leftT(leftPos, PxQuat(PxPi / 2, PxVec3(0, 0, 1)));
                            PxTransform leftWorldT = pose * leftT;
                            auto* leftLog = new SZ_Cylinder(leftWorldT, logRadius, (logLength - (windowWidth)) / 8.f, 10.f);
                            logs.push_back(leftLog);

                            PxVec3 rightPos(1.1f, y, -logLength / 2);
                            PxTransform rightT(rightPos, PxQuat(PxPi / 2, PxVec3(0, 0, 1)));
                            PxTransform rightWorldT = pose * rightT;
                            auto* rightLog = new SZ_Cylinder(rightWorldT, logRadius, (logLength - (windowWidth)) / 8.f, 10.f);
                            logs.push_back(rightLog);
                            layeredLogs[layer].push_back(rightLog);

                            createLog = false; // Skip full log
                        }
                    }

                    if (createLog)
                    {
                        PxTransform worldT = pose * t;
                        auto* log = new SZ_Cylinder(worldT, logRadius, currentLogLength / 2.f, 10.f);
                        logs.push_back(log);
                        layeredLogs[layer].push_back(log);
                    }
                }

                if (layer > 0)
                {
                    for (int i = 0; i < 4; ++i)
                    {
                        if (i == 0 && layer > 2 && layer < numLayers - 2)
                        {
                            // Window case: connect left and right logs to the full log below
                            if (!layeredLogs[layer - 1].empty())
                            {
                                // Left log joint
                                FixedJoint* leftJoint = new FixedJoint(
                                    layeredLogs[layer - 1][0],
                                    PxTransform(PxVec3(-0.8f, logRadius, 0)), // Top center of lower log at left segment
                                    layeredLogs[layer][0],
                                    PxTransform(PxVec3(0, -logRadius, 0))    // Bottom center of left log
                                );
                                FixedJoint* rightJoint = new FixedJoint(
                                    layeredLogs[layer - 1][0],
                                    PxTransform(PxVec3(0.8f, logRadius, 0)),  // Top center of lower log at right segment
                                    layeredLogs[layer][1],
                                    PxTransform(PxVec3(0, -logRadius, 0))    // Bottom center of right log
                                );
    
                            }
                        }
                        else if (!layeredLogs[layer].empty() && !layeredLogs[layer - 1].empty())
                        {
                            // Standard case: connect log to the one below it
                            int belowIdx = (i == 0 && layeredLogs[layer - 1].size() > 1) ? 0 : i; // Adjust for window layers
                            FixedJoint* joint = new FixedJoint(
                                layeredLogs[layer - 1][belowIdx], 
                                PxTransform(PxVec3(logRadius, 0, 0)), // Lower log's top
                                layeredLogs[layer][i], 
                                PxTransform(PxVec3(-logRadius, 0, 0))
                            ); 
                        }
                    }
                }
            }
        }

        vector<SZ_Cylinder*> GetLogs()
        {
            return logs;
        }

        Box* GetFloor()
        {
            return floor;
        }
    };
}