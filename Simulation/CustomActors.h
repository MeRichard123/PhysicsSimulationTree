
#include "BasicActors.h"
#include "PhysicsEngine.h"
#include "vector"
#include "RC_Cylinder.h"
#include <iomanip>
#include <random>

namespace PhysicsEngine {
    class Character : public DynamicActor {
    private:
        PxReal m_height;
        int m_currentPoseIndex = 0;

    public:
        Character(PxTransform pose = PxTransform(PxIdentity), PxReal height = PxReal(1.75f))
            : DynamicActor(pose), m_height(height)
        {
            PxReal headSize = height / 8.0f;              // Head is 1/8 of total height
            PxReal torsoHeight = headSize * 3.0f;         // Torso is 3 head lengths
            PxReal torsoWidth = headSize * 1.5f;          // Torso width is 1.5 head lengths
            PxReal limbWidth = headSize * 0.3f;           // Limb width is 0.3 head lengths
            PxReal armLength = headSize * 3.0f;           // Arm length is 3 head lengths
            PxReal legLength = headSize * 4.0f;           // Leg length is 4 head lengths
            
            // === Create torso (main body) ===
            CreateShape(PxBoxGeometry(PxVec3(torsoHeight / 4, torsoHeight / 2, torsoHeight / 8)), 1.0f);
            GetShape(0)->setLocalPose(PxTransform(PxVec3(0.0f, legLength + (torsoHeight / 2) + 0.01f, 0.0f)));

            SetKinematic(true);

            // === Head === 
            CreateShape(PxCapsuleGeometry(headSize / 1.5f, headSize / 3.5f), 1.0f);
            GetShape(1)->setLocalPose(PxTransform(PxVec3(0.0f, legLength + torsoHeight + headSize, 0.0f), PxQuat(PxPi / 2, PxVec3(0, 0, 1))));

            // === Legs ===
            CreateShape(CylinderGeometry(torsoHeight/10, legLength/2), 1.0f);
            GetShape(2)->setLocalPose(PxTransform(PxVec3(-(torsoHeight / 9), (legLength / 2), 0.0f)));

            CreateShape(CylinderGeometry(torsoHeight / 10, legLength / 2), 1.0f);
            GetShape(3)->setLocalPose(
                PxTransform(
                    PxVec3((torsoHeight / 9), (legLength / 2), 0.0f)
                )
            );

            // === Arms ===
            CreateShape(CylinderGeometry(torsoHeight/12, armLength/2), 1.0f);
            GetShape(4)->setLocalPose(
                PxTransform(
                    PxVec3(-(torsoHeight / 3), legLength + (torsoHeight * 0.6), - .15f), 
                    PxQuat(PxPi / 4, PxVec3(1, 0, 0))
                  )
            );

            CreateShape(CylinderGeometry(torsoHeight / 12, armLength / 2), 1.0f);
            GetShape(5)->setLocalPose(
                PxTransform(
                    PxVec3((torsoHeight / 4) - .05f, legLength + (torsoHeight * 0.6), -.25f),
                    PxQuat(PxPi / 4, PxVec3(1, 0, 0)) * PxQuat(-PxPi/6, PxVec3(0,0,1))
                )
            );

            // === Chainsaw ===
            CreateShape(PxBoxGeometry(0.2448f, 0.1794f, 0.1794f), 900.0f);
            GetShape(6)->setLocalPose(
                PxTransform(PxVec3(-0.2, 1.0, -.5), PxQuat(PxPi/2, PxVec3(0,1,0)))
            );

            CreateShape(PxBoxGeometry(0.4572f, 0.0762f, 0.0127f), 7860.0f);
            GetShape(7)->setLocalPose(
                PxTransform(PxVec3(-.05, 1.0, -1.2), PxQuat(PxPi / 2, PxVec3(0, 1, 0)))
            );
        }

        void updatePosition(PxVec3& delta, PxQuat& rot)
        {
            PxReal headSize = m_height / 8.0f;              
            PxReal torsoHeight = headSize * 3.0f;        
            PxReal legLength = headSize * 4.0f;       
            std::vector<PxTransform> legPoses1 = {
                PxTransform(PxVec3(-(torsoHeight / 9), (legLength / 2), 0.0f)),
                PxTransform(PxVec3(-(torsoHeight / 9), (legLength / 2) + 0.1f, -.4f), PxQuat(PxPi / 4, PxVec3(1, 0, 0))),
            };
            std::vector<PxTransform> logPoses2 = {
                PxTransform(PxVec3((torsoHeight / 9), (legLength / 2), 0.0f)),
                PxTransform(PxVec3((torsoHeight / 9), (legLength / 2) + 0.1f, .3f), PxQuat(-(PxPi / 4), PxVec3(1, 0, 0))),
            };

            PxTransform currentLegPoseLeft = legPoses1[m_currentPoseIndex];
            PxTransform currentLegPoseRight = logPoses2[m_currentPoseIndex];
        
            PxTransform currentPose = Get()->is<PxRigidActor>()->getGlobalPose();
            PxVec3 newPosition = currentPose.p + delta;
            GetShape(2)->setLocalPose(currentLegPoseLeft);
            GetShape(3)->setLocalPose(currentLegPoseRight);

            PxTransform newPos = PxTransform(newPosition, rot);
            Get()->is<PxRigidDynamic>()->setGlobalPose(newPos);

            m_currentPoseIndex = (m_currentPoseIndex + 1) % legPoses1.size();
        }
    };

    class DynamicTreePart : public DynamicActor
    {
    public:
        DynamicTreePart(PxTransform pose = PxTransform(PxIdentity), PxReal baseRadius = 2.f, PxReal topRadius = 1.f, PxReal height = 1.f)
            : DynamicActor(pose),
            m_verts(GenerateTaperedCylinderVertices(baseRadius, topRadius, height, 16))
        {
            PxConvexMesh* convexMesh = CreateConvexMesh(m_verts);
            if (!convexMesh) throw std::runtime_error("Convex mesh creation failed.");

            CreateShape(PxConvexMeshGeometry(convexMesh), 600.0f);
            GetShape(0)->setLocalPose(pose);
        }

        PxConvexMeshGeometry GetGeometry()
        {
            return PxConvexMeshGeometry(m_convexMesh);
        }

    private:
        PxConvexMesh* m_convexMesh = nullptr;
        std::vector<PxVec3> m_verts;

        std::vector<PxVec3> GenerateTaperedCylinderVertices(PxReal baseRadius, PxReal topRadius, PxReal height, int numSides)
        {
            std::vector<PxVec3> verts;
            float angleStep = PxTwoPi / numSides;
            float halfHeight = height / 2;

            for (int i = 0; i < numSides; i++)
            {
                float angle = i * angleStep;
                float x = baseRadius * cos(angle);
                float z = baseRadius * sin(angle);
                verts.push_back(PxVec3(x, -halfHeight, z));
            }

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

        PxConvexMesh* CreateConvexMesh(const std::vector<PxVec3>& vertices)
        {
            PxConvexMeshDesc convexDesc;
            convexDesc.points.count = static_cast<PxU32>(vertices.size());
            convexDesc.points.stride = sizeof(PxVec3);
            convexDesc.points.data = vertices.data();
            convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

            PxDefaultMemoryOutputStream buf;
            if (!GetCooking()->cookConvexMesh(convexDesc, buf))
                return nullptr;

            PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
            return GetPhysics()->createConvexMesh(input);
        }
    };

    class Tree : public DynamicActor
    {
    private:
        vector<DynamicTreePart*> m_trunkParts;
        vector<RC_Cylinder*> m_parts;
    public:
        DynamicTreePart* treeBase;
        FixedJoint* trunkBase;
        float bottomRad = 1.0;
        float topRad = bottomRad /= 1.5;
        float posY = 0.3;
        PxReal density = PxReal(600.0f);

        float height = 8.0f;

        Tree(PxTransform pose = PxTransform(PxIdentity))
            : DynamicActor(pose)
        {
            treeBase = new DynamicTreePart(PxTransform(PxVec3(5.f, posY, 0.f)), bottomRad, topRad, 1.25f);
            treeBase->SetKinematic(true);
            treeBase->Color(PxVec3(105 / 255.0f, 75 / 255.0f, 55 / 255.0f));  // Brown color (105,75,55)
            
            m_trunkParts.push_back(treeBase);

            CreateTrunk();

            AddBranches();
        }

        vector<RC_Cylinder*> getParts()
        {
            return m_parts;
        }

        vector<DynamicTreePart*> getTrunkParts()
        {
            return m_trunkParts;
        }

        void CreateBranch(Box* branch, int depth = 0, int side = -1) {
#if 0
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


            m_parts.push_back(b);

            depth++;
            side *= -1;

            CreateBranch(b, depth, side);
            CreateBranch(b, depth, side);
            //}
#endif
        }

        void CreateTrunk()
        {
            float topPoint = 0.323265;
            float trunkHeight = height;
                        
            posY += 2.34f;
            bottomRad= topRad;
            DynamicTreePart* trunk = new DynamicTreePart(PxTransform(PxVec3(5.f, posY, 0.f)), bottomRad, topPoint, trunkHeight);
            trunk->Color(PxVec3(105 / 255.0f, 75 / 255.0f, 55 / 255.0f));  // Brown color (105,75,55)
            m_trunkParts.push_back(trunk);


            trunkBase = new FixedJoint(
                treeBase,
                PxTransform(PxVec3(5.0f, 1.0f, 0.0f)),
                trunk,
                PxTransform(5.0f, -trunkHeight/8 - 0.27f, 0.0f)
            );
            trunkBase->Get()->setBreakForce(30000000.0f,30000000.0f);
        }

        void AddBranches()
        {
            RC_Cylinder* branch = new RC_Cylinder(PxTransform(PxVec3(0.0f, 0.0f, 0.0f)), PxReal(0.1), PxReal(2.3), PxReal(10.0f));
            branch->Color(PxVec3(105 / 255.0f, 75 / 255.0f, 55 / 255.0f));

            FixedJoint* j = new FixedJoint(
                getTrunkParts()[getTrunkParts().size() - 1],
                PxTransform(PxVec3(5.0f, 4.0f, 0.0f)),
                branch,
                PxTransform(PxVec3(0.0f, 2.3f, 0.0f), PxQuat(2 * (PxPi / 3), PxVec3(1, 0, 0)))
            );
            j->Get()->setBreakForce(9000000.0f, 9000000.0f);
            m_parts.push_back(branch);


            RC_Cylinder* branch1 = new RC_Cylinder(PxTransform(PxVec3(0.0f, 0.0f, 0.0f)), PxReal(0.1), PxReal(2.3), PxReal(10.0f));
            branch1->Color(PxVec3(105 / 255.0f, 75 / 255.0f, 55 / 255.0f));

            FixedJoint* j1 = new FixedJoint(
                getTrunkParts()[getTrunkParts().size() - 1],
                PxTransform(PxVec3(5.0f, 3.0f, 0.0f)),
                branch1,
                PxTransform(PxVec3(0.0f, 2.3f, 0.0f), PxQuat(2 * (PxPi / 3), PxVec3(0, 0, 1)))
            );
            j1->Get()->setBreakForce(210000.0f, 210000.0f);
            m_parts.push_back(branch1);

            RC_Cylinder* branch2 = new RC_Cylinder(PxTransform(PxVec3(0.0f, 0.0f, 0.0f)), PxReal(0.1), PxReal(2.3), PxReal(10.0f));
            branch2->Color(PxVec3(105 / 255.0f, 75 / 255.0f, 55 / 255.0f));

            FixedJoint* j2 = new FixedJoint(
                getTrunkParts()[getTrunkParts().size() - 1],
                PxTransform(PxVec3(5.0f, 1.5f, 0.0f), PxQuat(PxPi, PxVec3(0, 0, 1))),
                branch2,
                PxTransform(PxVec3(0.0f, 2.3f, 0.0f), PxQuat(4 * (PxPi / 3), PxVec3(0, 0, 1)) * PxQuat(PxPi, PxVec3(0, 0, 1)))
            );
            j2->Get()->setBreakForce(210000.0f, 210000.0f);
            m_parts.push_back(branch2);
        }
    };

    class WallSegment : public DynamicActor
    {
    public:
        WallSegment(PxTransform pose = PxTransform(PxIdentity), bool flip = false)
            : DynamicActor(pose)
        {
            float radius = 0.1f;
            float halfHeight = 1.0f;
            float density = 300.0f;
            float diameter = radius * 2;
            float quarter = halfHeight / 4;

            if (!flip)
            {
                CreateShape(CylinderGeometry(PxReal(radius), PxReal(halfHeight)), density);
                CreateShape(CylinderGeometry(PxReal(radius), PxReal(halfHeight)), density);
                GetShape(0)->setLocalPose(PxTransform(PxVec3(0, 0, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));
                PxQuat rotation90(PxPi / 2.0f, PxVec3(0, 1, 0)); // 90° around Z
                PxVec3 offset(halfHeight, 0, halfHeight);
                PxTransform localPose(offset, rotation90 * PxQuat(PxPi / 2, PxVec3(1, 0, 0)));
                GetShape(1)->setLocalPose(localPose);

                CreateShape(CylinderGeometry(PxReal(radius), PxReal(halfHeight - quarter)), density);
                CreateShape(CylinderGeometry(PxReal(radius), PxReal(halfHeight - quarter)), density);
                GetShape(2)->setLocalPose(PxTransform(PxVec3(0, diameter, quarter), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));
                PxVec3 offset2(halfHeight - quarter, diameter, halfHeight);
                PxTransform localPose2(offset2, rotation90 * PxQuat(PxPi / 2, PxVec3(1, 0, 0)));
                GetShape(3)->setLocalPose(localPose2);

                CreateShape(CylinderGeometry(PxReal(radius), PxReal(halfHeight - (2 * quarter))), density);
                CreateShape(CylinderGeometry(PxReal(radius), PxReal(halfHeight - (2 * quarter))), density);
                GetShape(4)->setLocalPose(PxTransform(PxVec3(0, 2 * diameter, 2 * quarter), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));
                PxVec3 offset3(halfHeight - (2 * quarter), 2 * diameter, halfHeight);
                PxTransform localPose3(offset3, rotation90 * PxQuat(PxPi / 2, PxVec3(1, 0, 0)));
                GetShape(5)->setLocalPose(localPose3);
            }
            else 
            {
                PxQuat rotation90(PxPi / 2.0f, PxVec3(0, 1, 0)); // 90° around Z
                CreateShape(CylinderGeometry(PxReal(radius), PxReal(halfHeight - (2 * quarter))), density);
                CreateShape(CylinderGeometry(PxReal(radius), PxReal(halfHeight - (2 * quarter))), density);
                GetShape(0)->setLocalPose(PxTransform(PxVec3(0, 0, 2 * quarter), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));
                PxVec3 offset0(halfHeight - (2 * quarter), 0, halfHeight);
                PxTransform localPose0(offset0, rotation90 * PxQuat(PxPi / 2, PxVec3(1, 0, 0)));
                GetShape(1)->setLocalPose(localPose0);

                // Create the middle cylinders
                CreateShape(CylinderGeometry(PxReal(radius), PxReal(halfHeight - quarter)), density);
                CreateShape(CylinderGeometry(PxReal(radius), PxReal(halfHeight - quarter)), density);
                GetShape(2)->setLocalPose(PxTransform(PxVec3(0, diameter, quarter), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));
                PxVec3 offset1(halfHeight - quarter, diameter, halfHeight);
                PxTransform localPose1(offset1, rotation90 * PxQuat(PxPi / 2, PxVec3(1, 0, 0)));
                GetShape(3)->setLocalPose(localPose1);

                // Create the smaller cylinders at the bottom last
                CreateShape(CylinderGeometry(PxReal(radius), PxReal(halfHeight)), density);
                CreateShape(CylinderGeometry(PxReal(radius), PxReal(halfHeight)), density);
                GetShape(4)->setLocalPose(PxTransform(PxVec3(0, 2 * diameter, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));
                PxVec3 offset2(halfHeight, 2 * diameter, halfHeight);
                PxTransform localPose2(offset2, rotation90 * PxQuat(PxPi / 2, PxVec3(1, 0, 0)));
                GetShape(5)->setLocalPose(localPose2);
            }
        }
    };

    class Cabin : public DynamicActor
    {
    public:
        vector<WallSegment*> logs;
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
            float density = 300.0f;


            PxVec3 floorLocalPos(0, floorThickness / 2.f, 0);
            PxTransform floorTransform = pose * PxTransform(floorLocalPos);
            floor = new Box(floorTransform, PxVec3(logLength / 2, floorThickness / 2, logLength / 2));
            CreateShape(PxBoxGeometry(PxVec3(logLength / 2, floorThickness, logLength / 2)), 10.0f);

            // Layer 1
            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength/2)), density);
            GetShape(1)->setLocalPose(PxTransform(PxVec3(logLength/2 - logRadius, logRadius,0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2)-0.2f)), density);
            GetShape(2)->setLocalPose(
                PxTransform(
                    PxVec3(0, logRadius, logLength / 2 - logRadius),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0)) * PxQuat(PxPi/2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(3)->setLocalPose(PxTransform(PxVec3(-1*(logLength / 2 - logRadius), logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - 0.2f)), density);
            GetShape(4)->setLocalPose(
                PxTransform(
                    PxVec3(0, logRadius, -1*(logLength / 2 - logRadius)),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0)) * PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            // Layer 2

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(5)->setLocalPose(PxTransform(PxVec3(logLength / 2 - logRadius, logRadius+logDiameter, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - 0.2f)), density);
            GetShape(6)->setLocalPose(
                PxTransform(
                    PxVec3(0, logRadius + logDiameter, logLength / 2 - logRadius),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0)) * PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(7)->setLocalPose(PxTransform(PxVec3(-1 * (logLength / 2 - logRadius), logRadius+logDiameter, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - 0.2f)), density);
            GetShape(8)->setLocalPose(
                PxTransform(
                    PxVec3(0, logRadius+logDiameter, -1 * (logLength / 2 - logRadius)),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0)) * PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            // Layer 3 

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(9)->setLocalPose(PxTransform(PxVec3(logLength / 2 - logRadius, logDiameter + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - 0.2f)), density);
            GetShape(10)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter + logRadius, logLength / 2 - logRadius),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0)) * PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(11)->setLocalPose(PxTransform(PxVec3(-1 * (logLength / 2 - logRadius), logDiameter + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - 0.2f)), density);
            GetShape(12)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter + logRadius, -1 * (logLength / 2 - logRadius)),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0)) * PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            // Layer 4

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(13)->setLocalPose(PxTransform(PxVec3(logLength / 2 - logRadius, logDiameter + logDiameter + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - 0.2f)), density);
            GetShape(14)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter + logDiameter + logRadius, logLength / 2 - logRadius),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0)) * PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(15)->setLocalPose(PxTransform(PxVec3(-1 * (logLength / 2 - logRadius), logDiameter + logDiameter + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - 0.2f)), density);
            GetShape(16)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter + logDiameter + logRadius, -1 * (logLength / 2 - logRadius)),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0)) * PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            // Layer 5

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(17)->setLocalPose(PxTransform(PxVec3(logLength / 2 - logRadius, logDiameter * 3 + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - 0.2f)), density);
            GetShape(18)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter * 3 + logRadius, logLength / 2 - logRadius),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(19)->setLocalPose(PxTransform(PxVec3(-1 * (logLength / 2 - logRadius), logDiameter * 3 + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 5) - 0.2f)), density);
            GetShape(20)->setLocalPose(
                PxTransform(
                    PxVec3(logLength/2 - logLength/5, logDiameter * 3 + logRadius, -1 * (logLength / 2 - logRadius)),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 5) - 0.2f)), density);
            GetShape(21)->setLocalPose(
                PxTransform(
                    PxVec3(-logLength / 2 + logLength / 5, logDiameter * 3 + logRadius, -1 * (logLength / 2 - logRadius)),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            // Layer 6

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(22)->setLocalPose(PxTransform(PxVec3(logLength / 2 - logRadius, logDiameter * 4 + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - 0.2f)), density);
            GetShape(23)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter * 4 + logRadius, logLength / 2 - logRadius),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(24)->setLocalPose(PxTransform(PxVec3(-1 * (logLength / 2 - logRadius), logDiameter * 4 + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 5) - 0.2f)), density);
            GetShape(25)->setLocalPose(
                PxTransform(
                    PxVec3(logLength / 2 - logLength / 5, logDiameter * 4 + logRadius, -1 * (logLength / 2 - logRadius)),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 5) - 0.2f)), density);
            GetShape(26)->setLocalPose(
                PxTransform(
                    PxVec3(-logLength / 2 + logLength / 5, logDiameter * 4 + logRadius, -1 * (logLength / 2 - logRadius)),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            // Layer 7

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(27)->setLocalPose(PxTransform(PxVec3(logLength / 2 - logRadius, logDiameter * 5 + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - 0.2f)), density);
            GetShape(28)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter * 5 + logRadius, logLength / 2 - logRadius),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(29)->setLocalPose(PxTransform(PxVec3(-1 * (logLength / 2 - logRadius), logDiameter * 5 + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 5) - 0.2f)), density);
            GetShape(30)->setLocalPose(
                PxTransform(
                    PxVec3(logLength / 2 - logLength / 5, logDiameter * 5 + logRadius, -1 * (logLength / 2 - logRadius)),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 5) - 0.2f)), density);
            GetShape(31)->setLocalPose(
                PxTransform(
                    PxVec3(-logLength / 2 + logLength / 5, logDiameter * 5 + logRadius, -1 * (logLength / 2 - logRadius)),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            // Layer 8

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(32)->setLocalPose(PxTransform(PxVec3(logLength / 2 - logRadius, logDiameter * 6 + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - 0.2f)), density);
            GetShape(33)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter * 6 + logRadius, logLength / 2 - logRadius),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(34)->setLocalPose(PxTransform(PxVec3(-1 * (logLength / 2 - logRadius), logDiameter * 6 + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 5) - 0.2f)), density);
            GetShape(35)->setLocalPose(
                PxTransform(
                    PxVec3(logLength / 2 - logLength / 5, logDiameter * 6 + logRadius, -1 * (logLength / 2 - logRadius)),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 5) - 0.2f)), density);
            GetShape(36)->setLocalPose(
                PxTransform(
                    PxVec3(-logLength / 2 + logLength / 5, logDiameter * 6 + logRadius, -1 * (logLength / 2 - logRadius)),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            // Layer 9
            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(37)->setLocalPose(PxTransform(PxVec3(logLength / 2 - logRadius, logDiameter * 7 + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - 0.2f)), density);
            GetShape(38)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter * 7 + logRadius, logLength / 2 - logRadius),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(39)->setLocalPose(PxTransform(PxVec3(-1 * (logLength / 2 - logRadius), logDiameter * 7 + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - 0.2f)), density);
            GetShape(40)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter * 7 + logRadius, -1 * (logLength / 2 - logRadius)),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            // Layer 10 - ROOF
       
            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2))), density);
            GetShape(41)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter * 8 + logRadius, logLength / 2 - logRadius),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2))), density);
            GetShape(42)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter * 8 + logRadius, -1 * (logLength / 2 - logRadius)),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(43)->setLocalPose(PxTransform(PxVec3((logLength / 2 - logRadius)+logDiameter, logDiameter * 8 + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(44)->setLocalPose(PxTransform(PxVec3(-1 * (logLength / 2 - logRadius + logDiameter), logDiameter * 8 + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            // Layer 11
            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - ((logLength / 2) * 0.125f))), density);
            GetShape(45)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter * 9 + logRadius, logLength / 2 - logRadius),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - ((logLength / 2) * 0.125f))), density);
            GetShape(46)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter * 9 + logRadius, -1 * (logLength / 2 - logRadius)),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(47)->setLocalPose(PxTransform(PxVec3(logLength / 2 - logRadius, logDiameter * 9 + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(48)->setLocalPose(PxTransform(PxVec3(-1 * (logLength / 2 - logRadius), logDiameter * 9 + logRadius, 0), PxQuat(PxPi / 2, PxVec3(1, 0, 0))));


            // Layer 12
            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - ((logLength / 2) * 0.25f))), density);
            GetShape(49)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter * 10 + logRadius, logLength / 2 - logRadius),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - ((logLength / 2) * 0.25f))), density);
            GetShape(50)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter * 10 + logRadius, -1 * (logLength / 2 - logRadius)),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(51)->setLocalPose(
                PxTransform(
                    PxVec3((logLength / 2 - logRadius) - logDiameter, logDiameter * 10 + logRadius, 0), 
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(52)->setLocalPose(
                PxTransform(
                    PxVec3(-1 * (logLength / 2 - logRadius - logDiameter), logDiameter * 10 + logRadius, 0), 
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))
                )
            );

            // Layer 13
            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - ((logLength / 2) * 0.4f))), density);
            GetShape(53)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter * 11 + logRadius, logLength / 2 - logRadius),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal((logLength / 2) - ((logLength / 2) * 0.4f))), density);
            GetShape(54)->setLocalPose(
                PxTransform(
                    PxVec3(0, logDiameter * 11 + logRadius, -1 * (logLength / 2 - logRadius)),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))* PxQuat(PxPi / 2, PxVec3(0, 0, 1))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(55)->setLocalPose(
                PxTransform(
                    PxVec3((logLength / 2 - logRadius) - (2*logDiameter), logDiameter * 11 + logRadius, 0),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(56)->setLocalPose(
                PxTransform(
                    PxVec3(-1 * (logLength / 2 - logRadius - (2*logDiameter)), logDiameter * 11 + logRadius, 0), 
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(57)->setLocalPose(
                PxTransform(
                    PxVec3((logLength / 2 - logRadius) - (3 * logDiameter), logDiameter * 12 + logRadius, 0), 
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(58)->setLocalPose(
                PxTransform(
                    PxVec3((logLength / 2 - logRadius) - (4 * logDiameter), logDiameter * 12 + logRadius, 0), 
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(59)->setLocalPose(
                PxTransform(
                    PxVec3((logLength / 2 - logRadius) - (5 * logDiameter), logDiameter * 12 + logRadius, 0), 
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(60)->setLocalPose(
                PxTransform(
                    PxVec3((logLength / 2 - logRadius) - (6 * logDiameter), logDiameter * 12 + logRadius, 0), 
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(61)->setLocalPose(
                PxTransform(
                    PxVec3(-1 * (logLength / 2 - logRadius - (7 * logDiameter)), logDiameter * 12 + logRadius, 0), 
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(62)->setLocalPose(
                PxTransform(
                    PxVec3((logLength / 2 - logRadius) - (8 * logDiameter), logDiameter * 12 + logRadius, 0),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(63)->setLocalPose(
                PxTransform(
                    PxVec3((logLength / 2 - logRadius) - (9 * logDiameter), logDiameter * 12 + logRadius, 0),
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(64)->setLocalPose(
                PxTransform(
                    PxVec3((logLength / 2 - logRadius) - (10 * logDiameter), logDiameter * 12 + logRadius, 0), 
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))
                )
            );

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength / 2)), density);
            GetShape(65)->setLocalPose(
                PxTransform(
                    PxVec3((logLength / 2 - logRadius) - (11 * logDiameter), logDiameter * 12 + logRadius, 0), 
                    PxQuat(PxPi / 2, PxVec3(1, 0, 0))
                )
            );
        }

        vector<WallSegment*> GetLogs()
        {
            return logs;
        }

        Box* GetFloor()
        {
            return floor;
        }
    };

    class RoofSegment : public DynamicActor
    {
    public:
        RoofSegment(PxTransform pose = PxTransform(PxIdentity), float length = 3.0f)
            : DynamicActor(pose)
        {
            float logLength = length;
            float logRadius = 0.1f;
            float spacing = logRadius * 2.f;
            float logGap = 0.01f;
            float windowWidth = .2f;
            float windowHeight = .5f;
            float logDiameter = logRadius * 2.f;
            float floorThickness = 0.01f;
            float density = 300.0f;

            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength)), density);
            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength)), density);
            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength)), density);
            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength)), density);
            CreateShape(CylinderGeometry(PxReal(logRadius), PxReal(logLength)), density);


            GetShape(1)->setLocalPose(PxTransform(PxVec3(logDiameter + logGap, 0, 0)));
            GetShape(2)->setLocalPose(PxTransform(PxVec3((2 * logDiameter) + logGap, 0, 0)));
            GetShape(3)->setLocalPose(PxTransform(PxVec3((3 * logDiameter) + logGap, 0, 0)));
            GetShape(4)->setLocalPose(PxTransform(PxVec3((4 * logDiameter) + logGap, 0, 0)));
        }
    };
}