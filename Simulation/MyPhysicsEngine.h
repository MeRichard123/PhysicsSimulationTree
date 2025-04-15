#pragma once

#include "BasicActors.h"
#include "CustomActors.h"
#include "ParticleSystem.h"
#include "RC_Cylinder.h"
#include <iostream>
#include <iomanip>
#include <random>

namespace PhysicsEngine
{

	using namespace std;

	//a list of colours: Circus Palette
	static const PxVec3 color_palette[] = {PxVec3(46.f/255.f,9.f/255.f,39.f/255.f),PxVec3(217.f/255.f,0.f/255.f,0.f/255.f),
		PxVec3(255.f/255.f,45.f/255.f,0.f/255.f),PxVec3(255.f/255.f,140.f/255.f,54.f/255.f),PxVec3(4.f/255.f,117.f/255.f,111.f/255.f)};

	//pyramid vertices
	static PxVec3 pyramid_verts[] = {PxVec3(0,1,0), PxVec3(1,0,0), PxVec3(-1,0,0), PxVec3(0,0,1), PxVec3(0,0,-1)};
	//pyramid triangles: a list of three vertices for each triangle e.g. the first triangle consists of vertices 1, 4 and 0
	//vertices have to be specified in a counter-clockwise order to assure the correct shading in rendering
	static PxU32 pyramid_trigs[] = {1, 4, 0, 3, 1, 0, 2, 3, 0, 4, 2, 0, 3, 2, 1, 2, 4, 1};

	class Pyramid : public ConvexMesh
	{
	public:
		Pyramid(PxTransform pose=PxTransform(PxIdentity), PxReal density=1.f) :
			ConvexMesh(vector<PxVec3>(begin(pyramid_verts),end(pyramid_verts)), pose, density)
		{
		}
	};

	class PyramidStatic : public TriangleMesh
	{
	public:
		PyramidStatic(PxTransform pose=PxTransform(PxIdentity)) :
			TriangleMesh(vector<PxVec3>(begin(pyramid_verts),end(pyramid_verts)), vector<PxU32>(begin(pyramid_trigs),end(pyramid_trigs)), pose)
		{
		}
	};

	struct FilterGroup
	{
		enum Enum
		{
			GROUND = (1 << 0),
			HOUSE  = (1 << 1),
			PLAYER = (1 << 2),
			TREE   = (1 << 3),
			//add more if you need
		};
	};


	///A customised collision class, implementing various callbacks
	class MySimulationEventCallback : public PxSimulationEventCallback
	{
	public:
		bool trigger;
		bool fallen;
		bool chainsawTrigger;

		MySimulationEventCallback() : trigger(false), fallen(false), chainsawTrigger(false) {}

		///Method called when the contact with the trigger object is detected.
		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count) 
		{
			//you can read the trigger information here
			for (PxU32 i = 0; i < count; i++)
			{
				//filter out contact with the planes
				if (pairs[i].otherShape->getGeometryType() != PxGeometryType::ePLANE)
				{
					//check if eNOTIFY_TOUCH_FOUND trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
					{
						if (pairs[i].triggerActor->getName() == "tree_trigger" && pairs[i].otherActor->getName() == "player")
						{
							std::cout << "CONTACTTT" << std::endl;
							cerr << "onTrigger::eNOTIFY_TOUCH_FOUND - chainsaw" << endl;
							chainsawTrigger = true;
						}
					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						if (pairs[i].triggerActor->getName() == "tree_trigger" && pairs[i].otherActor->getName() == "player")
						{
							cerr << "onTrigger::eNOTIFY_TOUCH_LOST - chainsaw" << endl;
							chainsawTrigger = true;
						}
					}
				}
			}
		}

		///Method called when the contact by the filter shader is detected.
		virtual void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs) 
		{
			cerr << "Contact found between " << pairHeader.actors[0]->getName() << " " << pairHeader.actors[1]->getName() << endl;

			//check all pairs
			for (PxU32 i = 0; i < nbPairs; i++)
			{
				//check eNOTIFY_TOUCH_FOUND
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
				{
					cerr << "onContact::eNOTIFY_TOUCH_FOUND" << endl;
					PxContactPairPoint contactPoints[64];
					PxU32 nbContacts = pairs[i].extractContacts(contactPoints, 64);
					PxReal maxImpluse = 0.0f;

					for (PxU32 j = 0; j < nbContacts; ++j)
					{
						maxImpluse = PxMax(maxImpluse, contactPoints[j].impulse.magnitude());
					}

					std::cout << maxImpluse << std::endl;
					if (maxImpluse > 0.0f)
					{
						fallen = true;
					}
				}
				//check eNOTIFY_TOUCH_LOST
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_LOST)
				{
					cerr << "onContact::eNOTIFY_TOUCH_LOST" << endl;
				}
			}
		}

		virtual void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
		virtual void onWake(PxActor **actors, PxU32 count) {}
		virtual void onSleep(PxActor **actors, PxU32 count) {}
#if PX_PHYSICS_VERSION >= 0x304000
		virtual void onAdvance(const PxRigidBody *const *bodyBuffer, const PxTransform *poseBuffer, const PxU32 count) {}
#endif
	};

	//A simple filter shader based on PxDefaultSimulationFilterShader 
	static PxFilterFlags CustomFilterShader( PxFilterObjectAttributes attributes0,	PxFilterData filterData0,
		PxFilterObjectAttributes attributes1,	PxFilterData filterData1,
		PxPairFlags& pairFlags,	const void* constantBlock,	PxU32 constantBlockSize)
	{
		// let triggers through
		if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
		{
			pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
			return PxFilterFlags();
		}

		pairFlags = PxPairFlag::eCONTACT_DEFAULT;
		//enable continous collision detection
//		pairFlags |= PxPairFlag::eCCD_LINEAR;


		// trigger the contact callback for pairs (A,B) where 
		// the filtermask of A contains the ID of B and vice versa.
		if((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		{
			//trigger onContact callback for this pair of objects
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_LOST;
			pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
		}

		return PxFilterFlags();
	};

	///Custom scene class
	class MyScene : public Scene
	{
	private:
		Plane* m_plane;
		Box* m_box, *m_box2;
		Character* m_player;
		Cloth* m_curtain;
		Cabin* m_house;
		Tree* m_tree;

		PxRigidDynamic* m_treeHouseRB;
		Emitter* m_sawdustEmitter;
		MySimulationEventCallback* my_callback;
		bool m_isBroken;
		float m_timeElapsed;
		bool m_timeStarted;
		
	public:
		vector<RC_Cylinder*> logs;
		//specify your custom filter shader here
		//PxDefaultSimulationFilterShader by default
		MyScene() : Scene(CustomFilterShader), m_sawdustEmitter(nullptr) {};

		///A custom scene class
		void SetVisualisation()
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f);	
		}


		//Custom scene initialisation
		virtual void CustomInit()
		{
			SetVisualisation();

			GetMaterial()->setDynamicFriction(.2f);

			///Initialise and set the customised event callback
			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);

			PxFilterData groundFilterData;
			groundFilterData.word0 = FilterGroup::GROUND;
			groundFilterData.word1 = FilterGroup::HOUSE;
			m_plane = new Plane();
			m_plane->Name("floor");
			m_plane->GetShape()->setSimulationFilterData(groundFilterData);

			m_plane->Color(PxVec3(39.0f / 255.0f, 174.0 / 255.0f, 96.0f / 255.0f));
			Add(m_plane);

			// Specify Height in Meters
			m_player = new Character(PxTransform(-2.f, 0.0, 0.0), PxReal(1.75f));
			PxFilterData playerFilterData;
			playerFilterData.word0 = FilterGroup::PLAYER;
			playerFilterData.word1 = FilterGroup::GROUND | FilterGroup::TREE;
			m_player->Get()->setName("player");
			m_player->GetShape()->setSimulationFilterData(playerFilterData);
			m_player->Color(PxVec3(0.1568627450980392f, 0.21176470588235294f, 0.09411764705882353), 0);
			m_player->Color(PxVec3(0.8666666666666667f, 0.7215686274509804f, 0.5725490196078431f), 1);
			m_player->Color(PxVec3(0.011764705882352941f, 0.01568627450980392f, 0.3686274509803922f), 2);
			m_player->Color(PxVec3(0.011764705882352941f, 0.01568627450980392f, 0.3686274509803922f), 3);
			m_player->Color(PxVec3(0.1568627450980392f, 0.21176470588235294f, 0.09411764705882353), 4);
			m_player->Color(PxVec3(0.1568627450980392f, 0.21176470588235294f, 0.09411764705882353), 5);
			m_player->Color(PxVec3(1.0f, 0.29411764705882354f, 0.24313725490196078f), 6);
			m_player->Color(PxVec3(0.1803921568627451, 0.19215686274509805f, 0.2196078431372549f), 7);
			Add(m_player);

			m_tree = new Tree();
			PxFilterData treeFilterData;
			treeFilterData.word0 = FilterGroup::TREE;
			treeFilterData.word1 = FilterGroup::PLAYER;
			std::cout << m_tree->getTrunkParts().size() << std::endl;

			Add(m_tree, color_palette[0], Entity::ETree);

			// Add a Trigger
			PxPhysics* physics = GetPhysics();
			if (!physics)
			{
				cerr << "Error: GetPhysics() returned null!" << endl;
				return;
			}
			PxRigidDynamic* treeTrigger = GetPhysics()->createRigidDynamic(PxTransform(PxVec3(9.85f, 1.0f, -0.04f)));
			treeTrigger->setName("tree_trigger");
			if (!treeTrigger)
			{
				cerr << "Error: Failed to create treeTrigger!" << endl;
				return;
			}
			PxMaterial* material = GetMaterial();
			if (!material)
			{
				cerr << "Error: GetMaterial() returned null!" << endl;
				return;
			}
			PxShape* triggerShape = GetPhysics()->createShape(CylinderGeometry(.5f, .2f), *GetMaterial(), true);
			if (!triggerShape)
			{
				cerr << "Error: Failed to create triggerShape!" << endl;
				treeTrigger->release();
				return;
			}
			triggerShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
			triggerShape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);
			triggerShape->setSimulationFilterData(treeFilterData);
			treeTrigger->attachShape(*triggerShape);
			treeTrigger->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
			Add(treeTrigger);
			triggerShape->release();


			PxFilterData houseFilterData;
			houseFilterData.word0 = FilterGroup::HOUSE;
			houseFilterData.word1 = FilterGroup::GROUND;

			m_house = new Cabin(PxTransform(PxVec3(10.0f, 10.0f, 0.f)));
			m_house->Color(PxVec3(0.6f, 0.34509803921568627f, 0.16470588235294117f));
			for (int i = 41; i <= 65; ++i)
			{
				m_house->Color(PxVec3(0.2627450980392157f, 0.1568627450980392f, 0.09411764705882353f), i);
			}
			m_house->Name("house");
			m_treeHouseRB = m_house->Get()->is<PxRigidDynamic>();

			m_house->GetShape()->setSimulationFilterData(houseFilterData);

			Add(m_house);

			FixedJoint* joint = new FixedJoint(
				m_tree->getTrunkParts()[m_tree->getTrunkParts().size() - 1],
				PxTransform(PxVec3(5.0f, 5.0f, 0)),
				m_house,
				PxTransform(PxVec3(0, -1.0f, 0))
			);


			m_curtain = new Cloth(PxTransform(PxVec3(-4.f, 9.f, 0.f)), PxVec2(2.f, 1.f), 5, 5);
			//Add(curtain);
			//curtain->attach(0, treeHouseRB, PxVec3(-5.0f, 0.0, 0.0));;
		}

		void BreakHouse()
		{
			float logLength = 3.0f;
			float logRadius = 0.1f;
			float density = 300.0f;
			if (!m_treeHouseRB)
			{
				cerr << "Error: treeHouse is null in breakHouse" << endl;
				return;
			}
			if (!m_treeHouseRB->is<PxRigidDynamic>())
			{
				cerr << "Error: treeHouse is not a valid PxRigidDynamic" << endl;
				return;
			}
			std::cout << "BreakHouse()::Breaking House" << std::endl;
			PxTransform houseTransform = m_treeHouseRB->getGlobalPose();
			Remove(m_house);

			for (int i = 0; i < 6; ++i)
			{
				WallSegment* wall = new WallSegment(
					PxTransform(houseTransform.p + PxVec3((float)(i - 2) * 0.5f, 0.0f, 0.0f)),
					i % 2 == 0
				);
				wall->Color(PxVec3(0.6f, 0.34509803921568627f, 0.16470588235294117f));
				Add(wall);
			}
			for (int i = 0; i < 3; ++i)
			{
				RoofSegment* roof = new RoofSegment(PxTransform(houseTransform.p + PxVec3((float)(i + 2) * 0.5f, 0.0f, 0.0f)));
				for (int i = 0; i < 5; ++i)
				{
					roof->Color(PxVec3(0.2627450980392157f, 0.1568627450980392f, 0.09411764705882353f), i);
				}
				Add(roof);

				PxTransform pose = PxTransform(houseTransform.p + PxVec3((float)(i + 7) * 0.5f, 0.0f, 0.0f));
				RC_Cylinder* log = new RC_Cylinder(pose, logRadius, logLength, density);
				log->Color(PxVec3(0.2627450980392157f, 0.1568627450980392f, 0.09411764705882353f));
				Add(log);
			}

			for (int i = 1; i < 3; ++i)
			{
				PxTransform pose = PxTransform(houseTransform.p + PxVec3((float)(i - 5) * 0.5f, 0.0f, 0.0f));
				RoofSegment* wall = new RoofSegment(pose, 2.0f);
				wall->Color(PxVec3(0.6f, 0.34509803921568627f, 0.16470588235294117f));
				Add(wall);
				RC_Cylinder* log = new RC_Cylinder(pose, logRadius, logLength - 1.0f, density);
				log->Color(PxVec3(0.6f, 0.34509803921568627f, 0.16470588235294117f));
				Add(log);
			}
		}

		//Custom udpate function
		virtual void CustomUpdate(float dt) 
		{
			if (my_callback->fallen && !m_isBroken)
			{
				m_isBroken = true;
				BreakHouse();
			}
			
			if (my_callback->chainsawTrigger && m_tree->getTrunkParts().size() > 0)
			{
				if (!m_timeStarted)
				{
					std::cout << "Time started" << std::endl;
					m_timeElapsed = 0;
					m_timeStarted = true;
				}
				else
				{
					m_timeElapsed += dt;
				}
				PxRigidDynamic* trunk = m_tree->getTrunkParts()[1]->Get()->is<PxRigidDynamic>();
				PxVec3 forceDir = (trunk->getGlobalPose().p -
					m_player->Get()->is<PxRigidDynamic>()->getGlobalPose().p).getNormalized();
				if (!(m_tree->trunkBase->Get()->getConstraintFlags() & PxConstraintFlag::eBROKEN))
				{
					trunk->addForce(forceDir * ((20000.0f * m_timeElapsed)), PxForceMode::eIMPULSE);
					PxVec3 force = forceDir * (20000.0f * m_timeElapsed);
					std::cout << force.x << " " << force.y << " " << force.z << std::endl;

					if (!m_sawdustEmitter)
					{
						PxTransform emitterPos = PxTransform(PxVec3(10.f, 1.0f, -.5f));
						m_sawdustEmitter = new Emitter(emitterPos, PxReal(.5f), 500);
						Add(m_sawdustEmitter);
					}
					else {
						PxTransform emitterPos = PxTransform(PxVec3(10.f, 1.0f, -.5f));
						m_sawdustEmitter->Get()->is<PxRigidDynamic>()->setGlobalPose(emitterPos);
					}
				}
			}
			else if (m_sawdustEmitter)
			{
				Remove(m_sawdustEmitter);
				m_sawdustEmitter = nullptr;
			}

			if (m_sawdustEmitter)
			{
				m_sawdustEmitter->Update(dt);

				for (Particle* part : m_sawdustEmitter->getParticles())
				{
					if (!part->inScene)
					{
						Add(part);
						part->inScene = true;
					}
				}
				for (Particle* part : m_sawdustEmitter->getDeadParticles())
				{
					Remove(part);
				}
				m_sawdustEmitter->ClearDeadParticles();
			}
		}

		/// An example use of key release handling
		void ExampleKeyReleaseHandler()
		{
			cerr << "I am realeased!" << endl;
		}

		/// An example use of key presse handling
		void ExampleKeyPressHandler()
		{
			cerr << "I am pressed!" << endl;
		}

		void MovePlayerLeft()
		{
			m_player->updatePosition(
				PxVec3(.5f, 0.0f, 0.0f),
				PxQuat(-PxPi / 2, PxVec3(0.0f, 1.0f, 0.0f))
			);
		}

		void MovePlayerRight()
		{
			m_player->updatePosition(
				PxVec3(-.5f, 0.0f, 0.0f),
				PxQuat(PxPi / 2, PxVec3(0.0f, 1.0f, 0.0f))
			);
		}

		void MovePlayerUp()
		{
			m_player->updatePosition(
				PxVec3(0.0f, 0.0f, -0.5f),
				PxQuat(0.0f, PxVec3(0.0f, 1.0f, 0.0f))
			);
		}

		void MovePlayerDown()
		{
			m_player->updatePosition(
				PxVec3(0.0f, 0.0f, 0.5f),
				PxQuat(0.0f, PxVec3(0.0f, 1.0f, 0.0f))
			);
		}
	};
}
