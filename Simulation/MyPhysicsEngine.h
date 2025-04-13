#pragma once

#include "BasicActors.h"
#include "CustomActors.h"
#include "SZ_Cylinder.h"
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
			GROUND		= (1 << 0),
			HOUSE		= (1 << 1),
			ACTOR2		= (1 << 2)
			//add more if you need
		};
	};


	///A customised collision class, implemneting various callbacks
	class MySimulationEventCallback : public PxSimulationEventCallback
	{
	public:
		//an example variable that will be checked in the main simulation loop
		bool trigger;
		bool fallen;

		MySimulationEventCallback() : trigger(false), fallen(false) {}

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
						cerr << "onTrigger::eNOTIFY_TOUCH_FOUND" << endl;
						trigger = true;
					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_LOST" << endl;
						trigger = false;
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
		Plane* plane;
		Box* box, * box2;
		MySimulationEventCallback* my_callback;
		Character* player;
		Cloth* curtain;

		PxRigidDynamic* treeHouseRB;
		Cabin* house;

		bool isBroken;
		
	public:
		vector<SZ_Cylinder*> logs;
		//specify your custom filter shader here
		//PxDefaultSimulationFilterShader by default
		MyScene() : Scene(CustomFilterShader) {};

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
			plane = new Plane();
			plane->Name("floor");
			plane->GetShape()->setSimulationFilterData(groundFilterData);

			plane->Color(PxVec3(39.0f/255.0f, 174.0/255.0f, 96.0f/255.0f));
			Add(plane);

			// Specify Height in Meters
			player = new Character(PxReal(1.75f));
			Add(player, color_palette[0], Entity::ECharacter);

			Tree* tree = new Tree();
			Add(tree, color_palette[0], Entity::ETree);

			PxFilterData houseFilterData;
			houseFilterData.word0 = FilterGroup::HOUSE;
			houseFilterData.word1 = FilterGroup::GROUND;

			house = new Cabin(PxTransform(PxVec3(10.0f, 10.0f, 0.f)));
			house->Color(PxVec3(0.6f, 0.34509803921568627f, 0.16470588235294117f));
			for (int i = 41; i <= 65; ++i)
			{
				house->Color(PxVec3(0.2627450980392157f, 0.1568627450980392f, 0.09411764705882353f), i);
			}
			house->Name("house");
			treeHouseRB = house->Get()->is<PxRigidDynamic>();

			house->GetShape()->setSimulationFilterData(houseFilterData);

			Add(house);

			FixedJoint* joint = new FixedJoint(
				tree->getTrunkParts()[tree->getTrunkParts().size() - 1],
				PxTransform(PxVec3(5.0f, 5.0f, 0)),
				house,
				PxTransform(PxVec3(0, -1.0f, 0))
			);

			SZ_Cylinder* branch = new SZ_Cylinder(PxTransform(PxVec3(0.0f, 0.0f, 0.0f)), PxReal(0.1), PxReal(2.3), PxReal(10.0f));
			branch->Color(PxVec3(105 / 255.0f, 75 / 255.0f, 55 / 255.0f));
			Add(branch);

			FixedJoint* j = new FixedJoint(
				tree->getTrunkParts()[tree->getTrunkParts().size() - 1],
				PxTransform(PxVec3(5.0f, 4.0f, 0.0f)),
				branch,
				PxTransform(PxVec3(0.0f, 2.3f, 0.0f), PxQuat(2*(PxPi/3), PxVec3(1, 0,0)))
			);
			j->Get()->setBreakForce(9000000.0f, 9000000.0f);

			SZ_Cylinder* branch1 = new SZ_Cylinder(PxTransform(PxVec3(0.0f, 0.0f, 0.0f)), PxReal(0.1), PxReal(2.3), PxReal(10.0f));
			branch1->Color(PxVec3(105 / 255.0f, 75 / 255.0f, 55 / 255.0f));
			Add(branch1);

			FixedJoint* j1 = new FixedJoint(
				tree->getTrunkParts()[tree->getTrunkParts().size() - 1],
				PxTransform(PxVec3(5.0f, 3.0f, 0.0f)),
				branch1,
				PxTransform(PxVec3(0.0f, 2.3f, 0.0f), PxQuat(2 * (PxPi / 3), PxVec3(0, 0, 1)))
			);
			j1->Get()->setBreakForce(210000.0f, 210000.0f);

			SZ_Cylinder* branch2 = new SZ_Cylinder(PxTransform(PxVec3(0.0f, 0.0f, 0.0f)), PxReal(0.1), PxReal(2.3), PxReal(10.0f));
			branch2->Color(PxVec3(105 / 255.0f, 75 / 255.0f, 55 / 255.0f));
			Add(branch2);

			FixedJoint* j2 = new FixedJoint(
				tree->getTrunkParts()[tree->getTrunkParts().size() - 1],
				PxTransform(PxVec3(5.0f, 1.5f, 0.0f), PxQuat(PxPi, PxVec3(0, 0, 1))),
				branch2,
				PxTransform(PxVec3(0.0f, 2.3f, 0.0f), PxQuat(4*(PxPi / 3), PxVec3(0, 0, 1)) * PxQuat(PxPi, PxVec3(0, 0, 1)))
			);
			j2->Get()->setBreakForce(210000.0f, 210000.0f);
		}

		void BreakHouse()
		{
			if (!treeHouseRB)
			{
				cerr << "Error: treeHouse is null in breakHouse" << endl;
				return;
			}
			if (!treeHouseRB->is<PxRigidDynamic>())
			{
				cerr << "Error: treeHouse is not a valid PxRigidDynamic" << endl;
				return;
			}
			std::cout << "BreakHouse()::Breaking House" << std::endl;
			PxTransform houseTransform = treeHouseRB->getGlobalPose();
			Remove(house);

			for (int i = 0; i < 5; ++i)
			{
				WallSegment* wall = new WallSegment(
					PxTransform(houseTransform.p + PxVec3((float)(i - 2) * 0.5f, 0.0f, 0.0f))
				);
				Add(wall);
			}
		}

		void MovePlayerLeft()
		{
			player->updatePosition(
				PxVec3(-.5f, 0.0f, 0.0f), 
				PxQuat(PxPi / 2, PxVec3(0.0f, 1.0f, 0.0f))
			);
		}

		void MovePlayerRight()
		{
			player->updatePosition(
				PxVec3(.5f, 0.0f, 0.0f),
				PxQuat(-PxPi / 2, PxVec3(0.0f, 1.0f, 0.0f))
			); 
		}

		void MovePlayerUp()
		{
			player->updatePosition(
				PxVec3(0.0f, 0.0f, -0.5f),
				PxQuat(0.0f, PxVec3(0.0f, 1.0f, 0.0f))
			);
		}

		void MovePlayerDown()
		{
			player->updatePosition(
				PxVec3(0.0f, 0.0f, 0.5f), 
				PxQuat(0.0f, PxVec3(0.0f, 1.0f, 0.0f))
			); 
		}

		//Custom udpate function
		virtual void CustomUpdate() 
		{

			if (my_callback->fallen && !isBroken)
			{
				std::cout << "the house is totalled" << std::endl;
				isBroken = true;
				BreakHouse();
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
	};
}
