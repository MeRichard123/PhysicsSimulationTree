#pragma once

#include "BasicActors.h"
#include "PhysicsEngine.h"
#include "vector"
#include <iomanip>
#include <random>

namespace PhysicsEngine
{

	class Particle : public DynamicActor
	{
		float lifeSpan = 100.0f;
	public:
		bool inScene = false;
		Particle(PxTransform pose) : DynamicActor(pose)
		{
			CreateShape(PxSphereGeometry(PxReal(0.05f)), 0.05);
		}

		void AddVelocity(PxVec3 v)
		{
			Get()->is<PxRigidDynamic>()->setLinearVelocity(v);
		}

		void Update()
		{
			lifeSpan -= 2.0f;
		}

		bool isDead()
		{
			return lifeSpan < 0.0f;
		}
	};

	class Emitter : public DynamicActor
	{
		std::vector<Particle*> particles;
		std::vector<Particle*> deadParticles;
		PxTransform pos;
		
		PxReal emitRate;
		int maxParticles;
		int numEmitted = 0;

		float timeSinceLastEmit = 0.0f;

	public:
		Emitter(PxTransform pose, PxReal rate, int maxParticles) 
			: DynamicActor(pose), pos(pose), emitRate(rate), maxParticles(maxParticles)
		{
			CreateShape(PxBoxGeometry(PxVec3(.01f, .01f, .01f)), 1.0f);
			SetKinematic(true);
		}

		void emit()
		{
			PxTransform currPos = Get()->is<PxRigidBody>()->getGlobalPose();
			Particle* particle = new Particle(currPos);
			particle->Color(PxVec3(0.6f, 0.4f, 0.2f));

			PxVec3 baseDirection(0.0f, 0.3f, 1.0f); // Slight upward and forward
			baseDirection.normalize();
			float angleSpread = PxPi / 6.0f;

			// random cone
			float theta = ((float(rand()) / RAND_MAX) - 0.5f) * 2.0f * angleSpread;
			float phi = ((float(rand()) / RAND_MAX)) * angleSpread; 
				
			PxVec3 offset(
				sinf(theta) * cosf(phi),
				sinf(phi),
				cosf(theta) * cosf(phi)
			);
			PxVec3 direction = baseDirection + offset;
			direction.normalize();

			float speed = emitRate * (0.5f + static_cast<float>(rand()) / RAND_MAX);
			particle->AddVelocity(speed * direction);
			particles.push_back(particle);
			numEmitted++;
		}

		std::vector<Particle*> getParticles()
		{
			return particles;
		}

		std::vector<Particle*> getDeadParticles()
		{
			return deadParticles;
		}

		void ClearDeadParticles()
		{
			deadParticles.clear();
		}

		void Update(float dt)
		{
			timeSinceLastEmit += dt;

			int emitCount = int(emitRate * timeSinceLastEmit);

			for (int i = 0; i < emitCount && numEmitted < maxParticles; ++i)
			{
				emit();
			}


			for (int i = 0; i < particles.size(); ++i)
			{
				Particle* particle = particles[i];
				if (!particle->isDead()) particle->Update();
				if (particle->isDead())
				{
					deadParticles.push_back(particle);
					particles.erase(particles.begin() + i);
				}
			}
		}
	};
}