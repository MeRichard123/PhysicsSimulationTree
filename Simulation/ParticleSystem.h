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
	private:
		std::vector<Particle*> m_particles;
		std::vector<Particle*> m_deadParticles;
		PxTransform m_pos;
		
		PxReal m_emitRate;
		int m_maxParticles;
		int m_numEmitted = 0;

		float m_timeSinceLastEmit = 0.0f;

	public:
		Emitter(PxTransform pose, PxReal rate, int maxParticles) 
			: DynamicActor(pose), m_pos(pose), m_emitRate(rate), m_maxParticles(maxParticles)
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

			float speed = m_emitRate * (0.5f + static_cast<float>(rand()) / RAND_MAX);
			particle->AddVelocity(speed * direction);
			m_particles.push_back(particle);
			m_numEmitted++;
		}

		std::vector<Particle*> getParticles()
		{
			return m_particles;
		}

		std::vector<Particle*> getDeadParticles()
		{
			return m_deadParticles;
		}

		void ClearDeadParticles()
		{
			m_deadParticles.clear();
		}

		void Update(float dt)
		{
			m_timeSinceLastEmit += dt;

			int emitCount = int(m_emitRate * m_timeSinceLastEmit);

			for (int i = 0; i < emitCount && m_numEmitted < m_maxParticles; ++i)
			{
				emit();
			}


			for (int i = 0; i < m_particles.size(); ++i)
			{
				Particle* particle = m_particles[i];
				if (!particle->isDead()) particle->Update();
				if (particle->isDead())
				{
					m_deadParticles.push_back(particle);
					m_particles.erase(m_particles.begin() + i);
				}
			}
		}
	};
}