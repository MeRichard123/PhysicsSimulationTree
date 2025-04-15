#pragma once

#include "PhysicsEngine.h"
#include <iostream>
#include <iomanip>
#include "Extras/Renderer.h"
#include <vector>


namespace PhysicsEngine {
	class RC_Cylinder : public DynamicActor {
	public:
		RC_Cylinder(const PxTransform& pose, PxReal radius, PxReal halfHeight, PxReal density);

		void Render();
		static PxConvexMesh* CreateConvexCylinder(PxReal radius, PxReal halfHeight, int slices = 16);

	private:
		PxReal m_radius;
		PxReal m_halfHeight;

	};

	inline PxConvexMeshGeometry CylinderGeometry(PxReal rad, PxReal halfHeight)
	{
		int slices = 20;
		PxConvexMesh* mesh = RC_Cylinder::CreateConvexCylinder(rad, halfHeight, slices);
		PxConvexMeshGeometry geometry(mesh);
		return geometry;
	}

}