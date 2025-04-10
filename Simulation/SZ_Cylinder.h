#pragma once

#include "PhysicsEngine.h"
#include <iostream>
#include <iomanip>
#include "Extras/Renderer.h"
#include <vector>


namespace PhysicsEngine {
	class SZ_Cylinder : public DynamicActor {
	public:
		SZ_Cylinder(const PxTransform& pose, PxReal radius, PxReal halfHeight, PxReal density);

		void Render();

	private:
		PxReal radius;
		PxReal halfHeight;

		PxConvexMesh* CreateConvexCylinder(PxReal radius, PxReal halfHeight, int slices = 16);
	};
}