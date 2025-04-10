#pragma once
#include "SZ_Cylinder.h"

namespace PhysicsEngine {

	std::vector<PxVec3> GenerateCylinderVertices(float radius, float halfHeight, int slices)
	{
		std::vector<PxVec3> verts;

		for (int i = 0; i < slices; ++i) {
			float angle = (PxPi * 2.0f * i) / slices;
			float x = radius * cos(angle);
			float z = radius * sin(angle);

			verts.emplace_back(x, -halfHeight, z); // bottom
			verts.emplace_back(x, halfHeight, z); // top
		}
		return verts;
	}


	PxConvexMesh* SZ_Cylinder::CreateConvexCylinder(PxReal radius, PxReal halfHeight, int slices)
	{
		auto verts = GenerateCylinderVertices(radius, halfHeight, slices);

		PxConvexMeshDesc desc;
		desc.points.count = static_cast<PxU32>(verts.size());
		desc.points.stride = sizeof(PxVec3);
		desc.points.data = verts.data();
		desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

		PxDefaultMemoryOutputStream buf;
		bool ok = GetCooking()->cookConvexMesh(desc, buf);
		if (!ok) return nullptr;

		PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
		return GetPhysics()->createConvexMesh(input);
	}

	SZ_Cylinder::SZ_Cylinder(const PxTransform& pose, PxReal radius, PxReal halfHeight, PxReal density)
		: DynamicActor(pose), radius(radius), halfHeight(halfHeight)
	{
		PxConvexMesh* mesh = CreateConvexCylinder(radius, halfHeight);
		if (mesh) {
			PxConvexMeshGeometry geom(mesh);
			CreateShape(geom, density);
		}
	}

	void SZ_Cylinder::Render()
	{
		PxTransform pose = ((PxRigidBody*)Get())->getGlobalPose();
		PxMat44 shapePose(pose);

		glEnable(GL_COLOR_MATERIAL);    
		glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
		glColor3f(0.6f, 0.34509803921568627f, 0.16470588235294117f);

		glPushMatrix();
		glMultMatrixf((float*)&shapePose);
		glTranslatef(0.0f, halfHeight, 0.f);
		glRotatef(90.0f, 1.0f, 0.0f, 0.0f);



		// Cylinder body
		GLUquadric* qobj = gluNewQuadric();
		gluQuadricNormals(qobj, GLU_SMOOTH);
		gluCylinder(qobj, radius, radius, halfHeight * 2.f, 64, 32);
		gluDeleteQuadric(qobj);

		glPopMatrix();
	}
}