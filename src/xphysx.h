#include <PxPhysicsAPI.h>
#include "ofMain.h"

using namespace std;
using namespace physx;

#pragma comment(lib, "PhysX3_x64.lib")
#pragma comment(lib, "Foundation.lib")
#pragma comment(lib, "PhysX3Extensions.lib")

PxPhysics* gPhysicsSDK = NULL;
PxDefaultErrorCallback gDefaultErrorCallback;
PxDefaultAllocator gDefaultAllocatorCallback;
PxSimulationFilterShader gDefaultFilterShader=PxDefaultSimulationFilterShader;

PxScene* gScene = NULL;
PxReal myTimestep = 1.0f/60.0f;
PxRigidActor *box;

void initPX() {
        PxFoundation* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
        gPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale() );
        if(gPhysicsSDK == NULL) {
                cerr<<"Error creating PhysX device."<<endl;
                cerr<<"Exiting..."<<endl;
                exit(1);
        }
	if(!PxInitExtensions(*gPhysicsSDK))
                cerr<< "PxInitExtensions failed!" <<endl;
        PxSceneDesc sceneDesc(gPhysicsSDK->getTolerancesScale());
        sceneDesc.gravity=PxVec3(0.0f, -9.8f, 0.0f);
        if(!sceneDesc.cpuDispatcher) {
	        PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
	        if(!mCpuDispatcher)
	           cerr<<"PxDefaultCpuDispatcherCreate failed!"<<endl;
	        sceneDesc.cpuDispatcher = mCpuDispatcher;
	}
        if(!sceneDesc.filterShader)
        	sceneDesc.filterShader  = gDefaultFilterShader;
        gScene = gPhysicsSDK->createScene(sceneDesc);
        if (!gScene)
	        cerr<<"createScene failed!"<<endl;
        gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE,             1.0);
        gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES,  1.0f);
        PxMaterial* mMaterial = gPhysicsSDK->createMaterial(0.5,0.5,0.5);
	PxReal d = 0.0f;
        PxTransform pose = PxTransform(PxVec3(0.0f, 0, 0.0f),PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));
        PxRigidStatic* plane = gPhysicsSDK->createRigidStatic(pose);
        if (!plane)
                cerr<<"create plane failed!"<<endl;
        PxShape* shape = plane->createShape(PxPlaneGeometry(), *mMaterial);
        if (!shape)
                cerr<<"create shape failed!"<<endl;
        gScene->addActor(*plane);
        PxReal density = 1.0f;
        PxTransform transform(PxVec3(0.0f, 10.0f, 0.0f), PxQuat::createIdentity());
        PxVec3 dimensions(0.5,0.5,0.5);
        PxBoxGeometry geometry(dimensions);
        PxRigidDynamic *actor = PxCreateDynamic(*gPhysicsSDK, transform, geometry, *mMaterial, density);
        actor->setAngularDamping(0.75);
        actor->setLinearVelocity(PxVec3(0,0,0));
        if (!actor)
                cerr<<"create actor failed!"<<endl;
        gScene->addActor(*actor);
        box = actor;
}

void stepPX() {
        gScene->simulate(myTimestep);
        while(!gScene->fetchResults() ) {
		//
        }
}

void shutdownPX() {
	gScene->removeActor(*box);
        gScene->release();
        box->release();
        gPhysicsSDK->release();
}

void getColumnMajor(PxMat33 m, PxVec3 t, float* mat) {
   mat[0] = m.column0[0];
   mat[1] = m.column0[1];
   mat[2] = m.column0[2];
   mat[3] = 0;

   mat[4] = m.column1[0];
   mat[5] = m.column1[1];
   mat[6] = m.column1[2];
   mat[7] = 0;

   mat[8] = m.column2[0];
   mat[9] = m.column2[1];
   mat[10] = m.column2[2];
   mat[11] = 0;

   mat[12] = t[0];
   mat[13] = t[1];
   mat[14] = t[2];
   mat[15] = 1;
}

void DrawBox(PxShape* pShape,PxRigidActor * actor) {
        PxTransform pT = PxShapeExt::getGlobalPose(*pShape,*actor);
        PxBoxGeometry bg;
        pShape->getBoxGeometry(bg);
        PxMat33 m = PxMat33(pT.q );
	float mat[16];
        getColumnMajor(m,pT.p, mat);
        ofPushMatrix();
                ofMultMatrix(mat);
                ofDrawBox(bg.halfExtents.x*2);
        ofPopMatrix();
}

void DrawShape(PxShape* shape,PxRigidActor *actor) {
    PxGeometryType::Enum type = shape->getGeometryType();
    switch(type) {
                case PxGeometryType::eBOX:
                        DrawBox(shape,actor);
                break;
    }
}

void DrawActor(PxRigidActor* actor) {
    PxU32 nShapes = actor->getNbShapes();
    PxShape** shapes=new PxShape*[nShapes];
    actor->getShapes(shapes, nShapes);
    while (nShapes--) {
        DrawShape(shapes[nShapes],actor);
    }
    delete [] shapes;
}

void render(){

        if (gScene) {
		stepPX();
        }
        ofDrawGrid(10.0f,8.0f,false,false,true,false);
	ofSetColor(255,0,0);
        DrawActor(box);

}
