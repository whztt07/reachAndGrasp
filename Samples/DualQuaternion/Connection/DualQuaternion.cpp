
#include "SamplePlugin.h"
#include "DualQuaternion.h"
#include <time.h>
//#include "..\Connection\motion_params.xml"
//locomotion.initLocomotion("..\\..\\Samples\\DualQuaternion\\Connection\\motion_params.xml", srcEntity);

#ifndef OGRE_STATIC_LIB

SamplePlugin* sp;
Sample* s;

extern "C" _OgreSampleExport void dllStartPlugin()
{
	s = new Ogre::Sample_DualQuaternion;
	sp = OGRE_NEW SamplePlugin(s->getInfo()["Title"] + " Sample");
	sp->addSample(s);
	Ogre::Root::getSingleton().installPlugin(sp);
}

extern "C" _OgreSampleExport void dllStopPlugin()
{
	Ogre::Root::getSingleton().uninstallPlugin(sp); 
	OGRE_DELETE sp;
	delete s;
}

namespace Ogre
{
	bool Sample_DualQuaternion::frameRenderingQueued(const FrameEvent& evt)
	{
		double dt = evt.timeSinceLastFrame; 
		update(dt);
		return SdkSample::frameRenderingQueued(evt);
	}

	void Sample_DualQuaternion::loadMeshData(Ogre::String name)
	{
		mMesh = MeshManager::getSingleton().load(name+".mesh",
			ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME).staticCast<Mesh>();

		// get the skeleton, animation, and the node track iterator
		mSkeleton = SkeletonManager::getSingleton().load(name+".skeleton",
			ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME).staticCast<Skeleton>();
	}

	void Sample_DualQuaternion::initSkeletonTransfer()
	{
		double gap = 1;
		Ogre::String meshName = "WT_wuhuanzhige";

		ROOTNAME = "hips1";

		getSkeletonNames();

		srcAs = NULL;	
		
		if (true)
		{
			loadMeshData(meshName);
			srcEntity = mSceneMgr->createEntity("srcEntity", mMesh->clone("srcMesh"));
			for(int i=0; i<skeNames.size(); i++) {
				srcEntity->getSkeleton()->addLinkedSkeletonAnimationSource(skeNames[i]);
			}
			srcEntity->refreshAvailableAnimationState();
			getAnimationNames(srcEntity);

			sn = mSceneMgr->getRootSceneNode()->createChildSceneNode("srcEntity");
			sn->attachObject(srcEntity);
			//sn->setPosition(-5,0,0);
			sn->setVisible(1);

			int aidx = 10;  //number of animations?
			srcAs = srcEntity->getAnimationState(aniNames[aidx]);  //?
			srcAs->setEnabled(0);
			srcAs->setWeight(0);
			srcAs->setLoop(false);
			ANIM_CHOP = srcAs->getLength();

			//locomotion.initLocomotion("..\\..\\Samples\\DualQuaternion\\Connection\\motion_params.xml", srcEntity);
			locomotion.initLocomotion("E:/ogre_old/Samples/Media/models/motion_params.xml", srcEntity);
			//Ling, ini reach
			reach.initReach("E:/ogre_old/Samples/Media/models/reach_params.xml", srcEntity, mSceneMgr);
			//reach.initReach("C:/Users/ling/Desktop/sinbad-ogre67/sinbad-ogre/Samples/DualQuaternion/Connection/reach_params.xml", srcEntity, mSceneMgr);
			visualize_pseudo_sample_points();
			
			grasp.initGrasp("E:/ogre_old/Samples/Media/models/grasp_params.xml", mSceneMgr, srcEntity);

			step = 0;
			runtime = 0;// 1.8;// 0;
		//	srcAs = 0;
		}
	}

	//visualize pseudo sample points (world -space)
	void Sample_DualQuaternion::visualize_pseudo_sample_points()
	{
		if (reach.pseudo_sample_points.size() != 0)
		{
			Ogre::Vector3 root_world_pos = reach.root_world_pos_;
			for (int i = 0; i < reach.pseudo_sample_points.size(); i++)
			{
				Ogre::Vector3 pseudo_sample_points_world_pos = Ogre::Vector3(&reach.pseudo_sample_points[i].Pos.x) + root_world_pos;

				Entity* sphere = mSceneMgr->createEntity("Sphere" + std::to_string(i), "sphere.mesh");
				sphere->setMaterialName("Examples/White");
				if (i<25) //original sample point
					sphere->setMaterialName("Examples/Green");
				sphere->setCastShadows(false); //true
				SceneNode* sphereNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("sphere_node" + std::to_string(i));
				//sphereNode->translate(0, 10, 0);
				sphereNode->translate(pseudo_sample_points_world_pos);
				sphereNode->scale(0.01, 0.01, 0.01);
				sphereNode->attachObject(sphere);
			}
		}
	}
	void Sample_DualQuaternion::offline_play_reach_get_parameter(double dt)
	{
		double speed = 1;

		if (srcAs)
		{
			Ogre::Skeleton* sSkeleton = srcEntity->getSkeleton();
			for (int i = 0; i < sSkeleton->getNumBones(); i++)
				sSkeleton->getBone(i)->setManuallyControlled(true);

			runtime += 0.01;// speed*dt;
			//double upperTime = 20.9;
			//float tp = 0.333;

			//srcAs->addTime(dt*speed);
			//float time = dt*speed ; // srcAs->getTimePosition();
			reach.pure_play(runtime, 25);

			////reset
			//if (runtime > upperTime){
			//	reach.resetReach();
			//	sSkeleton->reset(true);
			//	step = 0;
			//	runtime = 0;
			//}

			////add timestep
			//reach.addRunTime(speed*dt);

			//if (step == 0){
			//	step = 0;
			//	Ogre::Vector3 pos(0, 0, 0);
			//	Ogre::Vector3 dir(1, 0, 1);
			//	dir.normalise();
			//	reach.resetReach();
			//	reach.setRootState(pos, dir);
			//	/*std::vector<float> vps;
			//	pathPlanning(vps);
			//	reach.predictNewExamples(vps);*/
			//	std::vector<float> x;
			//	x = { 1., 0.0, 0.0 };///temporary
			//	reach.predictNewExamples(x);
			//	step++; //ling
			//}
			//else if (reach.getRuntime() > reach.getCliptime()){
			//	runtime -= (reach.getRuntime() - reach.getCliptime());
			//	Ogre::Vector3 rootpos = Root->_getDerivedPosition();
			//	reach.saveLastMotionState();
			//	reach.obtainNextRootState();
			//	//std::vector<float> vps;
			//	//pathPlanning(vps);
			//	std::vector<float> x;
			//	x = { 1., 0.0, 0.0 };///temporary
			//	reach.predictNewExamples(x);
			//	step++;  //ling
			//}

			//reach.motionSynthesis();

			//reach.transitionClips(tp);
		}
	}

	//int Sample_DualQuaternion::controller(float time)
	//{
	//	if (time < 2.5)
	//	{
	//		return 0;
	//	}
	//	else if (time > 2.5 && time < 3.0)
	//	{
	//		return 1;
	//	}
	//	else if (time >3.0 && time < 4.0)
	//	{
	//		return 2;
	//	}
	//}

	void Sample_DualQuaternion::offline_play_grasp(double dt)
	{
		double speed = 1;

		if (srcAs)
		{

			/*SceneNode* sphereNode = mSceneMgr->getSceneNode("sphere_node");
			sphereNode->_setDerivedPosition(target_pos);*/

			SceneNode* cubeNode = mSceneMgr->getSceneNode("cube_node");
			cubeNode->_setDerivedPosition(target_pos);

			Ogre::Skeleton* sSkeleton = srcEntity->getSkeleton();
			for (int i = 0; i < sSkeleton->getNumBones(); i++)
				sSkeleton->getBone(i)->setManuallyControlled(true);

			if (runtime > 4.0){
				grasp.reset();
				runtime = 0;// 1.8;
			}
			runtime += 0.005; // 0.01´©Í¸ÑÏÖØ
//			grasp.pure_play(runtime, 25);

			//int state = controller(runtime);
			grasp.update(runtime);
		}
	}

	void Sample_DualQuaternion::offline_play_reach(double dt)
	{
		double speed = 1;

		//update pos of sphere
		SceneNode* sphereNode = mSceneMgr->getSceneNode("sphere_node");
		sphereNode->_setDerivedPosition(target_pos);

		if (srcAs)
		{
			srcAs->addTime(dt*speed);

			Ogre::Skeleton* Skeleton = srcEntity->getSkeleton();
			Ogre::Bone* Root = Skeleton->getBone(ROOTNAME);

			for (int i = 0; i < Skeleton->getNumBones(); i++)
				Skeleton->getBone(i)->setManuallyControlled(true);

			runtime += speed*dt;
			
			//add timestep
			reach.addRunTime(speed*dt);

			if (reach.getRuntime() > reach.getCliptime()){ //one clip ends
				reach.updateTimeStep();
			}

			std::vector<float> x = { target_pos.x, target_pos.y, target_pos.z };
			reach.predictNewExamples(x);
			reach.motionSynthesis();
			//reach.update(speed*dt, target_pos);
		}
	}

	void Sample_DualQuaternion::offline_play(double dt)
	{
		double speed = 1;
		
		if(srcAs)
		{
			Ogre::Skeleton* sSkeleton = srcEntity->getSkeleton();
			Ogre::Bone* Root = sSkeleton->getBone(ROOTNAME);
			srcAs->addTime(dt*speed);
			
			for(int i=0; i<sSkeleton->getNumBones(); i++)
				sSkeleton->getBone(i)->setManuallyControlled(true);

			runtime += speed*dt;
			double upperTime = 20.9;
			float tp = 0.333;

			if(runtime > upperTime){
				locomotion.resetLocomotion();
				sSkeleton->reset(true);
				step = 0;
				runtime = 0;
			}
			locomotion.addRunTime(speed*dt);
			if(step==0){
				step = 0;
				Ogre::Vector3 pos(-628, 0, 0);
				Ogre::Vector3 dir(1, 0, 1);
				dir.normalise();
				locomotion.resetLocomotion();
				locomotion.setRootState(pos, dir);
				std::vector<float> vps;
				pathPlanning(vps);
				locomotion.predictNewExamples(vps);

			}
			else if(locomotion.getRuntime() > locomotion.getCliptime()){
				runtime -= (locomotion.getRuntime() - locomotion.getCliptime());
				Ogre::Vector3 rootpos = Root->_getDerivedPosition();
				locomotion.saveLastMotionState();
				locomotion.obtainNextRootState();
				std::vector<float> vps;
				pathPlanning(vps);
				locomotion.predictNewExamples(vps);
			}

			locomotion.motionSynthesis();

			locomotion.transitionClips(tp);		
		}
	}

	void Sample_DualQuaternion::pathPlanning(std::vector<float>& vparams, double upper)
	{
		//sin shape routine
		float initBias = -628;
		float add = 1;
		float x = 60*step;
		float x_ = 60*(step + 1);
	//	x = 60 * runtime;
	//	x_ = 60 * (runtime + add);

		float xs = x/200.0;
		float xs_ = x_/200.0;
		float z = 300 * Ogre::Math::Sin(xs);
		float z_ = 300 * Ogre::Math::Sin(xs_);
		float dz = 1.5 * Ogre::Math::Cos(xs);
		float dz_ = 1.5 * Ogre::Math::Cos(xs_);
		Ogre::Vector3 pos_1(x+initBias, 0, z);
		Ogre::Vector3 pos_2(x_+initBias, 0, z_);
		Ogre::Vector3 dir_1(1, 0, dz);
		Ogre::Vector3 dir_2(1, 0, dz_);

		dir_1.normalise();
		dir_2.normalise();
		Ogre::Vector3 deltaPos = pos_2 - pos_1;
		float vf = deltaPos.dotProduct(dir_1);
		float vs = (deltaPos - vf * dir_1).length();
		Ogre::Vector3 rot = dir_1.crossProduct(deltaPos);
		if(rot.y < 0)
			vs = -vs;
		Ogre::Radian turn = dir_1.angleBetween(dir_2);
		float vt = turn.valueDegrees();
		rot = dir_1.crossProduct(dir_2);
		if(rot.y < 0)
			vt = -vt;

		vparams.clear();
		vparams.push_back(vf/add);
		vparams.push_back(vs/add);
		vparams.push_back(vt/add);
		step ++;
	}
		
	void Sample_DualQuaternion::getSkeletonNames()
	{
		std::vector<Ogre::String> allSkeNames;

		allSkeNames.push_back("loco_walk_slow.skeleton");
		allSkeNames.push_back("loco_walk_fast.skeleton");	
		allSkeNames.push_back("loco_run.skeleton");
		allSkeNames.push_back("loco_left_bigcircle_slow.skeleton");
		allSkeNames.push_back("loco_right_bigcircle_slow.skeleton");
		allSkeNames.push_back("loco_left_bigcircle_fast.skeleton");
		allSkeNames.push_back("loco_right_bigcircle_fast.skeleton");
		allSkeNames.push_back("loco_left_smallcircle_slow.skeleton");
		allSkeNames.push_back("loco_right_smallcircle_slow.skeleton");
		allSkeNames.push_back("loco_left_smallcircle_fast.skeleton");
		allSkeNames.push_back("loco_right_smallcircle_fast.skeleton");
		allSkeNames.push_back("loco_left_walk_slow.skeleton");
		allSkeNames.push_back("loco_right_walk_slow.skeleton");
		allSkeNames.push_back("loco_left_walk_fast.skeleton");
		allSkeNames.push_back("loco_right_walk_fast.skeleton");

		//TODO: add reach animation name here, Ling
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachRtHigh.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachRtMidHigh.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachRtMidLow.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachRtLow.skeleton");

		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachMiddleHigh.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachMiddleMidHigh.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachMiddleMidLow.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachMiddleLow.skeleton");

		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachClose_Lf.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachClose_Rt.skeleton");

		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachClose_MiddleHigh.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachClose_MiddleMidHigh.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachClose_MiddleMidLow.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachClose_MiddleLow.skeleton");  //0~17

		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachBehind_High1.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachBehind_High2.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachBehind_Low1.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachBehind_Low2.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachBehind_MidLow1.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachBehind_MidLow2.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachBehind_MidHigh1.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachBehind_MidHigh2.skeleton");

		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachLfHigh.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachLfMidHigh.skeleton");
		allSkeNames.push_back("ChrHarmony_Relax001_ArmReachLfLow.skeleton");
		//allSkeNames.push_back("ChrHarmony_Relax001_ArmReachLfMidHigh.skeleton");


		allSkeNames.push_back("shou.skeleton");

		for(int i=0; i<allSkeNames.size(); i++)
			skeNames.push_back(allSkeNames[i]);
	}

	void Sample_DualQuaternion::getAnimationNames(Ogre::Entity *ent)
	{
		Ogre::Skeleton::LinkedSkeletonAnimSourceIterator linkIt = ent->getSkeleton()->getLinkedSkeletonAnimationSourceIterator();
		int i=0;
		while(linkIt.hasMoreElements())
		{
			const Ogre::LinkedSkeletonAnimationSource& link = linkIt.getNext();
			aniNames.push_back(link.pSkeleton->getAnimation(0)->getName());
	//		animatename = aniNames[i];
			i++;
		}
	}

	void Sample_DualQuaternion::update(float dt)
	{
		//offline_play(dt);		//locomotion
		offline_play_reach(dt); //reach
		//offline_play_reach_get_parameter(dt); //get reach parameter
		//offline_play_grasp(dt);
	}
}


#endif