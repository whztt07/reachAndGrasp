#include "grasp.h"
#include "intersection.h"

namespace Connection
{
	Grasp::Grasp()
	{}

	Grasp::~Grasp()
	{
		initRot.clear();
		holdRot.clear();
		spheres.clear();
		bs.clear();
	}

	bool Grasp::initGrasp(Ogre::String fname, Ogre::SceneManager* mSceneMgr, Ogre::Entity* ent)
	{
		this->mSceneMgr = mSceneMgr;
		this->ent = ent;
		this->skeleton = ent->getSkeleton();

		readGraspFile(fname);

		//finger_chains
		std::vector<Ogre::String> fingerJointName;
		fingerJointName.push_back("rightHandThumb4");
		fingerJointName.push_back("rightHandThumb3");
		fingerJointName.push_back("rightHandThumb2");
		fingerJointName.push_back("rightHandThumb1");
		//fingerJointName.push_back("rightHand");

		fingerJointName.push_back("rightHandIndex4");
		fingerJointName.push_back("rightHandIndex3");
		fingerJointName.push_back("rightHandIndex2");
		fingerJointName.push_back("rightHandIndex1");
		//fingerJointName.push_back("rightHand");

		fingerJointName.push_back("rightHandMiddle4");
		fingerJointName.push_back("rightHandMiddle3");
		fingerJointName.push_back("rightHandMiddle2");
		fingerJointName.push_back("rightHandMiddle1");
		//fingerJointName.push_back("rightHand");

		fingerJointName.push_back("rightHandRing4");
		fingerJointName.push_back("rightHandRing3");
		fingerJointName.push_back("rightHandRing2");
		fingerJointName.push_back("rightHandRing1");
		//fingerJointName.push_back("rightHand");

		fingerJointName.push_back("rightHandPinky4");
		fingerJointName.push_back("rightHandPinky3");
		fingerJointName.push_back("rightHandPinky2");
		fingerJointName.push_back("rightHandPinky1");
		//fingerJointName.push_back("rightHand");

		int count = 0;
		FingerChain temp_chain;
		for (int i = 0; i < fingerJointName.size(); i++)
		{
			FingerJoint temp;
			temp.Name = fingerJointName.at(i);
			temp.lock = false;
			temp.id = skeleton->getBone(fingerJointName[i])->getHandle();
			//temp.world_pos = Ogre::Vector3::ZERO;
			temp_chain.fingerJoints.push_back(temp);
			count++;

			if (count == 4){
				finger_chains.push_back(temp_chain);
				temp_chain.fingerJoints.clear();
				count = 0;
			}
		}

		store_bone_rot(initRot);

		grasp_time_start = 1.8;
		grasp_time_fully_hold_0 = 2.5;
		grasp_time_fully_hold_1 = 3.0;
		grasp_time_fully_release = 4.0;
		return true;
	}

	void Grasp::readGraspFile(Ogre::String fname)
	{
		std::map<Ogre::String, int> mapSknameIdx;
		cv::FileStorage f(fname, cv::FileStorage::READ);
		graspParams.clear();
		if (f.isOpened())
		{
			cv::FileNode params = f["parameters"];
			int pnum = params["num"];
			int pidx = 0;
			for (int i = 0; i<pnum; i++){
				char str[256];
				sprintf(str, "parameter%d", i);
				cv::FileNode param = params[str];
				if (param.empty())
					continue;
				Ogre::String name = param["name"];
				int idx = param["index"];
				/*float fv = param["forward_vel"];
				float tv = param["turn_vel"];
				float sv = param["side_vel"];*/
				GraspParam lp;// = this->initLocomotionParam(fv, tv, sv);

				lp.times.push_back(param["T0"]);
				lp.times.push_back(param["T1"]);
				lp.times.push_back(param["T2"]);
				lp.times.push_back(param["T3"]);
				graspParams.push_back(lp);
				mapSknameIdx[name] = pidx;
				pidx++;
			}
			f.release();
		}
		getAnimationNames(mapSknameIdx);

		//	cv::FileStorage fs("..\\..\\Samples\\DualQuaternion\\Connection\\motion_params_vice.xml", cv::FileStorage::WRITE); 	

	}

	void Grasp::getAnimationNames(std::map<Ogre::String, int> &mapSkIdx)
	{
		Ogre::Skeleton::LinkedSkeletonAnimSourceIterator linkIt = this->ent->getSkeleton()->getLinkedSkeletonAnimationSourceIterator();
		int i = 0;
		while (linkIt.hasMoreElements())
		{
			const Ogre::LinkedSkeletonAnimationSource& link = linkIt.getNext();
			std::map<Ogre::String, int>::iterator it = mapSkIdx.find(link.skeletonName);
			if (it != mapSkIdx.end()){
				int idx = mapSkIdx[link.skeletonName];
				graspParams[idx].aniname = link.pSkeleton->getAnimation(0)->getName();
			}
			//		animatename = aniNames[i];
			i++;
		}
	}

	void Grasp::reset()
	{
		//Ogre::Skeleton* skeleton = ent->getSkeleton();
		//skeleton->reset(true);

		//only reset rots of hand joints
		for (int i = 0; i < skeleton->getNumBones(); i++)
		{
			if (hand_mask_check(i)){
				Ogre::Bone* bone = skeleton->getBone(i);
				bone->setOrientation(bone->getInitialOrientation());
				//bone->setOrientation(initRot[i]);
			}
		}

		for (int i = 0; i < finger_chains.size(); i++)
		{
			for (int j = 0; j < finger_chains[i].fingerJoints.size(); j++)
			{
				finger_chains[i].fingerJoints[j].lock = false;
			}
		}
	}

	int Grasp::controller(float time)
	{
		if (time < grasp_time_fully_hold_0)
		{
			return 0;
		}
		else if (time > grasp_time_fully_hold_0 && time < grasp_time_fully_hold_1)
		{
			return 1;
		}
		else if (time > grasp_time_fully_hold_1 && time < grasp_time_fully_release)
		{
			return 2;
		}
	}

	void Grasp::update_parameters(const int grasp_type)
	{
		if (grasp_type >= graspParams.size())
			return;

		GraspParam temp = graspParams[grasp_type];
		grasp_time_start = temp.times[0];
		grasp_time_fully_hold_0 = temp.times[1];
		grasp_time_fully_hold_1 = temp.times[2];
		grasp_time_fully_release = temp.times[3];
		ani_name = temp.aniname;
	}

	void Grasp::update(float time, const int grasp_type)
	{
		update_parameters(grasp_type);

		time = time + grasp_time_start;

		int state = controller(time);
		//state = 0;
		if (state == 0) //hold
		{
			collision_detection();
			play_animation(time, ani_name /*"shou_s"*/);
		}
		else if (state == 1) //fully hold
		{
			//TODO: record rot vector
			store_bone_rot(holdRot);
		}
		else if (state == 2) //release
		{
			//TODO: interpolate fully hold and fully release
			std::vector<Ogre::Quaternion> out;
			float weight = (time - grasp_time_fully_hold_1) / (grasp_time_fully_release - grasp_time_fully_hold_1);
			interp_bone_rot(holdRot, initRot, out, weight);
			apply_rot_to_skeleton(out);
		}
	}

	//only apply rot to hand
	void Grasp::apply_rot_to_skeleton(std::vector<Ogre::Quaternion> &Rot)
	{
		for (int i = 0; i < skeleton->getNumBones(); i++)
		{
			if (hand_mask_check(i)){
				Ogre::Bone* bone = skeleton->getBone(i);
				//bone->setOrientation(bone->getInitialOrientation());
				bone->setOrientation(Rot[i]);
			}
		}
	}

	void Grasp::store_bone_rot(std::vector<Ogre::Quaternion> &Rot)
	{
		Rot.clear();
		for (int i = 0; i < skeleton->getNumBones(); i++)
		{
				Ogre::Bone* bone = skeleton->getBone(i);
				Ogre::Quaternion q = bone->getOrientation();
				Rot.push_back(q);
		}
	}

	//lerp: 0.0 nlerp, 1,0 slerp
	bool Grasp::interp_bone_rot(std::vector<Ogre::Quaternion> &Rot0, std::vector<Ogre::Quaternion> &Rot1, std::vector<Ogre::Quaternion> &out, float weight, const int lerp)
	{ 
		if (Rot0.size() != Rot1.size())
			return false;
		out.clear();
		for (int i = 0; i < Rot0.size(); i++)
		{
			if (lerp == 0.0){
				out.push_back(Ogre::Quaternion::nlerp(weight, Rot0[i], Rot1[i]));
			}
			else if (lerp == 1.0){
				out.push_back(Ogre::Quaternion::Slerp(weight, Rot0[i], Rot1[i]));
			}
		}
	}

	//only update joints of hand and no collision
	void Grasp::play_animation(float time, Ogre::String AniName)
	{
		//Ogre::Skeleton* skeleton = ent->getSkeleton();
		//skeleton->reset(true);
		Ogre::String rootname = "hips1";
		Ogre::String animaiton_name = AniName;// "shou_s";// "shou.skeleton";// this->reachParams[ani_numb].aniname;
		Ogre::Bone* root = skeleton->getBone(rootname);
		Ogre::Animation* anim = skeleton->getAnimation(animaiton_name);
		Ogre::TimeIndex timeIndex = anim->_getTimeIndex(time);   //timeIndex = time
		Ogre::Animation::NodeTrackList trackList = anim->_getNodeTrackList();
		Ogre::Animation::NodeTrackList::iterator it;
		//root->setPosition(root->getInitialPosition());
		for (it = trackList.begin(); it != trackList.end(); it++)
		{
			Ogre::Bone* bone = skeleton->getBone(it->first);
			if (!bone_lock_check(bone))
			{
				if (hand_mask_check(bone->getHandle())){
					bone->setOrientation(bone->getInitialOrientation());
					it->second->applyToNode(bone, timeIndex);
				}
			}
		}
	}

	//return true if lock
	bool Grasp::bone_lock_check(Ogre::Bone* bone)
	{
		for (int i = 0; i < finger_chains.size(); i++)
		{
			for (int j = 0; j < finger_chains[i].fingerJoints.size(); j++)
			{
				if (finger_chains[i].fingerJoints[j].Name == bone->getName()){
					//return finger_chains[i].bones_lock[j];
					return finger_chains[i].fingerJoints[j].lock;
				}
			}
		}

		return true; //if bone not in FingerChains, it would not update
	}

	//void Grasp::pure_play(float time, int ani_numb)
	//{
	//	Ogre::Skeleton* skeleton = ent->getSkeleton();
	//	//skeleton->reset(true);
	//	Ogre::String rootname = "hips1";
	//	Ogre::Bone* root = skeleton->getBone(rootname);
	//	Ogre::String animaiton_name = "shou_s";// "shou.skeleton";// this->reachParams[ani_numb].aniname;
	//	Ogre::Animation* anim = skeleton->getAnimation(animaiton_name);
	//	Ogre::TimeIndex timeIndex = anim->_getTimeIndex(time);   //timeIndex = time
	//	Ogre::Animation::NodeTrackList trackList = anim->_getNodeTrackList();
	//	Ogre::Animation::NodeTrackList::iterator it;
	//	root->setPosition(root->getInitialPosition());
	//	for (it = trackList.begin(); it != trackList.end(); it++)
	//	{
	//		Ogre::Bone* bone = skeleton->getBone(it->first);
	//		
	//		if (!bone_lock_check(bone))
	//		{
	//			bone->setOrientation(bone->getInitialOrientation());
	//			it->second->applyToNode(bone, timeIndex);
	//		}
	//	}
	//	//collision detection
	//	collision_detection();
	//}

	void Grasp::add_collision_shapes()
	{
		spheres.clear();
		bs.clear();
		float offset = 0.5; //finger radius
		//Ogre::Skeleton* skeleton = ent->getSkeleton();
		Ogre::SceneNode* sphereNode = mSceneMgr->getSceneNode("sphere_node");
		Ogre::Vector3 pos = sphereNode->_getDerivedPosition();
		Ogre::Entity* sphere = mSceneMgr->getEntity("Sphere");
		float radius = 100 * sphereNode->getScale().x;// sphere->getBoundingRadius() * sphereNode->getScale().x;
		Ogre::Sphere collisionSphere(pos, radius + offset);

		Ogre::SceneNode* cubeNode = mSceneMgr->getSceneNode("cube_node");
		Ogre::Vector3 pos_cube = cubeNode->_getDerivedPosition();
		//Ogre::Entity* cube = mSceneMgr->getEntity("Cube");
		float half_edge = 50 * cubeNode->getScale().x;// sphere->getBoundingRadius() * sphereNode->getScale().x;
		Ogre::Vector3 temp = (half_edge + offset) * Ogre::Vector3(1, 1, 1);
		Ogre::AxisAlignedBox AABB(pos_cube - temp, pos_cube + temp);

		spheres.push_back(collisionSphere);
		bs.push_back(AABB);
		//initialize collision shapes END
	}

	void Grasp::collision_detection()
	{
		add_collision_shapes();

		//Ogre::Skeleton* skeleton = ent->getSkeleton();
		for (int i = 0; i < finger_chains.size(); i++)
		{
			for (int j = 0; j < finger_chains[i].fingerJoints.size(); j++)
			{
				//1. already collision
				if (finger_chains[i].fingerJoints[j].lock == true)
				{
					////lock parent joints of this joint inside this chain
					//for (int m = j+1; m < finger_chains[i].fingerJoints.size(); m++)
					//{
					//	finger_chains[i].fingerJoints[m].lock = true;
					//}
					//next finger_chain
					break;
				}

				////2. check collision
				Ogre::String bone_name = finger_chains[i].fingerJoints[j].Name;
				Ogre::Bone* bone = skeleton->getBone(bone_name);
				Ogre::Vector3 lineStartPos = bone->_getDerivedPosition(); 
				Ogre::Vector3 lineEndPos = bone->getParent()->_getDerivedPosition(); 
//				if (IntersectionLineSphere(lineStartPos, lineEndPos, collisionSphere))
				//if (IntersectionLineAABB(lineStartPos, lineEndPos, AABB))
				if (IntersectionLine(lineStartPos, lineEndPos, spheres, bs))
				{
					finger_chains[i].fingerJoints[j].lock = true;
					//lock parent joints of this joint inside this chain
					for (int m = j + 1; m < finger_chains[i].fingerJoints.size(); m++)
					{
						finger_chains[i].fingerJoints[m].lock = true;
					}
					//next finger_chain
					break;
				}
				// if (bone_name == "rightHandMiddle4")
				//{
				//	finger_chains[i].fingerJoints[j].lock = true;
				//	//lock parent joints of this joint inside this chain
				//	for (int m = j + 1; m < finger_chains[i].fingerJoints.size(); m++)
				//	{
				//		finger_chains[i].fingerJoints[m].lock = true;
				//	}
				//	//next finger_chain
				//	break;
				//}
			}
		}
	}

	//check if bone_id is inside fingerChains
	bool Grasp::hand_mask_check(int bone_id)
	{
		for (int i = 0; i < finger_chains.size(); i++)
		{
			for (int j = 0; j < finger_chains[i].fingerJoints.size(); j++)
			{
				if (finger_chains[i].fingerJoints[j].id == bone_id)
				{
					return true;;
				}
			}
		}

		return false;
	}
}