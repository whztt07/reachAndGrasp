#pragma once

#include "Ogre.h"
#include "opencv2/core/core.hpp"

namespace Connection
{
	struct FingerJoint
	{
		Ogre::String Name;
		int id;
		bool lock;
		//Ogre::Vector3 world_pos;
	};

	struct FingerChain
	{
	/*	std::vector <int> bones_id;
		std::vector <bool> bones_lock;*/
		std::vector<FingerJoint> fingerJoints;
	};

	class GraspParam
	{
	public:
		/*float velf;
		float velt;
		float vels;*/
		std::vector<float> times;            //to-t1, t1-t2
		Ogre::String aniname;
	};

	class Grasp
	{
	public:
		Grasp();
		~Grasp();
		bool initGrasp(Ogre::String fname, Ogre::SceneManager* mSceneMgr, Ogre::Entity* ent);
		void update(float time, const int grasp_type = 0);
		//void pure_play(float time, int ani_numb);
		bool bone_lock_check(Ogre::Bone* bone);
		void reset();
		void play_animation(float time, Ogre::String AniName);
		void store_bone_rot(std::vector<Ogre::Quaternion> &Rot);
		bool interp_bone_rot(std::vector<Ogre::Quaternion> &Rot0, std::vector<Ogre::Quaternion> &Rot1, std::vector<Ogre::Quaternion> &out, float weight, const int lerp = 0.0);
		void apply_rot_to_skeleton(std::vector<Ogre::Quaternion> &Rot);
		void add_collision_shapes();
		void collision_detection();
		bool hand_mask_check(int bone_id);
		int controller(float time);
		void readGraspFile(Ogre::String fname);
		void getAnimationNames(std::map<Ogre::String, int> &mapSkIdx);
		void update_parameters(const int grasp_type);

	private:
		/*float current_time;
		float previouse_time;*/
		std::vector <FingerChain> finger_chains;
		Ogre::SceneManager* mSceneMgr;
		Ogre::Entity* ent;
		Ogre::Skeleton* skeleton;
		std::vector<Ogre::Quaternion> initRot;
		std::vector<Ogre::Quaternion> holdRot;

		std::vector<Ogre::Sphere> spheres;
		std::vector<Ogre::AxisAlignedBox> bs;
		
		float grasp_time_start;
		float grasp_time_fully_hold_0;
		float grasp_time_fully_hold_1;
		float grasp_time_fully_release;

		std::vector<Connection::GraspParam> graspParams;
		Ogre::String ani_name;
	};
}