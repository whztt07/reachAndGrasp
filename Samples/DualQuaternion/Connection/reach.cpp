#include "reach.h"
#include "OgreSkeleton.h"
//#include "Ogre.h"

//#include "locomotion.h"
#include <iostream>

namespace Connection
{

void Reach::initReach(Ogre::String fname, Ogre::Entity* entity, Ogre::SceneManager* mSceneMgr)
{
	//first read locomotion xml file, initialize locomotion parameters
	this->ent = entity;
	skeleton = ent->getSkeleton();
	rootname = "hips1";
	root = skeleton->getBone(rootname);
	root_world_pos_ = root->_getDerivedPosition();
	readReachFile(fname);

	runtime = 0;
	cliptime = 0;
	lasttime = 0;
	timeIdxMax = 2;
	timeIdx = 0;
	p_r_t_i = Ogre::Vector3(0.0);
	d_p_i = Ogre::Vector3(0.0);
	time_i = 0;

	effector_target.effectorName = "rightHand";
	effector_target.targetOrientation = Ogre::Quaternion::ZERO;
	effector_target.targetPosition = Ogre::Vector3::ZERO;
	effector_target.oriRestraint = false;
	effector_target.posRestraint = true;

	IK_params.dEntity = entity;
	IK_params.iterNum = 10;
	IK_params.relative = true;
	IK_params.rootName = rootname;
	IK_params.rootProcess = false;
	IK_params.scale = 1.0;
	IK_params.Targets.push_back(effector_target);
}

void Reach::resetReach()
{
	runtime = 0;
	cliptime = 0;
	lasttime = 0;
	timeIdxMax = 2;
	timeIdx = 0;
	diffs.clear();
	inputParam.clear();
	p_r_t_i = Ogre::Vector3(0.0);
	d_p_i = Ogre::Vector3(0.0);
	time_i = 0;
}

ReachParam Reach::initReachParam(float x, float y, float z)
{
	ReachParam lp;
	lp.x = x;
	lp.y = y;
	lp.z = z;
	return lp;
}

void Reach::readReachFile(Ogre::String fname)
{
	std::map<Ogre::String, int> mapSknameIdx;
	cv::FileStorage f(fname, cv::FileStorage::READ);
	reachParams.clear();
	if(f.isOpened())
	{
		cv::FileNode params = f["parameters"];
		int pnum = params["num"];
		int pidx = 0;
		for(int i=0; i<pnum; i++){
			char str[256];
			sprintf(str, "parameter%d", i);
			cv::FileNode param = params[str];
			if(param.empty())
				continue;
			Ogre::String name = param["name"];
			int idx = param["index"];
			float x = param["x"];
			float y = param["y"];
			float z = param["z"];
			ReachParam lp = this->initReachParam(x, y, z);
			
			lp.times.push_back(param["T0"]);
			lp.times.push_back(param["T1"]);
			lp.times.push_back(param["T2"]);
			lp.times.push_back(param["T3"]);

			reachParams.push_back(lp);
			mapSknameIdx[name] = pidx;
			pidx++;
		}
		f.release();
	}
	getAnimationNames(mapSknameIdx);

	for (int i = 0; i < reachParams.size(); i++){
		cv::Point3f temp;
		temp.x = reachParams[i].x;
		temp.y = reachParams[i].y;
		temp.z = reachParams[i].z;
		reachParams_pos.push_back(temp);
	}

	preProcess(root_rotation_list, root_translation_list);

	for (int i = 0; i < reachParams_pos.size(); i++){
		// transfer reachParams.x,y,x from world space to t-pose space using root_rotation_list, root_translation_list
		reachParams_pos[i].x -= root_translation_list[i].x;
		reachParams_pos[i].y -= root_translation_list[i].y;
		reachParams_pos[i].z -= root_translation_list[i].z;
		// transfer reachParams_pos from T-pose space to relative coordinate
		reachParams_pos[i].x -= root_world_pos_.x;
		reachParams_pos[i].y -= root_world_pos_.y;
		reachParams_pos[i].z -= root_world_pos_.z;
	}

	//pseudo_sample_points_generation	
	pseudo_sample_points = pseudo_sample_points_generation(reachParams_pos, 300);  //300, not 307

	//build kd_tree
	for (int i = 0; i < pseudo_sample_points.size(); i++){
		pseudo_sample_points_pos.push_back(pseudo_sample_points[i].Pos);
	}
	kd_tree_pseudo_sample_points_pos = KNN_build(pseudo_sample_points_pos);

}

void Reach::getAnimationNames(std::map<Ogre::String, int> &mapSkIdx)
{
	Ogre::Skeleton::LinkedSkeletonAnimSourceIterator linkIt = this->ent->getSkeleton()->getLinkedSkeletonAnimationSourceIterator();
	int i=0;
	while(linkIt.hasMoreElements())
	{
		const Ogre::LinkedSkeletonAnimationSource& link = linkIt.getNext();
		std::map<Ogre::String, int>::iterator it = mapSkIdx.find(link.skeletonName);
		if(it != mapSkIdx.end()){
			int idx = mapSkIdx[link.skeletonName];
			reachParams[idx].aniname = link.pSkeleton->getAnimation(0)->getName();
		}
//		animatename = aniNames[i];
		i++;
	}
}

void Reach::predictNewExamples(const std::vector<float>& mp)
{
	if (mp.size() != 3)  
		return;

	inputParam.clear();
	for(int i=0; i<mp.size(); i++)
		inputParam.push_back(mp[i]);

	Ogre::Vector3 query_pt(mp[0], mp[1], mp[2]);

	// map query_pt to relative coordinate
	query_pt.x -= root_world_pos_.x;
	query_pt.y -= root_world_pos_.y;
	query_pt.z -= root_world_pos_.z;
	
	cv::Point3f query_pt_cv(query_pt.x, query_pt.y, query_pt.z);
	this->interWeights = KNN_pseudo_sample_point(kd_tree_pseudo_sample_points_pos, pseudo_sample_points_pos, query_pt_cv, 4);
	
	//ontain clip time for the new queried samples
	this->obtainClipTime(this->interWeights, this->reachParams);
}

void Reach::obtainClipTime(const std::vector<std::pair<int, float>>& weights, const std::vector<Connection::ReachParam>& lps)
{
	std::vector<float> clips;
	float clipAvg = 0;
	for(int i=0; i<weights.size(); i++)
	{
		int lpIdx = weights[i].first;
		float weight = weights[i].second;
		float clip = lps[lpIdx].times[timeIdx+1] - lps[lpIdx].times[timeIdx];
		clips.push_back(clip);
		clipAvg += clip * weight;
	}
	//clipAvg /= weights.size();
	this->cliptime = clipAvg;

	timeScale.clear();
	for(int i=0; i<weights.size(); i++){
		timeScale.push_back(clips[i]/clipAvg);
	}
}

// cal orientation changing from initial to animation at t=0
void Reach::preProcess(std::vector<Ogre::Quaternion> &root_rotation_list, std::vector<Ogre::Vector3> &root_translation_list)
{
	float time = 0;
	int ani_numb = this->reachParams.size();
	// 1.0 T-pose
	Ogre::Quaternion qt = root->_getDerivedOrientation();;
	Ogre::Vector3 pt = root->_getDerivedPosition();
	for (int mm = 0; mm < ani_numb; mm++){
		// 2.0 pose at t = 0;
		skeleton->reset(true);
		Ogre::String animaiton_name = this->reachParams[mm].aniname;
		Ogre::Animation* anim = skeleton->getAnimation(animaiton_name);
		Ogre::TimeIndex timeIndex = anim->_getTimeIndex(time); 
		Ogre::Animation::NodeTrackList trackList = anim->_getNodeTrackList();
		Ogre::Animation::NodeTrackList::iterator it;
		//在时间点timeIndex， 把trackList在这个点的骨骼信息逐一设置到skeleton的bone
		for (it = trackList.begin(); it != trackList.end(); it++)
		{
			Ogre::Bone* bone = skeleton->getBone(it->first);
			it->second->applyToNode(bone, timeIndex);
		}

		Ogre::Quaternion q = root->_getDerivedOrientation();;
		Ogre::Vector3 p = root->_getDerivedPosition();

		// 3.0 cal dq
		Ogre::Quaternion dq = q * qt.Inverse();
		Ogre::Vector3 dp = p - pt;
		root_rotation_list.push_back(dq);
		root_translation_list.push_back(dp);
	}
}

void Reach::pure_play(float time, int ani_numb)
{
	//const std::vector<std::pair<int, float>> &weights = this->interWeights;
	//const std::vector<Connection::ReachParam> &lps = this->reachParams;
	//Ogre::Skeleton* skeleton = this->ent->getSkeleton();
	//Ogre::Vector3 initForward = Ogre::Vector3(0, 0, 1);
	//skeleton->reset(true);      //reset skeleton to T-pose(rest pose), so that we can apply skeleton anim(local quaternion) to animate skeleton 
	//Ogre::Bone* root = skeleton->getBone(rootname);
	//int bnum = skeleton->getNumBones();
	//Ogre::Quaternion rootParentRot = root->getParent()->_getDerivedOrientation();   //global orientation
	//Ogre::Quaternion rootParentRotInv = rootParentRot.Inverse();
	//std::vector<std::vector<Ogre::Quaternion>> boneRotsList(bnum);
	//std::vector<float> wList;
	//std::vector<Ogre::Vector3> rootPos;
	//std::vector<Ogre::Vector3> rootdirects;
	//initForward = root->_getDerivedOrientation().Inverse() * initForward;
	//Ogre::Skeleton* skeleton = this->ent->getSkeleton();
	//Ogre::Bone* root = skeleton->getBone(rootname);
	Ogre::String animaiton_name = this->reachParams[ani_numb].aniname;
	Ogre::Animation* anim = skeleton->getAnimation(animaiton_name);
	Ogre::TimeIndex timeIndex = anim->_getTimeIndex(time);   //timeIndex = time
	//time_i = time; ////Ling, temporary
	Ogre::Animation::NodeTrackList trackList = anim->_getNodeTrackList();
	Ogre::Animation::NodeTrackList::iterator it;
	root->setPosition(root->getInitialPosition());
	//在时间点timeIndex， 把trackList在这个点的骨骼信息逐一设置到skeleton的bone
	for (it = trackList.begin(); it != trackList.end(); it++)
	{
		Ogre::Bone* bone = skeleton->getBone(it->first);
		bone->setOrientation(bone->getInitialOrientation());
		it->second->applyToNode(bone, timeIndex);
	}

	Ogre::Bone*			bone = skeleton->getBone("rightHand");
	Ogre::Quaternion	q_l = bone->getOrientation();         //local orientation
	Ogre::Vector3		p_l = bone->getPosition();               //local position
	Ogre::Quaternion	q_w = bone->_getDerivedOrientation();         //local orientation
	Ogre::Vector3		p_w = bone->_getDerivedPosition();         //local orientation
	int a = 0;// set debug point here
}

//calculate end effector pose of  blending motion at time ta
void  Reach::cal_p_bar_t_a(Ogre::Vector3 &p_bar_t_a)
{
/////motion blending at ta
	//		time += timeScale[i] * cliptime;					//runtime, prta
	const std::vector<std::pair<int, float>> &weights = this->interWeights;
	const std::vector<Connection::ReachParam> &lps = this->reachParams;
	//Ogre::Skeleton* skeleton = this->ent->getSkeleton();
	skeleton->reset(true);      //reset skeleton to T-pose(rest pose), so that we can apply skeleton anim(local quaternion) to animate skeleton 
	int bnum = skeleton->getNumBones();
	std::vector<std::vector<Ogre::Quaternion>> boneRotsList(bnum);
	std::vector<float> wList;
	std::vector<Ogre::Vector3> rootPos;
	//for each bone, store animation inf from 4 example-animations：boneRotsList
	//each example animation has a weight pair (animation index, weight)
	for (int i = 0; i<weights.size(); i++)
	{
		skeleton->reset(true);
		wList.push_back(weights[i].second);
		int idx = weights[i].first;
		double time = lps[idx].times[timeIdx];
		time += timeScale[i] * cliptime;																//runtime, prta
		if (time > lps[idx].times[timeIdx + 1])
			time = lps[idx].times[timeIdx + 1];
		Ogre::Animation* anim = skeleton->getAnimation(lps[idx].aniname);
		Ogre::TimeIndex timeIndex = anim->_getTimeIndex(time);   //timeIndex = time
		//time_i = time; ////Ling, temporary
		Ogre::Animation::NodeTrackList trackList = anim->_getNodeTrackList();
		Ogre::Animation::NodeTrackList::iterator it;
		root->setPosition(root->getInitialPosition());
		//在时间点timeIndex， 把trackList在这个点的骨骼信息逐一设置到skeleton的bone
		for (it = trackList.begin(); it != trackList.end(); it++)
		{
			Ogre::Bone* bone = skeleton->getBone(it->first);
			bone->setOrientation(bone->getInitialOrientation());
			it->second->applyToNode(bone, timeIndex);
		}
		for (int j = 0; j<bnum; j++)
		{
			Ogre::Bone* bone = skeleton->getBone(j);
			Ogre::Quaternion q = bone->getOrientation();         //local orientation
			//Ogre::Vector3 t = bone->getPosition();               //local position
			if (bone->getName() == rootname){
				Ogre::Vector3 tw = bone->_getDerivedPosition();
				rootPos.push_back(tw);
			}
			boneRotsList[j].push_back(q);
		}
	}

	//基于动画第一帧的root_translation_list调整rootPos
	for (int i = 0; i < weights.size(); i++){
		rootPos[i] -= root_translation_list[weights[i].first];
	}

	//混合4个skeleton的信息，重新设置到skeleton
	skeleton->reset(true);
	for (int i = 0; i<bnum; i++)
	{
		Ogre::Quaternion qsyn = this->nlerpQuas(boneRotsList[i], wList);
		Ogre::Bone* bone = skeleton->getBone(i);
		bone->setOrientation(qsyn);
	}
	Ogre::Vector3 psyn = this->nlerpVecs(rootPos, wList);
	root->_setDerivedPosition(psyn);
/////motion blending at ta DONE

	Ogre::Bone*			bone = skeleton->getBone("rightHand");
	p_bar_t_a = bone->_getDerivedPosition();         //world position
}


void Reach::motionSynthesis(const int approach_mix_type, const int return_mix_type)
{
	bool if_return = false;
	if (timeIdx == 1)
		if_return = true;

#define IK
#ifdef IK
	Ogre::Vector3 p_r_t_iminus1 = p_r_t_i; 
	time_iminus1 = time_i;
	time_i = runtime;
#endif

	const std::vector<std::pair<int, float>> &weights = this->interWeights;
	const std::vector<Connection::ReachParam> &lps = this->reachParams;
	skeleton->reset(true);      //reset skeleton to T-pose(rest pose), so that we can apply skeleton anim(local quaternion) to animate skeleton 
	int bnum = skeleton->getNumBones();
	std::vector<std::vector<Ogre::Quaternion>> boneRotsList(bnum);
	std::vector<float> wList;
	std::vector<Ogre::Vector3> rootPos;
	//for each bone, store animation inf from 4 example-animations：boneRotsList
	//each example animation has a weight pair (animation index, weight)
	for(int i=0; i<weights.size(); i++)
	{
		skeleton->reset(true);
		wList.push_back(weights[i].second);
		int idx = weights[i].first;
		double time = lps[idx].times[timeIdx];
		time += timeScale[i]*runtime;
		if(time > lps[idx].times[timeIdx+1]) //?
			time = lps[idx].times[timeIdx+1];
		Ogre::Animation* anim = skeleton->getAnimation(lps[idx].aniname);
		Ogre::TimeIndex timeIndex = anim->_getTimeIndex(time);   //timeIndex = time
		Ogre::Animation::NodeTrackList trackList = anim->_getNodeTrackList();
		Ogre::Animation::NodeTrackList::iterator it;
		//在时间点timeIndex， 把trackList在这个点的骨骼信息逐一设置到skeleton的bone
		for(it = trackList.begin(); it!=trackList.end(); it++)
		{
			Ogre::Bone* bone = skeleton->getBone(it->first);
			it->second->applyToNode(bone, timeIndex);                   
		}
		for(int j=0; j<bnum; j++)
		{
			Ogre::Bone* bone = skeleton->getBone(j);
			Ogre::Quaternion q = bone->getOrientation();         //local orientation
			//Ogre::Vector3 t = bone->getPosition();               //local position
			if(bone->getName()==rootname){
				Ogre::Vector3 tw = bone->_getDerivedPosition();
				rootPos.push_back(tw);
			}		
			boneRotsList[j].push_back(q);
		}
	}

	//基于动画第一帧的root_translation_list调整rootPos到原点
	for (int i = 0; i < weights.size(); i++){
		rootPos[i] -= root_translation_list[weights[i].first];
	}

#ifndef IK
		//混合4个skeleton的信息，重新设置到skeleton
		skeleton->reset(true);
		for(int i=0; i<bnum; i++)
		{
			Ogre::Quaternion qsyn = this->nlerpQuas(boneRotsList[i], wList);
			Ogre::Bone* bone = skeleton->getBone(i);
			bone->setOrientation(qsyn);
		}
		Ogre::Vector3 psyn = this->nlerpVecs(rootPos, wList);
		root->_setDerivedPosition(psyn);
		return;
	
#else
	
	
	if (!if_return)
	{
		//end effector pose of final blending motion 
		cal_p_bar_t_a(p_bar_t_a);   //skeleton has be reset to t-pose
	}

	//calculate blended skeleton at ti
	skeleton->reset(true);
	std::vector<Ogre::Quaternion> referQuatList; //referene quatern for IK
	for (int i = 0; i<bnum; i++)
	{
		Ogre::Quaternion qsyn = this->nlerpQuas(boneRotsList[i], wList);
		Ogre::Bone* bone = skeleton->getBone(i);
		bone->setOrientation(qsyn);
		referQuatList.push_back(qsyn);
	}
	Ogre::Vector3 psyn = this->nlerpVecs(rootPos, wList);
	root->_setDerivedPosition(psyn);

	//calculate end-effector position, Pbar(ti)
	Ogre::Bone*			bone	=	skeleton->getBone("rightHand");
	Ogre::Vector3		p_bar_t_i = bone->_getDerivedPosition();         //world position

	//calculate d_p_i
	float t_a = cliptime;// 1.6;// lps[0].times[1];
	float alpha = cal_alpha(time_i, time_iminus1, t_a, approach_mix_type);

	Ogre::Vector3 p_r_t_a = Ogre::Vector3 ( inputParam[0], inputParam[1], inputParam[2] );
	Ogre::Vector3 d_p = p_r_t_a - p_bar_t_a;
	Ogre::Vector3 d_p_iminus1 = d_p_i;
	d_p_i = alpha * d_p + ( 1 - alpha ) * d_p_iminus1;

	if (if_return)
	{ 
		alpha = cal_alpha(time_i, time_iminus1, t_a, return_mix_type);

		float x = MIN(time_i / t_a, 1.0);
		alpha = x;            //linear

		d_p_i = (1 - alpha) * d_p;// +alpha * d_p_iminus1;
    }

	//calculate Pr(ti) = Pbar(ti) + dPi 
	p_r_t_i = p_bar_t_i + d_p_i;

	// IK which uses entity's skeleton
	//note: before using IK, the skeleton is of blended motion at ti
	effector_target.targetPosition = p_r_t_i;
	IK_params.Targets[0] = effector_target;
	//IK_solver.retargetIKDirect(IK_params);
	IK_solver.retargetIKJacobian(IK_params, referQuatList);
#endif

}


float Reach::cal_alpha(float time_i, float time_iminus1, float t_a, const int type)
{
	float alpha = 0;
	float x = MIN(time_i / t_a, 1.0);

	switch (type)
	{
		case 0:
			alpha = (time_i - time_iminus1) / (t_a - time_iminus1);  //non_linear//usc
		case 1:
			alpha = x;			//linear
		case 2:
			alpha = pow(x, 0.5); //up_linear
		case 3:
			alpha = pow(x, 2); //down_linear
	}

	return alpha;
}

void Reach::updateTimeStep()
{
	this->runtime = 0;
	timeIdx++;
	if(timeIdx >= timeIdxMax)
		timeIdx = 0;

	time_iminus1 = 0;
	d_p_i = Ogre::Vector3(0.0);
}

Ogre::Quaternion Reach::nlerpQuas(const std::vector<Ogre::Quaternion>& rots, const std::vector<float>& weights)
{
	assert(rots.size() == weights.size());
	Ogre::Quaternion q;
	if(rots.size()==0)
		return q;	
	float wsum = weights[0];
	q = rots[0];
	for(int i=1; i<rots.size(); i++)
	{
		wsum += weights[i];
		float p = weights[i]/wsum;
		q = Ogre::Quaternion::nlerp(p, q, rots[i]);
	}
	return q;
}

Ogre::Vector3 Reach::nlerpVecs(const std::vector<Ogre::Vector3>& tras, const std::vector<float>& weights)
{
	assert(tras.size() == weights.size());
	Ogre::Vector3 v = Ogre::Vector3(0,0,0);
	if(tras.size()==0)
		return v;
	float wsum = 0;
	int i=0;
	while(i < weights.size()){
		wsum += weights[i];
		i++;
	}
	for(i=0; i<tras.size(); i++)
	{
		v += tras[i] * weights[i]/wsum;
	}
	return v;
}

void Reach::resetRoot()
{
	//Ogre::Bone* root = this->ent->getSkeleton()->getBone(rootname);
	root->setPosition(root->getInitialPosition());
	root->setOrientation(root->getInitialOrientation());
}

void Reach::setRootState(Ogre::Vector3 pos, Ogre::Vector3 dir)
{
	rootStateWorld.dir = dir; 
	rootStateWorld.pos = pos;
	Ogre::Bone* root = this->ent->getSkeleton()->getBone(rootname);
	Ogre::Quaternion parentRotInv = root->getParent()->_getDerivedOrientation().Inverse();
}

void Reach::addRunTime(double dt) 
{
	this->runtime += dt;
}


//////////////////////////////////////////////////
/////////////////////  LING  /////////////////////
//////////////////////////////////////////////////

std::vector<std::pair<int, float>> Reach::KNN_pseudo_sample_point(cv::flann::Index* kd_tree_pseudo_sample_point, std::vector<cv::Point3f> &input_3Dpoints, const cv::Point3f &query_pt, const int k)
{
	//4. search kd_tree to calculate the pseudo-sample-motion index and weight for a point
	std::vector<std::pair<int, float>> index_dists2 = KNN_search(kd_tree_pseudo_sample_point, input_3Dpoints, query_pt, k);
	std::vector<std::pair<int, float>> index_weights;
	
	////approach 0
	//float sum = 0;
	//for (int m = 0; m < index_dists2.size(); m++)
	//{
	//	float dist = sqrt(index_dists2[m].second);
	//	sum += 1.0/dist;
	//	if (dist < 1e-6){  //special case
	//		sum = 1e6;
	//		break;
	//	}
	//}
	//for (int m = 0; m < index_dists2.size(); m++)
	//{
	//	float dist = sqrt(index_dists2[m].second);
	//	float weight = ( 1.0 / dist ) / sum;
	//	if (dist < 1e-6)  //special case
	//		weight = 1.0;
	//	weight = (weight < 1e-6) ? 0 : weight;   //special case
	//	index_weights.push_back(std::pair<int, float>(index_dists2[m].first, weight));
	//}
	//approach 1
	std::vector<float> dists;
	std::vector<float> outWeights;
	for (int m = 0; m < index_dists2.size(); m++)
	{
		dists.push_back( sqrt(index_dists2[m].second) );
	}
	generateDistWeights(dists, outWeights);
	for (int m = 0; m < index_dists2.size(); m++)
	{
		//dists.push_back(sqrt(index_dists2[m].second));
		index_weights.push_back(std::pair<int, float>(index_dists2[m].first, outWeights[m]));
	}

	std::cout << "cal index_weights DONE" << std::endl;
	//END, search kd_tree to calculate the pseudo-sample-motion index and weight for a point





	//5. cal original motion index and weights (for the 32 original sample motions)
	//index_weights (pseudo_sample_index, weight)
	// |
	// V
	//index_weights_original(original_sample_index, weight)
	std::vector<std::pair<int, float>> index_weights_original;
	std::vector<int> original_motion_indeces;
	std::vector<float> original_motion_weights;
	for (int m = 0; m < index_weights.size(); m++)
	{
		std::vector<std::pair<int, float>> pseudo_sample_point_index_weight = pseudo_sample_points[index_weights[m].first].index_weight;
		for (int n = 0; n < pseudo_sample_point_index_weight.size(); n++)
		{
			int motion_index = pseudo_sample_point_index_weight[n].first;
			float motion_weight = pseudo_sample_point_index_weight[n].second * index_weights[m].second;
			std::pair<int, float> temp(motion_index, motion_weight);
			index_weights_original.push_back(temp);
		}
	}

	std::vector<std::pair<int, float>> index_weights_original_compacted;
	index_weights_original_compacted.push_back(index_weights_original[0]);
	for (int m = 1; m < index_weights_original.size(); m++)
	{
		bool flag = false;
		for (int n = 0; n < index_weights_original_compacted.size(); n++)
		{
			if (index_weights_original_compacted[n].first == index_weights_original[m].first)
			{
				index_weights_original_compacted[n].second += index_weights_original[m].second ;
				flag = true;
				break;
			}
		}
		if (!flag)
		{
			index_weights_original_compacted.push_back(index_weights_original[m]);
		}
	}
	


	return index_weights_original_compacted;
}

void Reach::KNN_test()
{
	//1. ini points
	cv::Point3f p0(float(0), float(0), float(0)), p1(2, 3, 1), p2(5, 4, 2), p3(9, 6, 1), p4(4, 7, 2), p5(8, 1, 3), p6(7, 2, 1);
	std::vector<cv::Point3f> source;
	source.push_back(p0);
	source.push_back(p1);
	source.push_back(p2);
	source.push_back(p3);
	source.push_back(p4);
	source.push_back(p5);
	source.push_back(p6);
	
	//2. pseudo sample points generation
	std::vector<pseudoSamplePoint> pseudo_sample_points = pseudo_sample_points_generation(source, 300);  //300, not 307
	
	//3. build kd_tree
	std::vector <cv::Point3f> pseudo_sample_points_pos;
	for (int i = 0; i < pseudo_sample_points.size(); i++){
		pseudo_sample_points_pos.push_back(pseudo_sample_points[i].Pos);
	}
	cv::flann::Index* kd_tree = KNN_build(pseudo_sample_points_pos);
	
	//4. search kd_tree to calculate the pseudo-sample-motion index and weight for a point
	std::vector<std::pair<int, float>> index_dists2 = KNN_search(kd_tree, pseudo_sample_points_pos, cv::Point3f(0, 0, 0), 4);
	std::vector<std::pair<int, float>> index_weights;
	float sum = 0;
	for (int m = 0; m < index_dists2.size(); m++)
	{
		float dist = sqrt(index_dists2[m].second);
		sum += dist;
	}
	for (int m = 0; m < index_dists2.size(); m++)
	{
		float dist = sqrt(index_dists2[m].second);
		float weight = dist / sum;
		index_weights.push_back(std::pair<int, float>(m, weight));
	}
	std::cout << "cal index_weights DONE" << std::endl;
	//END, search kd_tree to calculate the pseudo-sample-motion index and weight for a point


	//5. cal original motion index and weights (for the 32 original sample motions)
	std::vector<int> original_motion_indeces; 
	std::vector<float> original_motion_weights;
	for (int m = 0; m < index_weights.size(); m++)
	{
		std::vector<std::pair<int, float>> pseudo_sample_point_index_weight = pseudo_sample_points[index_weights[m].first].index_weight;
		for (int n = 0; n < pseudo_sample_point_index_weight.size(); n++)
		{
			int motion_index = pseudo_sample_point_index_weight[n].first;
			float motion_weight = pseudo_sample_point_index_weight[n].second;
			original_motion_indeces.push_back(motion_index);
			original_motion_weights.push_back(motion_weight);
		}
	}

	//TODO: at each update, blend motion based on original_motion_indeces and original_motion_weights
	//motion synthesis?
	//void motion_blend( std::vector<MOTION> &original_motions, std::vector<int> motion_indeces, std::vector<float> motion_weights, MOTION &output_motion)


}

cv::flann::Index* Reach::KNN_build(std::vector<cv::Point3f> &input_3Dpoints, int KDTreeIndexParams)
{
	// KDTreeIndexParams:  When passing an object of this type, the index constructed will consist of a set of randomized kd-trees which will be searched in parallel
	cv::flann::KDTreeIndexParams indexParams(KDTreeIndexParams); //[1, 16]  //创建多颗kd-tree，并行搜索
	//cv::flann::Index kdtree(cv::Mat(input_3Dpoints).reshape(1), indexParams);
	cv::flann::Index* kdtree = new cv::flann::Index(cv::Mat(input_3Dpoints).reshape(1), indexParams);
	return kdtree;
}

std::vector<std::pair<int, float>> Reach::KNN_search( cv::flann::Index*				 kd_tree,
													  std::vector<cv::Point3f>		 &input_3Dpoints,
													  const cv::Point3f					 &query_pt, 
													  int							 numb_nearest_neighb)
{
	std::vector<std::pair<int, float>> neighb_points;
	std::vector<float> query;
	query.push_back(query_pt.x);
	query.push_back(query_pt.y);
	query.push_back(query_pt.z);
	int k = numb_nearest_neighb;                                      //4;//1; //number of nearest neighbors
	std::vector<int> indices(k);                                     //找到点的索引
	std::vector<float> dists(k);                                     //x^2+y^2+z^2

	cv::flann::SearchParams params(32);                             //搜索次数，越大，越准，越费时
	kd_tree->knnSearch(query, indices, dists, k, params);

	for (int i = 0; i < numb_nearest_neighb; i++){
		std::pair<int, float> temp = std::pair<int, float>(indices[i], dists[i]);
		neighb_points.push_back(temp); //std::cout << indices[i] << " " << dists[i] << std::endl;
	}

	return neighb_points;
}

float Random(float r_min, float r_max)
{
	static bool initRand = false;
	if (!initRand)
	{
		srand(int(time(NULL)));
		initRand = true;
	}
	float frand = (float)rand() / (float)RAND_MAX;
	frand = r_min + frand*(r_max - r_min);
	return frand;
}

std::vector<pseudoSamplePoint> Reach::pseudo_sample_points_generation( std::vector<cv::Point3f> &input_3Dpoints,
																	   int						number_pseudo_points)
{
	//add original motion sample
	for (int i = 0; i < reachParams_pos.size(); i++){
		pseudoSamplePoint pseudo_sample_point;
		std::pair<int, float> temp;
		temp.first = i; temp.second = 1;
		pseudo_sample_point.index_weight.push_back(temp);
		pseudo_sample_point.Pos = cv::Point3f(reachParams_pos[i].x, reachParams_pos[i].y, reachParams_pos[i].z);
		pseudo_sample_points.push_back(pseudo_sample_point);
	}


	//boudning box 
	cv::Point3f min = cv::Point3f(Ogre::Math::POS_INFINITY, Ogre::Math::POS_INFINITY, Ogre::Math::POS_INFINITY);
	cv::Point3f max = cv::Point3f(Ogre::Math::NEG_INFINITY, Ogre::Math::NEG_INFINITY, Ogre::Math::NEG_INFINITY);
	for (int i = 0; i < input_3Dpoints.size(); i++)
	{
		if (input_3Dpoints[i].x < min.x)  min.x = input_3Dpoints[i].x;
		if (input_3Dpoints[i].y < min.y)  min.y = input_3Dpoints[i].y;
		if (input_3Dpoints[i].z < min.z)  min.z = input_3Dpoints[i].z;

		if (input_3Dpoints[i].x > max.x)  max.x = input_3Dpoints[i].x;
		if (input_3Dpoints[i].y > max.y)  max.y = input_3Dpoints[i].y;
		if (input_3Dpoints[i].z > max.z)  max.z = input_3Dpoints[i].z;
	}


	//pseudo sample points generation
	//std::vector<pseudoSamplePoint> pseudo_sample_points;
//	std::vector<cv::Point3f> pseudo_sample_points(input_3Dpoints);
	float expected_mini_gap = ((max.x - min.x) + (max.y - min.y) + (max.z - min.z)) / 3.0 * (1.0/20);// (1.0 / number_pseudo_points);
	for (int i = 0; i < number_pseudo_points; i++){
		float delta = 3 * expected_mini_gap;
		pseudoSamplePoint pseudo_sample_point;
		pseudo_sample_point.Pos = cv::Point3f(Random(min.x - delta, max.x + delta), Random(min.y - delta, max.y + delta), Random(min.z - delta, max.z + delta));
		
		//check pseudo_sample_point
		bool valid_pseudo = true;
		// check0
		for (int j = 0; j < pseudo_sample_points.size(); j++){
			float dist2 = pow(pseudo_sample_point.Pos.x - pseudo_sample_points[j].Pos.x, 2) +
						  pow(pseudo_sample_point.Pos.y - pseudo_sample_points[j].Pos.y, 2) +
						  pow(pseudo_sample_point.Pos.z - pseudo_sample_points[j].Pos.z, 2);
			float dist = sqrt(dist2);
			if (dist < expected_mini_gap){
				valid_pseudo = false;
				break;
			}
		}
		// check1 if pseudo_sample_point is in (x+,z-), set to invalid
		if (pseudo_sample_point.Pos.x > 0 && pseudo_sample_point.Pos.z < 0){
			valid_pseudo = false;
		}

		//add pseudo point to list
		if (valid_pseudo){
			//find nearest k original sample motions which will be bleneded for this pseudo sample point
			// build and search kd_tree
			cv::flann::Index* kd_tree = KNN_build(input_3Dpoints);  //can be put outside the loop
			std::vector<std::pair<int, float>> index_dist2 = KNN_search(kd_tree, input_3Dpoints, pseudo_sample_point.Pos, 4);
			//TODO : filter samples from different zone

			std::vector<std::pair<int, float>> index_weights;
			// cal weight : reset index_weights.second, which is distance
			/*//approach 0: calculate weights using distance
			float sum = 0;
			for (int m = 0; m < index_dist2.size(); m++)
			{
				float dist = sqrt(index_dist2[m].second);
				sum += 1.0 / dist;
				if (dist < 1e-6){
					sum = 1e6;
					break;
				}
			}

			for (int m = 0; m < index_dist2.size(); m++)
			{
				std::pair<int, float> temp;
				temp.first = index_dist2[m].first;
				float dist = sqrt(index_dist2[m].second);
				float weight = (1.0/dist) / sum;
				if (dist < 1e-6)
					weight = 1.0;
				weight = (weight < 1e-6) ? 0 : weight;
				temp.second = weight;
				index_weights.push_back(temp);
			}*/
			//approach 1: calculate weights using random function
			std::vector<float> outWeights;
			generateRandomWeight(index_dist2.size(), outWeights);
			for (int m = 0; m < index_dist2.size(); m++)
			{
				std::pair<int, float> temp;
				temp.first = index_dist2[m].first;
				temp.second = outWeights[m];
				index_weights.push_back(temp);
			}

			//? TODO:blend a new motion
			//i=0,1,2,3
			//motion_index = result[i].first;
			//weight = result[i].second;

			//TODO: update the pseudo_sample_point's end effector position using the sample motions and weights(index_weights)
			//pseudo_sample_point.Pos = cv::Point3f(Random(min.x, max.x), Random(min.y, max.y), Random(min.z, max.z));
			getParameter(index_weights, pseudo_sample_point.Pos);
			
			//check pseudo_sample_point
			//valid_pseudo = true;
			// check0
			for (int j = 0; j < pseudo_sample_points.size(); j++){
				float dist2 = pow(pseudo_sample_point.Pos.x - pseudo_sample_points[j].Pos.x, 2) +
							  pow(pseudo_sample_point.Pos.y - pseudo_sample_points[j].Pos.y, 2) +
							  pow(pseudo_sample_point.Pos.z - pseudo_sample_points[j].Pos.z, 2);
				float dist = sqrt(dist2);
				if (dist < expected_mini_gap){
					valid_pseudo = false;
					break;
				}
			}
			// check1 if pseudo_sample_point is in (x+,z-), set to invalid
			if (pseudo_sample_point.Pos.x > 0 && pseudo_sample_point.Pos.z < 0){
				valid_pseudo = false;
			}
			if (valid_pseudo){
				pseudo_sample_point.index_weight = index_weights;
				pseudo_sample_points.push_back(pseudo_sample_point);
			}
		}

		if (!valid_pseudo)
			i--; //regenerate 
	}

	////add original motion sample // been moved to the front of this function -> add original points first
	//for (int i = 0; i < reachParams_pos.size(); i++){
	//	pseudoSamplePoint pseudo_sample_point;
	//	std::pair<int, float> temp;
	//	temp.first = i; temp.second = 1;
	//	pseudo_sample_point.index_weight.push_back(temp);
	//	pseudo_sample_point.Pos = cv::Point3f(reachParams_pos[i].x, reachParams_pos[i].y, reachParams_pos[i].z);
	//	pseudo_sample_points.push_back(pseudo_sample_point);
	//}


	return pseudo_sample_points;
}

void Reach::generateRandomWeight(int nK, std::vector<float>& outWeights)
{
float delta = 0.02f;
outWeights.resize(nK);
float weightSum = 0.0;
for (int i = 0; i<nK - 1; i++)
{
	float w = Random(std::max(-delta, -delta - weightSum), std::min(1.f + delta, 1.f + delta - weightSum));
	outWeights[i] = w;
	weightSum += w;
}
outWeights[nK - 1] = 1.f - weightSum;
}

void Reach::getParameter(std::vector<std::pair<int, float>> &index_weights, cv::Point3f &end_effector_position)
{
	/////motion blending at ta
	//		time += timeScale[i] * cliptime;					//runtime, prta
	const std::vector<std::pair<int, float>> &weights = index_weights;// this->interWeights;
	const std::vector<Connection::ReachParam> &lps = this->reachParams;
	Ogre::Skeleton* skeleton = this->ent->getSkeleton();
	Ogre::Vector3 initForward = Ogre::Vector3(0, 0, 1);
	skeleton->reset(true);      //reset skeleton to T-pose(rest pose), so that we can apply skeleton anim(local quaternion) to animate skeleton 
	//Ogre::Bone* root = skeleton->getBone(rootname);
	int bnum = skeleton->getNumBones();
	Ogre::Quaternion rootParentRot = root->getParent()->_getDerivedOrientation();   //global orientation
	Ogre::Quaternion rootParentRotInv = rootParentRot.Inverse();
	std::vector<std::vector<Ogre::Quaternion>> boneRotsList(bnum);
	std::vector<float> wList;
	std::vector<Ogre::Vector3> rootPos;
	std::vector<Ogre::Vector3> rootdirects;
	initForward = root->_getDerivedOrientation().Inverse() * initForward;
	Ogre::Vector3 a = root->_getDerivedPosition();
	//for each bone, store animation inf from 4 example-animations：boneRotsList
	//each example animation has a weight pair (animation index, weight)
	for (int i = 0; i<weights.size(); i++)
	{
		skeleton->reset(true);
		wList.push_back(weights[i].second);
		int idx = weights[i].first;
		//double time = lps[idx].times[timeIdx];
		//time += timeScale[i] * cliptime;					//runtime, prta
		//if (time > lps[idx].times[timeIdx + 1]) //?
		//	time = lps[idx].times[timeIdx + 1];
		double time = lps[idx].times[1];																	//t1
		Ogre::Animation* anim = skeleton->getAnimation(lps[idx].aniname);
		Ogre::TimeIndex timeIndex = anim->_getTimeIndex(time);   //timeIndex = time
		//time_i = time; ////Ling, temporary
		Ogre::Animation::NodeTrackList trackList = anim->_getNodeTrackList();
		Ogre::Animation::NodeTrackList::iterator it;
		root->setPosition(root->getInitialPosition());
		//在时间点timeIndex， 把trackList在这个点的骨骼信息逐一设置到skeleton的bone
		for (it = trackList.begin(); it != trackList.end(); it++)
		{
			Ogre::Bone* bone = skeleton->getBone(it->first);
			bone->setOrientation(bone->getInitialOrientation());
			it->second->applyToNode(bone, timeIndex);
		}
		for (int j = 0; j<bnum; j++)
		{
			Ogre::Bone* bone = skeleton->getBone(j);
			Ogre::Quaternion q = bone->getOrientation();         //local orientation
			//Ogre::Vector3 t = bone->getPosition();               //local position
			if (bone->getName() == rootname){
				//	Ogre::Quaternion qw = bone->_getDerivedOrientation();    //global orientation
				//	q = rootInitRot[i] * qw;
				//	rootdirects.push_back(q * initForward);
				////	q = rootParentRotInv * q;

				//	Ogre::Vector3 tw = bone->_getDerivedPosition();    //gloal position
				////	t = rootInitRot[i] * (tw -  rootInitTra[i]) + rootStateWorld.pos;
				//	t = rootOriRot * rootInitRot[i] * (tw - rootStateWorld.pos + rootInitTra[i]) + rootStateWorld.pos;
				//	t = rootParentRotInv * (t - root->getParent()->_getDerivedPosition());
				//	rootPos.push_back(t);

				Ogre::Vector3 tw = bone->_getDerivedPosition();
				rootPos.push_back(tw);
			}
			boneRotsList[j].push_back(q);
		}
	}

	//基于动画第一帧的root_translation_list调整rootPos
	for (int i = 0; i < weights.size(); i++){
		rootPos[i] -= root_translation_list[weights[i].first];
	}

	//混合4个skeleton的信息，重新设置到skeleton
	skeleton->reset(true);
	for (int i = 0; i<bnum; i++)
	{
		Ogre::Quaternion qsyn = this->nlerpQuas(boneRotsList[i], wList);
		Ogre::Bone* bone = skeleton->getBone(i);
		/*if(bone->getName() == rootname)
		qsyn = rootParentRotInv * rootOriRot * qsyn;*/
		bone->setOrientation(qsyn);
	}
	Ogre::Vector3 psyn = this->nlerpVecs(rootPos, wList);
	root->_setDerivedPosition(psyn);
	/////motion blending at ta DONE

	Ogre::Bone*			bone = skeleton->getBone("rightHand");
	//Ogre::Quaternion	q_l = bone->getOrientation();         //local orientation
	//Ogre::Vector3		p_l = bone->getPosition();               //local position
	//Ogre::Quaternion	q_w = bone->_getDerivedOrientation();         //world orientation
	Ogre::Vector3		p_w = bone->_getDerivedPosition();         //world position
	

	// map p_w to relative coordinate
	Ogre::Skeleton* skeleton_ = this->ent->getSkeleton();
	skeleton_->reset(true);  //t-pose
	Ogre::Bone* root_ = skeleton_->getBone(rootname);
	Ogre::Vector3 root_world_pos = root_->_getDerivedPosition();
	/*for (int i = 0; i < reachParams_pos.size(); i++){
		reachParams_pos[i].x -= root_world_pos.x;
		reachParams_pos[i].y -= root_world_pos.y;
		reachParams_pos[i].z -= root_world_pos.z;
	}*/
	p_w -= root_world_pos;

	//output value
	end_effector_position.x = p_w.x;
	end_effector_position.y = p_w.y;
	end_effector_position.z = p_w.z;
}


void Reach::generateDistWeights(std::vector<float>& dists, std::vector<float>& outWeights)
{
	int nK = dists.size();
	outWeights.resize(nK);

	double weightSum = 0.f;
	for (int i = 0; i<nK; i++)
	{
		float weight = 1.0f / dists[i] - 1.0f / dists[nK - 1];
		weightSum += weight;
		outWeights[i] = weight;
	}

	std::stringstream strstr;
	strstr << "weights = ";
	for (int i = 0; i<nK; i++)
	{
		outWeights[i] = (float)((double)outWeights[i] / weightSum);
		strstr << outWeights[i] << " ,";
	}
	strstr << std::endl;
}


}