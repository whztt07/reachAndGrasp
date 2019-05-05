#include "locomotion.h"

namespace Connection
{

void Locomotion::initLocomotion(Ogre::String fname, Ogre::Entity* entity)
{
	//fname = "C:/Users/ling/Desktop/sinbad-ogre67/sinbad-ogre/Samples/DualQuaternion/Connection/motion_params.xml"; //Ling
	//first read locomotion xml file, initialize locomotion parameters
	this->ent = entity;
	rootname = "hips1";
	readLocomotionFile(fname);
	getVelocityParameters();
	std::vector<std::vector<float>> lps;
	for(int i=0; i<locomotionParams.size(); i++)
	{
		LocomotionParam l = locomotionParams[i];
		std::vector<float> lp(3);
		lp[0] = l.velf;
		lp[1] = l.vels;
		lp[2] = l.velt;
		lps.push_back(lp);
	}
	baryInterp.constructTetrahedron(lps);
	runtime = 0;
	cliptime = 0;
	lasttime = 0;
	timeIdxMax = 2;
	timeIdx = 0;
}

void Locomotion::resetLocomotion()
{
	runtime = 0;
	cliptime = 0;
	lasttime = 0;
	timeIdxMax = 2;
	timeIdx = 0;
	diffs.clear();
	inputParam.clear();
}

LocomotionParam Locomotion::initLocomotionParam(float vf, float vs, float vt)
{
	LocomotionParam lp;
	lp.velf = vf;
	lp.vels = vs;
	lp.velt = vt;
	return lp;
}

void Locomotion::readLocomotionFile(Ogre::String fname)
{
	std::map<Ogre::String, int> mapSknameIdx;
	cv::FileStorage f(fname, cv::FileStorage::READ);
	locomotionParams.clear();
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
			float fv = param["forward_vel"];
			float tv = param["turn_vel"];
			float sv = param["side_vel"];
			LocomotionParam lp = this->initLocomotionParam(fv, tv, sv);
			
			lp.times.push_back(param["T0"]);
			lp.times.push_back(param["T1"]);
			lp.times.push_back(param["T2"]);
			lp.times.push_back(param["T3"]);
			locomotionParams.push_back(lp);
			mapSknameIdx[name] = pidx;
			pidx++;
		}
		f.release();
	}
	getAnimationNames(mapSknameIdx);

//	cv::FileStorage fs("..\\..\\Samples\\DualQuaternion\\Connection\\motion_params_vice.xml", cv::FileStorage::WRITE); 	
	
}

void Locomotion::getAnimationNames(std::map<Ogre::String, int> &mapSkIdx)
{
	Ogre::Skeleton::LinkedSkeletonAnimSourceIterator linkIt = this->ent->getSkeleton()->getLinkedSkeletonAnimationSourceIterator();
	int i=0;
	while(linkIt.hasMoreElements())
	{
		const Ogre::LinkedSkeletonAnimationSource& link = linkIt.getNext();
		std::map<Ogre::String, int>::iterator it = mapSkIdx.find(link.skeletonName);
		if(it != mapSkIdx.end()){
			int idx = mapSkIdx[link.skeletonName];
			locomotionParams[idx].aniname = link.pSkeleton->getAnimation(0)->getName();
		}
//		animatename = aniNames[i];
		i++;
	}
}

void Locomotion::getVelocityParameters()
{
	Ogre::Skeleton* skeleton = this->ent->getSkeleton();
	for(int i=0; i<skeleton->getNumBones(); i++)
		skeleton->getBone(i)->setManuallyControlled(true);
	skeleton->reset(true);
	Ogre::Bone* root = skeleton->getBone(this->rootname);
	Ogre::Vector3 forwardDirInit = root->_getDerivedOrientation().Inverse() * Ogre::Vector3(0,0,1);
	int tup = 2;
	int tsum = tup + 1;
	int ridx = root->getHandle();
	for(int i=0; i<locomotionParams.size(); i++)
	{
		LocomotionParam& lp = locomotionParams[i]; 
		Ogre::Animation* anim = skeleton->getAnimation(locomotionParams[i].aniname);
		Ogre::Animation::NodeTrackList track = anim->_getNodeTrackList();
		Ogre::Animation::NodeTrackList::iterator it = track.find(ridx);
		std::vector<float> vf(tup), vs(tup), vt(tup);
		std::vector<Ogre::Vector3> forwardDirVec(tsum), forwardPosVec(tsum); 
		for(int j=0; j<tsum; j++)
		{
			Ogre::TimeIndex timeindex = anim->_getTimeIndex(lp.times[j]);
			it->second->applyToNode(root, timeindex);
			forwardPosVec[j] = root->_getDerivedPosition();
			forwardDirVec[j] = root->_getDerivedOrientation() * forwardDirInit;
			forwardDirVec[j].y = 0;
			forwardPosVec[j].y = 0;
			forwardDirVec[j].normalise();
			root->reset();
		}
		float vfavg = 0, vsavg = 0, vtavg = 0;
		for(int j=0; j<tup; j++)
		{
			float deltat = lp.times[j+1] - lp.times[j];
			Ogre::Vector3 move = forwardPosVec[j+1] - forwardPosVec[j];
			vf[j] = move.dotProduct(forwardDirVec[j]); //forward speed
			vs[j] = std::sqrt(move.squaredLength() - vf[j]*vf[j]); //sideway speed
			Ogre::Radian angle = forwardDirVec[j].angleBetween(forwardDirVec[j+1]);
			vt[j] = angle.valueDegrees(); //turn angle speed
			Ogre::Vector3 vsr = forwardDirVec[j].crossProduct(move);
			if(vsr.y<0) // if right forward direction, value is negative 
				vs[j] = -vs[j];
			vsr = forwardDirVec[j].crossProduct(forwardDirVec[j+1]);
			if(vsr.y<0)
				vt[j] = -vt[j];	
			vf[j] /= deltat;
			vs[j] /= deltat;
			vt[j] /= deltat;
			vfavg += vf[j];
			vsavg += vs[j];
			vtavg += vt[j];
		}
		vfavg /= tup;
		vsavg /= tup;
		vtavg /= tup;
		lp.velf = vfavg;
		lp.vels = vsavg;
		lp.velt = vtavg;
	}
}



void Locomotion::predictNewExamples(const std::vector<float>& mp)
{
	inputParam.clear();
	for(int i=0; i<mp.size(); i++)
		inputParam.push_back(mp[i]);
	baryInterp.calculateTetraheWeights(mp, this->interWeights);
	this->findNearestParam(mp);
	 this->obtainClipTime(this->interWeights, this->locomotionParams);
	this->obtainInitChange(this->interWeights, this->locomotionParams);

//	this->calcEndError();
}

void Locomotion::obtainClipTime(const std::vector<std::pair<int, float>>& weights, const std::vector<Connection::LocomotionParam>& lps)
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

void Locomotion::motionSynthesis()
{
	const std::vector<std::pair<int, float>> &weights = this->interWeights;
	const std::vector<Connection::LocomotionParam> &lps = this->locomotionParams;
	Ogre::Skeleton* skeleton = this->ent->getSkeleton();
	Ogre::Vector3 initForward = Ogre::Vector3(0,0,1);
	skeleton->reset(true);      //reset skeleton to T-pose(rest pose), so that we can apply skeleton anim(local quaternion) to animate skeleton 
	Ogre::Bone* root = skeleton->getBone(rootname);
	int bnum = skeleton->getNumBones();
	Ogre::Quaternion rootParentRot = root->getParent()->_getDerivedOrientation();   //global orientation
	Ogre::Quaternion rootParentRotInv = rootParentRot.Inverse();
	std::vector<std::vector<Ogre::Quaternion>> boneRotsList(bnum);
	std::vector<float> wList;
	std::vector<Ogre::Vector3> rootPos;
	std::vector<Ogre::Vector3> rootdirects;
	initForward = root->_getDerivedOrientation().Inverse() * initForward;
	
	//for each bone, store animation inf from 4 example-animations：boneRotsList
	//each example animation has a weight pair (animation index, weight)
	for(int i=0; i<weights.size(); i++)
	{
		skeleton->reset(true);
		wList.push_back(weights[i].second);
		int idx = weights[i].first;
		double time = lps[idx].times[timeIdx];
		time += timeScale[i]*runtime;
		if(time > lps[idx].times[timeIdx+1])
			time = lps[idx].times[timeIdx+1];
		Ogre::Animation* anim = skeleton->getAnimation(lps[idx].aniname);
		Ogre::TimeIndex timeIndex = anim->_getTimeIndex(time);   //timeIndex = time
		Ogre::Animation::NodeTrackList trackList = anim->_getNodeTrackList();
		Ogre::Animation::NodeTrackList::iterator it;
		root->setPosition(root->getInitialPosition());
		//在时间点timeIndex， 把trackList在这个点的骨骼信息逐一设置到skeleton的bone
		for(it = trackList.begin(); it!=trackList.end(); it++)
		{
			Ogre::Bone* bone = skeleton->getBone(it->first);
			bone->setOrientation(bone->getInitialOrientation());
			it->second->applyToNode(bone, timeIndex);                   
		}
		for(int j=0; j<bnum; j++)
		{
			Ogre::Bone* bone = skeleton->getBone(j);
			Ogre::Quaternion q = bone->getOrientation();         //local orientation
			Ogre::Vector3 t = bone->getPosition();               //local position
			if(bone->getName()==rootname){
				Ogre::Quaternion qw = bone->_getDerivedOrientation();    //global orientation
				q = rootInitRot[i] * qw;
				rootdirects.push_back(q * initForward);
			//	q = rootParentRotInv * q;

				Ogre::Vector3 tw = bone->_getDerivedPosition();    //gloal position
			//	t = rootInitRot[i] * (tw -  rootInitTra[i]) + rootStateWorld.pos;
				t = rootOriRot * rootInitRot[i] * (tw - rootStateWorld.pos + rootInitTra[i]) + rootStateWorld.pos;
				t = rootParentRotInv * (t - root->getParent()->_getDerivedPosition());
				rootPos.push_back(t);
			}		
			boneRotsList[j].push_back(q);
		}
	}

	//混合4个skeleton的信息，重新设置到skeleton
	skeleton->reset(true);
	for(int i=0; i<bnum; i++)
	{
		Ogre::Quaternion qsyn = this->nlerpQuas(boneRotsList[i], wList);
		Ogre::Bone* bone = skeleton->getBone(i);
		if(bone->getName() == rootname)
			qsyn = rootParentRotInv * rootOriRot * qsyn;
		bone->setOrientation(qsyn);
	}
	Ogre::Vector3 psyn = this->nlerpVecs(rootPos, wList);
	root->setPosition(psyn);
//	skeleton->getRootBone()->_update(1,0);

	//float prop = runtime / cliptime;
	//if(prop > 1)
	//	prop = 1;
	//Ogre::Vector3 axis(0,1,0);
	//if(endAngleError.valueRadians() < 0){
	//	endAngleError = -endAngleError;
	//	axis.y = -1;
	//}
	//Ogre::Quaternion errorRot(prop*endAngleError, axis);
	//Ogre::Vector3 errorTran = prop * endPosError;
	//root->translate(errorTran, Ogre::Node::TS_WORLD);
	//root->rotate(errorRot, Ogre::Node::TS_WORLD);
}

void Locomotion::obtainNextRootState()
{
//	this->runtime = this->cliptime;
	std::vector<std::pair<int, float>>& weights = this->interWeights;
	std::vector<Connection::LocomotionParam>& lps = this->locomotionParams;
	Ogre::Skeleton* skeleton = ent->getSkeleton();
	Ogre::Vector3 initForward = Ogre::Vector3(0,0,1);
	Ogre::Bone* root = skeleton->getBone(rootname);
	Ogre::Quaternion rootParentRot = root->getParent()->_getDerivedOrientation();
	Ogre::Quaternion rootParentRotInv = rootParentRot.Inverse();
	Ogre::Vector3 oriForward = (rootParentRot * root->getInitialOrientation()).Inverse() * initForward;
	std::vector<Ogre::Quaternion> rootRot;
	std::vector<Ogre::Vector3> rootPos;
	std::vector<float> wlist;
	for(int i=0; i<weights.size(); i++)
	{
	//	skeleton->reset(true);
		this->resetRoot();
		int idx = weights[i].first;
		double time = lps[idx].times[timeIdx];
		time += timeScale[i]*runtime;
		if(time > lps[idx].times[timeIdx+1])
			time = lps[idx].times[timeIdx+1];
		Ogre::Animation* anim = skeleton->getAnimation(lps[idx].aniname);
		Ogre::TimeIndex timeIndex = anim->_getTimeIndex(time);
		Ogre::Animation::NodeTrackList trackList = anim->_getNodeTrackList();
		
		int rootIdx = root->getHandle();
		Ogre::Animation::NodeTrackList::iterator it = trackList.find(rootIdx);
		it->second->applyToNode(root, timeIndex);

		Ogre::Quaternion qw = root->_getDerivedOrientation();
		Ogre::Quaternion q = rootInitRot[i] * qw;
		Ogre::Vector3 tw = root->_getDerivedPosition();
		tw = rootOriRot * rootInitRot[i] * (tw - rootStateWorld.pos + rootInitTra[i]) + rootStateWorld.pos;
		Ogre::Vector3 t = rootParentRotInv * (tw - root->getParent()->_getDerivedPosition());

		rootRot.push_back(q);
		rootPos.push_back(t);
		wlist.push_back(weights[i].second);

	}
	this->resetRoot();
	//Ogre::Quaternion rootParentRot = root->getParent()->_getDerivedOrientation();
	//Ogre::Quaternion rootParentRotInv = rootParentRot.Inverse();

	Ogre::Quaternion rrot = this->nlerpQuas(rootRot, wlist);
	rrot = rootParentRotInv * rootOriRot * rrot;
	Ogre::Vector3 rtra = this->nlerpVecs(rootPos, wlist);
	Ogre::Quaternion wrrot = rootParentRot * rrot;
	rrot = wrrot * root->_getDerivedOrientation().Inverse();

	rootStateWorld.dir = rrot * initForward;
	rootStateWorld.dir.y = 0;
	rootStateWorld.pos = rootParentRot * rtra + root->getParent()->_getDerivedPosition();
	rootStateWorld.pos.y = 0;

	this->runtime = 0;
	timeIdx++;
	if(timeIdx >= timeIdxMax)
		timeIdx = 0;
}

void Locomotion::obtainInitChange(const std::vector<std::pair<int, float>> &weights, const std::vector<Connection::LocomotionParam> &lps)
{
	Ogre::Skeleton* skeleton = ent->getSkeleton();
	Ogre::Vector3 initForward = Ogre::Vector3(0,0,1);
	rootInitRot.clear();
	rootInitTra.clear();
	Ogre::Bone* root = skeleton->getBone(rootname);
	int rootIdx = root->getHandle();
	Ogre::Quaternion rootParentRot = root->getParent()->_getDerivedOrientation();
	Ogre::Quaternion rootParentRotInv = rootParentRot.Inverse();
	Ogre::Vector3 rootPosInit = root->_getDerivedPosition();
	Ogre::Vector3 oriForward = (rootParentRot * root->getInitialOrientation()).Inverse() * initForward;
	for(int i=0; i<weights.size(); i++)
	{
		this->resetRoot();
		int idx = weights[i].first;
		double time = lps[idx].times[timeIdx];
		time += timeScale[i]*runtime;
		Ogre::Animation* anim = skeleton->getAnimation(lps[idx].aniname);
		Ogre::TimeIndex timeIndex = anim->_getTimeIndex(time);
		Ogre::Animation::NodeTrackList trackList = anim->_getNodeTrackList();		
		Ogre::Animation::NodeTrackList::iterator it = trackList.find(rootIdx);
		it->second->applyToNode(root, timeIndex);

		Ogre::Quaternion delta = root->_getDerivedOrientation() * (rootParentRot * root->getInitialOrientation()).Inverse();
		Ogre::Vector3 forward = delta * initForward; //should use oriForward instead; cal root orientation after apply animation
		Ogre::Vector3 forwardy = forward;
		forward.y = 0;
		Ogre::Quaternion q = forward.getRotationTo(initForward);
	//	q = rootParentRotInv * q;
		Ogre::Vector3 p = rootStateWorld.pos - root->_getDerivedPosition();
		p.y = 0;
	//	p = rootParentRotInv * p;
		Ogre::Quaternion rinv = forward.getRotationTo(forwardy);
		rinv = rinv.Inverse() * delta;
		if(rinv.w < 0)
			q = -q;
		rootOriRot = initForward.getRotationTo(rootStateWorld.dir);
		Ogre::Vector3 verticle = initForward.crossProduct(rootStateWorld.dir);
		if(verticle.y < 0)
			rootOriRot = -rootOriRot;
		//q = rootOriRot * q;
		rootInitRot.push_back(q);
		rootInitTra.push_back(p);
	}

}

void Locomotion::saveLastMotionState()
{
	Ogre::Skeleton *ske = this->ent->getSkeleton();
	diffs.clear();
	for(int i=0; i<ske->getNumBones(); i++)
		diffs.push_back(ske->getBone(i)->getOrientation());
	difft = ske->getBone(rootname)->_getDerivedPosition();
}

void Locomotion::transitionClips(float propMax)
{
	float tran = propMax * cliptime;
	if(diffs.size() == 0)
		return;
	if(this->runtime > tran)
		return;
	Ogre::Skeleton *ske = this->ent->getSkeleton();
	Ogre::Bone* root = ske->getBone(rootname);
	if(this->runtime == 0)
	{
		Ogre::Vector3 rtstart = root->_getDerivedPosition();
		difft = difft - rtstart;
		difft = root->getParent()->_getDerivedOrientation().Inverse() * difft;
		for(int i=0; i<ske->getNumBones(); i++)
			diffs[i] = diffs[i]*ske->getBone(i)->getOrientation().Inverse();
		int ridx = root->getHandle();
		if(diffs[ridx].w < 0)
			diffs[ridx] = -diffs[ridx];
	}

	Ogre::Quaternion unit = Ogre::Quaternion(1,0,0,0);
	float prop = this->runtime/tran;
	for(int i=0; i<ske->getNumBones(); i++){
		Ogre::Quaternion q = Ogre::Quaternion::Slerp(prop, diffs[i], unit);
		ske->getBone(i)->setOrientation(q*ske->getBone(i)->getOrientation());
	}
	root->setPosition(difft*(1-prop) + root->getPosition());
}

void Locomotion::findNearestParam(const std::vector<float>& mp)
{
	assert(mp.size()==3);
	if(interWeights.size()!=0)
		return;
	LocomotionParam& lp = locomotionParams[0];
	int idx = 0;
	float minDis = (lp.velf-mp[0])*(lp.velf-mp[0]) + (lp.vels-mp[1])*(lp.vels-mp[1]) + (lp.velt-mp[2])*(lp.velt-mp[2]);
	for(int i=1; i<locomotionParams.size(); i++)
	{
		LocomotionParam& lpi = locomotionParams[i];
		float dis = (lp.velf-mp[0])*(lp.velf-mp[0]) + (lp.vels-mp[1])*(lp.vels-mp[1]) + (lp.velt-mp[2])*(lp.velt-mp[2]);
		if(dis < minDis)
			idx = i;
	}
	interWeights.resize(1);
	interWeights[0].first = idx;
	interWeights[0].second = 1;
}


Ogre::Quaternion Locomotion::nlerpQuas(const std::vector<Ogre::Quaternion>& rots, const std::vector<float>& weights)
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

Ogre::Vector3 Locomotion::nlerpVecs(const std::vector<Ogre::Vector3>& tras, const std::vector<float>& weights)
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

void Locomotion::resetRoot()
{
	Ogre::Bone* root = this->ent->getSkeleton()->getBone(rootname);
	root->setPosition(root->getInitialPosition());
	root->setOrientation(root->getInitialOrientation());
}

void Locomotion::setRootState(Ogre::Vector3 pos, Ogre::Vector3 dir)
{
	rootStateWorld.dir = dir; 
	rootStateWorld.pos = pos;
	Ogre::Bone* root = this->ent->getSkeleton()->getBone(rootname);
	Ogre::Quaternion parentRotInv = root->getParent()->_getDerivedOrientation().Inverse();
}

void Locomotion::addRunTime(double dt)
{
	this->runtime += dt;
}

}