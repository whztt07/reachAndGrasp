
#ifndef __RETARGET_SAMPLE_H__
#define __RETARGET_SAMPLE_H__

#include "Ogre.h"
typedef Ogre::String BoneName;
typedef Ogre::Vector3 BonePos;
typedef std::vector<BoneName> BoneNameList;
typedef std::map<BoneName,BoneNameList> BoneNameListMap;
typedef std::vector<BonePos> BonePosList;
typedef std::vector<Ogre::Quaternion> BoneQuaList;
//#define USE_SKETION_TRANSFER
#define USE_IKMETHORDS

#ifdef USE_IKMETHORDS

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace Connection
{

struct EffectorTarget
{
	Ogre::String effectorName;
	Ogre::Vector3 targetPosition;
	Ogre::Quaternion targetOrientation;
	bool posRestraint;
	bool oriRestraint;
};

struct RetargetParams
{
	Ogre::Entity* dEntity;
	std::vector<EffectorTarget> Targets;
	int iterNum;
	Ogre::String rootName; 
	bool rootProcess; // retarget root bone?
	double scale; // scale proportion of body changing
	bool relative; // use the result of last frame?

};

class Retarget
{
	Ogre::Entity* _entity;
	Ogre::Skeleton* _skeleton;

public:
	void retargetIKDirect(RetargetParams Params);
	void retargetIKJacobian(RetargetParams Params, const std::vector<Ogre::Quaternion> &referQuatList);
};

}
#endif

#endif