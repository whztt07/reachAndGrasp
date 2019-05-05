#ifndef    _LOCOMOTION_SAMPLE_H
#define    _LOCOMOTION_SAMPLE_H

#include "Ogre.h"
#include <math.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "../Connection/tetgen/tetgen.h"
#include "../Connection/barycentric/barycentric.h"

namespace Connection
{

struct RootState
{
	Ogre::Vector3 dir;
	Ogre::Vector3 pos;
};

class LocomotionParam
{
public:
	float velf;
	float velt;
	float vels;
	std::vector<float> times;            //to-t1, t1-t2
	Ogre::String aniname;
};

class Locomotion
{
public:
	void initLocomotion(Ogre::String filename, Ogre::Entity* ent);
	void addTimeIndex() {timeIdx++;}
	void addRunTime(double dt);
	int getTimeIndex() const {return timeIdx;}
	void generateLocomotion(const std::vector<float>& mp, double addtime);
	void setRootState(Ogre::Vector3 dir, Ogre::Vector3 pos); 
	Ogre::Vector3 getRootStatePos() const {return rootStateWorld.pos;}
	Ogre::Vector3 getRootStateDir() const {return rootStateWorld.dir;}
	void motionSynthesis();//animationstate is not enable
	void resetLocomotion();
	void obtainNextRootState();
	void predictNewExamples(const std::vector<float>& mp);
	void saveLastMotionState();
	void transitionClips(float prop);
	float getRuntime() const {return runtime;}
	float getCliptime() const {return cliptime;}
	void setClipTime(float time) {cliptime = time;}
	
private:
	void findNearestParam(const std::vector<float>& mp);
	void readLocomotionFile(Ogre::String filename);
	void getAnimationNames(std::map<Ogre::String, int> &mapSkIdx);
	LocomotionParam initLocomotionParam(float vf, float vs, float vt);
	void obtainClipTime(const std::vector<std::pair<int, float>> &weights, const std::vector<Connection::LocomotionParam> &lps);
	void obtainInitChange(const std::vector<std::pair<int, float>> &weights, const std::vector<Connection::LocomotionParam> &lps);
	Ogre::Quaternion nlerpQuas(const std::vector<Ogre::Quaternion>& rots, const std::vector<float>& weights);
	Ogre::Vector3 nlerpVecs(const std::vector<Ogre::Vector3>& tras, const std::vector<float>& weights);
	void resetRoot();
	void getVelocityParameters();

	float lasttime; //time of last blending finishing
	float cliptime; //clip motion time of current blending, decided by 4 chosen example motion
	float runtime; //running time
	int timeIdx; //period index in example's cycle
	int timeIdxMax;
	//float blending[4];
	std::vector<float> timeScale;
	Ogre::Entity* ent;
	Ogre::String rootname;
	BarycentricInterpolator baryInterp;
	Tetrahe tetraheIn;
	RootState rootStateWorld;
	std::vector<std::pair<int, float>> interWeights;
	std::vector<Tetrahe> tetrahes;
	Ogre::Quaternion rootOriRot;
	std::vector<Ogre::Quaternion> rootInitRot;
	std::vector<Ogre::Vector3> rootInitTra;
	std::vector<Connection::LocomotionParam> locomotionParams;
	std::vector<float> inputParam;
	std::vector<Ogre::Quaternion> diffs;
	Ogre::Vector3 difft;

};


}
#endif