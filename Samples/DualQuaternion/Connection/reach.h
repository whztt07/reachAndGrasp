#ifndef    _REACH_SAMPLE_H
#define    _REACH_SAMPLE_H

#include "Ogre.h"
#include <math.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "../Connection/tetgen/tetgen.h"
#include "../Connection/barycentric/barycentric.h"

#include "locomotion.h"
//#include "opencv2/opencv.hpp"
#include "opencv2/flann/miniflann.hpp"
#include "OgreAxisAlignedBox.h"
#include "retargetik\retargetIK.h"

namespace Connection
{

//struct RootState
//{
//	Ogre::Vector3 dir;
//	Ogre::Vector3 pos;
//};

class ReachParam
{
public:
	float x;
	float y;;
	float z;
	std::vector<float> times;            //to-t1-t2
	Ogre::String aniname;
};

struct pseudoSamplePoint
{
	cv::Point3f Pos;
	std::vector<std::pair<int, float>> index_weight; //index and weight for original sample motion
};

class Reach//Locomotion
{
public:
	void initReach(Ogre::String filename, Ogre::Entity* ent, Ogre::SceneManager* mSceneMgr);
	void addTimeIndex() {timeIdx++;}
	void addRunTime(double dt);
	int getTimeIndex() const {return timeIdx;}
	void generateLocomotion(const std::vector<float>& mp, double addtime);
	void setRootState(Ogre::Vector3 dir, Ogre::Vector3 pos); 
	Ogre::Vector3 getRootStatePos() const {return rootStateWorld.pos;}
	Ogre::Vector3 getRootStateDir() const {return rootStateWorld.dir;}
	void motionSynthesis(const int approach_mix_type = 0, const int return_mix_type = 1);//animationstate is not enable
	void resetReach();
	void updateTimeStep();
	void predictNewExamples(const std::vector<float>& mp);
	float getRuntime() const {return runtime;}
	float getCliptime() const {return cliptime;}
	void setClipTime(float time) {cliptime = time;}
	
	//////////
	///Ling///
	//////////
	Ogre::Vector3 p_r_t_i;
	Ogre::Vector3 d_p_i;
	float time_i;
	float time_iminus1;
	void KNN_test();
	cv::flann::Index* KNN_build( std::vector<cv::Point3f> &input_3Dpoints, int KDTreeIndexParams = 3);
	std::vector<std::pair<int, float>> Reach::KNN_search(cv::flann::Index* kd_tree,
											    std::vector<cv::Point3f> &input_3Dpoints,
											    const cv::Point3f &query_pt,
											    int numb_nearest_neighb);

	//generate random points inside the bounding box of a known point set
	std::vector<pseudoSamplePoint> pseudo_sample_points_generation(std::vector<cv::Point3f> &input_3Dpoints,
															  int number_pseudo_points
															  );

	std::vector<pseudoSamplePoint> pseudo_sample_points;
	std::vector <cv::Point3f> pseudo_sample_points_pos;

	cv::flann::Index* kd_tree_pseudo_sample_points_pos;
	std::vector<std::pair<int, float>> KNN_pseudo_sample_point(cv::flann::Index* kd_tree, std::vector<cv::Point3f> &input_3Dpoints, const cv::Point3f &target_point, const int k=4);
	void pure_play(float time, int ani_numb);
	void preProcess(std::vector<Ogre::Quaternion> &root_rotation_list, std::vector<Ogre::Vector3> &root_translation_list);
	std::vector<Ogre::Quaternion> root_rotation_list;
	std::vector<Ogre::Vector3> root_translation_list;
	void generateRandomWeight(int nK, std::vector<float>& outWeights);
	void getParameter(std::vector<std::pair<int, float>> &index_weights, cv::Point3f &end_effector_position);
	void generateDistWeights(std::vector<float>& dists, std::vector<float>& outWeights);
	std::vector<cv::Point3f> reachParams_pos;
	Ogre::Bone* root;
	Ogre::Vector3 root_world_pos_;
	float cal_alpha(float time_i, float time_iminus1, float t_a, const int type);

	//IK
	EffectorTarget effector_target;
	RetargetParams IK_params;
	Retarget IK_solver;
	void cal_p_bar_t_a(Ogre::Vector3 &p_bar_t_a);
	Ogre::Vector3 p_bar_t_a;

private:
	void findNearestParam(const std::vector<float>& mp);
	void readReachFile(Ogre::String filename);
	void getAnimationNames(std::map<Ogre::String, int> &mapSkIdx);
	ReachParam initReachParam(float vf, float vs, float vt);
	void obtainClipTime(const std::vector<std::pair<int, float>> &weights, const std::vector<Connection::ReachParam> &lps);
	void obtainInitChange(const std::vector<std::pair<int, float>> &weights, const std::vector<Connection::ReachParam> &lps);
	Ogre::Quaternion nlerpQuas(const std::vector<Ogre::Quaternion>& rots, const std::vector<float>& weights);
	Ogre::Vector3 nlerpVecs(const std::vector<Ogre::Vector3>& tras, const std::vector<float>& weights);
	void resetRoot();
	//void getVelocityParameters();

	float lasttime; //time of last blending finishing
	float cliptime; //clip motion time of current blending, decided by 4 chosen example motion
	float runtime; //running time
	int timeIdx; //period index in example's cycle
	int timeIdxMax;
	//float blending[4];
	std::vector<float> timeScale;
	Ogre::Entity* ent;
	Ogre::Skeleton* skeleton;
	Ogre::String rootname;
	BarycentricInterpolator baryInterp;
	Tetrahe tetraheIn;
	RootState rootStateWorld;
	std::vector<std::pair<int, float>> interWeights;
	std::vector<Tetrahe> tetrahes;
	Ogre::Quaternion rootOriRot;
	std::vector<Ogre::Quaternion> rootInitRot;
	std::vector<Ogre::Vector3> rootInitTra;
	std::vector<Connection::ReachParam> reachParams;
	std::vector<float> inputParam;
	std::vector<Ogre::Quaternion> diffs;
	Ogre::Vector3 difft;

};


}
#endif