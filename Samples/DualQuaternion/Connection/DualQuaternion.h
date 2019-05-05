#ifndef __DualQuaternion_Sample_H__
#define __DualQuaternion_Sample_H__

#include "SdkSample.h"

#if defined(INCLUDE_RTSHADER_SYSTEM) && defined(RTSHADER_SYSTEM_BUILD_EXT_SHADERS)
#include "OgreShaderExHardwareSkinning.h"
#endif

#include "../Connection/locomotion.h"
#include "../Connection/reach.h"
#include "../Connection/retargetik/retargetik.h"
#include "../Connection/grasp.h"

#define TRAINNUM 2

using namespace OgreBites;
namespace Ogre
{

class _OgreSampleClassExport Sample_DualQuaternion : public SdkSample
{
public:
	Sample_DualQuaternion() : ent(0), entDQ(0), totalTime(0)
#if defined(INCLUDE_RTSHADER_SYSTEM) && defined(RTSHADER_SYSTEM_BUILD_EXT_SHADERS)
		, mSrsHardwareSkinning(0)
#endif
	{
		mInfo["Title"] = "Dual Quaternion Skinning";
		mInfo["Description"] = "A demo of the dual quaternion skinning feature in conjunction with the linear skinning feature.";
		mInfo["Thumbnail"] = "thumb_dualquaternionskinning.png";
		mInfo["Category"] = "Animation";

	}

	bool frameRenderingQueued(const FrameEvent& evt);


protected:
	StringVector getRequiredPlugins()
	{
		StringVector names;
        if (!GpuProgramManager::getSingleton().isSyntaxSupported("glsl"))
            names.push_back("Cg Program Manager");
		return names;
	}

	void setupContent()
	{
		setupControls();

#if defined(INCLUDE_RTSHADER_SYSTEM) && defined(RTSHADER_SYSTEM_BUILD_EXT_SHADERS)
        //Add the hardware skinning to the shader generator default render state
        mSrsHardwareSkinning = mShaderGenerator->createSubRenderState(Ogre::RTShader::HardwareSkinning::Type);
        Ogre::RTShader::RenderState* renderState = mShaderGenerator->getRenderState(Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
        renderState->addTemplateSubRenderState(mSrsHardwareSkinning);

        Ogre::MaterialPtr pCast1 = Ogre::MaterialManager::getSingleton().getByName("Ogre/RTShader/shadow_caster_dq_skinning_1weight_twophase");
        Ogre::MaterialPtr pCast2 = Ogre::MaterialManager::getSingleton().getByName("Ogre/RTShader/shadow_caster_dq_skinning_2weight_twophase");
        Ogre::MaterialPtr pCast3 = Ogre::MaterialManager::getSingleton().getByName("Ogre/RTShader/shadow_caster_dq_skinning_3weight_twophase");
        Ogre::MaterialPtr pCast4 = Ogre::MaterialManager::getSingleton().getByName("Ogre/RTShader/shadow_caster_dq_skinning_4weight_twophase");

        Ogre::RTShader::HardwareSkinningFactory::getSingleton().setCustomShadowCasterMaterials(RTShader::ST_DUAL_QUATERNION, pCast1, pCast2, pCast3, pCast4);

        Ogre::MaterialPtr pCast1l = Ogre::MaterialManager::getSingleton().getByName("Ogre/RTShader/shadow_caster_skinning_1weight");
        Ogre::MaterialPtr pCast2l = Ogre::MaterialManager::getSingleton().getByName("Ogre/RTShader/shadow_caster_skinning_2weight");
        Ogre::MaterialPtr pCast3l = Ogre::MaterialManager::getSingleton().getByName("Ogre/RTShader/shadow_caster_skinning_3weight");
        Ogre::MaterialPtr pCast4l = Ogre::MaterialManager::getSingleton().getByName("Ogre/RTShader/shadow_caster_skinning_4weight");

        Ogre::RTShader::HardwareSkinningFactory::getSingleton().setCustomShadowCasterMaterials(RTShader::ST_LINEAR, pCast1l, pCast2l, pCast3l, pCast4l);
#endif

//#define SHADOW
#if defined(SHADOW)
		// set shadow properties
		mSceneMgr->setShadowTechnique(SHADOWTYPE_TEXTURE_MODULATIVE);
		mSceneMgr->setShadowTextureSize(2048);
		mSceneMgr->setShadowColour(ColourValue(0.6, 0.6, 0.6));
		mSceneMgr->setShadowTextureCount(1);
#endif

		// add a little ambient lighting
		mSceneMgr->setAmbientLight(ColourValue(0.2, 0.2, 0.2));

		SceneNode* lightsBbsNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		BillboardSet* bbs;

		// Create billboard set for lights .
		bbs = mSceneMgr->createBillboardSet();
		bbs->setMaterialName("Examples/Flare");
		lightsBbsNode->attachObject(bbs);

		std::vector<Ogre::Vector3> lightPos(4);
		lightPos[0] = Ogre::Vector3(-500, 1000, -500);
		lightPos[1] = Ogre::Vector3(-500, 1000, 500);
		lightPos[2] = Ogre::Vector3(500, 1000, -500);
		lightPos[3] = Ogre::Vector3(500, 1000, 500);
		for(int i=0; i<lightPos.size(); i++){
			Light* l = mSceneMgr->createLight();
			Vector3 dir;
 			l->setType(Light::LT_POINT);
 			l->setPosition(lightPos[i]);
 			dir = -l->getPosition();
 			dir.normalise();
 			l->setDirection(dir);
 			l->setDiffuseColour(0.3, 0.3, 0.3);
 			bbs->createBillboard(l->getPosition())->setColour(l->getDiffuseColour());
		}

 		/*Light* l = mSceneMgr->createLight();
		Vector3 dir;
 		l->setType(Light::LT_POINT);
 		l->setPosition(100, 190, 100);
 		dir = -l->getPosition();
 		dir.normalise();
 		l->setDirection(dir);
 		l->setDiffuseColour(1, 1, 1);
 		bbs->createBillboard(l->getPosition())->setColour(l->getDiffuseColour());*/

		// create a floor mesh resource
		MeshManager::getSingleton().createPlane("floor", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
			Plane(Vector3::UNIT_Y, -1), 2000, 2000, 25, 25, true, 1, 15, 15, Vector3::UNIT_Z);

		// add a floor to our scene using the floor mesh we created
		Entity* floor = mSceneMgr->createEntity("Floor", "floor");
		floor->setMaterialName("Examples/Rockwall");
		floor->setCastShadows(false);
		mSceneMgr->getRootSceneNode()->attachObject(floor);

		// set camera initial transform and speed
		mCamera->setPosition(130, 150, 450);
		//mCamera->lookAt(0, 0, 0);
		mCamera->lookAt(0,150,-60);
		
		//mCameraMan->setTopSpeed(200);
		mCameraMan->setStyle(CS_ORBIT);

		SceneNode* orbitTargetNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("orbit_target_node");
		orbitTargetNode->translate(0, 120, 0);
		mCameraMan->setTarget(orbitTargetNode);
		//setupModels();

		

		//sphere
		Entity* sphere = mSceneMgr->createEntity("Sphere", "sphere.mesh");
		sphere->setMaterialName("Examples/TransparentTest");
		sphere->setCastShadows(true);
		SceneNode* sphereNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("sphere_node");
		sphereNode->translate(0, 10, 0);
		sphereNode->scale(0.1, 0.1, 0.1);
		sphereNode->attachObject(sphere);


		//cube
		Entity* cube = mSceneMgr->createEntity("Cube", "cube.mesh");
		cube->setMaterialName("Examples/TransparentTest");
		cube->setCastShadows(true);
		SceneNode* cubeNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("cube_node");
		cubeNode->translate(0, 10, 0);
		cubeNode->scale(0.1, 0.1, 0.1);
		cubeNode->attachObject(cube);


//		target_pos = Vector3(-6.5, 140.0, 62.0);
		target_pos = Vector3(-49, 96.0, 10.6);

		initSkeletonTransfer();
	}

	void setupModels()
	{
		SceneNode* sn = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		sn->translate(0, 0, 20, Node::TS_LOCAL);

		//Create and attach a spine entity with standard skinning
		ent = mSceneMgr->createEntity("jaiqua", "jaiqua.mesh");
		ent->setMaterialName("spine");
		ent->getSkeleton()->getBone("Rhand")->setManuallyControlled(true);
		sn->attachObject(ent);
		sn->scale(Vector3(0.2,0.2,0.2));

		sn = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		sn->translate(0, 0, -20, Node::TS_LOCAL);

		//Create and attach a spine entity with dual quaternion skinning
		entDQ = mSceneMgr->createEntity("jaiquaDQ", "jaiqua.mesh");
		entDQ->setMaterialName("spineDualQuat");
		entDQ->getSkeleton()->getBone("Rhand")->setManuallyControlled(true);
		sn->attachObject(entDQ);
		sn->scale(Vector3(0.2,0.2,0.2));
		
#if defined(INCLUDE_RTSHADER_SYSTEM) && defined(RTSHADER_SYSTEM_BUILD_EXT_SHADERS)
        //In case the system uses the RTSS, the following line will ensure
        //that the entity is using hardware animation in RTSS as well.
        RTShader::HardwareSkinningFactory::getSingleton().prepareEntityForSkinning(ent);
        RTShader::HardwareSkinningFactory::getSingleton().prepareEntityForSkinning(entDQ, RTShader::ST_DUAL_QUATERNION, false, true);

        //The following line is needed only because the spine models' materials have shaders and
        //as such is not automatically reflected in the RTSS system		
        RTShader::ShaderGenerator::getSingleton().createShaderBasedTechnique(
            ent->getSubEntity(0)->getMaterialName(),
            Ogre::MaterialManager::DEFAULT_SCHEME_NAME,
            Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME,
            true);
        
        RTShader::ShaderGenerator::getSingleton().createShaderBasedTechnique(
            entDQ->getSubEntity(0)->getMaterialName(),
            Ogre::MaterialManager::DEFAULT_SCHEME_NAME,
            Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME,
            true);
#endif

		// create name and value for skinning mode
		StringVector names;
		names.push_back("Skinning");
		String value = "Software";

		// change the value if hardware skinning is enabled
		MaterialPtr dqMat = ent->getSubEntity(0)->getMaterial();
		if(!dqMat.isNull())
		{
			Technique* bestTechnique = dqMat->getBestTechnique();
			if(bestTechnique)
			{
				Pass* pass = bestTechnique->getPass(0);
				if (pass && pass->hasVertexProgram() && pass->getVertexProgram()->isSkeletalAnimationIncluded())
				{
					value = "Hardware";
				}
			}
		}

		// create a params panel to display the skinning mode
		mTrayMgr->createParamsPanel(TL_TOPLEFT, "Skinning", 170, names)->setParamValue(0, value);
	}

	void cleanupContent()
	{
		MeshManager::getSingleton().remove("floor");

#if defined(INCLUDE_RTSHADER_SYSTEM) && defined(RTSHADER_SYSTEM_BUILD_EXT_SHADERS)
        Ogre::RTShader::RenderState* renderState = mShaderGenerator->getRenderState(Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
        renderState->removeTemplateSubRenderState(mSrsHardwareSkinning);
#endif
	}

	Entity* ent;
	Entity* entDQ;

	Real totalTime;

#if defined(INCLUDE_RTSHADER_SYSTEM) && defined(RTSHADER_SYSTEM_BUILD_EXT_SHADERS)
	RTShader::SubRenderState* mSrsHardwareSkinning;
#endif



	void setupControls()
	{
		const float sliderWidth = 250;
		mTrayMgr->showCursor();

		mTrayMgr->createLabel(TL_TOPLEFT, "lblTarget", "Target Position", sliderWidth);
		mTrayMgr->createThickSlider(TL_TOPLEFT, "XTargetPositionSlider", "X", sliderWidth, 50, -50.0, 50.0, 100)->setValue(-49, true);
		mTrayMgr->createThickSlider(TL_TOPLEFT, "YTargetPositionSlider", "Y", sliderWidth, 50,	 0.0,200.0, 100)->setValue(96.0, true);
		mTrayMgr->createThickSlider(TL_TOPLEFT, "ZTargetPositionSlider", "Z", sliderWidth, 50, -50.0, 100.0, 100)->setValue(10.6, true);

		//mTrayMgr->showCursor();

		//// make room for the controls
		//mTrayMgr->showLogo(TL_TOPRIGHT);
		//mTrayMgr->showFrameStats(TL_TOPRIGHT);
		//mTrayMgr->toggleAdvancedFrameStats();

		//// create checkboxes to toggle lights
		//mTrayMgr->createCheckBox(TL_TOPLEFT, "Light1", "Light A")->setChecked(true, false);
		//mTrayMgr->createCheckBox(TL_TOPLEFT, "Light2", "Light B")->setChecked(true, false);
		//mTrayMgr->createCheckBox(TL_TOPLEFT, "MoveLights", "Move Lights")->setChecked(true, false);

		//// a friendly reminder
		//StringVector names;
		//names.push_back("Help");
		//mTrayMgr->createParamsPanel(TL_TOPLEFT, "Help", 100, names)->setParamValue(0, "H/F1");
	}

	void sliderMoved(Slider* slider)
	{
		float slider_value = (float)slider->getValue();
		if (slider->getName() == "XTargetPositionSlider")
		{
			target_pos.x = slider_value;// Vector3(0.0, 150.0, 50.0);
		}
		else if (slider->getName() == "YTargetPositionSlider")
		{
			target_pos.y = slider_value;// Vector3(0.0, 150.0, 50.0);
		}
		else if (slider->getName() == "ZTargetPositionSlider")
		{
			target_pos.z = slider_value;// Vector3(0.0, 150.0, 50.0);
		}
	}

	Ogre::MeshPtr mMesh;
	Ogre::SkeletonPtr mSkeleton;
	void loadMeshData(Ogre::String);

	Ogre::Entity* srcEntity; 
	Ogre::AnimationState* srcAs;

	std::vector<Ogre::String> skeNames;
	std::vector<Ogre::String> aniNames;
	Ogre::String ROOTNAME;
	float ANIM_CHOP;
	
	float props[2];

	std::vector<Ogre::String> boneNameList;

	void initSkeletonTransfer();

	void offline_play(double dt);


	void getSkeletonNames();
	void getAnimationNames(Ogre::Entity* ent);
	
	Ogre::SceneNode* sn;

	Connection::Locomotion locomotion;

	int step;
	double runtime;
	void pathPlanning(std::vector<float>& vps, double upper = 21);

	//Ling
	void visualize_pseudo_sample_points();
	void offline_play_reach(double dt);
	Connection::Reach reach;
	void offline_play_reach_get_parameter(double dt);
	Vector3 target_pos;
	void offline_play_grasp(double dt);

	Connection::Grasp grasp;
	//int controller(float time);
	void update(float dt);
};

}
#endif
