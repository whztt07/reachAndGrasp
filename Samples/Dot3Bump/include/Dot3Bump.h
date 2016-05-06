#ifndef __Dot3Bump_H__
#define __Dot3Bump_H__

#include "SdkSample.h"
#include "OgreBillboard.h"

using namespace Ogre;
using namespace OgreBites;

class _OgreSampleClassExport Sample_Dot3Bump : public SdkSample
{
public:
    
    Sample_Dot3Bump()
    : mMoveLights (false/*true*/)
    {
        mInfo["Title"] = "Bump Mapping";
        mInfo["Description"] = "Shows how to use the dot product blending operation and normalization cube map "
        "to achieve a bump mapping effect. Tangent space computations made through the guide of the tutorial "
        "on bump mapping from http://users.ox.ac.uk/~univ1234 by paul.baker@univ.ox.ac.uk.";
        mInfo["Thumbnail"] = "thumb_bump.png";
        mInfo["Category"] = "Lighting";
        mInfo["Help"] = "Left click and drag anywhere in the scene to look around. Let go again to show "
        "cursor and access widgets. Use WASD keys to move.";
        
        //Ling
        mMatPtr = MaterialManager::getSingleton().getByName("Examples/Skin_Preintegrate");
        assert(mMatPtr.isNull()==false);
        mMatPtr->load();
        
        mMatPtr_1 = MaterialManager::getSingleton().getByName("Examples/Skin_Old");
        assert(mMatPtr_1.isNull()==false);
        mMatPtr_1->load();
        
        mMatPtr_2 = MaterialManager::getSingleton().getByName("Examples/Skin_Preintegrate_Shenti");
        assert(mMatPtr_2.isNull()==false);
        mMatPtr_2->load();
        
        mMatPtr_3 = MaterialManager::getSingleton().getByName("Examples/Skin_Old_Shenti");
        assert(mMatPtr_3.isNull()==false);
        mMatPtr_3->load();
        
        mMatPtr_4 = MaterialManager::getSingleton().getByName("eye_rendering");
        assert(mMatPtr_4.isNull()==false);
        mMatPtr_4->load();

        mMatPtr_b_and_w = MaterialManager::getSingleton().getByName("Ogre/Compositor/BlackAndWhite_1_2");
        assert(mMatPtr_b_and_w.isNull()==false);
        mMatPtr_b_and_w->load();
        
        mMatPtr_tilling = MaterialManager::getSingleton().getByName("Ogre/Compositor/Tiling_1_20");
        assert(mMatPtr_tilling.isNull()==false);
        mMatPtr_tilling->load();

        mMatPtr_b_and_w_gradually = MaterialManager::getSingleton().getByName("Ogre/Compositor/BlackAndWhite_1_2_gradually");
        assert(mMatPtr_b_and_w_gradually.isNull()==false);
        mMatPtr_b_and_w_gradually->load();
        
        circle_time = 5.0;    //seconds
        up_bound = 1.0;
        down_bound = 0.0;

        intensity = up_bound;
        total_time = 0.0;
        decrease_flag = false;
        odd = 0.0;
        
        rendering_mode_number =1.0;
        //Ling, END
    }
    
    StringVector getRequiredPlugins()
    {
        StringVector names;
        if(!GpuProgramManager::getSingleton().isSyntaxSupported("glsles")
           && !GpuProgramManager::getSingleton().isSyntaxSupported("glsl")
           && !GpuProgramManager::getSingleton().isSyntaxSupported("hlsl"))
            names.push_back("Cg Program Manager");
        return names;
    }
    
    void testCapabilities(const RenderSystemCapabilities* caps)
    {
        if (!caps->hasCapability(RSC_VERTEX_PROGRAM) || !(caps->hasCapability(RSC_FRAGMENT_PROGRAM)))
        {
            OGRE_EXCEPT(Exception::ERR_NOT_IMPLEMENTED, "Your graphics card does not support vertex and fragment programs, "
                        "so you cannot run this sample. Sorry!", "Dot3BumpSample::testCapabilities");
        }
        
        if (!GpuProgramManager::getSingleton().isSyntaxSupported("arbfp1") &&
            !GpuProgramManager::getSingleton().isSyntaxSupported("ps_2_0") &&
            !GpuProgramManager::getSingleton().isSyntaxSupported("ps_4_0") &&
            !GpuProgramManager::getSingleton().isSyntaxSupported("glsl")   &&
            !GpuProgramManager::getSingleton().isSyntaxSupported("glsles"))
        {
            OGRE_EXCEPT(Exception::ERR_NOT_IMPLEMENTED, "Your card does not support the shader model needed for this sample, "
                        "so you cannot run this sample. Sorry!", "Dot3BumpSample::testCapabilities");
        }
        
        //        if ( GpuProgramManager::getSingleton().isSyntaxSupported("glsl") )
        //        {
        //            OGRE_EXCEPT(Exception::ERR_NOT_IMPLEMENTED, "OK, "
        //                        "GLSL", "Dot3BumpSample::testCapabilities");
        //        }
        
        //        if ( GpuProgramManager::getSingleton().isSyntaxSupported("glsles") )
        //        {
        //            OGRE_EXCEPT(Exception::ERR_NOT_IMPLEMENTED, "OK, "
        //                        "GLSLes", "Dot3BumpSample::testCapabilities");
        //        }
        
        //        if ( GpuProgramManager::getSingleton().isSyntaxSupported("glsl120") )
        //        {
        //            OGRE_EXCEPT(Exception::ERR_NOT_IMPLEMENTED, "OK, "
        //                        "GLSL120", "Dot3BumpSample::testCapabilities");
        //        }
        
    }
    
    bool frameRenderingQueued(const FrameEvent& evt)
    {
        
        
        float dt = float(evt.timeSinceLastFrame);
//        total_time += dt;
 
// comtrol 0
        if(decrease_flag)
            intensity = intensity - dt/circle_time;
        else
            intensity = intensity + dt/circle_time;
        
        if(intensity > up_bound) decrease_flag = true;
        if(intensity < down_bound) decrease_flag = false;

//////control 1
//        intensity = intensity - dt/circle_time;
//        if(intensity < down_bound)
//        {
//            intensity = up_bound;
//            
//            if(odd == 1.0)
//               odd = 0.0;
//            else if(odd == 0.0)
//               odd = 1.0;
//        }
        
////update material parameters in compositor
//        mMatPtr_b_and_w->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_intensity", intensity);
//        Ogre::CompositorManager::getSingleton()._reconstructAllCompositorResources();

//        mMatPtr_tilling->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("front_uv", Vector2(intensity, intensity));
//        mMatPtr_tilling->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_odd", odd);
//        Ogre::CompositorManager::getSingleton()._reconstructAllCompositorResources();

        mMatPtr_b_and_w_gradually->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_intensity", intensity);
        Ogre::CompositorManager::getSingleton()._reconstructAllCompositorResources();

////switch compositor
//        if(intensity > 1.0){
//        Ogre::CompositorManager::getSingleton().setCompositorEnabled(mCamera->getViewport(),"B&W_1_2",false);
//        Ogre::CompositorManager::getSingleton().setCompositorEnabled(mCamera->getViewport(),"Tiling_1_20",true);
//        }
//        else if (intensity < 0.0){
//            Ogre::CompositorManager::getSingleton().setCompositorEnabled(mCamera->getViewport(),"B&W_1_2",true);
//            Ogre::CompositorManager::getSingleton().setCompositorEnabled(mCamera->getViewport(),"Tiling_1_20",false);
//        }
        
        
        
        if (mMoveLights)
        {
            // rotate the light pivots
            mLightPivot1->roll(Degree(evt.timeSinceLastFrame * 30));
            mLightPivot2->roll(Degree(evt.timeSinceLastFrame * 10));
            mLightPivot3->roll(Degree(evt.timeSinceLastFrame * 10));
        }
        
        return SdkSample::frameRenderingQueued(evt);  // don't forget the parent class updates!
    }
    
    void itemSelected(SelectMenu* menu)
    {
        //        if (menu == mMeshMenu) //select mesh
        //        {
        //            // change to the selected entity
        //            mObjectNode->detachAllObjects();
        //            mObjectNode->attachObject(mSceneMgr->getEntity(mMeshMenu->getSelectedItem()));
        //
        //            // remember which material is currently selected
        //            int index = std::max<int>(0, mMaterialMenu->getSelectionIndex());
        //
        //            // update the material menu's options
        //            mMaterialMenu->setItems(mPossibilities[mMeshMenu->getSelectedItem()]);
        //
        //            mMaterialMenu->selectItem(index);   // select the material with the saved index
        //        }
        //        else //select material
        //        {
        //            // set the selected material for the active mesh
        //            ((Entity*)mObjectNode->getAttachedObject(0))->setMaterialName(menu->getSelectedItem());
        //        }
    }
    
    void checkBoxToggled(CheckBox* box)
    {
        if (StringUtil::startsWith(box->getName(), "Light", false))
        {
            // get the light pivot that corresponds to this checkbox
            SceneNode* pivot = box->getName() == "Light1" ? mLightPivot1 : mLightPivot2;
            SceneNode::ObjectIterator it = pivot->getAttachedObjectIterator();
            
            while (it.hasMoreElements())  // toggle visibility of light and billboard set
            {
                MovableObject* o = it.getNext();
                o->setVisible(box->isChecked());
            }
            
        }
        else if (box->getName() == "MoveLights")
        {
            mMoveLights = !mMoveLights;
        }
    }
    
protected:
    
    void setupContent()
    {
        // create our main node to attach our entities to
        mObjectNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
        mObjectNode_1 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
        mObjectNode_2 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
        
        setupModels();
        setupLights();
        setupControls();
        
        
        mCamera->setPosition(0, 70, 100);
        
        
        //        mCamera->rotate(Vector3(0, 1, 0), Degree(45));
        //        //mCamera->setPosition(0, 0, 500); //original
        //        mCamera->setPosition(35, 5, 35);
        //        //mCamera->roll(Degree(30));
        
        
        
#if OGRE_PLATFORM != OGRE_PLATFORM_APPLE_IOS
        setDragLook(true);
#endif
    }
    
    
    
    void loadResources()
    {
#ifdef INCLUDE_RTSHADER_SYSTEM
        Ogre::StringVector groupVector = Ogre::ResourceGroupManager::getSingleton().getResourceGroups();
        Ogre::StringVector::iterator itGroup = groupVector.begin();
        Ogre::StringVector::iterator itGroupEnd = groupVector.end();
        Ogre::String shaderCoreLibsPath;
        
        
        for (; itGroup != itGroupEnd; ++itGroup)
        {
            Ogre::ResourceGroupManager::LocationList resLocationsList = Ogre::ResourceGroupManager::getSingleton().getResourceLocationList(*itGroup);
            Ogre::ResourceGroupManager::LocationList::iterator it = resLocationsList.begin();
            Ogre::ResourceGroupManager::LocationList::iterator itEnd = resLocationsList.end();
            bool coreLibsFound = false;
            
            // Find the location of the core shader libs
            for (; it != itEnd; ++it)
            {
                if ((*it)->archive->getName().find("RTShaderLib") != Ogre::String::npos)
                {
                    shaderCoreLibsPath = (*it)->archive->getName() + "/";
                    coreLibsFound = true;
                    break;
                }
            }
            
            // Core libs path found in the current group.
            if (coreLibsFound)
                break;
        }
        
#endif
    }
    
    void unloadResources()
    {
        
    }
    
    void setupModels()
    {
        StringVector matNames;
        matNames.push_back("Examples/Skin_Preintegrate");
        matNames.push_back("Examples/Cook_Torrance_UE4");
        matNames.push_back("Examples/BumpMapping/MultiLight");
        matNames.push_back("Examples/BumpMapping/MultiLightSpecular");
        matNames.push_back("Examples/OffsetMapping/Specular");
        matNames.push_back("Examples/ShowUV");
        matNames.push_back("Examples/ShowNormals");
        matNames.push_back("Examples/ShowTangents");
        
#ifdef INCLUDE_RTSHADER_SYSTEM
        matNames.push_back("RTSS/NormalMapping_SinglePass");
        matNames.push_back("RTSS/NormalMapping_MultiPass");
#endif
        
        mPossibilities["sphere.mesh"] = matNames;
        mPossibilities["ogrehead.mesh"] = matNames;
        mPossibilities["knot.mesh"] = matNames;
        mPossibilities["uv_sphere.mesh"] = matNames;
        
        matNames.clear();
        matNames.push_back("Examples/Skin_Preintegrate");
        matNames.push_back("Examples/Cook_Torrance_UE4");
        matNames.push_back("Examples/Athene/NormalMapped");
        matNames.push_back("Examples/Athene/NormalMappedSpecular");
        matNames.push_back("Examples/Athene/NormalMappedSpecular");
        matNames.push_back("Examples/ShowUV");
        matNames.push_back("Examples/ShowNormals");
        matNames.push_back("Examples/ShowTangents");
#ifdef INCLUDE_RTSHADER_SYSTEM
        matNames.push_back("RTSS/Athene/NormalMapping_SinglePass");
        matNames.push_back("RTSS/Athene/NormalMapping_MultiPass");
#endif
        
        mPossibilities["athene.mesh"] = matNames;
        
        
        for (std::map<String, StringVector>::iterator it = mPossibilities.begin(); it != mPossibilities.end(); it++)
        {
            // load each mesh with non-default hardware buffer usage options
            MeshPtr mesh = MeshManager::getSingleton().load(it->first, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                            HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);
            
            // build tangent vectors for our mesh
            unsigned short src, dest;
            if (!mesh->suggestTangentVectorBuildParams(VES_TANGENT, src, dest))
            {
                mesh->buildTangentVectors(VES_TANGENT, src, dest);
                // this version cleans mirrored and rotated UVs but requires quality models
                // mesh->buildTangentVectors(VES_TANGENT, src, dest, true, true);
            }
            
            // create an entity from the mesh and set the first available material
            Entity* ent = mSceneMgr->createEntity(mesh->getName(), mesh->getName());
            ent->setMaterialName(it->second.front());
        }
        
        //Ling
        
        //        //Entity* ent = mSceneMgr->createEntity("sphere_0", "knot.mesh"); //uv_sphere.mesh
        //        Entity* ent = mSceneMgr->createEntity("sphere_0", "athene.mesh"); //uv_sphere.mesh
        //        mObjectNode->translate(-sliderWidth,0,0);
        //        mObjectNode->detachAllObjects();
        //        mObjectNode->attachObject(mSceneMgr->getEntity("sphere_0"));
        //        //((Entity*)mObjectNode->getAttachedObject(0))->setMaterialName("Examples/Cook_Torrance_Ground_Truth_UE4");
        //        ((Entity*)mObjectNode->getAttachedObject(0))->setMaterialName("Examples/Skin_Preintegrate");
        //
        //
        //        Entity* ent_1 = mSceneMgr->createEntity("sphere_1", "sphere.mesh");
        //        mObjectNode_1->translate(sliderWidth,0,0);
        //        mObjectNode_1->detachAllObjects();
        //        mObjectNode_1->attachObject(mSceneMgr->getEntity("sphere_1"));
        //        ((Entity*)mObjectNode_1->getAttachedObject(0))->setMaterialName("Examples/Cook_Torrance_UE4");
        
        
        //        //sphere
        //        Entity* ent = mSceneMgr->createEntity("sphere_0", "uv_sphere.mesh"); //uv_sphere.mesh human.mesh
        //        //SubEntity* ent0 = ent->getSubEntity(0);
        //        //ent0->setMaterialName("Examples/Skin_Preintegrate");
        //        //Entity* ent = mSceneMgr->createEntity("sphere_0", "uv_sphere.mesh"); //uv_sphere.mesh
        //        //Entity* ent = mSceneMgr->createEntity("sphere_0", "athene.mesh"); //uv_sphere.mesh
        //        mObjectNode->translate(0,0,-200);
        //        mObjectNode->detachAllObjects();
        //        mObjectNode->attachObject(mSceneMgr->getEntity("sphere_0"));
        //        ((Entity*)mObjectNode->getAttachedObject(0))->setMaterialName("Examples/Skin_Preintegrate");
        //
        
        
        //        //head ，mine
        //        Entity* ent = mSceneMgr->createEntity("head", "human.mesh"); //uv_sphere.mesh human.mesh head_l2.mesh head_l2_no_dup.mesh head_l2_dup.mesh
        //        int a =ent->getMesh()->getNumSubMeshes();
        //        //printf("%d\n", a);
        //        SubMesh* subMesh0 = ent->getMesh()->getSubMesh(0);
        //        a= subMesh0->vertexData->vertexCount;
        //        printf("%d\n", a);
        //        mObjectNode->detachAllObjects();
        //        mObjectNode->attachObject(mSceneMgr->getEntity("head"));
        //        ((Entity*)mObjectNode->getAttachedObject(0))->setMaterialName("Examples/Skin_Preintegrate");
        //        mObjectNode->translate(0,0,-200);
        //        //mObjectNode->rotate(Vector3(0.0,1.0,0.0), Degree(90.0));
        //        //mObjectNode->translate(-15,-170,20);
        //        mObjectNode->scale(30, 30, 30);
        
        
        
        //human
        Entity* ent = mSceneMgr->createEntity("human_0", "human.mesh"); //uv_sphere.mesh human.mesh
        SubEntity* subent0 = ent->getSubEntity(0);
        subent0->setMaterialName("Examples/Skin_Preintegrate");
        mObjectNode->attachObject(mSceneMgr->getEntity("human_0"));
        //mObjectNode->rotate(Vector3(0.0,1.0,0.0), Degree(90.0));
        
        //        mObjectNode->translate(-12,-100,20);
        mObjectNode->translate(-10,-100,0);
        
        //        mObjectNode->translate(0,-176,0);
        //        mObjectNode->scale(1.1,1.1,1.1);
        SubEntity* subent2 = ent->getSubEntity(1);
        subent2->setVisible(false);
        
        for(int i=2; i<24;i++){
            SubEntity* temp = ent->getSubEntity(i);
            temp->setMaterialName("Examples/Skin_Preintegrate_Shenti");
            //            printf("set sub entity material %d\n", i);
            //            printf("%s\n",  mMatPtr->getTechnique(0)->getPass(0)-> getTextureUnitState(5)->getTextureName().c_str());
            //            mMatPtr->getTechnique(0)->getPass(0)-> getTextureUnitState(5)->setTextureName("embed_user_upbody.png");
            //            printf("%s\n",  mMatPtr->getTechnique(0)->getPass(0)-> getTextureUnitState(5)->getTextureName().c_str());
        }
        
        mObjectNode->setVisible(false);
        
        
        //human old
        Entity* ent1 = mSceneMgr->createEntity("human_1", "human.mesh"); //uv_sphere.mesh human.mesh
        SubEntity* subent1 = ent1->getSubEntity(0);
        subent1->setMaterialName("Examples/Skin_Old");
        mObjectNode_1->attachObject(mSceneMgr->getEntity("human_1"));
        //mObjectNode_1->rotate(Vector3(0.0,1.0,0.0), Degree(90.0));
        
        //        mObjectNode_1->translate(12,-100,20);
        mObjectNode_1->translate(10,-100,0);
        
        //        mObjectNode_1->translate(0,-176,0);
        //        mObjectNode_1->scale(1.1,1.1,1.1);
        SubEntity* subent3 = ent1->getSubEntity(1);
        subent3->setVisible(false);
        
        
        
        for(int i=2; i<24;i++){
            SubEntity* temp = ent1->getSubEntity(i);
            temp->setMaterialName("Examples/Skin_Old_Shenti");
            //            printf("set sub entity material %d\n", i);
            //            printf("%s\n",  mMatPtr->getTechnique(0)->getPass(0)-> getTextureUnitState(5)->getTextureName().c_str());
            //            mMatPtr->getTechnique(0)->getPass(0)-> getTextureUnitState(5)->setTextureName("embed_user_upbody.png");
            //            printf("%s\n",  mMatPtr->getTechnique(0)->getPass(0)-> getTextureUnitState(5)->getTextureName().c_str());
        }
        
        mObjectNode_1->setVisible(false);

        
        
        //eye
        Entity* ent2 = mSceneMgr->createEntity("eye_0", "ogrehead.mesh"); //uv_sphere.mesh human.mesh
//        SubEntity* subent4 = ent2->getSubEntity(0);
//        subent4->setMaterialName("eye_rendering");
        mObjectNode_2->attachObject(mSceneMgr->getEntity("eye_0"));
        //mObjectNode->rotate(Vector3(0.0,1.0,0.0), Degree(90.0));
        
        
        mObjectNode_2->translate(0,80,0);
        mObjectNode_2->scale(1,1,1);

        //        mObjectNode->translate(0,-176,0);
        //        mObjectNode->scale(1.1,1.1,1.1);
//        SubEntity* subent2 = ent->getSubEntity(1);
//        subent2->setVisible(false);
        
//        for(int i=2; i<24;i++){
//            SubEntity* temp = ent->getSubEntity(i);
//            temp->setMaterialName("Examples/Skin_Preintegrate_Shenti");
//            //            printf("set sub entity material %d\n", i);
//            //            printf("%s\n",  mMatPtr->getTechnique(0)->getPass(0)-> getTextureUnitState(5)->getTextureName().c_str());
//            //            mMatPtr->getTechnique(0)->getPass(0)-> getTextureUnitState(5)->setTextureName("embed_user_upbody.png");
//            //            printf("%s\n",  mMatPtr->getTechnique(0)->getPass(0)-> getTextureUnitState(5)->getTextureName().c_str());
//        }

        
        
        //setup compositor
//        Ogre::CompositorManager::getSingleton().addCompositor(mCamera->getViewport(),"MyCompositor1");
//        Ogre::CompositorManager::getSingleton().setCompositorEnabled(mCamera->getViewport(),"Myompositor1",true);
        
        Ogre::CompositorManager::getSingleton().addCompositor(mCamera->getViewport(),"B&W_1_2");
        Ogre::CompositorManager::getSingleton().addCompositor(mCamera->getViewport(),"Tiling_1_20");
        Ogre::CompositorManager::getSingleton().addCompositor(mCamera->getViewport(),"B&W_1_2_gradually");

        Ogre::CompositorManager::getSingleton().setCompositorEnabled(mCamera->getViewport(),"B&W_1_2",false);
        Ogre::CompositorManager::getSingleton().setCompositorEnabled(mCamera->getViewport(),"Tiling_1_20",false);
        Ogre::CompositorManager::getSingleton().setCompositorEnabled(mCamera->getViewport(),"B&W_1_2_gradually",true);
        
        //Ling, END
    }
    
    void setupLights()
    {
        mSceneMgr->setAmbientLight(ColourValue::Black);   // disable ambient lighting
        
        // create pivot nodes
        mLightPivot1 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
        mLightPivot2 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
        mLightPivot3 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
        
        
        // create white light
        l = mSceneMgr->createLight();
        l->setPosition(10, 196, 20.0);   //ios is around 40
        l->setDiffuseColour(1, 1, 1);
        l->setSpecularColour(1, 1, 1);
        // create white flare
        bbs = mSceneMgr->createBillboardSet();
        bbs->setMaterialName("Examples/Flare");
        bbs->createBillboard(10, 196, 20.0)->setColour(ColourValue::White);
        bbs->getBillboard(0)->setDimensions(5, 5);
        
        mLightPivot1->attachObject(l);
        mLightPivot1->attachObject(bbs);
        
        
        // create white light
        l_1 = mSceneMgr->createLight();
        l_1->setPosition(-10, 196, 20.0 );   //ios is around 40
        l_1->setDiffuseColour(1, 1, 1);
        l_1->setSpecularColour(1, 1, 1);
        //        bbs = mSceneMgr->createBillboardSet();
        //        bbs->setMaterialName("Examples/Flare");
        //        bbs->createBillboard(-10, 196, 20.0)->setColour(ColourValue::White);
        
        mLightPivot2->attachObject(l_1);
        //        mLightPivot2->attachObject(bbs);
        
        
        //        // create white light
        //        l = mSceneMgr->createLight();
        //        l->setPosition(100, 0, 700);
        //        l->setDiffuseColour(1, 1, 1);
        //        l->setSpecularColour(1, 1, 1);
        //        // create white flare
        //        bbs = mSceneMgr->createBillboardSet();
        //        bbs->setMaterialName("Examples/Flare");
        //        bbs->createBillboard(0, 0, 70000)->setColour(ColourValue::White);
        //
        //        mLightPivot2->attachObject(l);
        //        mLightPivot2->attachObject(bbs);
        //
        //
        //        // create white light
        //        l = mSceneMgr->createLight();
        //        l->setPosition(-100, 0, 700);
        //        l->setDiffuseColour(1, 1, 1);
        //        l->setSpecularColour(1, 1, 1);
        //        // create white flare
        //        bbs = mSceneMgr->createBillboardSet();
        //        bbs->setMaterialName("Examples/Flare");
        //        bbs->createBillboard(4500, 0, 4500)->setColour(ColourValue::White);
        //
        //        mLightPivot3->attachObject(l);
        //        mLightPivot3->attachObject(bbs);
    }
    
    void setupControls()
    {
        const float sliderWidth = 250;
        mTrayMgr->showCursor();
        
        // make room for the controls
        mTrayMgr->showLogo(TL_TOPRIGHT);
        mTrayMgr->showFrameStats(TL_TOPRIGHT);
        mTrayMgr->toggleAdvancedFrameStats();
        
        //        // create a menu to choose the model displayed
        //        mMeshMenu = mTrayMgr->createLongSelectMenu(TL_BOTTOM, "Mesh", "Mesh", 370, 290, 10);
        //        for (std::map<String, StringVector>::iterator it = mPossibilities.begin(); it != mPossibilities.end(); it++)
        //            mMeshMenu->addItem(it->first);
        
        // create a menu to choose the material used by the model
        //        mMaterialMenu = mTrayMgr->createLongSelectMenu(TL_BOTTOM, "Material", "Material", 370, 290, 10);
        
        //        // create checkboxes to toggle lights
        //        mTrayMgr->createCheckBox(TL_TOPLEFT, "Light1", "Light A")->setChecked(false, false);  //origin ( true, false )
        //        //mTrayMgr->createCheckBox(TL_TOPLEFT, "Light2", "Light B")->setChecked(false, false);  //origin ( true, false )
        //        mTrayMgr->createCheckBox(TL_TOPLEFT, "MoveLights", "Move Lights")->setChecked(false, false);
        
        //Ling, slider, 2015.9.6
        //---------- TOP LEFT
//        mTrayMgr->createLabel(TL_TOPLEFT, "lblSpecular", "Specular Parameters: ", sliderWidth);
//        //mTrayMgr->createThickSlider(TL_TOPLEFT, "RoughnessSlider", "Roughness", sliderWidth, 50, 0.0, 20.0, 20)->setValue(0.0, false);
//        mTrayMgr->createThickSlider(TL_TOPLEFT, "SpecularPowerSlider", "0 Specular Power", sliderWidth, 50, 0.0, 200, 200)->setValue(7, true);
//        mTrayMgr->createThickSlider(TL_TOPLEFT, "SpecularLightIntensityColorSlider", "0 Specular Light Intensity", sliderWidth, 50, 0, 10.0/*3*/, 50)->setValue(0.6, true); //head
//        mTrayMgr->createThickSlider(TL_TOPLEFT, "SpecularLobe0Slider", "Specular Light weight 0", sliderWidth, 50, 0, 1.0/*3*/, 100)->setValue(0.8, true); //head
//        mTrayMgr->createSeparator(TL_TOPLEFT, "DebugRTTSep2");  // this is a hack to give the debug RTT a bit more room
//        
//        mTrayMgr->createThickSlider(TL_TOPLEFT, "2ndSpecularPowerSlider", "1 Specular Power", sliderWidth, 50, 0.0, 200, 200)->setValue(69, true);
//        mTrayMgr->createThickSlider(TL_TOPLEFT, "2ndSpecularLightIntensityColorSlider", "1 Specular Light Intensity", sliderWidth, 50, 0, 10.0/*3*/, 50)->setValue(1.0, true); //head
//        //        mTrayMgr->createThickSlider(TL_TOPLEFT, "SpecularLobeMixerSlider", "Specular Lobe Mixer", sliderWidth, 50, 0, 1.0/*3*/, 100)->setValue(0.5, true); //head
//        mTrayMgr->createThickSlider(TL_TOPLEFT, "SpecularLobe1Slider", "Specular Light weight 1", sliderWidth, 50, 0, 1.0/*3*/, 100)->setValue(1.0, true); //head
       //---------- TOP LEFT END
        
        
//        mTrayMgr->createLabel(TL_TOPLEFT, "lblEyeParas", "Eye Parameters: ", sliderWidth);
//        mTrayMgr->createThickSlider(TL_TOPLEFT, "u_cube_roughnessSlider", "u_cube_roughness", sliderWidth, 50, 0.0, 0.5, 50)->setValue(0.1, true);
//        mTrayMgr->createThickSlider(TL_TOPLEFT, "highFreqNormalLodSlider", "highFreqNormalLod", sliderWidth, 50, 0.0, 10.0, 50)->setValue(2.0, true);
//        mTrayMgr->createThickSlider(TL_TOPLEFT, "lowFreqNormalLodSlider", "lowFreqNormalLod", sliderWidth, 50, 0.0, 10.0, 50)->setValue(0.0, true);
//        mTrayMgr->createThickSlider(TL_TOPLEFT, "lowFreqDownBoundSlider", "lowFreqDownBound", sliderWidth, 50, 0.0, 0.5, 50)->setValue(0.05, true);
//        mTrayMgr->createThickSlider(TL_TOPLEFT, "lowFreqUpBoundSlider", "lowFreqUpBound", sliderWidth, 50, 0.0, 0.5, 50)->setValue(0.42, true);
//        mTrayMgr->createThickSlider(TL_TOPLEFT, "wetnessSlider", "wetness", sliderWidth, 50, 0.0, 0.98, 50)->setValue(0.97, true);
//        mTrayMgr->createThickSlider(TL_TOPLEFT, "u_cube_specular_light_intensitySlider", "u_cube_specular_light_intensity", sliderWidth, 50, 0.0, 2.0, 50)->setValue(1.0, true);
//        mTrayMgr->createThickSlider(TL_TOPLEFT, "u_cube_diffuse_light_intensitySlider", "u_cube_diffuse_light_intensity", sliderWidth, 50, 0.0, 3.0, 50)->setValue(1.5, true);

        
        //---------- BOTTOM LEFT
//        mTrayMgr->createLabel(TL_BOTTOMLEFT, "lblDiffuse", "New Skin Parameters: ", sliderWidth);
//        //        mTrayMgr->createThickSlider(TL_BOTTOMLEFT, "NormalMapBlurLevelSlider", "Normal Map BlurLevel", sliderWidth, 50, 0.0, 20.0, 20)->setValue(2.0, true);
//        mTrayMgr->createThickSlider(TL_BOTTOMLEFT, "lowNormalBiasSlider", "low Normal Bias", sliderWidth, 50, 0.0, 10.0, 100)->setValue(8.0, true);
//        mTrayMgr->createThickSlider(TL_BOTTOMLEFT, "RedNormalTuneSlider", "Red Normal Tune", sliderWidth, 50, 0.0, 1.0, 100)->setValue(0.89, true);
//        mTrayMgr->createThickSlider(TL_BOTTOMLEFT, "GreenNormalTuneSlider", "Green Normal Tune", sliderWidth, 50, 0.0, 1.0, 100)->setValue(0.83, true);
//        mTrayMgr->createThickSlider(TL_BOTTOMLEFT, "BlueNormalTuneSlider", "Blue Normal Tune", sliderWidth, 50, 0.0, 1.0, 100)->setValue(0.43, true);
//        mTrayMgr->createThickSlider(TL_BOTTOMLEFT, "CurvatureControlSlider", "Using One Curvature", sliderWidth, 50, 0, 1/*3*/, 2)->setValue(1.0, true); //head
//        mTrayMgr->createThickSlider(TL_BOTTOMLEFT, "CurvatureScaleSlider", "Curvature Scale", sliderWidth, 50, 0.0001, 10/*3*/, 500)->setValue(8.0, true); //head
//        
//        mTrayMgr->createThickSlider(TL_BOTTOMLEFT, "AlphaSlider", "LUT Exposure", sliderWidth, 50, 0, 3.0/*3*/, 500)->setValue(0.50, true);
//        //        mTrayMgr->createThickSlider(TL_BOTTOMLEFT, "Pre_Scatter_Mixer", "PreScatter Mixer", sliderWidth, 50, 0, 1.0/*3*/, 100)->setValue(0.0, true);
//        //        mTrayMgr->createThickSlider(TL_BOTTOMLEFT, "AmbientSSSSlider", "AmbientSSS", sliderWidth, 50, 0, 1.0/*3*/, 2)->setValue(0.0, true);
//        //        mTrayMgr->createThickSlider(TL_BOTTOMLEFT, "RandomMapSlider", "Random Map", sliderWidth, 50, 0, 0.1/*3*/, 100)->setValue(0.0, true);
//        //        mTrayMgr->createThickSlider(TL_BOTTOMLEFT, "AOMapSlider", "AOMap Scale", sliderWidth, 50, 1.0, 2.0/*3*/, 50)->setValue(2.0, true);
//        
//        
//        mTrayMgr->createThickSlider(TL_BOTTOMLEFT, "DiffuseLightIntensitySlider", "Diffuse Light Intensity", sliderWidth, 50, 0, 2.0/*3*/, 50)->setValue(0.7, true); //head
//        mTrayMgr->createThickSlider(TL_BOTTOMLEFT, "AmbientLightIntensitySlider", "Ambient Light Intensity", sliderWidth, 50, 0, 2.0/*3*/, 50)->setValue(0.57, true); //head
        //---------- BOTTOM LEFT END

        
        mTrayMgr->createLabel(TL_TOPRIGHT, "general_settings_lb", "General Settings", sliderWidth);
//        rendering_mode_number = 2.0;
//        mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_rendering_mode_number", rendering_mode_number);
//        mTrayMgr->createThickSlider(TL_TOPRIGHT, "RenderingModeSlider", "Rendering Mode", sliderWidth, 50, 0.0, rendering_mode_number-1.0, rendering_mode_number)->setValue(0.0, true);
//        mTrayMgr->createThickSlider(TL_TOPRIGHT, "RoughColorMapSlider", "Using Rough Color Map", sliderWidth, 50, 0.0, 1.0, 2)->setValue(0.0, true);
//        mTrayMgr->createThickSlider(TL_TOPRIGHT, "MicroGeometrySlider", "Micro Geometry Scale", sliderWidth, 50, 0.0, 30, 300)->setValue(0.0, true);
//        mTrayMgr->createThickSlider(TL_TOPRIGHT, "RotateSlider", "Rotate Body", sliderWidth, 50, -1.0, 1.0, 3)->setValue(0, true);
        mTrayMgr->createThickSlider(TL_TOPRIGHT, "LightPosSlider", "Light Horizontal Displacement", sliderWidth, 50, 0.0, 250.0/*3*/, 1000)->setValue(250, true);
        mTrayMgr->createThickSlider(TL_TOPRIGHT, "LightPosSlider2", "Light Vertical Displacement", sliderWidth, 50, 0.0, 250.0/*3*/, 1000)->setValue(250, true);
        mTrayMgr->createThickSlider(TL_TOPRIGHT, "TemporarySlider", "Tempodary Slider", sliderWidth, 50, 0.0, 2.0/*3*/, 100)->setValue(0.6, true);
        mTrayMgr->createThickSlider(TL_TOPRIGHT, "TemporarySlider2", "Tempodary Slider 2", sliderWidth, 50, 0.0, 2.0/*3*/, 100)->setValue(2.0, true);
        //        mTrayMgr->createLabel(TL_TOPRIGHT, "lblAmbientRGB", "Ambient Light RGB: ", sliderWidth);
        //        mTrayMgr->createThickSlider(TL_TOPRIGHT, "R_AmbientLightColorSlider", "R", sliderWidth, 50, 0, 1, 100)->setValue(255.0/255.0, true); //head
        //        mTrayMgr->createThickSlider(TL_TOPRIGHT, "G_AmbientLightColorSlider", "G", sliderWidth, 50, 0, 1, 100)->setValue(255.0/255.0, true); //head
        //        mTrayMgr->createThickSlider(TL_TOPRIGHT, "B_AmbientLightColorSlider", "B", sliderWidth, 50, 0, 1, 100)->setValue(255.0/255.0, true); //head
        
        
//        mTrayMgr->createLabel(TL_BOTTOMRIGHT, "TraditionalSkinPL", "Traditional Skin Parameters: ", sliderWidth);
//        mTrayMgr->createThickSlider(TL_BOTTOMRIGHT, "TraditionalDiffuseLightIntensitySlider", "Diffuse Light Intensity", sliderWidth, 50, 0, 2.0/*3*/, 50)->setValue(0.3, true); //head
//        mTrayMgr->createThickSlider(TL_BOTTOMRIGHT, "TraditionalAmbientLightIntensitySlider", "Ambient Light Intensity", sliderWidth, 50, 0, 2.0/*3*/, 50)->setValue(0.8, true); //head
//        
        //Ling, END
        
        // a friendly reminder
        //        StringVector names;
        //        names.push_back("Help");
        //        mTrayMgr->createParamsPanel(TL_TOPRIGHT, "Help", 100, names)->setParamValue(0, "H/F1");
        
        //        mMeshMenu->selectItem(0);  // select first mesh
    }
    
    
    //Ling, slider, 2015.9.6
    void sliderMoved(Slider* slider)
    {
        float slider_value = (float)slider->getValue();
        if (slider->getName() == "RoughnessSlider")
        {
            
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_roughness", slider_value);
            //mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_roughness", slider_value);
            
        }
        else if (slider->getName() == "CurvatureScaleSlider")
        {
            //materialScriptParameterSetting(slider_value);
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_CurvatureScale", slider_value);
            //mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_CurvatureScale", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_CurvatureScale", slider_value);
            
        }
        else if (slider->getName() == "CurvatureControlSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_CurvatureOne", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_CurvatureOne", slider_value);
        }
        else if (slider->getName() == "RedNormalTuneSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_tuneNormalNlurRed", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_tuneNormalNlurRed", slider_value);
        }
        else if (slider->getName() == "GreenNormalTuneSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_tuneNormalNlurGreen", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_tuneNormalNlurGreen", slider_value);
        }
        else if (slider->getName() == "BlueNormalTuneSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_tuneNormalNlurBlue", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_tuneNormalNlurBlue", slider_value);
        }
        else if (slider->getName() == "lowNormalBiasSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_lowNormalBias", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_lowNormalBias", slider_value);
        }
        //        else if (slider->getName() == "TurnSkinDiffuseSlider")
        //        {
        //            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_skin_diffuse", slider_value);
        //        }
        else if (slider->getName() == "SpecularPowerSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_specular_power", slider_value);
            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_specular_power", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_specular_power", slider_value);
            mMatPtr_3->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_specular_power", slider_value);
            
        }
        else if (slider->getName() == "2ndSpecularPowerSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_specular_power_2nd", slider_value);
            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_specular_power_2nd", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_specular_power_2nd", slider_value);
            mMatPtr_3->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_specular_power_2nd", slider_value);
            
        }
        
        else if (slider->getName() == "RenderingModeSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_rendering_mode", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_rendering_mode", slider_value);
        }
        else if (slider->getName() == "NormalMapBlurLevelSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_normal_map_blur_level", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_normal_map_blur_level", slider_value);
            
        }
        else if (slider->getName() == "DiffuseLightIntensitySlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_diffuse_intensity", slider_value);
            //            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_diffuse_intensity", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_diffuse_intensity", slider_value);
            
            
        }
        else if (slider->getName() == "SpecularLightIntensityColorSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_specular_intensity", slider_value);
            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_specular_intensity", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_specular_intensity", slider_value);
            mMatPtr_3->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_specular_intensity", slider_value);
            
        }
        else if (slider->getName() == "2ndSpecularLightIntensityColorSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_specular_intensity_2nd", slider_value);
            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_specular_intensity_2nd", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_specular_intensity_2nd", slider_value);
            mMatPtr_3->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_specular_intensity_2nd", slider_value);
            
        }
        //        else if (slider->getName() == "SpecularLobeMixerSlider")
        //        {
        //            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_mixer_spec_lobe", slider_value);
        //            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_mixer_spec_lobe", slider_value);
        //            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_mixer_spec_lobe", slider_value);
        //            mMatPtr_3->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_mixer_spec_lobe", slider_value);
        //
        //        }
        else if (slider->getName() == "SpecularLobe0Slider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_spec_lobe_0", slider_value);
            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_spec_lobe_0", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_spec_lobe_0", slider_value);
            mMatPtr_3->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_spec_lobe_0", slider_value);
        }
        else if (slider->getName() == "SpecularLobe1Slider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_spec_lobe_1", slider_value);
            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_spec_lobe_1", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_spec_lobe_1", slider_value);
            mMatPtr_3->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_spec_lobe_1", slider_value);
        }
        
        else if (slider->getName() == "AmbientLightIntensitySlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_ambient_intensity", slider_value);
            //            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_ambient_intensity", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_ambient_intensity", slider_value);
            
        }
        else if (slider->getName() == "TraditionalDiffuseLightIntensitySlider")
        {
            //            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_diffuse_intensity", slider_value);
            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_diffuse_intensity", slider_value);
            mMatPtr_3->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_diffuse_intensity", slider_value);
            
        }
        else if (slider->getName() == "TraditionalAmbientLightIntensitySlider")
        {
            //            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_ambient_intensity", slider_value);
            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_ambient_intensity", slider_value);
            mMatPtr_3->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_light_ambient_intensity", slider_value);
            
        }
        
        else if (slider->getName() == "AlphaSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("alpha", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("alpha", slider_value);
        }
        else if (slider->getName() == "LightPosSlider")
        {
            //          l->setPosition(60, 100, 20.0 + slider_value);   //ios is around 40
            //          l_1->setPosition(-60, 100, 20.0 + slider_value);   //ios is around 40
            //            l->setPosition(100, 100, 20.0 + slider_value);   //side light
            //            l_1->setPosition(100, 100, 20.0 + slider_value);   //ios is around 40
            
            //          bbs->getBillboard(0)->setPosition(12, 0, 20.0 + slider_value);
            
            
            l->setPosition( Vector3(l->getPosition().x, l->getPosition().y, slider_value) );   //ios is around 40
            l_1->setPosition( Vector3(l_1->getPosition().x, l->getPosition().y, slider_value) );   //ios is around 40
            bbs->getBillboard(0)->setPosition(l->getPosition());
        }
        else if (slider->getName() == "LightPosSlider2")
        {
            l->setPosition( Vector3(l->getPosition().x, slider_value-100, l->getPosition().z) );   //ios is around 40
            l_1->setPosition( Vector3(l_1->getPosition().x, slider_value-100, l->getPosition().z) );   //ios is around 40
            bbs->getBillboard(0)->setPosition(l->getPosition());
            
        }
        
        else if (slider->getName() == "RotateSlider")
        {
            if(slider_value ==1 )
            {
                mObjectNode->rotate(Vector3(0.0,1.0,0.0), Degree(1.0));
                mObjectNode_1->rotate(Vector3(0.0,1.0,0.0), Degree(1.0));
                mObjectNode_2->rotate(Vector3(0.0,1.0,0.0), Degree(1.0));
            }
            else if(slider_value ==-1 )
            {
                mObjectNode->rotate(Vector3(0.0,1.0,0.0), Degree(-1.0));
                mObjectNode_1->rotate(Vector3(0.0,1.0,0.0), Degree(-1.0));
                mObjectNode_2->rotate(Vector3(0.0,1.0,0.0), Degree(-1.0));
            }
            mViewport->setBackgroundColour(ColourValue(133.0/255.0, 139.0/255.0,151.0/255.0, 1.0));
        }
        else if (slider->getName() == "RoughColorMapSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_using_rough_color_map", slider_value);
            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_using_rough_color_map", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_using_rough_color_map", slider_value);
            mMatPtr_3->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_using_rough_color_map", slider_value);
        }
        else if (slider->getName() == "R_AmbientLightColorSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_ambient_light_color_r", slider_value);
            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_ambient_light_color_r", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_ambient_light_color_r", slider_value);
            mMatPtr_3->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_ambient_light_color_r", slider_value);
        }
        else if (slider->getName() == "G_AmbientLightColorSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_ambient_light_color_g", slider_value);
            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_ambient_light_color_g", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_ambient_light_color_g", slider_value);
            mMatPtr_3->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_ambient_light_color_g", slider_value);
        }
        else if (slider->getName() == "B_AmbientLightColorSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_ambient_light_color_b", slider_value);
            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_ambient_light_color_b", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_ambient_light_color_b", slider_value);
            mMatPtr_3->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_ambient_light_color_b", slider_value);
        }
        else if (slider->getName() == "Pre_Scatter_Mixer")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_pre_scatter_mix", slider_value);
            //            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_pre_scatter_mix", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_pre_scatter_mix", slider_value);
        }
        else if (slider->getName() == "AmbientSSSSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_ambient_sss", slider_value);
            //            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_pre_scatter_mix", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_ambient_sss", slider_value);
        }
        else if (slider->getName() == "RandomMapSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_random_scale", slider_value);
            //            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_pre_scatter_mix", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_random_scale", slider_value);
        }
        else if (slider->getName() == "AOMapSlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_AOMap_Scale", slider_value);
            //            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_pre_scatter_mix", slider_value);
            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_AOMap_Scale", slider_value);
        }
        else if (slider->getName() == "MicroGeometrySlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_micro_geometry_scale", slider_value);
            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_micro_geometry_scale", slider_value);
            //            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_micro_geometry_scale", slider_value);
            //            mMatPtr_3->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_micro_geometry_scale", slider_value);
        }
        else if (slider->getName() == "TemporarySlider")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_temporary", slider_value);
            //            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_micro_geometry_scale", slider_value);
            //            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_micro_geometry_scale", slider_value);
            //            mMatPtr_3->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_micro_geometry_scale", slider_value);
        }
        else if (slider->getName() == "TemporarySlider2")
        {
            mMatPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_temporary_2", slider_value);
            //            mMatPtr_1->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_micro_geometry_scale", slider_value);
            //            mMatPtr_2->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_micro_geometry_scale", slider_value);
            //            mMatPtr_3->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_micro_geometry_scale", slider_value);
//            mMatPtr_b_and_w->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_intensity", slider_value);
//            Ogre::CompositorManager::getSingleton()._reconstructAllCompositorResources();
        }
       
        else if (slider->getName() == "wetnessSlider")
        {
            mMatPtr_4->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_wetness", slider_value);
        }
        else if (slider->getName() == "lowFreqDownBoundSlider")
        {
            mMatPtr_4->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("lowFreqDownBound", slider_value);
        }
        else if (slider->getName() == "lowFreqUpBoundSlider")
        {
            mMatPtr_4->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("lowFreqUpBound", slider_value);
        }
        else if (slider->getName() == "highFreqNormalLodSlider")
        {
            mMatPtr_4->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("highFreqNormalLod", slider_value);
        }
        else if (slider->getName() == "lowFreqNormalLodSlider")
        {
            mMatPtr_4->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("lowFreqNormalLod", slider_value);
        }
        else if (slider->getName() == "u_cube_specular_light_intensitySlider")
        {
            mMatPtr_4->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_cube_specular_light_intensity", slider_value);
        }
        else if (slider->getName() == "u_cube_diffuse_light_intensitySlider")
        {
            mMatPtr_4->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_cube_diffuse_light_intensity", slider_value);
        }
        else if (slider->getName() == "u_cube_roughnessSlider")
        {
            mMatPtr_4->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("u_cube_roughness", slider_value);
        }
        
    }
    
    
    //Ling
    void materialScriptParameterSetting(float roughness)
    {
        
    }
    //Ling, End
    
    
    void cleanupContent()
    {
        // clean up properly to avoid interfering with subsequent samples
        for (std::map<String, StringVector>::iterator it = mPossibilities.begin(); it != mPossibilities.end(); it++)
            MeshManager::getSingleton().unload(it->first);
        mPossibilities.clear();
    }
    
    std::map<String, StringVector> mPossibilities;
    SceneNode* mObjectNode;
    SceneNode* mObjectNode_1;//Ling
    SceneNode* mObjectNode_2;//Ling
    SceneNode* mLightPivot1;
    SceneNode* mLightPivot2;
    SceneNode* mLightPivot3;
    bool mMoveLights;
    SelectMenu* mMeshMenu;
    SelectMenu* mMaterialMenu;
    Ogre::MaterialPtr mMatPtr; //Ling
    Ogre::MaterialPtr mMatPtr_1; //Ling
    Ogre::MaterialPtr mMatPtr_2; //Ling
    Ogre::MaterialPtr mMatPtr_3; //Ling
    Ogre::MaterialPtr mMatPtr_4; //Ling
    Ogre::MaterialPtr mMatPtr_b_and_w; //Ling
    Ogre::MaterialPtr mMatPtr_tilling; //Ling
    Ogre::MaterialPtr mMatPtr_b_and_w_gradually; //Ling

    float intensity;
    float total_time;
    bool decrease_flag;
    float circle_time ;    //seconds
    float up_bound;
    float down_bound;
    float odd;

    
    
    float rendering_mode_number;
    Light* l;
    Light* l_1;
    BillboardSet* bbs;
    
    
};

#endif
