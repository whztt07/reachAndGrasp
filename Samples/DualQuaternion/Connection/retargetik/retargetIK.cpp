#include "retargetIK.h"

#ifdef USE_IKMETHORDS

#define V3(v) VectorR3(v.x,v.y,v.z)
#define V3I(v) Ogre::Vector3(v.x,v.y,v.z)

namespace Connection
{

void Retarget::retargetIKDirect(RetargetParams Params)
{
	_entity = Params.dEntity;
	_skeleton = _entity->getSkeleton();
	Ogre::String rootName = Params.rootName;
	std::map<Ogre::String , Ogre::Quaternion> dRotate;
	bool rootProcess = Params.rootProcess;
	Ogre::Bone* root = _skeleton->getBone(Params.rootName);
	int targetNum = Params.Targets.size();
	std::vector<Ogre::String> freezeName(4);
	freezeName[0] = "leftForearmRoll"; freezeName[1] = "rightForearmRoll"; freezeName[2] = "leftArmRoll"; freezeName[3] = "rightArmRoll";
	double bScale = Params.scale;// body changing proportion 
	if(bScale>1.0)
		bScale = 2.0-bScale;
	bScale = bScale<0.2?0.2:bScale;
	double kp = 0.125;
	double scale = 0.1, step = 0.2;
	scale = 0.05 + (bScale-0.5)/6;
	scale = Params.scale;
	step = 2*scale;
	int num = 16;
	
	for(int n=0; n<Params.iterNum; n++)
	{
		
		// retarget end effector position to target position
		Ogre::Vector3 rootTrans = Ogre::Vector3::ZERO;
		dRotate.clear();
		for(int i=0; i<targetNum; i++)
		{
			if(!Params.Targets[i].posRestraint)
				continue;
			double pscale = scale+ step*n/Params.iterNum;
			pscale = 1.0/3;
			Ogre::Bone* effector = _skeleton->getBone(Params.Targets[i].effectorName);
			Ogre::Vector3 effectorPos = effector->_getDerivedPosition();
			Ogre::Vector3 targetPosition = Params.Targets[i].targetPosition;
			Ogre::Vector3 errorDis = targetPosition - effectorPos;
			Ogre::Quaternion targetOrient = Params.Targets[i].targetOrientation;
			Ogre::Bone* joint = (Ogre::Bone*)effector->getParent();
			while(joint)
			{
				Ogre::String boneName = joint->getName();
				std::vector<Ogre::String>::iterator it = std::find(freezeName.begin(), freezeName.end(), boneName);
				if(it!=freezeName.end()){
					joint = (Ogre::Bone*) joint->getParent();
					continue;
				}
				Ogre::Vector3 jointPos = joint->_getDerivedPosition();
				Ogre::Vector3 targetLine = targetPosition-jointPos;
				Ogre::Vector3 effectorLine = effectorPos-jointPos;
				Ogre::Vector3 targetRad = targetLine;
				Ogre::Vector3 effectorRad = effectorLine;
				double tlLength = targetLine.normalise();
				double elLength = effectorLine.normalise();
				double cosValue = targetLine.dotProduct(effectorLine);
				cosValue = (cosValue>1.0?1.0:(cosValue<-1.0?-1.0:cosValue));
				Ogre::Radian theta; 
				Ogre::Vector3 axis = effectorLine.crossProduct(targetLine);
				axis.normalise();
				Ogre::Vector3 dr = axis.crossProduct(effectorRad);
				double drLength = dr.squaredLength();
				double midAngle = std::acos(cosValue);
				theta = dr.dotProduct(errorDis)/drLength;
				theta = dr.angleBetween(errorDis);
				if(theta.valueRadians() > midAngle)
					theta = Ogre::Radian(midAngle);
				if(elLength==0)
					theta = 0;
				if(joint->getParent())
					axis = joint->getParent()->_getDerivedOrientation().Inverse()*axis;
				Ogre::Quaternion dQua(pscale*theta, axis);
			//	dQua = Ogre::Quaternion();
				int jHandle = joint->getHandle();
				
				if(dRotate.find(boneName)==dRotate.end())
					dRotate[boneName] = dQua;
				else
					dRotate[boneName] = dQua*dRotate[boneName];
				if(rootProcess){
					if(joint->getName()==rootName)
						break;
				}
				else
					if(joint->getParent()->getName()==rootName)
						break;
				joint = (Ogre::Bone*) joint->getParent();
				//pscale -= pscale/num;
				pscale *= (2.0/3);
			}
			if(rootProcess){
				Ogre::Vector3 trans = scale*errorDis;
				if(root->getParent())
					trans = root->getParent()->_getDerivedOrientation().Inverse()*trans;
				rootTrans += trans;
			//	root->setPosition(trans+root->getPosition());
			}
		}
		//std::map<Ogre::String, Ogre::Quaternion>::iterator it = dRotate.begin();
		//for(it=dRotate.begin(); it!=dRotate.end(); it++){
		//	Ogre::String boneName = it->first;
		//	Ogre::Bone* joint = _skeleton->getBone(boneName);
		//	Ogre::Quaternion rot = it->second*joint->getOrientation();
		//	if(boneName=="leftForearmRoll" || boneName=="rightForearmRoll" || boneName=="leftArmRoll" || boneName=="rightArmRoll")
		//		continue;
		//	joint->setOrientation(rot);
		//}
		//_skeleton->getRootBone()->_update(1,0);
		//// retarget end effector orientation to target orientation
		//dRotate.clear();
		for(int i=0; i<targetNum; i++)
		{
			int l=2;
			double tscale = 0.2;
			if(!Params.Targets[i].oriRestraint)
				continue;
			Ogre::Bone* effector = _skeleton->getBone(Params.Targets[i].effectorName);
			Ogre::Quaternion effectorOri = effector->_getDerivedOrientation();
			Ogre::Quaternion targetOri = Params.Targets[i].targetOrientation;
			Ogre::Quaternion Qua = targetOri*effectorOri.Inverse();
		//	Ogre::Quaternion qq = Qua*effectorOri;
			Ogre::Radian theta;
			Ogre::Vector3 axis;
			Qua.ToAngleAxis(theta, axis);
			theta *= 1.0;
		//	Ogre::Bone* joint = (Ogre::Bone*) effector->getParent();
			Ogre::Bone* joint = effector;
			while(joint)
			{
				Ogre::String boneName = joint->getName();
				std::vector<Ogre::String>::iterator it = std::find(freezeName.begin(), freezeName.end(), boneName);
				if(it!=freezeName.end()){
					joint = (Ogre::Bone*) joint->getParent();
					continue;
				}
				Ogre::Vector3 dAxis = axis;
				if(joint->getParent())
					dAxis = joint->getParent()->_getDerivedOrientation().Inverse()*dAxis;
				Ogre::Quaternion dQua = Ogre::Quaternion(tscale*theta, dAxis);
				int jHandle = joint->getHandle();
				
				if(dRotate.find(boneName)==dRotate.end())
					dRotate[boneName] = dQua;
				else
					dRotate[boneName] = dQua*dRotate[boneName];
				if(rootProcess){
					if(joint->getName()==rootName)
						break;
				}
				else
					if(joint->getParent()->getName()==rootName)
						break;
				joint = (Ogre::Bone*) joint->getParent();
				tscale /= l;
			}
		}
		std::map<Ogre::String, Ogre::Quaternion>::iterator it2 = dRotate.begin();
		for(it2=dRotate.begin(); it2!=dRotate.end(); it2++){
			//int handle = it2->first;
			Ogre::String boneName = it2->first;
		//	if(boneName=="leftForearmRoll" || boneName=="rightForearmRoll" || boneName=="leftArmRoll" || boneName=="rightArmRoll")
		//		continue;
			Ogre::Bone* joint = _skeleton->getBone(boneName);
			Ogre::Quaternion rot = it2->second*joint->getOrientation();
			joint->setOrientation(rot);
		}
		if(rootProcess)
			root->setPosition(rootTrans+root->getPosition());
		_skeleton->getRootBone()->_update(1,0);
		//_entity->_updateAnimation();

		// calculate end effector error of position and orientation
		double error=0, errorOri=0;
		int ptNum = 0, rtNum = 0;
		for(int i=0; i<Params.Targets.size(); i++)
		{
			EffectorTarget et = Params.Targets[i];
			Ogre::Bone* effect = _skeleton->getBone(et.effectorName);
			if(et.posRestraint){
				Ogre::Vector3 effectPos = effect->_getDerivedPosition();
				Ogre::Vector3 s = et.targetPosition - effectPos;
				error += s.length();
				ptNum++;
			}
			if(et.oriRestraint){
				Ogre::Quaternion effectorOri = effect->_getDerivedOrientation();
				Ogre::Quaternion c = et.targetOrientation*effectorOri.Inverse();
				Ogre::Radian r; Ogre::Vector3 a;
				c.ToAngleAxis(r,a);
				errorOri += r.valueDegrees();
				rtNum++;
			}
		}
		if(error < 0.5*ptNum && errorOri < 5*rtNum)
			break;
	}
}

void Retarget::retargetIKJacobian(RetargetParams Params, const std::vector<Ogre::Quaternion> &referQuatList)
{
	_entity = Params.dEntity;
	_skeleton = _entity->getSkeleton();
	Ogre::String rootName = Params.rootName;
	std::map<Ogre::String , Ogre::Quaternion> dRotate;
	bool rootProcess = Params.rootProcess;
	int targetNum = Params.Targets.size();
	double scale = 0.2;
	std::vector<Ogre::String> freezeName(4);
	freezeName[0] = "leftForearmRoll"; freezeName[1] = "rightForearmRoll"; freezeName[2] = "leftArmRoll"; freezeName[3] = "rightArmRoll";

	double maxposoff = 10.0;
	double maxorioff = 0.3;
	int row = 0;
	for(int i=0; i<targetNum; i++)
		if(Params.Targets[i].posRestraint)
			row++;
	int posrow = row;
	for(int i=0; i<targetNum; i++)
		if(Params.Targets[i].oriRestraint)
			row++;
	int orirow = row - posrow;
	cv::Mat dS = cv::Mat(row*3, 1, CV_32FC1);
	std::map<Ogre::String, int> mapColName;
	std::vector<Ogre::String> nodeNames;
	std::vector<std::vector<Ogre::String>> nodeNameList(targetNum);
	int col = 0;
	for(int i=0; i<Params.Targets.size(); i++)
	{	
		nodeNameList[i].clear();
		Ogre::String effectorName = Params.Targets[i].effectorName;
		Ogre::Bone* effector = _skeleton->getBone(effectorName);
		Ogre::Vector3 endPos = effector->_getDerivedPosition();
		Ogre::Vector3 targetPos = Params.Targets[i].targetPosition;
		nodeNameList[i].push_back(effectorName);
		std::vector<Ogre::String>::iterator it = std::find(nodeNames.begin(), nodeNames.end(),effectorName);
		if(it==nodeNames.end()){
			mapColName[effectorName] = col;
			nodeNames.push_back(effectorName);
			col++;
		}
		Ogre::Bone* node = (Ogre::Bone*) effector->getParent();
		if(effectorName == Params.rootName)
			continue;
		while(node )
		{
			Ogre::String nodeName = node->getName();
			if(nodeName==Params.rootName)
				break;
			std::vector<Ogre::String>::iterator itf = std::find(freezeName.begin(), freezeName.end(), nodeName);
			if(itf != freezeName.end()){
				node = (Ogre::Bone*) node->getParent();
				continue;
			}
			nodeNameList[i].push_back(nodeName);
			it = std::find(nodeNames.begin(), nodeNames.end(), nodeName);
			if(it==nodeNames.end()){
				mapColName[nodeName] = col;
				nodeNames.push_back(nodeName);
				col++;
			}
			node = (Ogre::Bone*) node->getParent();
		}
	}

	for(int n=0; n<Params.iterNum; n++)
	{
		cv::Mat jac = cv::Mat::zeros(3*row, 3*col, CV_32FC1);
		int rowidx = 0;
		/*  Posistion Jacobian Construction*/
		for(int i=0; i<targetNum; i++)
		{
			if(!Params.Targets[i].posRestraint)
				continue;
			Ogre::String effectorName = Params.Targets[i].effectorName;
			Ogre::Bone* effector = _skeleton->getBone(effectorName);
			Ogre::Vector3 endPos = effector->_getDerivedPosition();
			Ogre::Vector3 targetPos = Params.Targets[i].targetPosition;
			Ogre::Vector3 offset = targetPos - endPos;
			double len = offset.length();
			if(len > maxposoff)
				offset *= (maxposoff/len);
			dS.at<float>(rowidx*3+0,0) = offset.x;
			dS.at<float>(rowidx*3+1,0) = offset.y;
			dS.at<float>(rowidx*3+2,0) = offset.z;
		
			for(int j=1; j<nodeNameList[i].size(); j++)
			{
				Ogre::String nodeName = nodeNameList[i][j];
				int colidx = mapColName[nodeName];
				Ogre::Bone* node = _skeleton->getBone(nodeName);
				Ogre::Vector3 nodePos = node->_getDerivedPosition();
				Ogre::Vector3 radPos = endPos - nodePos;
				Ogre::Bone* parent = (Ogre::Bone*) node->getParent();
				Ogre::Quaternion parentQua;
				Ogre::Vector3 axis[3];

				if(parent)
					parentQua = parent->_getDerivedOrientation();
				else
					parentQua = Ogre::Quaternion();
				parentQua.ToAxes(axis[0], axis[1], axis[2]);
				for(int k=0; k<3; k++)
				{
					Ogre::Vector3 tangent = axis[k].crossProduct(radPos);
					jac.at<float>(rowidx*3+0, colidx*3+k) = tangent.x;
					jac.at<float>(rowidx*3+1, colidx*3+k) = tangent.y;
					jac.at<float>(rowidx*3+2, colidx*3+k) = tangent.z;
				}
			}
			rowidx++;
		}
		/* Orientation Jacobian Construction */
		for(int i=0; i<targetNum; i++)
		{
			if(!Params.Targets[i].oriRestraint)
				continue;
			Ogre::String effectorName = Params.Targets[i].effectorName;
			Ogre::Bone* effector = _skeleton->getBone(effectorName);
			Ogre::Quaternion endOri = effector->_getDerivedOrientation();
			Ogre::Quaternion targetOri = Params.Targets[i].targetOrientation;
			Ogre::Quaternion quaoffset = targetOri*endOri.Inverse();
			Ogre::Radian angle; Ogre::Vector3 axis;
			quaoffset.ToAngleAxis(angle, axis);
			double len = angle.valueRadians()*0.9;
			if(len > maxorioff)
				len = maxorioff;
			axis = axis*len;
			dS.at<float>(rowidx*3+0,0) = axis.x;
			dS.at<float>(rowidx*3+1,0) = axis.y;
			dS.at<float>(rowidx*3+2,0) = axis.z;

			for(int j=0; j<nodeNameList[i].size(); j++)
			{
				Ogre::String nodeName = nodeNameList[i][j];
				int colidx = mapColName[nodeName];
				Ogre::Bone* node = _skeleton->getBone(nodeName);
				Ogre::Bone* parent = (Ogre::Bone*) node->getParent();
				Ogre::Quaternion parentQua;
				Ogre::Vector3 axis[3];
				if(parent)
					parentQua = parent->_getDerivedOrientation();
				else
					parentQua = Ogre::Quaternion();
				parentQua.ToAxes(axis[0], axis[1], axis[2]);

				for(int k=0; k<3; k++)
				{
					jac.at<float>(rowidx*3+0, colidx*3+k) = axis[k].x;
					jac.at<float>(rowidx*3+1, colidx*3+k) = axis[k].y;
					jac.at<float>(rowidx*3+2, colidx*3+k) = axis[k].z;
				}
			}
			rowidx++;
		}
		/* solve jacobian equation */
		double damping = 150;
		cv::Mat jact = jac.t();
		cv::Mat jjt = jac*jact;
		cv::Mat jjtDamp;
		jjt.copyTo(jjtDamp);
		for(int i=0; i<jjtDamp.rows; i++)
			jjtDamp.at<float>(i,i) += damping;
		assert(jjtDamp.cols==dS.rows);
		cv::Mat jjtinv = jjtDamp.inv();
		cv::Mat dTheta = jact*jjtinv*dS;
		bool nullspace = 0;
		cv::Mat jjtNull, conNull;
		jjt.copyTo(jjtNull);
		if(nullspace)
		{
			float refRatio = 0.1;
			double refDamping = damping*refRatio;
			for(int i=0; i<jjtNull.rows; i++)
				if(i<posrow)
					jjtNull += refDamping;
			cv::Mat jjtNullInv = jjtNull.inv();
			conNull = jact*jjtNullInv*jac;
			cv::Mat ident = cv::Mat::eye(conNull.size(), conNull.type());
			conNull = ident - conNull;
		}

		/* set result to corespond bone */
		assert(dTheta.rows==(int)nodeNames.size()*3);
		/* transform to quaternion by axis-angle method */
		for(int i=0; i<nodeNames.size(); i++)
		{
			Ogre::String nodeName = nodeNames[i];
			Ogre::Vector3 axis;
			axis.x = dTheta.at<float>(i*3+0,0);
			axis.y = dTheta.at<float>(i*3+1,0);
			axis.z = dTheta.at<float>(i*3+2,0);
			double angle = axis.normalise();
			Ogre::Radian theta = Ogre::Radian(angle);
			Ogre::Quaternion qua(theta, axis);
			qua.normalise();
			if(theta.valueDegrees()>180.0)
				qua = -qua;
			Ogre::Bone* joint = _skeleton->getBone(nodeName);
			joint->setOrientation(qua*joint->getOrientation());
		}
		/* transform to quaternion by Euler-angle method */
		//for(int i=0; i<nodeNames.size(); i++)
		//{
		//	Ogre::String nodeName = nodeNames[i];
		//	Ogre::Vector3 axis[3];
		//	axis[0] = Ogre::Vector3(1,0,0);
		//	axis[1] = Ogre::Vector3(0,1,0);
		//	axis[2] = Ogre::Vector3(0,0,1);
		//	Ogre::Radian theta[3];
		//	theta[0] = Ogre::Radian(dTheta.at<float>(i*3+0,0));
		//	theta[1] = Ogre::Radian(dTheta.at<float>(i*3+1,0));
		//	theta[2] = Ogre::Radian(dTheta.at<float>(i*3+2,0));
		//	Ogre::Quaternion qua[3];
		//	for(int j=0; j<3; j++){
		//		qua[j] = Ogre::Quaternion(theta[j], axis[j]);
		//		qua[j].normalise();
		//	}
		//	Ogre::Bone* joint = _skeleton->getBone(nodeName);
		//	joint->setOrientation(qua[2]*qua[1]*qua[0]*joint->getOrientation());
		//}

		_skeleton->getRootBone()->_update(1,0);

		if(referQuatList.size() == 0)
			continue;
		if(nullspace)
		{
			cv::Mat dSref = cv::Mat::zeros(3*col, 1, jac.type());
			assert(dSref.rows==3*nodeNames.size());
			float maxRot = 0.5;
			for(int i=0; i<nodeNames.size(); i++)
			{
				Ogre::String nodeName = nodeNames[i];
				Ogre::Bone* node = _skeleton->getBone(nodeName);
				int handle = node->getHandle();
				Ogre::Quaternion ref = referQuatList[handle];
				Ogre::Quaternion cur = node->getOrientation();
				Ogre::Quaternion diff = ref*cur.Inverse();
				Ogre::Radian theta; Ogre::Vector3 axis;
				double angle = theta.valueRadians();
				if(angle > maxRot)
					angle = maxRot;
				axis *= angle;
				dSref.at<float>(i*3+0,0) = axis.x;
				dSref.at<float>(i*3+1,0) = axis.y;
				dSref.at<float>(i*3+2,0) = axis.z;
			}
			cv::Mat dTheta = conNull*dSref;
			for(int i=0; i<nodeNames.size(); i++)
			{
				Ogre::String nodeName = nodeNames[i];
				Ogre::Vector3 axis;
				axis.x = dTheta.at<float>(i*3+0,0);
				axis.y = dTheta.at<float>(i*3+1,0);
				axis.z = dTheta.at<float>(i*3+2,0);
				double angle = axis.normalise();
				Ogre::Radian theta = Ogre::Radian(angle);
				Ogre::Quaternion qua(theta, axis);
				Ogre::Bone* joint = _skeleton->getBone(nodeName);
				joint->setOrientation(qua*joint->getOrientation());
			}
			_skeleton->getRootBone()->_update(1,0);
		}
	}
}

}
#endif
