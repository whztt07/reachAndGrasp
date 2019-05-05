#include "barycentric.h"

void BarycentricInterpolator::constructTetrahedron(std::vector<std::vector<float>> lps)
{
	tetgenio ptIn, tetOut;
	ptIn.numberofpoints = lps.size();
	int ndim = lps[0].size();
	ptIn.pointlist = new REAL[lps.size()*ndim];
	for(int i=0; i<lps.size(); i++)
	{
		std::vector<float> lp = lps[i];
		for(int j=0; j<ndim; j++)
			ptIn.pointlist[i*ndim+j] = lp[j];

	}
	tetrahedralize(/*(char*)"V"*/&tetgenbehavior(), &ptIn, &tetOut);
	assert(tetOut.numberofcorners==4);
	tetrahes.clear();
	for(int i=0; i<tetOut.numberoftetrahedra; i++)
	{
		Tetrahe tet;
		for(int j=0; j<tetOut.numberofcorners; j++)
			tet.pidx[j] = tetOut.tetrahedronlist[i*tetOut.numberofcorners+j];
		tetrahes.push_back(tet);
	}

	tetraheDets.clear();
	tetraheMats.clear();
	int nvol = ndim + 1;
	
	cv::Mat lastVol = cv::Mat::ones(nvol, 1, CV_32FC1);
	cv::Mat coorVol(nvol, ndim, CV_32FC1);
	cv::Rect coor(0, 0, ndim, nvol);
	for(int i=0; i<tetrahes.size(); i++)
	{
		cv::Mat tetraVol(nvol, nvol, CV_32FC1);
		lastVol.copyTo(tetraVol.col(ndim));
		for(int j=0; j<nvol; j++)
			for(int k=0; k<ndim; k++)
				coorVol.at<float>(j,k) = lps[tetrahes[i].pidx[j]][k];
		coorVol.copyTo(tetraVol(coor));
		tetraheMats.push_back(tetraVol);
		tetraheDets.push_back(cv::determinant(tetraVol));
	}

}

bool BarycentricInterpolator::calculateTetraheWeights(const std::vector<float> &lp, std::vector<std::pair<int, float>> &weights)
{
	int ndim = lp.size();
	int nvol = ndim + 1;
	weights.clear();
	weights.resize(nvol);
	for(int i=0; i<tetrahes.size(); i++)
	{
		Tetrahe t = tetrahes[i];
		float td = tetraheDets[i];
		if(td == 0.0)
			continue;
		cv::Mat tm;
		tetraheMats[i].copyTo(tm);
		int j=0;
		double tmid = cv::determinant(tm);
		std::vector<double> vols(nvol);
		for(; j<nvol; j++)
		{
			cv::Mat temp;
			tm.copyTo(temp);
			cv::Mat row = cv::Mat::ones(1, nvol, CV_32FC1);
			for(int k=0; k<ndim; k++)
				row.at<float>(0,k) = lp[k];
			row.copyTo(temp.row(j));
			vols[j] = cv::determinant(temp);
			if(vols[j]*td < 0)
				break;
		}
		if(j<nvol) //not in this tetrahedron
			continue;
		for(int n=0; n<nvol; n++){
			weights[n].first = tetrahes[i].pidx[n];
			weights[n].second = (float)(vols[n]/td);
		}
		return true;
	}

	weights.clear();
	return false;
}