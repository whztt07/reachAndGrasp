#ifndef    _BARYCENTRIC_MATH_H
#define    _BARYCENTRIC_MATH_H

#include "Ogre.h"
#include <math.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "../Connection/tetgen/tetgen.h"

class Tetrahe
{
public:
	int pidx[4];

};

class BarycentricInterpolator
{
public:
	void constructTetrahedron(std::vector<std::vector<float>> lps);
	bool tetraheInside(std::vector<float> lp, Tetrahe &t);
	bool calculateTetraheWeights(const std::vector<float> &lp, std::vector<std::pair<int, float>> &weights);

private:
	std::vector<Tetrahe> tetrahes;
	std::vector<double> tetraheDets;
	std::vector<cv::Mat> tetraheMats;
};

#endif