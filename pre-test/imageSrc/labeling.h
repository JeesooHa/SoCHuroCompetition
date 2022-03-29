#pragma once
#include <vector>
#include "Hajima.h"
#include "ImageView.h"
#include "Histogram.h"

using namespace std;


class LabelInfo
{
public:
	vector<Point> pixels;	//넓이 기준으로 사용 pixels.size()
	int cx, cy;
	int minx, miny, maxx, maxy;


public:
	LabelInfo() : cx(0), cy(0), minx(9999), miny(9999), maxx(0), maxy(0)
	{
		pixels.clear();
	}
};

void Binarization(CByteImage& imgSrc, CByteImage& imgDst, int threshold);
int  BinarizationIterative(CByteImage& imgSrc);
int  Labeling(const CByteImage& imgSrc, CIntImage& imgDst, vector<LabelInfo>& labels);
void ContourTracing(CByteImage& imgSrc, int sx, int sy, vector<Point>& cp);
