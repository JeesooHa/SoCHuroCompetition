#pragma once
#include "MyImage.h"
#include "MyImageFunc.h"
#include "ImageFrameWndManager.h"
#include <vector>

using namespace std;

#ifndef __HISTOGRAM_H__
#define __HISTOGRAM_H__

class Histo {
public:
	CByteImage H_hist = CByteImage(256, 256); // 히스토그램 영상
	CByteImage S_hist = CByteImage(256, 256); // 히스토그램 영상
	CByteImage HS_Image = CByteImage(256, 256); // 히스토그램 영상
	int		H_histogram[256];
	int		S_histogram[256];

	CByteImage m_imageIn;
	CByteImage m_imageOut;

	Histo() {};
	~Histo() {};
	void _calcHistogram(const CByteImage& image, CByteImage& histImage, int* m_histogram);
	Histo HS_histo(CByteImage src);
	void SumHist(const CByteImage hist1, const CByteImage hist2, CByteImage& HS);
	void value_Histo(Histo& hist1, Histo& hist2, float h);
	float _compareHist(int * hist1, int* hist2);

};

void Histogram(CByteImage& img, int histo[256]);

#endif
