#pragma once
#include "MyImage.h"
#define MAX_CLUSTER 10

void SUM_HSV(const CByteImage& H_image, const CByteImage& S_image, const CByteImage& V_image, CByteImage& imageOut);
void BinarizationLH(const CByteImage& imageIn, CByteImage& imageOut, int nLow, int nHigh);
void Binarization(const CByteImage& imageIn, CByteImage& imageOut, int nThreshold);
int BinarizationAuto(const CByteImage& imageIn, CByteImage& imageOut, int nThreshold);
void BinarizationBG(const CByteImage& imageIn, const CByteImage& imageBG, CByteImage& imageOut, int nThreshold);
void BinarizationBGCol(const CByteImage& imageIn, const CByteImage& imageBG, CByteImage& imageOut, int nThreshold);
void KMeansSegmentation(const CByteImage& imageIn, CByteImage& imageOut, int nCluster, BYTE meanR[], BYTE meanG[], BYTE meanB[]);
	 