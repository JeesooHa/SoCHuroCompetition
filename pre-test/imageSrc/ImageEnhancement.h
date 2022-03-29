#pragma once
#include "MyImage.h"
int cmpInt(const void *arg1, const void *arg2);
void _MedianFiltering(const CByteImage& m_imageIn, CByteImage& m_imageOut);
void _MeanFiltering(const CByteImage& m_imageIn, CByteImage& m_imageOut);