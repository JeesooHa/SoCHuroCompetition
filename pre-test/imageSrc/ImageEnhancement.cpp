#include "stdafx.h"
#include "ImageEnhancement.h"




void _MeanFiltering(const CByteImage& m_imageIn, CByteImage& m_imageOut)
{
	ASSERT(m_imageIn.GetChannel() == 3); // 컬러 영상 확인

	int m_nMeanSize = 3;

	int nWidth = m_imageIn.GetWidth();
	int nHeight = m_imageIn.GetHeight();
	int nChnnl = m_imageIn.GetChannel();
	int nWStep = m_imageIn.GetWStep();
	int nHalf = m_nMeanSize / 2;

	BYTE* pIn = m_imageIn.GetPtr();
	BYTE* pOut = m_imageOut.GetPtr();

	int r, c, l;
	for (r = 0; r<nHeight; r++) // 행 이동
	{
		for (c = 0; c<nWidth; c++) // 열 이동
		{
			for (l = 0; l<nChnnl; l++) // 채널 이동
			{
				int nSum = 0;  // 픽셀 값의 합
				int nCnt = 0; // 픽셀 수
				for (int y = -nHalf; y <= nHalf; y++)
				{
					for (int x = -nHalf; x <= nHalf; x++)
					{
						int px = c + x;
						int py = r + y;

						if (px >= 0 && px<nWidth && py >= 0 && py<nHeight)
						{
							nSum += pIn[nWStep*py + nChnnl*px + l];
							nCnt++;
						}
					}
				}
				pOut[nWStep*r + nChnnl*c + l] = (BYTE)(nSum / (double)nCnt);
			} // 채널 이동 끝
		} // 열 이동 끝
	} // 행 이동 끝

}



//중간값 필터링

int cmpInt(const void *arg1, const void *arg2)
{
	return (*(int*)arg1 - *(int*)arg2);
}

void _MedianFiltering(const CByteImage& m_imageIn, CByteImage& m_imageOut)
{
	ASSERT(m_imageIn.GetChannel() == 3); // 컬러 영상 확인

	int	m_bufMed[9];
	int m_nMedSize = 3;	//3x3 마스크


	int nWidth = m_imageIn.GetWidth();
	int nHeight = m_imageIn.GetHeight();
	int nChnnl = m_imageIn.GetChannel();
	int nWStep = m_imageIn.GetWStep();
	BYTE* pIn = m_imageIn.GetPtr();
	BYTE* pOut = m_imageOut.GetPtr();

	int nHalf = m_nMedSize / 2;

	int r, c, l;
	for (r = 0; r<nHeight; r++) // 행 이동
	{
		for (c = 0; c<nWidth; c++) // 열 이동
		{
			for (l = 0; l<nChnnl; l++) // 채널 이동
			{
				int nCnt = 0; // 픽셀 수
				for (int y = -nHalf; y <= nHalf; y++)
				{
					for (int x = -nHalf; x <= nHalf; x++)
					{
						int px = c + x;
						int py = r + y;

						if (px >= 0 && px<nWidth && py >= 0 && py<nHeight)
						{
							m_bufMed[nCnt++] = pIn[nWStep*py + nChnnl*px + l];
						}
					}
				}
				qsort((void*)m_bufMed, nCnt, sizeof(int), cmpInt);
				pOut[nWStep*r + nChnnl*c + l] = m_bufMed[nCnt / 2];
			} // 채널 이동 끝
		} // 열 이동 끝
	} // 행 이동 끝
}
