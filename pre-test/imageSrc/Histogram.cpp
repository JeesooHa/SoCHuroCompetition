#include "stdafx.h"
#include "Histogram.h"
#include "iostream"

void Histogram(CByteImage& img, int histo[256])
{
	int size = img.GetSize();
	BYTE* p = img.GetPtr();

	// 히스토그램 계산
	int cnt[256];
	memset(cnt, 0, sizeof(int) * 256);
	for (int i = 0; i < size; i++)
		cnt[p[i]]++;

	// 히스토그램 정규화(histogram normalization)
	for (int i = 0; i < 256; i++)
	{
		histo[i] = static_cast<float>(cnt[i]) / size;
	}
}

void Histo::_calcHistogram(const CByteImage& image, CByteImage& histImage, int* m_histogram) {

	if (image.GetChannel() != 1)
	{
		CString szMsg;
		szMsg.Format("_calcHistogram : %d", image.GetChannel());
		AfxMessageBox(szMsg);
		return;
	}

	int nWidth = image.GetWidth();
	int nHeight = image.GetHeight();

	// int m_histogram[256] 배열 0으로 초기화
	memset(m_histogram, 0, 256 * sizeof(int));

	//히스토그램 탐색
	for (int r = 0; r<nHeight; r++)
	{
		BYTE* pIn = image.GetPtr(r);
		
		for (int c = 0; c<nWidth; c++)
			m_histogram[pIn[c]]++;
	}

	//히스토그램 최댓값 탐색
	int nMaxHist = INT_MIN;
	for (int i = 0; i<256; i++)
		if (m_histogram[i] > nMaxHist)
			nMaxHist = m_histogram[i];

	//히스토그램 최댓값으로 정규화
	double dDivMax = 256.0 / nMaxHist;
	m_histogram[0] *= dDivMax;

	for (int i = 1; i<256; i++){
		m_histogram[i] *= dDivMax;
	}

	//히스토그램 영상
	histImage.SetConstValue(255);
	for (int i = 0; i<256; i++){
		for (int h = 0; h<m_histogram[i]; h++)
			histImage.GetAt(i, 255 - h) = 128; // 히스토그램
	}

	

}

void Histo::SumHist(const CByteImage hist1, const CByteImage hist2, CByteImage& HS) {

	int nWidth = hist1.GetWidth();
	int nHeight = hist1.GetHeight();
	
	for (int r = 0; r < nHeight; r++)
	{
		for (int c = 0; c < nWidth; c++) {
		if (hist1.GetAt(c, r) == 128 && hist2.GetAt(c, r) == 128)	//H,S 공통
			HS.GetAt(c, r) = 0;
		else if(hist1.GetAt(c, r) == 128)	//H
			HS.GetAt(c, r) = 85;
		else if (hist2.GetAt(c, r) == 128)	//S
			HS.GetAt(c, r) = 170;
		else
			HS.GetAt(c, r) = 255;
		}
	}
}

Histo Histo::HS_histo(CByteImage src) {

	ASSERT(src.GetChannel() == 3);	//컬러 영상 확인

	int nWidth = src.GetWidth();
	int nHeight = src.GetHeight();

	CDoubleImage HSV = RGB2HSV(src); //Convert the captured frambgr_planese from BGR to HSV

	CByteImage H = CByteImage(nWidth, nHeight, 1);
	CByteImage S = CByteImage(nWidth, nHeight, 1);
	if (!HSV.IsEmpty()) {

		H = (HSV.GetChannelImg(2)*(255 / 360.0));	// H : 0~255
		S = (HSV.GetChannelImg(1) * 255);	// S : 0~255
		//CByteImage V = (HSV.GetChannelImg(0) * 255);	// V : 0~255
 
		//ShowImage(H, "check H");
		//ShowImage(S, "check S");
		//ShowImage(V, "check V");
	}

	Histo hist;

	_calcHistogram(H, hist.H_hist,hist.H_histogram);
	_calcHistogram(S, hist.S_hist,hist.S_histogram);
	SumHist(hist.H_hist, hist.S_hist, hist.HS_Image);

	return hist;

}




float GetHistoBar(int* histogram)//Hk_bar를 구하는 함수 히스토그램을 input으로 한다, reference 참고
{
	float HistoBar = 0;
	int SumHistogram = 0;

	for (int j = 0; j <= 255; j++)
	{
		SumHistogram += histogram[j];//히스토그램 값의 총 합(시그마 Hk(j))
	}
	HistoBar = (1.0 / 255.0)*SumHistogram;//Hk_bar는 히스토그램 값의 총 합을 히스토그램 빈들의 갯수(255개)의 총 합으로 나눠준 것
	
	return HistoBar;//계산한 결과 반환
}

float HistCalc(int i, int* histogram)//Hk(I) - Hk_bar를 구하기 위한 함수 ex)H1(I) - H1_bar, 히스토그램과 히스토그램에서의 밝기값(i)를 input으로 한다
{
	float histcalc = 0;

	histcalc = histogram[i] - GetHistoBar(histogram);//(히스토그램의 i의 값) - (GetHistoBar함수를 통해 구한 Hk_bar)
	return histcalc;//계산한결과 반환
}

float Histo::_compareHist(int * hist1, int* hist2)
{
	float Result;
	float SumUpper = 0;
	float SumLower = 0;
	float LowerLeft = 0;
	float LowerRight = 0;
	//float Result = 0;

	for (int i = 0; i <= 255; i++)
	{
		SumUpper += HistCalc(i,hist1)*HistCalc(i,hist2);//분자를 계산
	}
	for (int i = 0; i <= 255; i++)
	{
		LowerLeft += HistCalc(i, hist1)*HistCalc(i, hist1);// 분모중 루트 안의 왼쪽 (H1에 대한 식)
		LowerRight += HistCalc(i,hist2)*HistCalc(i, hist2);//분모중 루트 안의 오른쪽 (H2에 대한 식)
	}
	SumLower = sqrt(LowerLeft*LowerRight);//분모 계산

	Result = SumUpper / SumLower;// 분자/분모 를 통해 최종적인 correlation값을 구한다


	return Result;//결과 반환

}
void Histo::value_Histo(Histo& hist1, Histo& hist2, float h) {

	h = _compareHist(hist1.H_histogram, hist2.H_histogram);
	//double s = _compareHist(hist1.S_hist, hist2.S_hist);
}
