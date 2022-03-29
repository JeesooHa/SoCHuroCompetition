#include "stdafx.h"
#include "Histogram.h"
#include "iostream"

void Histogram(CByteImage& img, int histo[256])
{
	int size = img.GetSize();
	BYTE* p = img.GetPtr();

	// ������׷� ���
	int cnt[256];
	memset(cnt, 0, sizeof(int) * 256);
	for (int i = 0; i < size; i++)
		cnt[p[i]]++;

	// ������׷� ����ȭ(histogram normalization)
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

	// int m_histogram[256] �迭 0���� �ʱ�ȭ
	memset(m_histogram, 0, 256 * sizeof(int));

	//������׷� Ž��
	for (int r = 0; r<nHeight; r++)
	{
		BYTE* pIn = image.GetPtr(r);
		
		for (int c = 0; c<nWidth; c++)
			m_histogram[pIn[c]]++;
	}

	//������׷� �ִ� Ž��
	int nMaxHist = INT_MIN;
	for (int i = 0; i<256; i++)
		if (m_histogram[i] > nMaxHist)
			nMaxHist = m_histogram[i];

	//������׷� �ִ����� ����ȭ
	double dDivMax = 256.0 / nMaxHist;
	m_histogram[0] *= dDivMax;

	for (int i = 1; i<256; i++){
		m_histogram[i] *= dDivMax;
	}

	//������׷� ����
	histImage.SetConstValue(255);
	for (int i = 0; i<256; i++){
		for (int h = 0; h<m_histogram[i]; h++)
			histImage.GetAt(i, 255 - h) = 128; // ������׷�
	}

	

}

void Histo::SumHist(const CByteImage hist1, const CByteImage hist2, CByteImage& HS) {

	int nWidth = hist1.GetWidth();
	int nHeight = hist1.GetHeight();
	
	for (int r = 0; r < nHeight; r++)
	{
		for (int c = 0; c < nWidth; c++) {
		if (hist1.GetAt(c, r) == 128 && hist2.GetAt(c, r) == 128)	//H,S ����
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

	ASSERT(src.GetChannel() == 3);	//�÷� ���� Ȯ��

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




float GetHistoBar(int* histogram)//Hk_bar�� ���ϴ� �Լ� ������׷��� input���� �Ѵ�, reference ����
{
	float HistoBar = 0;
	int SumHistogram = 0;

	for (int j = 0; j <= 255; j++)
	{
		SumHistogram += histogram[j];//������׷� ���� �� ��(�ñ׸� Hk(j))
	}
	HistoBar = (1.0 / 255.0)*SumHistogram;//Hk_bar�� ������׷� ���� �� ���� ������׷� ����� ����(255��)�� �� ������ ������ ��
	
	return HistoBar;//����� ��� ��ȯ
}

float HistCalc(int i, int* histogram)//Hk(I) - Hk_bar�� ���ϱ� ���� �Լ� ex)H1(I) - H1_bar, ������׷��� ������׷������� ��Ⱚ(i)�� input���� �Ѵ�
{
	float histcalc = 0;

	histcalc = histogram[i] - GetHistoBar(histogram);//(������׷��� i�� ��) - (GetHistoBar�Լ��� ���� ���� Hk_bar)
	return histcalc;//����Ѱ�� ��ȯ
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
		SumUpper += HistCalc(i,hist1)*HistCalc(i,hist2);//���ڸ� ���
	}
	for (int i = 0; i <= 255; i++)
	{
		LowerLeft += HistCalc(i, hist1)*HistCalc(i, hist1);// �и��� ��Ʈ ���� ���� (H1�� ���� ��)
		LowerRight += HistCalc(i,hist2)*HistCalc(i, hist2);//�и��� ��Ʈ ���� ������ (H2�� ���� ��)
	}
	SumLower = sqrt(LowerLeft*LowerRight);//�и� ���

	Result = SumUpper / SumLower;// ����/�и� �� ���� �������� correlation���� ���Ѵ�


	return Result;//��� ��ȯ

}
void Histo::value_Histo(Histo& hist1, Histo& hist2, float h) {

	h = _compareHist(hist1.H_histogram, hist2.H_histogram);
	//double s = _compareHist(hist1.S_hist, hist2.S_hist);
}
