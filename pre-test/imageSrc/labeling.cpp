#include "stdafx.h"
#include "labeling.h"
#include <map>

using namespace std;

void Binarization(CByteImage& imgSrc, CByteImage& imgDst, int threshold)
{
	imgDst = CByteImage(imgSrc.GetWidth(), imgSrc.GetHeight());

	int size = imgSrc.GetSize();
	BYTE* pSrc = imgSrc.GetPtr();
	BYTE* pDst = imgDst.GetPtr();

	for (int i = 0; i < size; i++)
	{
		pDst[i] = (pSrc[i] <= threshold) ? 0 : 255;
	}
}

int BinarizationIterative(CByteImage& imgSrc)
{
	int hist[256] = { 0, };
	Histogram(imgSrc, hist); // ����ȭ�� ������׷�. hist �迭�� ������ [0, 1].

								// �ʱ� �Ӱ谪 ���� - �׷��̽����� ���� ��ü ���

	int i, T, Told;

	float sum = 0.f;
	for (i = 0; i < 256; i++)
		sum += (i * hist[i]);

	T = static_cast<int>(sum + .5f);

	// �ݺ��� ���� �Ӱ谪 ����

	float a1, b1, u1, a2, b2, u2;
	do {
		Told = T;

		a1 = b1 = u1 = 0.f;
		for (i = 0; i <= Told; i++)
		{
			a1 += (i*hist[i]);
			b1 += hist[i];
		}

		if (b1 != 0.f)
			u1 = a1 / b1;

		a2 = b2 = u2 = 0.f;
		for (i = Told + 1; i < 256; i++)
		{
			a2 += (i*hist[i]);
			b2 += hist[i];
		}

		if (b2 != 0.f)
			u2 = a2 / b2;

		T = static_cast<int>((u1 + u2) / 2 + 0.5f);
	} while (T != Told);

	return T;
}

int Labeling(const CByteImage& imgSrc, CIntImage& imgDst, vector<LabelInfo>& labels)
{
	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	//ShowImage(imgSrc,"imgSrc");
	//-------------------------------------------------------------------------
	// �ӽ÷� ���̺��� ������ �޸� ������ � ���̺� ����
	//-------------------------------------------------------------------------

	CIntImage imgMap(w, h);
	imgMap.SetConstValue(0);

	const int MAX_LABEL = 100000;
	int eq_tbl[MAX_LABEL][2] = { { 0, }, };

	//-------------------------------------------------------------------------
	// ù ��° ��ĵ - �ʱ� ���̺� ���� �� � ���̺� ����
	//-------------------------------------------------------------------------

	register int i, j;
	int label = 0, maxl, minl, min_eq, max_eq;

	for (j = 1; j < h; j++) {
		BYTE* pSrc = imgSrc.GetPtr(j);

		for (i = 1; i < w; i++){

			if (pSrc[i] == 255)
			{
				// �ٷ� �� �ȼ��� ���� �ȼ� ��ο� ���̺��� �����ϴ� ���
				if ((imgMap.GetAt(i, j-1) != 0) && (imgMap.GetAt(i-1, j) != 0))
				{
					if (imgMap.GetAt(i, j-1) == imgMap.GetAt(i-1, j))
					{
						// �� ���̺��� ���� ���� ���
						imgMap.GetAt(i, j) = imgMap.GetAt(i, j-1);
					}
					else
					{
						// �� ���̺��� ���� �ٸ� ���, ���� ���̺��� �ο�
						maxl = max(imgMap.GetAt(i, j-1), imgMap.GetAt(i-1, j));
						minl = min(imgMap.GetAt(i, j-1), imgMap.GetAt(i-1, j));

						imgMap.GetAt(i, j) = minl;

						// � ���̺� ����
						min_eq = min(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = max(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}
				}
				else if (imgMap.GetAt(i, j-1) != 0)
				{
					// �ٷ� �� �ȼ����� ���̺��� ������ ���
					imgMap.GetAt(i, j) = imgMap.GetAt(i, j-1);
				}
				else if (imgMap.GetAt(i-1, j) != 0)
				{
					// �ٷ� ���� �ȼ����� ���̺��� ������ ���
					imgMap.GetAt(i, j) = imgMap.GetAt(i-1, j);
				}
				else
				{
					// �̿��� ���̺��� �������� ������ ���ο� ���̺��� �ο�
					label++;
					imgMap.GetAt(i, j) = label;
					eq_tbl[label][0] = label;
					eq_tbl[label][1] = label;
				}

			}
		}
	}

	//ShowImage(imgMap, "MAP");
	/*
	CByteImage MMap(w, h);

	MMap.SetConstValue(0);

	for (j = 0; j<h; j++)
		for (i = 0; i < w; i++) {
			if (imgMap.GetAt(i, j) > 1)
				MMap.GetAt(i, j) = 255;
		}

	ShowImage(MMap, "Check");
	*/

	//-------------------------------------------------------------------------
	// � ���̺� ����
	//-------------------------------------------------------------------------

	int temp;
	for (i = 1; i <= label; i++)
	{
		temp = eq_tbl[i][1];
		if (temp != eq_tbl[i][0])
			eq_tbl[i][1] = eq_tbl[temp][1];
	}

	// � ���̺��� ���̺��� 1���� ���ʴ�� ������Ű��

	int* hash = new int[label + 1];
	memset(hash, 0, sizeof(int)*(label + 1));

	for (i = 1; i <= label; i++)
		hash[eq_tbl[i][1]] = eq_tbl[i][1];

	int label_cnt = 1;
	for (i = 1; i <= label; i++)
		if (hash[i] != 0)
			hash[i] = label_cnt++;

	for (i = 1; i <= label; i++)
		eq_tbl[i][1] = hash[eq_tbl[i][1]];

	delete[] hash;

	//-------------------------------------------------------------------------
	// �� ��° ��ĵ - � ���̺��� �̿��Ͽ� ��� �ȼ��� ������ ���̺� �ο�
	//-------------------------------------------------------------------------

	imgDst = CByteImage(w, h);
	imgDst.SetConstValue(0);

	int idx;
	for (j = 1; j < h; j++)
		for (i = 1; i < w; i++)
		{
			if (imgMap.GetAt(i, j) != 0)
			{
				idx = imgMap.GetAt(i, j);
				imgDst.GetAt(i, j) = eq_tbl[idx][1];
			}
		}

	//ShowImage(imgDst, "DST");

	//-------------------------------------------------------------------------
	// IppLabelInfo ���� �ۼ�
	//-------------------------------------------------------------------------

	labels.resize(label_cnt - 1);

	LabelInfo* pLabel;
	for (j = 1; j < h; j++)
		for (i = 1; i < w; i++){
			if (imgDst.GetAt(i,j) != 0){

				pLabel = &labels.at(imgDst.GetAt(i, j) - 1);
				pLabel->pixels.push_back(Point(i, j));
				pLabel->cx += i;
				pLabel->cy += j;
				
				
				if (i < pLabel->minx) pLabel->minx = i;
				if (i > pLabel->maxx) pLabel->maxx = i;
				if (j < pLabel->miny) pLabel->miny = j;
				if (j > pLabel->maxy) pLabel->maxy = j;
			}
		}

	for (LabelInfo& label : labels)
	{
		label.cx /= label.pixels.size();
		label.cy /= label.pixels.size();
	}
	
	return	(label_cnt - 1);
}


void ContourTracing(CByteImage& imgSrc, int sx, int sy, vector<Point>& cp)
{
	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	// �ܰ��� ��ǥ�� ������ ����ü �ʱ�ȭ
	cp.clear();

	// �ܰ��� ���� ���� �ȼ��� ��ü�� �ƴϸ� ����
	if (imgSrc.GetAt(sx,sy) != 255)
		return;

	int x, y, nx, ny;
	int d, cnt;
	int  dir[8][2] = { // ���� ������ ��Ÿ���� �迭
		{ 1,  0 },
		{ 1,  1 },
		{ 0,  1 },
		{ -1,  1 },
		{ -1,  0 },
		{ -1, -1 },
		{ 0, -1 },
		{ 1, -1 }
	};

	x = sx;
	y = sy;
	d = cnt = 0;

	while (1)
	{
		nx = x + dir[d][0];
		ny = y + dir[d][1];

		if (nx < 0 || nx >= w || ny < 0 || ny >= h || imgSrc.GetAt(nx, ny) == 0)
		{
			// ���� ���⿡ �ִ� �ȼ��� ��ü�� �ƴ� ���,
			// �ð� �������� ���� ������ �ٲٰ� �ٽ� �õ��Ѵ�.

			if (++d > 7) d = 0;
			cnt++;

			// 8���� ��� ����� ��� 
			if (cnt >= 8)
			{
				cp.push_back(Point(x, y));
				break;  // �ܰ��� ������ ����.
			}
		}
		else
		{
			// ���� ������ �ȼ��� ��ü�� ���, ���� ���� �ܰ��� ������ ����
			cp.push_back(Point(x, y));

			// ���� �������� �̵�
			x = nx;
			y = ny;

			// ���� ���� �ʱ�ȭ
			cnt = 0;
			d = (d + 6) % 8;	// d = d - 2 �� ���� ����
		}

		// ���������� ���ƿ԰�, ���� ������ �ʱ�ȭ�� ���
		// �ܰ��� ������ ������.
		if (x == sx && y == sy && d == 0)
			break;
	}
}


