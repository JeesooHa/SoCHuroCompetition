#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <math.h>
#include "graphic_api.h"
#include "hajima.h"
#include <time.h>

//------------------- BMP SAVE IMAGES ----------------

#define BI_RGB  0L
#define m_nChannels 3

//-------------------- Filtering -----------------------

void MeanFiltering(const U8* imgIn, U8* imgOut, int nWidth, int nHeight)
{
	int m_nMeanSize = 3;
	int nHalf = m_nMeanSize / 2;
	int px = 0, py = 0;
	int r = 0, c = 0, l = 0, x = 0, y = 0;

	for (l = 0; l<m_nChannels; l++) { // 채널 이동
		for (r = 0; r<nHeight; r++) { // 행 이동
			for (c = 0; c<nWidth; c++) { // 열 이동

				int nSum = 0;  // 픽셀 값의 합
				int nCnt = 0; // 픽셀 수
				for (y = -nHalf; y <= nHalf; y++) {
					for (x = -nHalf; x <= nHalf; x++) {
						px = c + x;
						py = r + y;

						if (px >= 1 && px<nWidth - 1 && py >= 1 && py<nHeight - 1) {
							nSum += *(imgIn + nWidth*m_nChannels*py + m_nChannels*px + l);
							nCnt++;
						}
					}
				}
				*(imgOut + nWidth*m_nChannels*py + m_nChannels*px + l) = nSum / (float)nCnt;
			} // 채널 이동 끝
		} // 열 이동 끝
	} // 행 이동 끝
}

//--------------------- BINARY FUCTIONS -------------------

void Binary8_to_RGB565(const U8* src, U16* RGB16, int m_nWidth, int m_nHeight, int mode) {
	
	srand(time(NULL));
	int r = rand() % 256;
	int g = rand() % 256;
	int b = rand() % 256;

	int x, y, pos;
	for (y = 0; y < m_nHeight; y++) {
		for (x = 0; x < m_nWidth; x++) {
			pos = x + y * m_nWidth;

			if (src[pos] == 0)
				RGB16[pos] = MAKE_RGB565(0, 0, 0);            // [Black] R:0, G:0, B:0

			else if (mode == 1)	//Gray mode
				RGB16[pos] = MAKE_RGB565(255, 255, 255);   // [White] R:255, G:255, B:255

			else if (mode == 3)	//Color mode
				RGB16[pos] = MAKE_RGB565(r*src[pos] % 256, g*src[pos] % 256, b*src[pos] % 256);   // [White] R:255, G:255, B:255
		}
	}
}

void HSV_to_Binary8(const U8* src1, const U8* src2, const U8* src3, U8* BIN8, int m_nWidth, int m_nHeight) {
	int x, y, pos;

	for (y = 0; y < m_nHeight; y++){
		for (x = 0; x < m_nWidth; x++) {
			pos = x + y*m_nWidth;

			if (src1[pos] == 255 && src2[pos] == 255 && src3[pos] == 255)	BIN8[pos] = 255;
			else	BIN8[pos] = 0;

		}
	}
}


void HSV_to_Binary8_R(const U8* src1, const U8* src2, const U8* src3, U8* BIN8, int m_nWidth, int m_nHeight) {
	int x, y, pos;

	for (y = 0; y < m_nHeight; y++) {
		for (x = 0; x < m_nWidth; x++) {
			pos = x + y*m_nWidth;

			if (src1[pos] == 255 && src2[pos] == 255 && src3[pos] == 255) BIN8[pos] = 0;
			else  BIN8[pos] = 255;
		}
	}
}

//------------------------- TRANSFER FUNCTIONS ----------------------------
/*
void RGB565_to_BGR888(const U16* src, U8* BGR888, int m_nWidth, int m_nHeight) {
	int m_nWStep = m_nWidth * 3;
	int x, y, pos, src_pos;

	int i = 0, j = 0, R = 0, G = 0, B = 0;

	for (y = 0; y < m_nHeight; y += 6) {
		for (x = 0; x < m_nWidth; x += 6) {

			R = 0;
			G = 0;
			B = 0;

			for (i = 0; i < 6; i++) {
				for (j = 0; j < 6; j++) {

					src_pos = *(src + (y + j)*m_nWidth + (x + i));
					R += GetRed565Value(src_pos);
					G += GetGreen565Value(src_pos);
					B += GetBlue565Value(src_pos);
				}
			}

			R = R / 36;
			G = G / 36;
			B = B / 36;

			for (i = 0; i < 6; i++) {
				for (j = 0; j < 6; j++) {
					pos = (x + i)*m_nChannels + (y + j)*m_nWStep;
					BGR888[pos + 0] = B;
					BGR888[pos + 1] = G;
					BGR888[pos + 2] = R;
				}
			}


		}//x
	}//y
}*/



void RGB565_to_BGR888(const U16* src, U8* BGR888, int m_nWidth, int m_nHeight) {
	int x, y, pos, src_pos;

	int i = 0, j = 0, R = 0, G = 0, B = 0;
	int tmp_z = 0;
	for (y = 0; y < m_nHeight; y += M_SIZE) {
		for (x = 0; x < m_nWidth; x += M_SIZE) {

			R = 0;
			G = 0;
			B = 0;

			for (i = 0; i < M_SIZE; i++) {
				for (j = 0; j < M_SIZE; j++) {

					src_pos = *(src + (y + j)*m_nWidth + (x + i));
					R += GetRed565Value(src_pos);
					G += GetGreen565Value(src_pos);
					B += GetBlue565Value(src_pos);
				}
			}

			tmp_z = M_SIZE * M_SIZE;

			R = R / tmp_z;
			G = G / tmp_z;
			B = B / tmp_z;

			pos = (x/M_SIZE)*m_nChannels + (y/M_SIZE)*3*WIDTH;
			BGR888[pos + 0] = B;
			BGR888[pos + 1] = G;
			BGR888[pos + 2] = R;

		}//x
	}//y
}


/*
void RGB565_to_BGR888(const U16* src, U8* BGR888, int m_nWidth, int m_nHeight) {
	int m_nWStep = m_nWidth * 3;
	int x, y, pos, src_pos;

	int i = 0, j = 0, R = 0, G = 0, B = 0;

	for (y = 0; y < m_nHeight; y += 3) {
		for (x = 0; x < m_nWidth; x += 3) {

			R = 0;
			G = 0;
			B = 0;

			for (i = 0; i < 3; i++) {
				for (j = 0; j < 3; j++) {

					src_pos = *(src + (y + j)*m_nWidth + (x + i));
					R += GetRed565Value(src_pos);
					G += GetGreen565Value(src_pos);
					B += GetBlue565Value(src_pos);
				}
			}

			R = R / 9;
			G = G / 9;
			B = B / 9;

			pos = (x / 3)*m_nChannels + (y / 3) * 3 * 60;
			BGR888[pos + 0] = B;
			BGR888[pos + 1] = G;
			BGR888[pos + 2] = R;

		}//x
	}//y
}

*/
//------------------------ HSV fuction -------------------

void VSH888_to_H8_thresh(const U8* src, U8* BIN8, int m_nWidth, int m_nHeight, int HLow, int HHigh) {
	int m_nWStep = m_nWidth*3;
	int x, y, pos;
	int H;

	for (y = 0; y < m_nHeight; y++) {
		for (x = 0; x < m_nWidth; x++) {
			pos = x*m_nChannels + y*m_nWStep;
			H = src[pos + 2];

			if (H <= HHigh && H >= HLow)  BIN8[x + y*m_nWidth] = 255;
			else  BIN8[x + y*m_nWidth] = 0;
		}
	}
}

void VSH888_to_S8_thresh(const U8* src, U8* BIN8, int m_nWidth, int m_nHeight, int SLow, int SHigh) {
	int m_nWStep = m_nWidth * 3;
	int x, y, pos;
	int S;

	for (y = 0; y < m_nHeight; y++) {
		for (x = 0; x < m_nWidth; x++) {
			pos = x*m_nChannels + y*m_nWStep;
			S = src[pos + 1];

			if (S <= SHigh && S >= SLow) BIN8[x + y*m_nWidth] = 255;
			else BIN8[x + y*m_nWidth] = 0;
		}
	}
}

void VSH888_to_V8_thresh(const U8* src, U8* BIN8, int m_nWidth, int m_nHeight, int VLow, int VHigh) {
	int m_nWStep = m_nWidth * 3;
	int x, y, pos;
	int V;

	for (y = 0; y < m_nHeight; y++) {
		for (x = 0; x < m_nWidth; x++) {
			pos = x*m_nChannels + y*m_nWStep;
			V = src[pos + 0];

			if (V <= VHigh && V >= VLow) BIN8[x + y*m_nWidth] = 255;
			else BIN8[x + y*m_nWidth] = 0;
		}
	}
}

//--------------- BGR888 <-> VSH888 30deg ----------------

void BGR888_to_VSH888_30(const U8* src, U8* VSH888, int m_nWidth, int m_nHeight) {
	int m_nWStep = m_nWidth * 3;
	int x, y, pos;
	float vB, vG, vR;
	float vMax, vMin, delta, H, S, V;

	for (y = 0; y < m_nHeight; y++) {
		for (x = 0; x < m_nWidth; x++) {
			pos = x*m_nChannels + y*m_nWStep;
			vB = src[pos + 0] / 255.0;   //B
			vG = src[pos + 1] / 255.0;   //G
			vR = src[pos + 2] / 255.0;   //R

			vMax = MAX(MAX(vR, vG), vB);
			vMin = MIN(MIN(vR, vG), vB);
			delta = vMax - vMin;

			V = vMax;
			VSH888[pos + 0] = V * 255;   // V : 0~1

			if (delta == 0) {
				VSH888[pos + 1] = 0;
				VSH888[pos + 2] = 0;
			}
			else {
				S = delta / vMax;
				VSH888[pos + 1] = S * 255;   // S : 0~1 

				if (vR == vMax)
					H = (vG - vB) / delta;
				else if (vG == vMax)
					H = 2.0 + (vB - vR) / delta;
				else
					H = 4.0 + (vR - vG) / delta;

				H *= 60.0;
				////////////////////////
				H = H + 40;
				if (H < 0)
					H += 360.0;
				else if (H >= 360.0)
					H -= 360.0;

				VSH888[pos + 2] = H / 360 * 255;      // H : 0~360
			}
		}
	}
}
