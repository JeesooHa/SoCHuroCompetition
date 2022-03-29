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

#define BI_RGB        0L

int m_nChannels = 3;

typedef struct {
	U8    rgbBlue;
	U8    rgbGreen;
	U8    rgbRed;
	U8    rgbReserved;
} RGBQUAD;

typedef struct {
	U16   bfType;
	U32   bfSize;
	U16   bfReserved1;
	U16   bfReserved2;
	U32   bfOffBits;
} SAVEBMPFILEHEADER;

typedef struct {
	U32   biSize;
	S32   biWidth;
	S32   biHeight;
	U16   biPlanes;
	U16   biBitCount;
	U32   biCompression;
	U32   biSizeImage;
	U32   biClrImportant;
	U32   biClrUsed;
	S32   biXPelsPerMeter;
	S32   biYPelsPerMeter;
} SAVEBMPINFOHEADER;


void save_fpga_video_data(U16* buf, char* fname)
{
	int m_nWidth = 180;
	int m_nHeight = 120;
	int m_nWStep = ((m_nWidth*m_nChannels*sizeof(U8) + 3)&~3) / sizeof(U8);

	FILE* fp;
	fp = fopen(fname, "wb");

	if (fp != NULL) {
		//windows - 리틀 엔디언 방식
		int szFh = 3 * sizeof(U16) + 2 * sizeof(U32);

		//비트맵 파일 헤더
		SAVEBMPFILEHEADER Fh;
		Fh.bfType = 0x4D42; // 'BM'
		Fh.bfSize = szFh + sizeof(SAVEBMPINFOHEADER) + m_nWStep*m_nHeight + (m_nChannels == 1) * 1024;
		Fh.bfReserved1 = 0;
		Fh.bfReserved2 = 0;
		Fh.bfOffBits = szFh + sizeof(SAVEBMPINFOHEADER) + (m_nChannels == 1) * 256 * sizeof(RGBQUAD);

		fwrite(&Fh.bfType, sizeof(Fh.bfType), 1, fp);
		fwrite(&Fh.bfSize, sizeof(Fh.bfSize), 1, fp);
		fwrite(&Fh.bfReserved1, sizeof(Fh.bfReserved1), 1, fp);
		fwrite(&Fh.bfReserved2, sizeof(Fh.bfReserved2), 1, fp);
		fwrite(&Fh.bfOffBits, sizeof(Fh.bfOffBits), 1, fp);

		//비트맵 정보
		SAVEBMPINFOHEADER Fi;
		Fi.biSize = sizeof(SAVEBMPINFOHEADER);
		Fi.biWidth = m_nWidth;
		Fi.biHeight = m_nHeight;
		Fi.biPlanes = 1;
		Fi.biBitCount = m_nChannels * 8;
		Fi.biCompression = BI_RGB;
		Fi.biSizeImage = m_nWStep*m_nHeight;
		Fi.biXPelsPerMeter = 0;
		Fi.biYPelsPerMeter = 0;
		Fi.biClrUsed = 0;
		Fi.biClrImportant = 0;

		fwrite(&Fi.biSize, sizeof(Fi.biSize), 1, fp);
		fwrite(&Fi.biWidth, sizeof(Fi.biWidth), 1, fp);
		fwrite(&Fi.biHeight, sizeof(Fi.biHeight), 1, fp);
		fwrite(&Fi.biPlanes, sizeof(Fi.biPlanes), 1, fp);
		fwrite(&Fi.biBitCount, sizeof(Fi.biBitCount), 1, fp);
		fwrite(&Fi.biCompression, sizeof(Fi.biCompression), 1, fp);
		fwrite(&Fi.biSizeImage, sizeof(Fi.biSizeImage), 1, fp);
		fwrite(&Fi.biXPelsPerMeter, sizeof(Fi.biXPelsPerMeter), 1, fp);
		fwrite(&Fi.biYPelsPerMeter, sizeof(Fi.biYPelsPerMeter), 1, fp);
		fwrite(&Fi.biClrUsed, sizeof(Fi.biClrUsed), 1, fp);
		fwrite(&Fi.biClrImportant, sizeof(Fi.biClrImportant), 1, fp);

		if (m_nChannels == 1) {	//회색조영상
			int l;
			for (l = 0; l < 256; l++) {
				RGBQUAD GrayPalette = { l, l, l, 0 };
				fwrite(&GrayPalette, sizeof(RGBQUAD), 1, fp);
			}
		}

		U8* BGR32 = (U8*)malloc(m_nHeight*m_nWStep);

		int x, y;
		for (y = 0; y < m_nHeight; y++) {
			for (x = 0; x < m_nWidth; x++) {
				BGR32[x*m_nChannels + m_nWStep*y + 0] = GetBlue565Value(*(buf + y*m_nWidth + x));
				BGR32[x*m_nChannels + m_nWStep*y + 1] = GetGreen565Value(*(buf + y*m_nWidth + x));
				BGR32[x*m_nChannels + m_nWStep*y + 2] = GetRed565Value(*(buf + y*m_nWidth + x));
			}
		}

		for (y = m_nHeight - 1; y >= 0; y--)
			fwrite(&BGR32[m_nWStep*y], m_nWStep, 1, fp);

		fclose(fp);

		//printf("save_fpga_video_data END\n");
	}
}

//-------------------- Filtering -----------------------

void MeanFiltering(const U8* imgIn, U8* imgOut, int nWidth, int nHeight)
{
	int m_nMeanSize = 3;
	int nChnnl = 3;
	int nHalf = m_nMeanSize / 2;
	int px = 0, py = 0;
	int r, c, l, x, y;

	for (l = 0; l<nChnnl; l++){ // 채널 이동
		for (r = 0; r<nHeight; r++){ // 행 이동
			for (c = 0; c<nWidth; c++){ // 열 이동

				int nSum = 0;  // 픽셀 값의 합
				int nCnt = 0; // 픽셀 수
				for (y = -nHalf; y <= nHalf; y++){
					for (x = -nHalf; x <= nHalf; x++){
						px = c + x;
						py = r + y;

						if (px >= 1 && px<nWidth-1 && py >= 1 && py<nHeight-1) {		
							nSum += *(imgIn + nWidth*nChnnl*py + nChnnl*px + l);
							nCnt++;
						}
					}
				}
				*(imgOut + nWidth*nChnnl*py + nChnnl*px + l) = nSum / (float)nCnt;
			} // 채널 이동 끝
		} // 열 이동 끝
	} // 행 이동 끝

	//printf("Mean Filtering end \n");
}

//--------------------- BINARY FUCTIONS -------------------

void Binary8_to_RGB565(const U8* src, U16* RGB16, int m_nWidth, int m_nHeight, int mode) {
	int m_nGChannels = 1;
	int m_nGWStep = ((m_nWidth*m_nGChannels * sizeof(U8) + 3)&~3) / sizeof(U8);

	srand(time(NULL));
	int r = rand() % 256;
	int g = rand() % 256;
	int b = rand() % 256;

	int x, y, pos;
	for (y = 0; y < m_nHeight; y++) {
		for (x = 0; x < m_nWidth; x++) {
			pos = x*m_nGChannels + y*m_nGWStep;
			if (src[pos] == 0)
				RGB16[x + y*m_nWidth] = MAKE_RGB565(0, 0, 0);            // [Black] R:0, G:0, B:0
			else if (mode == 1)
				RGB16[x + y*m_nWidth] = MAKE_RGB565(255, 255, 255);   // [White] R:255, G:255, B:255
			else if (mode == 3)
				RGB16[x + y*m_nWidth] = MAKE_RGB565(r*src[pos] % 256, g*src[pos] % 256, b*src[pos] % 256);   // [White] R:255, G:255, B:255
		}
	}

	//printf("Binary8_to_RGB565 END\n"); 
}

void HSV_to_Binary8(const U8* src1, const U8* src2, const U8* src3, U8* BIN8, int m_nWidth, int m_nHeight) {
	int x, y, pos;

	for (y = 0; y < m_nHeight; y++) {
		for (x = 0; x < m_nWidth; x++) {
			pos = x + y*m_nWidth;

			if (src1[pos] == 255 && src2[pos] == 255 && src3[pos] == 255)
				BIN8[x + y*m_nWidth] = 255;
			else
				BIN8[x + y*m_nWidth] = 0;
		}
	}
	//printf("HSV_to_Binary8 END\n");
}


void HSV_to_Binary8_R(const U8* src1, const U8* src2, const U8* src3, U8* BIN8, int m_nWidth, int m_nHeight) {
	int x, y, pos;

	for (y = 0; y < m_nHeight; y++) {
		for (x = 0; x < m_nWidth; x++) {
			pos = x + y*m_nWidth;

			if (src1[pos] == 255 && src2[pos] == 255 && src3[pos] == 255)
				BIN8[x + y*m_nWidth] = 0;
			else
				BIN8[x + y*m_nWidth] = 255;
		}
	}
	
	//printf("HSV_to_Binary8_R END\n");
}

//------------------------- TRANSFER FUNCTIONS ----------------------------

void RGB565_to_BGR888(const U16* src, U8* BGR888, int m_nWidth, int m_nHeight) {
	int m_nChannels = 3;
	int m_nWStep = ((m_nWidth*m_nChannels * sizeof(U8) + 3)&~3) / sizeof(U8);

	int x, y;
	for (y = 0; y < m_nHeight; y++) {
		for (x = 0; x < m_nWidth; x++) {
			BGR888[x*m_nChannels + y*m_nWStep + 0] = GetBlue565Value(*(src + y*m_nWidth + x));
			BGR888[x*m_nChannels + y*m_nWStep + 1] = GetGreen565Value(*(src + y*m_nWidth + x));
			BGR888[x*m_nChannels + y*m_nWStep + 2] = GetRed565Value(*(src + y*m_nWidth + x));
		}
	}
	//printf("RGB565_to_BGR888 END\n");
}





//------------------------ HSV fuction -------------------

void VSH888_to_H8_thresh(const U8* src, U8* BIN8, int m_nWidth, int m_nHeight, int HLow, int HHigh) {
	int m_nChannels = 3;
	int m_nWStep = ((m_nWidth*m_nChannels * sizeof(U8) + 3)&~3) / sizeof(U8);

	int x, y, pos;
	int H;

	//printf("H8_thresh : %d, %d \n", HLow, HHigh);

	for (y = 0; y < m_nHeight; y++) {
		for (x = 0; x < m_nWidth; x++) {
			pos = x*m_nChannels + y*m_nWStep;
			H = src[pos+ 2];

			if (H <= HHigh && H >= HLow)
				BIN8[x + y*m_nWidth] = 255;
			else
				BIN8[x + y*m_nWidth] = 0;
		}
	}

	//printf("VSH888_to_H8_thresh END\n");
}

void VSH888_to_S8_thresh(const U8* src, U8* BIN8, int m_nWidth, int m_nHeight, int SLow, int SHigh) {
	int m_nChannels = 3;
	int m_nBChannels = 1;
	int m_nWStep = ((m_nWidth*m_nChannels * sizeof(U8) + 3)&~3) / sizeof(U8);

	int x, y, pos;
	int S;

	//printf("S8_thresh : %d, %d \n", SLow, SHigh);

	for (y = 0; y < m_nHeight; y++) {
		for (x = 0; x < m_nWidth; x++) {
			pos = x*m_nChannels + y*m_nWStep;
			S = src[pos + 1];

			if (S <= SHigh && S >= SLow)
				BIN8[x*m_nBChannels + y*m_nWidth] = 255;
			else
				BIN8[x*m_nBChannels + y*m_nWidth] = 0;
		}
	}

	//printf("VSH888_to_S8_thresh END\n");
}

void VSH888_to_V8_thresh(const U8* src, U8* BIN8, int m_nWidth, int m_nHeight, int VLow, int VHigh) {
	int m_nChannels = 3;
	int m_nBChannels = 1;
	int m_nWStep = ((m_nWidth*m_nChannels * sizeof(U8) + 3)&~3) / sizeof(U8);

	int x, y, pos;
	int V;

	//printf("V8_thresh : %d, %d \n", VLow, VHigh);

	for (y = 0; y < m_nHeight; y++) {
		for (x = 0; x < m_nWidth; x++) {
			pos = x*m_nChannels + y*m_nWStep;
			V = src[pos + 0];

			if (V <= VHigh && V >= VLow)
				BIN8[x*m_nBChannels + y*m_nWidth] = 255;
			else
				BIN8[x*m_nBChannels + y*m_nWidth] = 0;
		}
	}

	//printf("VSH888_to_V8_thresh END\n");
}

//--------------- BGR888 <-> VSH888 30deg ----------------

void BGR888_to_VSH888_30(const U8* src, U8* VSH888 , int m_nWidth, int m_nHeight) {

	//printf("BGR888_to_VSH888_30 START\n");

	int m_nChannels = 3;
	int m_nWStep = ((m_nWidth*m_nChannels * sizeof(U8) + 3)&~3) / sizeof(U8);

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
				else if(H>=360.0)
					H -= 360.0;

				VSH888[pos + 2] = H / 360 * 255;      // H : 0~360
			}

		}
	}

	//printf("BGR888_to_VSH888_30 END\n");

}
