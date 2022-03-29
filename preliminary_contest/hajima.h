#ifndef __AMAZON2_HAJIMA_H__
#define __AMAZON2_HAJIMA_H__

#include "amazon2_sdk.h"

#define GetRed565Value(C)	((((C >> 11) & 0x1F) * 527) + 23) >> 6
#define GetGreen565Value(C)	((((C >> 5) & 0x3F) * 259) + 33) >> 6
#define GetBlue565Value(C)	(((C & 0x1F) * 527) + 23) >> 6

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

struct Label
{
	long sumX;
	long sumY;
	int cX;
	int cY;
	int pix;
	int maxX;
	int minX;
	int maxY;
	int minY;
};

// Filter
void MeanFiltering(const U8* imgIn, U8* imgOut, int nWidth, int nHeight);

//Binary
void Binary8_to_RGB565(const U8* src, U16* RGB16, int m_nWidth, int m_nHeight, int mode);
void HSV_to_Binary8(const U8* src1, const U8* src2, const U8* src3, U8* BIN8, int m_nWidth, int m_nHeight);

//RBG Transfer
void RGB565_to_BGR888(const U16* src, U8* BGR888, int m_nWidth, int m_nHeight);
void BGR888_to_VSH888_30(const U8* src, U8* VSH888, int m_nWidth, int m_nHeight);

//HSV functions
void VSH888_to_H8_thresh(const U8* src, U8* BIN8, int m_nWidth, int m_nHeight, int HLow, int HHigh);
void VSH888_to_S8_thresh(const U8* src, U8* BIN8, int m_nWidth, int m_nHeight, int SLow, int SHigh);
void VSH888_to_V8_thresh(const U8* src, U8* BIN8, int m_nWidth, int m_nHeight, int VLow, int VHigh);

//Image Reverse
void HSV_to_Binary8_R(const U8* src1, const U8* src2, const U8* src3, U8* BIN8, int m_nWidth, int m_nHeight);

//save images
void save_fpga_video_data(U16* buf, char* fname);

#endif //__AMAZON2_GRAPHIC_API_H__
