////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
/////MDF 색 다시 맞추기, 멀리있을때 그림자 조금만 있으면 안보임
/////line find개선


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <math.h>
#include "amazon2_sdk.h"
#include "graphic_api.h"
#include "hajima.h"

//////////////// robot /////////////////
#include <getopt.h>
#include "uart_api.h"
#include "robot_protocol.h"
#include <termios.h>
static struct termios inittio, newtio;
void init_console(void)
{
	tcgetattr(0, &inittio);
	newtio = inittio;
	newtio.c_lflag &= ~ICANON;
	newtio.c_lflag &= ~ECHO;
	newtio.c_lflag &= ~ISIG;
	newtio.c_cc[VMIN] = 1;
	newtio.c_cc[VTIME] = 0;
	cfsetispeed(&newtio, B115200);
	tcsetattr(0, TCSANOW, &newtio);
}
//////////////////////////////////////////
//화면크기
#define m_nWidth 180
#define m_nHeight 120

//화면 출력 모드
#define B_GRAY_MODE 1
#define B_COLOR_MODE 3

/////////// 각 색깔별 HSV 값 ////////////
//빨강
#define RED_HL 0   
#define RED_HH 50
#define RED_SL 0  
#define RED_SH 255      
#define RED_VL 0
#define RED_VH 255

//주황 - 골프공
#define BALL_HL 0   
#define BALL_HH 50
#define BALL_SL 120
#define BALL_SH 255
#define BALL_VL 0
#define BALL_VH 255

//골 나무
#define GOAL_HL 0   
#define GOAL_HH 70
#define GOAL_SL 50
#define GOAL_SH 255
#define GOAL_VL 40
#define GOAL_VH 255

//노랑
#define YELLO_HL 50   
#define YELLO_HH 100
#define YELLO_SL 0//50  //0  
#define YELLO_SH 255
#define YELLO_VL 50//100 //120   
#define YELLO_VH 255

//노랑 - GATE
#define GATE_HL 50   
#define GATE_HH 100
#define GATE_SL 60  //0  
#define GATE_SH 255
#define GATE_VL 50//100 //120   
#define GATE_VH 255

/*//초록	5공
#define GREEN_HL 60      
#define GREEN_HH 150
#define GREEN_SL 0//0  
#define GREEN_SH 255      
#define GREEN_VL 70		//100		// 0
#define GREEN_VH 255 
*/
//초록	112
#define GREEN_HL 70      
#define GREEN_HH 150
#define GREEN_SL 50//0  
#define GREEN_SH 255      
#define GREEN_VL 70		//100		// 0
#define GREEN_VH 255 

//파랑
#define BLUE_HL 150
#define BLUE_HH 255
#define BLUE_SL 100
#define BLUE_SH 255     
#define BLUE_VL 0   
#define BLUE_VH 255

/*//검정	5공
#define BLK_HL 0	//0
#define BLK_HH 255
#define BLK_SL 0
#define BLK_SH 170	//70
#define BLK_VL 0	//10
#define BLK_VH 60	//90	//70
*/
/*
//검정	112인줄 알았는데 초록색나옴
#define BLK_HL 10	//0
#define BLK_HH 255
#define BLK_SL 0
#define BLK_SH 150	//70
#define BLK_VL 0	//10
#define BLK_VH 100	//90	//70
*/
//검정	112다시
#define BLK_HL 10	//0
#define BLK_HH 255
#define BLK_SL 0
#define BLK_SH 100	//70
#define BLK_VL 0	//10
#define BLK_VH 90	//90	//70


//흰
#define WHT_HL 0
#define WHT_HH 255
#define WHT_SL 0
#define WHT_SH 50 //70
#define WHT_VL 180 //100
#define WHT_VH 255

///////////// 미션 ///////////////////
typedef enum {
	END = 0, M1, M2, M3, M4, M5, M6, M7,
	M8,	//파란게이트 1
	M9  //파란게이트 2
} MISSION;


void MISSION_1(void);
void MISSION_2(void);
void MISSION_4(void);
void MISSION_5R(void);
void MISSION_6(void);

//필요한 색깔과 방향을 사용하여 경기장 라인으로 정렬하는 함수
#define LP_HEAD_RIGHT 2
#define LP_HEAD_FRONT 3
#define LP_COLOR_BLACK 4
#define LP_COLOR_WHITE 5
#define JAL 10
#define DAECHUNG 20

void COLOR_LINE_UP(int head_mode, int color_set, int level);
void COLOR_LINE_UP2(int head_mode, int color_set, int level);

//필요한 색깔 영역에서 중심점과 픽셀 갯수 얻는 함수
void INFO_CHECK(
	U16* fpga_src, U16* fpga_dst1, U16* fpga_dst2, U16* fpga_dst3, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix	//OUPUT
	);

void INFO_GOAL_CHECK(
	U16* fpga_src, U16* fpga_dst1, U16* fpga_dst2, U16* fpga_dst3, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix	//OUPUT
	);

//추세선으로 라인 정렬할 정보 찾는 함수
void INFO_TREND_LINE(
	U16* fpga_src, U16* fpga_dst1, U16* fpga_dst2, U16* fpga_dst3, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix, float* G, int mode 	//OUPUT
	);

void INFO_MEAN_VAR(
	U16* fpga_src, U16* fpga_dst1, U16* fpga_dst2, U16* fpga_dst3, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix, float* mean, float* std_dev 	//OUPUT
	);

int main(int argc, char **argv)
{
	////////////////// Robot Init /////////////////////
	int ret;
	init_console();
	ret = uart_open();
	if (ret < 0) return EXIT_FAILURE;
	uart_config(UART1, 4800, 8, UART_PARNONE, 1);
	////////////////////////////////////////////////////

	BOOL b_loop = TRUE;

	if (open_graphic() < 0) return -1;
	if (direct_camera_display_stat() > 0)	direct_camera_display_off();

	clear_screen();
	flip();
	clear_screen();

	int JJ_NUM = 0;			//JJWALKING 걸음수
	int flag_state = M1;	//시작하는 미션 설정
	double cnt;
	while (b_loop) {
		switch (flag_state) {
	
		case 1:												//미션1. 직각-바리게이트

			MISSION_1(); //게이트가 없을 경우 지나가기 까지, 없어짐 확인까지
			
			GO_GREEN(11);	//GO_GREEN(10);
			GO_JONGJONG(4);
	
			JJ_NUM = 10;
			while (JJ_NUM--) { JJWALKING(); }

			flag_state = M2;
			break;

		case 2:				//미션2. 빨간 다리 건너기 - 오르기/내리기시 바닥면 접촉 허용

			MISSION_2();	//빨간색 보면서 좌우 정렬까지
			
			JJ_NUM = 3;
			while (JJ_NUM--) { JJWALKING(); }

			TUCK();	//다리위에서 덤블링

			GO_GREEN(3);
			GO_JONGJONG(4);

			JJ_NUM = 5;
			while (JJ_NUM--) { JJWALKING(); }

			flag_state = M3;
			break;

		case 3:								//미션3. 허들 장애물

			NORMAL_FORM();
		//	cnt = 500000;
		// while (cnt--) {}

			HUDDLE();			//허들 덤블링		

			GO_GREEN(4);
								
			BIG_TURN_LEFT();	//90도 턴		
			BIG_TURN_LEFT();
			//BIG_TURN_LEFT();

			GO_GREEN(5);

			COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK,JAL);		//검정색으로 중심 맞춤

			GO_GREEN(13);	//GO_GREEN(16);
			GO_JONGJONG(11);	//GO_JONGJONG(8);

			flag_state = M4;
			break;

		case 4:				//미션4. 초록색 다리건너기
			MISSION_4();	//검정색으로 내려가는 조건 검사까지

			GO_GREEN(4);	//내려갈 위치로 이동
			//GO_JONGJONG(1);
			//JJWALKING(); //JJWALKING(); JJWALKING();
			//G_TURN_RIGHT();
			DOWN_STAIR(); 

			COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK,JAL);

			GO_GREEN(7);

			flag_state = M5;
			break;

		case 5:			//미션5. 골프공 차기

		    MISSION_5R(); 	//주황색 공 대강 보기 ====> 골과 공 위치 탐색 ====> 킥까지

			BIG_TURN_LEFT();	//90도
			BIG_TURN_LEFT();
			//BIG_TURN_LEFT();
		
			COLOR_LINE_UP2(LP_HEAD_RIGHT, LP_COLOR_BLACK, JAL);
			
			GO_GREEN(7);	

			COLOR_LINE_UP2(LP_HEAD_RIGHT, LP_COLOR_BLACK,JAL);

			GO_GREEN(14);
			
			GO_JONGJONG(11);

			JJ_NUM = 5;
			while (JJ_NUM--) { JJWALKING(); }

			flag_state = M6;
			break;

		case 6:			//미션6. 노란색 함정 다리 건너기 - 오르기/내리기시 바닥면 접촉 허용

			MISSION_6();	//덤블링 가능 위치까지 이동, 기본자세까지

			YELLO_TUCK();	//덤블링

			//COLOR_LINE_UP(LP_HEAD_RIGHT, LP_COLOR_BLACK,DAECHUNG);

			flag_state = M7;
			break;

		case 7:	//미션7. 회전-바리게이트
			GO_GREEN(6);

			MISSION_1();	//노란색 인식 될 때 까지 기다리기 - 카메라 스캔 노란색 인식 - 카메라 멈춤
		
			GO_GREEN(10);

			flag_state = END;
			break;

		case 0:
			b_loop = FALSE;
			break;

		}//switch
	}//while

	close_graphic();
	uart_close();	//robot

	return 0;
}


//111111111111111111111111111111111111111111111111111111111111111111111111111111111
void MISSION_1(void)		//미션1. 직각-바리게이트, //미션7. 회전-바리게이트						
{
	U16* fpga_src = (U16*)malloc(m_nWidth * m_nHeight * 2); //FPGA Original image
	U16* fpga_dst1 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst2 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst3 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U8* imgBGR = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* Filt = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgVSH = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgThresholded_H = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_S = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_V = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded = (U8*)malloc(m_nWidth * m_nHeight);
	U8* labelOUT = (U8*)malloc(m_nHeight * m_nWidth);
	U8* MAXlabelOUT = (U8*)malloc(m_nHeight * m_nWidth);

	int Y_cx = 0, Y_cy = 0, Y_pix = 0;
	int Y_base = 500;									//실험값

	//노란색 영역 찾기 - 게이트가 없을 경우 기다림
	while (Y_pix < Y_base) {
		INFO_CHECK(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			GATE_HL, GATE_HH, GATE_SL, GATE_SH, GATE_VL, GATE_VH, //INPUT
			&Y_cx, &Y_cy, &Y_pix	//OUPUT
			);
	}//while

	//노란색 영역 찾기 - 게이트가 있을경우 기다림
	while (Y_pix >= Y_base) {
		INFO_CHECK(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			GATE_HL, GATE_HH, GATE_SL, GATE_SH, GATE_VL, GATE_VH, //INPUT
			&Y_cx, &Y_cy, &Y_pix	//OUPUT
			);
	}//while

}

//222222222222222222222222222222222222222222222222222222222222222222222222
void MISSION_2(void)		//미션2. 빨간 다리 건너기 - 오르기/내리기시 바닥면 접촉 허용					
{
	U16* fpga_src = (U16*)malloc(m_nWidth * m_nHeight * 2); //FPGA Original image
	U16* fpga_dst1 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst2 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst3 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U8* imgBGR = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* Filt = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgVSH = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgThresholded_H = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_S = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_V = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded = (U8*)malloc(m_nWidth * m_nHeight);
	U8* labelOUT = (U8*)malloc(m_nHeight * m_nWidth);
	U8* MAXlabelOUT = (U8*)malloc(m_nHeight * m_nWidth);

	int R_cx = 0, R_cy = 0, R_pix = 0;

	while (1) {

		HEAD_4RED();
		double cnt = 200000;
		while (cnt--) {}

		INFO_CHECK(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			RED_HL, RED_HH, RED_SL, RED_SH, RED_VL, RED_VH, //INPUT
			&R_cx, &R_cy, &R_pix	//OUPUT
			);	

		int d = 2;		
		int G_RD = 93;
		int G_LD = 87;

		//빨간색 중심 맞추기
		if (R_cx > G_RD) {			
						
			if (R_cx > G_RD + 9 * d) GO_RIGHT();
			if (R_cx > G_RD + 8 * d) { GO_RIGHT(); JJWALKING(); }
			if (R_cx > G_RD + 7 * d) { GO_RIGHT(); JJWALKING(); }
			if (R_cx > G_RD + 6 * d) { GO_RIGHT(); JJWALKING(); }
			if (R_cx > G_RD + 5 * d) { GO_RIGHT(); JJWALKING(); }
			if (R_cx > G_RD + 4 * d) { GO_RIGHT_17(); JJWALKING(); }
			if (R_cx > G_RD + 3 * d) { GO_RIGHT_17(); JJWALKING(); }
			if (R_cx > G_RD + 2 * d) { GO_RIGHT_17(); JJWALKING(); }
			if (R_cx > G_RD + 1 * d) { GO_RIGHT_17(); JJWALKING(); }
			GO_RIGHT_17(); JJWALKING(); JJWALKING(); JJWALKING();
		}
		else if (R_cx < G_LD) {
			
			if (R_cx < G_LD - 9 * d) GO_LEFT();
			if (R_cx < G_LD - 8 * d) { GO_LEFT(); JJWALKING(); }
			if (R_cx < G_LD - 7 * d) { GO_LEFT(); JJWALKING(); }
			if (R_cx < G_LD - 6 * d) { GO_LEFT(); JJWALKING(); }
			if (R_cx < G_LD - 5 * d) { GO_LEFT(); JJWALKING(); }
			if (R_cx < G_LD - 4 * d) { GO_LEFT_17(); JJWALKING(); }
			if (R_cx < G_LD - 3 * d) { GO_LEFT_17(); JJWALKING(); }
			if (R_cx < G_LD - 2 * d) { GO_LEFT_17(); JJWALKING(); }
			if (R_cx < G_LD - 1 * d) { GO_LEFT_17(); JJWALKING(); }
			GO_LEFT_17(); JJWALKING(); JJWALKING(); JJWALKING();
		}
		else { break;}
	}//while
	
}

//444444444444444444444444444444444444444444444444444444444444444444444444444444444
void MISSION_4(void)	//미션4. 초록색 다리건너기
{
	U16* fpga_src = (U16*)malloc(m_nWidth * m_nHeight * 2); //FPGA Original image
	U16* fpga_dst1 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst2 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst3 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U8* imgBGR = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* Filt = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgVSH = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgThresholded_H = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_S = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_V = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded = (U8*)malloc(m_nWidth * m_nHeight);
	U8* labelOUT = (U8*)malloc(m_nHeight * m_nWidth);
	U8* MAXlabelOUT = (U8*)malloc(m_nHeight * m_nWidth);

	int G_cx = 0, G_cy = 0, G_pix = 0;
	int tmp = 0;
	
	//초록색 영역 찾기 - 오르기 전 가운데 정렬
	while (1) {

		HEAD_4GREEN();
		double cnt = 250000;
		while (cnt--) {}

		INFO_CHECK(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			GREEN_HL, GREEN_HH, GREEN_SL, GREEN_SH, GREEN_VL, GREEN_VH, //INPUT
			&G_cx, &G_cy, &G_pix	//OUPUT
			);
	
		//초록색 중심 맞추기
		int d = 2;
		int G_RD = 93;
		int G_LD = 87;

		//초록색 중심 맞추기
		if (G_cx > G_RD) {

			if (G_cx > G_RD + 9 * d) GO_RIGHT();
			if (G_cx > G_RD + 8 * d) { GO_RIGHT(); JJWALKING(); }
			if (G_cx > G_RD + 7 * d) { GO_RIGHT(); JJWALKING(); }
			if (G_cx > G_RD + 6 * d) { GO_RIGHT(); JJWALKING(); }
			if (G_cx > G_RD + 5 * d) { GO_RIGHT(); JJWALKING(); }
			if (G_cx > G_RD + 4 * d) { GO_RIGHT_17(); JJWALKING(); }
			if (G_cx > G_RD + 3 * d) { GO_RIGHT_17(); JJWALKING(); }
			if (G_cx > G_RD + 2 * d) { GO_RIGHT_17(); JJWALKING(); }
			if (G_cx > G_RD + 1 * d) { GO_RIGHT_17(); JJWALKING(); }
			GO_RIGHT_17(); JJWALKING(); JJWALKING(); JJWALKING();
		}
		else if (G_cx < G_LD) {

			if (G_cx < G_LD - 9 * d) GO_LEFT();
			if (G_cx < G_LD - 8 * d) { GO_LEFT(); JJWALKING(); }
			if (G_cx < G_LD - 7 * d) { GO_LEFT(); JJWALKING(); }
			if (G_cx < G_LD - 6 * d) { GO_LEFT(); JJWALKING(); }
			if (G_cx < G_LD - 5 * d) { GO_LEFT(); JJWALKING(); }
			if (G_cx < G_LD - 4 * d) { GO_LEFT_17(); JJWALKING(); }
			if (G_cx < G_LD - 3 * d) { GO_LEFT_17(); JJWALKING(); }
			if (G_cx < G_LD - 2 * d) { GO_LEFT_17(); JJWALKING(); }
			if (G_cx < G_LD - 1 * d) { GO_LEFT_17(); JJWALKING(); }
			GO_LEFT_17(); JJWALKING(); JJWALKING(); JJWALKING();
		}
		else { break; }
		
		/*
		int d = 3;
		int G_RD = 96;
		int G_LD = 86;
		
		if (G_cx > G_RD) {
			if (G_cx > G_RD + 12 * d) { GO_RIGHT(); }
			if (G_cx > G_RD + 11 * d) { GO_RIGHT(); JJWALKING(); JJWALKING(); }
			if (G_cx > G_RD + 10 * d) { GO_RIGHT(); }
			if (G_cx > G_RD + 9 * d) { GO_RIGHT();	JJWALKING(); JJWALKING();}
			if (G_cx > G_RD + 8 * d) { GO_RIGHT(); }
			if (G_cx > G_RD + 7 * d) { GO_RIGHT();	JJWALKING(); JJWALKING();}
			if (G_cx > G_RD + 6 * d) { GO_RIGHT(); }
			if (G_cx > G_RD + 5 * d) { GO_RIGHT_17();	JJWALKING(); JJWALKING();}
			if (G_cx > G_RD + 4 * d) { GO_RIGHT_17(); }
			if (G_cx > G_RD + 3 * d) { GO_RIGHT_17();	JJWALKING(); JJWALKING();}
			if (G_cx > G_RD + 2 * d) { GO_RIGHT_17(); }
			if (G_cx > G_RD + d) { GO_RIGHT_5();	JJWALKING(); JJWALKING(); }
			GO_RIGHT_5();
			tmp = 4;
			while (tmp--) { JJWALKING(); }
		}
		else if (G_cx < G_LD) {
			if (G_cx < G_LD - 12 * d) { GO_LEFT(); }
			if (G_cx < G_LD - 11 * d) { GO_LEFT(); JJWALKING(); JJWALKING(); }
			if (G_cx < G_LD - 10 * d) { GO_LEFT(); }
			if (G_cx < G_LD - 9 * d) { GO_LEFT(); JJWALKING(); JJWALKING();}
			if (G_cx < G_LD - 8 * d) { GO_LEFT(); }
			if (G_cx < G_LD - 7 * d) { GO_LEFT(); JJWALKING(); JJWALKING();}
			if (G_cx < G_LD - 6 * d) { GO_LEFT(); }
			if (G_cx < G_LD - 5 * d) { GO_LEFT_17(); JJWALKING(); JJWALKING();}
			if (G_cx < G_LD - 4 * d) { GO_LEFT_17(); }
			if (G_cx < G_LD - 3 * d) { GO_LEFT_17(); JJWALKING(); JJWALKING();}
			if (G_cx < G_LD - 2 * d) { GO_LEFT_17();  }
			if (G_cx < G_LD - 1 * d) { GO_LEFT_5();  JJWALKING(); JJWALKING(); }
			GO_LEFT_5();
			tmp = 4;
			while (tmp--) { JJWALKING(); }
		}
		else {	break;	}
		*/
	}//while


	UP_STAIR();	//계단 오르기
	GO_GREEN(3);
	

	int num_run = 2;	//최소 초록색 다리 가는 수

	//초록색 영역 찾기 - 초록색 다리 위에서 중심 찾으며 가기
	while (1) {	

		HEAD_4GREEN();
		double cnt = 250000;
		while (cnt--) {}

		float mean = 0, std_dev = 0;

		INFO_MEAN_VAR(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			GREEN_HL, GREEN_HH, GREEN_SL, GREEN_SH, GREEN_VL, GREEN_VH, //INPUT
			&G_cx, &G_cy, &G_pix, &mean, &std_dev	//OUPUT
			);
	
		/////104 ~ 94 ~ 84
		////89-4	   82-4
		float d = 7;// 4;// 5;
		if (num_run <= 0){
			if (G_pix > 500) {
				if(std_dev > 2){ //4
					if (mean < 89 - 1.5){
						G_TURN_LEFT();
						if (mean < 89 - 15) G_TURN_LEFT();
					}
					else if (mean > 89 + 1.5){
						G_TURN_RIGHT(); 
						if (mean > 89 + 15) G_TURN_RIGHT(); 
					}	
				}
		
				if (mean < 89 - 4) {
					GO_LEFT_17();
					if (mean < 89 - 5 - 1 * d) GO_LEFT_17();
					if (mean < 89 - 5 - 2 * (d + 1)) GO_LEFT_17();
				}
				else if (mean>89 + 4) {
					GO_RIGHT_17();
					if (mean > 89 + 5 + 1 * d) GO_RIGHT_17();
					if (mean > 89 + 5 + 2 * (d + 1)) GO_RIGHT_17();
				}
				else {
					if (G_cy > 90)	GO_GREEN(4);
					else	GO_GREEN(5);
				}
			}
			else break;
		}
		else {
			if (std_dev > 2) {
				if (mean < 89 - 1.5) {
					G_TURN_LEFT();
					if (mean < 89 - 15) G_TURN_LEFT();
				}
				else if (mean > 89 + 1.5) {
					G_TURN_RIGHT();
					if (mean > 89 + 15) G_TURN_RIGHT();
				}
				else JJWALKING();
			}
			
			
			if (mean < 89 - 4) {
				GO_LEFT_17();
				if (mean < 89 - 5 - 1 * d) GO_LEFT_17();
				if (mean < 89 - 5 - 2 * (d + 1)) GO_LEFT_17();
			}
			else if (mean>89 + 4) {
				GO_RIGHT_17();
				if (mean > 89 + 5 + 1 * d) GO_RIGHT_17();
				if (mean > 89 + 5 + 2 * (d + 1)) GO_RIGHT_17();
			}
			else {
				num_run--;
				GO_GREEN(5);
			}
			
		}


		/*
		if (num_run <= 0){
			if (G_pix > 500) {
				if(std_dev > 2){ //4
					if (mean < 89 - 1.5){
						G_TURN_LEFT();
						if (mean < 89 - 15) G_TURN_LEFT();
					}
					else if (mean > 89 + 1.5){
						G_TURN_RIGHT(); 
						if (mean > 89 + 15) G_TURN_RIGHT(); 
					}	
				}
				else {
					if (mean < 89 - 4.5) {
						GO_LEFT_17();
						if (mean < 84.5 - 1 * d) GO_LEFT_17();
						if (mean < 84.5 - 2 * d) GO_LEFT_17();
					}
					else if (mean>89 + 4.5) {
						GO_RIGHT_17();
						if (mean > 93.5 + 1 * d) GO_RIGHT_17();
						if (mean > 93.5 + 2 * d) GO_RIGHT_17();
					}
					else {
						if (G_cy > 90)	GO_GREEN(4);
						else	GO_GREEN(5);
					}
				}
			
			}else break;
		}
		else {
			if (std_dev > 2) {	
				if (mean < 89 - 1.5){
					G_TURN_LEFT();
					if (mean < 89 - 15) G_TURN_LEFT(); 
				}
				else if (mean > 89 + 1.5){
					G_TURN_RIGHT(); 
					if (mean > 89 + 15) G_TURN_RIGHT(); 
				}
				else JJWALKING();
			}
			else {
				if (mean < 89 - 4.5) {
					GO_LEFT_17();
					if(mean < 84.5 - 1 * d ) GO_LEFT_17();
					if(mean < 84.5 - 2 * d) GO_LEFT_17();
				}
				else if (mean>89 + 4.5){
					GO_RIGHT_17();
					if (mean > 93.5 + 1 * d) GO_RIGHT_17();
					if (mean > 93.5 + 2 * d) GO_RIGHT_17();
				}
				else {
					num_run--;
					GO_GREEN(5);
				}
			}
		}*/

	}//while


	//printf("\n##############GREEN END =========> BLACK ################\n");
	float G = 0;
	int B_cx = 0, B_cy = 0, B_pix = 0;

	//검정색 선 찾기
	while (1) {	

		HEAD_4BLACK();
		double cnt = 200000;
		while (cnt--) {}

		INFO_TREND_LINE(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			BLK_HL, BLK_HH, BLK_SL, BLK_SH, BLK_VL, BLK_VH, //INPUT
			&B_cx, &B_cy, &B_pix, &G, LP_HEAD_FRONT	//OUPUT
			);
	
		double M = atan2((double)G, 1) * 180 / 3.14159265;
			
		if (M > 2)  G_TURN_LEFT(); 
		else if (M < -2)	G_TURN_RIGHT(); 
		else break;
	}//while

}

//555555555555555555555555555555555555555555555555555555555555555555555555555555555
void MISSION_5R(void)	//미션5. 골프공 차기
{
	U16* fpga_src = (U16*)malloc(m_nWidth * m_nHeight * 2); //FPGA Original image
	U16* fpga_dst1 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst2 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst3 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U8* imgBGR = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* Filt = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgVSH = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgThresholded_H = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_S = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_V = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded = (U8*)malloc(m_nWidth * m_nHeight);
	U8* labelOUT = (U8*)malloc(m_nHeight * m_nWidth);
	U8* MAXlabelOUT = (U8*)malloc(m_nHeight * m_nWidth);

	int BALL_cx = 0, BALL_cy = 0, BALL_pix = 0;
	int GOAL_cx = 0, GOAL_cy = 0, GOAL_pix = 0;
	int flag = 0;

	//골프공 인식해서 가까이 가기
	while (1) {	//미션 처음 공 영역 찾기

		HEAD_4GREEN();
		double cnt = 200000;
		while (cnt--) {}
		INFO_CHECK(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			BALL_HL, BALL_HH, BALL_SL, BALL_SH, BALL_VL, BALL_VH, //INPUT
			&BALL_cx, &BALL_cy, &BALL_pix	//OUPUT
			);

		//머리 각도 조절

		/*
		if (BALL_cy == 0 && BALL_cx == 0 && flag == 0) {
			GO_GREEN(1);
			flag = 1;
		}
		else if (BALL_cy < 50) { GO_GREEN(5); }
		else if (BALL_cy < 60) { GO_GREEN(4); }
		else if (BALL_cy < 70) { GO_GREEN(3); }
		else if (BALL_cy < 80) { GO_GREEN(2); }
		else if (BALL_cy < 90) { GO_GREEN(1); }
		else {
			if (BALL_cx > 90 + 15) {
				if (BALL_cx > 135 )	GO_RIGHT();
				if (BALL_cx > 125 )	GO_RIGHT();
				if (BALL_cx > 115 )	GO_RIGHT_17();
				GO_RIGHT_17();
			}
			else if (BALL_cx < 90-15) {
				
				if (BALL_cx > 75 - 30 )	GO_LEFT();
				if (BALL_cx > 75 - 20 )	GO_LEFT();
				if (BALL_cx > 75 - 10 )	GO_LEFT_17();
				GO_LEFT_17();
			}
			else {
				break;
			}
		}
		*/
		if (flag == 1 && BALL_cy == 0 && BALL_cx == 0 ) { break; }
		
		if (BALL_cy == 0 && BALL_cx == 0 && flag == 0) {
			GO_GREEN(1);
			flag = 1;
		}
		else if (BALL_cy < 50) { GO_GREEN(5); }
		else if (BALL_cy < 60) { GO_GREEN(4); }
		else if (BALL_cy < 70) { GO_GREEN(3); }
		else if (BALL_cy < 80) { GO_GREEN(2); }
		else if (BALL_cy < 90) { GO_GREEN(1); }
		else {
			if (BALL_cx > 80) {
				GO_RIGHT_17();
				if (BALL_cx > 90 - 5)	GO_RIGHT_17();
				if (BALL_cx > 100 - 5)	GO_RIGHT();
				if (BALL_cx > 110 - 5)	GO_RIGHT();
			}
			else if (BALL_cx < 60) {
				GO_LEFT_17();
				if (BALL_cx > 50 + 5)	GO_LEFT_17();
				if (BALL_cx > 40 + 5)	GO_LEFT();
				if (BALL_cx > 30 + 5)	GO_LEFT();
			}
			else {
				break;
			}
		}


	}//while


	//printf("=================> MORE \n");

	while (1) {	//공에 더 가까이//머리 더 내리고

		HEAD_4BLACK();
		double cnt = 200000;
		while (cnt--) {}

		//printf("<<<<<<<<<<<<<<<<<<< BALL >>>>>>>>>>>>>>>>>>");
		INFO_CHECK(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			BALL_HL, BALL_HH, BALL_SL, BALL_SH, BALL_VL, BALL_VH, //INPUT
			&BALL_cx, &BALL_cy, &BALL_pix	//OUPUT
			);

		printf("BALL ::: cx : %d, cy : %d, pixel : %d \n", BALL_cx, BALL_cy, BALL_pix);

		if (BALL_cx == 0 && BALL_cy == 0 && BALL_pix == 0)	break;

		HEAD_4GREEN();
		cnt = 200000;
		while (cnt--) {}

		//printf("<<<<<<<<<<<<<<<<<<< GOAL >>>>>>>>>>>>>>>>>>");

		INFO_GOAL_CHECK(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			GOAL_HL, GOAL_HH, GOAL_SL, GOAL_SH, GOAL_VL, GOAL_VH, //INPUT
			&GOAL_cx, &GOAL_cy, &GOAL_pix	//OUPUT
			);

		printf("GOAL ::: cx : %d, cy : %d, pixel : %d \n", GOAL_cx, GOAL_cy, GOAL_pix);

		int d, R, L;
		R = 68 + 5;	//73
		L = 64 + 5;	//69
		d = 5;

		if(GOAL_cx <= BALL_cx + 6  && GOAL_cx >= BALL_cx - 6)
			if (BALL_cy < 50   &&   BALL_cx <= R && BALL_cx >= L) { break; }

		//회전
		if (GOAL_cx > BALL_cx + 6){				//6
			TURN_RIGHT();
			if (GOAL_cx > BALL_cx + 10)	G_TURN_RIGHT();	// 10 으로 나중!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		}	
		else if (GOAL_cx < BALL_cx - 6){		//6  	
			TURN_LEFT();
			if (GOAL_cx < BALL_cx - 10) G_TURN_LEFT();
		}

		//좌우로 이동
		if (BALL_cy >= 50 || BALL_cx > 130 || BALL_cx < 50) {
			if (BALL_cx > R) {
					
				if(BALL_cx > R + 2 * d) 	GO_RIGHT_17();
				if (BALL_cx > R + 3 * d)	GO_RIGHT_17();
				if (BALL_cx > R + 4 * d)	GO_RIGHT_17();
				if (BALL_cx > R + 5 * d)	GO_RIGHT_17();
				GO_RIGHT_5();
			}
			else if (BALL_cx < L) {
				if (BALL_cx < L - 2 * d)	GO_LEFT_17();
				if (BALL_cx < L - 3 * d)	GO_LEFT_17();
				if (BALL_cx < L - 4 * d)	GO_LEFT_17();
				if (BALL_cx < L - 5 * d)	GO_LEFT_17();
				GO_LEFT_5();
			}
			else break;
		}
		else {
			int d = 4; //3;
		//앞으로 조금
			if (BALL_cy < 55 - d * 2)	JJWALKING();
			if (BALL_cy < 55 - d * 3)	JJWALKING();
			if (BALL_cy < 55 - d * 4)	JJWALKING();
			if (BALL_cy < 55 - d * 5)	JJWALKING();
			JJWALKING();
		}
	}//while

	NORMAL_FORM();
	KICK_BALL();
}

//6666666666666666666666666666666666666666666666666666666666666666666666666666666666
void MISSION_6(void)	//미션6. 노란색 함정 다리 건너기 - 오르기/내리기시 바닥면 접촉 허용
{
	U16* fpga_src = (U16*)malloc(m_nWidth * m_nHeight * 2); //FPGA Original image
	U16* fpga_dst1 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst2 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst3 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U8* imgBGR = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* Filt = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgVSH = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgThresholded_H = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_S = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_V = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded = (U8*)malloc(m_nWidth * m_nHeight);
	U8* labelOUT = (U8*)malloc(m_nHeight * m_nWidth);
	U8* MAXlabelOUT = (U8*)malloc(m_nHeight * m_nWidth);

	int Y_cx = 0, Y_cy = 0, Y_pix = 0;

	//올라간 상태에서 덤블링 가능한 위치로 이동
	while (1) {

		HEAD_4RED();
		double cnt = 200000;
		while (cnt--) {}

		INFO_CHECK(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			YELLO_HL, YELLO_HH, YELLO_SL, YELLO_SH, YELLO_VL, YELLO_VH, //INPUT
			&Y_cx, &Y_cy, &Y_pix	//OUPUT
			);

		int d = 2;		
		int G_RD = 93;
		int G_LD = 87;

		if (Y_cx > G_RD) {	
			if (Y_cx > G_RD + 9 * d) { GO_RIGHT(); JJWALKING();  JJWALKING();}
			if (Y_cx > G_RD + 8 * d) { GO_RIGHT(); }
			if (Y_cx > G_RD + 7 * d) { GO_RIGHT(); JJWALKING();  JJWALKING();}
			if (Y_cx > G_RD + 6 * d) { GO_RIGHT(); }
			if (Y_cx > G_RD + 5 * d) { GO_RIGHT_17(); JJWALKING();  JJWALKING();}
			if (Y_cx > G_RD + 4 * d) { GO_RIGHT_17(); }
			if (Y_cx > G_RD + 3 * d) { GO_RIGHT_17(); JJWALKING();  JJWALKING();}
			if (Y_cx > G_RD + 2 * d) { GO_RIGHT_17(); }
			if (Y_cx > G_RD + 1 * d) { GO_RIGHT_17(); JJWALKING();  JJWALKING();}
			GO_RIGHT_17();	
			JJWALKING();
		}
		else if (Y_cx < G_LD) {
			if (Y_cx < G_LD - 9 * d) { GO_LEFT(); JJWALKING();  JJWALKING();}
			if (Y_cx < G_LD - 8 * d) { GO_LEFT(); }
			if (Y_cx < G_LD - 7 * d) { GO_LEFT(); JJWALKING();  JJWALKING();}
			if (Y_cx < G_LD - 6 * d) { GO_LEFT(); }
			if (Y_cx < G_LD - 5 * d) { GO_LEFT_17(); JJWALKING();  JJWALKING();}
			if (Y_cx < G_LD - 4 * d) { GO_LEFT_17(); }
			if (Y_cx < G_LD - 3 * d) { GO_LEFT_17(); JJWALKING();  JJWALKING();}
			if (Y_cx < G_LD - 2 * d) { GO_LEFT_17(); }
			if (Y_cx < G_LD - 1 * d) { GO_LEFT_17(); JJWALKING();  JJWALKING();}
			GO_LEFT_17();
			JJWALKING();
		}
		else {	break;	}
	}

	int tmp = 4;
	while (tmp--) { JJWALKING(); }

	UP_STAIR();
	GO_GREEN(2);
	
	float G = 0;

	//검은색으로 찾기
	while (1) {

		HEAD_4BLACK();
		double cnt = 200000;
		while (cnt--) {}

		INFO_TREND_LINE(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			YELLO_HL, YELLO_HH, YELLO_SL, YELLO_SH, YELLO_VL, YELLO_VH, //INPUT
			&Y_cx, &Y_cy, &Y_pix, &G, LP_HEAD_FRONT	//OUPUT
			);
		double M = atan2((double)G, 1) * 180 / 3.14159265;

		int d = 3;

		if (M > 3) {
			G_TURN_LEFT();
			if(M>7)	G_TURN_LEFT();
		}
		else if (M< -3) {
			G_TURN_RIGHT();
			if(M<-7)	G_TURN_RIGHT();
		}
		else {
			d = 4;
			if (Y_cx < 89 - 3) {
				GO_LEFT_17(); 
				if (Y_cx < 89 - 3 - 1 * d) GO_LEFT_17();
				if (Y_cx < 89 - 3 - 2 * d) GO_LEFT_17();
			}
			else if (Y_cx>89 + 3) {
				GO_RIGHT_17(); 
				if (Y_cx > 89 + 3 + 1 * d) GO_RIGHT_17();
				if (Y_cx > 89 + 3 + 2 * d) GO_RIGHT_17();
			}
			else {
				d = 4;

				if (Y_cy >= 65) {	break;	}
				else { 			
					JJWALKING();
					if (Y_cy < 65 - 1 * d)	JJWALKING();
					if (Y_cy < 65 - 2 * d)	JJWALKING();
					if (Y_cy < 65 - 3 * d)	JJWALKING();
					if (Y_cy < 65 - 4 * d)	JJWALKING();
					if (Y_cy < 65 - 5 * d)	JJWALKING();
				}
			}
		}

	}//while

	//JJWALKING();
	NORMAL_FORM();
}

//LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
void COLOR_LINE_UP(int mode,int color_set, int level) {	//검정색 기준 라인 정렬 함수
	U16* fpga_src = (U16*)malloc(m_nWidth * m_nHeight * 2); //FPGA Original image
	U16* fpga_dst1 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst2 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst3 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U8* imgBGR = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* Filt = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgVSH = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgThresholded_H = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_S = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_V = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded = (U8*)malloc(m_nWidth * m_nHeight);
	U8* labelOUT = (U8*)malloc(m_nHeight * m_nWidth);
	U8* MAXlabelOUT = (U8*)malloc(m_nHeight * m_nWidth);

	int B_cx = 0, B_cy = 0, B_pix = 0;
	float G = 0;
	int  HL = 0, HH = 0, SL = 0, SH = 0, VL = 0, VH = 0;
	int G_LD;
	int G_RD;
	if (color_set == LP_COLOR_WHITE) {
		HL = WHT_HL; HH = WHT_HH;  SL = WHT_SL; SH = WHT_SH; VL = WHT_VL; VH = WHT_VH;
	}
	else if (color_set == LP_COLOR_BLACK) {
		HL = BLK_HL; HH = BLK_HH;  SL = BLK_SL; SH = BLK_SH; VL = BLK_VL; VH = BLK_VH;
	}

	while (1) {

		ARM_BACK(mode);
		double cnt = 250000;
		while (cnt--) {}

		INFO_TREND_LINE(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			HL, HH, SL, SH, VL, VH, //INPUT
			&B_cx, &B_cy, &B_pix, &G, mode //OUPUT
			);
		if (mode == LP_HEAD_RIGHT) {			
			if(color_set == LP_COLOR_BLACK){

				double M = atan2((double)G, 1) * 180 / 3.14159265;
				int d = 4;
				int T_RD = -13;
				int T_LD = -7;
			
				if (M <T_RD) {	//회전 먼저 비교
					if (M <T_RD - 9 * d)	G_TURN_RIGHT();
					if (M <T_RD - 8 * d)	G_TURN_RIGHT();
					if (M <T_RD - 7 * d)	G_TURN_RIGHT();
					if (M <T_RD - 6 * d)	G_TURN_RIGHT();
					if (M <T_RD - 5 * d)	G_TURN_RIGHT();
					if (M <T_RD - 4 * d)	G_TURN_RIGHT();
					if (M <T_RD - 3 * d)	G_TURN_RIGHT();
					if (M <T_RD - 2 * d)	G_TURN_RIGHT();
					if (M <T_RD - 1 * d - 2)	G_TURN_RIGHT();
					G_TURN_RIGHT();
				}
				else if (M > T_LD){
					if (M > T_LD + 9 * d)	G_TURN_LEFT();
					if (M > T_LD + 8 * d)	G_TURN_LEFT();
					if (M > T_LD + 7 * d)	G_TURN_LEFT();
					if (M > T_LD + 6 * d)	G_TURN_LEFT();
					if (M > T_LD + 5 * d)	G_TURN_LEFT();
					if (M > T_LD + 4 * d)	G_TURN_LEFT();
					if (M > T_LD + 3 * d)	G_TURN_LEFT();
					if (M > T_LD + 2 * d + 1)	G_TURN_LEFT();
					if (M > T_LD + 1 * d + 2)	G_TURN_LEFT();
					G_TURN_LEFT();
				}
				else {
					if (level == JAL) {
						G_LD = 95;//90;//97;
						G_RD = 81;//76;//83;
					}
					else if (level == DAECHUNG) {
						G_LD = 90 + 8;
						G_RD = 76 - 8;
					}
					d = 2;
					if (B_cy > G_LD) {		//중심찾기
						if (B_cy > G_LD + 9 * d) GO_LEFT();
						if (B_cy > G_LD + 8 * d) GO_LEFT();
						if (B_cy > G_LD + 7 * d) GO_LEFT();
						if (B_cy > G_LD + 6 * d) GO_LEFT();
						if (B_cy > G_LD + 5 * d) GO_LEFT();
						if (B_cy > G_LD + 4 * d) GO_LEFT();
						if (B_cy > G_LD + 3 * d) GO_LEFT_17();
						if (B_cy > G_LD + 2 * d) GO_LEFT_17();
						if (B_cy > G_LD + 1 * d) GO_LEFT_17();
						GO_LEFT_17();
					}
					else if (B_cy < G_RD){
						if (B_cy < G_RD - 9 * d)	GO_RIGHT();
						if (B_cy < G_RD - 8 * d)	GO_RIGHT();
						if (B_cy < G_RD - 7 * d)	GO_RIGHT();
						if (B_cy < G_RD - 6 * d)	GO_RIGHT();
						if (B_cy < G_RD - 5 * d)	GO_RIGHT();
						if (B_cy < G_RD - 4 * d)	GO_RIGHT();
						if (B_cy < G_RD - 3 * d)	GO_RIGHT_17();
						if (B_cy < G_RD - 2 * d)	GO_RIGHT_17();
						if (B_cy < G_RD - 1 * d)	GO_RIGHT_17();
						GO_RIGHT_17();
					}
					else {	break;	}
				}

			}//black color

			else if (color_set == LP_COLOR_WHITE) {
				double M = atan2((double)G, 1) * 180 / 3.14159265;
				int d = 6;
				if (M < -13){
					if (M < -13 - 9 * d)	G_TURN_RIGHT();
					if (M < -13 - 8 * d)	G_TURN_RIGHT();
					if (M < -13 - 7 * d)	G_TURN_RIGHT();
					if (M < -13 - 6 * d)	G_TURN_RIGHT();
					if (M < -13 - 5 * d)	G_TURN_RIGHT();
					if (M < -13 - 4 * d)	G_TURN_RIGHT();
					if (M < -13 - 3 * d)	G_TURN_RIGHT();
					if (M < -13 - 2 * d)	G_TURN_RIGHT();
					if (M < -13 - 1 * d)	G_TURN_RIGHT();
					G_TURN_RIGHT();
				}
				else if (M > -4){
					if (M > -4 + 9 * d)	G_TURN_LEFT();
					if (M > -4 + 8 * d)	G_TURN_LEFT();
					if (M > -4 + 7 * d)	G_TURN_LEFT();
					if (M > -4 + 6 * d)	G_TURN_LEFT();
					if (M > -4 + 5 * d)	G_TURN_LEFT();
					if (M > -4 + 4 * d)	G_TURN_LEFT();
					if (M > -4 + 3 * d)	G_TURN_LEFT();
					if (M > -4 + 2 * d)	G_TURN_LEFT();
					if (M > -4 + 1 * d)	G_TURN_LEFT();
					G_TURN_LEFT();
				}
				else {
					d = 2;
					if (B_cy > 106 + 4){
						if (B_cy > 113 + 9 * d) GO_LEFT_17();
						if (B_cy > 113 + 8 * d) GO_LEFT_17();
						if (B_cy > 113 + 7 * d) GO_LEFT_17();
						if (B_cy > 113 + 6 * d) GO_LEFT_17();
						if (B_cy > 113 + 5 * d) GO_LEFT_17();
						if (B_cy > 113 + 4 * d) GO_LEFT_17();
						if (B_cy > 113 + 3 * d) GO_LEFT_17();
						if (B_cy > 113 + 2 * d) GO_LEFT_17();
						if (B_cy > 113 + 1 * d) GO_LEFT_17();
						GO_LEFT_17();	//중심찾기
					}
					else if (B_cy < 106 - 4){
						if (B_cy < 99 - 9 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 8 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 7 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 6 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 5 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 4 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 3 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 2 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 1 * d)	GO_RIGHT_17();
						GO_RIGHT_17();
					}
					else { break; }
				}
			}//white color

		}//RIGHT

		else if(mode == LP_HEAD_FRONT) {		
			if (G * 100 > 7)	TURN_RIGHT();	//회전 먼저 비교
			else if (G * 100 < -5)	TURN_LEFT();
			else {
				if (B_cx > 55)	GO_LEFT();	//중심찾기
				else if (B_cx < 45)		GO_RIGHT();
				else {
					if (G * 100 > 7)	TURN_RIGHT();
					else if (G * 100 < -5)		TURN_LEFT();
					else {		break;	}
				}
			}
		}

	}//while

	NORMAL_FORM();

}


//LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
void COLOR_LINE_UP2(int mode, int color_set, int level) {	//검정색 기준 라인 정렬 함수
	U16* fpga_src = (U16*)malloc(m_nWidth * m_nHeight * 2); //FPGA Original image
	U16* fpga_dst1 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst2 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U16* fpga_dst3 = (U16*)malloc(m_nWidth * m_nHeight * 2);
	U8* imgBGR = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* Filt = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgVSH = (U8*)malloc(m_nWidth * m_nHeight * 3);
	U8* imgThresholded_H = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_S = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded_V = (U8*)malloc(m_nWidth * m_nHeight);
	U8* imgThresholded = (U8*)malloc(m_nWidth * m_nHeight);
	U8* labelOUT = (U8*)malloc(m_nHeight * m_nWidth);
	U8* MAXlabelOUT = (U8*)malloc(m_nHeight * m_nWidth);

	int B_cx = 0, B_cy = 0, B_pix = 0;
	float G = 0;
	int  HL = 0, HH = 0, SL = 0, SH = 0, VL = 0, VH = 0;
	int G_LD;
	int G_RD;
	if (color_set == LP_COLOR_WHITE) {
		HL = WHT_HL; HH = WHT_HH;  SL = WHT_SL; SH = WHT_SH; VL = WHT_VL; VH = WHT_VH;
	}
	else if (color_set == LP_COLOR_BLACK) {
		HL = BLK_HL; HH = BLK_HH;  SL = BLK_SL; SH = BLK_SH; VL = BLK_VL; VH = BLK_VH;
	}

	while (1) {

		ARM_BACK(mode);
		double cnt = 250000;
		while (cnt--) {}

		INFO_TREND_LINE(
			fpga_src, fpga_dst1, fpga_dst2, fpga_dst3, imgBGR, Filt,
			imgVSH, imgThresholded_H, imgThresholded_S, imgThresholded_V,
			imgThresholded, labelOUT, MAXlabelOUT,
			HL, HH, SL, SH, VL, VH, //INPUT
			&B_cx, &B_cy, &B_pix, &G, mode //OUPUT
			);
		if (mode == LP_HEAD_RIGHT) {
			if (color_set == LP_COLOR_BLACK) {

				double M = atan2((double)G, 1) * 180 / 3.14159265;
				int d = 4;
				int T_RD = -13;
				int T_LD = -7;

				if (M <T_RD) {	//회전 먼저 비교
					if (M <T_RD - 9 * d)	G_TURN_RIGHT();
					if (M <T_RD - 8 * d)	G_TURN_RIGHT();
					if (M <T_RD - 7 * d)	G_TURN_RIGHT();
					if (M <T_RD - 6 * d)	G_TURN_RIGHT();
					if (M <T_RD - 5 * d)	G_TURN_RIGHT();
					if (M <T_RD - 4 * d)	G_TURN_RIGHT();
					if (M <T_RD - 3 * d)	G_TURN_RIGHT();
					if (M <T_RD - 2 * d)	G_TURN_RIGHT();
					if (M <T_RD - 1 * d - 2)	G_TURN_RIGHT();
					G_TURN_RIGHT();
				}
				else if (M > T_LD) {
					if (M > T_LD + 9 * d)	G_TURN_LEFT();
					if (M > T_LD + 8 * d)	G_TURN_LEFT();
					if (M > T_LD + 7 * d)	G_TURN_LEFT();
					if (M > T_LD + 6 * d)	G_TURN_LEFT();
					if (M > T_LD + 5 * d)	G_TURN_LEFT();
					if (M > T_LD + 4 * d)	G_TURN_LEFT();
					if (M > T_LD + 3 * d)	G_TURN_LEFT();
					if (M > T_LD + 2 * d + 1)	G_TURN_LEFT();
					if (M > T_LD + 1 * d + 2)	G_TURN_LEFT();
					G_TURN_LEFT();
				}
				else {
					if (level == JAL) {
						G_LD = 97;
						G_RD = 83;
					}
					else if (level == DAECHUNG) {
						G_LD = 90 + 8;
						G_RD = 76 - 8;
					}
					d = 2;
					if (B_cy > G_LD) {		//중심찾기
						if (B_cy > G_LD + 9 * d) GO_LEFT();
						if (B_cy > G_LD + 8 * d) GO_LEFT();
						if (B_cy > G_LD + 7 * d) GO_LEFT();
						if (B_cy > G_LD + 6 * d) GO_LEFT();
						if (B_cy > G_LD + 5 * d) GO_LEFT();
						if (B_cy > G_LD + 4 * d) GO_LEFT();
						if (B_cy > G_LD + 3 * d) GO_LEFT_17();
						if (B_cy > G_LD + 2 * d) GO_LEFT_17();
						if (B_cy > G_LD + 1 * d) GO_LEFT_17();
						GO_LEFT_17();
					}
					else if (B_cy < G_RD) {
						if (B_cy < G_RD - 9 * d)	GO_RIGHT();
						if (B_cy < G_RD - 8 * d)	GO_RIGHT();
						if (B_cy < G_RD - 7 * d)	GO_RIGHT();
						if (B_cy < G_RD - 6 * d)	GO_RIGHT();
						if (B_cy < G_RD - 5 * d)	GO_RIGHT();
						if (B_cy < G_RD - 4 * d)	GO_RIGHT();
						if (B_cy < G_RD - 3 * d)	GO_RIGHT_17();
						if (B_cy < G_RD - 2 * d)	GO_RIGHT_17();
						if (B_cy < G_RD - 1 * d)	GO_RIGHT_17();
						GO_RIGHT_17();
					}
					else { break; }
				}

			}//black color

			else if (color_set == LP_COLOR_WHITE) {
				double M = atan2((double)G, 1) * 180 / 3.14159265;
				int d = 6;
				if (M < -13) {
					if (M < -13 - 9 * d)	G_TURN_RIGHT();
					if (M < -13 - 8 * d)	G_TURN_RIGHT();
					if (M < -13 - 7 * d)	G_TURN_RIGHT();
					if (M < -13 - 6 * d)	G_TURN_RIGHT();
					if (M < -13 - 5 * d)	G_TURN_RIGHT();
					if (M < -13 - 4 * d)	G_TURN_RIGHT();
					if (M < -13 - 3 * d)	G_TURN_RIGHT();
					if (M < -13 - 2 * d)	G_TURN_RIGHT();
					if (M < -13 - 1 * d)	G_TURN_RIGHT();
					G_TURN_RIGHT();
				}
				else if (M > -4) {
					if (M > -4 + 9 * d)	G_TURN_LEFT();
					if (M > -4 + 8 * d)	G_TURN_LEFT();
					if (M > -4 + 7 * d)	G_TURN_LEFT();
					if (M > -4 + 6 * d)	G_TURN_LEFT();
					if (M > -4 + 5 * d)	G_TURN_LEFT();
					if (M > -4 + 4 * d)	G_TURN_LEFT();
					if (M > -4 + 3 * d)	G_TURN_LEFT();
					if (M > -4 + 2 * d)	G_TURN_LEFT();
					if (M > -4 + 1 * d)	G_TURN_LEFT();
					G_TURN_LEFT();
				}
				else {
					d = 2;
					if (B_cy > 106 + 4) {
						if (B_cy > 113 + 9 * d) GO_LEFT_17();
						if (B_cy > 113 + 8 * d) GO_LEFT_17();
						if (B_cy > 113 + 7 * d) GO_LEFT_17();
						if (B_cy > 113 + 6 * d) GO_LEFT_17();
						if (B_cy > 113 + 5 * d) GO_LEFT_17();
						if (B_cy > 113 + 4 * d) GO_LEFT_17();
						if (B_cy > 113 + 3 * d) GO_LEFT_17();
						if (B_cy > 113 + 2 * d) GO_LEFT_17();
						if (B_cy > 113 + 1 * d) GO_LEFT_17();
						GO_LEFT_17();	//중심찾기
					}
					else if (B_cy < 106 - 4) {
						if (B_cy < 99 - 9 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 8 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 7 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 6 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 5 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 4 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 3 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 2 * d)	GO_RIGHT_17();
						if (B_cy < 99 - 1 * d)	GO_RIGHT_17();
						GO_RIGHT_17();
					}
					else { break; }
				}
			}//white color

		}//RIGHT

		else if (mode == LP_HEAD_FRONT) {
			if (G * 100 > 7)	TURN_RIGHT();	//회전 먼저 비교
			else if (G * 100 < -5)	TURN_LEFT();
			else {
				if (B_cx > 55)	GO_LEFT();	//중심찾기
				else if (B_cx < 45)		GO_RIGHT();
				else {
					if (G * 100 > 7)	TURN_RIGHT();
					else if (G * 100 < -5)		TURN_LEFT();
					else { break; }
				}
			}
		}

	}//while

	NORMAL_FORM();

}

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

//필요한 색깔 영역에서 중심점과 픽셀 갯수 얻는  함수
void INFO_CHECK(
	U16* fpga_src, U16* fpga_dst1, U16* fpga_dst2, U16* fpga_dst3, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix	//OUPUT
	)
{
	clear_screen();
	read_fpga_video_data(fpga_src);
	draw_img_from_buffer(fpga_src, 0, 18, 0, 0, 1.77, 0);

	RGB565_to_BGR888(fpga_src, imgBGR, m_nWidth, m_nHeight);	//565 -> 888
	MeanFiltering(imgBGR, Filt, m_nWidth, m_nHeight);			//Median filterling
	BGR888_to_VSH888_30(Filt, imgVSH, m_nWidth, m_nHeight);		//RGB -> HSV

	VSH888_to_H8_thresh(imgVSH, imgThresholded_H, m_nWidth, m_nHeight, HL, HH);	//H
	VSH888_to_S8_thresh(imgVSH, imgThresholded_S, m_nWidth, m_nHeight, SL, SH);	//S
	VSH888_to_V8_thresh(imgVSH, imgThresholded_V, m_nWidth, m_nHeight, VL, VH);	//V

	HSV_to_Binary8(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, m_nWidth, m_nHeight);

	//바운더리 예외처리
	int i = 0, j = 0;
	for (j = 0; j < m_nHeight; j++) {		//왼쪽 테두리
		imgThresholded[m_nWidth * j] = 0;
		for (i = 0; i < 5; i++)		imgThresholded[i + m_nWidth * j] = 0;
	}
	for (i = 0; i < m_nWidth; i++)			//위쪽 테두리
		imgThresholded[i] = 0;

	int label = 0, MAXlabelc = 0;
	const int MAX_LABEL = 1000;
	int eq_tbl[MAX_LABEL][2];
	for (i = 0; i < MAX_LABEL; i++) {
		eq_tbl[i][0] = 0;
		eq_tbl[i][1] = 0;
	}

	int maxl = 0, minl = 0, min_eq = 0, max_eq = 0;
	int L = 0, R = 0, U = 0, D = 0;

	for (j = 1; j < m_nHeight - 1; j++) {
		for (i = 1; i < m_nWidth - 1; i++) {
			L = i - 1;	//왼
			R = i + 1;	//오
			U = j - 1;	//위
			D = j + 1;	//아래

			if (imgThresholded[i + m_nWidth * j] == 255) {

				// 바로 위 픽셀과 왼쪽 픽셀 모두에 레이블이 존재하는 경우
				if ((imgThresholded[L + m_nWidth * j] != 0) && (imgThresholded[i + m_nWidth * U] != 0)) {

					//위, 왼 레이블이 같음
					if (imgThresholded[L + m_nWidth * j] == imgThresholded[i + m_nWidth * U])
						imgThresholded[i + m_nWidth * j] = imgThresholded[i + m_nWidth * U];

					else { // 두 레이블이 서로 다른 경우, 작은 레이블을 부여

						maxl = MAX(imgThresholded[L + m_nWidth * j], imgThresholded[i + m_nWidth * U]);
						minl = MIN(imgThresholded[L + m_nWidth * j], imgThresholded[i + m_nWidth * U]);

						imgThresholded[i + m_nWidth * j] = minl;          //픽셀에 작은값 대입
																		  //등가 테이블 조정
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// 바로 위 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[i + m_nWidth * U] != 0)
					imgThresholded[i + m_nWidth * j] = imgThresholded[i + m_nWidth * U];

				//왼쪽 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[L + m_nWidth * j] != 0)
					imgThresholded[i + m_nWidth * j] = imgThresholded[L + m_nWidth * j];

				//주위에 레이블이 없음. 새로운 레이블 부여
				else {

					label++;
					imgThresholded[i + m_nWidth * j] = label;
					eq_tbl[label][0] = label;
					eq_tbl[label][1] = label;
				}
			}
		}

	}

	int temp = 0;
	for (i = 1; i <= label; i++) {
		temp = eq_tbl[i][1];
		if (temp != eq_tbl[i][0])	eq_tbl[i][1] = eq_tbl[temp][1];
	}

	// 등가 테이블의 레이블을 1부터 차례대로 증가시키기
	int hash[label + 1];
	for (i = 0; i <= label; i++)	hash[i] = 0;

	for (i = 1; i <= label; i++)	hash[eq_tbl[i][1]] = eq_tbl[i][1];

	int label_cnt = 1;
	for (i = 1; i <= label; i++)	if (hash[i] != 0) hash[i] = label_cnt++;

	int m_label = 0;
	for (i = 1; i <= label; i++) {
		eq_tbl[i][1] = hash[eq_tbl[i][1]];
		if (m_label < eq_tbl[i][1])	m_label = eq_tbl[i][1];
	}

	for (j = 0; j < m_nHeight; j++)
		for (i = 0; i< m_nWidth; i++) {
			labelOUT[i + m_nWidth * j] = 0;
			MAXlabelOUT[i + m_nWidth * j] = 0;
		}

	int idx = 0;
	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (imgThresholded[i + m_nWidth * j] != 0) {
				idx = imgThresholded[i + m_nWidth * j];
				labelOUT[i + m_nWidth * j] = eq_tbl[idx][1]; // eq_tbl[idx][1]가 255이상이면 
			}
		}

	struct Label LABEL[m_label + 1];
	struct Label MAXLABEL = { 0, 0, 0, 0, 0, 0, 180, 0, 120 };

	int m = 0;
	for (m = 0; m < m_label + 1; m++) {
		LABEL[m].sumX = 0;
		LABEL[m].sumY = 0;
		LABEL[m].pix = 0;
	}

	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (labelOUT[i + m_nWidth * j] != 0) {
				LABEL[labelOUT[i + m_nWidth * j]].pix++;
				LABEL[labelOUT[i + m_nWidth * j]].sumX += i;
				LABEL[labelOUT[i + m_nWidth * j]].sumY += j;

				if (i > LABEL[labelOUT[i + m_nWidth * j]].maxX) LABEL[labelOUT[i + m_nWidth * j]].maxX = i;
				if (i < LABEL[labelOUT[i + m_nWidth * j]].minX) LABEL[labelOUT[i + m_nWidth * j]].minX = i;
				if (j > LABEL[labelOUT[i + m_nWidth * j]].maxY) LABEL[labelOUT[i + m_nWidth * j]].maxY = j;
				if (j < LABEL[labelOUT[i + m_nWidth * j]].minY) LABEL[labelOUT[i + m_nWidth * j]].minY = j;
			}
		}

	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (labelOUT[i + m_nWidth * j]>0 && LABEL[labelOUT[i + m_nWidth * j]].pix > 0) {
				LABEL[labelOUT[i + m_nWidth * j]].cX = LABEL[labelOUT[i + m_nWidth * j]].sumX / LABEL[labelOUT[i + m_nWidth * j]].pix;
				LABEL[labelOUT[i + m_nWidth * j]].cY = LABEL[labelOUT[i + m_nWidth * j]].sumY / LABEL[labelOUT[i + m_nWidth * j]].pix;
			}
		}

	for (i = 1; i < m_label + 1; i++) {
		if (LABEL[i].pix > 100 && LABEL[i].pix > MAXLABEL.pix) {
			MAXLABEL.pix = LABEL[i].pix;
			MAXLABEL.cX = LABEL[i].cX;
			MAXLABEL.cY = LABEL[i].cY;
			MAXlabelc = i;
		}
	}

	//////////////////   MAXLABEL 만 MAXlabelOUT에 남기기    //////////////////////
	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (MAXlabelc == 0 || labelOUT[i + m_nWidth * j] != MAXlabelc) {
				MAXlabelOUT[i + m_nWidth * j] = 0;
			}
			else if (labelOUT[i + m_nWidth * j] == MAXlabelc || m_label == 0) {
				MAXlabelOUT[i + m_nWidth * j] = 255;
			}
		}

	Binary8_to_RGB565(MAXlabelOUT, fpga_dst3, m_nWidth, m_nHeight, B_GRAY_MODE);
	draw_img_from_buffer(fpga_dst3, 0, 250, 0, 0, 1.77, 0);

	flip();

	*out_cX = MAXLABEL.cX;
	*out_cY = MAXLABEL.cY;
	*out_pix = MAXLABEL.pix;

	//printf("cx : %d , cy : %d, pix : %d \n", *out_cX, *out_cY, *out_pix);
}
void INFO_GOAL_CHECK(
	U16* fpga_src, U16* fpga_dst1, U16* fpga_dst2, U16* fpga_dst3, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix	//OUPUT
	)
{
	clear_screen();
	read_fpga_video_data(fpga_src);
	draw_img_from_buffer(fpga_src, 0, 18, 0, 0, 1.77, 0);

	RGB565_to_BGR888(fpga_src, imgBGR, m_nWidth, m_nHeight);	//565 -> 888
	MeanFiltering(imgBGR, Filt, m_nWidth, m_nHeight);			//Median filterling
	BGR888_to_VSH888_30(Filt, imgVSH, m_nWidth, m_nHeight);		//RGB -> HSV

	VSH888_to_H8_thresh(imgVSH, imgThresholded_H, m_nWidth, m_nHeight, HL, HH);	//H
	VSH888_to_S8_thresh(imgVSH, imgThresholded_S, m_nWidth, m_nHeight, SL, SH);	//S
	VSH888_to_V8_thresh(imgVSH, imgThresholded_V, m_nWidth, m_nHeight, VL, VH);	//V

	HSV_to_Binary8(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, m_nWidth, m_nHeight);

	//바운더리 예외처리
	int i = 0, j = 0;
	for (j = 0; j < m_nHeight; j++) {		//왼쪽 테두리
		imgThresholded[m_nWidth * j] = 0;
		for (i = 0; i < 5; i++)		imgThresholded[i + m_nWidth * j] = 0;
	}
	for (i = 0; i < m_nWidth; i++)			//위쪽 테두리
		imgThresholded[i] = 0;

	int label = 0, MAXlabelc = 0;
	const int MAX_LABEL = 1000;
	int eq_tbl[MAX_LABEL][2];
	for (i = 0; i < MAX_LABEL; i++) {
		eq_tbl[i][0] = 0;
		eq_tbl[i][1] = 0;
	}

	int maxl = 0, minl = 0, min_eq = 0, max_eq = 0;
	int L = 0, R = 0, U = 0, D = 0;

	for (j = 1; j < m_nHeight - 1; j++) {
		for (i = 1; i < m_nWidth - 1; i++) {
			L = i - 1;	//왼
			R = i + 1;	//오
			U = j - 1;	//위
			D = j + 1;	//아래

			if (imgThresholded[i + m_nWidth * j] == 255) {

				// 바로 위 픽셀과 왼쪽 픽셀 모두에 레이블이 존재하는 경우
				if ((imgThresholded[L + m_nWidth * j] != 0) && (imgThresholded[i + m_nWidth * U] != 0)) {

					//위, 왼 레이블이 같음
					if (imgThresholded[L + m_nWidth * j] == imgThresholded[i + m_nWidth * U])
						imgThresholded[i + m_nWidth * j] = imgThresholded[i + m_nWidth * U];

					else { // 두 레이블이 서로 다른 경우, 작은 레이블을 부여

						maxl = MAX(imgThresholded[L + m_nWidth * j], imgThresholded[i + m_nWidth * U]);
						minl = MIN(imgThresholded[L + m_nWidth * j], imgThresholded[i + m_nWidth * U]);

						imgThresholded[i + m_nWidth * j] = minl;          //픽셀에 작은값 대입
																		  //등가 테이블 조정
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// 바로 위 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[i + m_nWidth * U] != 0)
					imgThresholded[i + m_nWidth * j] = imgThresholded[i + m_nWidth * U];

				//왼쪽 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[L + m_nWidth * j] != 0)
					imgThresholded[i + m_nWidth * j] = imgThresholded[L + m_nWidth * j];

				//주위에 레이블이 없음. 새로운 레이블 부여
				else {

					label++;
					imgThresholded[i + m_nWidth * j] = label;
					eq_tbl[label][0] = label;
					eq_tbl[label][1] = label;
				}
			}
		}

	}

	int temp = 0;
	for (i = 1; i <= label; i++) {
		temp = eq_tbl[i][1];
		if (temp != eq_tbl[i][0])	eq_tbl[i][1] = eq_tbl[temp][1];
	}

	// 등가 테이블의 레이블을 1부터 차례대로 증가시키기
	int hash[label + 1];
	for (i = 0; i <= label; i++)	hash[i] = 0;

	for (i = 1; i <= label; i++)	hash[eq_tbl[i][1]] = eq_tbl[i][1];

	int label_cnt = 1;
	for (i = 1; i <= label; i++)	if (hash[i] != 0) hash[i] = label_cnt++;

	int m_label = 0;
	for (i = 1; i <= label; i++) {
		eq_tbl[i][1] = hash[eq_tbl[i][1]];
		if (m_label < eq_tbl[i][1])	m_label = eq_tbl[i][1];
	}

	for (j = 0; j < m_nHeight; j++)
		for (i = 0; i< m_nWidth; i++) {
			labelOUT[i + m_nWidth * j] = 0;
			MAXlabelOUT[i + m_nWidth * j] = 0;
		}

	int idx = 0;
	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (imgThresholded[i + m_nWidth * j] != 0) {
				idx = imgThresholded[i + m_nWidth * j];
				labelOUT[i + m_nWidth * j] = eq_tbl[idx][1]; // eq_tbl[idx][1]가 255이상이면 
			}
		}

	struct Label LABEL[m_label + 1];
	struct Label MAXLABEL = { 0, 0, 0, 0, 0, 0, 180, 0, 120 };

	int m = 0;
	for (m = 0; m < m_label + 1; m++) {
		LABEL[m].sumX = 0;
		LABEL[m].sumY = 0;
		LABEL[m].pix = 0;
	}

	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (labelOUT[i + m_nWidth * j] != 0) {
				LABEL[labelOUT[i + m_nWidth * j]].pix++;
				LABEL[labelOUT[i + m_nWidth * j]].sumX += i;
				LABEL[labelOUT[i + m_nWidth * j]].sumY += j;

				if (i > LABEL[labelOUT[i + m_nWidth * j]].maxX) LABEL[labelOUT[i + m_nWidth * j]].maxX = i;
				if (i < LABEL[labelOUT[i + m_nWidth * j]].minX) LABEL[labelOUT[i + m_nWidth * j]].minX = i;
				if (j > LABEL[labelOUT[i + m_nWidth * j]].maxY) LABEL[labelOUT[i + m_nWidth * j]].maxY = j;
				if (j < LABEL[labelOUT[i + m_nWidth * j]].minY) LABEL[labelOUT[i + m_nWidth * j]].minY = j;
			}
		}

	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (labelOUT[i + m_nWidth * j]>0 && LABEL[labelOUT[i + m_nWidth * j]].pix > 0) {
				LABEL[labelOUT[i + m_nWidth * j]].cX = LABEL[labelOUT[i + m_nWidth * j]].sumX / LABEL[labelOUT[i + m_nWidth * j]].pix;
				LABEL[labelOUT[i + m_nWidth * j]].cY = LABEL[labelOUT[i + m_nWidth * j]].sumY / LABEL[labelOUT[i + m_nWidth * j]].pix;
			}
		}

	for (i = 1; i < m_label + 1; i++) {
		if ( LABEL[i].cY < 70 && LABEL[i].pix > 60  &&LABEL[i].pix < 190 && LABEL[i].cY> MAXLABEL.cY) {	//LABEL[i].pix > MAXLABEL.pix &&
			MAXLABEL.pix = LABEL[i].pix;
			MAXLABEL.cX = LABEL[i].cX;
			MAXLABEL.cY = LABEL[i].cY;
			MAXlabelc = i;
		}
	}

	//////////////////   MAXLABEL 만 MAXlabelOUT에 남기기    //////////////////////
	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (MAXlabelc == 0 || labelOUT[i + m_nWidth * j] != MAXlabelc) {
				MAXlabelOUT[i + m_nWidth * j] = 0;
			}
			else if (labelOUT[i + m_nWidth * j] == MAXlabelc || m_label == 0) {
				MAXlabelOUT[i + m_nWidth * j] = 255;
			}
		}

	Binary8_to_RGB565(MAXlabelOUT, fpga_dst3, m_nWidth, m_nHeight, B_GRAY_MODE);
	draw_img_from_buffer(fpga_dst3, 0, 250, 0, 0, 1.77, 0);
																		
	flip();

	*out_cX = MAXLABEL.cX;
	*out_cY = MAXLABEL.cY;
	*out_pix = MAXLABEL.pix;

	//printf("cx : %d , cy : %d, pix : %d \n", *out_cX, *out_cY, *out_pix);
}

//추세선으로 라인 정렬할 정보 찾는 함수
void INFO_TREND_LINE(
	U16* fpga_src, U16* fpga_dst1, U16* fpga_dst2, U16* fpga_dst3, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix, float* G, int mode  //OUPUT
	)
{
	clear_screen();
	read_fpga_video_data(fpga_src);
	draw_img_from_buffer(fpga_src, 0, 18, 0, 0, 1.77, 0);


	RGB565_to_BGR888(fpga_src, imgBGR, m_nWidth, m_nHeight);	//565 -> 888
	MeanFiltering(imgBGR, Filt, m_nWidth, m_nHeight);			//Median filterling
	BGR888_to_VSH888_30(Filt, imgVSH, m_nWidth, m_nHeight);		//RGB -> HSV

	VSH888_to_H8_thresh(imgVSH, imgThresholded_H, m_nWidth, m_nHeight, HL, HH);	//H
	VSH888_to_S8_thresh(imgVSH, imgThresholded_S, m_nWidth, m_nHeight, SL, SH);	//S
	VSH888_to_V8_thresh(imgVSH, imgThresholded_V, m_nWidth, m_nHeight, VL, VH);	//V

	if (HL == YELLO_HL && HH == YELLO_HH)
		HSV_to_Binary8_R(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, m_nWidth, m_nHeight);
	else
		HSV_to_Binary8(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, m_nWidth, m_nHeight);

	//바운더리 예외처리
	int i = 0, j = 0;
	for (j = 0; j < m_nHeight; j++) {		//왼쪽 테두리
		imgThresholded[m_nWidth * j] = 0;
		for (i = 0; i < 5; i++)		imgThresholded[i + m_nWidth * j] = 0;
	}
	for (i = 0; i < m_nWidth; i++)			//위쪽 테두리
		imgThresholded[i] = 0;

	//앞 or 우를 볼 경우 반 자르기
	if (mode == LP_HEAD_RIGHT) {
		for (j = 0; j < m_nHeight; j++) {
			for (i = 91; i < m_nWidth; i++)	imgThresholded[i + m_nWidth * j] = 0;
		}
	}
	else if(mode == LP_HEAD_FRONT) {
		for (j = 75; j < m_nHeight; j++) {	//앞 자르기
			for (i = 0; i < m_nWidth; i++)	imgThresholded[i + m_nWidth * j] = 0;
		}
		for (j = 0; j < m_nHeight; j++) {
			for (i = 0; i < 20; i++)	imgThresholded[i + m_nWidth * j] = 0;
			for (i = m_nWidth-20; i < m_nWidth; i++)	imgThresholded[i + m_nWidth * j] = 0;
		}
	}

	int label = 0, MAXlabelc = 0;
	const int MAX_LABEL = 1000;
	int eq_tbl[MAX_LABEL][2];
	for (i = 0; i < MAX_LABEL; i++) {
		eq_tbl[i][0] = 0;
		eq_tbl[i][1] = 0;
	}

	int maxl = 0, minl = 0, min_eq = 0, max_eq = 0;
	int L = 0, R = 0, U = 0, D = 0;

	for (j = 1; j < m_nHeight - 1; j++) {
		for (i = 1; i < m_nWidth - 1; i++) {
			L = i - 1;	//왼
			R = i + 1;	//오
			U = j - 1;	//위
			D = j + 1;	//아래

			if (imgThresholded[i + m_nWidth * j] == 255) {

				// 바로 위 픽셀과 왼쪽 픽셀 모두에 레이블이 존재하는 경우
				if ((imgThresholded[L + m_nWidth * j] != 0) && (imgThresholded[i + m_nWidth * U] != 0)) {

					//위, 왼 레이블이 같음
					if (imgThresholded[L + m_nWidth * j] == imgThresholded[i + m_nWidth * U])
						imgThresholded[i + m_nWidth * j] = imgThresholded[i + m_nWidth * U];

					else { // 두 레이블이 서로 다른 경우, 작은 레이블을 부여

						maxl = MAX(imgThresholded[L + m_nWidth * j], imgThresholded[i + m_nWidth * U]);
						minl = MIN(imgThresholded[L + m_nWidth * j], imgThresholded[i + m_nWidth * U]);

						imgThresholded[i + m_nWidth * j] = minl;          //픽셀에 작은값 대입
																		  //등가 테이블 조정
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// 바로 위 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[i + m_nWidth * U] != 0)
					imgThresholded[i + m_nWidth * j] = imgThresholded[i + m_nWidth * U];

				//왼쪽 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[L + m_nWidth * j] != 0)
					imgThresholded[i + m_nWidth * j] = imgThresholded[L + m_nWidth * j];

				//주위에 레이블이 없음. 새로운 레이블 부여
				else {

					label++;
					imgThresholded[i + m_nWidth * j] = label;
					eq_tbl[label][0] = label;
					eq_tbl[label][1] = label;
				}
			}
		}

	}

	int temp = 0;
	for (i = 1; i <= label; i++) {
		temp = eq_tbl[i][1];
		if (temp != eq_tbl[i][0])	eq_tbl[i][1] = eq_tbl[temp][1];
	}

	// 등가 테이블의 레이블을 1부터 차례대로 증가시키기
	int hash[label + 1];
	for (i = 0; i <= label; i++)	hash[i] = 0;

	for (i = 1; i <= label; i++)	hash[eq_tbl[i][1]] = eq_tbl[i][1];

	int label_cnt = 1;
	for (i = 1; i <= label; i++)	if (hash[i] != 0) hash[i] = label_cnt++;

	int m_label = 0;
	for (i = 1; i <= label; i++) {
		eq_tbl[i][1] = hash[eq_tbl[i][1]];
		if (m_label < eq_tbl[i][1])	m_label = eq_tbl[i][1];
	}

	for (j = 0; j < m_nHeight; j++)
		for (i = 0; i< m_nWidth; i++) {
			labelOUT[i + m_nWidth * j] = 0;
			MAXlabelOUT[i + m_nWidth * j] = 0;
		}

	int idx = 0;
	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (imgThresholded[i + m_nWidth * j] != 0) {
				idx = imgThresholded[i + m_nWidth * j];
				labelOUT[i + m_nWidth * j] = eq_tbl[idx][1]; // eq_tbl[idx][1]가 255이상이면 
			}
		}

	struct Label LABEL[m_label + 1];
	struct Label MAXLABEL = { 0, 0, 0, 0, 0, 0, 180, 0, 120 };

	int m = 0;
	for (m = 0; m < m_label + 1; m++) {
		LABEL[m].sumX = 0;
		LABEL[m].sumY = 0;
		LABEL[m].pix = 0;
		LABEL[m].minX = 255;
		LABEL[m].maxX = 0;
		LABEL[m].minY = 255;
		LABEL[m].maxY = 0;
	}

	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (labelOUT[i + m_nWidth * j] != 0) {
				LABEL[labelOUT[i + m_nWidth * j]].pix++;
				LABEL[labelOUT[i + m_nWidth * j]].sumX += i;
				LABEL[labelOUT[i + m_nWidth * j]].sumY += j;

				if (i > LABEL[labelOUT[i + m_nWidth * j]].maxX) LABEL[labelOUT[i + m_nWidth * j]].maxX = i;
				if (i < LABEL[labelOUT[i + m_nWidth * j]].minX) LABEL[labelOUT[i + m_nWidth * j]].minX = i;
				if (j > LABEL[labelOUT[i + m_nWidth * j]].maxY) LABEL[labelOUT[i + m_nWidth * j]].maxY = j;
				if (j < LABEL[labelOUT[i + m_nWidth * j]].minY) LABEL[labelOUT[i + m_nWidth * j]].minY = j;
			}
		}

	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (labelOUT[i + m_nWidth * j]>0 && LABEL[labelOUT[i + m_nWidth * j]].pix > 0) {
				LABEL[labelOUT[i + m_nWidth * j]].cX = LABEL[labelOUT[i + m_nWidth * j]].sumX / LABEL[labelOUT[i + m_nWidth * j]].pix;
				LABEL[labelOUT[i + m_nWidth * j]].cY = LABEL[labelOUT[i + m_nWidth * j]].sumY / LABEL[labelOUT[i + m_nWidth * j]].pix;
			}
		}

	if (HL == YELLO_HL && HH == YELLO_HH){
		for (i = 1; i < m_label + 1; i++) {
			if (LABEL[i].pix > 100 && LABEL[i].pix > MAXLABEL.pix) {	//if (LABEL[i].pix > 100 && LABEL[i].pix > MAXLABEL.pix) {
				MAXLABEL.pix = LABEL[i].pix;
				MAXLABEL.cX = LABEL[i].cX;
				MAXLABEL.cY = LABEL[i].cY;
				MAXlabelc = i;
				MAXLABEL.maxX = LABEL[i].maxX;
				MAXLABEL.minX = LABEL[i].minX;
				MAXLABEL.maxY = LABEL[i].maxY;
				MAXLABEL.minY = LABEL[i].minY;
			}
		}
	}
	else {
		for (i = 1; i < m_label + 1; i++) {
			if (LABEL[i].pix > 200 && LABEL[i].cY > MAXLABEL.cY) {	//if (LABEL[i].pix > 100 && LABEL[i].pix > MAXLABEL.pix) {
				MAXLABEL.pix = LABEL[i].pix;
				MAXLABEL.cX = LABEL[i].cX;
				MAXLABEL.cY = LABEL[i].cY;
				MAXlabelc = i;
				MAXLABEL.maxX = LABEL[i].maxX;
				MAXLABEL.minX = LABEL[i].minX;
				MAXLABEL.maxY = LABEL[i].maxY;
				MAXLABEL.minY = LABEL[i].minY;
			}
		}
	}
	//////////////////   MAXLABEL 만 MAXlabelOUT에 남기기    //////////////////////
	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (MAXlabelc == 0 || labelOUT[i + m_nWidth * j] != MAXlabelc) {
				MAXlabelOUT[i + m_nWidth * j] = 0;
			}
			else if (labelOUT[i + m_nWidth * j] == MAXlabelc || m_label == 0) {
				MAXlabelOUT[i + m_nWidth * j] = 255;
			}
		}


	Binary8_to_RGB565(MAXlabelOUT, fpga_dst3, m_nWidth, m_nHeight, B_GRAY_MODE);
	draw_img_from_buffer(fpga_dst3, 0, 250, 0, 0, 1.77, 0);

	if (HH == WHT_HH && HL == WHT_HL  && SH == WHT_SH && SL == WHT_SL  && VH == WHT_VH && VL == WHT_VL) {	//흰색일 경우 - 예비용

		float GI_55 = 0, GI_60 = 0, GI_65 = 0, GI_70 = 0, GI_75 = 0, GI_80 = 0;
		float G1 = 0, G2 = 0, G3 = 0, G4 = 0, G5 = 0;

		for (i = 55; i < 81; i = i + 5){
			for (j = 118; j >= 0; j--){
				if (MAXlabelOUT[i + j * m_nWidth] > 0){
					if (i == 55) GI_55++;
					if (i == 60) GI_60++;
					if (i == 65) GI_65++;
					if (i == 70) GI_70++;
					if (i == 75) GI_75++;
					if (i == 80) GI_80++;
				}
			}
		}

		G1 = (GI_60 - GI_55) / 5;
		G2 = (GI_65 - GI_60) / 5;
		G3 = (GI_70 - GI_65) / 5;
		G4 = (GI_75 - GI_70) / 5;
		G5 = (GI_80 - GI_75) / 5;

		*out_cX = MAXLABEL.cX;
		*out_cY = MAXLABEL.cY;
		*out_pix = MAXLABEL.pix;
		*G = (G1 + G2 + G3 + G4 + G5) / 5;
	}

	//밑에서 6개만 빼내서 추세선 그리기
	else if (HH == BLK_HH && HL == BLK_HL  && SH == BLK_SH && SL == BLK_SL  && VH == BLK_VH && VL == BLK_VL) {	//검은색일 경우

		//추세선 기울기 구하기
		float q = 0, w = 0, e = 0, r = 0, sX = 0, sY = 0;
		int numb = 0;
		int cnt = 0;

		for (i = MAXLABEL.minX; i <= MAXLABEL.maxX ; i++) {
			cnt = 0;
			for (j = MAXLABEL.maxY; j >= MAXLABEL.minY; j--) {
				if (MAXlabelOUT[i + m_nWidth*j] == 255 && cnt < 6 ) {
					e = e + i*i;
					r = r + i;
					sX = sX + i;
					q = q + i * (120 - j);
					numb++;
					sY = sY + (120 - j);
					cnt++;
				}
			}
		}
		
		r = r * r;
		e = e * numb;
		q = q * numb;
		w = sY * sX;
		*out_cX = sX / numb;
		*out_cY = (120 - sY / numb);
		*out_pix = MAXLABEL.pix;
		*G = (q - w) / (e - r);
	}
	else if (HH == YELLO_HH && HL == YELLO_HL  && SH == YELLO_SH && SL == YELLO_SL  && VH == YELLO_VH && VL == YELLO_VL) {	
		//추세선 기울기 구하기
		float q = 0, w = 0, e = 0, r = 0, sX = 0, sY = 0;
		int numb = 0;
		int cnt = 0;

		for (i = MAXLABEL.minX; i <= MAXLABEL.maxX; i++) {
			cnt = 0;
			for (j = MAXLABEL.maxY; j >= MAXLABEL.minY; j--) {
				if (MAXlabelOUT[i + m_nWidth*j] == 255 && cnt < 6) {
					e = e + i*i;
					r = r + i;
					sX = sX + i;
					q = q + i * (120 - j);
					numb++;
					sY = sY + (120 - j);
					cnt++;
				}
			}
		}

		r = r * r;
		e = e * numb;
		q = q * numb;
		w = sY * sX;
		*out_cX = sX / numb;
		*out_cY = (120 - sY / numb);
		*out_pix = MAXLABEL.pix;
		*G = (q - w) / (e - r);
	}

	double M = atan2((double)*G, 1) * 180 / 3.14159265;
	/*
	float gi = *G, b = 0;
	int cx = *out_cX, cy = *out_cY, y = 0;

	b = gi * cx + cy;

	for (i = 0; i < 180; i++) {
		y = -1 * gi * i + b;
		if (y > 0 && y < 120)
			draw_rectfill(160 - 1.3*y, 240 + 1.3*i, 2, 2, MAKE_COLORREF(255, 0, 255));
	}
	*/
	
	//printf("cx : %d, cy : %d , pix : %d, G : %f, ATAN : %f \n", *out_cX ,*out_cY, *out_pix , *G , M);

	flip();

}


//평균값과 표준편차 정보 찾는 함수
void INFO_MEAN_VAR(
	U16* fpga_src, U16* fpga_dst1, U16* fpga_dst2, U16* fpga_dst3, U8* imgBGR, U8* Filt,
	U8* imgVSH, U8* imgThresholded_H, U8* imgThresholded_S, U8* imgThresholded_V,
	U8* imgThresholded, U8* labelOUT, U8* MAXlabelOUT,
	const int HL, const int HH, const int SL, const int SH, const int VL, const int VH, //INPUT
	int* out_cX, int* out_cY, int* out_pix, float* mean, float* std_dev 	//OUPUT
	)
{
	clear_screen();
	read_fpga_video_data(fpga_src);
	draw_img_from_buffer(fpga_src, 0, 18, 0, 0, 1.77, 0);

	RGB565_to_BGR888(fpga_src, imgBGR, m_nWidth, m_nHeight);	//565 -> 888
	MeanFiltering(imgBGR, Filt, m_nWidth, m_nHeight);			//Median filterling
	BGR888_to_VSH888_30(Filt, imgVSH, m_nWidth, m_nHeight);		//RGB -> HSV

	VSH888_to_H8_thresh(imgVSH, imgThresholded_H, m_nWidth, m_nHeight, HL, HH);	//H
	VSH888_to_S8_thresh(imgVSH, imgThresholded_S, m_nWidth, m_nHeight, SL, SH);	//S
	VSH888_to_V8_thresh(imgVSH, imgThresholded_V, m_nWidth, m_nHeight, VL, VH);	//V

	if(HL == YELLO_HL && HH == YELLO_HH )
		HSV_to_Binary8_R(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, m_nWidth, m_nHeight);
	else
		HSV_to_Binary8(imgThresholded_H, imgThresholded_S, imgThresholded_V, imgThresholded, m_nWidth, m_nHeight);

	//바운더리 예외처리
	int i = 0, j = 0;
	for (j = 0; j < m_nHeight; j++) {		//왼쪽 테두리
		imgThresholded[m_nWidth * j] = 0;
		for (i = 0; i < 5; i++)		imgThresholded[i + m_nWidth * j] = 0;
	}
	for (i = 0; i < m_nWidth; i++)			//위쪽 테두리
		imgThresholded[i] = 0;

	if (HL == YELLO_HL && HH == YELLO_HH){	//노란색 예외처리
		for (j = 75; j < 120; j++)
			for (i = 0; i < m_nWidth; i++)		imgThresholded[i + m_nWidth * j] = 0;
	}

	int label = 0, MAXlabelc = 0;
	const int MAX_LABEL = 1000;
	int eq_tbl[MAX_LABEL][2];
	for (i = 0; i < MAX_LABEL; i++) {
		eq_tbl[i][0] = 0;
		eq_tbl[i][1] = 0;
	}

	int maxl = 0, minl = 0, min_eq = 0, max_eq = 0;
	int L = 0, R = 0, U = 0, D = 0;

	for (j = 1; j < m_nHeight - 1; j++) {
		for (i = 1; i < m_nWidth - 1; i++) {
			L = i - 1;	//왼
			R = i + 1;	//오
			U = j - 1;	//위
			D = j + 1;	//아래

			if (imgThresholded[i + m_nWidth * j] == 255) {

				// 바로 위 픽셀과 왼쪽 픽셀 모두에 레이블이 존재하는 경우
				if ((imgThresholded[L + m_nWidth * j] != 0) && (imgThresholded[i + m_nWidth * U] != 0)) {

					//위, 왼 레이블이 같음
					if (imgThresholded[L + m_nWidth * j] == imgThresholded[i + m_nWidth * U])
						imgThresholded[i + m_nWidth * j] = imgThresholded[i + m_nWidth * U];

					else { // 두 레이블이 서로 다른 경우, 작은 레이블을 부여

						maxl = MAX(imgThresholded[L + m_nWidth * j], imgThresholded[i + m_nWidth * U]);
						minl = MIN(imgThresholded[L + m_nWidth * j], imgThresholded[i + m_nWidth * U]);

						imgThresholded[i + m_nWidth * j] = minl;          //픽셀에 작은값 대입
																		  //등가 테이블 조정
						min_eq = MIN(eq_tbl[maxl][1], eq_tbl[minl][1]);
						max_eq = MAX(eq_tbl[maxl][1], eq_tbl[minl][1]);

						eq_tbl[eq_tbl[max_eq][1]][1] = min_eq;
					}

				}

				// 바로 위 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[i + m_nWidth * U] != 0)
					imgThresholded[i + m_nWidth * j] = imgThresholded[i + m_nWidth * U];

				//왼쪽 픽셀에만 레이블이 존재할 경우
				else if (imgThresholded[L + m_nWidth * j] != 0)
					imgThresholded[i + m_nWidth * j] = imgThresholded[L + m_nWidth * j];

				//주위에 레이블이 없음. 새로운 레이블 부여
				else {

					label++;
					imgThresholded[i + m_nWidth * j] = label;
					eq_tbl[label][0] = label;
					eq_tbl[label][1] = label;
				}
			}
		}

	}

	int temp = 0;
	for (i = 1; i <= label; i++) {
		temp = eq_tbl[i][1];
		if (temp != eq_tbl[i][0])	eq_tbl[i][1] = eq_tbl[temp][1];
	}

	// 등가 테이블의 레이블을 1부터 차례대로 증가시키기
	int hash[label + 1];
	for (i = 0; i <= label; i++)	hash[i] = 0;

	for (i = 1; i <= label; i++)	hash[eq_tbl[i][1]] = eq_tbl[i][1];

	int label_cnt = 1;
	for (i = 1; i <= label; i++)	if (hash[i] != 0) hash[i] = label_cnt++;

	int m_label = 0;
	for (i = 1; i <= label; i++) {
		eq_tbl[i][1] = hash[eq_tbl[i][1]];
		if (m_label < eq_tbl[i][1])	m_label = eq_tbl[i][1];
	}

	for (j = 0; j < m_nHeight; j++)
		for (i = 0; i< m_nWidth; i++) {
			labelOUT[i + m_nWidth * j] = 0;
			MAXlabelOUT[i + m_nWidth * j] = 0;
		}

	int idx = 0;
	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (imgThresholded[i + m_nWidth * j] != 0) {
				idx = imgThresholded[i + m_nWidth * j];
				labelOUT[i + m_nWidth * j] = eq_tbl[idx][1]; // eq_tbl[idx][1]가 255이상이면 
			}
		}

	struct Label LABEL[m_label + 1];
	struct Label MAXLABEL = { 0, 0, 0, 0, 0, 0, 180, 0, 120 };

	int m = 0;
	for (m = 0; m < m_label + 1; m++) {
		LABEL[m].sumX = 0;
		LABEL[m].sumY = 0;
		LABEL[m].pix = 0;
		LABEL[m].minX = 255;
		LABEL[m].maxX = 0;
		LABEL[m].minY = 255;
		LABEL[m].maxY = 0;
	}

	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (labelOUT[i + m_nWidth * j] != 0) {
				LABEL[labelOUT[i + m_nWidth * j]].pix++;
				LABEL[labelOUT[i + m_nWidth * j]].sumX += i;
				LABEL[labelOUT[i + m_nWidth * j]].sumY += j;

				if (i > LABEL[labelOUT[i + m_nWidth * j]].maxX) LABEL[labelOUT[i + m_nWidth * j]].maxX = i;
				if (i < LABEL[labelOUT[i + m_nWidth * j]].minX) LABEL[labelOUT[i + m_nWidth * j]].minX = i;
				if (j > LABEL[labelOUT[i + m_nWidth * j]].maxY) LABEL[labelOUT[i + m_nWidth * j]].maxY = j;
				if (j < LABEL[labelOUT[i + m_nWidth * j]].minY) LABEL[labelOUT[i + m_nWidth * j]].minY = j;
			}
		}

	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (labelOUT[i + m_nWidth * j]>0 && LABEL[labelOUT[i + m_nWidth * j]].pix > 0) {
				LABEL[labelOUT[i + m_nWidth * j]].cX = LABEL[labelOUT[i + m_nWidth * j]].sumX / LABEL[labelOUT[i + m_nWidth * j]].pix;
				LABEL[labelOUT[i + m_nWidth * j]].cY = LABEL[labelOUT[i + m_nWidth * j]].sumY / LABEL[labelOUT[i + m_nWidth * j]].pix;
			}
		}

	for (i = 1; i < m_label + 1; i++) {
		if (LABEL[i].pix > 100 && LABEL[i].pix > MAXLABEL.pix) {
			MAXLABEL.pix = LABEL[i].pix;
			MAXLABEL.cX = LABEL[i].cX;
			MAXLABEL.cY = LABEL[i].cY;
			MAXlabelc = i;
			MAXLABEL.maxX = LABEL[i].maxX;
			MAXLABEL.minX = LABEL[i].minX;
			MAXLABEL.maxY = LABEL[i].maxY;
			MAXLABEL.minY = LABEL[i].minY;
		}
	}

	//////////////////   MAXLABEL 만 MAXlabelOUT에 남기기    //////////////////////
	for (j = 1; j < m_nHeight - 1; j++)
		for (i = 1; i < m_nWidth - 1; i++) {
			if (MAXlabelc == 0 || labelOUT[i + m_nWidth * j] != MAXlabelc) {
				MAXlabelOUT[i + m_nWidth * j] = 0;
			}
			else if (labelOUT[i + m_nWidth * j] == MAXlabelc || m_label == 0) {
				MAXlabelOUT[i + m_nWidth * j] = 255;
			}
		}

	Binary8_to_RGB565(MAXlabelOUT, fpga_dst3, m_nWidth, m_nHeight, B_GRAY_MODE);
	draw_img_from_buffer(fpga_dst3, 0, 250, 0, 0, 1.77, 0);

	//평균 표준편차 구하기
	int tp_minY = MAXLABEL.minY + 10;
	int tp_maxY = MAXLABEL.maxY -10;

	int MidPoint[tp_maxY - tp_minY + 1];
	for (i = 0; i <= tp_maxY - tp_minY; i++)	MidPoint[i] = 0;
	int MPcnt[tp_maxY - tp_minY + 1];
	for (i = 0; i <= tp_maxY - tp_minY; i++)	MPcnt[i] = 0;

	int tp = 0;
	for (j = tp_minY; j <= tp_maxY; j++) {
		tp = j - tp_minY;
		for (i = MAXLABEL.minX; i <= MAXLABEL.maxX; i++) {
			if (MAXlabelOUT[i + m_nWidth*j] == 255){
				MidPoint[tp] = MidPoint[tp] + i;
				MPcnt[tp]++;
			}
		}
		MidPoint[tp] = MidPoint[tp] / MPcnt[tp];
	}

	int n = sizeof(MidPoint) / sizeof(int);
	float sum = 0, var = 0;
	for (i = 0; i<n; i = i + 1)
		sum += MidPoint[i];
	*mean = sum / n;		//평균

	sum = 0;
	for (i = 0; i<n; i = i + 1)
		sum += (MidPoint[i] - *mean)*(MidPoint[i] - *mean);
	var = sum / (n - 1);
	*std_dev = sqrt(var);		//표준편차

	//printf("MEAN = %8.3f	VAR = %8.3f	STAN DEV = %8.3f\n", *mean,var, *std_dev);

	*out_cX = MAXLABEL.cX;
	*out_cY = MAXLABEL.cY;
	*out_pix = MAXLABEL.pix;

	flip();

}