// ==========================================================================
//
//  EMG - Signal Amplifier
//
//  Copyright (c) Theresa Roland / 2017
//	The copyright of this software belongs exclusively to the author!
//	Redistribution and use, with or without modification are not permitted!
// ==========================================================================
// 
//     MAIN-Program
//
// --> Tools
// --> Callbacks (Interrupts)
// --> Initialisation
// --> MAIN-Loop
//
// ==========================================================================


#include <asf.h>
#include "arm_math.h"
#include <Configuration.h>
#include <BLE.h>
#include <DEC.h>
//#include <stdio.h>
//#include <stdlib.h>


// ==========================================================================
// ===================== FUNCTION DECLARATIONS ==============================
//

// ======= Reference signal - Calibration - Function Declarations ===========
void	CLBR_Reference (int32_t * CLBR_in, int16_t CLBR_LEN);

// ====== Daub4 Wavelet Transformation "DAUB4" - Function Declarations ======
void	DAUB4_Tran	(q15_t *x, q15_t *y, int16_t LEN, int16_t FULL);

// === Short Time Fourier Transformation "STFT" - Function Declarations =====
void	STFT_Init	(void);
void	STFT_Write_Input (q15_t STFT_Input);
void	STFT_Write_Output (void);
void	STFT_abs	(q15_t * ABS_IN, q31_t * ABS_OUT, int16_t ABS_LEN);
void	STFT_Write_to_DAC (void);
void	STFT_Write_to_BLE (void);
void	STFT_smoothed_Write_to_BLE (void);

// ========== Evaluation of FFT "EVAL_FFT" - Function Declarations ===========
int8_t	EVAL_FFT	(int32_t * EVAL_in, int32_t * EVAL_out, int16_t EVAL_LEN);

// ========= Gain-Offset-Control "GOC" - Function Declarations ===============
void	GOC_signal	(int32_t GOC_Signal);
void	GOC_output	(void);

// =================== Combfilter - Function Declarations ====================
int16_t	COMB_Filter	(int16_t COMB_In);

// =================== Notch - Function Declarations ====================
int16_t	NOTCH_Filter	(uint16_t NOTCH_In);

// =================== Hochpassfilter - Function Declarations ================
int16_t HP_Filter(int16_t HP_In);

// ==== Absolute value and Moving average filter - Function Declarations =====
void	ABS_MAF		(int16_t ABS_MAF_Input, uint8_t removeNoiseLevel);

// =============    BLE - Service - Menu  ====================================
uint16_t ble_menu	(char * ble_menu_cmd);


// dwt declarations

void wpt( q15_t *s, q15_t *B);
void dwt1(q15_t *s_in, q15_t *out, int16_t d);
void dwt2(q15_t *s_in, q15_t *out, int16_t d, int16_t step, int16_t ind);
void dwt3( q15_t *s_in, q15_t *out, int16_t d, int16_t step, int16_t ind);
void dwt4( q15_t *s_in, q15_t *out, int16_t d, int16_t step, int16_t ind);
int16_t eval_wpt(q15_t *signal_to_check, int16_t maximum);
void conv(q15_t *x, q15_t *h, q15_t *Y, q15_t len_h, int16_t ind);
void normalize_vector (q15_t *inout, int16_t length);

// ==========================================================================
// ============= VARIABLE DECLARATIONS + INITIALIZATIONS ====================
//

#define				DAC_MAX				4095			// Max. DAC Output Value (12bit DAC)

static int64_t		I64_4096		 =	4096;
static int64_t		I64_4096_shifted =	4294967296LL;	// 4294967296LL = 4096*2^20
static uint16_t		dac_val =			0;
static uint16_t		adc_result		 =	0;


static uint8_t		main_DEC_status_old = 0;
static uint8_t		main_DEC_status	 =	0;
static uint8_t		main_DEC_CLBR_enable = 0;


// ============== NVM-Memory - Variable Declarations + Initializations ======================

static int16_t		page_buffer_read[NVMCTRL_PAGE_SIZE*8];

// === Daub4 Wavelet Transformation "DAUB4" - Variable Declarations + Initializations =======
//#define				DAUB4_LEN			32				// Block length
//static q15_t		DAUB4_Input[DAUB4_LEN]	=	{2, 4095, 200, 2, 2, 2, 4095, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 4095, 200, 2, 2, 2, 4095, 0, 0, 2, 2, 2, 2, 2, 2, 2};
//static q15_t		DAUB4_Output[DAUB4_LEN];

// === Short Time Fourier Transformation "STFT" - Variable Declarations + Initializations ===
#define				STFT_Freq_divide	5				// Divider to calculate ST-FFT with reduced Frequency
#define				STFT_LEN			512				// Block length 512
arm_rfft_instance_q15	STFT_instance_q15;
static int16_t		STFT_Input_mean_CNT=0;
static q15_t		STFT_Input_mean	=	0;
static q15_t		STFT_Input_transmit=0;
static q15_t		STFT_Window[STFT_LEN];
static q15_t		STFT_Input1[STFT_LEN];
static int16_t		STFT_Input1_CNT	=	0;
static int8_t		STFT_Input1_OK	=	0;
static q15_t		STFT_Output1[STFT_LEN*2];
static q15_t		STFT_Input2[STFT_LEN];
static int16_t		STFT_Input2_CNT	=	(STFT_LEN*3)/4;
static int8_t		STFT_Input2_OK	=	0;
static q15_t		STFT_Output2[STFT_LEN*2];
static q31_t		STFT_Output[STFT_LEN/2];
static int8_t		STFT_Output_DAC_OK= 0;
static int8_t		STFT_Output_BLE_OK=	0;
static int8_t		STFT_Output_EVAL_OK=0;
static int8_t		STFT_BLE_isFirstRun = 1;
static uint8_t		STFT_On				= 0;

// ======== Reference signal - Calibration - Variable Declarations + Initializations ========
static int32_t		CLBR_FFT_Sum[STFT_LEN/2]={0};
static int32_t		CLBR_smooth[STFT_LEN/2]={0};
static uint8_t		CLBR_enable		=	0;
static uint8_t		CLBR_count		=	1;
static uint32_t		CLBR_count2		=	0;
static uint8_t		CLBR_start		=	0;
static int32_t		CLBR_Num_FFT	=	16;

// ======= Evaluation of FFT "EVAL_FFT" - Variable Declarations + Initializations ===========
#define				EVAL_WINDOW_SIZE	13
#define				EVAL_NUM_MAX		6
#define				EVAL_NUM_COMP		STFT_LEN/2
static int32_t		EVAL_Output[STFT_LEN/2];
static int32_t		EVAL_sortedWindow[EVAL_WINDOW_SIZE];
static int16_t		ii				=	0;
static int16_t		kk				=	0;
static int16_t		ll				=	0;
static int64_t		EVAL_Sum		=	0;
//static int64_t		EVAL_Thresh		=	100*4096;
static int64_t		EVAL_Thresh		=	16*4096;
static int64_t		EVAL_Diff		=	0;
static int8_t		EVAL_Emg_FD		= 0;
static int8_t		EVAL_Emg_FD_old = 0;
static int64_t		EVAL_max_val;
static int64_t		EVAL_Ref[STFT_LEN/2]={4, 24, 57, 211, 249, 274, 360, 420, 492, 625, 819, 1026, 1127, 1187, 1669, 1787, 2129, 2327, 2745, 3136, 3344, 3753, 3923, 3943, 4096, 4070, 4056, 3809, 3846, 3639, 3625, 3456, 3550, 3672, 3318, 3202, 3190, 2893, 2861, 2865, 2771, 2689, 2534, 2446, 2248, 1749, 1482, 1405, 1253, 1057, 1049, 1049, 1030, 953, 854, 743, 645, 546, 556, 498, 495, 536, 517, 508, 508, 528, 522, 603, 608, 643, 721, 670, 664, 638, 646, 663, 652, 652, 640, 613, 547, 556, 542, 474, 452, 439, 402, 347, 310, 306, 304, 299, 294, 248, 212, 166, 151, 136, 127, 130, 118, 101, 101, 101, 101, 97, 90, 86, 85, 78, 74, 70, 64, 68, 66, 67, 67, 73, 68, 65, 69, 64, 63, 60, 60, 55, 50, 50, 50, 50, 43, 42, 42, 34, 33, 27, 25, 23, 23, 20, 18, 18, 17, 17, 15, 14, 13, 13, 12, 11, 11, 11, 11, 11, 11, 11, 10, 11, 10, 10, 10, 10, 9, 9, 8, 7, 7, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
static uint8_t		EVAL_Output_OK=0;
static uint8_t		EVAL_weighting[EVAL_NUM_COMP] = {0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2
};
// ========== Gain-Offset-Control "GOC" - Variable Declarations + Initializations ===========
#define				GOC_Fs				10000		// Sampling Frequency [Hz]
#define				GOC_TIME			1			// Sampling time [s]
#define				GOC_MAX				(1<<12)-1	// 12bit ADC
#define				GOC_Deadband		10			//
#define				GOC_Gain_plus		15			// Increase Gain delay
static int32_t		GOC_Signal_max	=	0;
static int32_t		GOC_Signal_min	=	0;
static int32_t		GOC_Signal_avg	=	0;
static int32_t		GOC_Signal_cnt	=	0;
static int8_t		GOC_Signal_OK	=	0; // 0->Sense; 1->Output; 2->Wait
//static int8_t		GOC_Gain_step	=	1;
//static int8_t		GOC_Gain_step_old=	1;
static int8_t		GOC_Gain_step	=	3;
static int8_t		GOC_Gain_step_old=	3;
static int16_t		GOC_Gain_plus_cnt=	0;
static int16_t		GOC_Gain		=	0;
//static uint16_t		GOC_Offset		=	2048;
static uint16_t		GOC_Offset		=	512;


// ============= Combfilter - Variable Declarations + Initializations =======================
#define				COMB_Fs				10000
#define				COMB_f				50
#define				COMB_Fs_k			COMB_Fs/COMB_f
#define				COMB_Samples		4*COMB_Fs_k+1
static int32_t		COMB_Fs_k1		=	0;
static int32_t		COMB_Fs_k2		=	0;
static int32_t		COMB_Fs_k3		=	0;
static int32_t		COMB_Fs_k4		=	0;
static int32_t		COMB_N			=	0;
static int16_t		COMB_Input[COMB_Samples];
static int16_t		COMB_Output		=	0;

// ============= Notchfilter - Variable Declarations + Initializations =======================
#define				NOTCH_Fs			10000
#define				NOTCH_f				50
#define				NOTCH_Fs_k			NOTCH_Fs/NOTCH_f
#define				NOTCH_Samples		2*NOTCH_Fs_k+1
#define				NOTCH_WS			3
static int16_t		NOTCH_Fs_k1		=	0;
static int16_t		NOTCH_Fs_k2		=	0;
static int16_t		NOTCH_N			=	0;
static int32_t		NOTCH_Center[NOTCH_Samples];
static int64_t		NOTCH_count		=	0;

// ============= Hochpassfilter - Variable Declarations + Initializations ==================
#define				HP_Samples			3
#define				HP_WS				3
static int16_t		HP_In[HP_Samples];
static int32_t		HP_Out[HP_Samples];
static int16_t		HP_Output		=	0;
int64_t				HP_count		= 0;

// === Absolute value and Moving average filter - Variable487x_initble Declarations + Initializations ===
#define				ABS_MAF_DATAPOINTS	9			// = 2^10 Datapoints
static int16_t		ABS_MAF_Output	=	0;
static int64_t		ABS_MAF_mean	=	0;


// === I2C - Interface to ARDUINO  ============================
static struct i2c_slave_packet packet;
#define	SLAVE_ADDRESS			0x08
#define CONF_I2C_SLAVE_MODULE   SERCOM5
#define DATA_LENGTH				6
static uint8_t write_buffer[DATA_LENGTH] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05 };
static uint8_t read_buffer [DATA_LENGTH];



// ==== DWPT 

// static used variables for the dwt
static q15_t dv1[8] = {-1, 3, 3, -19, -3, 65, 73, 24};

static q15_t dv2[8] = {-24, 73, -65, -3, 19, 3, 3, -1};

static q15_t norm_cust_con[9] = {19,32,	15,	28,	5,	6,	12,	9,	3};
	
static q15_t Con1_vector [1024] = {-3,-3,-2,0,1,2,1,-2,-4,-4,-4,-2,0,1,1,2,2,2,-2,-2,-1,1,-1,-2,-2,-3,-4,-4,-2,-1,-2,-2,-1,1,2,2,2,2,1,-3,-6,-6,-4,-2,0,2,5,8,8,6,2,-1,-3,-3,-3,1,3,5,5,4,3,1,-1,-6,-8,-8,-6,-6,-7,-4,4,6,6,6,5,5,6,9,7,3,-3,-2,-5,-8,-8,-9,-9,-7,-4,-3,-2,-3,-3,-3,2,4,2,2,5,8,8,5,1,-1,-3,-3,-3,-3,-6,-7,-6,-4,-4,-3,4,5,5,2,-2,-4,-5,-4,-2,2,2,4,6,5,5,1,-4,-5,-4,-6,-7,-7,-7,-6,-2,1,4,2,-2,-3,0,4,4,-4,-7,-6,-3,-2,-5,-6,-3,-2,2,3,5,4,3,4,1,-2,-2,-2,-2,-4,-4,-6,-4,2,4,3,4,6,6,5,1,-5,-4,-3,-1,2,2,1,1,2,2,4,3,1,-1,-2,-2,-2,-2,1,2,2,2,3,1,1,-2,-4,-3,-3,-2,3,5,5,6,7,5,4,1,-2,-4,-7,-8,-6,-6,-4,-4,-3,2,3,4,3,2,1,1,4,5,4,4,2,0,-3,-3,-5,-5,-4,-4,-3,-3,-4,-4,-3,-2,-2,-3,-4,-1,1,5,4,3,2,2,1,1,0,2,2,2,-2,-4,-3,-4,-3,-3,-3,-2,-1,-1,-3,-2,-2,-3,-1,1,1,1,-2,-2,-3,-2,3,4,4,1,-3,-6,-7,-7,-7,-7,-6,-1,3,5,5,4,4,3,1,-2,-2,2,2,2,-3,-4,-3,-5,-3,-2,2,1,1,2,1,1,1,0,-2,1,-1,-3,-3,-3,-4,-2,1,2,3,4,5,5,4,3,2,-1,-2,-2,-2,2,2,0,1,3,1,1,1,4,6,7,7,5,2,1,-2,-2,-4,-6,-8,-6,-6,-6,-3,-2,2,6,7,8,6,5,3,-3,-4,-5,-4,-3,-2,-1,-2,0,-2,-3,-3,-1,0,-2,-5,-8,-7,-3,-2,0,0,3,4,4,2,1,-3,-7,-9,-9,-10,-8,-6,2,4,5,4,2,-1,-2,-3,-5,-4,2,2,1,-1,-4,-7,-6,-6,-7,-6,-3,1,4,5,3,0,-1,0,1,-1,-2,-1,-2,-1,2,4,5,5,4,3,0,-2,-2,-2,-3,-3,-1,2,3,5,5,3,3,2,-2,-4,-4,-2,1,3,3,3,3,1,-2,-4,-4,-3,1,2,6,6,5,2,-3,-3,-3,-1,3,3,4,4,4,6,1,-3,-7,-6,-6,-4,-2,0,1,0,1,3,5,5,3,1,1,0,-2,-2,-2,-2,-4,-3,-2,2,1,1,-2,-2,-3,-3,-3,-1,-2,-2,1,5,8,6,4,1,1,-1,-2,-2,-4,-6,-6,-4,-3,-3,-1,1,1,2,2,1,1,2,1,-2,-4,-5,-2,1,1,-3,-5,-4,-3,-1,2,3,2,-1,3,4,7,6,5,5,1,-5,-10,-11,-7,-4,-3,-4,-2,-3,-4,-3,2,2,2,1,-2,-6,-3,2,6,11,11,9,6,2,-2,-10,-15,-17,-14,-5,1,1,2,2,2,-4,-6,-5,-1,0,1,3,4,4,1,3,5,1,-1,0,3,1,-2,-5,-4,-4,-4,-6,-8,-4,1,3,5,4,6,7,5,3,-2,-2,-2,-4,-4,2,6,6,6,3,2,3,5,-4,-4,-3,-2,-3,-4,-4,3,6,6,2,-2,-6,-3,-2,-1,-5,-6,-5,-2,1,3,6,6,4,2,1,2,0,-2,-5,-9,-12,-11,-11,-13,-10,-4,3,6,9,8,9,10,7,4,2,-2,-2,0,0,-2,-7,-11,-7,-5,-4,-3,1,5,6,6,3,2,5,7,6,7,7,6,4,-6,-7,-5,-2,-2,-6,-10,-12,-13,-13,-9,-3,1,4,5,4,4,3,1,1,3,1,-2,0,-1,-2,-4,-5,-6,-6,-5,-7,-9,-7,-4,1,6,9,12,12,7,3,-3,-7,-9,-5,-4,-6,-4,5,6,3,-4,-7,-9,-7,-3,1,3,3,6,9,6,0,-2,-4,-7,-9,-7,-4,-2,4,8,13,13,10,5,4,1,-3,-4,-2,-6,-8,-6,2,-2,-3,-7,-11,-10,-6,5,7,6,8,11,12,9,4,0,1,-5,-8,-10,-6,-2,0,1,5,9,9,7,5,1,-4,-8,-9,-7,-2,5,9,7,-2,-2,-1,-1,-3,-2,1,2,1,-2,-2,-2,-2,-2,-4,-7,-10,-9,-5,2,4,3,3,-2,-3,-3,1,2,5,9,9,4,1,2,1,-5,-7,-8,-8,-6,-3,-2,-4,-2,1,5,10,9,4,-2,-6,-6,-5,-3,-3,-4,-5,-5,-4,-2,-2,1,4,7,6,3,-3,-7,-14,-15,-10,-2,4,8,9,10,9,3,-2,-8,-10,-7,-4,1,4,9,11,7,3,0,-1,1,1,-2,-2,-4,-6,-2,3,5,4,1,-3,3,3,-3,-5,-5,-2,-2,1,6,7,5,1,-3,-7,-8,-8,-7,-6,-6,-6,-4,-4,-3,2,11,18,22,20,14,10,4,-2,-4,-7,-7,-8,-7,-8,-9,-8,-4,5,8,4,-5,-9,-10,-11,-9,-3,3,8,12,16,18,12,3,-6,-14,-19,-23,-22,-15,-7};
static q15_t Con2_vector [1024] = {-1,3,5,6,7,7,6,5,5,1,-3,-7,-10,-12,-10,-4,0,1,5,9,11,10,8,4,0,-3,-3,-1,1,2,-1,-5,-7,-8,-6,-1,3,2,1,1,0,1,0,0,0,-1,-1,2,2,1,1,4,4,4,2,2,0,-1,-2,-5,-8,-12,-10,-7,-5,-3,-3,-3,-2,-3,0,4,7,8,8,5,2,1,1,0,-3,-6,-10,-10,-6,-3,-1,-1,1,7,11,13,9,3,-2,-7,-10,-11,-11,-10,-9,-7,-3,0,1,4,6,3,2,1,0,-1,1,1,1,1,2,2,0,0,0,1,1,0,-1,0,0,0,-2,-6,-7,-6,-1,3,6,3,0,-1,-2,-3,-3,-2,1,2,3,2,0,-2,-1,-1,-2,-4,-5,-6,-6,-4,-2,0,2,4,5,9,8,5,1,-2,-5,-7,-6,-5,-2,-1,-3,-3,-1,3,5,7,5,4,2,0,1,1,0,1,2,3,3,2,2,1,2,1,-1,-5,-7,-7,-5,-3,-1,-2,0,2,3,3,1,0,1,4,3,4,1,-1,-4,-4,-1,-1,-1,-1,0,1,3,0,-5,-9,-8,-4,0,2,2,3,4,6,7,6,4,2,1,0,-2,-3,-5,-7,-7,-8,-6,-3,-2,-2,-4,-3,-1,1,2,-1,-3,-4,-1,-2,-1,-4,-6,-6,-2,1,4,2,-2,-4,-2,-1,-2,-1,-1,4,7,11,9,5,0,-5,-3,-3,-4,-2,-2,-2,-2,-2,-1,-1,-1,0,0,0,-3,-5,-8,-8,-5,-1,1,-1,0,3,6,6,5,0,-3,-4,-1,1,1,3,3,3,-1,-3,-4,-3,-1,0,2,4,5,3,0,-5,-9,-11,-10,-7,-3,0,3,4,8,10,10,4,0,-1,-1,-2,-3,-5,-6,-4,0,3,4,2,-2,-7,-10,-10,-9,-6,-2,-2,-2,0,-1,-1,1,5,7,11,16,18,14,9,4,1,0,0,-1,-1,-2,-4,-3,0,0,-6,-9,-8,-4,0,3,2,0,-3,-3,-2,-3,-5,-7,-5,-2,1,3,1,1,0,-1,-3,-5,-9,-10,-10,-8,-2,2,7,11,13,11,4,2,3,4,3,3,4,6,4,2,-4,-9,-14,-16,-14,-12,-10,-10,-5,1,8,12,12,8,5,4,4,2,1,1,-1,-8,-12,-11,-7,-2,1,3,4,4,5,4,0,-1,-1,-4,-8,-9,-8,-4,0,3,5,6,6,5,1,-1,-3,-3,-4,-6,-6,-5,-8,-8,-4,0,4,6,9,12,14,11,4,-4,-10,-14,-14,-12,-13,-8,-4,1,1,3,4,5,8,7,5,0,-1,2,2,2,2,4,4,3,0,-1,0,1,4,5,2,-1,0,-1,0,1,1,1,4,4,2,-5,-9,-10,-8,-6,-9,-10,-11,-6,-1,4,6,6,7,7,3,0,-2,-1,2,5,5,2,0,-2,-2,-1,2,1,-1,-2,-4,-5,-3,-1,-2,-2,-4,-5,-7,-7,-4,-1,3,8,8,8,8,10,8,2,-5,-7,-6,-4,-4,-4,-3,-1,0,-1,-1,-1,-1,-2,-5,-6,-3,1,6,4,3,0,-2,-2,-3,-2,-1,1,2,2,2,0,-1,-1,-3,-2,-2,-3,-3,-4,-4,-3,0,0,2,2,1,-3,-5,-5,-2,0,1,3,7,10,10,4,-1,-6,-7,-8,-9,-8,-8,-8,-6,-3,2,5,5,2,1,3,5,8,7,1,-2,-3,-2,-4,-6,-7,-6,-4,0,2,3,2,3,6,10,10,7,0,-7,-12,-11,-6,-4,-5,-5,0,4,8,8,6,4,6,9,10,8,2,0,-5,-5,-5,-6,-6,-6,-9,-11,-12,-8,-1,4,8,10,8,6,4,0,-7,-11,-9,-5,0,1,1,2,3,3,1,-1,-7,-7,-5,-4,-4,-3,-1,1,2,2,-1,-4,-5,-3,0,4,8,12,14,12,8,2,-5,-10,-10,-10,-10,-9,-8,-8,-7,-5,0,2,1,1,6,7,6,5,3,2,2,-1,-5,-10,-9,-7,-7,-4,-1,4,8,13,13,8,0,-4,-5,-2,-3,-5,-6,-7,-5,-4,-5,-3,-1,2,1,0,-3,-3,0,4,4,3,2,2,2,3,2,1,0,-1,-1,-4,-5,-3,-2,-4,-6,-4,-3,1,9,9,6,5,6,8,7,4,0,-4,-5,-8,-9,-8,-6,-3,2,3,2,1,0,-1,-1,-1,0,1,2,5,6,3,1,-1,0,-4,-7,-6,-3,-1,2,2,1,2,1,0,-2,-4,-1,3,5,4,3,3,4,5,4,3,1,2,4,6,3,0,-4,-4,-4,-7,-10,-12,-9,-2,5,7,5,1,0,0,0,-2,-7,-10,-12,-9,-6,-6,-4,-2,0,3,7,7,5,4,4,4,1,-2,-2,-4,-6,-7,-7,-6,-6,-4,-2,-1,2,3,3,0,-2,-1,2,7,11,11,8,4,-2,-7,-10,-8,-7,-7,-4,-1,2,1,-2,-4,-4,-4,-5,-2,0,0,-1,-2,-1,-1,1,3,4,4,3,1,-2,-2,1,0,-3,-4,-2,-2,-4,-4,-2,-1,1,0,2,2,4,8,6,0,-3,-6,-9,-9,-4,-1,2,3,2,2,3,3,3,1,0,0,1,0,2,2,1,-1,-2,-4,-3,-5,-5,-5,-3,-3};
static q15_t Art1_vector [1024] = {2,1,0,-2,0,3,4,1,0,6,13,15,5,1,1,4,6,5,2,1,-4,-10,-14,-14,-13,-10,-4,-1,-2,-2,3,6,6,0,-5,-7,-9,-9,-7,-6,-6,-4,-2,0,2,4,5,5,4,4,3,0,1,1,0,1,-1,-3,-4,-4,-3,1,1,0,0,2,3,2,0,1,3,5,6,5,7,9,9,8,6,2,0,0,0,-1,0,2,4,5,5,5,4,4,5,4,2,-1,-3,-2,-3,-4,-3,-2,-2,-3,-3,-3,-2,-2,-1,-1,-2,-2,-2,-4,-5,-5,-5,-4,-2,-2,1,1,1,2,2,3,5,5,2,-2,-1,-2,-2,-2,-2,-2,-2,-4,-5,-5,-3,0,0,1,1,1,2,3,6,8,8,7,6,3,-4,-7,-9,-8,-6,-4,-2,0,1,1,-1,-2,-2,0,2,1,-2,-2,-3,-4,-4,-3,-2,-2,-4,-7,-8,-9,-9,-7,-5,-4,-2,-1,-2,-2,-2,-2,-1,1,2,4,5,6,5,4,3,3,3,2,2,3,1,-1,-2,-3,-2,0,1,1,0,0,0,1,2,2,2,1,1,-1,0,2,2,3,2,1,1,1,1,1,0,1,1,0,2,3,4,4,4,3,2,-2,-3,-4,-5,-4,-4,-4,-5,-5,-4,-4,-3,-2,-2,-3,-2,-2,-2,-2,1,2,2,2,2,1,1,1,1,1,-2,-2,-1,0,-2,-3,-5,-6,-6,-6,-6,-6,-6,-6,-7,-7,-8,-8,-8,-8,-7,-6,-5,-5,-4,-3,-3,-3,-3,-8,-10,-11,-11,-11,-11,-10,-10,-8,-7,-5,-5,-4,-4,-4,-4,-4,-4,-4,-4,-3,-2,1,1,2,3,2,1,-3,-5,-6,-7,-8,-8,-8,-8,-8,-8,-3,0,1,1,2,2,2,1,-2,-1,1,1,1,1,1,2,1,0,-2,-3,-4,-5,-6,-5,-4,-3,-2,-1,1,2,3,3,4,4,4,4,4,4,4,5,5,6,7,7,8,9,9,9,8,8,8,7,7,6,5,5,6,6,6,7,7,7,6,6,5,4,4,4,5,5,5,5,5,5,5,5,5,4,3,3,3,3,3,3,2,2,2,2,2,2,2,1,2,1,1,1,1,1,1,1,1,0,0,-2,-2,-2,-3,-3,-3,-3,-4,-4,-4,-4,-4,-4,-4,-4,-4,-4,-3,-3,-3,-3,-3,-3,-3,-3,-2,-2,-2,-2,-2,-2,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-1,-1,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-1,0,0,0,1,1,1,1,1,2,2,1,1,1,1,1,1,0,0,0,0,0,1,1,0,0,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,1,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-2,-2,-2,-2,-2,-2,-2,-1,0,0,0,0,0,0,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,-2,-2,-2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-2,-2,-2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-2,-2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,-1,-1,-1,-1,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-1,-1,-1,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-1,-2,-1,-1,-1,-2,-2,-2,-2,-2,-1,-1,-1,-1,-2,-2,-1,-1,-1,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,-1,-1,0,0,0,0,0,0,0,0,0,1,0,0,-1,-1,-2,-2,-2,-2,-2,-2,-1,-2,-2,-2,-3,-3,-3,-4,-3,-4,-4,-4,-4,-4,-4,-4,-5,-5,-5,-5,-5,-6,-6,-5,-5,-6,-6,-6,-6,-7,-7,-8,-8,-8,-9,-8,-8,-8,-8,-8,-7,-6,-5,-5,-5,-5,-4,-4,-4,-4,-4,-4,-4,-4,-3,-3,-3,-4,-4,-4,-4,-4,-4,-4,-4,-5,-5,-4,-4,-3,-3,-2,-2,-2,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-2,-2,-2,-2,-2,-2,-2,0,1,2,2,2,2,0,0,2,3,5,7,9,10,11,12,12,12,12,12,11,10,10,9,9,9,7,4,4,4,5,6,6,7,8,7,7,6,6,6,7,8,15,19,20,20,20,21,19,15,11,7,5,2,1,1,-2,2,4,6,5,4,2};
static q15_t Art2_vector [1024] = {-1,-1,-1,0,0,0,-1,-1,0,-1,0,0,0,0,0,0,0,0,0,-1,0,0,0,-1,-1,-1,0,0,0,-1,0,0,-1,0,0,0,0,0,0,0,-1,0,-1,0,0,0,0,0,0,-1,-1,0,0,-1,-1,0,-1,-1,0,0,0,0,-1,0,0,0,0,-1,-1,0,0,0,0,-1,0,-1,-1,-1,-1,-1,0,0,0,0,0,0,-1,-1,-1,0,-1,0,-1,0,0,-1,-1,-1,0,0,0,0,-1,0,-1,-1,0,-1,-1,0,0,-1,-1,0,-1,-1,-1,-1,0,-1,0,-1,-1,0,0,-1,0,-1,-1,-1,-1,-1,-1,0,0,0,-1,0,0,0,0,0,0,-1,-1,-1,-1,-1,-1,0,-1,-1,-1,0,-1,-1,0,0,0,0,-1,-1,0,-1,0,0,0,0,-1,-1,-1,-1,-1,-1,0,-1,-1,-1,-1,0,-1,-1,-1,-1,0,-1,0,-1,-1,0,0,0,-1,-1,0,0,-1,0,-1,-1,-1,0,-1,0,-1,-1,0,0,-1,0,-1,-1,-1,0,0,-1,0,0,-1,-1,0,0,0,-1,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,-1,0,-1,0,-1,0,-1,0,0,0,-1,0,-1,0,-1,-1,0,-1,0,0,-1,-1,0,-1,0,-1,-1,-1,-1,-1,-1,0,-1,-1,0,0,0,0,-1,0,0,0,0,-1,0,0,0,0,-1,-1,-1,-1,-1,0,0,0,0,0,-1,0,-1,0,-1,-1,-1,-1,0,0,-1,-1,0,-1,0,0,-1,-1,-1,0,0,-1,-1,0,0,-1,0,-1,0,-1,-1,0,0,0,-1,-1,0,-1,-1,-1,-1,-1,-1,-1,0,0,0,0,-1,0,-1,0,-1,-1,-1,-1,-1,-1,-1,-1,0,-1,-1,0,-1,-1,-1,-1,-1,-1,-1,-1,0,0,-1,-1,-1,0,-1,-1,-1,0,-1,-1,0,0,0,-1,-1,0,-1,-1,0,0,-1,-1,0,-1,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,-1,0,-1,-1,0,0,0,-1,0,-1,-1,-1,-1,-1,0,-1,0,0,0,0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,-1,-1,0,-1,-1,-1,0,-1,-1,0,0,-1,-1,-1,-1,0,-1,-1,0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,-1,-1,-1,0,-1,0,-1,-1,0,-1,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,-1,0,0,0,-1,0,0,-1,-1,-1,0,0,-1,-1,-1,0,0,0,-1,-1,-1,-1,-1,-1,-1,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,-1,-1,-1,0,-1,-1,-1,-1,-1,0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,-1,-1,-1,-1,-1,-1,0,-1,-1,-1,0,0,0,-1,-1,0,-1,-1,0,-1,-1,-1,0,0,0,-1,-1,0,0,-1,-1,-1,0,-1,-1,-1,0,0,0,0,-1,0,-1,-1,-1,0,0,0,0,-1,-1,-1,-1,-1,-1,-1,0,-1,0,0,-1,-1,0,0,-1,-1,-1,-1,0,-1,0,-1,0,0,-1,-1,-1,0,-1,0,0,-1,-1,0,0,0,0,0,0,-1,-1,0,-1,-1,-1,0,-1,0,0,0,-1,0,-1,-1,0,0,-1,0,0,-1,0,-1,-1,-1,-1,0,0,-1,0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,0,-1,-1,-1,-1,-1,0,0,-1,-1,-1,0,0,0,0,0,0,0,0,-1,0,0,0,-1,0,0,-1,-1,0,-1,0,0,0,0,0,0,-1,0,0,-1,0,0,-1,0,0,-1,0,-1,0,0,0,0,0,0,0,-1,-1,-1,-1,0,0,0,0,-1,-1,-1,-1,-1};


static int16_t WPT_maximum = 32;    
static	q15_t Calc [1];
static q15_t H[1030];
static q15_t cal[1030];
static q15_t cdl[1030];
q15_t X_c [8];

static q15_t save[1120]; //1120
static q15_t B[1120];
// === USART - BLE (Bletooth Low Energy Module  ============================
#define				BLE_PROGRAM			0 // first use of a new BLE module set to 1. set to 0 in normal use
#define				BLE_MAX_RX_BUFFER_LENGTH   1
#define				BLE_MAX_IN_DATA		25
#define				BLE_param_len		20

static int8_t		BLE_connectedState=	0;
static uint8_t		BLE_rx_buffer[BLE_MAX_RX_BUFFER_LENGTH];
static char			BLE_in_data	[BLE_MAX_IN_DATA];
static char			BLE_cmd		[BLE_MAX_IN_DATA];
static int8_t		BLE_i			=	0;	
static uint16_t		BLE_service		=	0;
static uint32_t		BLE_USART_Val	=	0;
static uint8_t		BLE_USART_Len	=	2;
static uint16_t		BLE_USART_cnt	=	0;
static uint8_t		BLE_isFirst		=	1;

uint16_t			BLE_param	[BLE_param_len]=
{
		// Parameter Array for Transmission via BLE
60,		// BLE_param[0] ... ZCR Threshold (Zero Crossing Rate)
9,		// BLE_param[1] ... ZCR WS Moving Average Filter (Zero Crossing Rate Windowsize in cycles) Filter length = 2^DEC_SSC_WS 2^6=64
//2000,	// BLE_param[2] ... SSC Threshold (Slope Sign Change)
//11,		// BLE_param[3] ... SSC WS Moving Average Filter (SLope Sign Change Windowsize in cycles) Filter length = 2^DEC_SSC_WS
1000,	// BLE_param[2] ... SSC Threshold (Slope Sign Change)
10,		// BLE_param[3] ... SSC WS Moving Average Filter (SLope Sign Change Windowsize in cycles) Filter length = 2^DEC_SSC_WS
5,		// BLE_param[4] ... DEC ON DELAY (quantity of decisions out of threshold in cycles)
30,		// BLE_param[5] ... DEC ART DELAY (time till signal returns in cycles)
0,		// BLE_param[6] ...
0,		// BLE_param[7] ...
90,		// BLE_param[8] ... ZCD min
110,	// BLE_param[9] ... ZCD max
10,		// BLE_param[10] ... ZCD quantity Threshold (Zero Crossing Distance)
330,	// BLE_param[11] ... ZCD upper Thresh
0,		// BLE_param[12] ... 
2000,	// BLE_param[13] ... SAT_COUNT (Quantity of cycles till signal returns) 0.1 ms = 1 cycle
1,		// BLE_param[14] ... SAT_ON_DELAY (Quantity of cycles till saturation is on)
0,		// BLE_param[15] ... 
0,		// BLE_param[16] ... 
0,		// BLE_param[17] ... 
0,		// BLE_param[18] ... EVAL_Thresh - Low Word
0};		// BLE_param[19] ... EVAL_Thresh - High Word


// === External variables defined as extern in DEC.h  ==========================
uint16_t		dec_param		= 0;
uint16_t		ZCD_counter		= 0;
uint16_t		ZCD_BLE			= 0;
uint16_t		DEC_SSC_count	= 0;
uint16_t		DEC_MCR_count	= 0;
uint16_t		DEC_MCRs_count	= 0;
uint16_t		SAT_WFL_BLE_count	= 0;
uint16_t		SAT_ZC_detectedcounter = 0;
uint16_t		DEC_VAR_count	= 0;
uint16_t		main_DLY_Output = 0;
uint8_t			SAT_Status = 0;

// === LED-Control ==========================================================
static uint32_t LED_Counter_0 = 0;
static int8_t	LED_State	  = 0;

static uint16_t LED_Counter_1 = 0;

// === WDT-counter monitoring interrups and main ============================
static int8_t	WDT_CNT	  = 0;


// ==========================================================================
// ===================== FUNCTION IMPLEMENTATIONS ===========================
//

// ==========================================================================
// =============    BLE - Service - Menu  ===================================
//
uint16_t ble_menu(char * ble_menu_cmd)
{
	uint16_t ble_menu_service=0;
	uint16_t ble_menu_gain=0;
	
	//strcpy(BLE_store, ble_menu_cmd, 4);
	if (BLE_isFirst&&strncmp(ble_menu_cmd, "SE1", 3)==0)
	{
		usart_write_buffer_job(&usart_ble_instance, (uint8_t*) ble_menu_cmd, 4);
		BLE_isFirst=0;
	}

	if (strncmp(ble_menu_cmd, "OPEN", 4)==0)
	{
		BLE_connectedState=1;
	}
	else if (strncmp(ble_menu_cmd, "DISC", 4)==0)
	{
		BLE_connectedState=0;
	}
	
	// =============    "SE" - Services  ==============================
	//
	else if (strncmp(ble_menu_cmd, "SE", 2)==0)
	{
		ble_menu_service=10*(ble_menu_cmd[2]-48)+ble_menu_cmd[3]-48;
		switch (ble_menu_service)
		{
			case 1:
			usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "Menu-01:", 8);
			DEC_set_status(0);
			main_DEC_status = 0;
			break;
			
			case 2:
			usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "Setgain:", 8);
			ble_menu_gain=ble_menu_cmd[4]-48;
			if ((ble_menu_gain>0)&&(ble_menu_gain<9))
			{
				GOC_Gain_step=ble_menu_gain;
				GOC_Gain=configure_opamp(GOC_Gain_step);
			}
			break;
			
			case 3:
			usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "OK\r", 4);
			break;
			
			case 4:
			STFT_BLE_isFirstRun=1;
			BLE_isFirst=1;
			break;	
			
			case 5:
			//Calibration
			CLBR_enable=1;
			//main_DEC_CLBR_enable=1;
			usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "CLBR\r", 6);
			break;
			
			case 6:
			//Change to LDO
			SUPC->VREG.bit.SEL = SYSTEM_VOLTAGE_REGULATOR_LDO;
			usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "LDO\n", 3);
			break;
			
			case 7:
			//Change to BUCK
			SUPC->VREG.bit.SEL = SYSTEM_VOLTAGE_REGULATOR_BUCK;
			usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "BUCK\n", 4);
			break;
		
			case 8:
			//Display DECMode ON
			main_DEC_status = 1;
			DEC_set_status(1);
			usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "DEC\n", 3);
			break;
			
			default:
			break;
		}
	}
	
	// =============    "SP" - Set Parameters  ==========================
	//
	else if (strncmp(ble_menu_cmd, "SP", 2)==0)
	{
		ble_menu_service=10*(ble_menu_cmd[2]-48)+ble_menu_cmd[3]-48;
		
		if ((ble_menu_service>=0)&(ble_menu_service<(BLE_param_len-1)))
		{
			
			BLE_param[ble_menu_service]=10000*(ble_menu_cmd[4]-48)+1000*(ble_menu_cmd[5]-48)+100*(ble_menu_cmd[6]-48)+10*(ble_menu_cmd[7]-48)+(ble_menu_cmd[8]-48);
			usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "DONE", 4);
			delay_ms(10);
			usart_write_buffer_job(&usart_ble_instance, (uint8_t *) &BLE_param[ble_menu_service], 2);
		}
		ble_menu_service=00;
	}

	// =============    "GP" - Get Parameters  ==========================
	//
	else if (strncmp(ble_menu_cmd, "GP", 2)==0)
	{
		ble_menu_service=10*(ble_menu_cmd[2]-48)+ble_menu_cmd[3]-48;
		
		if (ble_menu_service==1)
		{
			for (uint16_t i=0; i<BLE_param_len; i++)
			{
				usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "PAR\r", 4);
				delay_ms(10);
				usart_write_buffer_job(&usart_ble_instance, (uint8_t *) &i, 2);
				delay_ms(10);
				usart_write_buffer_job(&usart_ble_instance, (uint8_t *) &BLE_param[i], 2);
				delay_ms(10);
			}
		}
		ble_menu_service=00;
	}	
	
	else 
	{
		ble_menu_service=00;
	}
	
	return ble_menu_service;
}


// ==========================================================================
// ============= FD - Reference signal - Calibration - Implementation =======
//
//	Step 1: Calculate sum of smoothed FFT-Output 
//	Step 2: Calculate Reference and store to NVM and "EVAL_Ref"
//	Step 3: Calculate sum of "EVAL_Sum"
//	Step 4: Calculate "EVAL_Thresh" (= mean value * 2)
//

void CLBR_Reference (int32_t * CLBR_in, int16_t CLBR_LEN)
{
	int16_t i;
	//	Step 1: Calculate sum of smoothed FFT-Output
	if (CLBR_count<CLBR_Num_FFT)
	{
		EVAL_FFT(CLBR_in, CLBR_smooth, CLBR_LEN);
		for (i=0;i<CLBR_LEN;i++)
		{
			CLBR_FFT_Sum[i]=CLBR_FFT_Sum[i]+CLBR_smooth[i];
		}
	}
	//	Step 2: Calculate Reference and store to NVM and "EVAL_Ref"
	else if (CLBR_count==CLBR_Num_FFT)
	{
		EVAL_max_val	= 0;
		EVAL_Thresh		= 0;
		for (i=0;i<CLBR_LEN;i++)
		{
			CLBR_FFT_Sum[i]=CLBR_FFT_Sum[i]>>4;
			if (EVAL_max_val<CLBR_FFT_Sum[i])
			{
				EVAL_max_val=CLBR_FFT_Sum[i];
			}
		}
		EVAL_max_val=I64_4096_shifted/EVAL_max_val;
		for (i=0;i<CLBR_LEN;i++)
		{
			page_buffer_read[i]=(CLBR_FFT_Sum[i]*EVAL_max_val)>>20;
		}

		//Write to Reference Array
 		for (i=0;i<STFT_LEN/2;i++)
 		{
 			EVAL_Ref[i]=page_buffer_read[i];
 		}
				
		for (i=0;i<CLBR_LEN;i++)
		{
			CLBR_FFT_Sum[i]=0;
		}
	}
	else if (CLBR_count>CLBR_Num_FFT)
	{
		//	Step 3: Calculate sum of "EVAL_Sum"
		if (CLBR_count<=CLBR_Num_FFT+16)
		{
			EVAL_FFT(CLBR_in, CLBR_smooth, CLBR_LEN);
			EVAL_Thresh = EVAL_Thresh + EVAL_Sum;
		}
		//	Step 4: Calculate "EVAL_Thresh" (= mean value * 2)
		else
		{
			CLBR_enable=0;
			CLBR_count=0;
			CLBR_start=0;
			//EVAL_Thresh = EVAL_Thresh >> 3;
//  			EVAL_Thresh=EVAL_Thresh*3;
//  			EVAL_Thresh = EVAL_Thresh >> 4;
			//EVAL_Thresh=EVAL_Thresh>>2;
			EVAL_Thresh=EVAL_Thresh>>3;
			
			BLE_param[18] = EVAL_Thresh;
			BLE_param[19] = EVAL_Thresh>>16;
			page_buffer_read[510]=1;
			page_buffer_read[511]=2;
			for (i=0;i<BLE_param_len;i++)
			{
				page_buffer_read[256+i] = BLE_param[i];
			}
			// Write to NVM (Non Volatile-Memory)
			system_interrupt_disable_global();			// disable global interrupts
			NVM_Write_I16(page_buffer_read, 0, 512);
			system_interrupt_enable_global();			// enable global interrupts
		}		

	}
	CLBR_count++;
}


// ==========================================================================
// ====== Daub4 Wavelet Transformation "DAUB4" - Implementation =============
//

void DAUB4_Tran (q15_t *x, q15_t *y, int16_t LEN, int16_t FULL)
{
	uint16_t i;
	uint16_t j;
	uint16_t j0;
	uint16_t j1;
	uint16_t j2;
	uint16_t j3;
	uint16_t k;
	uint16_t l=0;
	uint16_t m;
	q31_t c[4] = {0.4829629131445341*(1<<19), 0.8365163037378079*(1<<19), 0.2241438680420133*(1<<19), -0.1294095225512603*(1<<19) };
	q31_t z[LEN];

	m = LEN;
	k = 1;
	for ( i = 0; i < m; i++ ) y[i] = x[i];
	
	while (m >= 4)
	{

		for ( l = 0; l < k; l++ )
		{
			i = 0;

			for ( j = 0; j < m - 1; j = j + 2 )
			{
				j0 = l*m + j;
				j1 = j0 + 1;
				j2 = j0 + 2;
				j3 = j0 + 3;
				if (j>=m-2) j2=j2-m;			// ACHTUNG stimmt so nicht wirklich??
				if (j>=m-3) j3=j3-m;

				z[i+l*m]		= c[0] * y[j0] + c[1] * y[j1] + c[2] * y[j2] + c[3] * y[j3];
				z[i+m/2+l*m]	= c[3] * y[j0] - c[2] * y[j1] + c[1] * y[j2] - c[0] * y[j3];
				i = i + 1;
			}
		}

		for ( i = 0; i < LEN; i++ )
		{
			y[i] = z[i]>>19;
		}
		
		if (FULL == 1) k=k*2;
		m = m / 2;
	}
	//	for ( i = 0; i < m; i++ ) y[i] = y[i]>>16;
}



// ==========================================================================
// ====== Short Time Fourier Transformation "STFT" - Implementation =========
//

void STFT_Init (void)
{
	uint16_t i;
	for (i=0; i<STFT_LEN; i++)
	{
		STFT_Window[i]=4;
	}
	
	uint32_t ifftFlagR=0;
	uint32_t bitReverseFlag=1;
	arm_rfft_init_q15(&STFT_instance_q15, STFT_LEN, ifftFlagR, bitReverseFlag);
}
	
	
void STFT_Write_Input (q15_t STFT_Input)
{

	STFT_Input_mean=STFT_Input_mean+STFT_Input;		// Input average value (filter)
	if (STFT_Input_mean_CNT<STFT_Freq_divide-1)		// frequency divider
	{
		STFT_Input_mean_CNT++;
	}
	else
	{												// Read Input average value to FFT-Input-Buffer
		if (STFT_Input1_CNT<STFT_LEN)
		{
			STFT_Input1[STFT_Input1_CNT]=STFT_Input_mean*STFT_Window[STFT_Input1_CNT];
		}
		if (STFT_Input2_CNT<STFT_LEN)
		{
			STFT_Input2[STFT_Input2_CNT]=STFT_Input_mean*STFT_Window[STFT_Input2_CNT];
		}
		STFT_Input1_CNT++;
		STFT_Input2_CNT++;
		if (STFT_Input1_CNT==STFT_LEN) STFT_Input1_OK = 1;
		if (STFT_Input2_CNT==STFT_LEN) STFT_Input2_OK = 1;
		if (STFT_Input1_CNT>(STFT_LEN+STFT_LEN/2)) STFT_Input1_CNT=0;
		if (STFT_Input2_CNT>(STFT_LEN+STFT_LEN/2)) STFT_Input2_CNT=0;

		STFT_Input_transmit=STFT_Input_mean;
		STFT_Input_mean_CNT=0;
		STFT_Input_mean=0;
	}
}



void STFT_Write_Output (void)
{
	if (STFT_Input1_OK==1)
	{		
		STFT_Input1_OK=0;
		arm_rfft_q15(&STFT_instance_q15, STFT_Input1, STFT_Output1);
		STFT_abs(STFT_Output1, STFT_Output, STFT_LEN/2);
		STFT_Input1_OK=0;
		STFT_Output_BLE_OK=1;
		STFT_Output_DAC_OK=1;
		STFT_Output_EVAL_OK=1;
		
	}
	if (STFT_Input2_OK==1)
	{
		STFT_Input2_OK=0;
		arm_rfft_q15(&STFT_instance_q15, STFT_Input2, STFT_Output2);
		STFT_abs(STFT_Output2, STFT_Output, STFT_LEN/2);
		STFT_Input2_OK=0;
		STFT_Output_BLE_OK=1;
		STFT_Output_DAC_OK=1;
		STFT_Output_EVAL_OK=1;
	}
}


static uint16_t STFT_dac_val = 0;
static int32_t STFT_dac_Counter=-10;
void STFT_Write_to_DAC (void)
{
	
	if(STFT_Output_DAC_OK) 
		{
		 if (STFT_dac_Counter<0)
		 	{
				STFT_dac_val = 0;
			}
		 else if (STFT_dac_Counter<STFT_LEN/2)
		 {
			 STFT_dac_val = (STFT_Output[STFT_dac_Counter]>>4);
		 }
		 else if (STFT_dac_Counter<(STFT_LEN/2+10))
			{
				STFT_dac_val = EVAL_Emg_FD*4095;
				//STFT_dac_val = 0;
			}
		 else 
			{
		 		STFT_dac_Counter=-10;
				STFT_Output_DAC_OK=0;
			}
		 STFT_dac_Counter++;
		}
	else 
		{
			STFT_dac_val =dac_val;
		}
	
}


#define			STFT_BLE_LEN			150			// Block length 150
static uint16_t STFT_BLE_val = 0;
static int32_t STFT_BLE_Counter=-10;
void STFT_Write_to_BLE (void)
{
	if(STFT_Output_BLE_OK)
	{
		if (STFT_BLE_Counter<STFT_BLE_LEN)
		{
			//STFT_BLE_val = ((DAC_MAX+1)/2)+(STFT_Output[STFT_BLE_Counter]>>2);
			STFT_BLE_val = (STFT_Output[STFT_BLE_Counter]);
			if (STFT_BLE_val==0) STFT_BLE_val=1;
		}
		else
		{
			STFT_BLE_Counter=0;
			STFT_Output_BLE_OK=0;
		}
		STFT_BLE_Counter++;
	}
	else
	{
		STFT_BLE_val = 0;
	}
}


//Write the REF STFT and Smoothed STFT to BLE
#define			STFT_BLE_smoothed_LEN			255			// Block length 150
static uint16_t STFT_BLE_smoothed_val = 0;
static int64_t  STFT_BLE_smoothed_max = 0; //wie groﬂ???
static int8_t	STFT_BLE_smoothed_OK =	0;
//static int32_t STFT_BLE_Counter=-10;
static int32_t STFT_BLE_smoothed_Counter=0;
static uint32_t STFT_BLE_smoothed_Store[STFT_BLE_smoothed_LEN];
void STFT_smoothed_Write_to_BLE (void)
{
	if(STFT_BLE_smoothed_OK)
	{
		if (STFT_BLE_smoothed_Counter<STFT_BLE_smoothed_LEN)
		{
			if (STFT_BLE_isFirstRun==1)
			{
				STFT_BLE_smoothed_val = EVAL_Ref[STFT_BLE_smoothed_Counter];
				if (STFT_BLE_smoothed_val==0) STFT_BLE_smoothed_val=1;
			}
			else
			{
				//normieren
				//STFT_BLE_smoothed_Store[STFT_BLE_smoothed_Counter]=(STFT_BLE_smoothed_max*STFT_BLE_smoothed_Store[STFT_BLE_smoothed_Counter])>>20;
				//STFT_BLE_smoothed_val = STFT_BLE_smoothed_Store[STFT_BLE_smoothed_Counter];
				STFT_BLE_smoothed_val=EVAL_Ref[STFT_BLE_smoothed_Counter]; //TEST
				STFT_BLE_smoothed_val=EVAL_Ref[25]; //TEST
				STFT_BLE_smoothed_val=4012; //TEST
				if (STFT_BLE_smoothed_val==0) STFT_BLE_smoothed_val=1;
			}
			STFT_BLE_smoothed_Counter++;
		}
		else
		{
			STFT_BLE_smoothed_Counter++;
			STFT_BLE_smoothed_OK = 0;
			STFT_BLE_isFirstRun	 = 0;
			STFT_BLE_smoothed_Counter=0;
		}
		//STFT_BLE_smoothed_Counter++;
	}
	else
	{
		STFT_BLE_smoothed_val = 0;
	}
}

void STFT_abs (q15_t *ABS_IN, q31_t *ABS_OUT, int16_t ABS_LEN)
{
	uint16_t i;
	for (i=0; i<ABS_LEN; i++)
	{
		ABS_OUT[i]=abs(ABS_IN[i*2])+abs(ABS_IN[i*2+1]);
		//ABS_OUT[i]=(abs(ABS_IN[i*2])*abs(ABS_IN[i*2])+abs(ABS_IN[i*2+1])*abs(ABS_IN[i*2+1]));
	}
}


	
// ==========================================================================
// ============ Evaluation of FFT "EVAL_FFT" - Implementation ===============
//

int8_t EVAL_FFT(int32_t * EVAL_in, int32_t * EVAL_out, int16_t EVAL_LEN)
{
	int cmpfunc (const void * a, const void * b)
	{
		return ( *(int*)a - *(int*)b );
	}
	
	// smooth with window and sum max values
	EVAL_max_val=0;
	int32_t	EVAL_window[EVAL_WINDOW_SIZE]={0,0,0,0,0,0,0,0,0,0,0,0,0};
	for (ll=0; ll<EVAL_WINDOW_SIZE;ll++)
	{
		EVAL_window[ll]=0;
	}
	EVAL_Output_OK=0;
	for (ii = 0; ii < EVAL_LEN; ii++)
	{
		kk=(ii % EVAL_WINDOW_SIZE);
		EVAL_window[kk]=EVAL_in[ii];
		for (ll=0; ll<EVAL_WINDOW_SIZE;ll++)
		{
			EVAL_sortedWindow[ll]=EVAL_window[ll];
		}
		qsort(EVAL_sortedWindow, EVAL_WINDOW_SIZE, sizeof(int32_t), cmpfunc);		
		//sort EVAL_sortedWindow
//   		for( int8_t j=0; j<EVAL_WINDOW_SIZE; j++)
//   		{
//   			for( int8_t i=0; i<(EVAL_WINDOW_SIZE-1); i++)
//   			{
//   				if(EVAL_sortedWindow[i]>EVAL_sortedWindow[i+1])
//   				{
//   					int16_t w=EVAL_sortedWindow[i];
//   					EVAL_sortedWindow[i]=EVAL_sortedWindow[i+1];
//   					EVAL_sortedWindow[i+1]=w;
//   				}
//   			}
//   		}

		EVAL_Sum=0;
		for (ll=0; ll<EVAL_NUM_MAX;ll++)
		{
			EVAL_Sum=EVAL_Sum+EVAL_sortedWindow[EVAL_WINDOW_SIZE-1-ll];	
		}
		EVAL_out[ii]=EVAL_Sum;
		if (EVAL_max_val<EVAL_Sum)
		{
			EVAL_max_val=EVAL_Sum;
		}
		EVAL_Output_OK=1;
	}

	// Evaluation
	
	EVAL_Sum=0;
	for (ii = 0; ii < EVAL_NUM_COMP; ii++)
	{
		EVAL_Diff=EVAL_Ref[ii];
		EVAL_Diff=EVAL_Diff*EVAL_max_val;
		EVAL_Diff=EVAL_Diff-EVAL_out[ii]*I64_4096;
		//if (EVAL_weighting[ii]==1) EVAL_Diff=EVAL_Diff*4;
		//if (EVAL_weighting[ii]==2) EVAL_Diff=0;
		if (EVAL_Diff<0)
		{
			EVAL_Sum=EVAL_Sum-EVAL_Diff;
		}
		else
		{
			EVAL_Sum=EVAL_Sum+EVAL_Diff;
		}
	}
	EVAL_Sum=EVAL_Sum;
	EVAL_Sum=EVAL_Sum/EVAL_max_val;

	// Return evaluation result
	
	if (CLBR_enable==0)
	{		
		if (EVAL_Sum>EVAL_Thresh)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return 0;
	}
}



// ==========================================================================
// =============== GOC_signal - Sense min, max, avg - Implementation ========
//

void GOC_signal(int32_t GOC_Signal)
{
	if (GOC_Signal_OK == 0) 
	{
		if (GOC_Signal > GOC_Signal_max) GOC_Signal_max = GOC_Signal;
		if (GOC_Signal < GOC_Signal_min) GOC_Signal_min = GOC_Signal;
		GOC_Signal_avg = GOC_Signal_avg + GOC_Signal;	
	}

	if (GOC_Signal_cnt < GOC_Fs*GOC_TIME)
	{
		GOC_Signal_cnt++;
	}
	else
	{
		GOC_Signal_cnt=0;
		if (GOC_Signal_OK == 0)
		{
			GOC_Signal_OK=1;
		}
		else  
		{
			GOC_Signal_OK=0;
			GOC_Signal_avg=0;
		}
	}
}

// ==========================================================================
// ==== GOC_output - Evaluation and Output OPAMP, DAC0 - Implementation =====
//

void GOC_output(void)
{
	if (GOC_Signal_OK == 1)
	{
		if ((GOC_Signal_min < GOC_Deadband) & (GOC_Signal_max < (GOC_MAX-GOC_Deadband)))
		{
			// Offset++
			GOC_Gain_plus_cnt=0;
			if(GOC_Offset<30)
			{
				GOC_Offset++;
				dac_chan_write(&dac_instance, DAC_CHANNEL_0, GOC_Offset);
			}
		}
		else if ((GOC_Signal_min > GOC_Deadband) & (GOC_Signal_max > (GOC_MAX-GOC_Deadband)))
		{
			// Offset--
			GOC_Gain_plus_cnt=0;
			if(GOC_Offset>0)
			{
				GOC_Offset--;
				dac_chan_write(&dac_instance, DAC_CHANNEL_0, GOC_Offset);
			}
			else
			{
				GOC_Gain_step--;
			}
		}
		else if ((GOC_Signal_min < GOC_Deadband) & (GOC_Signal_max > (GOC_MAX-GOC_Deadband)))
		{
			// GAIN--
			GOC_Gain_plus_cnt=0;
			if(GOC_Gain_step>0)
			{
				GOC_Gain_step--;
			}
		}		
		else if (((GOC_Signal_max - GOC_Signal_min) < (GOC_MAX/4)) & ((GOC_Signal_max<(GOC_MAX-GOC_Deadband)/2) | (GOC_Offset>0)))
		{
			// GAIN++
			if(GOC_Gain_step<10)
			{
				GOC_Gain_plus_cnt++;
				if (GOC_Gain_plus_cnt > GOC_Gain_plus)
				{
					GOC_Gain_step++;
					GOC_Gain_plus_cnt=0;
				}
			}					
		}
		else
		{
			GOC_Gain_plus_cnt=0;
		}
		
		if (GOC_Gain_step!=GOC_Gain_step_old)
		{
			// Configure OPAMPs
			GOC_Gain_step_old=GOC_Gain_step;
			GOC_Gain=configure_opamp(GOC_Gain_step);			
		}
		GOC_Signal_OK=2;
		GOC_Signal_max=0;
		GOC_Signal_min=GOC_MAX;
		GOC_Signal_avg=0;
	}
}


// ==========================================================================
// =============== Combfilter (Kamm-Filter) - Implementation ================
//
//uint16_t mycounter = 0;
//int16_t  mystate = 0;
int16_t COMB_Filter(int16_t COMB_In)
{
	//if (mycounter<10000)
	//{
		//mycounter++;
		//mystate = 0;
	//}
	//else if (mycounter<20000)
	//{
		//mycounter++;
		//mystate = 2048;
	//}
	//else mycounter=0;
		
	uint16_t COMB_Output_Calc=0;
	
	COMB_Input[COMB_N]=COMB_In;
	
	COMB_Fs_k1 = COMB_N - COMB_Fs_k;
	if (COMB_Fs_k1 < 0) COMB_Fs_k1 = COMB_Samples + COMB_Fs_k1;
	
	COMB_Fs_k2 = COMB_N - 2*COMB_Fs_k;
	if (COMB_Fs_k2 < 0) COMB_Fs_k2 = COMB_Samples + COMB_Fs_k2;
	
	COMB_Fs_k3 = COMB_N - 3*COMB_Fs_k;
	if (COMB_Fs_k3 < 0) COMB_Fs_k3 = COMB_Samples + COMB_Fs_k3;
	
	COMB_Fs_k4 = COMB_N - 4*COMB_Fs_k;
	if (COMB_Fs_k4 < 0) COMB_Fs_k4 = COMB_Samples + COMB_Fs_k4;

	//COMB_Output = ((COMB_Input[COMB_N]-COMB_Input[COMB_Fs_k1])+(3*COMB_Input[COMB_N]-COMB_Input[COMB_Fs_k2]-COMB_Input[COMB_Fs_k3]-COMB_Input[COMB_Fs_k4])/2)/4;
	COMB_Output_Calc = ((COMB_Input[COMB_N]-COMB_Input[COMB_Fs_k1])+(2*COMB_Input[COMB_N]-COMB_Input[COMB_Fs_k2]-COMB_Input[COMB_Fs_k3])/2)/4;
	//	COMB_Output = (COMB_Input[COMB_N]-COMB_Input[COMB_Fs_k1])/2;
	
	COMB_N++;
	if (COMB_N>=COMB_Samples) COMB_N=0;
	
	return COMB_Output_Calc;
	//return COMB_In;
	//return mystate;
}


// ==========================================================================
// =============== Notchfilter (Kamm-Filter) - Implementation ================
//
// IIR-Filter - Direct form I (Matlab - Filter Designer:  Highpass / IIR-Butterworth / Fs = 50 / Fc = 5)
// Filtercoeffizienten aus "Filter Designer" multipliziert mit 1024 (entspricht Schiebeoperation um 10 Stellen nach rechts)
//
// num1  = 1 * 1024		= 1024;
// num2  = -2 * 1024	= 2048;
// num3  = 1 * 1024		= 1024;
// dnom1 = 1 * 1024		= 1024
// dnom2 = -1,1429805 * dnom1  = -1171;
// dnom3 =  0,4128016 * dnom1  = 422;
//

int16_t NOTCH_Filter(uint16_t NOTCH_In)
{
	int16_t NOTCH_Out=0;
		
	NOTCH_Fs_k1 = NOTCH_N - NOTCH_Fs_k;
	if (NOTCH_Fs_k1 < 0) NOTCH_Fs_k1 = NOTCH_Samples + NOTCH_Fs_k1;
	
	NOTCH_Fs_k2 = NOTCH_N - 2*NOTCH_Fs_k;
	if (NOTCH_Fs_k2 < 0) NOTCH_Fs_k2 = NOTCH_Samples + NOTCH_Fs_k2;

	NOTCH_Center[NOTCH_N] = ((int64_t)NOTCH_In*1024 + (int64_t)NOTCH_Center[NOTCH_Fs_k1]*1171 - (int64_t)NOTCH_Center[NOTCH_Fs_k2]*422)>>10;
	NOTCH_Out = ((int64_t)NOTCH_Center[NOTCH_N]*1024 - (int64_t)NOTCH_Center[NOTCH_Fs_k1]*2048 + (int64_t)NOTCH_Center[NOTCH_Fs_k2]*1024)>>11;
	
	NOTCH_N++;
	if (NOTCH_N>=NOTCH_Samples) NOTCH_N=0;
	
	// Lowpass filter for NOTCH outputsignal
	NOTCH_count = NOTCH_count + NOTCH_Out;
	NOTCH_count=NOTCH_count*((1<<NOTCH_WS)-1)>>NOTCH_WS;
	NOTCH_Out = NOTCH_count>>(NOTCH_WS-1);
	
	return NOTCH_Out;
}

// ==========================================================================
// =============== Hochpassfilter - Implementation ==========================
//
// IIR-Filter - Direct form I (Matlab)
// Filtercoeffizienten aus "Filter Designer" multipliziert mit 1024*32 (entspricht Schiebeoperation um 15 Stellen nach rechts)
//
// num1  = 1;
// num2  = -2;
// num3  = 1;
// dnom1 = 1024*32			  = 32768 
// dnom2 = -1,946697 * dnom1  = -63789;
// dnom3 =  0,949097 * dnom1  = 31100;
//
// Mit Signalanhebung um Faktor 1024 wegen Rechengenauigkeit mit Integerzahlen

int16_t HP_Filter(int16_t HP_Input)
{
	HP_In[0] = HP_Input;
	
	// Formel f¸r Filterberechnung aus Matlab
	// HP_Store = int64((input(i)* num1 + input(i-1) * num2 + input(i-2) * num3) * 1024);
	// HP_Out   = int64((int64((HP_Store * dnom1  - output(i-1) * dnom2 - output(i-2)*dnom3)) / dnom1));
	HP_Out[0] = (((int64_t)(HP_In[0] - HP_In[1] * 2 + HP_In[2]) * 1024) * 32768 + (int64_t)HP_Out[1] * 63789 - (int64_t)HP_Out[2] * 31100) >> 15;	
	
	HP_In[2]=HP_In[1];
	HP_In[1]=HP_In[0];
	
	HP_Out[2]=HP_Out[1];
	HP_Out[1]=HP_Out[0];
	
	// Lowpass filter for HP outputsignal
	HP_count = HP_count + HP_Out[0];
	HP_count=HP_count*((1<<HP_WS)-1)>>HP_WS;
	HP_Out[0] = HP_count>>(HP_WS-1);
	
	return HP_Out[0]>>10;
}


// ==========================================================================
// ========== DEC_TD (Decision in Time Domain ) - Implementation ============
//

//previous version (SSC)
/*int8_t	DEC_TD	(int16_t DEC_In)
{
	
	
	// ========== SSC - Slope Sign Change - Evaluate ========================
	//
	if ((DEC_In-DEC_SSC_OldVal) > 0)
	{
		if (DEC_SSC_OldSlope==0) DEC_SSC_count=DEC_SSC_count+100;
		DEC_SSC_OldSlope=1;
	}
	else
	{
		if (DEC_SSC_OldSlope==1) DEC_SSC_count=DEC_SSC_count+100;
		DEC_SSC_OldSlope=0;
	}
	
	DEC_SSC_OldVal=DEC_In;
	
	// weighting
	DEC_SSC_count=DEC_SSC_count*((1<<DEC_SSC_WS)-1)>>DEC_SSC_WS;


	// ========== Platzhalter f¸r weitere Entscheidungen ===================
	//

	if (DEC_SSC_count > DEC_SSC_THRESH)  return 1;	// No Contraction -> Signal OFF
									else return 0;	// Contraction	  -> Signal ON

}*/



// ==========================================================================
// ======= Absolute value and Moving average filter - Implementation ========
//

void ABS_MAF(int16_t ABS_MAF_Input, uint8_t removeNoiseLevel)
{
	static int32_t squared_32=0;
	
	if(1)	// 1...Squaring   /   0...Linear absolute value
	{
		squared_32=ABS_MAF_Input*ABS_MAF_Input;
		squared_32=squared_32>>3;		
	}
	else
	{
		squared_32=ABS_MAF_Input<<5;
		if (squared_32<0) squared_32=-squared_32;
	}
			
	if (squared_32>DAC_MAX*4) squared_32=DAC_MAX*4;
	
	//Smoothing- Moving Average Filter
	ABS_MAF_mean = (ABS_MAF_mean + squared_32);
	if (ABS_MAF_mean>(DAC_MAX<<(ABS_MAF_DATAPOINTS+1)))
	{
		ABS_MAF_mean=DAC_MAX<<(ABS_MAF_DATAPOINTS+1);
	}
	ABS_MAF_mean=(ABS_MAF_mean*((1<<ABS_MAF_DATAPOINTS)-1)) >> ABS_MAF_DATAPOINTS;

	//removeNoiseLevel=0;
	if (removeNoiseLevel) ABS_MAF_Output=(ABS_MAF_mean >> (ABS_MAF_DATAPOINTS-1)) - (DAC_MAX>>4);	// Remove noise level
	else ABS_MAF_Output=((ABS_MAF_mean) >> (ABS_MAF_DATAPOINTS-1));
	if (ABS_MAF_Output>DAC_MAX) ABS_MAF_Output=DAC_MAX;
	ABS_MAF_Output=ABS_MAF_Output<<1;
	ABS_MAF_Output=ABS_MAF_Output+475;
	//if ((ABS_MAF_Output<0)|(SAT_On==1)|(SAT_ZC_On==1))	ABS_MAF_Output=0;				// Min Limit
	if ((ABS_MAF_Output<0))	ABS_MAF_Output=0;				// Min Limit
	else if (ABS_MAF_Output>DAC_MAX)	ABS_MAF_Output=DAC_MAX;			// Max Limit
}

// ==========================================================================
// =================== I2C - Interrupt ======================================
//

//	Callback function for read request from a master:
void i2c_read_request_callback(
struct i2c_slave_module *const module)
{
	/* Init i2c packet */
	packet.data_length = DATA_LENGTH;
	    packet.data_length = 10;
		dac_val=12;
		packet.data[0] = dac_val%256;
		packet.data[1] = dac_val/256;
		packet.data[2] = dac_val%256;
		packet.data[3] = dac_val/256;
		packet.data[4] = dac_val%256;
		packet.data[5] = dac_val/256;
		packet.data[6] = dac_val%256;
		packet.data[7] = dac_val/256;
		packet.data[8] = dac_val%256;
		packet.data[9] = dac_val/256;
		//packet.data[10] = dac_val%256;
		//packet.data[11] = dac_val/256;
		//packet.data[12] = dac_val%256;
		//packet.data[13] = dac_val/256;
		//packet.data[14] = dac_val%256;
		//packet.data[15] = dac_val/256;
		//packet.data[16] = dac_val%256;
		//packet.data[17] = dac_val/256;
		//packet.data[18] = dac_val%256;
		//packet.data[19] = dac_val/256;
		//packet.data[20] = dac_val%256;
						
	//packet.data = write_buffer;
	/* Write buffer to master */
	i2c_slave_write_packet_job(module, &packet);
}

//  Callback function for write request from a master:
void i2c_write_request_callback(
struct i2c_slave_module *const module)
{
	/* Init i2c packet */
	packet.data_length = DATA_LENGTH;
	packet.data = read_buffer;
	/* Read buffer from master */
	if (i2c_slave_read_packet_job(module, &packet) != STATUS_OK) {
	}
}


// ==========================================================================
// =================== System Takt Interrupt ================================
//

void SysTick_Handler(void)
{
	LED_Counter_1++;
	if (LED_Counter_1>=1000)			// 1 sec clock
	{
		LED_Counter_1 = 0;
		port_pin_set_output_level(LED1_PIN,LED1_ON);
	}
	if (LED_Counter_1==20)	port_pin_toggle_output_level(LED1_PIN);
	 
 	if (!port_pin_get_input_level(PIN_PA16))
 	{
 		//start calibration state
 		CLBR_enable=1;
		
 	}
 
	if (WDT_CNT==0) WDT_CNT=1;
}

// ==========================================================================
// =================== TC0 Compare Channel 0 Interrupt ======================
//

void tc0_cc_c0_callback( struct tc_module * const module_inst)
{
	//Delay before Calibration
	if (CLBR_enable&&(CLBR_start==0))
	{
		CLBR_count2++;
		if (CLBR_count2>70000) //10000 = 1 s
		{
			CLBR_start  = 1;
			CLBR_count2 = 0;
		}
	}
		
	usart_read_buffer_job(&usart_ble_instance, (uint8_t *)BLE_rx_buffer, BLE_MAX_RX_BUFFER_LENGTH);
	
	adc_read(&adc_instance, &adc_result);
	adc_start_conversion(&adc_instance);

	//GOC_signal(adc_result);
	
	//COMB_Output = COMB_Filter(adc_result);
	COMB_Output = NOTCH_Filter(adc_result);
	HP_Output = HP_Filter(COMB_Output);
	//COMB_Output = HP_Filter(NOTCH_Filter(adc_result));
	
	main_DLY_Output=DLY_Signal(HP_Output);
	
	dac_val=((DAC_MAX+1)/2)+(HP_Output>>2);
	if (1)
	//if (BLE_service < 10)
	{
 		if (main_DEC_CLBR_enable)
		{
 			//DEC_CLBR(COMB_Output, &main_DEC_CLBR_enable);
		}
 		else
 		{
			// DEC_TD(COMB_Output);
 			//dac_val=((DAC_MAX+1))+SAT_Signal(adc_result, main_DLY_Output, HP_Output, DEC_TD(COMB_Output))/2; //Saturation, ZCD, ZCR, SSC
			//dac_val=dac_val>>1;
			//dac_val=((DAC_MAX+1)/2)+SAT_Signal(main_DLY_Output, COMB_Output+(DAC_MAX+1)/2, 0); //Saturation, ZCD
			if (dac_val!=(DAC_MAX+1)/2)	SAT_Status=0;
			else						SAT_Status=1;
 		}
		//ABS_MAF((dac_val-(DAC_MAX+1)/2),0);		// Absolute value and moving average filter
		ABS_MAF((dac_val-(DAC_MAX+1)/2)/2,0);		// Absolute value and moving average filter
	}
 	
	if (STFT_On)
	{
		//STFT_Write_Input(COMB_Output>>4);	// Input data for STFT (short time fourier transformation)
		STFT_Write_Input(HP_Output);	// Input data for STFT (short time fourier transformation)
		//STFT_Write_Input(adc_result);	// Input data for STFT (short time fourier transformation)

		STFT_Write_to_DAC();
	}

	if (STFT_On)
	{
		dac_chan_write(&dac_instance, DAC_CHANNEL_1, STFT_dac_val);
	}
	else
	{
	dac_chan_write(&dac_instance, DAC_CHANNEL_1, dac_val);
//	dac_chan_write(&dac_instance, DAC_CHANNEL_1, ((DAC_MAX+1)/2)+(HP_Output>>2));
		
//	dac_chan_write(&dac_instance, DAC_CHANNEL_1, dac_val*EVAL_Emg_FD); //FD Evaluation
//	dac_chan_write(&dac_instance, DAC_CHANNEL_1, adc_result>>4);
//	dac_chan_write(&dac_instance, DAC_CHANNEL_1, adc_result);
//dac_chan_write(&dac_instance, DAC_CHANNEL_1, ABS_MAF_Output);
//	dac_chan_write(&dac_instance, DAC_CHANNEL_1, STFT_Input1_OK*400+STFT_Input2_OK*800+STFT_Output_DAC_OK*1600+STFT_Output_EVAL_OK*2400);
//	dac_chan_write(&dac_instance, DAC_CHANNEL_1, ABS_MAF_Output*EVAL_Emg_FD);
	}
		
// =================== BLE Send measuring data via BLE_USART =====

	switch (BLE_USART_cnt)
	{
		case 0:
			switch (BLE_service)
			{
				case 11:
					BLE_USART_Val = COMB_Output;
					break;
				case 12:
					//STFT_smoothed_Write_to_BLE();
					BLE_USART_Val = HP_Output;
					break;
				case 13:
					BLE_USART_Val = dac_val;
					break;
				case 14:
					BLE_USART_Val = adc_result;
					break;
				case 15:
					if (STFT_On)
					{
						STFT_Write_to_BLE();
						BLE_USART_Val = STFT_BLE_val;
					}
					break;
				case 16:
					BLE_USART_Val = EVAL_Sum;
					BLE_USART_Len = 4;
					break;
				case 17:
					if (STFT_On)	BLE_USART_Val = STFT_dac_val;
					break;
				case 18:
					if (STFT_On)	BLE_USART_Val = (((5*(DAC_MAX+1))/2)+STFT_Input_transmit+4)/8;
					break;					
				case 21:
					BLE_USART_Val = LED_Counter_0;
					break;
				case 22:
					BLE_USART_Val = DEC_MCR_count;
					break;
				case 23:
					BLE_USART_Val = DEC_MCRs_count;
					break;
				case 24:
					BLE_USART_Val = SAT_WFL_BLE_count;
					break;
				case 25:
					BLE_USART_Val = DEC_VAR_count;
					break;
				case 26:
					BLE_USART_Val = DEC_SSC_count;
					break;
				case 29:
					if (STFT_On)
					{
						STFT_smoothed_Write_to_BLE();
						BLE_USART_Val = STFT_BLE_smoothed_val;
					}
					break;
				default:
					break;
			}
			break;
		
		case 1:
			usart_write_buffer_job(&usart_ble_instance, (uint8_t *) &BLE_USART_Val, BLE_USART_Len);
			break;
			
// 		case 3:
// 			usart_write_buffer_job(&usart_ble_instance, (uint8_t *) &EVAL_Emg_FD, 2);
// 			break;
			
		default:
			break;
			
	}
	BLE_USART_cnt++;
	if ((BLE_USART_cnt>4)||(BLE_service<10)) BLE_USART_cnt=0;				

// =================== BLE Send Status Information via BLE_USART ==
	if ((EVAL_Emg_FD==0)&(EVAL_Emg_FD_old==1))
	{
		//DEC_set_status(9);
	}
	EVAL_Emg_FD_old=EVAL_Emg_FD;
	main_DEC_status=getDECstatus();
	if (main_DEC_status!=main_DEC_status_old)
	{
		main_DEC_status_old=main_DEC_status;
		switch (main_DEC_status)
		{
			case 0:
				usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "DEC_OFF\r", 9);
				break;
			case 1:
				usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "DEC_ON\r", 8);
				break;
			case 2:
				usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "SAT_ADC_ON\r", 12);
				break;	
			case 3:
				usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "SAT_FLT_ON\r", 12);
				break;
			case 4:
				usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "VAR\r", 5);
				break;	
			case 5:
				usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "MCR_mod\r", 9);
				break;		
			case 6:
				usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "MCR_sml\r", 9);
				break;
			case 7:
				usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "WFL_mod\r", 9);
				break;
			case 8:
				usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "SSC_mod\r", 9);
				break;
			case 9:
				usart_write_buffer_job(&usart_ble_instance, (uint8_t*) "STFT\r", 6);
				break;
			default:
				break;
		}
	}

// =================== LED Control -  ============================
	
	if (BLE_connectedState)		LED_State=3;
		else					LED_State=4;
	if (CLBR_enable)
	{	
		if (CLBR_count2 < 50000) LED_State=0;
			else				 LED_State=7;
	}
	
	
	LED_CTRL(LED_State, LED_Counter_0);
	LED_Counter_0++;
	if (LED_Counter_0>=10000)			// 1 sec clock
	{
		LED_Counter_0 = 0;
	}
	if (WDT_CNT==1) WDT_CNT=2; // Reset WDT   =================== 
}

// ==========================================================================
// ======================= BLE-USART - Read - Write Interrupts ==============	
//
uint16_t percent_count=0;
void usart_ble_read_callback(struct usart_module *const usart_module)
{
	BLE_in_data[BLE_i]=BLE_rx_buffer[0];

	if (BLE_in_data[BLE_i]==10||BLE_in_data[BLE_i]==13||BLE_in_data[BLE_i]==95||BLE_in_data[BLE_i]==37)
	{
		memset((char*) &BLE_cmd,'0',BLE_MAX_IN_DATA);
		memcpy((char*) &BLE_cmd, (char*) &BLE_in_data, BLE_i+1 );
		BLE_i=-1;
	}
	if (BLE_i>=BLE_MAX_IN_DATA-1)
	{
		BLE_i=-1;
	}
	BLE_i++;
	
	usart_read_buffer_job(&usart_ble_instance, (uint8_t *)BLE_rx_buffer, BLE_MAX_RX_BUFFER_LENGTH);
}

void usart_ble_write_callback(struct usart_module *const usart_module)
{    

}

// ==================================================
//   DWPT



//convolution function that convolutes h with x to the output vector y

void conv(q15_t *x, q15_t *h, q15_t *Y, q15_t len_h, int16_t ind)  // stern oder und je nach adresse oder value verwenden.
{
	// initializing
	q15_t len = len_h + 7;
	q15_t i;
	q15_t j;
	
	// save the vector to H
	for (i=0; i<= len_h; i++){
		H[i] = 0;
		H[i] = h[i+ind];
	}
	// save db_coeff to X_c
	for (i=0; i <= 7; i++){
		X_c [i] = x[i];
	}
	
	// calculate the concolution
	for (i = 0; i <= len; i++) {
		Y[i] = 0;
		for (j = 0; j <= 7; j++) {
			if ((i - j) +1> 0) {
				Y[i] = Y[i] + (X_c[j] * H[i - j]);
			}
		}
	}
}

//DWT function to make the first dwt of the input vector 
void dwt1(q15_t *s_in, q15_t *out, int16_t d){
	// initalize variables
	int16_t i=0;
	int16_t ind = 0;
	
	// make convolution for lp and hp and save it in cal and cdl
	conv(dv1, s_in,cal, 1024, ind);
	conv(dv2, s_in,cdl, 1024, ind);
	
	/* Downsample by 2 to perserve signal length */
		for (i = 0; i < d; i++) {
			out[i] = cal[1 + (i << 1)];
			out[d + i] = cdl[ 1+ (i << 1)];
		}
}


//DWT function to make the second dwt of the input vector 
void dwt2(q15_t *s_in, q15_t *out, int16_t d, int16_t step, int16_t ind){
	// initalize variables
	int16_t i;
	
	// make convolution for lp and hp and save it in cal and cdl
	conv(dv1, s_in,cal, 515, ind);
	conv(dv2, s_in,cdl, 515, ind);
	
	/* downsample by 2 to perserve signal length */
		for (i = 0; i < 260; i++) {
			out[step+i] = cal[1 + (i << 1)];
			out[step+260 + i] = cdl[1 + (i << 1)];
		}
}

//DWT function to make the third dwt of the input vector 
void dwt3(q15_t *s_in, q15_t *out, int16_t d, int16_t step, int16_t ind){
	// initalize variables
	int16_t i;

	// make convolution for lp and hp and save it in cal and cdl
	conv(dv1, s_in,cal, 260, ind);
	conv(dv2, s_in,cdl, 260, ind);
	/* Downsample by 2 to perserve signal length */
		for (i = 0; i < 133; i++) {
			out[step +i] = cal[1 + (i << 1)];
			out[step + 133 + i] = cdl[1 + (i << 1)];
		}
}

//DWT function to make the fourth dwt of the input vector 
void dwt4(q15_t *s_in, q15_t *out, int16_t d, int16_t step, int16_t ind){
	// initalize variables
	int16_t i;

	// make convolution for lp and hp and save it in cal and cdl
	conv(dv1, s_in,cal, 133, ind);
	conv(dv2, s_in,cdl, 133, ind);
	/* Downsample by 2 to perserve signal length */
		for (i = 0; i < 70; i++) {
			out[step + i] = cal[1 + (i << 1)];
			out[step + 70 + i] = cdl[1 + (i << 1)];
		}	
}


//WPT function to  get depth 7 wpt of the input vector
void wpt(q15_t *s, q15_t *wpt_out){

	// initialize variables
	int16_t i=0;
	int16_t save_ind = 0;
	int16_t wpt_out_ind = 0;



	// make the level 1 dwt and save it to save
	dwt1(s,save, 1030);
	
	// normalize the signal
	normalize_vector (save, 1030);
	
	// make the level 2 dwt and save it to wptout
	for (i = 1; i <= 2; i++){
		save_ind = 512*(i-1);
		wpt_out_ind = 520*(i-1);
		dwt2(save, wpt_out, 520, wpt_out_ind, save_ind);
	}
	// normalize the signal
	normalize_vector (wpt_out, 1040);
	
	// make the level 3 dwt and save it to save
	for (i = 1; i <= 4; i++){
		
		save_ind = 260 * (i-1);
		wpt_out_ind = 266 * (i-1);
		dwt3(wpt_out, save,266, wpt_out_ind, save_ind);
	}
	
	// normalize the signal
	normalize_vector (save, 1064);
	
	
	// make the level 4 dwt and save it to wptout
	for (i = 1; i <= 8; i++){
		save_ind = 133 * (i-1);
		wpt_out_ind = 140 * (i-1);
		dwt4(save, wpt_out,140, wpt_out_ind, save_ind);
	}
	
	// normalize the signal
	normalize_vector (wpt_out, 1120);
	
}

// function to normalize a vector to a given max and lenght (same as floor in matlab)
void normalize_vector (q15_t *inout, int16_t length){
	int16_t temp = 0;
	int16_t i = 0;
	q15_t value_max = 0;
	for (i=0; i< length; i++){
			temp = inout[i];
			if (temp <= 0 ){
				temp = -1*temp;	
			}
			
			if (value_max < temp){
				value_max = temp;
			}
		}
		for (i=0; i< length; i++){
			if(inout[i] <= 0){
				inout[i] = inout[i]- (value_max/32);
			}
			inout[i] = (WPT_maximum* inout[i])/value_max;
		}
	
}

//EVAl WPT function with 0 or 1 output to see if this is a contraction or not (1 is yes)
int16_t eval_wpt(q15_t *signal_to_check, int16_t maximum){
	
	// initialize used variables
	int16_t eval_val;
	q15_t value_cust[16];
	int16_t k;
	int16_t i;
	int16_t l;
	int16_t i0;
	q15_t c_B[16];
	q15_t value_cust_max_val = 0;
	q15_t difference_cust[9];
	q15_t dv0[1];

	/* wpt of the signal to check */
	wpt(signal_to_check, &B[0]);

	// initialize the c_B
	for(i=0; i< 16; i++){
		c_B[i] =0;
	}
	
	// build the abs of B
	for(i=0; i< 1120; i++){
		if (B[i] <= 0){
			B[i] = -1*B[i];
		}	
	}
	/* get the wavelet coefficients form the bottom level and make the norm of the vector */
	k = 0;
	for (i = 0; i <= 15; i++) {
		l = 70 * i;
		c_B[i] = (c_B[i]*c_B[i])<<4;
		for (i0 = 0; i0 < 70; i0++) {
			c_B[i] = c_B[i] + ((B[l+i0+1]*B[l+i0+1]));
		}
		value_cust[k] = sqrt(c_B[i]);
		if (value_cust_max_val<value_cust[k]){
			value_cust_max_val=value_cust[k];
		}
		k++;
	}

	// make the average of the high frequency depending coefficients
	for(i=0;i <= 5; i++){
		value_cust[9] = value_cust[9] +value_cust[10+i] ;
	}
	value_cust[9] = value_cust[9]/8;
	
	// normalize the wpt vector
	for (i=0;i<=9;i++){
		value_cust[i] = (maximum * value_cust[i]) / value_cust_max_val;
	}
	/* get the percentual difference between the signal_to_check and a given */
	/* norm */
	dv0[0] = 0;
	for (i = 0; i <= 8; i++) {
		difference_cust[i] = ((100*(value_cust[i]-norm_cust_con[i])) / (norm_cust_con[i]));
		if (difference_cust[i] < 0){
			difference_cust[i] = -1*difference_cust[i];
		}
		if (difference_cust[i] <= 50){
			difference_cust[i] = 0;
		}else{
			difference_cust[i] = 1;
		}
		dv0[0] = dv0[0] + difference_cust[i];
	}

	/* sum up the coefficients and check if the value is below or above the */
	/* threshold */
	eval_val = 0.0;
	if (dv0[0] <= 1) {
		eval_val = 1.0;
	}

	return eval_val;
}






// ==========================================================================
// =====================   M  A  I  N   =====================================
//

int main(void)
{

	/*
	system_init();								// basic controller settings

	configure_dfll();

	SysTick_Config(CLOCK_FREQENCY_MHz*1000/1);	// CLOCK_FREQENCY_MHz*1000/1kHz (48MHz Clock -> 48000 f¸r 1kHz SysTick)

	configure_io();								// configure IO-Ports

	configure_dac();							// configure DAC
	configure_dac0_channel();
	configure_dac1_channel();

	configure_adc();							// configure ADC
	configure_nvm();
//	configure_usart();
	// configure_power_supply();					// switch from LDO to Buck mode

	dac_enable(&dac_instance);					// enable DAC
	dac_chan_write(&dac_instance, DAC_CHANNEL_0, GOC_Offset);

	
	GOC_Gain=configure_opamp(GOC_Gain_step);	// configure onboard amplifiers
	STFT_Init();

	configure_tc0();							// configure TC0
	configure_tc0_callbacks();

	system_interrupt_set_priority(TC0_IRQn, SYSTEM_INTERRUPT_PRIORITY_LEVEL_3);
	system_interrupt_set_priority(SERCOM0_IRQn, SYSTEM_INTERRUPT_PRIORITY_LEVEL_0);
	system_interrupt_enable_global();			// enable global interrupts
	
	ble487x_init(BLE_PROGRAM);					// initialize the BLE chip

	configure_i2c_slave(SLAVE_ADDRESS, CONF_I2C_SLAVE_MODULE);	// configure I2C interface to ARDUINO
	configure_i2c_slave_callbacks();
	
	NVM_Read_I16(page_buffer_read, 0, 512);		// Read Data from NVM (EEPROM)
	//EVAL_Thresh=50*4096;
	BLE_param[18]=EVAL_Thresh;
	BLE_param[19]=EVAL_Thresh>>16;
	// Check if data available in NVM and copy to RAM
	if((page_buffer_read[0]!=-1)&&(page_buffer_read[255]!=-1)&&(page_buffer_read[510]==1)&&(page_buffer_read[511]==2))
	{
		for (uint16_t i=0;i<BLE_param_len;i++)
		{
			BLE_param[i] = page_buffer_read[256+i];
		}

		EVAL_Thresh=BLE_param[19];				
		EVAL_Thresh=EVAL_Thresh<<16;
		EVAL_Thresh=EVAL_Thresh+BLE_param[18];

		for (uint16_t i=0;i<STFT_LEN/2;i++)
		{
			EVAL_Ref[i]=page_buffer_read[i];
		}	
	}

 	configure_wdt();							// configure and start Watchdog Timer
 	configure_bod33();							// configure VDD Brown Out Detector (BOD33)
	*/
	int16_t art1 =  eval_wpt(Art1_vector, WPT_maximum);
	//int16_t art2 =  eval_wpt(Art2_vector, WPT_maximum);
	//int16_t con1 =  eval_wpt(Con1_vector, WPT_maximum);
	//int16_t con2 =  eval_wpt(Con2_vector, WPT_maximum);
	/*
	while (true) 
	{
		if (STFT_On)
		{
			STFT_Write_Output();	// Calculate STFT (short time fourier transformation)
			
			// DAUB4_Tran (DAUB4_Input, DAUB4_Output, DAUB4_LEN, 1); // DAUB 4 - TEST!!!!

			if (STFT_Output_EVAL_OK==1)	// enter only when new STFT_Output is calculated
			{				
 				if (CLBR_enable&&CLBR_start)
 				{
	 				CLBR_Reference(STFT_Output, STFT_LEN/2);
 				}
 				else
 				{
					 //SAT_Status from Saturation in TD
					 SAT_Status=0; //????????????????????????? FOR TESTING ???????????????????????????????????????????????????????
					 if (SAT_Status==1)
					 {
						 EVAL_Emg_FD=0;
						 EVAL_Emg_FD_old=0;
					 }
					 else
					 {
						//STFT_BLE_smoothed_Counter=0;
 						EVAL_Emg_FD = EVAL_FFT(STFT_Output, EVAL_Output, STFT_LEN/2);
						//EVAL_Emg_FD=1;
						 
						 if (STFT_BLE_smoothed_OK==0)
						 {
							//STFT_BLE_smoothed_max=EVAL_max_val;
							//STFT_BLE_smoothed_max=I64_4096_shifted/STFT_BLE_smoothed_max;
							//Store current EVAL_Output
							for (uint8_t i=0; i<STFT_BLE_smoothed_LEN; i++)
							{
								//STFT_BLE_smoothed_Store[i]=EVAL_Output[i];
								//STFT_BLE_smoothed_Store[i]=(STFT_BLE_smoothed_max*EVAL_Output[i])>>20;
								//STFT_BLE_smoothed_Store[i]=EVAL_Ref[i];
								STFT_BLE_smoothed_Store[i]=15;
								//if (STFT_BLE_smoothed_max<STFT_BLE_smoothed_Store[i])	STFT_BLE_smoothed_max = STFT_BLE_smoothed_Store[i];
							}
							//STFT_BLE_smoothed_max=I64_4096_shifted/STFT_BLE_smoothed_max;
							STFT_BLE_smoothed_OK=1;
							STFT_BLE_smoothed_Counter=0;
						 }
					 }
				}
				STFT_Output_EVAL_OK=0;
			}
		}
 			
 		//BLE Service - Menu 				
 		BLE_service=ble_menu((char*) &BLE_cmd);
 		if (BLE_service<10)
 		{
 			memset ((char*) &BLE_cmd,'0',BLE_MAX_IN_DATA);
 		}
		else BLE_USART_Len=2;
			 
		//Reset Watchdog 
 		if (WDT_CNT==2)
 		{
	 		wdt_reset_count();
	 		WDT_CNT=0;
 		}
 	}
	 */
}