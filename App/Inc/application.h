/*
 * application.h
 *
 *  Created on: 2026年8月15日
 *      Author: bitstream
 */

#ifndef INC_APPLICATION_H_
#define INC_APPLICATION_H_

#include <stdint.h>

class Application
{
public:
	void Init();
	void Loop();

	void UartTxCplt();

private:
	uint32_t Timer;
	uint32_t TimerBMM150;

	struct __attribute__((packed)){
		volatile unsigned long iTOW;
		volatile uint8_t fixType; //0=无定位，2=2D定位，3=3D定位，4=GNSS+DR组合，5=RTK浮点，6=RTK固定。飞控必须判断此值 ≥3 才能使用 GPS 数据
		// 🚨 新增：参与定位的卫星数量
		volatile uint8_t numSV;
		volatile float fLon;
		volatile float fLat;
		volatile signed long hMSL;   //海拔高度。相对于平均海平面的高度（单位：毫米）。飞控通常用这个值与气压计或 IMU 融合来定高。
		volatile unsigned long hAcc; //水平定位精度因子（单位：毫米）。例如 hAcc = 500 表示水平误差在 0.5 米左右。飞控可用此值判断 GPS 信号好不好，太差就降低 GPS 权重。
		volatile signed long velN;  //北向速度（单位：毫米/秒）。向北为正，向南为负。
		volatile signed long velE;  //东向速度（单位：毫米/秒）。向东为正，向西为负。
		int16_t mx;
		int16_t my;
		int16_t mz;
		volatile uint8_t checksum;
	} send_data;
	volatile uint32_t send_data_update_tick = 0;
	volatile uint8_t tx_data_index = 0;

	uint8_t tx_buffer[sizeof(send_data) + 20];

	volatile bool uart_send_done = false;
};

extern Application app;

#endif /* INC_APPLICATION_H_ */
