/*
 * application.cpp
 *
 *  Created on: 2026年8月15日
 *      Author: bitstream
 */
#include "main.h"
#include "application.h"
#include "gnss.h"
#include <cstdio>
#include <cmath>
#include "spi.h"
#include "usart.h"
#include "../Bmm150/bmm150.h"

Application app;

// 内存屏障（ARM DSB）
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
  #define MEM_BARRIER()  __dsb(0xF)
#elif defined(__GNUC__)
  #define MEM_BARRIER()  __asm__ volatile ("dsb 0xF" ::: "memory")
#elif defined(__ICCARM__)
  #define MEM_BARRIER()  __iar_builtin_DSB()
#else
  #error "Unknown compiler, add memory barrier"
#endif


BMM150 bmm = BMM150();

float value_offset_x = 0;
float value_offset_y = 0;
float value_offset_z = 0;

void calibrate(uint32_t timeout) {
    int16_t value_x_min = 0;
    int16_t value_x_max = 0;
    int16_t value_y_min = 0;
    int16_t value_y_max = 0;
    int16_t value_z_min = 0;
    int16_t value_z_max = 0;
    uint32_t timeStart = 0;

    bmm.read_mag_data();
    value_x_min = bmm.raw_mag_data.raw_datax;
    value_x_max = bmm.raw_mag_data.raw_datax;
    value_y_min = bmm.raw_mag_data.raw_datay;
    value_y_max = bmm.raw_mag_data.raw_datay;
    value_z_min = bmm.raw_mag_data.raw_dataz;
    value_z_max = bmm.raw_mag_data.raw_dataz;
    HAL_Delay(100);

    timeStart = HAL_GetTick();

    while ((HAL_GetTick() - timeStart) < timeout) {
        bmm.read_mag_data();

        /* Update x-Axis max/min value */
        if (value_x_min > bmm.raw_mag_data.raw_datax) {
            value_x_min = bmm.raw_mag_data.raw_datax;
            // Serial.print("Update value_x_min: ");
            // Serial.println(value_x_min);

        } else if (value_x_max < bmm.raw_mag_data.raw_datax) {
            value_x_max = bmm.raw_mag_data.raw_datax;
            // Serial.print("update value_x_max: ");
            // Serial.println(value_x_max);
        }

        /* Update y-Axis max/min value */
        if (value_y_min > bmm.raw_mag_data.raw_datay) {
            value_y_min = bmm.raw_mag_data.raw_datay;
            // Serial.print("Update value_y_min: ");
            // Serial.println(value_y_min);

        } else if (value_y_max < bmm.raw_mag_data.raw_datay) {
            value_y_max = bmm.raw_mag_data.raw_datay;
            // Serial.print("update value_y_max: ");
            // Serial.println(value_y_max);
        }

        /* Update z-Axis max/min value */
        if (value_z_min > bmm.raw_mag_data.raw_dataz) {
            value_z_min = bmm.raw_mag_data.raw_dataz;
            // Serial.print("Update value_z_min: ");
            // Serial.println(value_z_min);

        } else if (value_z_max < bmm.raw_mag_data.raw_dataz) {
            value_z_max = bmm.raw_mag_data.raw_dataz;
            // Serial.print("update value_z_max: ");
            // Serial.println(value_z_max);
        }

        //Serial.print(".");
        HAL_Delay(100);

    }

    value_offset_x = value_x_min + (value_x_max - value_x_min) / 2;
    value_offset_y = value_y_min + (value_y_max - value_y_min) / 2;
    value_offset_z = value_z_min + (value_z_max - value_z_min) / 2;
}

void Application::Init()
{
	GNSS_Init();
	HAL_Delay(1000);
	GNSS_LoadConfig();
	Timer = HAL_GetTick();
	TimerBMM150 = HAL_GetTick();

	if (bmm.initialize() == BMM150_E_ID_NOT_CONFORM)
	  {
		    //sprintf(message, "Initialization Failed: Chip ID can not read! \r\n");
		    //HAL_UART_Transmit(&huart2, (uint8_t*)&message, strlen(message), 0xFFFF);
	      //while (1);
	  }
	else
	  {
		    //sprintf(message, "Initialization Successful... \r\n");
		    //HAL_UART_Transmit(&huart2, (uint8_t*)&message, strlen(message), 0xFFFF);
	  }

	  //sprintf(message, "Calibration in 3 seconds...\r\n");
	  //HAL_UART_Transmit(&huart2, (uint8_t*)&message, strlen(message), 0xFFFF);

	  //calibrate(10000, &hi2c1);

	  //sprintf(message, "Calibration Complete...\r\n");
	  //HAL_UART_Transmit(&huart2, (uint8_t*)&message, strlen(message), 0xFFFF);

}


void Application::Loop()
{

	float homelat = 0;
	float homelon = 0;
	float prevlat = 0;
	float prevlon = 0;

	uint32_t prev_update_tick = 0;
	static bool id_checked = false; // 标记开机是否查过ID

	while (1)
	{
		if ((HAL_GetTick() - TimerBMM150) > 100)
		{
			bmm150_mag_data value;
			bmm.read_mag_data();

			value.x = bmm.raw_mag_data.raw_datax - value_offset_x;
			value.y = bmm.raw_mag_data.raw_datay - value_offset_y;
			value.z = bmm.raw_mag_data.raw_dataz - value_offset_z;

			float xyHeading = atan2(value.x, value.y);
			float zxHeading = atan2(value.z, value.x);
			float heading = xyHeading;

			if (heading < 0) {
				heading += 2 * M_PI;
			}
			if (heading > 2 * M_PI) {
				heading -= 2 * M_PI;
			}

			float headingDegrees = heading * 180 / M_PI;
			float xyHeadingDegrees = xyHeading * 180 / M_PI;
			float zxHeadingDegrees = zxHeading * 180 / M_PI;

			//char RX_Buffer_Char[100];
			//sprintf(RX_Buffer_Char, "Hdg deg: %f  X-Y Hdg: %f  Z-X Hdg: %f \r\n", headingDegrees,xyHeadingDegrees, zxHeadingDegrees);
			//HAL_UART_Transmit(&huart2, (uint8_t*)&RX_Buffer_Char, strlen(RX_Buffer_Char), 0xFFFF);

			send_data.mx = bmm.raw_mag_data.raw_datax;
			send_data.my = bmm.raw_mag_data.raw_datay;
			send_data.mz = bmm.raw_mag_data.raw_dataz;

			TimerBMM150 = HAL_GetTick();
		}

		// 🚨 修复：开机前 2 秒找一次 ID，找不到就算了，绝不阻塞主循环
		if (!id_checked && (HAL_GetTick() < 10000))
		{
			GNSS_GetUniqID();
			GNSS_ParseBuffer();
			if (GNSS_Handle.uniqueID[0] != 0) {
				id_checked = true; // 找到了，不再查
			}
		} else {
			id_checked = true; // 超过2秒还没找到，放弃查找
		}

		if ((HAL_GetTick() - Timer) > 10) {

			GNSS_GetPVTData();
			GNSS_ParseBuffer();

			/*
			printf("Day: %d-%d-%d \n", GNSS_Handle.day, GNSS_Handle.month,GNSS_Handle.year);
			printf("Time: %d:%d:%d \n", GNSS_Handle.hour, GNSS_Handle.min,GNSS_Handle.sec);
			printf("Status of fix: %d \n", GNSS_Handle.fixType);
			printf("Latitude: %f \n", GNSS_Handle.fLat);
			printf("Longitude: %f \n",(float) GNSS_Handle.lon / 10000000.0);
			printf("Height above ellipsoid: %ld \n", GNSS_Handle.height);
			printf("Height above mean sea level: %ld \n", GNSS_Handle.hMSL);
			printf("Ground Speed (2-D): %ld \n", GNSS_Handle.gSpeed);
			printf("Unique ID: %04X %04X %04X %04X %04X \n",
					GNSS_Handle.uniqueID[0], GNSS_Handle.uniqueID[1],
					GNSS_Handle.uniqueID[2], GNSS_Handle.uniqueID[3],
					GNSS_Handle.uniqueID[4]);
			*/

			//计算当前位置相对起始位置距离
			float dist_home = get_distance(homelat, homelon, GNSS_Handle.fLat, GNSS_Handle.fLon);
			//计算上一次位置的距离
			float dist_prev = get_distance(prevlat, prevlon, GNSS_Handle.fLat, GNSS_Handle.fLon);

			static uint32_t prev_iTOW = 0;
			// 如果 iTOW 变了，说明 GPS 真正吐出了新数据
			//if (GNSS_Handle.iTOW != prev_iTOW)
			//{
			//	prev_iTOW = GNSS_Handle.iTOW;
			if (prev_update_tick != GNSS_Handle.update_tick)
			{
				/*
				printf("fix:%d, numsv:%d, lat:%f, lon:%f, hmsl:%ld, hacc:%ld, veln:%ld, vele:%ld, dhome:%f, dprev:%f\n",
						GNSS_Handle.fixType, GNSS_Handle.numSV, GNSS_Handle.fLat, GNSS_Handle.fLon,
						GNSS_Handle.hMSL, GNSS_Handle.hAcc, GNSS_Handle.velN, GNSS_Handle.velE,
						dist_home, dist_prev);
				*/
				prev_update_tick = GNSS_Handle.update_tick;

				//填充spi数据
				send_data.iTOW = GNSS_Handle.iTOW;
				send_data.fixType = GNSS_Handle.fixType;
				send_data.numSV = GNSS_Handle.numSV;
				send_data.fLon = GNSS_Handle.fLon;
				send_data.fLat = GNSS_Handle.fLat;
				send_data.hMSL = GNSS_Handle.hMSL;
				send_data.hAcc = GNSS_Handle.hAcc;
				send_data.velN = GNSS_Handle.velN;
				send_data.velE = GNSS_Handle.velE;

				uint8_t checksum = 0;
				for (int i=0; i<(sizeof(send_data) - 1); i++)
				{
					checksum += ((uint8_t *)&send_data)[i];
					//printf("%x ", ((uint8_t *)&spi_data)[i]);
				}
				//printf("\n");
				send_data.checksum = checksum;

				printf("fix:%d, numsv:%d, lat:%f, lon:%f, hmsl:%ld, hacc:%ld, veln:%ld, vele:%ld, checksum:%x\n",
						send_data.fixType, send_data.numSV, send_data.fLat, send_data.fLon,
						send_data.hMSL, send_data.hAcc, send_data.velN, send_data.velE,
						send_data.checksum);

				send_data_update_tick = HAL_GetTick();

				tx_buffer[0] = 0xAA;
				tx_buffer[1] = 0x55;
				tx_buffer[2] = sizeof(send_data);
				//MEM_BARRIER();
				for (int i = 0; i < sizeof(send_data); i++) {
					tx_buffer[3 + i] = ((uint8_t *)&send_data)[i];
				}

				//通过串口发送数据
				//HAL_UART_Transmit(&huart1, (const uint8_t *)&spi_data, sizeof(spi_data), 10);
				//for (int i=0; i<2; i++)
				{
					HAL_UART_Transmit(&huart1, tx_buffer, 3 + sizeof(send_data), 50);
					//HAL_Delay(3);
				}
				/* DMA方式有概率对方收到乱码，原因未知
				static uint32_t send_tick = 0;
				uint32_t now = HAL_GetTick();
				if ((huart1.gState == HAL_UART_STATE_READY && uart_send_done) || ((now - send_tick) > 200))
				//if (huart1.gState == HAL_UART_STATE_READY)
				{
					MEM_BARRIER();
					uart_send_done = false;
					HAL_UART_Transmit_DMA(&huart1, tx_buffer, 3 + sizeof(send_data));
					send_tick = now;
				}
				//*/

				//统计刷新频率
			    static uint32_t last_tick = 0;
			    static uint32_t call_count = 0;
			    const uint32_t now_tick = HAL_GetTick();
			    const uint32_t diff_ms = now_tick - last_tick;
			    call_count++;
			    static float freq_hz = 0;
				static uint32_t log_ms = 0;
			    if (diff_ms >= 1000) {
			        freq_hz = (float)call_count * 1000.0f / diff_ms;
			        call_count = 0;
			        last_tick = now_tick;
			    }
			}
			else
			{
				printf("no update\n");
			}
			if (GNSS_Handle.fixType >= 3 && GNSS_Handle.numSV >= 8 && homelat == 0 && homelon == 0)
			{
				homelat = GNSS_Handle.fLat;
				homelon = GNSS_Handle.fLon;
			}
			prevlat = GNSS_Handle.fLat;
			prevlon = GNSS_Handle.fLon;
			Timer = HAL_GetTick();
		}

		HAL_Delay(1);
	}
}


void Application::UartTxCplt()
{
	uart_send_done = true;
}

#ifdef __cplusplus
extern "C" {
#endif


// UART DMA 发送完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 确保是你使用的串口
    if (huart->Instance == USART1)
    {
        // 可以调用你 Application 类里的方法，或者留空
        app.UartTxCplt();
    }
}

#ifdef __cplusplus
}
#endif
