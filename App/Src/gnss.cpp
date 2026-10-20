/*
 * GNSS.c
 *
 *  Created on: 03.10.2020
 *      Author: SimpleMethod
 *
 *Copyright 2020 SimpleMethod
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy of
 *this software and associated documentation files (the "Software"), to deal in
 *the Software without restriction, including without limitation the rights to
 *use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *of the Software, and to permit persons to whom the Software is furnished to do
 *so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *THE SOFTWARE.
 ******************************************************************************
 */

#include "gnss.h"
#include "spi.h"
#include "common.h"
#include <cstdio>
#include <math.h>
#include <cstring>
#include "usart.h"


union u_Short uShort;
union i_Short iShort;
union u_Long uLong;
union i_Long iLong;

GNSS_StateHandle GNSS_Handle;
GNSS_StateHandle *GNSS = &GNSS_Handle;

#define MAX(x, y) ((x) > (y) ? (x) : (y))


#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define EARTH_RADIUS_M 6371000.0f // 地球平均半径（米）
#define DEG_TO_RAD     (PI / 180.0f)

/**
 * @brief 使用 Haversine 公式计算两个经纬度坐标之间的直线距离
 *
 * @param lat1 起点纬度 (度，如 22.123456f)
 * @param lon1 起点经度 (度，如 113.987654f)
 * @param lat2 终点纬度 (度)
 * @param lon2 终点经度 (度)
 * @return float 两点之间的直线距离 (单位：米)
 */
float get_distance(float lat1, float lon1, float lat2, float lon2)
{
    // 1. 将经纬度差值转换为弧度
    float dlat = (lat2 - lat1) * DEG_TO_RAD;
    float dlon = (lon2 - lon1) * DEG_TO_RAD;

    // 2. 将纬度转换为弧度
    float lat1_rad = lat1 * DEG_TO_RAD;
    float lat2_rad = lat2 * DEG_TO_RAD;

    // 3. 计算 Haversine 公式的核心部分
    float a = sinf(dlat / 2.0f) * sinf(dlat / 2.0f) +
              cosf(lat1_rad) * cosf(lat2_rad) *
              sinf(dlon / 2.0f) * sinf(dlon / 2.0f);

    // 4. 计算中心角 (c)
    float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));

    // 5. 乘以地球半径得到距离
    return EARTH_RADIUS_M * c;
}

uint8_t tx_buffer[180];
uint8_t rx_buffer[180];

#if 0
uint8_t spi_tx_buffer[180];
uint8_t spi_rx_buffer[180];

/**
 * @brief 通用 SPI 发送指令并读取响应
 * @param cmd 指令数组
 * @param cmd_len 指令长度
 * @param read_len 需要读取的响应长度
 */
void GNSS_SPI_SendAndReceive(const uint8_t* cmd, uint16_t cmd_len, uint16_t read_len) {
    // 1. 统一加 50 的余量，应对大量前导 0xFF (假设最大报文 100 字节)
    uint16_t actual_read = read_len + 50;
    uint16_t total_len = cmd_len + actual_read;

    if (total_len > 180) return; // 防止溢出 spi_tx_buffer[180]

    // 2. 填充发送缓冲区：指令 + 大量 0xFF
    for (int i = 0; i < cmd_len; i++) {
        spi_tx_buffer[i] = cmd[i];
    }
    for (int i = 0; i < actual_read; i++) {
        spi_tx_buffer[cmd_len + i] = 0xFF;
    }

    HAL_GPIO_WritePin(GPIOB, GPS_NSS_Pin, GPIO_PIN_RESET);
    delay_us(10);

    // 3. 一次性连续传输！速度极快，SCK 不会断续，模块不会卡死
    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi2, spi_tx_buffer, spi_rx_buffer, total_len, 200);

    delay_us(10);
    HAL_GPIO_WritePin(GPIOB, GPS_NSS_Pin, GPIO_PIN_SET);

    if (ret != HAL_OK) {
        printf("SPI Transfer Error: %d\n", ret);
        return;
    }

    // 4. 在接收到的数据中，寻找 0xB5 0x62 帧头
    bool found_header = false;
    // 限制搜索范围，保证后面至少还有 read_len 长度的数据可用
    for (int i = 0; i <= (total_len - read_len); i++) {
        if (spi_rx_buffer[i] == 0xB5 && spi_rx_buffer[i + 1] == 0x62) {
            // 找到了！把从这个位置开始的 read_len 个字节，移动到工作缓冲区的开头
            memmove(GNSS_Handle.uartWorkingBuffer, &spi_rx_buffer[i], read_len);
            found_header = true;
            break;
        }
    }

    // 5. 如果没找到，说明模块没新数据，正常现象，不用报错
    if (!found_header) {
        // printf("Header not found (Module has no new data yet).\n");
    }
}
#endif


/**
 * @brief 通用 SPI 发送指令并读取响应
 * @param cmd 指令数组
 * @param cmd_len 指令长度
 * @param read_len 需要读取的响应长度
 */
void GNSS_Uart_SendAndReceive(const uint8_t* cmd, uint16_t cmd_len, uint16_t read_len) {
	HAL_StatusTypeDef ret;
	//ret = HAL_UART_Transmit_DMA(&huart2, cmd, cmd_len);
	ret = HAL_UART_Transmit(&huart2, cmd, cmd_len, 50);
    if (ret != HAL_OK) {
        printf("uart Transfer Error: %d\n", ret);
        return;
    }
    /*
    if (!is_data_receive)
    {
    	return;
    }
    is_data_receive = false;
    ret = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, GNSS_Handle.uartWorkingBuffer, sizeof(GNSS_Handle.uartWorkingBuffer));
    //*/
    ret = HAL_UART_Receive(&huart2, GNSS_Handle.uartWorkingBuffer, read_len, 200);
    if (ret != HAL_OK) {
    	memset(GNSS_Handle.uartWorkingBuffer, 0, sizeof(GNSS_Handle.uartWorkingBuffer));
    	//is_data_receive = true;
        printf("uart Receive Error: %d\n", ret);
        return;
    }
    /*
	for (int i=0; i<200; i++)
	{
		if (is_data_receive)
		{
			break;
		}
		HAL_Delay(1);
	}
	*/
}


/*!
 * Structure initialization.
 * @param GNSS Pointer to main GNSS structure.
 * @param huart Pointer to uart handle.
 */
void GNSS_Init() {
	GNSS->year = 0;
	GNSS->month = 0;
	GNSS->day = 0;
	GNSS->hour = 0;
	GNSS->min = 0;
	GNSS->sec = 0;
	GNSS->fixType = 0;
	GNSS->lon = 0;
	GNSS->lat = 0;
	GNSS->height = 0;
	GNSS->hMSL = 0;
	GNSS->hAcc = 0;
	GNSS->vAcc = 0;
	GNSS->gSpeed = 0;
	GNSS->headMot = 0;
}

/*!
 * Searching for a header in data buffer and matching class and message ID to buffer data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParseBuffer() {
    // 在前 20 个字节里寻找 UBX 头部 0xB5 0x62
    for (int var = 0; var <= 50; ++var) {
        if (GNSS_Handle.uartWorkingBuffer[var] == 0xB5 &&
            GNSS_Handle.uartWorkingBuffer[var + 1] == 0x62) {

            // 如果不在数组开头，将其移动到开头 (对齐数据)
            if (var > 0) {
                memmove(GNSS_Handle.uartWorkingBuffer, GNSS_Handle.uartWorkingBuffer + var, 180 - var);
            }

            //验证校验和，防止解析到错误数据
			if (!GNSS_VerifyChecksum(GNSS_Handle.uartWorkingBuffer)) {
				printf("UBX Checksum Failed!\n");
				break; // 校验失败，丢弃这一帧，不再往下解析
			}

            // 现在数据已对齐，0xB5 在索引 0，Class 在 2，ID 在 3
            if (GNSS_Handle.uartWorkingBuffer[2] == 0x27 && GNSS_Handle.uartWorkingBuffer[3] == 0x03) {
                GNSS_ParseUniqID();
                GNSS_Handle.update_tick = HAL_GetTick();
            } else if (GNSS_Handle.uartWorkingBuffer[2] == 0x01 && GNSS_Handle.uartWorkingBuffer[3] == 0x21) {
                GNSS_ParseNavigatorData();
                GNSS_Handle.update_tick = HAL_GetTick();
            } else if (GNSS_Handle.uartWorkingBuffer[2] == 0x01 && GNSS_Handle.uartWorkingBuffer[3] == 0x07) {
                GNSS_ParsePVTData();
                GNSS_Handle.update_tick = HAL_GetTick();
            } else if (GNSS_Handle.uartWorkingBuffer[2] == 0x01 && GNSS_Handle.uartWorkingBuffer[3] == 0x02) {
                GNSS_ParsePOSLLHData();
                GNSS_Handle.update_tick = HAL_GetTick();
            }
            break; // 找到并解析完毕，退出循环
        }
    }

}

/*!
 * Make request for unique chip ID data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetUniqID() {
	/*
	HAL_UART_Transmit_DMA(GNSS->huart, getDeviceID,
			sizeof(getDeviceID) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 17);
	*/
	GNSS_Uart_SendAndReceive(getDeviceID, sizeof(getDeviceID), 17);
}

/*!
 * Make request for UTC time solution data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetNavigatorData() {
	/*
	HAL_UART_Transmit_DMA(GNSS->huart, getNavigatorData,
			sizeof(getNavigatorData) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 28);
	*/
	GNSS_Uart_SendAndReceive(getNavigatorData, sizeof(getNavigatorData), 28);
}

/*!
 * Make request for geodetic position solution data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetPOSLLHData() {
	/*
	HAL_UART_Transmit_DMA(GNSS->huart, getPOSLLHData,
			sizeof(getPOSLLHData) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 36);
	*/
	GNSS_Uart_SendAndReceive(getPOSLLHData, sizeof(getPOSLLHData), 36);
}

/*!
 * Make request for navigation position velocity time solution data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetPVTData() {
	/*
	HAL_UART_Transmit_DMA(GNSS->huart, getPVTData,
			sizeof(getPVTData) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 100);
	*/
	GNSS_Uart_SendAndReceive(getPVTData, sizeof(getPVTData), 100);
}

/*!
 * Parse data to unique chip ID standard.
 * Look at: 32.19.1.1 u-blox 8 Receiver description
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParseUniqID() {
    for (int var = 0; var < 5; ++var) {
        GNSS->uniqueID[var] = GNSS_Handle.uartWorkingBuffer[10 + var];
    }
}

/*!
 * Changing the GNSS mode.
 * Look at: 32.10.19 u-blox 8 Receiver description
 */
void GNSS_SetMode(short gnssMode) {

	if (gnssMode == 0) {
		//HAL_UART_Transmit_DMA(GNSS->huart, setPortableMode,sizeof(setPortableMode) / sizeof(uint8_t));
		GNSS_Uart_SendAndReceive(setPortableMode, sizeof(setPortableMode), 10);
	} else if (gnssMode == 1) {
		//HAL_UART_Transmit_DMA(GNSS->huart, setStationaryMode,sizeof(setStationaryMode) / sizeof(uint8_t));
		GNSS_Uart_SendAndReceive(setStationaryMode, sizeof(setStationaryMode), 10);
	} else if (gnssMode == 2) {
		//HAL_UART_Transmit_DMA(GNSS->huart, setPedestrianMode,sizeof(setPedestrianMode) / sizeof(uint8_t));
		GNSS_Uart_SendAndReceive(setPedestrianMode, sizeof(setPedestrianMode), 10);
	} else if (gnssMode == 3) {
		//HAL_UART_Transmit_DMA(GNSS->huart, setAutomotiveMode,sizeof(setAutomotiveMode) / sizeof(uint8_t));
		GNSS_Uart_SendAndReceive(setAutomotiveMode, sizeof(setAutomotiveMode), 10);
	} else if (gnssMode == 4) {
		//HAL_UART_Transmit_DMA(GNSS->huart, setAutomotiveMode,sizeof(setAutomotiveMode) / sizeof(uint8_t));
		GNSS_Uart_SendAndReceive(setAutomotiveMode, sizeof(setAutomotiveMode), 10);
	} else if (gnssMode == 5) {
		//HAL_UART_Transmit_DMA(GNSS->huart, setAirbone1GMode,sizeof(setAirbone1GMode) / sizeof(uint8_t));
		GNSS_Uart_SendAndReceive(setAirbone1GMode, sizeof(setAirbone1GMode), 10);
	} else if (gnssMode == 6) {
		//HAL_UART_Transmit_DMA(GNSS->huart, setAirbone2GMode,sizeof(setAirbone2GMode) / sizeof(uint8_t));
		GNSS_Uart_SendAndReceive(setAirbone2GMode, sizeof(setAirbone2GMode), 10);
	} else if (gnssMode == 7) {
		//HAL_UART_Transmit_DMA(GNSS->huart, setAirbone4GMode,sizeof(setAirbone4GMode) / sizeof(uint8_t));
		GNSS_Uart_SendAndReceive(setAirbone4GMode, sizeof(setAirbone4GMode), 10);
	} else if (gnssMode == 8) {
		//HAL_UART_Transmit_DMA(GNSS->huart, setWirstMode,sizeof(setWirstMode) / sizeof(uint8_t));
		GNSS_Uart_SendAndReceive(setWirstMode, sizeof(setWirstMode), 10);
	} else if (gnssMode == 9) {
		//HAL_UART_Transmit_DMA(GNSS->huart, setBikeMode,sizeof(setBikeMode) / sizeof(uint8_t));
		GNSS_Uart_SendAndReceive(setBikeMode, sizeof(setBikeMode), 10);
	}

}
/*!
 * Parse data to navigation position velocity time solution standard.
 * Look at: 32.17.15.1 u-blox 8 Receiver description.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParsePVTData() {
	// 偏移量 0 (索引 6) 是 iTOW
	for (int var = 0; var < 4; ++var) {
		uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 6];
	}
	GNSS_Handle.iTOW = uLong.uLong;
	// 偏移量4 (索引10) 开始是 year
	uShort.bytes[0] = GNSS_Handle.uartWorkingBuffer[10];
	GNSS->yearBytes[0]=GNSS_Handle.uartWorkingBuffer[10];
	uShort.bytes[1] = GNSS_Handle.uartWorkingBuffer[11];
	GNSS->yearBytes[1]=GNSS_Handle.uartWorkingBuffer[11];
	GNSS->year = uShort.uShort;

	GNSS->month = GNSS_Handle.uartWorkingBuffer[12]; // 偏移6
	GNSS->day = GNSS_Handle.uartWorkingBuffer[13];   // 偏移7
	GNSS->hour = GNSS_Handle.uartWorkingBuffer[14];  // 偏移8
	GNSS->min = GNSS_Handle.uartWorkingBuffer[15];   // 偏移9
	GNSS->sec = GNSS_Handle.uartWorkingBuffer[16];   // 偏移10

	// 🚨 新增：解析卫星数 numSV (偏移量 23 -> 索引 29) [正确]
	GNSS->numSV = GNSS_Handle.uartWorkingBuffer[29];

	// fixType (偏移量 20 -> 索引 26) [正确]
	GNSS->fixType = GNSS_Handle.uartWorkingBuffer[26];

    // lon 在有效载荷偏移 24 处 -> 数组索引 6 + 24 = 30
    for (int var = 0; var < 4; ++var) {
        iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 30];
        GNSS->lonBytes[var]= GNSS_Handle.uartWorkingBuffer[var + 30];
    }
    GNSS->lon = iLong.iLong;
    GNSS->fLon=(float)iLong.iLong/10000000.0;

    for (int var = 0; var < 4; ++var) {
        iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 34];
        GNSS->latBytes[var]=GNSS_Handle.uartWorkingBuffer[var + 34];
    }
    GNSS->lat = iLong.iLong;
    GNSS->fLat=(float)iLong.iLong/10000000.0;
    for (int var = 0; var < 4; ++var) {
        iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 38];
    }
    GNSS->height = iLong.iLong;

    for (int var = 0; var < 4; ++var) {
        iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 42];
        GNSS->hMSLBytes[var] = GNSS_Handle.uartWorkingBuffer[var + 42];
    }
    GNSS->hMSL = iLong.iLong;

    for (int var = 0; var < 4; ++var) {
        uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 46];
    }
    GNSS->hAcc = uLong.uLong;

    for (int var = 0; var < 4; ++var) {
        uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 50];
    }
    GNSS->vAcc = uLong.uLong;

    for (int var = 0; var < 4; ++var) {
        iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 54];
    }
    GNSS->velN = iLong.iLong;

    for (int var = 0; var < 4; ++var) {
        iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 58];
    }
    GNSS->velE = iLong.iLong;

    for (int var = 0; var < 4; ++var) {
        iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 62];
    }
    GNSS->velD = iLong.iLong;

    for (int var = 0; var < 4; ++var) {
        iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 66];
        GNSS->gSpeedBytes[var] = GNSS_Handle.uartWorkingBuffer[var + 66];
    }
    GNSS->gSpeed = iLong.iLong;

    for (int var = 0; var < 4; ++var) {
        iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 70];
    }
    GNSS->headMot = iLong.iLong * 1e-5;
}

/*!
 * Parse data to UTC time solution standard.
 * Look at: 32.17.30.1 u-blox 8 Receiver description.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParseNavigatorData() {
    // 偏移量0 (索引6) 直接就是 year
    uShort.bytes[0] = GNSS_Handle.uartWorkingBuffer[6];
    uShort.bytes[1] = GNSS_Handle.uartWorkingBuffer[7];
    GNSS->year = uShort.uShort;

    GNSS->month = GNSS_Handle.uartWorkingBuffer[8];  // 偏移2
    GNSS->day = GNSS_Handle.uartWorkingBuffer[9];    // 偏移3
    GNSS->hour = GNSS_Handle.uartWorkingBuffer[10];  // 偏移4
    GNSS->min = GNSS_Handle.uartWorkingBuffer[11];   // 偏移5
    GNSS->sec = GNSS_Handle.uartWorkingBuffer[12];   // 偏移6
}

/*!
 * Parse data to geodetic position solution standard.
 * Look at: 32.17.14.1 u-blox 8 Receiver description.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParsePOSLLHData() {
	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 10];
	}
	GNSS->lon = iLong.iLong;
	GNSS->fLon=(float)iLong.iLong/10000000.0;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 14];
	}
	GNSS->lat = iLong.iLong;
	GNSS->fLat=(float)iLong.iLong/10000000.0;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 18];
	}
	GNSS->height = iLong.iLong;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 22];
	}
	GNSS->hMSL = iLong.iLong;

	for (int var = 0; var < 4; ++var) {
		uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 26];
	}
	GNSS->hAcc = uLong.uLong;

	for (int var = 0; var < 4; ++var) {
		uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 30];
	}
	GNSS->vAcc = uLong.uLong;
}

/*!
 *  Sends the basic configuration: Activation of the UBX standard, change of NMEA version to 4.10 and turn on of the Galileo system.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_LoadConfig() {
	/*
	HAL_UART_Transmit_DMA(GNSS->huart, configUBX,
			sizeof(configUBX) / sizeof(uint8_t));
	HAL_Delay(250);
	HAL_UART_Transmit_DMA(GNSS->huart, setNMEA410,
			sizeof(setNMEA410) / sizeof(uint8_t));
	HAL_Delay(250);
	HAL_UART_Transmit_DMA(GNSS->huart, setGNSS,
			sizeof(setGNSS) / sizeof(uint8_t));
	HAL_Delay(250);
	*/


	GNSS_Uart_SendAndReceive(configUBX, sizeof(configUBX), 10); // 发配置不需要读太多
	HAL_Delay(250);
	GNSS_Uart_SendAndReceive(setNMEA410, sizeof(setNMEA410), 10);
	HAL_Delay(250);
	GNSS_Uart_SendAndReceive(setGNSS, sizeof(setGNSS), 10);
	HAL_Delay(250);

	GNSS_Uart_SendAndReceive(setRate20Hz, sizeof(setRate20Hz), 10);
	HAL_Delay(250); // 给模块一点处理时间
}



/*!
 *  @brief 验证 UBX 报文的校验和
 *  @param buffer 数据缓冲区 (必须从 0xB5 开始)
 *  @return bool 校验通过返回 true，否则返回 false
 */
bool GNSS_VerifyChecksum(uint8_t *buffer) {
    if (buffer[0] != 0xB5 || buffer[1] != 0x62) return false;

    uint8_t CK_A = 0;
    uint8_t CK_B = 0;

    uint16_t payload_len = buffer[4] | (buffer[5] << 8);

    // 🚨 新增：如果算出来的长度大得离谱，说明数据是乱码，直接返回失败，防止越界死机
    if (payload_len > 110) return false;

    for (uint16_t i = 2; i < (6 + payload_len); i++) {
        CK_A += buffer[i];
        CK_B += CK_A;
    }

    uint8_t expected_CK_A = buffer[6 + payload_len];
    uint8_t expected_CK_B = buffer[6 + payload_len + 1];

    return (CK_A == expected_CK_A && CK_B == expected_CK_B);
}

/*!
 *  @brief 为构造的 UBX 报文计算并追加校验和
 *  @param buffer 包含 [0xB5, 0x62, Class, ID, Len_LSB, Len_MSB, Payload...] 的缓冲区
 *  @param payload_len 有效载荷长度
 *  @return uint16_t 包含校验和后的总报文长度
 */
uint16_t GNSS_AppendChecksum(uint8_t *buffer, uint16_t payload_len) {
    uint8_t CK_A = 0;
    uint8_t CK_B = 0;

    // 填充长度字段
    buffer[4] = payload_len & 0xFF;
    buffer[5] = (payload_len >> 8) & 0xFF;

    // 计算校验和 (从索引 2 到 5 + payload_len)
    for (uint16_t i = 2; i < (6 + payload_len); i++) {
        CK_A += buffer[i];
        CK_B += CK_A;
    }

    // 追加校验和
    buffer[6 + payload_len] = CK_A;
    buffer[6 + payload_len + 1] = CK_B;

    // 返回总长度
    return 6 + payload_len + 2;
}

