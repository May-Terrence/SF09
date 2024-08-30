/*
 * mtf01.cpp
 *
 *  Created on: 2024年3月5日
 *      Author: Terrence
 */

#include "mtf01.hpp"

/*
 	 此为光流测距一体模组数据接收程序
说明： 用户使用micolink_decode作为串口数据处理函数即可

距离有效值最小为10(mm),为0说明此时距离值不可用
光流速度值单位：cm/s@1m
飞控中只需要将光流速度值*高度，即可得到真实水平位移速度
计算公式：实际速度(cm/s)=光流速度*高度(m)
*/
MTF01 mtf01(USART2,(char*) "MTF01");
void USART2_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_IDLE(mtf01.huart))
	{
		LL_USART_ClearFlag_IDLE(mtf01.huart);
		LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_5);
		LL_DMA_ClearFlag_DME5(DMA1);
		LL_DMA_ClearFlag_HT5(DMA1);
		LL_DMA_ClearFlag_TC5(DMA1);
		LL_DMA_ClearFlag_TE5(DMA1);
		LL_DMA_ClearFlag_FE5(DMA1);
		LL_USART_ClearFlag_CM(USART2);
		LL_USART_ClearFlag_EOB(USART2);
		LL_USART_ClearFlag_FE(USART2);
		LL_USART_ClearFlag_LBD(USART2);
		LL_USART_ClearFlag_NE(USART2);
		LL_USART_ClearFlag_ORE(USART2);
		LL_USART_ClearFlag_PE(USART2);
		LL_USART_ClearFlag_RTO(USART2);
		LL_USART_ClearFlag_TC(USART2);
		LL_USART_ClearFlag_WKUP(USART2);
		LL_USART_ClearFlag_nCTS(USART2);
		LL_USART_ClearFlag_IDLE(USART2);
		mtf01.RxDataSize = MICOLINK_MAX_LEN - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_5);

		do{
			if(mtf01.RxRawDat[0]!=0xEF) break;
			if(mtf01.LockRx == HAL_LOCKED) break;
			mtf01.LockRx = HAL_LOCKED;
			memcpy(mtf01.RxDat, mtf01.RxRawDat, mtf01.RxDataSize);
			mtf01.RxDat[mtf01.RxDataSize] = 0;
			mtf01.LockRx = HAL_UNLOCKED;
			mtf01.RxFlag = true;   //收到完整一帧

			BaseType_t YieldRequired = xTaskResumeFromISR(laserFlowTaskHandle);
			if(YieldRequired==pdTRUE)
			{
				/*如果函数xTaskResumeFromISR()返回值为pdTRUE，那么说明要恢复的这个
				任务的任务优先级等于或者高于正在运行的任务(被中断打断的任务),所以在
				退出中断的时候一定要进行上下文切换！*/
				portYIELD_FROM_ISR(YieldRequired);
			}
		}while(0);
		LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, MICOLINK_MAX_LEN);
		LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
	}
}
void MTF01::MTF01_Init(void)
{
	LockRx = HAL_UNLOCKED;
	RxFlag = false;
	RxDataSize = 0;
	heightFil.CNT = 0;
	heightFil.CCR = 30;
	VelxFil.CNT = 0;
	VelxFil.CCR = 70;
	VelyFil.CNT = 0;
	VelyFil.CCR = 70;
	osDelay(250);

	/* 配置接收DMA */
	LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_5);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)&huart->RDR);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)RxRawDat);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, MICOLINK_MAX_LEN);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
	/* 配置接收DMA */

	LL_USART_EnableDMAReq_RX(huart);
	LL_USART_ClearFlag_IDLE(huart);
	LL_USART_EnableIT_IDLE(huart);
//	osDelay(250);
}
void MTF01::laserFlowRun(){
	if(RxFlag == false) return;
	RxFlag = false;
	for(uint8_t data : mtf01.RxDat){
		micolink_decode(data);
	}
}
void MTF01::micolink_decode(uint8_t data)
{

    if(micolink_parse_char(&msg, data) == false)
        return;

    switch(msg.msg_id)
    {
        case MICOLINK_MSG_ID_RANGE_SENSOR:
        {
            memcpy(&payload, msg.payload, msg.len);

            /*
                此处可获取传感器数据:

                距离        = payload.distance;
                强度        = payload.strength;
                精度        = payload.precision;
                距离状态    = payload.tof_status;
                光流速度x轴 = payload.flow_vel_x;
                光流速度y轴 = payload.flow_vel_y;
                光流质量    = payload.flow_quality;
                光流状态    = payload.flow_status;
            */
            break;
        }

        default:
            break;
        }
    laserFlow.height = static_cast<float>(payload.distance)/1000; //mm->m
    SlideFilt(&laserFlow.heightFil, &laserFlow.height, 1, &heightFil, 1);

    laserFlow.VelxRaw = static_cast<float>(payload.flow_vel_x)/100; //cm/s->m/s
    laserFlow.VelyRaw = static_cast<float>(payload.flow_vel_y)/100; //cm/s->m/s
    SlideFilt(&laserFlow.VelxRawFil, &laserFlow.VelxRaw, 1, &VelxFil, 1);
    SlideFilt(&laserFlow.VelyRawFil, &laserFlow.VelyRaw, 1, &VelyFil, 1);

    laserFlow.Velx = laserFlow.VelxRaw * laserFlow.height;
    laserFlow.Vely = laserFlow.VelyRaw * laserFlow.height;
    laserFlow.VelxFil = laserFlow.VelxRawFil* laserFlow.heightFil;
    laserFlow.VelyFil = laserFlow.VelyRawFil* laserFlow.heightFil;
	xQueueOverwrite(queuelaserFlow,&laserFlow);


}

bool MTF01::micolink_check_sum(MICOLINK_MSG_t* msg)
{
    uint8_t length = msg->len + 6;
    uint8_t temp[MICOLINK_MAX_LEN];
    uint8_t checksum = 0;

    memcpy(temp, msg, length);

    for(uint8_t i=0; i<length; i++)
    {
        checksum += temp[i];
    }

    if(checksum == msg->checksum)
        return true;
    else
        return false;
}

bool MTF01::micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data)
{
    switch(msg->status)
    {
    case 0:     //帧头
        if(data == MICOLINK_MSG_HEAD)
        {
            msg->head = data;
            msg->status++;
        }
        break;

    case 1:     // 设备ID
        msg->dev_id = data;
        msg->status++;
        break;

    case 2:     // 系统ID
        msg->sys_id = data;
        msg->status++;
        break;

    case 3:     // 消息ID
        msg->msg_id = data;
        msg->status++;
        break;

    case 4:     // 包序列
        msg->seq = data;
        msg->status++;
        break;

    case 5:     // 负载长度
        msg->len = data;
        if(msg->len == 0)
            msg->status += 2;
        else if(msg->len > MICOLINK_MAX_PAYLOAD_LEN)
            msg->status = 0;
        else
            msg->status++;
        break;

    case 6:     // 数据负载接收
        msg->payload[msg->payload_cnt++] = data;
        if(msg->payload_cnt == msg->len)
        {
            msg->payload_cnt = 0;
            msg->status++;
        }
        break;

    case 7:     // 帧校验
        msg->checksum = data;
        msg->status = 0;
        if(micolink_check_sum(msg))
        {
            return true;
        }

    default:
        msg->status = 0;
        msg->payload_cnt = 0;
        break;
    }

    return false;
}

extern "C" void laserFlow_main(void *argument)
{
	osDelay(500);//等待系统完成初始化
	mtf01.MTF01_Init();	//参数初始化
	for(;;)
	{
		vTaskSuspend(laserFlowTaskHandle);
//		osSemaphoreAcquire(semlaserFlow,0xffffffff);

		mtf01.laserFlowRun();
	}
}
