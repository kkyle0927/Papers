#include "Basics.h"

void UpdateXsensImuEnable(void)
{
    if (!xsensIMUenableRes) {
        if (XM_EnableExternalImu()) {
            HAL_Delay(50);
            xsensIMUenableRes = true;
        }
    }
}

void FillAndSendSavingData(void)
{
    // 1) loopCnt 채우고 증가
    SavingData.loopCnt = s_dataSaveLoopCnt++;

    // 2) payload 필드 채우기
    SavingData.h10Mode        = (uint8_t)XM.status.h10.h10Mode;
    SavingData.h10AssistLevel = XM.status.h10.h10AssistLevel;
    SavingData.SmartAssist    = SmartAssist;

    SavingData.leftHipAngle   = XM.status.h10.leftHipAngle;
    SavingData.rightHipAngle  = XM.status.h10.rightHipAngle;
    SavingData.leftThighAngle = XM.status.h10.leftThighAngle;
    SavingData.rightThighAngle= XM.status.h10.rightThighAngle;

    SavingData.leftHipTorque      = XM.status.h10.leftHipTorque;
    SavingData.rightHipTorque     = XM.status.h10.rightHipTorque;
    SavingData.leftHipMotorAngle  = XM.status.h10.leftHipMotorAngle;
    SavingData.rightHipMotorAngle = XM.status.h10.rightHipMotorAngle;

    SavingData.leftHipImuGlobalAccX = XM.status.h10.leftHipImuGlobalAccX;
    SavingData.leftHipImuGlobalAccY = XM.status.h10.leftHipImuGlobalAccY;
    SavingData.leftHipImuGlobalAccZ = XM.status.h10.leftHipImuGlobalAccZ;
    SavingData.leftHipImuGlobalGyrX = XM.status.h10.leftHipImuGlobalGyrX;
    SavingData.leftHipImuGlobalGyrY = XM.status.h10.leftHipImuGlobalGyrY;
    SavingData.leftHipImuGlobalGyrZ = XM.status.h10.leftHipImuGlobalGyrZ;

    SavingData.rightHipImuGlobalAccX = XM.status.h10.rightHipImuGlobalAccX;
    SavingData.rightHipImuGlobalAccY = XM.status.h10.rightHipImuGlobalAccY;
    SavingData.rightHipImuGlobalAccZ = XM.status.h10.rightHipImuGlobalAccZ;
    SavingData.rightHipImuGlobalGyrX = XM.status.h10.rightHipImuGlobalGyrX;
    SavingData.rightHipImuGlobalGyrY = XM.status.h10.rightHipImuGlobalGyrY;
    SavingData.rightHipImuGlobalGyrZ = XM.status.h10.rightHipImuGlobalGyrZ;

    SavingData.is_moving = (int32_t)s_is_moving;
    SavingData.hc_count  = (int32_t)hc_count;
    SavingData.R_count_upeak = (int32_t)R_count_upeak;
    SavingData.L_count_upeak = (int32_t)L_count_upeak;
    SavingData.R_count_dpeak = (int32_t)R_count_dpeak;
    SavingData.L_count_dpeak = (int32_t)L_count_dpeak;

    SavingData.tau_max_setting = (uint8_t)tau_max_setting;

    SavingData.s_gait_mode = (uint8_t)s_gait_mode;
    SavingData.s_g_knn_conf    = (float)s_g_knn_conf;

    SavingData.T_swing_ms     = (float)T_swing_ms;
    SavingData.T_swing_SOS_ms = (float)T_swing_SOS_ms;
    SavingData.T_swing_STS_ms = (float)T_swing_STS_ms;
    SavingData.s_vel_HC       = (float)s_vel_HC_dbg;
    SavingData.s_T_HC_s       = (float)s_T_HC_s_dbg;
    SavingData.s_norm_vel_HC  = (float)s_dbg_norm_vel_HC;
    SavingData.s_norm_T_HC    = (float)s_dbg_norm_T_HC;
    SavingData.s_scaling_X    = (float)s_dbg_scaling_X;
    SavingData.s_scaling_Y    = (float)s_dbg_scaling_Y;

    // 3) 헤더/CRC 설정
    SavingData.sof = TRANSMIT_SOF;
    SavingData.len = (uint16_t)sizeof(SavingData_t);

    // crc 자신을 제외한 전체에 대해 CRC 계산
    uint16_t crc = CalcCrc16((const uint8_t*)&SavingData,
                              sizeof(SavingData_t) - sizeof(SavingData.crc));
    SavingData.crc = crc;

    // 4) 전송 (리턴값은 필요하면 체크)
    (void)XM_SendUsbData(&SavingData, sizeof(SavingData_t));
}

uint16_t CalcCrc16(const uint8_t* data, uint32_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void UpdateRecordingState(void)
{
    RecordingState_t prev = Recording;

    XM_button2  = XM_GetButtonEvent(XM_BTN_2);
    sync_signal = XM_DigitalRead(XM_EXT_DIO_3);

    if (sync_signal == XM_HIGH) {
        Recording = Record_ON;   // SYNC HIGH → 강제 ON
    } else {
        if (XM_button2 == XM_BTN_CLICK) {
            // 토글
            Recording = (Recording == Record_ON) ? Record_OFF : Record_ON;
        }
        // 버튼이 없으면 기존 상태 유지
    }

    // SYNC H->L 또는 ON->OFF 시 카운터 리셋
    if (sync_signal_pre == XM_HIGH && sync_signal == XM_LOW) {
        Recording = Record_OFF;
        s_dataSaveLoopCnt = 0;
    } else if (prev == Record_ON && Recording == Record_OFF) {
        s_dataSaveLoopCnt = 0;
    }

    sync_signal_pre = sync_signal;

    if (Recording == Record_ON) {
        XM_SetLedEffect(3, XM_LED_HEARTBEAT, 1000);
        FillAndSendSavingData();
    } else {
        XM_SetLedEffect(3, XM_LED_OFF, 1000);
    }
}

void UpdateSmartAssistState(void)
{
    // 1) H10 모드가 STANDBY이면 무조건 SmartAssist OFF
    if (XM.status.h10.h10Mode == XM_H10_MODE_STANDBY) {
        if (SmartAssist != SmartAssist_OFF) {
            SmartAssist = SmartAssist_OFF;
            XM_SetH10AssistExistingMode(false);
        }

        // STANDBY 상태에서는 BTN1 이벤트를 읽어서 그냥 버린다.
        // (STANDBY에서 누른 클릭이 나중에 ACTIVE 모드에서 반영되지 않도록)
        (void)XM_GetButtonEvent(XM_BTN_1);

        return;
    }

    // 2) 버튼 입력 처리 (BTN1)
    XM_button1 = XM_GetButtonEvent(XM_BTN_1);
    if (XM_button1 != XM_BTN_CLICK) {
        return;
    }

    // 3) SmartAssist 토글
    if (SmartAssist == SmartAssist_ON) {
        SmartAssist = SmartAssist_OFF;
        XM_SetH10AssistExistingMode(false);
    } else {
        SmartAssist = SmartAssist_ON;
        XM_SetH10AssistExistingMode(true);
    }
}

