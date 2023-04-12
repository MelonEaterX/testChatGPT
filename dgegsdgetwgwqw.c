
/*******************************************************************************************
* Function: FISD_vSeatbeltDetect
* Description:下面的代码和逻辑是基于bx11项目修改的。
*             实现V0.8版本需求，12.6.2.54.6节关于安全带指示灯和安全带报警音策略，
*             具体需求，请阅读文档
*             TAN YI
* Parameters: none
* Return: none
********************************************************************************************/
static void LEDAppl_SeatbeltDetect(void)
{
    static uint32 u32DriveDistance = 0u;                    /* 用于记录IGN ON后的行驶距离，用于判断IGN ON后是否行驶超过300m */
    static uint16 u16SeatbeltIndTimer = TIMECOUNT_62S;      /* 用于安全带指示灯，第一点需求62秒计时 */
    static bool boLastRLStatus = true;                      /* 上一次的RL安全带状态，用于判断RL安全带由扣上到解开动作 */
    static bool boLastRMStatus = true;                      /* 上一次的RM安全带状态，用于判断RM安全带由扣上到解开动作 */
    static bool boLastRRStatus = true;                      /* 上一次的RR安全带状态，用于判断RR安全带由扣上到解开动作 */

    static bool boPreRLWarn = false;
    static bool boPreRMWarn = false;
    static bool boPreRRWarn = false;

    static bool boDrvWarn = false;                          /* 用于记录当前Driver安全带报警状态 */
    static bool boPasWarn = false;                          /* 用于记录当前Passenger安全带报警状态 */
    static bool boRLWarn = false;                           /* 用于记录当前RL安全带报警状态，静态变量，某些逻辑下，需要保持上一次的值，用于逻辑处理。 */
    static bool boRMWarn = false;                           /* 用于记录当前RM安全带报警状态，静态变量，某些逻辑下，需要保持上一次的值，用于逻辑处理。 */
    static bool boRRWarn = false;                           /* 用于记录当前RR安全带报警状态，静态变量，某些逻辑下，需要保持上一次的值，用于逻辑处理。 */
    static bool boDrvWarnLast = false;                      /* 用于判断Driver安全带报警状态由off变成on */
    static bool boPasWarnLast = false;                      /* 用于判断Passenger安全带报警状态由off变成on */
    static bool boRLWarnLast = false;                       /* 用于判断RL安全带报警状态由off变成on */
    static bool boRMWarnLast = false;                       /* 用于判断RM安全带报警状态由off变成on */
    static bool boRRWarnLast = false;                       /* 用于判断RR安全带报警状态由off变成on */

    static uint32 u32FDBuzTimer = 0u;                       /* 用于V0.8版本需求，12.6.2.54.6节第五点，前排安全带报警计时120秒 */
    static uint32 u32FPBuzTimer = 0u;                       /* 用于V0.8版本需求，12.6.2.54.6节第五点，前排安全带报警计时120秒 */
    static uint32 u32RLBuzTimer = 0u;                       /* 用于V0.8版本需求，12.6.2.54.6节第五点，后排安全带报警计时32秒 */
    static uint32 u32RMBuzTimer = 0u;                       /* 用于V0.8版本需求，12.6.2.54.6节第五点，后排安全带报警计时32秒 */
    static uint32 u32RRBuzTimer = 0u;                       /* 用于V0.8版本需求，12.6.2.54.6节第五点，后排安全带报警计时32秒 */
    static uint8 u8FDBuzLevel = 0u;
    static uint8 u8FPBuzLevel = 0u;
    static uint8 u8RBuzLevel[3] = {0u, 0u, 0u};
    static uint8 u8BuzLevel = 0u;                           /* 用于V0.8版本需求，12.6.2.54.6节第五点，当前安全带报警音等级 */

    bool boCurFlash1HzSt = false;
    bool boCurFlash05HzSt = false;

    tenIndicatorStatus enLastLedSt = nInd_Off;
    uint16 u16VehSpeed = 0u;                                /* 用于临时记录当前车速，判断安全带指示灯行为和安全带报警音等级 */
    bool boGearR = false;                                   /* 用于临时记录当前档位状态是否为倒车档，用于判断安全带报警灯的第一条需求中关于倒车的要求和安全带报警音中关于倒车的要求 */

    bool boReatOutCfgFlag = false;                          /* 是否配置了后排安全带功能。 */

    boCurFlash1HzSt = SYN_boGetEntireFlashStatus(nFlash500msOn500msOff);
    boCurFlash05HzSt = SYN_boGetEntireFlashStatus(nFlash1000msOn1000msOff);

    boReatOutCfgFlag = Rte_LEDApp_boGetCONFIG_Rear_seat_belt_output();

    if ((IGN_ON == LEDApp_u8IgnState) && \
        (true == LEDApp_boHmiIgnOnReady))                   /*IGN ON && HMI已经准备好了可以显示图标了。*/
    {
        if (false == LEDApp_boSelfCheckEnableFlag)
        {
            u16VehSpeed = Rte_LEDAppl_VehicleSpeed_Get();
            boGearR = Rte_LEDAppl_TargetGearIsR_Get();

            if (0u == u32DriveDistance)                         /* 上电后，需要获取到当前里程 */
            {
                u32DriveDistance = Rte_LEDApp_u32GetOdoValue();
            }
        
            /* Drv安全带报警激活，V0.8版本需求，12.6.2.54.6节第四点，h)主安全带未系报警 */
            if ((0u == FISD_u16SignalValue[nSigFISD_ACU_DrvSeatbeltBuckleInvalid]) && \
                (1u == FISD_u16SignalValue[nSigFISD_ACU_DrvSeatbeltBucklestatus]))
            {
                boDrvWarn = true;
            }
            else
            {
                boDrvWarn = false;
            }

            /* Passenger安全带报警激活，V0.8版本需求，12.6.2.54.6节第四点，h)副安全带未系报警 */
            if ((0u == FISD_u16SignalValue[nSigFISD_ACU_PassSeatbeltBuckleInvalid]) && \
                (1u == FISD_u16SignalValue[nSigFISD_ACU_PassSeatOccupantSensorStat]) && \
                (1u == FISD_u16SignalValue[nSigFISD_ACU_PassSeatbeltWarning]))
            {
                boPasWarn = true;
            }
            else
            {
                boPasWarn = false;
            }

            if (((Rte_LEDAppl_VehicleMoveSts_Get() == true) \
                    || ((FISD_u16SignalValue[nSigFISD_BCM_RearLeftDoorAjarStatus] == 0u) \
                        && (FISD_u16SignalValue[nSigFISD_BCM_RearRightDoorAjarStatus] == 0u))) \
                && (true == boReatOutCfgFlag))
            {
                /* RL安全带报警激活，V0.8版本需求，12.6.2.54.6节第四点，i)未系报警 */
                if ((false == boLastRLStatus) && \
                    (1u == FISD_u16SignalValue[nSigFISD_ACU_2nd_LSeatbeltBucklestatus]))
                {
                    boPreRLWarn = true;
                    if (Rte_LEDAppl_TargetGearIsDNM_Get() == true)
                    {
                        u16SeatbeltIndTimer = 0u;
                    }
                }
                else if (0u == FISD_u16SignalValue[nSigFISD_ACU_2nd_LSeatbeltBucklestatus])
                {
                    boPreRLWarn = false;
                }

                /* RM安全带报警激活，V0.8版本需求，12.6.2.54.6节第四点，i)未系报警 */
                if ((false == boLastRMStatus) && \
                    (1u == FISD_u16SignalValue[nSigFISD_ACU_2nd_MSeatbeltBucklestatus]))
                {
                    boPreRMWarn = true;
                    if (Rte_LEDAppl_TargetGearIsDNM_Get() == true)
                    {
                        u16SeatbeltIndTimer = 0u;
                    }
                }
                else if (0u == FISD_u16SignalValue[nSigFISD_ACU_2nd_MSeatbeltBucklestatus])
                {
                    boPreRMWarn = false;
                }

                /* RR安全带报警激活，V0.8版本需求，12.6.2.54.6节第四点，i)未系报警 */
                if ((false == boLastRRStatus) && \
                    (1u == FISD_u16SignalValue[nSigFISD_ACU_2nd_RSeatbeltBucklestatus]))
                {
                    boPreRRWarn = true;
                    if (Rte_LEDAppl_TargetGearIsDNM_Get() == true)
                    {
                        u16SeatbeltIndTimer = 0u;
                    }
                }
                else if (0u == FISD_u16SignalValue[nSigFISD_ACU_2nd_RSeatbeltBucklestatus])
                {
                    boPreRRWarn = false;
                }
            }
            else
            {
                boPreRLWarn = false;
                boPreRMWarn = false;
                boPreRRWarn = false;
                if (false == boReatOutCfgFlag)
                {
                    u16SeatbeltIndTimer = 0u;
                }
            }

            boRLWarn = boPreRLWarn;
            boRMWarn = boPreRMWarn;
            boRRWarn = boPreRRWarn;

            if ((Rte_LEDAppl_TargetGearIsDNM_Get() == false) && (boGearR == false))
            {
                boDrvWarn = false;
                boPasWarn = false;
                boRLWarn = false;
                boRMWarn = false;
                boRRWarn = false;
            }

            if ((0u == FISD_u16SignalValue[nSigFISD_ACU_2nd_LSeatbeltBucklestatus]) \
                && (0u == FISD_u16SignalValue[nSigFISD_ACU_2nd_MSeatbeltBucklestatus]) \
                && (0u == FISD_u16SignalValue[nSigFISD_ACU_2nd_RSeatbeltBucklestatus]))
            {
                u16SeatbeltIndTimer = 0u;
            }

            if ((boRLWarn == true) || (boRMWarn == true) || (boRRWarn == true))
            {
                if (Rte_LEDAppl_TargetGearIsDNM_Get() == true)
                {
                    u16SeatbeltIndTimer = 0u;
                }
            }

/* V0.8版本需求，12.6.2.54.6节第五点 */
/*************************************************************BUZZER******************************************************************/
            if ((false == boDrvWarn) \
                && (false == boPasWarn) \
                && (false == boRLWarn) \
                && (false == boRMWarn) \
                && (false == boRRWarn))
            {
                u32DriveDistance = Rte_LEDApp_u32GetOdoValue();                         /* 安全带都系上了，不累加行驶距离 */
            }

            if ((true == boGearR) \
                || (Rte_LEDAppl_VehicleMoveSts_Get() == false) \
                || (Rte_LEDAppl_TargetGearIsDNM_Get() == false))                          /* 倒车或非前进挡或车速为0，停止buzzer */
            {
                u32FDBuzTimer = 0u;
                u32FPBuzTimer = 0u;
                u32RLBuzTimer = 0u;
                u32RMBuzTimer = 0u;
                u32RRBuzTimer = 0u;
                u8FDBuzLevel = 0u;
                u8FPBuzLevel = 0u;
                u8RBuzLevel[0] = 0u;
                u8RBuzLevel[1] = 0u;
                u8RBuzLevel[2] = 0u;
                u32DriveDistance = Rte_LEDApp_u32GetOdoValue();
            }
            else
            {
                if (u16VehSpeed > 25u)
                {
                    /* 报警音优先级只提升，不下降。如果更高级报警音播放完成，则判断是否还要触发低级报警音 */
                    if (u8FDBuzLevel < 2u)
                    {
                        if (true == boDrvWarn)
                        {
                            u32FDBuzTimer = TIMECOUNT_120S;
                            u8FDBuzLevel = 2u;
                        }
                    }
                    else
                    {
                        if (u32FDBuzTimer == 0u)
                        {
                            u8FDBuzLevel = 2u;
                        }
                    }
                    if (u8FPBuzLevel < 2u)
                    {
                        if (true == boPasWarn)
                        {
                            u32FPBuzTimer = TIMECOUNT_120S;
                            u8FPBuzLevel = 2u;
                        }
                    }
                    else
                    {
                        if (u32FPBuzTimer == 0u)
                        {
                            u8FPBuzLevel = 2u;
                        }
                    }
                    if (u8RBuzLevel[0] < 2u)
                    {
                        if (true == boRLWarn)
                        {
                            u32RLBuzTimer = TIMECOUNT_32S;
                            u8RBuzLevel[0] = 2u;
                        }
                    }
                    else
                    {
                        if (u32RLBuzTimer == 0u)
                        {
                            u8RBuzLevel[0] = 2u;
                        }
                    }
                    if (u8RBuzLevel[1] < 2u)
                    {
                        if (true == boRMWarn)
                        {
                            u32RMBuzTimer = TIMECOUNT_32S;
                            u8RBuzLevel[1] = 2u;
                        }
                    }
                    else
                    {
                        if (u32RMBuzTimer == 0u)
                        {
                            u8RBuzLevel[1] = 2u;
                        }
                    }
                    if (u8RBuzLevel[2] < 2u)
                    {
                        if (true == boRRWarn)
                        {
                            u32RRBuzTimer = TIMECOUNT_32S;
                            u8RBuzLevel[2] = 2u;
                        }
                    }
                    else
                    {
                        if (u32RRBuzTimer == 0u)
                        {
                            u8RBuzLevel[2] = 2u;
                        }
                    }
                }
                else if ((u16VehSpeed > 10u) || \
                    (((Rte_LEDApp_u32GetOdoValue() - u32DriveDistance) > 300u) && (Rte_LEDAppl_VehicleMoveSts_Get() == true)))
                {
                    /* 报警音优先级只提升，不下降。如果更高级报警音播放完成，则判断是否还要触发低级报警音 */
                    if (u8FDBuzLevel < 1u)
                    {
                        if (true == boDrvWarn)
                        {
                            u32FDBuzTimer = TIMECOUNT_120S;
                            u8FDBuzLevel = ((u8BuzLevel <= 1u) ? 1u : u8BuzLevel);
                        }
                    }
                    else
                    {
                        ;
                    }
                    if (u8FPBuzLevel < 1u)
                    {
                        if (true == boPasWarn)
                        {
                            u32FPBuzTimer = TIMECOUNT_120S;
                            u8FPBuzLevel = u8BuzLevel <= 1u ? 1u : u8BuzLevel;
                        }
                    }
                    else
                    {
                        ;
                    }
                    if (u8RBuzLevel[0] < 1u)
                    {
                        if (true == boRLWarn)
                        {
                            u32RLBuzTimer = TIMECOUNT_32S;
                            u8RBuzLevel[0] = ((u8BuzLevel <= 1u) ? 1u : u8BuzLevel);
                        }
                    }
                    else
                    {
                        ;
                    }
                    if (u8RBuzLevel[1] < 1u)
                    {
                        if (true == boRMWarn)
                        {
                            u32RMBuzTimer = TIMECOUNT_32S;
                            u8RBuzLevel[1] = ((u8BuzLevel <= 1u) ? 1u : u8BuzLevel);
                        }
                    }
                    else
                    {
                        ;
                    }
                    if (u8RBuzLevel[2] < 1u)
                    {
                        if (true == boRRWarn)
                        {
                            u32RRBuzTimer = TIMECOUNT_32S;
                            u8RBuzLevel[2] = ((u8BuzLevel <= 1u) ? 1u : u8BuzLevel);
                        }
                    }
                    else
                    {
                        ;
                    }
                }
                else
                {
                    ;
                }
                if (((u16VehSpeed > 10u) \
                    || ((Rte_LEDApp_u32GetOdoValue() - u32DriveDistance) > 300u) && (Rte_LEDAppl_VehicleMoveSts_Get() == true)) \
                    || (u32FDBuzTimer != 0u) 
                    || (u32FPBuzTimer != 0u) 
                    || (u32RLBuzTimer != 0u) 
                    || (u32RMBuzTimer != 0u) 
                    || (u32RRBuzTimer != 0u))
                {
                    if ((true == boDrvWarn) && (false == boDrvWarnLast))                        /* 任一前排安全带报警由OFF变成ON，将前排计时刷新为120秒 */
                    {
                        u32FDBuzTimer = TIMECOUNT_120S;
                    }
                    else if (false == boDrvWarn)                                                   /* 前排安全带都系上，则清除前排计时时间。 */
                    {
                        u32FDBuzTimer = 0u;
                    }

                    if ((true == boPasWarn) && (false == boPasWarnLast))
                    {
                        u32FPBuzTimer = TIMECOUNT_120S;
                    }
                    else if (false == boPasWarn)
                    {
                        u32FPBuzTimer = 0u;
                    }
                }
                if ((u16VehSpeed > 10u) \
                    || (((Rte_LEDApp_u32GetOdoValue() - u32DriveDistance) > 300u) \
                        && ((Rte_LEDAppl_VehicleMoveSts_Get() == true) \
                            || ((0u == FISD_u16SignalValue[nSigFISD_BCM_RearLeftDoorAjarStatus]) \
                                && (0u == FISD_u16SignalValue[nSigFISD_BCM_RearRightDoorAjarStatus])))) \
                    || (u32FDBuzTimer != 0u) 
                    || (u32FPBuzTimer != 0u) 
                    || (u32RLBuzTimer != 0u) 
                    || (u32RMBuzTimer != 0u) 
                    || (u32RRBuzTimer != 0u))
                {
                    if ((true == boRLWarn) && (false == boRLWarnLast))                          /* 后左安全带报警由OFF变成ON，将后左计时刷新为32秒 */
                    {
                        u32RLBuzTimer = TIMECOUNT_32S;
                    }
                    else if (false == boRLWarn)                                                 /* 后左安全带都系上，则清除后左计时时间 */
                    {
                        u32RLBuzTimer = 0u;
                    }

                    if ((true == boRMWarn) && (false == boRMWarnLast))
                    {
                        u32RMBuzTimer = TIMECOUNT_32S;
                    }
                    else if (false == boRMWarn)
                    {
                        u32RMBuzTimer = 0u;
                    }

                    if ((true == boRRWarn) && (false == boRRWarnLast))
                    {
                        u32RRBuzTimer = TIMECOUNT_32S;
                    }
                    else if (false == boRRWarn)
                    {
                        u32RRBuzTimer = 0u;
                    }
                }
                if (false == boDrvWarn)
                {
                    u8FDBuzLevel = 0u;
                }
                if (false == boPasWarn)
                {
                    u8FPBuzLevel = 0u;
                }
                if (false == boRLWarn)
                {
                    u8RBuzLevel[0] = 0u;
                }
                if (false == boRMWarn)
                {
                    u8RBuzLevel[1] = 0u;
                }
                if (false == boRRWarn)
                {
                    u8RBuzLevel[2] = 0u;
                }

                if (u32FDBuzTimer > 0u)                                                         /* 计时 */
                {
                    if (u8FDBuzLevel > 0u)                                                      /* 报警时间只有在报警等级达到要求时才开始计时，否则报警时间刷新后，等待等级被速度信号触发后才计时 */
                    {
                        if (u32FDBuzTimer > 0u)
                        {
                            u32FDBuzTimer--;
                        }
                    }
                }
                if (u32FPBuzTimer > 0u)
                {
                    if (u8FPBuzLevel > 0u)
                    {
                        if (u32FPBuzTimer > 0u)
                        {
                            u32FPBuzTimer--;
                        }
                    }
                }
                if (u32RLBuzTimer > 0u)
                {
                    if (u8RBuzLevel[0] > 0u)
                    {
                        if (u32RLBuzTimer > 0u)
                        {
                            u32RLBuzTimer--;
                        }
                    }
                }
                if (u32RMBuzTimer > 0u)
                {
                    if (u8RBuzLevel[1] > 0u)
                    {
                        if (u32RMBuzTimer > 0u)
                        {
                            u32RMBuzTimer--;
                        }
                    }
                }
                if (u32RRBuzTimer > 0u)
                {
                    if (u8RBuzLevel[2] > 0u)
                    {
                        if (u32RRBuzTimer > 0u)
                        {
                            u32RRBuzTimer--;
                        }
                    }
                }
            }
            
            if (((u8FDBuzLevel == 2u) && (u32FDBuzTimer > 0u)) \
                || ((u8FPBuzLevel == 2u) && (u32FPBuzTimer > 0u)) \
                || ((u8RBuzLevel[0] == 2u) && (u32RLBuzTimer > 0u)) \
                || ((u8RBuzLevel[1] == 2u) && (u32RMBuzTimer > 0u))\
                || ((u8RBuzLevel[2] == 2u) && (u32RRBuzTimer > 0u)))
            {
                u8BuzLevel = 2u;
            }
            else if (((u8FDBuzLevel == 1u) && (u32FDBuzTimer > 0u)) \
                || ((u8FPBuzLevel == 1u) && (u32FPBuzTimer > 0u)) \
                || ((u8RBuzLevel[0] == 1u) && (u32RLBuzTimer > 0u)) \
                || ((u8RBuzLevel[1] == 1u) && (u32RMBuzTimer > 0u)) \
                || ((u8RBuzLevel[2] == 1u) && (u32RRBuzTimer > 0u)))
            {
                u8BuzLevel = 1u;
            }
            else
            {
                u8BuzLevel = 0u;
            }
            
            if ((u32FDBuzTimer > 0u) 
                || (u32FPBuzTimer > 0u)
                || (u32RLBuzTimer > 0u)
                || (u32RMBuzTimer > 0u)
                || (u32RRBuzTimer > 0u))                                                        /* 报警音计时未完成，则激活对应等级的报警音，否则关闭所有报警音 */
            {
                if (3u == u8BuzLevel)
                {
                    BUZ_SET(nBUZ_UnSeatBeltHigh, nBuz_Active);
                    BUZ_SET(nBUZ_UnSeatBeltMid, nBuz_Inactive);
                    BUZ_SET(nBUZ_UnSeatBeltLow, nBuz_Inactive);
                }
                else if (2u == u8BuzLevel)
                {
                    BUZ_SET(nBUZ_UnSeatBeltHigh, nBuz_Inactive);
                    BUZ_SET(nBUZ_UnSeatBeltMid, nBuz_Active);
                    BUZ_SET(nBUZ_UnSeatBeltLow, nBuz_Inactive);
                }
                else if (1u == u8BuzLevel)
                {
                    BUZ_SET(nBUZ_UnSeatBeltHigh, nBuz_Inactive);
                    BUZ_SET(nBUZ_UnSeatBeltMid, nBuz_Inactive);
                    BUZ_SET(nBUZ_UnSeatBeltLow, nBuz_Active);
                }
                else
                {
                    BUZ_SET(nBUZ_UnSeatBeltHigh, nBuz_Inactive);
                    BUZ_SET(nBUZ_UnSeatBeltMid, nBuz_Inactive);
                    BUZ_SET(nBUZ_UnSeatBeltLow, nBuz_Inactive);
                }
            }
            else
            {
                BUZ_SET(nBUZ_UnSeatBeltHigh, nBuz_Inactive);
                BUZ_SET(nBUZ_UnSeatBeltMid, nBuz_Inactive);
                BUZ_SET(nBUZ_UnSeatBeltLow, nBuz_Inactive);
            }
/* V0.8版本需求，12.6.2.54.6节第三点 */
/***************************************************************LED*******************************************************************/

            enLastLedSt = (tenIndicatorStatus)LED_CHK(nIND_DrvSeatbelt);
            LED_SET(nIND_DrvSeatbelt, nInd_Off);                                        /* 先默认关闭报警灯，后面根据逻辑判断决定是否要点亮，或者闪烁报警灯。 */
            if (false == LEDApp_boSeatBeltSelfCheckEnableFlag)
            {
                if (((true == boDrvWarn) || \
                    (true == boPasWarn) || \
                    (true == boRLWarn) || \
                    (true == boRMWarn) || \
                    (true == boRRWarn)))
                {
                    if ((false == boRLWarn) && (false == boRMWarn) && (false == boRRWarn))
                    {
                        if (u16SeatbeltIndTimer > 0u)                                           /* 计时60秒，60秒内要点亮报警灯 */
                        {
                            u16SeatbeltIndTimer--;
                        }
                    }

                    /* 安全带指示灯，第1条需求，关于倒车的说明。 */
                    /* 报警音结束后，继续闪烁 */
                    if ((boGearR == true)||(Rte_LEDAppl_VehicleMoveSts_Get() == false))
                    {
                        if (((enLastLedSt == nInd_Flash_1Hz) && (boCurFlash1HzSt == true))
                            || ((enLastLedSt == nInd_Flash_05Hz) && (boCurFlash05HzSt == true))
                            || (enLastLedSt == nInd_Off)
                            || (enLastLedSt == nInd_On))
                        {
                            LED_SET(nIND_DrvSeatbelt, nInd_On);
                        }
                        else
                        {
                            LED_SET(nIND_DrvSeatbelt, enLastLedSt);
                        }
                    }
                    else
                    {
                        if (Rte_LEDAppl_TargetGearIsDNM_Get() == true)
                        {
                            if (u8BuzLevel >= 2u)                                              /* 安全带指示灯，第4条需求 */
                            {
                                LED_SET(nIND_DrvSeatbelt, nInd_Flash_1Hz);
                            }
                            else if (u8BuzLevel == 1u)                                          /* 安全带指示灯，第3条需求 */
                            {
                                LED_SET(nIND_DrvSeatbelt, nInd_Flash_05Hz);
                            }
                            else
                            {
                                if (enLastLedSt == nInd_Off)
                                {
                                    LED_SET(nIND_DrvSeatbelt, nInd_On);                             /* 速度小于10km且行驶距离小于300m */
                                }
                                else
                                {
                                    LED_SET(nIND_DrvSeatbelt, enLastLedSt);
                                }
                            }
                        }
                        else
                        {
                            ;
                        }
                    }
                }
                /* 安全带指示灯，第1条需求 */
                else if (/* (true == Rte_LEDAppl_EngineRunningStateX_Get()) && */ \
                    (true == boReatOutCfgFlag) && \
					(u16SeatbeltIndTimer > 0u) && \
                    ((false == boDrvWarn) && (false == boPasWarn) && (false == boRLWarn) && (false == boRMWarn) && (false == boRRWarn)) && \
                    ((1u == FISD_u16SignalValue[nSigFISD_ACU_2nd_LSeatbeltBucklestatus]) || (1u == FISD_u16SignalValue[nSigFISD_ACU_2nd_MSeatbeltBucklestatus]) || (1u == FISD_u16SignalValue[nSigFISD_ACU_2nd_RSeatbeltBucklestatus])))
                {
                     /* 计时60秒，60秒内要点亮报警灯 */
                        u16SeatbeltIndTimer--;
                        LED_SET(nIND_DrvSeatbelt, nInd_On);
                }
                else if ((1u == FISD_u16SignalValue[nSigFISD_ACU_DrvSeatbeltBuckleInvalid]) || (1u == FISD_u16SignalValue[nSigFISD_ACU_PassSeatbeltBuckleInvalid]))
                {
                	LED_SET(nIND_DrvSeatbelt, nInd_On);      //安全带故障
                }
            }
            else
            {
                LED_SET(nIND_DrvSeatbelt, nInd_On);
            }
        }
        else
        {
            u32DriveDistance = Rte_LEDApp_u32GetOdoValue();
            u16SeatbeltIndTimer = TIMECOUNT_62S;
            boDrvWarn = false;
            boPasWarn = false;
            boRLWarn = false;
            boRMWarn = false;
            boRRWarn = false;
            boPreRLWarn = false;
            boPreRMWarn = false;
            boPreRRWarn = false;
            u32FDBuzTimer = 0u;
            u32FPBuzTimer = 0u;
            u32RLBuzTimer = 0u;
            u32RMBuzTimer = 0u;
            u32RRBuzTimer = 0u;
            u8BuzLevel = 0u;
            u8FDBuzLevel = 0u;
            u8FPBuzLevel = 0u;
            u8RBuzLevel[0] = 0u;
            u8RBuzLevel[1] = 0u;
            u8RBuzLevel[2] = 0u;
            BUZ_SET(nBUZ_UnSeatBeltHigh, nBuz_Inactive);
            BUZ_SET(nBUZ_UnSeatBeltMid, nBuz_Inactive);
            BUZ_SET(nBUZ_UnSeatBeltLow, nBuz_Inactive);
            LED_SET(nIND_DrvSeatbelt, nInd_On);
        }
    }
    else
    {
        u32DriveDistance = Rte_LEDApp_u32GetOdoValue();                                 /* IGN OFF实时记录里程，用于IGN ON后判断IGN ON状态下，行驶的距离是否超过300m */
        u16SeatbeltIndTimer = TIMECOUNT_62S;
        boDrvWarn = false;
        boPasWarn = false;
        boRLWarn = false;
        boRMWarn = false;
        boRRWarn = false;
        boPreRLWarn = false;
        boPreRMWarn = false;
        boPreRRWarn = false;
        u32FDBuzTimer = 0u;
        u32FPBuzTimer = 0u;
        u32RLBuzTimer = 0u;
        u32RMBuzTimer = 0u;
        u32RRBuzTimer = 0u;
        u8BuzLevel = 0u;
        u8FDBuzLevel = 0u;
        u8FPBuzLevel = 0u;
        u8RBuzLevel[0] = 0u;
        u8RBuzLevel[1] = 0u;
        u8RBuzLevel[2] = 0u;
        LED_SET(nIND_DrvSeatbelt, nInd_Off);
        BUZ_SET(nBUZ_UnSeatBeltHigh, nBuz_Inactive);
        BUZ_SET(nBUZ_UnSeatBeltMid, nBuz_Inactive);
        BUZ_SET(nBUZ_UnSeatBeltLow, nBuz_Inactive);
    }

    /* 退出该检测函数前，下面的代码将当前状态记录到上一次状态的静态缓存变量中，用于下一次polling判断 */
    if (1u == FISD_u16SignalValue[nSigFISD_ACU_2nd_LSeatbeltBucklestatus])
    {
        boLastRLStatus = true;
    }
    else
    {
        boLastRLStatus = false;
    }
    if (1u == FISD_u16SignalValue[nSigFISD_ACU_2nd_MSeatbeltBucklestatus])
    {
        boLastRMStatus = true;
    }
    else
    {
        boLastRMStatus = false;
    }
    if (1u == FISD_u16SignalValue[nSigFISD_ACU_2nd_RSeatbeltBucklestatus])
    {
        boLastRRStatus = true;
    }
    else
    {
        boLastRRStatus = false;
    }
    boDrvWarnLast = boDrvWarn;
    boPasWarnLast = boPasWarn;
    boRLWarnLast = boRLWarn;
    boRMWarnLast = boRMWarn;
    boRRWarnLast = boRRWarn;
}
