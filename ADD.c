/*******************************************************************************************/
/*===================== COPYRIGHT (C) HUIZHOU DESAY SV AUOTMOTIVE 2016 ====================*/
/*******************************************************************************************
* C Source     : FISD.c
*
* Description  : Fault and Indicator State Detect.
*
* Environment  : autosar 4.x SC1, Renesas RH850, Greenhills Multi 6.1.4
*
* Created_by   :
*
* Revision history:
*
*    Rev 1.0.0
*    create.
*
********************************************************************************************/

/*******************************************************************************************
* FILE DECLARATION
********************************************************************************************/
#ifndef __FISD_C__
#define __FISD_C__ /* Name of the Module */

/*******************************************************************************************
* HEADER-FILES (Only those that are needed in this file)
********************************************************************************************/

/* System-headerfiles */
/* Own headerfiles */
#include "SYNC.h"
#include "FISD.h"
#include "TextMgr.h"
#include "FISD_Cfg.h"
#include "DIDGeneral.h"
#include "ODO_MAINTAIN.h"
#include <math.h>
#include "mc_poll.h"
#include "CfgMgr.h"
#include "sf_seatbelt_chart.h"
#include "CAN_APP_DIAG_MISC_IF.H"
#include "ENG_APP_IF.h"
#include "POWER_DRV.H"
#include "RTE_CAN_SIG.H"
#if defined(MCU_LOG_OPTION)
#include "Log_app_if.h"
#endif

/*******************************************************************************************
* LOCAL DEFINITION (Only those that are needed in this file)
********************************************************************************************/
/* Local macro Define */

/*eeprom config*/
#define FISD_u16SeatbeltChimeOffSpeed                         250u /*Default 250,unit 0.1km/h*/
#define FISD_u16SeatbeltChimeOnSpeed                          250u /*Default 250,unit 0.1km/h*/
#define FISD_u16HandbrakeNotReleaseOnSpeed                    50u /*Default 50,unit 0.1km/h*/

#define FISD_u8EngineCoolantOnTemp                            118  /*Default 120,unit 1℃*/
#define FISD_u8EngineCoolantOffTemp                           112 /*Default 116,unit 1℃*/

#define FISD_u16SelfCheckEngineSpeedThreshold           300u /*Unit 1rpm*/
#define FISD_u16OverspeedHysteresisValue                   5u/*Unit 1km/h*/
#define FISD_u16EngineRunValue                                   300u /*Unit 1rpm*/

/*local macro*/
#define TIMECOUNT_30ms                                          3u
#define TIMECOUNT_100ms                                          10u
#define TIMECOUNT_200ms                                          20u
#define TIMECOUNT_300ms                                          30u
#define TIMECOUNT_400ms                                          40u
#define TIMECOUNT_500ms                                           50u
#define TIMECOUNT_1s                                                 100u
#define TIMECOUNT_1310ms                                             131u
#define TIMECOUNT_2s                                                 200u
#define TIMECOUNT_3s                                                300u
#define TIMECOUNT_5s                                               500u
#define TIMECOUNT_6s                                               600u
#define TIMECOUNT_9s                                               900u
#define TIMECOUNT_10s                                               1000u
#define TIMECOUNT_11s                                               1100u
#define TIMECOUNT_12s                                               1200u
#define TIMECOUNT_15s                                               1500u
#define TIMECOUNT_20s                                               2000u
#define TIMECOUNT_25s                                               2500u
#define TIMECOUNT_30s                                               3000u
#define TIMECOUNT_60s                                                6000u
#define TIMECOUNT_75s                                               7500u
#define TIMECOUNT_600s                                             60000u

#define BIT_READ(para,bit)                                          (para & (1<<(bit)))


/* Local Type Define */

/*Signal enum Value*/
enum
{
    nPinLow = 0,
    nPinHigh
};              /*pin level*/

enum
{
    nInvalid = 0,
    nValid
};              /*singal valid*/

enum
{
    nbit0 = 0,
    nbit1,
    nbit2,
    nbit3,
    nbit4,
    nbit5,
    nbit6,
    nbit7
};              /*the bit of signal*/


/* Local ROM-constants */

/* Local Variables Define*/
static U16 FISD_u16SignalValue[nSigMaxNum];
static BOOL  FISD_enIgnState = FALSE;
static BOOL   FISD_boIgnOnToOffFlag = 0xff;
static U16 FISD_u16VehicleSpeed; /*Unit 0.1km/h*/
static U16 FISD_u16IgnOnTimerS;  /*Ignition on time count*/
static U16 FISD_u16IgnOffTimerS; /*Ignition off time count*/
static BOOL   FISD_boSelfCheckFlag = FALSE;
static U32 FISD_u32FollowMeElapseTime = 0u; /*Unit:10ms*/
static BOOL   FISD_boFollowMeActiveFlag = FALSE;
static U32 FISD_u32FollowMeRestTime = 0u; /*Unit:10ms*/
static U16 FISD_u16FollowMeLastValue = 0u;
static U32 FISD_u32TurnOffLcdTime = 0u; /*Unit:10ms*/
static U16 FISD_u16LastEngineSpeed = 0u;
static U16 FISD_u16LastAVHSts = 0u;
static U16 FISD_u16LastHDCCtrlSts = 0u;
static U32 FISD_u32OilPressureLowTime = 0u;
static U32 FISD_u32EcoSwitchTime = 0u;
static BOOL   FISD_boEngineShutdown = FALSE;
static U8  FISD_u8RainLastRawValue = 0u;
static U8  FISD_u8RainLastValue = 0u;
static U8  FISD_u8RainNextValue = 0u;
static U16 FISD_u16LimitCruiseSpeedValue = 0u;
static BOOL   FISD_boLimitCruiseSpeedValid = FALSE;
static U8  FISD_u8PlgPercent = 0u;
static U16 FISD_u16PlgHmiShowAngle_Pre = 0u;
static BOOL   FISD_boPlgOpenNotChange2sFlag = FALSE;
static U16 FISD_u16PlgOpenSignal_Pre = 0u;
static BOOL   FISD_boWarmingHappenFlag = FALSE;
static BOOL   FISD_boMenuDeniedFlag = FALSE;
static U16 FISD_u16MenuDeniedCount = 0u;
static U16 FISD_u16DebounceCount_Crank_D1_D2 = 0u;
static U16 FISD_u16DebounceCount_Crank_D1 = 0u;
static U16 FISD_u16DebounceCount_D1 = 0u;
BOOL WarningMessageIDneedInitFlag = FALSE;
BOOL WarningRmninfoneedInitFlag = FALSE;
static BOOL boCurTimeGapSet_ICMpreIsZero;
static BOOL boACCSpeedSts = FALSE;
static BOOL   FISD_boDebounceFlag_Crank_D1 = TRUE;
static U8 FISD_enLastPowerMode = OomPWRMAXMODE;
static tenKeySts FISD_enLastKeySts = nOFF;
static BOOL   FISD_boFuelLevelLowFlag = FALSE;
static U32 FISD_u32FuelLevelLowOdo = 0u;

static tenIndicatorStatus FISD_aenIndicatorState[nIND_MaxNum];
static tenTextStatus FISD_aenTextActive[nDIC_MaxNum];
static tenBuzzerStatus FISD_aenSoundActive[nBUZ_MaxNum];
static BOOL FISD_aboTextShowTimeResetFlag[nDIC_MaxNum];
static BOOL FISD_aboSoundOnTimeResetFlag[nBUZ_MaxNum];
static BOOL sboSeatRearMirrorInit = FALSE;
static U8 u8SeatExtMirror_Pre = 0;
static U8 u8Partnumber = PART_NUM_MAX;
U8 FISD_u8LeftAreaValue = 0u;
U8 FISD_u8RightAreaValue = 0u;

U8 FISD_u8ObjectDistance = 0u;
BOOL need_TTS_broadcast = FALSE;
static BOOL FISD_boIsCrankOn = False;

U8 g_doorMap[6][2] = 
	{
		{nHoodSts,      nIND_HoodOpen},
		{nTrunkSts,     nIND_TrunkOpen},
		{nDriverDoorSts,nIND_DriverDoorOpen},
		{nPsngrDoorSts, nIND_PsngrDoorOpen},
		{nLHRdoorSts,   nIND_RLDoorOpen},
		{nRHRDoorSts,   nIND_RRDoorOpen}
	};

static BOOL FISD_boCalendarAndCarSts = FALSE;
static U16 FISD_u16CalendarAndCarStsDelay = 0u;

static U8 FISD_BackLightStsRecord = 0u;
static U8 FISD_BackLightStsRecord_BACK=0u;

tenAdasWarnCurFuncTypedef FISD_enCurAdasWarn = enAdasWarn_Off;
__noinit__ U8 FISD_enWorkMode = enMode_MaxNum;
__noinit__ U8 FISD_driveMode_SetByIHU = enMode_MaxNum;

// used for animation dubbling at wakeup
static BOOL need_detect_driver_door_open = FALSE;

/*INDENT-OFF*/
#if USE_FUNCCONFIG==1
static BOOL FISD_boIndFuncValid[nIND_MaxNum];
static BOOL FISD_boDicFuncValid[nDIC_MaxNum];
static BOOL FISD_boBuzFuncValid[nBUZ_MaxNum];

#define IND_SET(IND_ID, IND_STATE)      if(1u == FISD_boIndFuncValid[(IND_ID)])           \
                                            FISD_aenIndicatorState[(IND_ID)]=(IND_STATE); \
                                        else                                              \
                                            FISD_aenIndicatorState[(IND_ID)]=(nInd_Off);
#define DIC_SET(DIC_ID, DIC_STATE)      if(1u == FISD_boDicFuncValid[(DIC_ID)])           \
                                            FISD_aenTextActive[(DIC_ID)]=(DIC_STATE);     \
                                        else                                              \
                                            FISD_aenTextActive[(DIC_ID)]=(nDic_Inactive);
#define BUZ_SET(BUZ_ID, BUZ_STATE)      if(1u == FISD_boBuzFuncValid[(BUZ_ID)])           \
                                            FISD_aenSoundActive[(BUZ_ID)]=(BUZ_STATE);    \
                                        else                                              \
                                            FISD_aenSoundActive[(BUZ_ID)]=(nBuz_Inactive);

#define IND_CHK(IND_ID, IND_STATE)      ((FISD_aenIndicatorState[(IND_ID)]==(IND_STATE))? TRUE: FALSE)
#define DIC_CHK(DIC_ID, DIC_STATE)      ((FISD_aenTextActive[(DIC_ID)]==(DIC_STATE))? TRUE: FALSE)
#define BUZ_CHK(BUZ_ID, BUZ_STATE)      ((FISD_aenSoundActive[(BUZ_ID)]==(BUZ_STATE))? TRUE: FALSE)
#else

#define IND_SET(IND_ID, IND_STATE)      FISD_aenIndicatorState[(IND_ID)]=(IND_STATE)
#define DIC_SET(DIC_ID, DIC_STATE)      FISD_aenTextActive[(DIC_ID)]=(DIC_STATE)
#define BUZ_SET(BUZ_ID, BUZ_STATE)      FISD_aenSoundActive[(BUZ_ID)]=(BUZ_STATE)

#define IND_CHK(IND_ID, IND_STATE)      ((FISD_aenIndicatorState[(IND_ID)]==(IND_STATE))? TRUE: FALSE)
#endif
/*INDENT-ON*/

/*******************************************************************************************
* FILE LOCAL FUNCTION PROTOTYPES
*
* In this section declare
* - all file local function prototypes needed for your module.
********************************************************************************************/

/* Local Function Prototypes*/
static void FISD_vGetSignalValue(void);
static void FISD_vFaultIndicatorDetect(void);
static void FISD_vIgnOnTimeCount(void);
static void FISD_vInitLocalVar(void);
static void FISD_vCheckEngineShutdown(void);
static void FISD_vFuncConfigInit(void);
static void FISD_vDebounceThreeSecond(void);
static void FISD_vCrankOnDetect(void);
static void FISD_vCalendarAndCarSts(void);

/*Location: Column C of FISD_ProcessFunc sheet in T18_Configuration_Tool(FISD_SoundMgr_SYN_TextMgr).xlsx*/
static void FISD_vLittleLampDetect(void);
static void FISD_vHighBeamDetect(void);
static void FISD_vSeatbeltDetect(void);
static void FISD_vRearSeatbeltDetect(void);
static void FISD_vThirdRowSeatbeltDetect(void);
static void FISD_vEpcIndicatorDetect(void);
static void FISD_vMilLampDetect(void);
static void FISD_vEpbAndAutoHoldDetect(void);
static void FISD_vCharger_IemDetect(void);
static void FISD_vBrakeFluidLevelLow_EbdFaultDetect(void);
static void FISD_vAbsFaultDetect(void);
static void FISD_vEspFaultShieldDetect(void);
static void FISD_vCruiseLimitControl(void);
static void FISD_vAirbagDetect(void);
static void FISD_vTransmissionFaultDetect(void);
static void FISD_vEpsFaultDetect(void);
static void FISD_vEngineOilPressureDetect(void);
static void FISD_vTurnStateDetect(void);
static void FISD_vRearFogDetect(void);
static void FISD_vCoolantStatusDetect(void);
static void FISD_vFuelLevelLowDetect(void);
static void FISD_vStartStopSysDetect(void);
static void FISD_vDriveWorkModeDetect(void);
static void FISD_vLdwLkaDetect(void);
static void FISD_vOverspeedDetect(void);
static void FISD_vDoorOpenDetect(void);
static void FISD_vRadarDetect(void);
static void FISD_vPepsWarningDetect(void);
static void FISD_vFatigueDetect(void);
static void FISD_vAvmWarningDetect(void);
static void FISD_vFollowMeDetect(void);
static void FISD_vBsdAndRctaDetect(void);
static void FISD_vTyrePressureDetect(void);
static void FISD_vKeyInCarDetect(void);
static void FISD_vMaintainDetect(void);
static void FISD_vDrlDetect(void);
static void FISD_vFrontFogDetect(void);
static void FISD_vHdcDetect(void);
static void FISD_vRainDetect(void);
static void FISD_vHhcDetect(void);
static void FISD_vAccAebFcwDetect(void);
static void FISD_vHmaDetect(void);
static void FISD_vSlaDetect(void);
static void FISD_vIsaDetect(void);
static void FISD_vIsaIsaDetect(void);
static void FISD_vSLASpdLimChgAud(void);
static void FISD_vTJA_ICADetect(void);
static void FISD_vESCLDetect(void);
static void FISD_vElecShiftFaultDetect(void);
static void FISD_vAPADetect(void);
static void FISD_vWarnIconDetect(void);
static void FISD_vLanguageSetDetect(void);
static void FISD_vLdwLkaSwitchDetect(void);
static void FISD_vSeatRearmirrorDedect(void);
static void FISD_vChildLockDedect(void);
static void FISD_vRemoteStartModeDetect(void);
static void FISD_vCarLampFaultDetect(void);
static void FISD_vMemoryKeyStsDetect(void);
static void FISD_vGpfFuncDetect(void);
static void FISD_vBattEnergyWarningDetect(void);
static void FISD_vTakeYourPhoneDetect(void);
static void FISD_vCameraShotDetect(void);
static void FISD_vAWD_FaultDetect(void);
static void FISD_vIceRoadDetect(void);
static void FISD_vWashingWaterDetect(void);
static void FISD_vActiveSafetyBeltDetect(void);
static void FISD_vHUDOverHotDetect(void);
static void FISD_vClothesHookedDetect(void);
static void FISD_vElectricDoorOpenDetect(void);
static void FISD_vCloseLightDetect();
static void FISD_vPHEV_SOCDisp_Detect(void);
static void FISD_vPHEV_LowSOCDetect(void);
static void FISD_vPHEVSystemDetect(void);
static void FISD_vPHEV_PowerModeDetect(void);
static void FISD_vPHEV_eletricityModeDetect(void);
static void	FISD_vPHEV_RGCdetect(void);
static void	FISD_vPHEV_RegenerateLevelDetect(void);
static void	FISD_vPowerLimitDetect(void);
static void	FISD_vPHEV_AVAS_Detect(void);
static void	FISD_vPHEV_BMSH_InsulationStsDetect(void);
static void	FISD_vPHEV_TurnOffIgnDetect(void);
static void FISD_vPHEV_BATPower_Red_Detect(void);
static void FISD_vPHEV_DisChargeFaultDetect(void);
static void	FISD_vPHEV_ChargingStsDetect(void);
static void FISD_vPHEV_WarmerRemindDetect(void);
static void	FISD_vPHEV_HvSysFltStopReq_Detect(void);
static void	FISD_vPHEV_BatteryTempLightSts_Detect(void);
static void FISD_vPHEV3_LowBatteryInfo_Detect(void);
static void FISD_vPHEV3_HighTempLimit_Detect(void);
static void FISD_vPHEV3_ChrgFillerAjar_Detect(void);
static void FISD_vPHEV3_EDrvOnly_Detect(void);
static void FISD_vPHEV3_ForcedPrkgChrg_Detect(void);
static void FISD_vPHEV3_EngSelfMaiTin_Detect(void);
static void FISD_vPHEV3_VehStrtWarn_Detect(void);
static void FISD_vEV_VehicleSystemFailure_Detect(void);
static void FISD_vEV_InvldCdnToDrvr_Detect(void);
static void FISD_vEV_LowSOCCLMLimitSts_Detect(void);
static void FISD_vEV_TowMode_Detect(void);
static void FISD_vEV_V2LFuncSts_Detect(void);
static void FISD_vEV_VCU_HVReady_Detect(void);
static void FISD_vEV_TMF_MILSts_Detect(void);
static void FISD_vEV_BMSH_PreWarmDis_Detect(void);
static void FISD_vEV_TrailerConnectSts_Detect(void);
static void FISD_vSmartADASDetect(void);
static void FISD_vMaintainWarningDetect(void);
static void FISD_vPilotDetect(void);
static void FISD_vBatLow_Detect(void);
static void FISD_vSlaveVehModDetect(void);
static void FISD_vASUFaultDetect(void);
static void FISD_vOUTDMS_TipsDetect(void);
static void FISD_vINTDMS_TipsDetect(void);
static void FISD_vLdwLdpEklDetect(void);
static void FISD_vTakeOverDetect(void);
static void FISD_vRctbRaebDetect(void);
static void FISD_vCornerRadarDetect(void);
static void FISD_vBsdSystemDetect_Ver4(void);
static void FISD_vNotParkingDetect(void);
static void FISD_vDoorSeatbeltSync(void);
static void FISD_vDoorStsDetect(void);
static void FISD_vDMS_TipsDtetct(void);
static void FISD_vIPB_WarnDtetct(void);
static void FISD_vRearChildDetect(void);
static void FISD_vChildProtectionDetect(void);
static void FISD_vCrankOnAndSelfCheckDetect(void);
static void	FISD_vVINComparisonDetect(void);



/*******************************************************************************************
* FILE GLOBAL FUNCTION DEFINITIONS
*
* In this section definition
* - all file Global function definitions needed for your module.
* - all tasks
********************************************************************************************/

/* Global Functions */

/*******************************************************************************************
* Function: FISD_vFCInit
* Description:  Reset function of FISD
* Parameters: none
* Return: none
********************************************************************************************/
void FISD_vFCInit(void)
{
    U8 language = 0;
    /*Get function configuration*/

    // 7DCT, three driving mode, default normal
    if(TRUE == Rte_FISD_u8GetCONFIG_ThreeDriverModeCount())
    {
        FISD_enWorkMode = enMode_NORMAL;
    }
    // 6DCT, CVT19, CVT25, two driving mode, default ECO
    else
    {
        FISD_enWorkMode = enMode_ECO;
    }
}

/*******************************************************************************************
* Function: FISD_vInit
* Description:  Initialize function of FISD
* Parameters: none
* Return: none
********************************************************************************************/
void FISD_vInit(void)
{
	u8Partnumber = eng_get_partnumtype();
#if USE_FUNCCONFIG==1
    FISD_vFuncConfigInit();
#endif
    FISD_vInitLocalVar();

    need_detect_driver_door_open = TRUE;
	sboSeatRearMirrorInit = FALSE;
	u8SeatExtMirror_Pre = 0;
}

/*******************************************************************************************
* Function: FISD_vDeinit
* Description:  Deinit function of FISD
* Parameters: none
* Return: none
********************************************************************************************/
void FISD_vDeinit(void)
{
    FISD_vInitLocalVar();
    need_detect_driver_door_open = FALSE;
}

/*******************************************************************************************
* Function: FISD_vMain
* Description: Main container function of FISD
* Parameters: none
* Return: none
********************************************************************************************/
void FISD_vMain(void)
{
    BOOL sys_fullon = boSystemOOMFullOnMode();
    BOOL sys_parton = boSystemOOMPartOnMode();

    U8 u8avmType = can_diag_get_sw_conf(CONF_IDX_AVMIntergatedMethod);
    U8 u8avm = can_diag_get_sw_conf(CONF_IDX_AVM);

    if((TRUE == FISD_enIgnState) && (FALSE == sys_fullon))/*On to Off*/
    {
        FISD_boIgnOnToOffFlag = TRUE;/*Ignition on to ignition off flag*/
    }
    else if((FALSE == FISD_enIgnState) && (TRUE == sys_fullon))/*Off to On*/
    {
        FISD_boIgnOnToOffFlag = FALSE;/*Ignition on to ignition off flag*/
        WarningMessageIDneedInitFlag = TRUE;
		WarningRmninfoneedInitFlag = TRUE;
        // since we come to fullon, we does never need detect door open, until sys come to sleep state
        need_detect_driver_door_open = FALSE;
        //FISD_u16LastHDCCtrlSts = 0u;
    }
    else
    {
        FISD_boIgnOnToOffFlag = 0xff;/*Ignition on to ignition off flag*/
    }

    FISD_enIgnState = sys_fullon;
    FISD_u16VehicleSpeed = Rte_FISD_u16GetSpeedGaugeDispValue();

    if(sys_fullon||sys_parton)
    {
        FISD_vCheckEngineShutdown();
        FISD_vGetSignalValue();
        FISD_vIgnOnTimeCount();
        FISD_vDebounceThreeSecond();
        FISD_vCrankOnDetect();
        FISD_vFaultIndicatorDetect();
        FISD_vCalendarAndCarSts();
    }
    else
    {
        /*do nothing*/
    }
    
#if defined(AVM_BUILT_IN_OPTION)
	if((u8avmType > 0) && (u8avm == 1))
	{
		can_mbx_send_avm_on_off_status(1U);
	}
 #endif
}

/*******************************************************************************************
* Function: FISD_enGetIndicatorState
* Description: Get indicator state by indicator ID
* Parameters: @enIndId: indicator ID
* Return: indicator state
********************************************************************************************/
tenIndicatorStatus FISD_enGetIndicatorState(tenIndicatorId enIndId)
{
    tenIndicatorStatus result = nInd_Off;

    if(enIndId < nIND_MaxNum)
    {
        result = FISD_aenIndicatorState[enIndId];
    }

    return result;
}

/*******************************************************************************************
* Function: FISD_enGetTextState
* Description: Get warning text state by warning text ID
* Parameters: @enTextId: WarningText ID
* Return: WarningText state
********************************************************************************************/
tenTextStatus FISD_enGetTextState(tenTextId enTextId)
{
    tenTextStatus result = nDic_Inactive;

    if(enTextId < nDIC_MaxNum)
    {
        result = FISD_aenTextActive[enTextId];
    }

    return result;
}


void FISD_enSetTextState(tenTextId enTextId ,tenTextStatus enTextStatus)
{
    if(enTextId < nDIC_MaxNum)
    {
        FISD_aenTextActive[enTextId] = enTextStatus;
    }
}

U8 FISD_u8GetPartnumber(void)
{
	return u8Partnumber;
}
/*******************************************************************************************
* Function: FISD_boGetTextShowTimeResetFlag
* Description: Get warning text show time reset flag
* Parameters: @enTextId: WarningText ID
* Return: WarningText state
********************************************************************************************/
BOOL FISD_boGetTextShowTimeResetFlag(tenTextId enTextId)
{
    BOOL boResult = FALSE;

    if(enTextId < nDIC_MaxNum)
    {
        boResult = FISD_aboTextShowTimeResetFlag[enTextId];
    }

    return boResult;
}

void FISD_vResetTextShowTimeResetFlag(tenTextId enTextId)
{
    if(enTextId < nDIC_MaxNum)
    {
        FISD_aboTextShowTimeResetFlag[enTextId] = FALSE;
    }
}

BOOL FISD_vSetTextShowTimeResetFlag(tenTextId enTextId)
{
    BOOL boResult = FALSE; /*Set failure*/

    if(enTextId < nDIC_MaxNum)
    {
        FISD_aboTextShowTimeResetFlag[enTextId] = TRUE;
        boResult = TRUE;
    }

    return boResult;
}
/*******************************************************************************************
* Function: FISD_boGetSoundOnTimeResetFlag
* Description: Get sound on time reset flag
* Parameters: @enBuzzerId: sound warning ID
* Return: reset flag
********************************************************************************************/
BOOL FISD_boGetSoundOnTimeResetFlag(tenBuzzerId enBuzzerId)
{
    BOOL boResult = FALSE;

    if(enBuzzerId < nBUZ_MaxNum)
    {
        boResult = FISD_aboSoundOnTimeResetFlag[enBuzzerId];
    }

    return boResult;
}

/*******************************************************************************************
* Function: FISD_boGetTextShowTimeResetFlag
* Description: Get warning text show time reset flag
* Parameters: @enTextId: WarningText ID
* Return: WarningText state
********************************************************************************************/
void FISD_vSetConfig(void)
{
    FISD_vFuncConfigInit();
}

/*******************************************************************************************
* Function: FISD_enGetAdasCurWarnFunction
* Description: get adas current warn function
* Parameters: None
* Return: adas warn
********************************************************************************************/
tenAdasWarnCurFuncTypedef FISD_enGetAdasCurWarnFunction(void)
{
    return FISD_enCurAdasWarn;
}

/*******************************************************************************************
* Function: FISD_u8GetPlgPercent
* Description: PLG open percent
* Parameters: None
* Return: PLG open percent
********************************************************************************************/
U16 FISD_u16GetPlgAngleHmiShowValue(void)
{
    return FISD_u16PlgHmiShowAngle_Pre;
}
/*******************************************************************************************
* Function: FISD_boGetCalendarAndCarSts
* Description: Calendar and car status
* Parameters: None
* Return: Calendar and car status
********************************************************************************************/
BOOL FISD_boGetCalendarAndCarSts(void)
{
    return FISD_boCalendarAndCarSts;
}
/*******************************************************************************************
* Function: FISD_enGetKeySts
* Description: Get Key status
* Parameters: None
* Return: Key status
********************************************************************************************/
tenKeySts FISD_enGetKeySts(void)
{
    return (tenKeySts)FISD_u16SignalValue[nKeySts];
}
/*******************************************************************************************
* Function: FISD_vSetMenuDenied
* Description: Menu denied reminder
* Parameters: None
* Return: None
********************************************************************************************/
void FISD_vSetMenuDenied(void)
{
    FISD_boMenuDeniedFlag = TRUE;
}

/*******************************************************************************************
* Function: FISD_boInstrumentIsInSelfCheckSts
* Description: Menu denied reminder
* Parameters: None
* Return: None
********************************************************************************************/
BOOL FISD_boInstrumentIsInSelfCheckSts(void)
{
    BOOL boRteVal = FALSE;
    if((FISD_boSelfCheckFlag == TRUE) && \
            ((FISD_u16DebounceCount_D1 < TIMECOUNT_3s)))
    {
        boRteVal = TRUE;
    }

    return boRteVal;
}
/*******************************************************************************************
* Function: FISD_enGetBuzzerState
* Description: Get Buzzer state by buzzer ID
* Parameters: @enBuzId: buzzer ID
* Return: @Buzzer state
********************************************************************************************/
tenBuzzerStatus FISD_enGetBuzzerState(tenBuzzerId enBuzId)
{
    tenBuzzerStatus result = nBuz_Inactive;

    if(enBuzId < nBUZ_MaxNum)
    {
        result = FISD_aenSoundActive[enBuzId];
    }

    return result;
}

void FISD_enSetBuzzerState(tenBuzzerId enBuzId , tenBuzzerStatus enBuzzerStatus)
{
    if(enBuzId < nBUZ_MaxNum)
    {
        FISD_aenSoundActive[enBuzId] = enBuzzerStatus;
    }
}


/*******************************************************************************************
* Function: FISD_u32GetFollowMeRestTime
* Description: Return Follow Me rest time in second
* Parameters: none
* Return: none
********************************************************************************************/
U32 FISD_u32GetFollowMeRestTime(void)
{
	return (FISD_u32FollowMeRestTime / 100u) + 1; /*10ms to 1s*/		
}

/*******************************************************************************************
* Function: FISD_boGetEcoModeSwSts
* Description: Return ECO work status
* Parameters: none
* Return: none
********************************************************************************************/
BOOL FISD_boGetEcoModeSwSts(void)
{
    return ((TRUE == FISD_boIndFuncValid[nIND_EcoWork]) ? FISD_aenIndicatorState[nIND_EcoWork] : 1u);
}

/*******************************************************************************************
* Function: FISD_u8GetRainSensorValue
* Description: Return Rain sensor current value status
* Parameters: none
* Return: none
********************************************************************************************/
U8 FISD_u8GetRainSensorValue(void)
{
    static U8 su8RteVal = 0u;

    if(FISD_u8RainNextValue != 0)
    {
        su8RteVal = FISD_u8RainNextValue;
    }
    return su8RteVal;
}

/*******************************************************************************************
* Function: FISD_boGetLimitCruiseSpeedValid
* Description: Return limit cruise speed is valid or not
* Parameters: none
* Return: none
********************************************************************************************/
BOOL FISD_boGetLimitCruiseSpeedValid(void)
{
    return FISD_boLimitCruiseSpeedValid;
}

/*******************************************************************************************
* Function: FISD_u16GetLimitCruiseSpeedValue
* Description: Return limit cruise speed value
* Parameters: none
* Return: none
********************************************************************************************/
U16 FISD_u16GetLimitCruiseSpeedValue(void)
{
    return FISD_u16LimitCruiseSpeedValue;
}

/*******************************************************************************************
* Function: FISD_u8GetAccObjectDistance
* Description: Return Acc Time Gap Distance
* Parameters: none
* Return: none
********************************************************************************************/
U8 FISD_u8GetAccObjectDistance(void)
{
    return FISD_u8ObjectDistance;
}

/*******************************************************************************************
* Function: FISD_boGetSelfCheckFlag
* Description: Return self check flag
* Parameters: none
* Return: none
********************************************************************************************/
BOOL FISD_boGetSelfCheckFlag(void)
{
    return FISD_boSelfCheckFlag;
}

/*******************************************************************************************
* FILE LOCAL FUNCTION
*
* In this section declare
* - all file local function prototypes needed for your module.
********************************************************************************************/

/* Local Function Define*/
/*******************************************************************************************
* Description of Behaviour:
* Initialize local variable
*
* Parameters:
* None
*
* Return Value  (Value Typ Range):
* None
*
********************************************************************************************/
static void FISD_vInitLocalVar(void)
{
    U16 u16Num = 0u;
    U8 language = 0;

    /*Get eeprom data*/

    /*Reset indicator*/
    for(u16Num = 0u; u16Num < nIND_MaxNum; u16Num++)
    {
        FISD_aenIndicatorState[u16Num] = nInd_Off;
    }
#if 0
    CfgMgr_u8ConfigValueGet(CONFIG_LANGUAGE, &language);
    if(LANGUAGE_PORTUGUESE==language)
    {
        // nothing to do, just use latest workmode
    }
    else if(TRUE == Rte_FISD_u8GetCONFIG_ThreeDriverModeCount())
    {
        IND_SET(nIND_NormalWork, nInd_On);
    }
    else
    {
        FISD_aenIndicatorState[nIND_EcoWork] = nInd_On;/*Default on*/
    }
#endif
    /*Reset WarnText*/
    for(u16Num = 0u; u16Num < nDIC_MaxNum; u16Num++)
    {
        FISD_aenTextActive[u16Num] = nDic_Inactive;
        FISD_aboTextShowTimeResetFlag[u16Num] = FALSE;
    }

    /*Reset Buzzer*/
    for(u16Num = 0u; u16Num < nBUZ_MaxNum; u16Num++)
    {
        FISD_aenSoundActive[u16Num] = nBuz_Inactive;
        FISD_aboSoundOnTimeResetFlag[u16Num] = FALSE;
    }

    /*Signal value init*/
    for(u16Num = 0u; u16Num < nSigMaxNum; u16Num++)
    {
        FISD_u16SignalValue[u16Num] = 0u;
    }

    FISD_boIgnOnToOffFlag = 0xff;
    FISD_u16IgnOffTimerS = 0u;
    FISD_boSelfCheckFlag = FALSE;
    FISD_u32FollowMeElapseTime = 0u;
    FISD_u32FollowMeRestTime = 0u;
    FISD_boFollowMeActiveFlag = FALSE;
    FISD_u32TurnOffLcdTime = 0u; /*Unit:10ms*/
    FISD_u16FollowMeLastValue = 0u;
    FISD_u16LastEngineSpeed = 0u;
    FISD_u16LastAVHSts = 0xFFFF;
    FISD_u16LastHDCCtrlSts = 0u;
    FISD_u32OilPressureLowTime = 0u;
    FISD_u32EcoSwitchTime = 0u;
    FISD_boEngineShutdown = FALSE;
    FISD_u8RainLastRawValue = 0u;
    FISD_u8RainLastValue = 0u;
    FISD_u8RainNextValue = 0u;
    FISD_u16LimitCruiseSpeedValue = 0u;
    FISD_boLimitCruiseSpeedValid = FALSE;
    FISD_u8PlgPercent = 0u;
    FISD_u16PlgHmiShowAngle_Pre = 0u;
    FISD_boPlgOpenNotChange2sFlag = FALSE;
    FISD_u16PlgOpenSignal_Pre = 0u;
    FISD_boWarmingHappenFlag = FALSE;
    FISD_enIgnState = FALSE;
    FISD_u16IgnOnTimerS = 0u;
    FISD_boMenuDeniedFlag = FALSE;
    FISD_u16MenuDeniedCount = 0u;
    FISD_u16DebounceCount_Crank_D1_D2 = 0u;
    FISD_u16DebounceCount_Crank_D1 = 0u;
    FISD_u16DebounceCount_D1 = 0u;
    FISD_boDebounceFlag_Crank_D1 = TRUE;
    FISD_enLastPowerMode = OomPWRMAXMODE;
    FISD_enLastKeySts = nOFF;
    FISD_boFuelLevelLowFlag = FALSE;
    FISD_u32FuelLevelLowOdo = Rte_FISD_u32GetOdometer();
    WarningMessageIDneedInitFlag = FALSE;
	WarningRmninfoneedInitFlag = FALSE;
    FISD_boCalendarAndCarSts = FALSE;
    FISD_BackLightStsRecord = 0u;
    FISD_u16CalendarAndCarStsDelay = 0u;
    FISD_enCurAdasWarn = enAdasWarn_Off;
    FISD_boIsCrankOn = False;
}

/*******************************************************************************************
* Function: FISD_vCheckEngineShutdown
* Description: Check engine shutdown or not
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vCheckEngineShutdown(void)
{
    U16 u16NextEngineSpeed = Rte_FISD_u16GetEngineSpeedValue();

    if((FISD_u16LastEngineSpeed > FISD_u16EngineRunValue) && \
            (u16NextEngineSpeed <= FISD_u16EngineRunValue))
    {
        FISD_boEngineShutdown = TRUE;
    }

    FISD_u16LastEngineSpeed = u16NextEngineSpeed;

    if(u16NextEngineSpeed > FISD_u16EngineRunValue)
    {
        FISD_boEngineShutdown = FALSE;
    }
}

/*******************************************************************************************
* Function: FISD_vFuncConfigInit
* Description: Load node configuration from EEprom
* Parameters: none
* Return: none
********************************************************************************************/
/*INDENT-OFF*/
static void FISD_vFuncConfigInit(void)
{
    /*Location: Column I of FISD_Valid_Config sheet in T18_Configuration_Tool(FISD_SoundMgr_SYN_TextMgr).xlsx*/
    do
    {
    /*Indicator Function config*/\
	 FISD_boIndFuncValid[nIND_TurnLeft]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_TurnRight]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_LittleLamp]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_HighBeam]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_HMA_Grey]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_HMA_White]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_HMA_Yellow]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RearFog]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_DrvDaylight]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_Seatbelt]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RearLeftSeatBelt_Red]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RearLeftSeatBelt_Green]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RearMidSeatBelt_Red]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RearMidSeatBelt_Green]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RearRightSeatBelt_Red]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RearRightSeatBelt_Green]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_EpcIndicator]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	 FISD_boIndFuncValid[nIND_MIL_Lamp]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	 FISD_boIndFuncValid[nIND_HandBrake_EpbWork_Red_P]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_EpbFault_Yellow_P]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_AutoHold_Green_P]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_Charger_Iem]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_BrakeFluidLevelLow_EbdFault]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_ABS_Fault]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_ESP_Fault]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_ESP_Shield]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_Airbag]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_TyrePressureLow]=(BOOL)(Rte_FISD_u8GetCONFIG_TPMS());\
	 FISD_boIndFuncValid[nIND_TransmissionFault_CVT_DCT]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	 FISD_boIndFuncValid[nIND_EPS_Fault_Yellow]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_CoolantTempHigh]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	 FISD_boIndFuncValid[nIND_WarmingUp]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	 FISD_boIndFuncValid[nIND_EngineOilPressureLow]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	 FISD_boIndFuncValid[nIND_FuelLevelLow]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	 FISD_boIndFuncValid[nIND_HDC]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_HDC_Fault]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_StartStopSysWork]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	 FISD_boIndFuncValid[nIND_StartStopSysFault]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	 FISD_boIndFuncValid[nIND_BlindGreen]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_BlindYellow]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_LDW_Green]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_LDW_Yellow]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_LDW_Grey]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_LDW_LKA_Green]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_LDW_LKA_Yellow]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_LDW_LKA_Grey]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_SLASpd_Defect]=(BOOL)(Rte_FISD_u8GetCONFIG_CountryOrRegion() == 0);\
	 FISD_boIndFuncValid[nIND_SLASpd_120]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_SLASpd_40]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_Cruise]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_ACC_GreyBlack]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_ACC_Greywhite]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_ACC_GreenBlack]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_ACC_GreenGreen]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_ACC_YellowSolid]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_FCW_AEB_Yellow]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_FCW_Red]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_AEB_OFF_Yellow]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_FuckTraffic_Grey]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_FuckTraffic_Green]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_FuckTraffic_Yellow]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_EcoWork]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_SportWork]=(BOOL)(1u);\
     FISD_boIndFuncValid[nIND_Overspeed]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_FatigueReminder]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_Mainten]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_WarningIcon]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_DoorOpen]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_LimitOn]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()!= nPowerType_EV);\
	 FISD_boIndFuncValid[nIND_CruiseOn]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_ESCLSerFault]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_ESCLFault]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_ESCLUnlock]=(BOOL)(0u);\
	 FISD_boIndFuncValid[nIND_ACC_YellowEmpty]=(BOOL)(0u);\
	 FISD_boIndFuncValid[nIND_SlaSpdLimitWarn]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_CoolantWhite]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	 FISD_boIndFuncValid[nIND_FuelWhite]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	 FISD_boIndFuncValid[nIND_LimitCancel]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()!= nPowerType_EV);\
	 FISD_boIndFuncValid[nIND_LimitDefault]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()!= nPowerType_EV);\
	 FISD_boIndFuncValid[nIND_CruiseCancel]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_LimitSpdFlash]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_CruiseSpdFlash]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_NormalWork]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_CancelDisplay]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_FrontFog]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	 FISD_boIndFuncValid[nIND_LeftBlindLineYellow]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_LeftBlindAreaYellow]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RightBlindLineYellow]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RightBlindAreaYellow]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_HoodOpen]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_TrunkOpen]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_DriverDoorOpen]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_PsngrDoorOpen]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RLDoorOpen]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RRDoorOpen]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_GpfFull_Green]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_NotFindCar_OneBar]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_NotFindCar_TwoBar]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_NotFindCar_ThreeBar]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_FindCar_OneBar]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_FindCar_TwoBar]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_FindCar_ThreeBar]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_LDW_LKA_Switch]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_GpfOverLimited_Yellow]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_GearboxFault_Red]=(BOOL)((Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	 FISD_boIndFuncValid[nIND_TransmisionFault_Red]=(BOOL)((Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	 FISD_boIndFuncValid[nIND_TransmisionFault_Yellow]=(BOOL)((Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	 FISD_boIndFuncValid[nIND_DriverSeatbelt]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_CopilotSeatbelt]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RearLeftSeatbelt]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RearMiddleSeatbelt]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RearRightSeatbelt]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_LDW_LL_White]=(BOOL)(Rte_FISD_u8GetCONFIG_FCM());\
	 FISD_boIndFuncValid[nIND_LDW_LL_Grey]=(BOOL)(Rte_FISD_u8GetCONFIG_FCM());\
	 FISD_boIndFuncValid[nIND_LDW_LL_Yellow]=(BOOL)(Rte_FISD_u8GetCONFIG_FCM());\
	 FISD_boIndFuncValid[nIND_LDW_LL_Green]=(BOOL)(Rte_FISD_u8GetCONFIG_FCM());\
	 FISD_boIndFuncValid[nIND_LDW_RL_White]=(BOOL)(Rte_FISD_u8GetCONFIG_FCM());\
	 FISD_boIndFuncValid[nIND_LDW_RL_Grey]=(BOOL)(Rte_FISD_u8GetCONFIG_FCM());\
	 FISD_boIndFuncValid[nIND_LDW_RL_Yellow]=(BOOL)(Rte_FISD_u8GetCONFIG_FCM());\
	 FISD_boIndFuncValid[nIND_LDW_RL_Green]=(BOOL)(Rte_FISD_u8GetCONFIG_FCM());\
	 FISD_boIndFuncValid[nIND_PleaseTieSeatbelt]=(BOOL)(0u);\
	 FISD_boIndFuncValid[nIND_PleaseColseDoor]=(BOOL)(0u);\
	 FISD_boIndFuncValid[nIND_CloseDoorTieSeatbelt]=(BOOL)(0u);\
	 FISD_boIndFuncValid[nIND_AFS_Fault]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RL_SeatBelt_grey]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RM_SeatBelt_grey]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RR_SeatBelt_grey]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_LimitSpd_Red_Flag]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_SnowWork]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	 FISD_boIndFuncValid[nIND_MudWork]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	 FISD_boIndFuncValid[nIND_SandWork]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	 FISD_boIndFuncValid[nIND_AWD_Yellow]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	 FISD_boIndFuncValid[nIND_AWD_Red]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	 FISD_boIndFuncValid[nIND_IceRoad]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_SmartHeadLight_Whilte]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_SmartHeadLight_Yellow]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_WashingWater]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_TimeGap_Fourbar]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_TimeGap_Fivebar]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_LDW_LKA_Red]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_RaebRctb]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_EPS_Fault_Red]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_CheryPilot_Grey]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	 FISD_boIndFuncValid[nIND_CheryPilot_Green]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	 FISD_boIndFuncValid[nIND_CheryPilot_Red]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	 FISD_boIndFuncValid[nIND_CheryPilot_Yellow]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	 FISD_boIndFuncValid[nIND_OffRoadWork]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	 FISD_boIndFuncValid[nIND_AutoHold_White_P]=(BOOL)(1u);\
	 FISD_boIndFuncValid[nIND_AutoHold_Yellow_A]=(BOOL)(1u);\
  	 FISD_boIndFuncValid[nIND_BSDWarn_Left]=(BOOL)(1u);\
  	 FISD_boIndFuncValid[nIND_BSDWarn_Right]=(BOOL)(1u);\
  	 FISD_boIndFuncValid[nIND_RCWWarn]=(BOOL)(1u);\
  	 FISD_boIndFuncValid[nIND_BatLowAlarm_Yellow]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
    FISD_boIndFuncValid[nIND_BatLowAlarm_Red]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
    FISD_boIndFuncValid[nIND_BatLowAlarm_White]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
    FISD_boIndFuncValid[nIND_PTReady]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
    FISD_boIndFuncValid[nIND_SocDisRed]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
    FISD_boIndFuncValid[nIND_PowerModeEV]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	 FISD_boIndFuncValid[nIND_PowerModeHEV]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1);\
	 FISD_boIndFuncValid[nIND_PowerModeEVplus]=(BOOL)((Rte_FISD_u8GetCONFIG_DrivingPowerType() == nPowerType_PHEV) && (Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3));\
  	 FISD_boIndFuncValid[nIND_ElectricityModeDefault]=(BOOL)(((Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV) && (Rte_FISD_u8GetCONFIG_PHEV_TYPE() != nIND_PHEV_TYPE_3))||(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_EV));\
  	 FISD_boIndFuncValid[nIND_ElectricityModeAuto]=(BOOL)(((Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV) && (Rte_FISD_u8GetCONFIG_PHEV_TYPE() != nIND_PHEV_TYPE_3))||(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_EV));\
  	 FISD_boIndFuncValid[nIND_ElectricityModeSave]=(BOOL)(((Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV) && (Rte_FISD_u8GetCONFIG_PHEV_TYPE() != nIND_PHEV_TYPE_3))||(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_EV));\
  	 FISD_boIndFuncValid[nIND_ElectricityModeCharge]=(BOOL)(((Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV) && (Rte_FISD_u8GetCONFIG_PHEV_TYPE() != nIND_PHEV_TYPE_3))||(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_EV));\
    FISD_boIndFuncValid[nIND_RegenerateLevel]=(BOOL)((Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2)||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
    FISD_boIndFuncValid[nIND_LimitPower_Yellow]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
    FISD_boIndFuncValid[nIND_AVAS_Yellow]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
    FISD_boIndFuncValid[nIND_BMSH_InsulationSts_Yellow]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	 FISD_boIndFuncValid[nIND_BMSH_InsulationSts_Red]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
    FISD_boIndFuncValid[nIND_BATPower_Red]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
    FISD_boIndFuncValid[nIND_OBC_CC_ConnectSts_Red]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
    FISD_boIndFuncValid[nIND_OBC_CC_ConnectSts_Blue]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
    FISD_boIndFuncValid[nIND_Reserve_Charging_Open]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
  	FISD_boIndFuncValid[nIND_DMSSts_Green]=(BOOL)(1u);\
    FISD_boIndFuncValid[nIND_DMSSts_Yellow]=(BOOL)(1u);\
    FISD_boIndFuncValid[nIND_DrvCameraSts]=(BOOL)(1u);\
    FISD_boIndFuncValid[nIND_DrvAbnormStAla]=(BOOL)(1u);\
    FISD_boIndFuncValid[nIND_ISA_SpdLimtyp1]=(BOOL)(1u);\
    FISD_boIndFuncValid[nIND_ISA_SpdLimtyp2]=(BOOL)(1u);\
    FISD_boIndFuncValid[nIND_ISA_SPLUnconfirmed]=(BOOL)(1u);\
    FISD_boIndFuncValid[nIND_ISA_FuncErr]=(BOOL)(Rte_FISD_u8GetCONFIG_CountryOrRegion() != 0);\
    FISD_boIndFuncValid[nIND_SCF_PopoverReq]=(BOOL)(1u);\
    FISD_boIndFuncValid[nIND_ISA_UnknownWarning]=(BOOL)(1u);\
    FISD_boIndFuncValid[nIND_VCU_HVReady]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
    FISD_boIndFuncValid[nIND_TMF_MILSts]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
    FISD_boIndFuncValid[nIND_BatteryTempLight]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
    FISD_boIndFuncValid[nIND_ThirdRowSB_Left_Red]=(BOOL)(Rte_FISD_u8GetCONFIG_ThdSeatBeltFunc() == 1u);\
    FISD_boIndFuncValid[nIND_ThirdRowSB_Left_Green]=(BOOL)(Rte_FISD_u8GetCONFIG_ThdSeatBeltFunc() == 1u);\
    FISD_boIndFuncValid[nIND_ThirdRowSB_Left_Grey]=(BOOL)(Rte_FISD_u8GetCONFIG_ThdSeatBeltFunc() == 1u);\
    FISD_boIndFuncValid[nIND_ThirdRowSB_Right_Red]=(BOOL)(Rte_FISD_u8GetCONFIG_ThdSeatBeltFunc() == 1u);\
    FISD_boIndFuncValid[nIND_ThirdRowSB_Right_Green]=(BOOL)(Rte_FISD_u8GetCONFIG_ThdSeatBeltFunc() == 1u);\
    FISD_boIndFuncValid[nIND_ThirdRowSB_Right_Grey]=(BOOL)(Rte_FISD_u8GetCONFIG_ThdSeatBeltFunc() == 1u);\
    FISD_boIndFuncValid[nIND_ThdLeftSeatbelt]=(BOOL)(Rte_FISD_u8GetCONFIG_ThdSeatBeltFunc() == 1u);\
    FISD_boIndFuncValid[nIND_ThdRightSeatbelt]=(BOOL)(Rte_FISD_u8GetCONFIG_ThdSeatBeltFunc() == 1u);\
    FISD_boIndFuncValid[nIND_BATPower_Red_PHEV3]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
    FISD_boIndFuncValid[nIND_PHEV_CC2_Charging]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV);\
	FISD_boIndFuncValid[nIND_PsngerSeatBlt]=(BOOL)(Rte_FISD_u8GetCONFIG_LeftAndRightRudderType()== nRudder_Right);\
	FISD_boIndFuncValid[nIND_LimitOn_Green]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_EV);\
	FISD_boIndFuncValid[nIND_LimitOn_Grey]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_EV);\
	FISD_boIndFuncValid[nIND_LDW_OFF]=(BOOL)(Rte_FISD_u8GetCONFIG_CountryOrRegion() != 0);\
	FISD_boIndFuncValid[nIND_LDP_ELK_OFF]=(BOOL)(Rte_FISD_u8GetCONFIG_CountryOrRegion() != 0);\
	FISD_boIndFuncValid[nIND_EcoPlusWork]=(BOOL)(1u);\
	FISD_boIndFuncValid[nIND_ISA_bell]=(BOOL)(1u);\
	FISD_boIndFuncValid[nIND_ISA_horn]=(BOOL)(1u);\
	FISD_boIndFuncValid[nIND_ISA_OFF]=(BOOL)(1u);\
	FISD_boIndFuncValid[nIND_OverSpeedWarn]=(BOOL)(1u);\
	FISD_boIndFuncValid[nIND_TrailerConnectSts]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType() == nPowerType_EV);\

    
	/*DIC Function config*/\
	FISD_boDicFuncValid[nDIC_RemoteModeActive_AT]=(BOOL)((Rte_FISD_u8GetCONFIG_Tcu1OrTcu2OrTcu3OrTcu4AndPeps())||(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_EV));\
	FISD_boDicFuncValid[nDIC_RemoteModeActive_MT]=(BOOL)((Rte_FISD_u8GetCONFIG_NoTcu1NoTcu2NoTcu3NoTcu4())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()!= nPowerType_EV));\
	FISD_boDicFuncValid[nDIC_2]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_FollowMeHome]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ESCLCritiFailure]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_5]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_FcwActive]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_SlowDownImmediately]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	FISD_boDicFuncValid[nDIC_AebActive]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_AEB_Working]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	FISD_boDicFuncValid[nDIC_ReabRctWarning]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_11]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_CareForRearCar]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_TakeOverFunExit]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_14]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_NOC_GoingtoExit]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	FISD_boDicFuncValid[nDIC_16]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_TJA_ICA_IsExitReminder]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_AccCancel]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_TransTempHighPleaseStop]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boDicFuncValid[nDIC_TransTempHighHold5Minute_7DCT]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boDicFuncValid[nDIC_21]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_22]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_DoorLock]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_DOW_CloseDoor]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_25]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_26]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_27]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_28]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_29]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_ShutEngineCheckCoolant]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boDicFuncValid[nDIC_ReleaseHandBrake]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_32]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_BattOverHot]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boDicFuncValid[nDIC_BattRelayOpen]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boDicFuncValid[nDIC_35]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_TransmissionTempHigh]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU1_CVT19()||Rte_FISD_u8GetCONFIG_TCU2_6DCT())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boDicFuncValid[nDIC_GearboxSeriousFault]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boDicFuncValid[nDIC_TransmissionTempHighHold5Minute]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU1_CVT19()||Rte_FISD_u8GetCONFIG_TCU2_6DCT())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boDicFuncValid[nDIC_39]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_40]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_ESCLFuncFailure]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_DoorOpenWithText]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_43]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_BeltWarnFastenSeatbeltWithText]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_45]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_46]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_CautionParkOnHighSlope]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_NoSmartKeyDetected2_2]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_LimitSpeedCancel]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boDicFuncValid[nDIC_50]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_51]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_TakeOverWarning]=(BOOL)(1U);\
	FISD_boDicFuncValid[nDIC_TakeOverSafty]=(BOOL)(1U);\
	FISD_boDicFuncValid[nDIC_TakeOver_1]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_TakeOverReq]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_LongTimeTakeOver]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_57]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_58]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_59]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_PleaseCheckRearSeats]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_AwdFault]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_62]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_CDP_Failure]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_FillBrakeFluid]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_BrakeFluidSts]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CheckEspSystem]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CheckAbsSystem]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_68]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_TransmissionCheck]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU1_CVT19()||Rte_FISD_u8GetCONFIG_TCU2_6DCT())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boDicFuncValid[nDIC_TransmissionFault_7DTC]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boDicFuncValid[nDIC_CheckPas]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_72]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_73]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_74]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_AebErr]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_AEB_Error]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	FISD_boDicFuncValid[nDIC_FcwErr]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_FCW_Error]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	FISD_boDicFuncValid[nDIC_FWAWarningSts]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_80]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_81]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_82]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_RadarCover]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CleanRadar]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	FISD_boDicFuncValid[nDIC_RadarErr]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CheckRadar]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	FISD_boDicFuncValid[nDIC_CameraFault]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CheckCamera]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	FISD_boDicFuncValid[nDIC_CameraDirty]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CleanCamera]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	FISD_boDicFuncValid[nDIC_BlkStsLeft]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_BlkStsRight]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_LdpFault]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_LDP_Error]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	FISD_boDicFuncValid[nDIC_LdwFault]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_LDW_Error]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	FISD_boDicFuncValid[nDIC_ElkFault]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ELK_Error]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	FISD_boDicFuncValid[nDIC_CheckBlindSystem]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_BSD_Error]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	FISD_boDicFuncValid[nDIC_AccErr]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ACC_Error]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	FISD_boDicFuncValid[nDIC_TJA_ICA_Check]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CP_Error]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_RL_RadarProbe]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_RLM_RadarProbe]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_107]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_RRM_RadarProbe]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_RR_RadarProbe]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_FL_RadarProbe]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_FLM_RadarProbe]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_FRM_RadarProbe]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_FR_RadarProbe]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_RightRear_RadarProbe]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_LeftRear_RadarProbe]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_RightFront_RadarProbe]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_LeftFront_RadarProbe]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ShutEngineCheckEngineOil]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()!= nPowerType_EV);\
	FISD_boDicFuncValid[nDIC_HDC_Failure]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_HHC_Failure]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ISS_MHEVFailure]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boDicFuncValid[nDIC_IssPlsCheckIss]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boDicFuncValid[nDIC_GearFault_P]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boDicFuncValid[nDIC_CheckAirbag]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_AutoHoldFailure]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CheckTpms]=(BOOL)(Rte_FISD_u8GetCONFIG_TPMS());\
	FISD_boDicFuncValid[nDIC_GearShiftFailToStop]=(BOOL)(((Rte_FISD_u8GetCONFIG_GEAR_STRUCTURE() == 4) || (Rte_FISD_u8GetCONFIG_GEAR_STRUCTURE() == 5))?TRUE:FALSE);\
	FISD_boDicFuncValid[nDIC_GearShiftFailToSlow]=(BOOL)(((Rte_FISD_u8GetCONFIG_GEAR_STRUCTURE() == 4) || (Rte_FISD_u8GetCONFIG_GEAR_STRUCTURE() == 5))?TRUE:FALSE);\
	FISD_boDicFuncValid[nDIC_CheckAvmSystem]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CheckFrontCamera]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CheckLeftCamera]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CheckRightCamera]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CheckRearCamera]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_134]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_GpfParticleTrapOverPlsRepair]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ChildSaftyFault]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_AFSFault]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_BrakeLampFault]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ADBFault]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ReverseLampFault]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_RearFogFault]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_142]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_143]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_144]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_145]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_146]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_147]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_148]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_149]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_150]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_BatLowPlsStartEngine]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_BatLowEntertainmentLimit]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_153]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_CheckRainSensor]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_SmartKeyPowerLow]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_APAFailureSts]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_GpfParticleTrapFullPlsClear]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_158]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_FillFuel]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boDicFuncValid[nDIC_WashingWater]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_161]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_PLG_Open]=(BOOL)(Rte_FISD_u8GetCONFIG_PLG());\
	FISD_boDicFuncValid[nDIC_DoorOpenErr]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_DrvrStall]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_PassStall]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_LHRStall]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_RHRStall]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_DoorOpenWithNoText]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CloseDoorSeatbelt]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ActiveSeatBeltFailure]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ActiveSeatBeltsExpire]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CloseDoor_Avh_Epb]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_173]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_HUDOverHot]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_ESCLUnlocklamming]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_EPS_FailureToInitAngle]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_EotNotLearnt]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_PleaseStampBraking]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boDicFuncValid[nDIC_TakeOver_2]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_TakeOver_3]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_CarMovePleaseSwitch_P]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boDicFuncValid[nDIC_ACC_Exit]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	FISD_boDicFuncValid[nDIC_PleaseShiftTheGearLeverToTheRight]=(BOOL)((Rte_FISD_u8GetCONFIG_GEAR_STRUCTURE() == 1)?TRUE:FALSE);\
	FISD_boDicFuncValid[nDIC_TwofeetBrake]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_StepBrakePedalReleaseHandbrake]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_NotInGearP]=(BOOL)(Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18());\
	FISD_boDicFuncValid[nDIC_SwitchtoP_AfterStopped]=(BOOL)(Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18());\
	FISD_boDicFuncValid[nDIC_IssUnstartPlsSwitchToGearN]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boDicFuncValid[nDIC_IssUnstartPlsStartEngineByHand]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boDicFuncValid[nDIC_LongitudinalOverride]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	FISD_boDicFuncValid[nDIC_AccStandWait]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_AEB_Off]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_FCW_Off]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_ACC_Recover]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_FrontCarDriveAway]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_DrivewayInformationActive]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CP_Exit]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_NotTime2ChangeL]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_EntranceRamp]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_NOC_Turn2LeftLane]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boDicFuncValid[nDIC_NOC_Turn2RightLane]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boDicFuncValid[nDIC_Turn2LeftLane]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boDicFuncValid[nDIC_Turn2RightLane]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boDicFuncValid[nDIC_Confirm2ChaneLeft]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boDicFuncValid[nDIC_Confirm2ChaneRight]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boDicFuncValid[nDIC_DriverconfirmRequest]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_207]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_208]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_209]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_210]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_211]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_212]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_213]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_214]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_NoSmartKeyDetected1_1]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ShiftToP_N]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_StartStepClutchPedal]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_StartStepBrakePedal]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ShiftToP]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CautionSmartKeyInCar]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ShutPower]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_NoSmartKeyDetected2_0]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_BrakingTillShowGear_P]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boDicFuncValid[nDIC_OperateGearPWhenEngineRun]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boDicFuncValid[nDIC_225]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_226]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_FatigueReminder]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_228]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_229]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_230]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_231]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_APANotDisplay]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_IntervalTime_Warning]=(BOOL)(1u);\
    FISD_boDicFuncValid[nDIC_LookCamara]=(BOOL)(1u);\
    FISD_boDicFuncValid[nDIC_LookAhead]=(BOOL)(1u);\
    FISD_boDicFuncValid[nDIC_KeepFaceClean]=(BOOL)(1u);\
    FISD_boDicFuncValid[nDIC_KeepEyeClean]=(BOOL)(1u);\
    FISD_boDicFuncValid[nDIC_KeepMouthClean]=(BOOL)(1u);\
    FISD_boDicFuncValid[nDIC_PleaseTurnRight]=(BOOL)(1u);\
    FISD_boDicFuncValid[nDIC_PleaseTurnLeft]=(BOOL)(1u);\
    FISD_boDicFuncValid[nDIC_PleaseTurnDown]=(BOOL)(1u);\
    FISD_boDicFuncValid[nDIC_PleaseTurnUp]=(BOOL)(1u);\
    FISD_boDicFuncValid[nDIC_FaceIDRegistered]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_IntervalDistance_Warning]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_Interval_Warning]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_MaintainVehicle]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_247]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_Warming]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boDicFuncValid[nDIC_Warmed]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boDicFuncValid[nDIC_IssOfflineCheckSts]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boDicFuncValid[nDIC_ClothesHooked]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_FactoryMode]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_TransportMode]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_DynoMode]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CrashMode]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_IceRoad]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ACC_Unused]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_ADAS_UnusedCuzCamera]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_ADAS_UnsedCuzRadar]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_AccActiveFalse]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_GearShiftPosition]=(BOOL)((Rte_FISD_u8GetCONFIG_GEAR_STRUCTURE() == 4)?TRUE:FALSE);\
	FISD_boDicFuncValid[nDIC_262]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_AutoHoldActive]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_HDC_Active]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_AEB_Unused]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_AebInactive]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_FCW_Unused]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_FcwActiveFalse]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_TJA_ICA_FailActive]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_TJA_ICA_Demand]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_AebSwitchOff_Textinfo]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_AEB_On]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_AebSwitchOn]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_FcwSwitchOff]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_FCW_On]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_FcwSwitchOn]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_TJA_ICA_Stop]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_IssHasStop]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boDicFuncValid[nDIC_IssHasStart]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boDicFuncValid[nDIC_TJA_ICA_On]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CP_Unsed]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_ChangLaneUnableCuzSpd]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_ChildSaftyUnlock]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ChildSaftyLock]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_285]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_286]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_287]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_288]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_289]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_290]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_EcoWork]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_NormalWork]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_SportWork]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_SnowWork]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_MudWork]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_OffRoadWork]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_SandWork]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_RememberSetSuccess]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_RememberSetFailure]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_RainSensorSensitivity_1]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_RainSensorSensitivity_2]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_RainSensorSensitivity_3]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_RainSensorSensitivity_4]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_OverSpdWarn]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_LDW_LKA_Switch_Not_Use]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_WarnSwitch2ADAS]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ASUFault]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ACC_Active]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_CP_Active]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boDicFuncValid[nDIC_DCLC_BadState]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boDicFuncValid[nDIC_CPMod_StopSafety]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boDicFuncValid[nDIC_AttentionVehiclesRight]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boDicFuncValid[nDIC_AttentionVehiclesLeft]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boDicFuncValid[nDIC_Seatbelt_avh_epb]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CloseDoorSeatbelt_Avh_epb]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_TransmissionCheck_1]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU1_CVT19()||Rte_FISD_u8GetCONFIG_TCU2_6DCT())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boDicFuncValid[nDIC_TransmissionCheck_2]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU1_CVT19()||Rte_FISD_u8GetCONFIG_TCU2_6DCT())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boDicFuncValid[nDIC_TimeGap]=(BOOL)(Rte_FISD_u8GetCONFIG_TIEJ() == 0);\
	FISD_boDicFuncValid[nDIC_GearLeverFailurePleaseRepairInTime]=(BOOL)((Rte_FISD_u8GetCONFIG_GEAR_STRUCTURE() == 1)?TRUE:FALSE);\
	FISD_boDicFuncValid[nDIC_FaceRecognizing]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_RegistrationFailedCameraAbnormal ]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_RegistrationFailedPleaseAgain]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_FaceUndetected]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_FaceUnregistered]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_RegistrationFailedDuplicateFace]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_RegistrationTimedOutPleaseAgain]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_LoginTimedOutPleaseLoginAgain]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_RegistrationCloudErr]=(BOOL)(1u);\
    FISD_boDicFuncValid[nDIC_FltySts]=(BOOL)(1u);\
    FISD_boDicFuncValid[nDIC_ClsdWarn]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_ACCSpeed]=(BOOL)(Rte_FISD_u8GetCONFIG_TIEJ());\
	FISD_boDicFuncValid[nDIC_TimeGapSetICM_1]=(BOOL)(Rte_FISD_u8GetCONFIG_TIEJ());\
	FISD_boDicFuncValid[nDIC_LimitSpeed]=(BOOL)(Rte_FISD_u8GetCONFIG_TIEJ());\
	FISD_boDicFuncValid[nDIC_CruiseControl]=(BOOL)(Rte_FISD_u8GetCONFIG_TIEJ());\
	FISD_boDicFuncValid[nDIC_BatPowerLow]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boDicFuncValid[nDIC_BatSeriousLoss]=(BOOL)((Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2)||Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_EV);\
   FISD_boDicFuncValid[nDIC_PT_Ready]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
   FISD_boDicFuncValid[nDIC_Unable2ChangeMode]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
   FISD_boDicFuncValid[nDIC_ReadyForRefuel]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1);\
   FISD_boDicFuncValid[nDIC_CompleteForRefuel]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1);\
   FISD_boDicFuncValid[nDIC_CheckRGC]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2);\
   FISD_boDicFuncValid[nDIC_CloseGasCapAlways]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2);\
   FISD_boDicFuncValid[nDIC_CloseGasCap]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1);\
   FISD_boDicFuncValid[nDIC_ShiftGear2Right]=(BOOL)((Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV) && (Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2));\
   FISD_boDicFuncValid[nDIC_Recovery_Level_High]=(BOOL)((Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2)||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
   FISD_boDicFuncValid[nDIC_Recovery_Level_Middle]=(BOOL)((Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2)||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
   FISD_boDicFuncValid[nDIC_Recovery_Level_Low]=(BOOL)((Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2)||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
   FISD_boDicFuncValid[nDIC_RiskOfLeakage]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
   FISD_boDicFuncValid[nDIC_Turnoff_Ign]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV);\
   FISD_boDicFuncValid[nDIC_Check_BMS]=(BOOL)(((Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2) && (Rte_FISD_u8GetCONFIG_DrivingPowerType() == nPowerType_PHEV))||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
   FISD_boDicFuncValid[nDIC_BAT_OverHeatAndGetAway]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
   FISD_boDicFuncValid[nDIC_Check_eMcu]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
   FISD_boDicFuncValid[nDIC_Charging_Failure]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
   FISD_boDicFuncValid[nDIC_DisCharging_Failure]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
   FISD_boDicFuncValid[nDIC_EnterChargingPage]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
   FISD_boDicFuncValid[nDIC_SevereHigVolFault]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
   FISD_boDicFuncValid[nDIC_BAT_temp_over_High]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
   FISD_boDicFuncValid[nDIC_BAT_temp_over_Low]=(BOOL)((Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2)||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
   FISD_boDicFuncValid[nDIC_WarmerRemind]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1);\
   FISD_boDicFuncValid[nDIC_EnterRegenerationSetting]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
   FISD_boDicFuncValid[nDIC_DMSSts]=(BOOL)(1u);\
    FISD_boDicFuncValid[nDIC_DrvCameraSts]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_DrvAbnormStAla_1]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_DrvAbnormStAla_2]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_DrvAbnormStAla_3]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_DrvAbnormStAla_4]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_DrvAbnormStAla_5]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_DrvAbnormStAla_6]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_DrvAbnormStAla_7]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_CloseLight]=(BOOL)(1u);\
	FISD_boDicFuncValid[nDIC_VehicleSystemFailure]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boDicFuncValid[nDIC_InvldCdnToDrvr_1]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boDicFuncValid[nDIC_InvldCdnToDrvr_2]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boDicFuncValid[nDIC_InvldCdnToDrvr_3]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boDicFuncValid[nDIC_InvldCdnToDrvr_4]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boDicFuncValid[nDIC_InvldCdnToDrvr_5]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boDicFuncValid[nDIC_InvldCdnToDrvr_6]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boDicFuncValid[nDIC_InvldCdnToDrvr_7]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boDicFuncValid[nDIC_LowSOCCLMLimitSts]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boDicFuncValid[nDIC_TowMode]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boDicFuncValid[nDIC_V2LSts_FuncSts]=(BOOL)(0u);\
	FISD_boDicFuncValid[nDIC_V2LFuncNotOpen]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boDicFuncValid[nDIC_V2LFuncDisable]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boDicFuncValid[nDIC_BatteryPreHeat]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boDicFuncValid[nDIC_BatteryKeepWarm]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boDicFuncValid[nDIC_TrailerConnectSts]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType() != nPowerType_EV);\
	FISD_boDicFuncValid[nDIC_TrailerConnectSts_EV]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType() == nPowerType_EV);\
    FISD_boDicFuncValid[nDIC_TransLimitedFunction]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boDicFuncValid[nDIC_TransHighTemperature]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boDicFuncValid[nDIC_ClutchHighTemperature]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boDicFuncValid[nDIC_ClutchSelfLearning]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boDicFuncValid[nDIC_BatSeriousLowSoc]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boDicFuncValid[nDIC_HcuBrkPsd]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boDicFuncValid[nDIC_OpenAirXConditioning]=(BOOL)(Rte_FISD_u8GetCONFIG_T26());\
    FISD_boDicFuncValid[nDIC_Check_BMS_PHEV3]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boDicFuncValid[nDIC_Check_eMcu_PHEV3]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
	FISD_boDicFuncValid[nDIC_ClsdWarn_BCM5]=(BOOL)(1u);\
    FISD_boDicFuncValid[nDIC_PowerLimit]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_EV);\
    FISD_boDicFuncValid[nDIC_LowBatteryInfo]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boDicFuncValid[nDIC_LowBatteryLowFuelInfo]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
	FISD_boDicFuncValid[nDIC_HVILSts]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
	FISD_boDicFuncValid[nDIC_HighTempLimit]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
	FISD_boDicFuncValid[nDIC_ChrgFillerAjar1]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
	FISD_boDicFuncValid[nDIC_ChrgFillerAjar2]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
	FISD_boDicFuncValid[nDIC_FuelTankLidSts]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
	FISD_boDicFuncValid[nDIC_FuelTankLidSystemFailureSts]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
	FISD_boDicFuncValid[nDIC_EDrvOnly]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
	FISD_boDicFuncValid[nDIC_ForcedPrkgChrg]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
	FISD_boDicFuncValid[nDIC_EngSelfMaiTin]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
	FISD_boDicFuncValid[nDIC_VehStrtWarn]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boDicFuncValid[nDIC_VINIncorrect]=(BOOL)(1u);\
    FISD_boDicFuncValid[nDIC_SCFSwitchedOn]=(BOOL)(1u);\
    FISD_boDicFuncValid[nDIC_SCFSwitchedOff]=(BOOL)(1u);\
    FISD_boDicFuncValid[nDIC_UnableToSwitchedOnSCF]=(BOOL)(1u);\

	 /*BUZ Function config*/\
	FISD_boBuzFuncValid[nBUZ_DriverTakeOver]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_FCWActive]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ESCLSerFault]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ESCLFault]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_PleaseStampBraking]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boBuzFuncValid[nBUZ_CarMovePleaseSwitch_P]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boBuzFuncValid[nBUZ_GearboxSeriousFault]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boBuzFuncValid[nBUZ_TransmissionFault_7DTC]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boBuzFuncValid[nBUZ_TransTempHighHold5Minute_7DCT]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boBuzFuncValid[nBUZ_GearFault_P]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boBuzFuncValid[nBUZ_BrakingTillShowGear_P]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boBuzFuncValid[nBUZ_TransTempHighPleaseStop]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boBuzFuncValid[nBUZ_NotInGearP]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boBuzFuncValid[nBUZ_SwitchtoP_AfterStopped]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boBuzFuncValid[nBUZ_OperateGearPWhenEngineRun]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU4_7DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boBuzFuncValid[nBUZ_TransmissionTempHighHold5Minute]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU1_CVT19()||Rte_FISD_u8GetCONFIG_TCU2_6DCT())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boBuzFuncValid[nBUZ_OVS_2]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_TakeOverTurnReq]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_NoSmartKeyDetected2_2]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_DOW]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_RCW]=(BOOL)(Rte_FISD_u8GetCONFIG_T18FL4() == 0);\
	FISD_boBuzFuncValid[nBUZ_PlgOpen]=(BOOL)(0u);\
	FISD_boBuzFuncValid[nBUZ_DoorOpen]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_GearShiftFailToStop]=(BOOL)(((Rte_FISD_u8GetCONFIG_GEAR_STRUCTURE() == 4) || (Rte_FISD_u8GetCONFIG_GEAR_STRUCTURE() == 5))?TRUE:FALSE);\
	FISD_boBuzFuncValid[nBUZ_Radar_PDC_1]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_Radar_PDC_2]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_Radar_PDC_3]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_Radar_PDC_4]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_BSD]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_RCTA]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_LDW]=(BOOL)(Rte_FISD_u8GetCONFIG_LDW()||Rte_FISD_u8GetCONFIG_FCM());\
	FISD_boBuzFuncValid[nBUZ_NoSmartKeyDetected1_1]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ShiftToP_N]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_StartStepClutchPedal]=(BOOL)(Rte_FISD_u8GetCONFIG_NoTcu1NoTcu2NoTcu3NoTcu4());\
	FISD_boBuzFuncValid[nBUZ_StartStepBrakePedal]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ShiftToP]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_CautionSmartKeyInCar]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ShutPower]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_NoSmartKeyDetected2_0]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_SmartKeyPowerLow]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ESCLUnlocklamming]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ESCLNotLocked]=(BOOL)(0u);\
	FISD_boBuzFuncValid[nBUZ_OVS_1]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_SLA]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_OVS_3]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_APANotDisplay]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_SBW]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_RearSBW]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_CoolantTempHigh]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boBuzFuncValid[nBUZ_EngineOilPressureLow]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boBuzFuncValid[nBUZ_HandBrakeNotRelease]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_CDP_RWU]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_GpfParticleTrapOverPlsRepair]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_GpfParticleTrapFullPlsClear]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_TransmissionTempHigh]=(BOOL)((Rte_FISD_u8GetCONFIG_TCU1_CVT19()||Rte_FISD_u8GetCONFIG_TCU2_6DCT())&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1));\
	FISD_boBuzFuncValid[nBUZ_EcoWork]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_SportWork]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_NormalWork]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_FatigueReminder]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_BrakeLampFault]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ReverseLampFault]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_RearFogFault]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ChildSaftyLock]=(BOOL)(0u);\
	FISD_boBuzFuncValid[nBUZ_ChildSaftyUnlock]=(BOOL)(0u);\
	FISD_boBuzFuncValid[nBUZ_ChildSaftyFault]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_RememberKeepSet]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_MenorySetKey]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_MaintainVehicle]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_FillFuel]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boBuzFuncValid[nBUZ_KeyInCar]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_TurningTick]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_TurningTock]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_LDWLKA_StatusChange]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_TJAICA_StatusChange]=(BOOL)(Rte_FISD_u8GetCONFIG_LDW()&&Rte_FISD_u8GetCONFIG_LKA());\
	FISD_boBuzFuncValid[nBUZ_ANIMATION_Start]=(BOOL)(0u);\
	FISD_boBuzFuncValid[nBUZ_FluidLevelLow]=(BOOL)(0u);\
	FISD_boBuzFuncValid[nBUZ_TakeYourPhone]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_CameraShot]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_SBW_Test]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_DoorOpen_Test]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_TurningTick_Test]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_TurningTock_Test]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_SQUARE_400Hz]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_SQUARE_1KHz]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_SQUARE_2K5Hz]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_SnowWork]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_MudWork]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_SandWork]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_AwdFault]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_AvmKey]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_IceRoad]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ADBFault]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_WashingWater]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_HUDOverHot]=(BOOL)(0u);\
	FISD_boBuzFuncValid[nBUZ_DriveAway]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_TJAICA_isexitreminder]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_LdpLdwWarning]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ElkSafty]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_TakeOverWarning]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_TakeOverSafty]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_SCF]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_NoParking]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_Warming]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boBuzFuncValid[nBUZ_EotNotLearnt]=(BOOL)(0u);\
	FISD_boBuzFuncValid[nBUZ_GearShiftFailToSlow]=(BOOL)(((Rte_FISD_u8GetCONFIG_GEAR_STRUCTURE() == 4) || (Rte_FISD_u8GetCONFIG_GEAR_STRUCTURE() == 5))?TRUE:FALSE);\
	FISD_boBuzFuncValid[nBUZ_GearShiftPosition]=(BOOL)((Rte_FISD_u8GetCONFIG_GEAR_STRUCTURE() == 4)?TRUE:FALSE);\
	FISD_boBuzFuncValid[nBUZ_CheckPas]=(BOOL)(0u);\
	FISD_boBuzFuncValid[nBUZ_ACC_Exit]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_AEB_Working]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_SlowDownImmediately]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_FrontCarDriveAway]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_ACC_Recover]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_ADAS_UnusedCuzCamera]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_ADAS_UnsedCuzRadar]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_CP_Exit]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_NOC_GoingtoExit]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_TakeOver_1]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_TakeOver_2]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_TakeOver_3]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_LongTimeTakeOver]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_EntranceRamp]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_NOC_Turn2LeftLane]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boBuzFuncValid[nBUZ_NOC_Turn2RightLane]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boBuzFuncValid[nBUZ_LateralOverride]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boBuzFuncValid[nBUZ_LongitudinalOverride]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E()==0);\
	FISD_boBuzFuncValid[nBUZ_Confirm2ChaneLeft]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boBuzFuncValid[nBUZ_Confirm2ChaneRight]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boBuzFuncValid[nBUZ_AutoHoldActive]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_HDC_Active]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_StepBrakePedalReleaseHandbrake]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_IssOfflineCheckSts]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boBuzFuncValid[nBUZ_IssUnstartPlsSwitchToGearN]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boBuzFuncValid[nBUZ_IssUnstartPlsStartEngineByHand]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boBuzFuncValid[nBUZ_IssHasStop]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boBuzFuncValid[nBUZ_IssHasStart]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2?0:1);\
	FISD_boBuzFuncValid[nBUZ_ClothesHooked]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_TJA_ICA_Stop]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_TJA_ICA_On]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_TJA_ICA_FailActive]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_TJA_ICA_Demand]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_CareForRearCar]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ELKOncoming]=(BOOL)((Rte_FISD_u8GetCONFIG_TIEJ() == 0)&&(Rte_FISD_u8GetCONFIG_DrivingPowerType()!=nPowerType_EV));\
	FISD_boBuzFuncValid[nBUZ_AccActiveFalse]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_AccStandWait]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_AebSwitchOff_Textinfo]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_AebSwitchOn]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_AebInactive]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_FcwSwitchOff]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_FcwSwitchOn]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_FcwActiveFalse]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ASUFault]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ACC_Unused]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_AEB_Unused]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_FCW_Unused]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_CP_Unsed]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_ChangLaneUnableCuzSpd]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_NotTime2ChangeL]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_AEB_Off]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_AEB_On]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_ACC_Error]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_Turn2LeftLane]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boBuzFuncValid[nBUZ_Turn2RightLane]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boBuzFuncValid[nBUZ_FCW_Off]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_FCW_On]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_CP_Error]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_ACC_Active]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_CP_Active]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_DCLC_BadState]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boBuzFuncValid[nBUZ_CPMod_StopSafety]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boBuzFuncValid[nBUZ_AttentionVehiclesRight]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boBuzFuncValid[nBUZ_AttentionVehiclesLeft]=(BOOL)(1==can_diag_get_sw_conf(CONF_IDX_IDCU));\
	FISD_boBuzFuncValid[nBUZ_ProhibStandAndPark]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_AustralianNoOvertak]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_AustralianProhibitStandAndPark]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_AustralianNoPark]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_MalaysiaNoOvertak]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_OffRoadWork]=(BOOL)(Rte_FISD_u8GetCONFIG_M1E() == 0);\
	FISD_boBuzFuncValid[nBUZ_NoOvertake]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_EndOfNoOvertake]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_DOW_CloseDoor]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ElkSafty_Always]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_SBW_Psngr_TTS]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_BSD_1]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_BSD_2]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_HoodOpen_TTS]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_TrunkOpen_TTS]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_TwofeetBrake]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_REARCHILDDET_SEATBELT]=(BOOL)(0u);\
	FISD_boBuzFuncValid[nBUZ_RememberKeepSet_failure]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_GearLeverFailurePleaseRepairInTime]=(BOOL)((Rte_FISD_u8GetCONFIG_GEAR_STRUCTURE() == 1)?TRUE:FALSE);\
	FISD_boBuzFuncValid[nBUZ_PleaseShiftTheGearLeverToTheRight]=(BOOL)((Rte_FISD_u8GetCONFIG_GEAR_STRUCTURE() == 1)?TRUE:FALSE);\
	FISD_boBuzFuncValid[nBUZ_BatPowerLow]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boBuzFuncValid[nBUZ_BatSeriousLoss]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boBuzFuncValid[nBUZ_CheckRGC]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2);\
	FISD_boBuzFuncValid[nBUZ_CloseGasCapAlways]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2);\
	FISD_boBuzFuncValid[nBUZ_Turnoff_Ign]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV);\
	FISD_boBuzFuncValid[nBUZ_Check_BMS]=(BOOL)(((Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2) && (Rte_FISD_u8GetCONFIG_DrivingPowerType() == nPowerType_PHEV))||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boBuzFuncValid[nBUZ_BAT_OverHeatAndGetAway]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boBuzFuncValid[nBUZ_Check_eMcu]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boBuzFuncValid[nBUZ_Charging_Failure]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boBuzFuncValid[nBUZ_DisCharging_Failure]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boBuzFuncValid[nBUZ_SevereHigVolFault]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boBuzFuncValid[nBUZ_BAT_Temp_Over_High]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boBuzFuncValid[nBUZ_BAT_Temp_Over_Low]=(BOOL)((Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2)||Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_EV);\
	FISD_boBuzFuncValid[nBUZ_WarmerRemind]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boBuzFuncValid[nBUZ_EnterRegenerationSetting]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boBuzFuncValid[nBUZ_ReadyForRefuel]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1 && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2);\
	FISD_boBuzFuncValid[nBUZ_CompleteForRefuel]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1 && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2);\
	FISD_boBuzFuncValid[nBUZ_PT_Ready]=(BOOL)(0u);\
	FISD_boBuzFuncValid[nBUZ_ShiftGear2Right]=(BOOL)((Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV) && (Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2));\
	FISD_boBuzFuncValid[nBUZ_Unable2ChangeMode]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 1||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boBuzFuncValid[nBUZ_RegenerateLevelLow]=(BOOL)((Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2)||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boBuzFuncValid[nBUZ_RegenerateLevelMid]=(BOOL)((Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2)||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boBuzFuncValid[nBUZ_RegenerateLevelHigh]=(BOOL)((Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2)||Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boBuzFuncValid[nBUZ_LimitSpeedCancel]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boBuzFuncValid[nBUZ_DrvAbnormStAla_1]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_DrvAbnormStAla_2]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_DrvAbnormStAla_3]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_DrvAbnormStAla_4]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_DrvAbnormStAla_5]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_DrvAbnormStAla_6]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_DrvAbnormStAla_7]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_DMSSts]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_DrvCameraSts]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_CloseLight]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_VehicleSystemFailure]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== 2);\
	FISD_boBuzFuncValid[nBUZ_SCFSpdlimWarn]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_SLASpdlimitWarningAud]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ISA_SCF_PopoverReq]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ThirdRowSBW]=(BOOL)(Rte_FISD_u8GetCONFIG_ThdSeatBeltFunc() == 1u);\
    FISD_boBuzFuncValid[nBUZ_TransLimitedFunction]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boBuzFuncValid[nBUZ_TransHighTemperature]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boBuzFuncValid[nBUZ_ClutchHighTemperature]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boBuzFuncValid[nBUZ_ClutchSelfLearning]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boBuzFuncValid[nBUZ_HcuBrkPsd]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
	FISD_boBuzFuncValid[nBUZ_SLASpdLimChgAud]=(BOOL)(1u);\
	FISD_boBuzFuncValid[nBUZ_ELK_EV]=(BOOL)((Rte_FISD_u8GetCONFIG_TIEJ() == 1)||(Rte_FISD_u8GetCONFIG_DrivingPowerType()==nPowerType_EV));\
    FISD_boBuzFuncValid[nBUZ_Check_BMS_PHEV3]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
	FISD_boBuzFuncValid[nBUZ_Check_eMcu_PHEV3]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boBuzFuncValid[nBUZ_LowBatteryInfo]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boBuzFuncValid[nBUZ_LowBatteryLowFuelInfo]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boBuzFuncValid[nBUZ_HVILSts]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boBuzFuncValid[nBUZ_HighTempLimit]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boBuzFuncValid[nBUZ_ChrgFillerAjar]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boBuzFuncValid[nBUZ_FuelTankLidSts]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
	FISD_boBuzFuncValid[nBUZ_FuelTankLidSystemFailureSts]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
	FISD_boBuzFuncValid[nBUZ_EDrvOnly]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
	FISD_boBuzFuncValid[nBUZ_VehStrtWarn]=(BOOL)(Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3);\
    FISD_boBuzFuncValid[nBUZ_CPCancelled_TTS]=(BOOL)(1u);\
    FISD_boBuzFuncValid[nBUZ_NocGoingLeft_TTS]=(BOOL)(1u);\
    FISD_boBuzFuncValid[nBUZ_NocGoingRight_TTS]=(BOOL)(1u);\
    FISD_boBuzFuncValid[nBUZ_ELK_EV_1]=(BOOL)((Rte_FISD_u8GetCONFIG_TIEJ() == 1)||(Rte_FISD_u8GetCONFIG_DrivingPowerType()==nPowerType_EV));\
	}while(0);
}
BOOL FISD_FuncDICValid(U16 id)
{
	return FISD_boDicFuncValid[id];
}
BOOL FISD_FuncBuzValid(U16 id)
{
	return FISD_boBuzFuncValid[id];
}
/*INDENT-ON*/
/*******************************************************************************************
* Function: FISD_vGetSignalValue
* Description: Get signal value from other modules
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vGetSignalValue(void)
{
    U8 u8Num = 0u;

    /*Get CAN signal value*/
    for(u8Num = 0u; u8Num < nSigMaxNum; u8Num++)
    {
        if(TRUE == FISD_stRxSignalState[u8Num].boGetTimeoutFlag())
        {
            /*Signal timeout*/
            if(nDefault == FISD_stRxSignalState[u8Num].enGetValueMode)
            {
                FISD_u16SignalValue[u8Num] = FISD_stRxSignalState[u8Num].u16Default; /*Default value*/
            }
            else
            {
                /*Remain last value*/
            }
        }
        else
        {
            FISD_u16SignalValue[u8Num] = FISD_stRxSignalState[u8Num].u16GetSigValue(); /*Get the signal value*/
        }
    }
}

/*******************************************************************************************
* Function: FISD_vCrankOnDetect
* Description: if true when keysts is Crank on, until keysts change to other value for 3s
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vCrankOnDetect(void)
{
    static U16 u16CrankOnEndCnt = 0u;
    U8 u8KeySts = ((FALSE == IlGetBCM_4_KeyStsRxTimeout()) ? IlGetRxBCM_4_KeySts() : 0u);

    if(u8KeySts == 3u)
    {
        FISD_boIsCrankOn = True;
        u16CrankOnEndCnt = 0u;
    }
    else
    {
        if(FISD_boIsCrankOn == True)
        {
            if(u16CrankOnEndCnt < TIMECOUNT_3s)
            {
                u16CrankOnEndCnt++;
            }
            else
            {
                FISD_boIsCrankOn = False;
            }
        }
    }
}

/*******************************************************************************************
* Function: FISD_vIgnOnTimeCount
* Description: count the time from Ignition on, the max time is 600s
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vIgnOnTimeCount(void)
{
    
	
    // TODO:
    //U8 u8SocSts = SYN_SOC_WorkingStateGet();
	//static BOOL boCheckStartflag = FALSE;
    static BOOL boCheckEndFlag = FALSE;
    U8 u8KeySts = ((FALSE == IlGetBCM_4_KeyStsRxTimeout()) ? IlGetRxBCM_4_KeySts() : 0u);
    if(TRUE == FISD_enIgnState)
    {
        /*
        if(u8SocSts == SOC_WORKING_SELF_CHECK_START)
    	{
			boCheckStartflag = TRUE;
		}
		*/
        if(FISD_u16IgnOnTimerS < TIMECOUNT_600s) /*Ignition on count, when reach 600s, stop time*/
        {
            FISD_u16IgnOnTimerS++;
        }

        FISD_u16IgnOffTimerS = 0u;
#ifdef G6SH_TBD
        if(TRUE == Rte_FISD_boAnimationPlayIsFinish() && HMI_WORK_NORMALLY == Menu_u8GetCurrentHmiState() && \
                (FALSE == Rte_FISD_boGetEngineSpeedValid() || FISD_u16SelfCheckEngineSpeedThreshold >= Rte_FISD_u16GetEngineSpeedValue()))
        {
        	
            FISD_boSelfCheckFlag = TRUE;
        }
        else
        {
            FISD_boSelfCheckFlag = FALSE;
        }
#else // test self check code
		if(boCheckEndFlag != TRUE)
		{
			if(TRUE == Rte_FISD_boSelfCheckConditionCheck())
			{
		        if(/*(TRUE == Rte_FISD_boSelfCheckConditionCheck())&&*/\
		        	(FALSE == Rte_FISD_boGetEngineSpeedValid() || FISD_u16SelfCheckEngineSpeedThreshold >= Rte_FISD_u16GetEngineSpeedValue()))
		        {
		            FISD_boSelfCheckFlag = TRUE;
		        }
		        else
		        {
		            FISD_boSelfCheckFlag = FALSE;
	                boCheckEndFlag = TRUE;
		        }
			}
		}
#endif
        if(u8KeySts == 0x03u)
        {
            FISD_boSelfCheckFlag = FALSE;
            boCheckEndFlag = TRUE;
        }
    }
    else if(FALSE == FISD_enIgnState)
    {
        if(FISD_u16IgnOffTimerS < TIMECOUNT_600s) /*Ignition off count, when reach 600s, stop time*/
        {
            FISD_u16IgnOffTimerS ++;
        }
        FISD_u16IgnOnTimerS = 0u;
        FISD_boSelfCheckFlag = FALSE;
        boCheckEndFlag = FALSE;
    }
    else
    {
    }
}
static void FISD_vCoolantStatusClear(void);
/*******************************************************************************************
* Function: FISD_vFaultIndicatorDetect
* Description: detect fault and indicator state
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vFaultIndicatorDetect(void)
{
    /*Location: Column D of FISD_ProcessFunc sheet in T18_Configuration_Tool(FISD_SoundMgr_SYN_TextMgr).xlsx*/
    FISD_vTurnStateDetect();
    FISD_vLittleLampDetect();
    FISD_vHighBeamDetect();
    FISD_vHmaDetect();	//自动大灯&自适应远光
    FISD_vRearFogDetect();
    FISD_vDrlDetect();
    FISD_vSeatbeltDetect();
    FISD_vRearSeatbeltDetect();
	FISD_vThirdRowSeatbeltDetect();
    FISD_vEpcIndicatorDetect();
    FISD_vMilLampDetect();
    FISD_vEpbAndAutoHoldDetect();
    FISD_vCharger_IemDetect();
    FISD_vBrakeFluidLevelLow_EbdFaultDetect();
    FISD_vAbsFaultDetect();
    FISD_vEspFaultShieldDetect();
    FISD_vAirbagDetect();
    FISD_vTyrePressureDetect();
    FISD_vTransmissionFaultDetect();
    FISD_vEpsFaultDetect();
    FISD_vCoolantStatusDetect();
    FISD_vEngineOilPressureDetect();
    FISD_vFuelLevelLowDetect();
    FISD_vHdcDetect();
    FISD_vStartStopSysDetect();
    FISD_vLdwLdpEklDetect();
    FISD_vTakeOverDetect();
    FISD_vNotParkingDetect();
    FISD_vBsdAndRctaDetect();
    FISD_vSlaDetect();//限速辅助
    if(Rte_FISD_u8GetCONFIG_ISA() == 1u)
    {
    	if(can_diag_get_sw_conf(CONF_IDX_IntelligentSpeedAssistance) == nIntelligentSpeedAssistance_Present)
    	{
    		FISD_vIsaIsaDetect();//限速辅助-ISA-20230914
    	}
		else
		{
        	FISD_vIsaDetect();//限速辅助-ISA
		}
        FISD_vSLASpdLimChgAud();
    }
    FISD_vCruiseLimitControl();// 巡航&主动限速&定速巡航
    FISD_vAccAebFcwDetect();
	
    FISD_vTJA_ICADetect(); //交通拥堵辅助& 集成巡航辅助
    FISD_vDriveWorkModeDetect();
    FISD_vOverspeedDetect();
    FISD_vFatigueDetect();
    FISD_vFrontFogDetect();
    FISD_vESCLDetect();//电子转向柱
    FISD_vPepsWarningDetect();
    FISD_vFollowMeDetect();
    FISD_vDoorOpenDetect();
	FISD_vRadarDetect();
    FISD_vKeyInCarDetect();
    FISD_vRainDetect();
    FISD_vHhcDetect();//上坡辅助
    FISD_vAPADetect();
    FISD_vElecShiftFaultDetect();
    FISD_vAvmWarningDetect();//全景影像
    FISD_vWarnIconDetect();//报警图标
    FISD_vLanguageSetDetect();
    FISD_vLdwLkaSwitchDetect();
    FISD_vSeatRearmirrorDedect();
    FISD_vChildLockDedect();
    FISD_vRemoteStartModeDetect();
    FISD_vCarLampFaultDetect();
    FISD_vMemoryKeyStsDetect();
    FISD_vGpfFuncDetect();
    FISD_vBattEnergyWarningDetect();
    #if 0
    FISD_vTakeYourPhoneDetect();
    #endif
    FISD_vCameraShotDetect();
    FISD_vAWD_FaultDetect();
    FISD_vIceRoadDetect();
	FISD_vWashingWaterDetect();		//洗涤液位低
    FISD_vActiveSafetyBeltDetect();
    FISD_vHUDOverHotDetect();
    FISD_vClothesHookedDetect();
    FISD_vElectricDoorOpenDetect();	//电动门释放故障
    FISD_vCloseLightDetect();

	FISD_vPHEV_SOCDisp_Detect(); //PHEV相关功能
	FISD_vPHEV_LowSOCDetect();
	FISD_vPHEVSystemDetect();
    FISD_vPHEV_PowerModeDetect();
    FISD_vPHEV_eletricityModeDetect();
    FISD_vPHEV_RGCdetect();
    FISD_vPHEV_RegenerateLevelDetect();
    FISD_vPowerLimitDetect();
    FISD_vPHEV_AVAS_Detect();
    FISD_vPHEV_BMSH_InsulationStsDetect();
    FISD_vPHEV_TurnOffIgnDetect();
    FISD_vPHEV_BATPower_Red_Detect();
    FISD_vPHEV_ChargingStsDetect();
    FISD_vPHEV_DisChargeFaultDetect(); 
    FISD_vPHEV_WarmerRemindDetect();
    FISD_vPHEV_HvSysFltStopReq_Detect();
    FISD_vPHEV_BatteryTempLightSts_Detect();
	FISD_vPHEV3_LowBatteryInfo_Detect();
	FISD_vPHEV3_HighTempLimit_Detect();
	FISD_vPHEV3_ChrgFillerAjar_Detect();
	FISD_vPHEV3_EDrvOnly_Detect();
	FISD_vPHEV3_ForcedPrkgChrg_Detect();
	FISD_vPHEV3_EngSelfMaiTin_Detect();
	FISD_vPHEV3_VehStrtWarn_Detect();

    FISD_vEV_VehicleSystemFailure_Detect();//EV相关功能
    FISD_vEV_InvldCdnToDrvr_Detect();
    FISD_vEV_LowSOCCLMLimitSts_Detect();
    FISD_vEV_TowMode_Detect();
    FISD_vEV_V2LFuncSts_Detect();
    FISD_vEV_VCU_HVReady_Detect();
    FISD_vEV_TMF_MILSts_Detect();
    FISD_vEV_BMSH_PreWarmDis_Detect();
	FISD_vEV_TrailerConnectSts_Detect();
  
    if(can_diag_get_sw_conf(CONF_IDX_IDCU) == TRUE)  //T1CFL没有该需求
    {
    	FISD_vSmartADASDetect();  //IDCU_12  高阶ADAS
		FISD_vPilotDetect(); 	  //IDCU_12  高阶ADAS （  T1CFL没有该需求，但需要自检）
    }
    
	FISD_vMaintainDetect();
	FISD_vBatLow_Detect();
	FISD_vSlaveVehModDetect();  //整车模式
	FISD_vASUFaultDetect();     //ASU减震器故障
	if((1 == can_diag_get_sw_conf(CONF_IDX_DMS)) &&\
        (0 == can_diag_get_sw_conf(CONF_IDX_Fatiguemonitoring)) &&\
        (0 == can_diag_get_sw_conf(CONF_IDX_CVBOX)))
	{
		FISD_vOUTDMS_TipsDetect();
	}
	if((0 == can_diag_get_sw_conf(CONF_IDX_DMS)) &&\
        (1 == can_diag_get_sw_conf(CONF_IDX_Fatiguemonitoring)) &&\
        (0 == can_diag_get_sw_conf(CONF_IDX_CVBOX)))
	{
		FISD_vINTDMS_TipsDetect();
	}
	FISD_vDoorSeatbeltSync();
	FISD_vDMS_TipsDtetct();
	
	if(Rte_FISD_u8GetCONFIG_T18P() == 0)
	{
		FISD_vIPB_WarnDtetct();
		FISD_vRearChildDetect();//后排儿童探测提醒
	}

	if(Rte_FISD_u8GetCONFIG_T18P() == 0)
	{
		FISD_vChildProtectionDetect();
	}
	
	FISD_vCrankOnAndSelfCheckDetect();  //用于自检前3s不显示部分报警，只能放在最后面执行
	FISD_vVINComparisonDetect();
}

/*******************************************************************************************
* Function: FISD_vLittleLampDetect
* Description: Little lamp detect
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nActive             1u

static void FISD_vLittleLampDetect(void)
{
    /*Clear Status*/
    IND_SET(nIND_LittleLamp, nInd_Off);

    /*Redetect Status*/
    if(FISD_nActive == FISD_u16SignalValue[nParkLightSts])
    {
        IND_SET(nIND_LittleLamp, nInd_On);
    }
}

#undef FISD_nActive

/*******************************************************************************************
* Function: FISD_vHighBeamDetect
* Description: Little lamp detect
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nActive  1u

static void FISD_vHighBeamDetect(void)
{
    /*Clear Status*/
    IND_SET(nIND_HighBeam, nInd_Off);

    /*Redetect Status*/
    if(TRUE == Rte_boGetHighBeamStsTimeout() && ((TRUE == Rte_FISD_boAnimationPlayIsFinish())||(boSystemOOMPartOnMode() == TRUE)))
    {
        IND_SET(nIND_HighBeam, nInd_On);
    }

    if(FISD_nActive == FISD_u16SignalValue[nHighBeamSts])
    {
        IND_SET(nIND_HighBeam, nInd_On);
    }
}

#undef FISD_nActive

/*******************************************************************************************
* Function: FISD_vSeatbeltDetect
* Description: SeatbeltDetect
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nFasten          0u
#define FISD_nUnfasten        2u
#define FISD_nSeatbeltUntieMaxCnt 28u
static void FISD_vSeatbeltBuzStatus(void);
static void FISD_vSeatbeltBuzDiagnose(void);

static void FISD_vSeatbeltDetect(void)
{
    BUZ_SET(nBUZ_SBW_Test, nBuz_Inactive);
    if((TRUE == Rte_FISD_boGetOemControlSoundSts()) && (1u == Rte_FISD_u32GetOemControlSoundData()))
    {
        FISD_vSeatbeltBuzDiagnose();
    }
    else
    {
        FISD_vSeatbeltBuzStatus();
    }
}

static void FISD_vSeatbeltBuzStatus(void)
{
	BOOL sys_fullon = boSystemOOMFullOnMode();
	U16 speedTemp = SpeedGauge_DisplayValueGet();
    IND_SET(nIND_Seatbelt, nInd_Off);
	IND_SET(nIND_PsngerSeatBlt, nInd_Off);
    IND_SET(nIND_DriverSeatbelt, nInd_Off);
    IND_SET(nIND_CopilotSeatbelt, nInd_Off);
	BUZ_SET(nBUZ_SBW, nBuz_Inactive);

	if(nRudder_Left == Rte_FISD_u8GetCONFIG_LeftAndRightRudderType())/*左舵*/
	{
	    if((nSeatbelt_Stateflow_Flash == sf_seatbelt_chart_Y.driver_ind_state) || (nSeatbelt_Stateflow_Flash == sf_seatbelt_chart_Y.passenger_ind_state))
	    {
	    	IND_SET(nIND_Seatbelt, nInd_Flash_1_5Hz);
	    }
	    else if((nSeatbelt_Stateflow_On == sf_seatbelt_chart_Y.driver_ind_state) || (nSeatbelt_Stateflow_On == sf_seatbelt_chart_Y.passenger_ind_state))
	    {
	    	IND_SET(nIND_Seatbelt, nInd_On);
	    }
	}
	else/*右舵*/
	{
		if(nSeatbelt_Stateflow_Flash == sf_seatbelt_chart_Y.driver_ind_state)
	    {
	    	IND_SET(nIND_Seatbelt, nInd_Flash_1_5Hz);
	    }
	    else if(nSeatbelt_Stateflow_On == sf_seatbelt_chart_Y.driver_ind_state)
	    {
	    	IND_SET(nIND_Seatbelt, nInd_On);
	    }

		if(nSeatbelt_Stateflow_Flash == sf_seatbelt_chart_Y.passenger_ind_state)
	    {
	    	IND_SET(nIND_PsngerSeatBlt, nInd_Flash_1_5Hz);
	    }
	    else if(nSeatbelt_Stateflow_On == sf_seatbelt_chart_Y.passenger_ind_state)
	    {
	    	IND_SET(nIND_PsngerSeatBlt, nInd_On);
	    }
	}

    if((!IND_CHK(nIND_Seatbelt, nInd_Off)) || (!IND_CHK(nIND_PsngerSeatBlt, nInd_Off)))
    {
    	if((2 == IlGetRxABM_1_PsngrSeatBeltWarning())&&\
            (FALSE==IlGetABM_1_PsngrSeatBeltWarningRxTimeout()))
	    {
	    	if((speedTemp >= 3) || \
			((FISD_u16SignalValue[nEPBWarningMessage] == 2) || \
			(FISD_u16SignalValue[nAVHWarningMessage] == 1)))
	    	{
	        	IND_SET(nIND_CopilotSeatbelt, nInd_Flash_1Hz);
	    	}
			else
			{
				IND_SET(nIND_CopilotSeatbelt, nInd_On);
			}
	    }

	    if(FALSE == Rte_FISD_enGetDriverSeatbeltBuckle())
	    {
	    	if((speedTemp >= 3) || \
			((FISD_u16SignalValue[nEPBWarningMessage] == 2) || \
			(FISD_u16SignalValue[nAVHWarningMessage] == 1)))
	    	{
	        	IND_SET(nIND_DriverSeatbelt, nInd_Flash_1Hz);
	    	}
			else
			{
				IND_SET(nIND_DriverSeatbelt, nInd_On);
			}
	    }
    }
	
	if(IlGetABM_1_PsngrSeatBeltWarningRxTimeout() == TRUE)
	{
		if(Rte_FISD_enGetDriverSeatbeltBuckle() == TRUE)
		{
			IND_SET(nIND_Seatbelt, nInd_Off);
			IND_SET(nIND_PsngerSeatBlt, nInd_Off);
		}
        IND_SET(nIND_CopilotSeatbelt, nInd_Off);
	}
    if(sf_seatbelt_chart_Y.front_seatbelt_buzzer_active)
    {
        BUZ_SET(nBUZ_SBW, nBuz_Active);
    }

    if((sf_seatbelt_chart_Y.front_seatbelt_text_active) && (sys_fullon == TRUE))
    {
        //DIC_SET(nDIC_BeltWarnFastenSeatbeltWithText, nDic_Active);
    }
    else
    {
    }


	
    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
    	IND_SET(nIND_Seatbelt, nInd_Off);
		IND_SET(nIND_PsngerSeatBlt, nInd_Off);
        BUZ_SET(nBUZ_SBW, nBuz_Inactive);
        DIC_SET(nDIC_DoorOpenWithText, nDic_Inactive);
    }
}

static void FISD_vSeatbeltBuzDiagnose(void)
{
    BUZ_SET(nBUZ_SBW_Test, nBuz_Active);
    DIC_SET(nDIC_BeltWarnFastenSeatbeltWithText, nDic_Active);
    DIC_SET(nDIC_DoorOpenWithText, nDic_Active);
}

/*******************************************************************************************
* Function: FISD_vRearSeatbeltDetect
* Description: RearSeatbeltDetect
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nFASTEN          0
#define FISD_nUNFASTEN     1

static void FISD_vClearRearSeatbeltSts(void);
static void FISD_vShowRearSeatbeltSts(void);
static void FISD_vClearThirdRowSeatbeltSts(void);
static void FISD_vShowThdSeatbeltSts(void);

static void FISD_vRearSeatbeltDetect(void)
{
	IND_SET(nIND_RearLeftSeatbelt,nInd_Off);
	IND_SET(nIND_RearMiddleSeatbelt,nInd_Off);
	IND_SET(nIND_RearRightSeatbelt,nInd_Off);  

	U16 speedTemp = SpeedGauge_DisplayValueGet();
	
    if(Rte_FISD_u8GetCONFIG_SECOND_ROW_SBR())
    {
    	FISD_vClearRearSeatbeltSts();

    	if(sf_seatbelt_chart_Y.RL_ind_state == nSeatbelt_Stateflow_Flash)
    	{
    		IND_SET(nIND_RearLeftSeatBelt_Red,nInd_Flash_1_5Hz);
    	}
    	else if(sf_seatbelt_chart_Y.RL_ind_state == nSeatbelt_Stateflow_On)
    	{

    		IND_SET(nIND_RearLeftSeatBelt_Red,nInd_On);
    	}
    	else
    	{
    		IND_SET(nIND_RL_SeatBelt_grey,nInd_On);
    	}

    	if(sf_seatbelt_chart_Y.RM_ind_state == nSeatbelt_Stateflow_Flash)
    	{
    		IND_SET(nIND_RearMidSeatBelt_Red,nInd_Flash_1_5Hz);
    	}
    	else if(sf_seatbelt_chart_Y.RM_ind_state == nSeatbelt_Stateflow_On)
    	{

    		IND_SET(nIND_RearMidSeatBelt_Red,nInd_On);
    	}
    	else
    	{
		    IND_SET(nIND_RM_SeatBelt_grey,nInd_On);
    	}

    	if(sf_seatbelt_chart_Y.RR_ind_state == nSeatbelt_Stateflow_Flash)
    	{
    		IND_SET(nIND_RearRightSeatBelt_Red,nInd_Flash_1_5Hz);
    	}
    	else if(sf_seatbelt_chart_Y.RR_ind_state == nSeatbelt_Stateflow_On)
    	{

    		IND_SET(nIND_RearRightSeatBelt_Red,nInd_On);
    	}
    	else
    	{
		    IND_SET(nIND_RR_SeatBelt_grey,nInd_On);
    	}

    	if((IND_CHK(nIND_RR_SeatBelt_grey, nInd_On)) && \
    	(IND_CHK(nIND_RM_SeatBelt_grey, nInd_On)) && \
    	(IND_CHK(nIND_RL_SeatBelt_grey, nInd_On)))
    	{
    		IND_SET(nIND_RL_SeatBelt_grey,nInd_Off);
		    IND_SET(nIND_RM_SeatBelt_grey,nInd_Off);
		    IND_SET(nIND_RR_SeatBelt_grey,nInd_Off);
    	}

    	if(sf_seatbelt_chart_Y.second_row_sound)
    	{
    		BUZ_SET(nBUZ_SBW,nBuz_Active);
    	}
    }
    else
    {   
	    IND_SET(nIND_RL_SeatBelt_grey,nInd_Off);
	    IND_SET(nIND_RM_SeatBelt_grey,nInd_Off);
	    IND_SET(nIND_RR_SeatBelt_grey,nInd_Off);

	    if(sf_seatbelt_chart_Y.rear_seatbelt_indicator_overall)
	    {
	        FISD_vShowRearSeatbeltSts();
	    }
	    else
	    {
	        // clear rear seatbelt indicators, REDs and GREENs
	        IND_SET(nIND_RearLeftSeatBelt_Red,nInd_Off);
	        IND_SET(nIND_RearLeftSeatBelt_Green,nInd_Off);

	        IND_SET(nIND_RearMidSeatBelt_Red,nInd_Off);
	        IND_SET(nIND_RearMidSeatBelt_Green,nInd_Off);

	        IND_SET(nIND_RearRightSeatBelt_Red,nInd_Off);
	        IND_SET(nIND_RearRightSeatBelt_Green,nInd_Off);
	    }

	    if(sf_seatbelt_chart_Y.rear_seatbelt_buzzer_active)
	    {
	        BUZ_SET(nBUZ_RearSBW, nBuz_Active);
	    }
	    else
	    {
	        BUZ_SET(nBUZ_RearSBW, nBuz_Inactive);
	    }
	}
    // text warning is not needed here since we process it in front-seatbelt process

	/*update the text window rear seatbelt indicator*/
	if(Rte_FISD_boGetPowerDependent() == TRUE)
	{
	    if(FISD_nUNFASTEN == FISD_u16SignalValue[nRearLeftPsngrSeatBeltSwitch])
	    {
			if((speedTemp >= 3) || \
			((FISD_u16SignalValue[nEPBWarningMessage] == 2) || \
			(FISD_u16SignalValue[nAVHWarningMessage] == 1)))
	    	{
	        	IND_SET(nIND_RearLeftSeatbelt, nInd_Flash_1Hz);
	    	}
			else
			{
				IND_SET(nIND_RearLeftSeatbelt, nInd_On);
			}
	    }

	    if(FISD_nUNFASTEN == FISD_u16SignalValue[nRearMiddlePsngrSeatBeltSwitch])
	    {
			if((speedTemp >= 3) || \
			((FISD_u16SignalValue[nEPBWarningMessage] == 2) || \
			(FISD_u16SignalValue[nAVHWarningMessage] == 1)))
	    	{
	        	IND_SET(nIND_RearMiddleSeatbelt, nInd_Flash_1Hz);
	    	}
			else
			{
				IND_SET(nIND_RearMiddleSeatbelt, nInd_On);
			}
	    }

	    if(FISD_nUNFASTEN == FISD_u16SignalValue[nRearRightPsngrSeatBeltSwitch])
	    {
			if((speedTemp >= 3) || \
			((FISD_u16SignalValue[nEPBWarningMessage] == 2) || \
			(FISD_u16SignalValue[nAVHWarningMessage] == 1)))
	    	{
	        	IND_SET(nIND_RearRightSeatbelt, nInd_Flash_1Hz);
	    	}
			else
			{
				IND_SET(nIND_RearRightSeatbelt, nInd_On);
			}
	    }
	}

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        FISD_vClearRearSeatbeltSts();
    }
}

static void FISD_vThirdRowSeatbeltDetect(void)
{
	IND_SET(nIND_ThdLeftSeatbelt,nInd_Off);
	IND_SET(nIND_ThdRightSeatbelt,nInd_Off); 
	
	U16 speedTemp = SpeedGauge_DisplayValueGet();

    if(Rte_FISD_u8GetCONFIG_THIRD_ROW_SBR())
    {
    	FISD_vClearThirdRowSeatbeltSts();

    	if(sf_seatbelt_chart_Y.ThdL_ind_state == nSeatbelt_Stateflow_Flash)
    	{
    		IND_SET(nIND_ThirdRowSB_Left_Red,nInd_Flash_1_5Hz);
    	}
    	else if(sf_seatbelt_chart_Y.ThdL_ind_state == nSeatbelt_Stateflow_On)
    	{

    		IND_SET(nIND_ThirdRowSB_Left_Red,nInd_On);
    	}
    	else
    	{
    		IND_SET(nIND_ThirdRowSB_Left_Grey,nInd_On);
    	}

    	if(sf_seatbelt_chart_Y.ThdR_ind_state == nSeatbelt_Stateflow_Flash)
    	{
    		IND_SET(nIND_ThirdRowSB_Right_Red,nInd_Flash_1_5Hz);
    	}
    	else if(sf_seatbelt_chart_Y.ThdR_ind_state == nSeatbelt_Stateflow_On)
    	{

    		IND_SET(nIND_ThirdRowSB_Right_Red,nInd_On);
    	}
    	else
    	{
		    IND_SET(nIND_ThirdRowSB_Right_Grey,nInd_On);
    	}

    	if((IND_CHK(nIND_ThirdRowSB_Left_Grey, nInd_On)) && \
    	(IND_CHK(nIND_ThirdRowSB_Right_Grey, nInd_On)))
    	{
    		IND_SET(nIND_ThirdRowSB_Left_Grey,nInd_Off);
		    IND_SET(nIND_ThirdRowSB_Right_Grey,nInd_Off);
    	}

    	if(sf_seatbelt_chart_Y.thd_row_sound)
    	{
    		BUZ_SET(nBUZ_SBW,nBuz_Active);
    	}
    }
    else
    {   
	    IND_SET(nIND_ThirdRowSB_Left_Grey,nInd_Off);
	    IND_SET(nIND_ThirdRowSB_Right_Grey,nInd_Off);

	    if(sf_seatbelt_chart_Y.thd_seatbelt_indicator_overall)
	    {
	        FISD_vShowThdSeatbeltSts();
	    }
	    else
	    {
	        // clear third seatbelt indicators, REDs and GREENs
	        IND_SET(nIND_ThirdRowSB_Left_Red,nInd_Off);
	        IND_SET(nIND_ThirdRowSB_Left_Green,nInd_Off);

	        IND_SET(nIND_ThirdRowSB_Right_Red,nInd_Off);
	        IND_SET(nIND_ThirdRowSB_Right_Green,nInd_Off);
	    }

	    if(sf_seatbelt_chart_Y.thd_seatbelt_buzzer_active)
	    {
	        BUZ_SET(nBUZ_ThirdRowSBW, nBuz_Active);
	    }
	    else
	    {
	        BUZ_SET(nBUZ_ThirdRowSBW, nBuz_Inactive);
	    }
	}
    // text warning is not needed here since we process it in front-seatbelt process

	/*update the text window rear seatbelt indicator*/
	if(Rte_FISD_boGetPowerDependent() == TRUE)
	{
		if(FISD_nUNFASTEN == FISD_u16SignalValue[nThdRowLBeltWarning])
		{
			if((speedTemp >= 3) || \
			((FISD_u16SignalValue[nEPBWarningMessage] == 2) || \
			(FISD_u16SignalValue[nAVHWarningMessage] == 1)))
	    	{
	        	IND_SET(nIND_ThdLeftSeatbelt, nInd_Flash_1Hz);
	    	}
			else
			{
				IND_SET(nIND_ThdLeftSeatbelt, nInd_On);
			}
		}

		if(FISD_nUNFASTEN == FISD_u16SignalValue[nThdRowRBeltWarning])
		{
			if((speedTemp >= 3) || \
			((FISD_u16SignalValue[nEPBWarningMessage] == 2) || \
			(FISD_u16SignalValue[nAVHWarningMessage] == 1)))
	    	{
	        	IND_SET(nIND_ThdRightSeatbelt, nInd_Flash_1Hz);
	    	}
			else
			{
				IND_SET(nIND_ThdRightSeatbelt, nInd_On);
			}
		}
	}

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        FISD_vClearThirdRowSeatbeltSts();
    }
}




static void FISD_vShowRearSeatbeltSts(void)
{
    if(FISD_nUNFASTEN == FISD_u16SignalValue[nRearLeftPsngrSeatBeltSwitch])
    {
        IND_SET(nIND_RearLeftSeatBelt_Red,nInd_On);
        IND_SET(nIND_RearLeftSeatBelt_Green,nInd_Off);
    }
    else if(FISD_nFASTEN == FISD_u16SignalValue[nRearLeftPsngrSeatBeltSwitch])
    {
        IND_SET(nIND_RearLeftSeatBelt_Red,nInd_Off);
        IND_SET(nIND_RearLeftSeatBelt_Green,nInd_On);
    }

    if(FISD_nUNFASTEN == FISD_u16SignalValue[nRearMiddlePsngrSeatBeltSwitch])
    {
        IND_SET(nIND_RearMidSeatBelt_Red,nInd_On);
        IND_SET(nIND_RearMidSeatBelt_Green,nInd_Off);
    }
    else if(FISD_nFASTEN == FISD_u16SignalValue[nRearMiddlePsngrSeatBeltSwitch])
    {
        IND_SET(nIND_RearMidSeatBelt_Red,nInd_Off);
        IND_SET(nIND_RearMidSeatBelt_Green,nInd_On);
    }

    if(FISD_nUNFASTEN == FISD_u16SignalValue[nRearRightPsngrSeatBeltSwitch])
    {
        IND_SET(nIND_RearRightSeatBelt_Red,nInd_On);
        IND_SET(nIND_RearRightSeatBelt_Green,nInd_Off);
    }
    else if(FISD_nFASTEN == FISD_u16SignalValue[nRearRightPsngrSeatBeltSwitch])
    {
        IND_SET(nIND_RearRightSeatBelt_Red,nInd_Off);
        IND_SET(nIND_RearRightSeatBelt_Green,nInd_On);
    }
}

static void FISD_vClearRearSeatbeltSts(void)
{
    IND_SET(nIND_RearLeftSeatBelt_Red,nInd_Off);
    IND_SET(nIND_RearLeftSeatBelt_Green,nInd_Off);

    IND_SET(nIND_RearMidSeatBelt_Red,nInd_Off);
    IND_SET(nIND_RearMidSeatBelt_Green,nInd_Off);

    IND_SET(nIND_RearRightSeatBelt_Red,nInd_Off);
    IND_SET(nIND_RearRightSeatBelt_Green,nInd_Off);

    IND_SET(nIND_RL_SeatBelt_grey,nInd_Off);
    IND_SET(nIND_RM_SeatBelt_grey,nInd_Off);
    IND_SET(nIND_RR_SeatBelt_grey,nInd_Off);

    BUZ_SET(nBUZ_RearSBW, nBuz_Inactive);
}

static void FISD_vShowThdSeatbeltSts(void)
{
    if(FISD_nUNFASTEN == FISD_u16SignalValue[nThdRowLBeltWarning])
    {
        IND_SET(nIND_ThirdRowSB_Left_Red,nInd_On);
        IND_SET(nIND_ThirdRowSB_Left_Green,nInd_Off);
    }
    else if(FISD_nFASTEN == FISD_u16SignalValue[nThdRowLBeltWarning])
    {
        IND_SET(nIND_ThirdRowSB_Left_Red,nInd_Off);
        IND_SET(nIND_ThirdRowSB_Left_Green,nInd_On);
    }

    if(FISD_nUNFASTEN == FISD_u16SignalValue[nThdRowRBeltWarning])
    {
        IND_SET(nIND_ThirdRowSB_Right_Red,nInd_On);
        IND_SET(nIND_ThirdRowSB_Right_Green,nInd_Off);
    }
    else if(FISD_nFASTEN == FISD_u16SignalValue[nThdRowRBeltWarning])
    {
        IND_SET(nIND_ThirdRowSB_Right_Red,nInd_Off);
        IND_SET(nIND_ThirdRowSB_Right_Green,nInd_On);
    }
}

static void FISD_vClearThirdRowSeatbeltSts(void)
{
    IND_SET(nIND_ThirdRowSB_Left_Green,nInd_Off);
	IND_SET(nIND_ThirdRowSB_Left_Red,nInd_Off);
	
	IND_SET(nIND_ThirdRowSB_Right_Green,nInd_Off);
	IND_SET(nIND_ThirdRowSB_Right_Red,nInd_Off);

	IND_SET(nIND_ThirdRowSB_Left_Grey,nInd_Off);
	IND_SET(nIND_ThirdRowSB_Right_Grey,nInd_Off);
	
	BUZ_SET(nBUZ_ThirdRowSBW, nBuz_Inactive);
}

/*******************************************************************************************
* Function: FISD_vEpcIndicatorDetect
* Description: EpcIndicatorDetect
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nActive      1u
static void FISD_vEpcIndicatorDetect(void)
{
    IND_SET(nIND_EpcIndicator, nInd_Off);

    if((FISD_nActive == FISD_u16SignalValue[nEPCSts]))
    {
        IND_SET(nIND_EpcIndicator, nInd_On);
    }

    if((TRUE == Rte_boGetEPCStsTimeout()))
    {
        IND_SET(nIND_EpcIndicator, nInd_On);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        IND_SET(nIND_EpcIndicator, nInd_Off);
    }
}
#undef FISD_nActive

/*******************************************************************************************
* Function: FISD_vMilLampDetect
* Description: MilLampDetect
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nActive   2u
static void FISD_vMilLampDetect(void)
{
    IND_SET(nIND_MIL_Lamp, nInd_Off);

    if((FISD_nActive == FISD_u16SignalValue[nMILSts]))
    {
        IND_SET(nIND_MIL_Lamp, nInd_On);
    }

    if((TRUE == Rte_boGetMILStsTimeout()))
    {
        IND_SET(nIND_MIL_Lamp, nInd_On);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        IND_SET(nIND_MIL_Lamp, nInd_Off);
    }
}
#undef FISD_nActive

/*******************************************************************************************
* Function: FISD_vEpbAndAutoHoldDetect
* Description: EpbAndAutoHoldDetect
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nActive            1u
static void FISD_vHandbrakeNotReleaseDetect(void);
static void FISD_vEpbWorkDetect(void);
static void FISD_vEpbFaultDetect(void);
static void FISD_vCdpWorkDetect(void);
static void FISD_vAutoHoldDetect(void);

static void FISD_vEpbAndAutoHoldDetect(void)
{
	DIC_SET(nDIC_ReleaseHandBrake, nDic_Inactive);
	BUZ_SET(nBUZ_HandBrakeNotRelease, nBuz_Inactive);
	
    FISD_vHandbrakeNotReleaseDetect();    
    FISD_vEpbWorkDetect();
    FISD_vEpbFaultDetect();
    FISD_vCdpWorkDetect();

    if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
    {
    	if(Rte_FISD_u8GetCONFIG_EPB() == TRUE)
    	{
        	IND_SET(nIND_EpbFault_Yellow_P, nInd_On);
    	}
        FISD_aenIndicatorState[nIND_HandBrake_EpbWork_Red_P] = nInd_On;
    }
    FISD_vAutoHoldDetect();
}

static void FISD_vHandbrakeNotReleaseDetect(void)
{
#define HANDBRAKE_IO_DEBOUNCE_TIMER_SETTING  15
    static U8 handbrake_io_debounce_timer = 0;

    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
    BOOL sys_parton = boSystemOOMPartOnMode();

    IND_SET(nIND_HandBrake_EpbWork_Red_P, nInd_Off);
    IND_SET(nIND_EpbFault_Yellow_P, nInd_Off);
    /*Redetect Status*/
    if((FALSE == Rte_FISD_enGetHandbrakeSwitch())&&sys_fullon)
    {
        if(handbrake_io_debounce_timer<HANDBRAKE_IO_DEBOUNCE_TIMER_SETTING)
        {
            handbrake_io_debounce_timer++;
        }
        if(handbrake_io_debounce_timer>=HANDBRAKE_IO_DEBOUNCE_TIMER_SETTING)
        {
            if(FISD_u16VehicleSpeed >= FISD_u16HandbrakeNotReleaseOnSpeed)
            {
                FISD_aenIndicatorState[nIND_HandBrake_EpbWork_Red_P] = nInd_Flash_1Hz;
                BUZ_SET(nBUZ_HandBrakeNotRelease, nBuz_Active);
                DIC_SET(nDIC_ReleaseHandBrake, nDic_Active);
            }
            else
            {
                if(sys_fullon||sys_parton)
                {
                    FISD_aenIndicatorState[nIND_HandBrake_EpbWork_Red_P] = nInd_On;
                    BUZ_SET(nBUZ_HandBrakeNotRelease, nBuz_Inactive);
                    DIC_SET(nDIC_ReleaseHandBrake, nDic_Inactive);
                }
            }
        }
    }
    else
    {
        handbrake_io_debounce_timer = 0;
        FISD_aenIndicatorState[nIND_HandBrake_EpbWork_Red_P] = nInd_Off;
        BUZ_SET(nBUZ_HandBrakeNotRelease, nBuz_Inactive);
        DIC_SET(nDIC_ReleaseHandBrake, nDic_Inactive);
    }

#ifdef ENABLE_TEXT_SOUND_MIN_SHOW_2S
    /*deal with the sound warning connect to textwarn which is cancelled in 2s.*/
    if(FISD_u16VehicleSpeed >= FISD_u16HandbrakeNotReleaseOnSpeed)
    {
        if(nDIC_ReleaseHandBrake == TextMgr_enGetDisplayText())
        {
            BUZ_SET(nBUZ_HandBrakeNotRelease,nBuz_Active);
        }
    }
#endif
    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        FISD_aenIndicatorState[nIND_HandBrake_EpbWork_Red_P] = nInd_Off;
        BUZ_SET(nBUZ_HandBrakeNotRelease, nBuz_Inactive);
        DIC_SET(nDIC_ReleaseHandBrake, nDic_Inactive);
    }
}

static void FISD_vEpbWorkDetect(void)
{
    BOOL sys_fullon = boSystemOOMFullOnMode();
    BOOL sys_parton = boSystemOOMPartOnMode();

 	if((FISD_u16SignalValue[nRedwarning]==2)&&(sys_fullon || sys_parton))
    {
        IND_SET(nIND_HandBrake_EpbWork_Red_P, nInd_On);
    }
    else if((FISD_u16SignalValue[nRedwarning] == 1)&&(sys_fullon || sys_parton))
    {
        IND_SET(nIND_HandBrake_EpbWork_Red_P, nInd_Flash_1Hz);
    }
    else
    {
    }
	if((TRUE == Rte_boGetEPBWarningMessageTimeout())&&(sys_fullon || sys_parton))
	{
		if(TRUE == Rte_FISD_enGetHandbrakeSwitch())
		{
			IND_SET(nIND_HandBrake_EpbWork_Red_P, nInd_Off);
		}
	}
}

static void FISD_vEpbFaultDetect(void)
{
    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
    BOOL sys_parton = boSystemOOMPartOnMode();

    IND_SET(nIND_EpbFault_Yellow_P, nInd_Off);

    if((FISD_u16SignalValue[nYellowwarning]==1)&&sys_fullon)
    {
        IND_SET(nIND_EpbFault_Yellow_P, nInd_Flash_1Hz);
    }
    else if((FISD_u16SignalValue[nYellowwarning]==2)&&sys_fullon)
    {
        IND_SET(nIND_EpbFault_Yellow_P, nInd_On);
    }
    else
    {
        IND_SET(nIND_EpbFault_Yellow_P, nInd_Off);
    }

    if((TRUE == Rte_boGetEPBWarningMessageTimeout())&&sys_fullon)
    {
    	IND_SET(nIND_EpbFault_Yellow_P, nInd_Off);
		BUZ_SET(nBUZ_StepBrakePedalReleaseHandbrake, nBuz_Inactive);
    	if(Rte_FISD_u8GetCONFIG_EPB() == TRUE)
    	{
        	IND_SET(nIND_EpbFault_Yellow_P, nInd_On);
    	}	
    }

    /*EPBWarningMassage*/
    DIC_SET(nDIC_StepBrakePedalReleaseHandbrake, nDic_Inactive);
	BUZ_SET(nBUZ_StepBrakePedalReleaseHandbrake, nBuz_Inactive);
    DIC_SET(nDIC_CautionParkOnHighSlope, nDic_Inactive);

    if((FISD_u16SignalValue[nEPBWarningMessage] == 1u)&&sys_fullon)
    {
        DIC_SET(nDIC_StepBrakePedalReleaseHandbrake, nDic_Active);
		BUZ_SET(nBUZ_StepBrakePedalReleaseHandbrake, nBuz_Active);
    }
    else if((FISD_u16SignalValue[nEPBWarningMessage] == 3u)&&((TRUE == Rte_FISD_boAnimationPlayIsFinish())||sys_parton))
    {
        DIC_SET(nDIC_CautionParkOnHighSlope, nDic_Active);
    }
    else
    {
    }
	
    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        IND_SET(nIND_EpbFault_Yellow_P, nInd_Off);
        DIC_SET(nDIC_StepBrakePedalReleaseHandbrake, nDic_Inactive);
		BUZ_SET(nBUZ_StepBrakePedalReleaseHandbrake, nBuz_Inactive);
    }
}

static void FISD_vCdpWorkDetect(void)
{
    BOOL sys_fullon = boSystemOOMFullOnMode();
    BOOL sys_parton = boSystemOOMPartOnMode();

    BUZ_SET(nBUZ_CDP_RWU,nBuz_Inactive);
    if(FALSE == Rte_FISD_boIsStayInAnimationPlayStatus())
    {
        if((FISD_u16SignalValue[nRWUActive] == 0u) && (FISD_u16SignalValue[nCDPActive] == 0u))
        {
            if((FISD_u16SignalValue[nRedwarning] == 2u)&&(sys_fullon||sys_parton))
            {
                IND_SET(nIND_HandBrake_EpbWork_Red_P, nInd_On);
            }
            else if((FISD_u16SignalValue[nRedwarning] == 1u)&&(sys_fullon||sys_parton))
            {
                IND_SET(nIND_HandBrake_EpbWork_Red_P, nInd_Flash_1Hz);
            }
        }
        else if((((FISD_u16SignalValue[nRWUActive] == 0) && (FISD_u16SignalValue[nCDPActive] == 1)) || \
                 ((FISD_u16SignalValue[nRWUActive] == 1) && (FISD_u16SignalValue[nCDPActive] == 0)) ||\
                 ((FISD_u16SignalValue[nRWUActive] == 1) && (FISD_u16SignalValue[nCDPActive] == 1)) )&&\
                (sys_fullon||sys_parton))
        {
            IND_SET(nIND_HandBrake_EpbWork_Red_P, nInd_Flash_1Hz);
            BUZ_SET(nBUZ_CDP_RWU,nBuz_Active);
        }
        else
        {
        }
		if((Rte_boGetRWUActiveTimeout() == TRUE)&&(Rte_boGetCDPActiveTimeout() == TRUE))
		{
			BUZ_SET(nBUZ_CDP_RWU,nBuz_Inactive);
		}
    }
}

static void FISD_vAutoHoldDetect(void)
{
    static U16 AutoHoldActive_Time = 0u;
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    BOOL avh_ind_condition = TRUE;
	//0224 自动驻车报警增加超时判断
	avh_ind_condition = (FALSE ==Rte_boGetAVHStsTimeout());
	
    /*Clear Status*/
    IND_SET(nIND_AutoHold_Green_P, nInd_Off);
	IND_SET(nIND_AutoHold_White_P, nInd_Off);
	IND_SET(nIND_AutoHold_Yellow_A, nInd_Off);
    DIC_SET(nDIC_AutoHoldFailure, nDic_Inactive);
    DIC_SET(nDIC_CDP_Failure, nDic_Inactive);
	if(FISD_u16SignalValue[nAVHSts] == 1u)
	{
		IND_SET(nIND_AutoHold_White_P, nInd_On);
	}
	if(TRUE == Rte_FISD_boAnimationPlayIsFinish())
	{
	    if((FISD_u16LastAVHSts == 0u) &&
	            ((FISD_u16SignalValue[nAVHSts] == 1u) || (FISD_u16SignalValue[nAVHSts] == 2u))&&\
	            avh_ind_condition&&sys_fullon)
	    {
	        DIC_SET(nDIC_AutoHoldActive, nDic_Active);
			BUZ_SET(nBUZ_AutoHoldActive, nBuz_Active);
			if(FISD_u16SignalValue[nAVHSts] == 1u)
			{
				//IND_SET(nIND_AutoHold_White_P, nInd_On);
			}
	        AutoHoldActive_Time = TIMECOUNT_3s;
	    }
	    else if((FISD_u16SignalValue[nAVHSts] == 0u) || \
	            (FISD_u16SignalValue[nAVHSts] == 3u) || \
	            ((FISD_u16LastAVHSts == 2u) && (FISD_u16SignalValue[nAVHSts] == 1u)) || \
	            ((FISD_u16LastAVHSts == 3u) && (FISD_u16SignalValue[nAVHSts] == 1u))|| \
	            ((FISD_u16LastAVHSts == 1u) && (FISD_u16SignalValue[nAVHSts] == 2u)) || \
	            ((FISD_u16LastAVHSts == 3u) && (FISD_u16SignalValue[nAVHSts] == 2u)))
	    {
	        DIC_SET(nDIC_AutoHoldActive, nDic_Inactive);
			BUZ_SET(nBUZ_AutoHoldActive, nBuz_Inactive);
			//IND_SET(nIND_AutoHold_White_P, nInd_Off);
	    }
	    else if(FISD_u16SignalValue[nAVHSts] == 2u)
	    {
	    	//IND_SET(nIND_AutoHold_White_P, nInd_Off);
	    }

	
    	FISD_u16LastAVHSts = FISD_u16SignalValue[nAVHSts];
	}

    if((FISD_u16SignalValue[nAVHSts] == 2u)&&avh_ind_condition&&sys_fullon)
    {
        IND_SET(nIND_AutoHold_Green_P, nInd_On);
    }	
	
    if((FISD_u16SignalValue[nAVHFailSts] == 1u)&&avh_ind_condition&&sys_fullon)
    {
        DIC_SET(nDIC_AutoHoldFailure, nDic_Active);
		IND_SET(nIND_AutoHold_Yellow_A, nInd_On);
		IND_SET(nIND_AutoHold_Green_P, nInd_Off);
		IND_SET(nIND_AutoHold_White_P, nInd_Off);
    }

    if((FISD_u16SignalValue[nCDPFailSts] == 1u)&&avh_ind_condition)
    {
        DIC_SET(nDIC_CDP_Failure, nDic_Active);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        IND_SET(nIND_AutoHold_Green_P, nInd_Off);
		IND_SET(nIND_AutoHold_White_P, nInd_Off);
        DIC_SET(nDIC_AutoHoldFailure, nDic_Inactive);
        DIC_SET(nDIC_CDP_Failure, nDic_Inactive);
        DIC_SET(nDIC_AutoHoldActive, nDic_Inactive);
		BUZ_SET(nBUZ_AutoHoldActive, nBuz_Inactive);
    }
}
#undef FISD_nActive


/*******************************************************************************************
* Function: FISD_vCharger_IemDetect
* Description: Charger_IemDetect
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nLampOn            1u
static void  FISD_vCharger_IemDetect(void)
{
#define ALTERNATOR_IO_DEBOUNCE_TIMER_SETTING  10
    static U8 alternator_io_debounce_timer = 0;
    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    IND_SET(nIND_Charger_Iem, nInd_Off);

    if((FISD_nLampOn == FISD_u16SignalValue[nIEMSts]) || (FALSE == Rte_FISD_enGetAlternatorTerminal())||\
	 	 ((IlGetDCDC_EnableStsRxTimeout()== FALSE)&& (IlGetRxDCDC_EnableSts() == 0u) && (sys_fullon)) ||\
	 	 ((IlGetDCDC_DCDCSysFaultRxTimeout()== FALSE)&& (IlGetRxDCDC_DCDCSysFault() == 1u) && (sys_fullon)))
    {
        if(alternator_io_debounce_timer<ALTERNATOR_IO_DEBOUNCE_TIMER_SETTING)
        {
            alternator_io_debounce_timer++;
        }
        if((alternator_io_debounce_timer>=ALTERNATOR_IO_DEBOUNCE_TIMER_SETTING)||\
		  	((IlGetDCDC_EnableStsRxTimeout()== FALSE)&& (IlGetRxDCDC_EnableSts() == 0u) && (sys_fullon)) ||\
	 	   ((IlGetDCDC_DCDCSysFaultRxTimeout()== FALSE)&& (IlGetRxDCDC_DCDCSysFault() == 1u) && (sys_fullon)))
        {
            IND_SET(nIND_Charger_Iem, nInd_On);
        }
    }
    else
    {
        if(((IlGetDCDC_EnableStsRxTimeout()== FALSE)&& (IlGetRxDCDC_EnableSts() == 1u) && (sys_fullon))||\
		    ((IlGetDCDC_DCDCSysFaultRxTimeout()== FALSE)&& (IlGetRxDCDC_DCDCSysFault() == 0u) && (sys_fullon)))
	     {
		     IND_SET(nIND_Charger_Iem, nInd_Off);
	     }
        alternator_io_debounce_timer = 0;
    }
	
    if((TRUE != Rte_FISD_boAnimationPlayIsFinish()) || (sys_fullon == False))
    {
        IND_SET(nIND_Charger_Iem, nInd_Off);
    }
}
#undef FISD_nLampOn


/*******************************************************************************************
* Function: FISD_vBrakeFluidLevelLow_EbdFaultDetect
* Description: BrakeFluidLevelLow_EbdFaultDetect
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nFailPresent   1u
static void  FISD_vBrakeFluidLevelLow_EbdFaultDetect(void)
{
#define BRAKEFLUID_IO_DEBOUNCE_TIMER  10
    static U8 brakefluid_io_debounce_timer = 0;
    BOOL ebd_ind_condition = TRUE;//(FISD_u16DebounceCount_D1 >= TIMECOUNT_3s);

    IND_SET(nIND_BrakeFluidLevelLow_EbdFault, nInd_Off);
    DIC_SET(nDIC_FillBrakeFluid, nDic_Inactive);
	
    if(FALSE == Rte_FISD_enGetBrakeFluidSensor())
    {
        if(brakefluid_io_debounce_timer<BRAKEFLUID_IO_DEBOUNCE_TIMER)
        {
            brakefluid_io_debounce_timer++;
        }
        if((brakefluid_io_debounce_timer>=BRAKEFLUID_IO_DEBOUNCE_TIMER)&&ebd_ind_condition)
        {
            IND_SET(nIND_BrakeFluidLevelLow_EbdFault, nInd_On);
            DIC_SET(nDIC_FillBrakeFluid, nDic_Active);
        }
    }
    else
    {
        brakefluid_io_debounce_timer = 0;
    }

    if((FISD_nFailPresent == FISD_u16SignalValue[nEBDFailSts])&&ebd_ind_condition)
    {
        IND_SET(nIND_BrakeFluidLevelLow_EbdFault, nInd_On);
    }

    if((TRUE == Rte_boGetEBDFailStsTimeout())&&ebd_ind_condition)
    {
        IND_SET(nIND_BrakeFluidLevelLow_EbdFault, nInd_On);
    }

    if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
    {
        IND_SET(nIND_BrakeFluidLevelLow_EbdFault, nInd_On);
        //FISD_aenIndicatorState[nIND_BrakeFluidLevelLow_EbdFault] = nInd_On;
        //DIC_SET(nDIC_FillBrakeFluid, nDic_Inactive);
    }

    if((TRUE != Rte_FISD_boAnimationPlayIsFinish())||(FISD_boIsCrankOn == True))
    {
        IND_SET(nIND_BrakeFluidLevelLow_EbdFault, nInd_Off);
        DIC_SET(nDIC_FillBrakeFluid, nDic_Inactive);
    }
}
#undef FISD_nFailPresent

/*******************************************************************************************
* Function: FISD_vAbsFaultDetect
* Description: AbsFaultDetect
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nFailPresent  1u
static void  FISD_vAbsFaultDetect(void)
{
    IND_SET(nIND_ABS_Fault, nInd_Off);
    DIC_SET(nDIC_CheckAbsSystem, nDic_Inactive);

    if(TRUE == Rte_FISD_boGetPowerDependent())
    {
        if((FISD_nFailPresent == FISD_u16SignalValue[nABSFailSts]))
        {
            IND_SET(nIND_ABS_Fault, nInd_On);
            DIC_SET(nDIC_CheckAbsSystem, nDic_Active);
        }
    }

    if((TRUE == Rte_boGetABSFailStsTimeout()))
    {
		IND_SET(nIND_ABS_Fault, nInd_On);
    }

    if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
    {
        IND_SET(nIND_ABS_Fault, nInd_On);
    }

    if((TRUE != Rte_FISD_boAnimationPlayIsFinish())||(FISD_boIsCrankOn == True))
    {
        IND_SET(nIND_ABS_Fault, nInd_Off);
        DIC_SET(nDIC_CheckAbsSystem, nDic_Inactive);
    }
}
#undef FISD_nFailPresent

/*******************************************************************************************
* Function: FISD_vEspFaultShieldDetect
* Description: EspFaultShieldDetect
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nActive           1u
#define FISD_nValid            0u
#define FISD_nFailPresent      1u
static void  FISD_vEspFaultShieldDetect(void)
{
    IND_SET(nIND_ESP_Fault, nInd_Off);
    IND_SET(nIND_ESP_Shield, nInd_Off);
    DIC_SET(nDIC_CheckEspSystem, nDic_Inactive);

    if(TRUE == Rte_FISD_boGetPowerDependent())
    {
        if(((FISD_nActive == FISD_u16SignalValue[nVDCActive]) || \
                (FISD_nActive == FISD_u16SignalValue[nTCSActive])))
        {
            IND_SET(nIND_ESP_Fault, nInd_Flash_1Hz);
        }

        if((FISD_nActive == FISD_u16SignalValue[nESPSwitchStatus]))
        {
            IND_SET(nIND_ESP_Shield, nInd_On);
            IND_SET(nIND_ESP_Fault, nInd_Off);
        }

        if(((FISD_nFailPresent == FISD_u16SignalValue[nVDCFailSts]) || \
                (FISD_nFailPresent == FISD_u16SignalValue[nTCSFailSts])))
        {
            IND_SET(nIND_ESP_Fault, nInd_On);
            IND_SET(nIND_ESP_Shield, nInd_Off);
            DIC_SET(nDIC_CheckEspSystem, nDic_Active);
        }

        if((TRUE == Rte_boGetVDCFailStsTimeout()))
        {
        	if(Rte_FISD_u8GetCONFIG_ESP() == TRUE)
        	{
				IND_SET(nIND_ESP_Fault, nInd_On);
        	}
        }
    }

    Rte_FISD_vSetESPOnOffStatus((U8)FISD_u16SignalValue[nESPSwitchStatus]);

    if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
    {
        IND_SET(nIND_ESP_Fault, nInd_On);
        IND_SET(nIND_ESP_Shield, nInd_Off);
    }

    if((TRUE != Rte_FISD_boAnimationPlayIsFinish())||(FISD_boIsCrankOn == True))
    {
        IND_SET(nIND_ESP_Fault, nInd_Off);
        IND_SET(nIND_ESP_Shield, nInd_Off);
        DIC_SET(nDIC_CheckEspSystem, nDic_Inactive);
    }
}
#undef FISD_nActive
#undef FISD_nValid
#undef FISD_nFailPresent

/*******************************************************************************************
* Function: FISD_vCruiseLimitControl(void)
* Description: CruiseDetect
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nInserted   1u
#define FISD_nPrecruise  2u
#define FISD_nNotUsed    3u

U16 u16CruiseLimitSpd = 0u;
U16 u16VehicleLimitSpd = 0u;
static void  FISD_vCruiseDetect(void);
BOOL  FISD_vLimitSpdDetect(void);
static void  FISD_vCruiseCtrlDetect(void);
static void FISD_ClearLimitSpdSts(void);
static void FISD_ClearCruiseCtrlSpdSts(void);
static BOOL  FISD_vCruiseControlDetect(void);
static void  FISD_vLimitSpeedDetect(void);

static void FISD_vCruiseLimitControl(void)
{
    u16CruiseLimitSpd = 0u;
    u16VehicleLimitSpd = 0u;

    /*clear sts*/
    IND_SET(nIND_Cruise, nInd_Off);
    FISD_ClearLimitSpdSts();
    FISD_ClearCruiseCtrlSpdSts();
	DIC_SET(nDIC_LimitSpeed, nDic_Inactive);
	DIC_SET(nDIC_CruiseControl, nDic_Inactive);

	if(boACCSpeedSts == FALSE)
	{
		if(FISD_vCruiseControlDetect() == FALSE)
		{
			FISD_vLimitSpeedDetect();
		}
	}	
	
	if(FALSE == Rte_FISD_boAnimationPlayIsFinish())
	{
		DIC_SET(nDIC_LimitSpeed, nDic_Inactive);
		DIC_SET(nDIC_CruiseControl, nDic_Inactive);
	}
    /*detect*/
    //priopty: ACC > Cruise > LimitSpd > CruiseCtrl
  
    if((TRUE == Rte_boGetCruiseLimitSpeedValueTimeout())&&(TRUE == Rte_boGetVCU_LimitSpeedStsTimeout()))
    {
        FISD_vCruiseDetect();
    }
    else
    {
    	FISD_vCruiseCtrlDetect();
		if(IND_CHK(nIND_CruiseSpdFlash, nInd_Off) == TRUE)
		{
        	FISD_vLimitSpdDetect();
		}
    }
}


static void FISD_ClearCruiseCtrlSpdSts(void)
{
    IND_SET(nIND_CruiseOn, nInd_Off);
    IND_SET(nIND_CruiseCancel, nInd_Off);
    IND_SET(nIND_CruiseSpdFlash, nInd_Off);
    IND_SET(nIND_CancelDisplay, nInd_Off);
}


static void FISD_vLimitSpeedDetect(void)/*主动限速操作提示弹窗*/
{
	U16 u16CruiseLimitSpeedValue;
	static U16 u16CruiseLimitSpeedValue_pre;
	U8 u8LimitSpeedSts;
	if(Rte_boGetCruiseLimitSpeedValueTimeout() == FALSE)
	{
		u8LimitSpeedSts = FISD_u16SignalValue[nLimitSpeedSts];
		u16CruiseLimitSpeedValue = FISD_u16SignalValue[nCruiseLimitSpeedValue];
	}
	else
	{
		u8LimitSpeedSts = 0;
		u16CruiseLimitSpeedValue = 0;;
	}


	if(u8LimitSpeedSts == 0x03)
	{
		if(u16CruiseLimitSpeedValue > 0)
		{
			DIC_SET(nDIC_LimitSpeed, nDic_Active);
	
			if(u16CruiseLimitSpeedValue != u16CruiseLimitSpeedValue_pre)
			{
				if(TRUE == Rte_FISD_boGetTextShowFinishFlag(nDIC_LimitSpeed))
				{
				   	DIC_SET(nDIC_LimitSpeed, nDic_Inactive);/*重新触发门开显示*/
				}
				else
				{
				    FISD_vSetTextShowTimeResetFlag(nDIC_LimitSpeed);/*延长文字的显示时间*/
				}
			}	
		}
	}

	u16CruiseLimitSpeedValue_pre = u16CruiseLimitSpeedValue;
}


static BOOL  FISD_vCruiseControlDetect(void)
{
	U16 u16CruiseLimitSpeedValue;
	static U16 u16CruiseLimitSpeedValue_pre;
	U8 u8CruiseControlStsForDisplay;
	BOOL boCruiseControlSts = FALSE;
	
	if(Rte_boGetCruiseLimitSpeedValueTimeout() == FALSE)
	{
		u8CruiseControlStsForDisplay = FISD_u16SignalValue[nCruiseControlStsForDisplay];
	}
	else
	{
		u8CruiseControlStsForDisplay = 0;
	}

	if(Rte_boGetCruiseControlStsForDisplayTimeout() == FALSE)
	{
		u16CruiseLimitSpeedValue = FISD_u16SignalValue[nCruiseLimitSpeedValue];
	}
	else
	{
		u16CruiseLimitSpeedValue = 0;
	}


	if(u8CruiseControlStsForDisplay == 0x01)
	{
		if(u16CruiseLimitSpeedValue > 0)
		{
			boCruiseControlSts = TRUE;
			DIC_SET(nDIC_CruiseControl, nDic_Active);
			
			if(u16CruiseLimitSpeedValue != u16CruiseLimitSpeedValue_pre)
			{
				if(TRUE == Rte_FISD_boGetTextShowFinishFlag(nDIC_LimitSpeed))
				{
				   	DIC_SET(nDIC_CruiseControl, nDic_Inactive);/*重新触发门开显示*/
				}
				else
				{
				    FISD_vSetTextShowTimeResetFlag(nDIC_CruiseControl);/*延长文字的显示时间*/
				}
			}	
		}

		
	}

	
	u16CruiseLimitSpeedValue_pre = u16CruiseLimitSpeedValue;
	
	return boCruiseControlSts;
}

static void  FISD_vCruiseCtrlDetect(void)
{
    U16 CruiseCtrlSts = 0u;
    U16 CruiseLimitSpdVal = 0u;
    static U8 cruise_2_kind = 0;

    static U8 internal_cuise_to_on_counter = 0;
    U16 u16Wave = Rte_FISD_boGetEntireFlashStatus(nFlash500msOn500msOff);
    static U16 prev_u16Wave = 0;

    if(TRUE == Rte_FISD_boAnimationPlayIsFinish())
    {
        CruiseCtrlSts = FISD_u16SignalValue[nCruiseControlStsForDisplay];
        CruiseLimitSpdVal = FISD_u16SignalValue[nCruiseLimitSpeedValue];
        switch(CruiseCtrlSts)
        {
        case 1:
            cruise_2_kind = 0;
            if(CruiseLimitSpdVal < 0x1FFF)
            {
                IND_SET(nIND_CruiseOn, nInd_On);
                IND_SET(nIND_CruiseSpdFlash, nInd_On);
                u16CruiseLimitSpd = CruiseLimitSpdVal >> 4;
            }
            break;
        case 2:
        {
            if(CruiseLimitSpdVal == 0x1FFF)
            {
                if(cruise_2_kind!=1)
                {
                    internal_cuise_to_on_counter = 100;
                }
                cruise_2_kind = 1;
                IND_SET(nIND_CruiseSpdFlash, nInd_Flash_1Hz);
                u16CruiseLimitSpd = 0x1FFF;
            }
            else if(CruiseLimitSpdVal < 0x1FFF)
            {

                if(cruise_2_kind!=2)
                {
                    internal_cuise_to_on_counter = 100;
                }
                cruise_2_kind = 2;
                IND_SET(nIND_CruiseSpdFlash, nInd_Flash_1Hz);
                u16CruiseLimitSpd = CruiseLimitSpdVal >> 4;
            }
            else
            {
                internal_cuise_to_on_counter = 0;
                cruise_2_kind = 0;
            }

            if(internal_cuise_to_on_counter)
            {
                internal_cuise_to_on_counter--;
                if((prev_u16Wave==0)&&u16Wave)
                {
                    internal_cuise_to_on_counter = 0;
                }
            }
            if(internal_cuise_to_on_counter==0)
            {
                if(cruise_2_kind==1)
                {
                    IND_SET(nIND_CruiseCancel, nInd_On);
                }
                else if(cruise_2_kind==2)
                {
                    IND_SET(nIND_CruiseCancel, nInd_On);
                    IND_SET(nIND_CancelDisplay, nInd_On);
                }
                else
                {
                }
            }
        }
        break;
        case 3:
            if(CruiseLimitSpdVal < 0x1FFF)
            {
                if(cruise_2_kind!=3)
                {
                    internal_cuise_to_on_counter = 100;
                    cruise_2_kind = 3;
                }
                IND_SET(nIND_CruiseSpdFlash, nInd_Flash_1Hz);
                u16CruiseLimitSpd = CruiseLimitSpdVal >> 4;
            }
            else
            {
                internal_cuise_to_on_counter = 0;
                cruise_2_kind = 0;
            }

            if(internal_cuise_to_on_counter)
            {
                internal_cuise_to_on_counter--;
                if((prev_u16Wave==0)&&u16Wave)
                {
                    internal_cuise_to_on_counter = 0;
                }
            }
            if(internal_cuise_to_on_counter==0)
            {
                if(cruise_2_kind==3)
                {
                    IND_SET(nIND_CruiseCancel, nInd_On);
                    IND_SET(nIND_CancelDisplay, nInd_On);
                }
                else
                {
                }
            }
            break;
        default:
            cruise_2_kind = 0;
            break;
        }
        prev_u16Wave = u16Wave;
    }
}


U8 FISD_u8GetVehicleLimitSpd(void)/*主动限速车速值*/
{
	return (U8)(u16VehicleLimitSpd & 0xFF);
}

static void FISD_ClearLimitSpdSts(void)
{
    IND_SET(nIND_LimitOn, nInd_Off);
    IND_SET(nIND_LimitCancel, nInd_Off);
    IND_SET(nIND_LimitDefault, nInd_Off);
	IND_SET(nIND_LimitOn_Grey, nInd_Off);//EV
	IND_SET(nIND_LimitOn_Green, nInd_Off);//EV
    IND_SET(nIND_LimitSpdFlash, nInd_Off);
    BUZ_SET(nBUZ_OVS_3, nBuz_Inactive);
	
	DIC_SET(nDIC_LimitSpeedCancel, nDic_Inactive);
	BUZ_SET(nBUZ_LimitSpeedCancel, nBuz_Inactive);
}
BOOL FISD_vLimitSpdDetect(void)
{
    U16 LimitSpdsts = 0u;
    U16 CruiseLimitSpdVal = 0u;
    BOOL boLimitSpdIsWorking = FALSE;
	U8 u8DrivingPowerType = Rte_FISD_u8GetCONFIG_DrivingPowerType();
	
    IND_SET(nIND_LimitSpd_Red_Flag,nInd_Off);

    if(TRUE == Rte_FISD_boAnimationPlayIsFinish())
    {
        if(u8DrivingPowerType == nPowerType_EV)
        {
            LimitSpdsts = FISD_u16SignalValue[nVCU_LimitSpeedSts];
            CruiseLimitSpdVal = FISD_u16SignalValue[nVCU_LimitSpeed];
        }
        else
        {
            LimitSpdsts = FISD_u16SignalValue[nLimitSpeedSts];
            CruiseLimitSpdVal = FISD_u16SignalValue[nCruiseLimitSpeedValue];
        }
        if(CruiseLimitSpdVal < 0x1FFF)
        {
            switch(LimitSpdsts)
            {
            case 1:
                IND_SET(nIND_LimitDefault, nInd_On);
				IND_SET(nIND_LimitOn_Grey, nInd_On);//EV
                IND_SET(nIND_LimitSpdFlash, nInd_Flash_1Hz);
                u16VehicleLimitSpd = CruiseLimitSpdVal >> 4;
                boLimitSpdIsWorking = TRUE;
				if(u8DrivingPowerType == nPowerType_EV)
				{
					u16VehicleLimitSpd = (u16VehicleLimitSpd < 30 ? 0xFFFF : u16VehicleLimitSpd);
				}
                break;
            case 2:
                IND_SET(nIND_LimitOn, nInd_On);
				IND_SET(nIND_LimitOn_Green, nInd_On);//EV
                IND_SET(nIND_LimitSpdFlash, nInd_Flash_1Hz);
                IND_SET(nIND_LimitSpd_Red_Flag,nInd_On);
                u16VehicleLimitSpd = CruiseLimitSpdVal >> 4;
                BUZ_SET(nBUZ_OVS_3, nBuz_Active);
                boLimitSpdIsWorking = TRUE;
                break;
            case 3:
                IND_SET(nIND_LimitOn, nInd_On);
				IND_SET(nIND_LimitOn_Green, nInd_On);//EV
                IND_SET(nIND_LimitSpdFlash, nInd_On);
                u16VehicleLimitSpd = CruiseLimitSpdVal >> 4;
                boLimitSpdIsWorking = TRUE;
                break;
            case 4:
                IND_SET(nIND_LimitCancel, nInd_On);
				IND_SET(nIND_LimitOn_Grey, nInd_On);//EV
                IND_SET(nIND_LimitSpdFlash, nInd_On);
                u16VehicleLimitSpd = CruiseLimitSpdVal >> 4;
                boLimitSpdIsWorking = TRUE;
				
				DIC_SET(nDIC_LimitSpeedCancel, nDic_Active);
				BUZ_SET(nBUZ_LimitSpeedCancel, nBuz_Active);

				if(u8DrivingPowerType == nPowerType_EV)
				{
					u16VehicleLimitSpd = (u16VehicleLimitSpd < 30 ? 0xFFFF : u16VehicleLimitSpd);
				}
                break;
            case 5:
                IND_SET(nIND_LimitOn, nInd_On);
				IND_SET(nIND_LimitOn_Green, nInd_On);//EV
                IND_SET(nIND_LimitSpdFlash, nInd_Flash_1Hz);
                u16VehicleLimitSpd = CruiseLimitSpdVal >> 4;
                boLimitSpdIsWorking = TRUE;
                break;
            default:
                break;
            }
			if(u16VehicleLimitSpd != 0xFFFF)
			{
				u16VehicleLimitSpd = (u16VehicleLimitSpd > 150 ? 150 : u16VehicleLimitSpd);
			}
        }
    }
    else
    {
    }

    return boLimitSpdIsWorking;
}
static void  FISD_vCruiseDetect(void)
{
    if(TRUE == Rte_FISD_boAnimationPlayIsFinish())
    {
        if(FISD_nInserted == FISD_u16SignalValue[nCruiseControlStsForDisplay])
        {
            IND_SET(nIND_Cruise, nInd_On);
        }

        if((FISD_nPrecruise == FISD_u16SignalValue[nCruiseControlStsForDisplay]) || \
                (FISD_nNotUsed   == FISD_u16SignalValue[nCruiseControlStsForDisplay]))
        {
            IND_SET(nIND_Cruise, nInd_Flash_1Hz);
        }
    }
}
#undef FISD_nInserted
#undef FISD_nPrecruise
#undef FISD_nNotUsed

/*******************************************************************************************
* Function: FISD_vAirbagDetect(void)
* Description: AirbagDetect
* Parameters: none
* Return: none
********************************************************************************************/
static void  FISD_vAirbagDetect(void)
{
    IND_SET(nIND_Airbag, nInd_Off);
    DIC_SET(nDIC_CheckAirbag, nDic_Inactive);

    if((2u == FISD_u16SignalValue[nAirBagFailSts]))
    {
        IND_SET(nIND_Airbag, nInd_On);
        DIC_SET(nDIC_CheckAirbag, nDic_Active);
    }
    else if((3u == FISD_u16SignalValue[nAirBagFailSts]))
    {
        IND_SET(nIND_Airbag, nInd_On);
    }
    else if((1u == FISD_u16SignalValue[nAirBagFailSts]))
    {
        IND_SET(nIND_Airbag, nInd_Flash_1Hz);
    }
    else
    {
    }

    if((TRUE == Rte_boGetAirBagFailStsTimeout()))
    {
    	IND_SET(nIND_Airbag, nInd_Off);
        DIC_SET(nDIC_CheckAirbag, nDic_Inactive);
    	if(Rte_FISD_u8GetCONFIG_ABM() == TRUE)
    	{
			IND_SET(nIND_Airbag, nInd_On);
		}
    }

    if((FISD_boSelfCheckFlag == TRUE) && \
            (FISD_u16DebounceCount_D1 < TIMECOUNT_6s))
    {
        IND_SET(nIND_Airbag, nInd_On);
        DIC_SET(nDIC_CheckAirbag, nDic_Inactive);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        IND_SET(nIND_Airbag, nInd_Off);
        DIC_SET(nDIC_CheckAirbag, nDic_Inactive);
    }
}

/*******************************************************************************************
* Function: FISD_vTyrePressureDetect(void)
* Description: TyrePressure
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nTyrePressureLedFlashNum  75u

static void  FISD_vTyrePressureClear(void);
static void  FISD_vTyrePressureLedDetect(void);
static void  FISD_vCheckTyrePositionWarn(tenIndicatorStatus *penState);

static void  FISD_vTyrePressureDetect(void)
{
    /*Clear Status*/
    FISD_vTyrePressureClear();
    /*Redetect Status*/
    FISD_vTyrePressureLedDetect();

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        FISD_vTyrePressureClear();
    }
}

static BOOL tyre_selfcheck_status = FALSE;
BOOL FISD_VTyreGetSelfCheckStatus(void)
{
    return tyre_selfcheck_status;
}

static void  FISD_vTyrePressureLedDetect(void)
{
    tenIndicatorStatus enState = nInd_Off;
    static U16 u16sig_last_tirepressure_system_fail = 0;
	#if 0
    static U16 u16sig_last_tireposition_warning_lhf = 0;
    static U16 u16sig_last_tireposition_warning_Rhf = 0;
    static U16 u16sig_last_tireposition_warning_lhR = 0;
    static U16 u16sig_last_tireposition_warning_Rhr = 0;
	#endif
    /*Redetect Status*/
    if(FISD_u16SignalValue[nTirePressureWarningLampSts] == 1u)
    {
        IND_SET(nIND_TyrePressureLow, nInd_Flash_1Hz);
        DIC_SET(nDIC_CheckTpms, nDic_Active);
    }
    else if(FISD_u16SignalValue[nTirePressureWarningLampSts] == 2u)
    {
        IND_SET(nIND_TyrePressureLow, nInd_On);
    }
    else
    {
    }

 
	#if 0  
    FISD_vCheckTyrePositionWarn(&enState);
	
    if(enState == nInd_On)
    {
        IND_SET(nIND_TyrePressureLow, nInd_On);
    }
	#endif

    if((FISD_u16SignalValue[nTirePressureSystemFailSts] == 1u) /*|| (enState == nInd_Flash_1Hz)*/)
    {
        if((u16sig_last_tirepressure_system_fail!=1)&&(FISD_u16SignalValue[nTirePressureSystemFailSts]==1))
        {
            Rte_FISD_u8ResetIndicatorFlashCount(nIND_TyrePressureLow);
        }
		#if 0
        else if((u16sig_last_tireposition_warning_lhf!=6)&&(FISD_u16SignalValue[nTirePositionWarning_LHFTire]==6))
        {
            Rte_FISD_u8ResetIndicatorFlashCount(nIND_TyrePressureLow);
        }
        else if((u16sig_last_tireposition_warning_Rhf!=6)&&(FISD_u16SignalValue[nTirePositionWarning_RHFTire]==6))
        {
            Rte_FISD_u8ResetIndicatorFlashCount(nIND_TyrePressureLow);
        }
        else if((u16sig_last_tireposition_warning_lhR!=6)&&(FISD_u16SignalValue[nTirePositionWarning_LHRTire]==6))
        {
            Rte_FISD_u8ResetIndicatorFlashCount(nIND_TyrePressureLow);
        }
        else if((u16sig_last_tireposition_warning_Rhr!=6)&&(FISD_u16SignalValue[nTirePositionWarning_RHRTire]==6))
        {
            Rte_FISD_u8ResetIndicatorFlashCount(nIND_TyrePressureLow);
        }
		#endif
        else
        {
        }
        if(Rte_FISD_u8GetIndicatorFlashCount(nIND_TyrePressureLow) <= FISD_nTyrePressureLedFlashNum)
        {
            IND_SET(nIND_TyrePressureLow, nInd_Flash_1Hz);
        }
        else
        {
            IND_SET(nIND_TyrePressureLow, nInd_On);
        }
    }
    else
    {
    }

    if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
    {
        IND_SET(nIND_TyrePressureLow, nInd_On);
    }

    if((FISD_u16DebounceCount_D1 < TIMECOUNT_3s))
    {
        tyre_selfcheck_status = TRUE;
    }
    else
    {
        tyre_selfcheck_status = FALSE;
    }

    if(FISD_u16SignalValue[nTirePressureSystemFailSts] == 1u)
    {
        DIC_SET(nDIC_CheckTpms, nDic_Active);
    }

	if(Rte_boGetTirePressureSystemFailStsTimeout() == TRUE)
	{
		DIC_SET(nDIC_CheckTpms, nDic_Inactive);
	}
	if((TRUE == Rte_boGetTirePressureSystemFailStsTimeout())&&(TRUE == IlGetBCM_16_RHFTirePressureRxTimeout()))
	 {
		 IND_SET(nIND_TyrePressureLow, nInd_On);
	 }

    u16sig_last_tirepressure_system_fail = FISD_u16SignalValue[nTirePressureSystemFailSts];
	#if 0
    u16sig_last_tireposition_warning_lhf = FISD_u16SignalValue[nTirePositionWarning_LHFTire];
    u16sig_last_tireposition_warning_Rhf = FISD_u16SignalValue[nTirePositionWarning_RHFTire];
    u16sig_last_tireposition_warning_lhR = FISD_u16SignalValue[nTirePositionWarning_LHRTire];
    u16sig_last_tireposition_warning_Rhr = FISD_u16SignalValue[nTirePositionWarning_RHRTire];
	#endif
	if(Rte_FISD_boAnimationPlayIsFinish() == FALSE)
	{
		DIC_SET(nDIC_CheckTpms, nDic_Inactive);
	}
}

static void  FISD_vCheckTyrePositionWarn(tenIndicatorStatus *penState)
{
    U32 u32Num = 0u;
    U32 u32Interval = nTirePositionWarning_RHRTire - nTirePositionWarning_LHFTire;

    for(u32Num = 0u; u32Num <= u32Interval; u32Num ++)
    {
        if((FISD_u16SignalValue[nTirePositionWarning_LHFTire+ u32Num] == 1u) || \
                (FISD_u16SignalValue[nTirePositionWarning_LHFTire + u32Num] == 2u) || \
                (FISD_u16SignalValue[nTirePositionWarning_LHFTire + u32Num] == 4u) || \
                (FISD_u16SignalValue[nTirePositionWarning_LHFTire + u32Num] == 5u))
        {
            *penState = nInd_On;
            break;
        }
    }

    if((FISD_u16SignalValue[nTirePositionWarning_LHFTire] == 6u) || \
            (FISD_u16SignalValue[nTirePositionWarning_RHFTire] == 6u) || \
            (FISD_u16SignalValue[nTirePositionWarning_LHRTire] == 6u) || \
            (FISD_u16SignalValue[nTirePositionWarning_RHRTire] == 6u))
    {
        *penState = nInd_Flash_1Hz;
    }
}

static void  FISD_vTyrePressureClear(void)
{
    U32 u32Num = 0u;
    IND_SET(nIND_TyrePressureLow, nInd_Off);
    DIC_SET(nDIC_CheckTpms, nDic_Inactive);
}
#undef FISD_nTyrePressureLedFlashNum
/*******************************************************************************************
* Function: FISD_vTransmissionFaultDetect(void)
* Description: TransmissionFaultDetect
* Parameters: none
* Return: none
********************************************************************************************/
#define WARNMESSAGE_NUM   11u
#define WM_STATUS_SET       0u
#define WM_STATUS_RESET   1u
#define WM_SWITCH_TIME_MIN           TIMECOUNT_2s
#define WARNMESSAGE_TIMEOUT_MAX TIMECOUNT_15s
U8 WarnMessageIDActiveNum = 0u;
U8 FISD_u8WMIndicator2beDisp[WARNMESSAGE_NUM] = {0u};
U16  FISD_u16WMIDCostTimeFromLastRefresh[WARNMESSAGE_NUM] = {0u};
BOOL FISD_boWarnMessageIDSts_Pre[WARNMESSAGE_NUM] = {1u,1u,1u,1u,1u,1u,1u,1u, 1u, 1u, 1u};
tenTextId FISD_boWarnMessageID_Enum[WARNMESSAGE_NUM][2] = 
{
	nDIC_PleaseStampBraking, 				nBUZ_PleaseStampBraking, 
	nDIC_CarMovePleaseSwitch_P, 			nBUZ_CarMovePleaseSwitch_P, 	
	nDIC_GearboxSeriousFault, 				nBUZ_GearboxSeriousFault, 	
	nDIC_TransmissionFault_7DTC, 			nBUZ_TransmissionFault_7DTC, 	
	nDIC_TransTempHighHold5Minute_7DCT,		nBUZ_TransTempHighHold5Minute_7DCT,		
	nDIC_GearFault_P, 						nBUZ_GearFault_P, 
	nDIC_BrakingTillShowGear_P, 			nBUZ_BrakingTillShowGear_P, 	
	nDIC_TransTempHighPleaseStop, 			nBUZ_TransTempHighPleaseStop, 	
	nDIC_NotInGearP, 						nBUZ_NotInGearP, 
	nDIC_SwitchtoP_AfterStopped, 			nBUZ_SwitchtoP_AfterStopped, 
	nDIC_OperateGearPWhenEngineRun,			nBUZ_OperateGearPWhenEngineRun
	
};

static void FISD_vClearTransmissionFaultStatus(void);
static void FISD_vNotCfg7DCTTransmissionFaultDetect(void);
static void FISD_vTransmissionGBFaultstatusDetect(void);
static void FISD_vCfgAs7DCTTransmissionFaultDetect(void);
static void  FISD_vInsertWMID2IndicatorDispLine(U8 u8Ind_ID);
static void FISD_vDeleteWMIDFromIndicatorDispLine(U8 u8Ind_ID);
static U8 FISD_u8LookUpWhichOneIndcator2Display(void);
static void FISD_vAccumulateCostTimeFromLastRefresh(void);
static void FISD_vClearCostTimeFromLastRefresh(U8 WMID_2Clear);


static void  FISD_vTransmissionFaultDetect(void)
{
    FISD_vClearTransmissionFaultStatus();

    if(TRUE == Rte_FISD_boAnimationPlayIsFinish())
    {
        if ((Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV 
            && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3))
        {
            /*PHEV3变速箱报警提醒*/
            FISD_vTransmissionGBFaultstatusDetect();
        }
        else
        {
            /*config as 7DCT*/
            if(TRUE == Rte_FISD_u8GetCONFIG_TCU4_7DCT()|| TRUE == Rte_FISD_u8GetCONFIG_TCU_CVT25_18())
            {
                FISD_vCfgAs7DCTTransmissionFaultDetect();
            }
            /*Not config asDCT*/
            FISD_vNotCfg7DCTTransmissionFaultDetect();
        }
    }
    else
    {
    }
}

static void FISD_vClearCostTimeFromLastRefresh(U8 WMID_2Clear)
{
    FISD_u16WMIDCostTimeFromLastRefresh[WMID_2Clear - 1u] = 0u;
}

static void FISD_vAccumulateCostTimeFromLastRefresh(void)
{
    U8 u8i;
    for(u8i = 0u; u8i < WARNMESSAGE_NUM; u8i ++)
    {
        if((WM_STATUS_SET == FISD_boWarnMessageIDSts_Pre[u8i]) && \
                (TIMECOUNT_15s > FISD_u16WMIDCostTimeFromLastRefresh[u8i]))
        {
            FISD_u16WMIDCostTimeFromLastRefresh[u8i] ++;
        }

        if(FISD_u16WMIDCostTimeFromLastRefresh[u8i] >= TIMECOUNT_15s)
        {
            FISD_vDeleteWMIDFromIndicatorDispLine(u8i + 1u); //need 验证
            FISD_boWarnMessageIDSts_Pre[u8i] = WM_STATUS_RESET;
        }
    }
}

static U8 FISD_u8LookUpWhichOneIndcator2Display(void)
{
    int8_t int8i = WARNMESSAGE_NUM - 1u;
    U8 u8Need2Disp = 0xFF; /*none to display*/

    /*look for the last one*/
    for( ; int8i >= 0; int8i --)
    {
        if(0u != FISD_u8WMIndicator2beDisp[int8i])
        {
            u8Need2Disp = FISD_u8WMIndicator2beDisp[int8i];
            break;
        }
    }

    return u8Need2Disp;
}
static void  FISD_vInsertWMID2IndicatorDispLine(U8 u8Ind_ID)
{
    U8 u8i;
    BOOL boInsertSts = FALSE;
    U8 *pIndVisitID = FISD_u8WMIndicator2beDisp;

    /*the one to be inserted is the one current displayed*/
    if(u8Ind_ID == FISD_u8LookUpWhichOneIndcator2Display())
    {
        return;
    }
    else
    {
        for(u8i = 0u; u8i < WARNMESSAGE_NUM; u8i ++)
        {
            if(*pIndVisitID != 0u)
            {
                pIndVisitID ++;
            }
            else
            {
                *pIndVisitID = u8Ind_ID;
                boInsertSts = TRUE;
                break;
            }
        }

        /*the inserted line is full*/
        if(FALSE == boInsertSts)
        {
            FISD_vDeleteWMIDFromIndicatorDispLine(u8Ind_ID);
            FISD_u8WMIndicator2beDisp[WARNMESSAGE_NUM - 1u] = u8Ind_ID;
        }
    }
}

static void FISD_vDeleteWMIDFromIndicatorDispLine(U8 u8Ind_ID)
{
    U8 u8i;
    BOOL boFindVal = FALSE;

    for(u8i = 0u; u8i < WARNMESSAGE_NUM; u8i ++)
    {
        if(FISD_u8WMIndicator2beDisp[u8i] == u8Ind_ID)
        {
            FISD_u8WMIndicator2beDisp[u8i] = 0u;
            boFindVal = TRUE;
            u8i ++;
            break;
        }
    }

    if(boFindVal == TRUE)
    {
        for( ; u8i < WARNMESSAGE_NUM; u8i++)
        {
            if(0u != FISD_u8WMIndicator2beDisp[u8i])
            {
                FISD_u8WMIndicator2beDisp[u8i -1] = FISD_u8WMIndicator2beDisp[u8i];
                FISD_u8WMIndicator2beDisp[u8i] = 0u;
            }
        }
    }
}
static void FISD_vCfgAs7DCTTransmissionFaultDetect(void)
{
    U8 FISD_boGetWarnMessageIDSts_Cur = FISD_u16SignalValue[nBitStatus];
    U8 u8i,FISD_u8WarnMessageIDGet_Cur = (U8)FISD_u16SignalValue[nWarningMessageID];

    /*WarningMessageID Init*/
    if(TRUE == WarningMessageIDneedInitFlag)
    {
        WarningMessageIDneedInitFlag= FALSE;
        for(u8i = 0u; u8i < WARNMESSAGE_NUM; u8i ++)
        {
            FISD_boWarnMessageIDSts_Pre[u8i] = WM_STATUS_RESET;
            FISD_u8WMIndicator2beDisp[u8i] = 0u;
            FISD_u16WMIDCostTimeFromLastRefresh[u8i] = 0u;
        }
        WarnMessageIDActiveNum = 0u;
    }

    if(FALSE == WarningMessageIDneedInitFlag)/*Init success,and work normally*/
    {
        /*receive valid WarnMessageID*/
        if((FISD_u8WarnMessageIDGet_Cur > 0u) && (FISD_u8WarnMessageIDGet_Cur < 12u) )
        {
            if(WM_STATUS_SET == FISD_boGetWarnMessageIDSts_Cur)            /*value == set*/
            {
                if(WM_STATUS_RESET == FISD_boWarnMessageIDSts_Pre[FISD_u8WarnMessageIDGet_Cur - 1u]) /*pre status == reset*/
                {
                    FISD_boWarnMessageIDSts_Pre[FISD_u8WarnMessageIDGet_Cur - 1u] = WM_STATUS_SET;
                    FISD_vInsertWMID2IndicatorDispLine(FISD_u8WarnMessageIDGet_Cur);
                    FISD_vClearCostTimeFromLastRefresh(FISD_u8WarnMessageIDGet_Cur);
                }
                else if(WM_STATUS_SET == FISD_boWarnMessageIDSts_Pre[FISD_u8WarnMessageIDGet_Cur - 1u]) /*pre status == set*/
                {
                    FISD_vClearCostTimeFromLastRefresh(FISD_u8WarnMessageIDGet_Cur);
                }
                else
                {
                }
            }
            else if(WM_STATUS_RESET == FISD_boGetWarnMessageIDSts_Cur)  /*value == reset*/
            {
                FISD_boWarnMessageIDSts_Pre[FISD_u8WarnMessageIDGet_Cur - 1u] = WM_STATUS_RESET;
                FISD_vDeleteWMIDFromIndicatorDispLine(FISD_u8WarnMessageIDGet_Cur);
                FISD_vClearCostTimeFromLastRefresh(FISD_u8WarnMessageIDGet_Cur);
            }
            else
            {
            }
        }
        else
        {
        }
    }

    FISD_vAccumulateCostTimeFromLastRefresh();

    /*display on*/
    switch(FISD_u8LookUpWhichOneIndcator2Display())
    {
    case 3u:
    case 4u:
        IND_SET(nIND_GearboxFault_Red, nInd_On);
        break;
    case 5u:
        IND_SET(nIND_TransmisionFault_Red, nInd_On);
        break;
    case 6u:
        IND_SET(nIND_TransmissionFault_CVT_DCT, nInd_On);
        break;
    case 8u:
        IND_SET(nIND_TransmisionFault_Yellow, nInd_On);
        break;
    default:
        break;
    }   

    for(u8i = 0u; u8i < WARNMESSAGE_NUM; u8i ++)
    {
        if(WM_STATUS_SET == FISD_boWarnMessageIDSts_Pre[u8i])
        {
            DIC_SET(FISD_boWarnMessageID_Enum[u8i][0], nDic_Active);
            BUZ_SET(FISD_boWarnMessageID_Enum[u8i][1], nBuz_Active);
        }

        /*deal with the sound warning connect to textwarn which is cancelled in 2s.*/
        if(FISD_boWarnMessageID_Enum[u8i][0] == TextMgr_enGetDisplayText())
        {
            BUZ_SET(nBUZ_PleaseStampBraking + u8i,nBuz_Active);
        }
        else
        {
            BUZ_SET(nBUZ_PleaseStampBraking + u8i,nBuz_Inactive);
        }
    }

    if(TRUE == Rte_boGetGBFaultstatusTimeout())
    {
    	//0224 7DCT报警增加超时判断
    	//FISD_vClearTransmissionFaultStatus();
    	if(Rte_FISD_u8GetCONFIG_TCU1_CVT19()||Rte_FISD_u8GetCONFIG_TCU2_6DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18()||Rte_FISD_u8GetCONFIG_TCU4_7DCT())
    	{
        	IND_SET(nIND_TransmissionFault_CVT_DCT, nInd_On);
    	}
    }

    if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
    {
	    if(Rte_FISD_u8GetCONFIG_TCU1_CVT19()||Rte_FISD_u8GetCONFIG_TCU2_6DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18()||Rte_FISD_u8GetCONFIG_TCU4_7DCT())
	    {
	        IND_SET(nIND_TransmissionFault_CVT_DCT, nInd_On);
	    }
    }
}
static void FISD_vNotCfg7DCTTransmissionFaultDetect(void)
{
    if((1u == FISD_u16SignalValue[nGBFaultstatus]) && (2u != FISD_u16SignalValue[nClutchTemperature]))
    {
    	if(FISD_u16SignalValue[nClutchTemperature] == 0)
    	{
			DIC_SET(nDIC_TransmissionCheck, nDic_Active);
		}
		else if(FISD_u16SignalValue[nClutchTemperature] == 1)
    	{
			DIC_SET(nDIC_TransmissionCheck_1, nDic_Active);
		}
		else if(FISD_u16SignalValue[nClutchTemperature] == 3)
    	{
			DIC_SET(nDIC_TransmissionCheck_2, nDic_Active);
		}
		
        IND_SET(nIND_TransmissionFault_CVT_DCT, nInd_On);
        
    }
	
    if((2u == FISD_u16SignalValue[nGBFaultstatus]) || (3u == FISD_u16SignalValue[nGBFaultstatus]))
    {
        IND_SET(nIND_TransmissionFault_CVT_DCT, nInd_Flash_1Hz);
    }

    if((0u == FISD_u16SignalValue[nGBFaultstatus]) && (1u == FISD_u16SignalValue[nClutchTemperature]))
    {
        DIC_SET(nDIC_TransmissionTempHigh, nDic_Active);
        BUZ_SET(nBUZ_TransmissionTempHigh, nBuz_Active);
    }

    if((1u == FISD_u16SignalValue[nGBFaultstatus]) && (2u == FISD_u16SignalValue[nClutchTemperature]))
    {
        DIC_SET(nDIC_TransmissionTempHighHold5Minute, nDic_Active);
        BUZ_SET(nBUZ_TransmissionTempHighHold5Minute, nBuz_Active);
    }

#ifdef ENABLE_TEXT_SOUND_MIN_SHOW_2S
    /*deal with the sound warning connect to textwarn which is cancelled in 2s.*/
    if(nDIC_TransmissionTempHighHold5Minute == TextMgr_enGetDisplayText())
    {
        BUZ_SET(nBUZ_TransmissionTempHighHold5Minute,nBuz_Active);
    }
    else if(nDIC_TransmissionTempHigh == TextMgr_enGetDisplayText())
    {
        BUZ_SET(nBUZ_TransmissionTempHigh,nBuz_Active);
    }
    else
    {
    }
#endif

    if(TRUE == Rte_boGetGBFaultstatusTimeout())
    {
    	//0224 非7DCT报警增加超时判断
    	FISD_vClearTransmissionFaultStatus();
		if(Rte_FISD_u8GetCONFIG_TCU1_CVT19()||Rte_FISD_u8GetCONFIG_TCU2_6DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18()||Rte_FISD_u8GetCONFIG_TCU4_7DCT())
		{
       		IND_SET(nIND_TransmissionFault_CVT_DCT, nInd_On);
		}
    }

    if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
    {
    	if(Rte_FISD_u8GetCONFIG_TCU1_CVT19()||Rte_FISD_u8GetCONFIG_TCU2_6DCT()||Rte_FISD_u8GetCONFIG_TCU_CVT25_18()||Rte_FISD_u8GetCONFIG_TCU4_7DCT())
    	{
        	IND_SET(nIND_TransmissionFault_CVT_DCT, nInd_On);
    	}
    }
}

static void FISD_vTransmissionGBFaultstatusDetect(void)
{
    /*变速箱故障报警*/
	if(TRUE == Rte_FISD_boGetPowerDependent()
        && (FALSE == Rte_boGetGBFaultstatusTimeout()) 
        && Rte_FISD_boAnimationPlayIsFinish()
        && Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV 
        && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3)
	{
		if(Rte_u16GetGBFaultstatus() == 1)
		{
            DIC_SET(nDIC_TransLimitedFunction, nDic_Active);
            BUZ_SET(nBUZ_TransLimitedFunction,nBuz_Active);
			IND_SET(nIND_TransmissionFault_CVT_DCT, nInd_On);
		}
		else if(Rte_u16GetGBFaultstatus() == 2)
		{
            DIC_SET(nDIC_TransHighTemperature, nDic_Active);
            BUZ_SET(nBUZ_TransHighTemperature,nBuz_Active);
			IND_SET(nIND_GearboxFault_Red, nInd_On);
		}
		else if(Rte_u16GetGBFaultstatus() == 3)
		{
            DIC_SET(nDIC_ClutchHighTemperature, nDic_Active);
            BUZ_SET(nBUZ_ClutchHighTemperature,nBuz_Active);
			IND_SET(nIND_TransmissionFault_CVT_DCT, nInd_On);
		}
	}

    /*离合器自学习报警*/
	if(TRUE == Rte_FISD_boGetPowerDependent()
        && (FALSE == Rte_boGetGBFaultstatusTimeout()) 
        && Rte_FISD_boAnimationPlayIsFinish()
        && Rte_FISD_u8GetCONFIG_DrivingPowerType()== nPowerType_PHEV 
        && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3)
	{
        if(Rte_u16GetCluSelfLrng() == 1)
        {
            DIC_SET(nDIC_ClutchSelfLearning, nDic_Active);
            BUZ_SET(nBUZ_ClutchSelfLearning,nBuz_Active);
        }
    }
        
    if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
    {
        if((Rte_FISD_u8GetCONFIG_TCU1_CVT19()==nPresent)||(Rte_FISD_u8GetCONFIG_TCU2_6DCT()==nPresent)||\
            (Rte_FISD_u8GetCONFIG_TCU_CVT25_18()==nPresent)||(Rte_FISD_u8GetCONFIG_TCU4_7DCT()==nPresent))
        {
            IND_SET(nIND_GearboxFault_Red, nInd_Off);
            IND_SET(nIND_TransmissionFault_CVT_DCT, nInd_On);
        }
    }
}

static void FISD_vClearTransmissionFaultStatus(void)
{
    /*7DCT*/
    IND_SET(nIND_GearboxFault_Red, nInd_Off);
    IND_SET(nIND_TransmisionFault_Red, nInd_Off);
    IND_SET(nIND_TransmisionFault_Yellow, nInd_Off);

    DIC_SET(nDIC_PleaseStampBraking, nDic_Inactive);
    DIC_SET(nDIC_CarMovePleaseSwitch_P, nDic_Inactive);
    DIC_SET(nDIC_GearboxSeriousFault, nDic_Inactive);
    DIC_SET(nDIC_TransmissionFault_7DTC, nDic_Inactive);
    DIC_SET(nDIC_GearFault_P, nDic_Inactive);
    DIC_SET(nDIC_BrakingTillShowGear_P, nDic_Inactive);
    DIC_SET(nDIC_TransTempHighPleaseStop, nDic_Inactive);
	DIC_SET(nDIC_NotInGearP, nDic_Inactive);
	DIC_SET(nDIC_SwitchtoP_AfterStopped, nDic_Inactive);
	DIC_SET(nDIC_OperateGearPWhenEngineRun, nDic_Inactive);
    DIC_SET(nDIC_TransTempHighHold5Minute_7DCT, nDic_Inactive);

    BUZ_SET(nBUZ_PleaseStampBraking, nBuz_Inactive);
    BUZ_SET(nBUZ_CarMovePleaseSwitch_P, nBuz_Inactive);
    BUZ_SET(nBUZ_GearboxSeriousFault, nBuz_Inactive);
    BUZ_SET(nBUZ_TransmissionFault_7DTC, nBuz_Inactive);
    BUZ_SET(nBUZ_GearFault_P, nBuz_Inactive);
    BUZ_SET(nBUZ_BrakingTillShowGear_P, nBuz_Inactive);
    BUZ_SET(nBUZ_TransTempHighPleaseStop, nBuz_Inactive);
	BUZ_SET(nBUZ_NotInGearP, nBuz_Inactive);
	BUZ_SET(nBUZ_SwitchtoP_AfterStopped, nBuz_Inactive);
	BUZ_SET(nBUZ_OperateGearPWhenEngineRun, nBuz_Inactive);
    BUZ_SET(nBUZ_TransTempHighHold5Minute_7DCT, nBuz_Inactive);

    /*Not 7DCT*/
    IND_SET(nIND_TransmissionFault_CVT_DCT, nInd_Off);
    DIC_SET(nDIC_TransmissionCheck, nDic_Inactive);
    DIC_SET(nDIC_TransmissionCheck_1, nDic_Inactive);
    DIC_SET(nDIC_TransmissionCheck_2, nDic_Inactive);
    DIC_SET(nDIC_TransmissionTempHigh, nDic_Inactive);
    DIC_SET(nDIC_TransmissionTempHighHold5Minute, nDic_Inactive);
    BUZ_SET(nBUZ_TransmissionTempHighHold5Minute, nBuz_Inactive);
    BUZ_SET(nBUZ_TransmissionTempHigh, nBuz_Inactive);

	/*PHEV3.0*/
	//IND_SET(nIND_GearboxFault_Red, nInd_Off);
    //IND_SET(nIND_TransmisionFault_Yellow, nInd_Off);

    DIC_SET(nDIC_TransLimitedFunction, nDic_Inactive);
    DIC_SET(nDIC_TransHighTemperature, nDic_Inactive);
    DIC_SET(nDIC_ClutchHighTemperature, nDic_Inactive);
    DIC_SET(nDIC_ClutchSelfLearning, nDic_Inactive);

    BUZ_SET(nBUZ_TransLimitedFunction,nBuz_Inactive);
    BUZ_SET(nBUZ_TransHighTemperature,nBuz_Inactive);
    BUZ_SET(nBUZ_ClutchHighTemperature,nBuz_Inactive);
    BUZ_SET(nBUZ_ClutchSelfLearning,nBuz_Inactive);
}

/*******************************************************************************************
* Function: FISD_vEpsFaultDetect(void)
* Description: EpsFaultDetect
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nNoCalibration  1u
#define FISD_nFailPresent    1u
#define FISD_nNotLearnt      1u
static void  FISD_vEpsFaultDetect(void)
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
    IND_SET(nIND_EPS_Fault_Red, nInd_Off);
    IND_SET(nIND_EPS_Fault_Yellow, nInd_Off);
    DIC_SET(nDIC_CheckPas, nDic_Inactive);
    DIC_SET(nDIC_EPS_FailureToInitAngle, nDic_Inactive);
    DIC_SET(nDIC_EotNotLearnt, nDic_Inactive);//软止点未学习
	BUZ_SET(nBUZ_CheckPas, nBuz_Inactive);
	BUZ_SET(nBUZ_EotNotLearnt, nBuz_Inactive);
	if(sys_fullon == TRUE)
	{
		if(FISD_nNotLearnt == FISD_u16SignalValue[nEPS_EOTLearning_Sts])
		{
	        IND_SET(nIND_EPS_Fault_Yellow, nInd_Flash_1Hz);
	        DIC_SET(nDIC_EotNotLearnt, nDic_Active);
			BUZ_SET(nBUZ_EotNotLearnt, nBuz_Active);
		}

		if(FISD_nNoCalibration == FISD_u16SignalValue[nEPSSteeringAngleCalibrationSts])
	    {
	        IND_SET(nIND_EPS_Fault_Yellow, nInd_On);
	        DIC_SET(nDIC_EPS_FailureToInitAngle, nDic_Active);
			BUZ_SET(nBUZ_CheckPas,nBuz_Active);
	    }
		
	    if(FISD_nFailPresent == FISD_u16SignalValue[nEPSFailSts])
	    {
	    	IND_SET(nIND_EPS_Fault_Yellow, nInd_Off);
	        IND_SET(nIND_EPS_Fault_Red, nInd_On);
	        DIC_SET(nDIC_CheckPas, nDic_Active);
			BUZ_SET(nBUZ_CheckPas,nBuz_Active);
	    }
		
	}

    if(TRUE == Rte_boGetEPSFailStsTimeout())
    {
        IND_SET(nIND_EPS_Fault_Red, nInd_Off);
		if(Rte_FISD_u8GetCONFIG_EPS() == TRUE)
		{
			IND_SET(nIND_EPS_Fault_Red, nInd_On);
		}
    }

    if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
    {
        IND_SET(nIND_EPS_Fault_Yellow, nInd_On);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
	    IND_SET(nIND_EPS_Fault_Red, nInd_Off);
        IND_SET(nIND_EPS_Fault_Yellow, nInd_Off);
        DIC_SET(nDIC_CheckPas, nDic_Inactive);
        DIC_SET(nDIC_EPS_FailureToInitAngle, nDic_Inactive);
        DIC_SET(nDIC_EotNotLearnt, nDic_Inactive);//软止点未学习
    }
}
#undef FISD_nFailPresent
#undef FISD_nNoCalibration

/*******************************************************************************************
* Function:  FISD_vEngineOilPressureDetect(void)
* Description: EngineOilPressureDetect
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nActive            1u
static void   FISD_vEngineOilPressureClear(void);
static void   FISD_vEngineOilPressureDetect(void)
{
    U16 u16TachoSpeed = Rte_FISD_u16GetEngineSpeedValue();
    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	static U32 u32EngineOilPressureLowtime = 0;
    /*Clear status*/
    FISD_vEngineOilPressureClear();

    /*Redetect status*/
    if((FALSE == Rte_FISD_enGetEngineOilPressureSwitch()) && \
            (u16TachoSpeed >= FISD_u16SelfCheckEngineSpeedThreshold)&&\
            sys_fullon)
    {
	    
        FISD_u32OilPressureLowTime ++;
		
        if(FISD_u32OilPressureLowTime >= TIMECOUNT_3s)
        {
            FISD_u32OilPressureLowTime = TIMECOUNT_3s;
            IND_SET(nIND_EngineOilPressureLow, nInd_Flash_1Hz);
            DIC_SET(nDIC_ShutEngineCheckEngineOil, nDic_Active);
            BUZ_SET(nBUZ_EngineOilPressureLow, nBuz_Active);
        }

		if((BUZ_CHK(nBUZ_EngineOilPressureLow, nBuz_Active) == TRUE) && (SoundMgr_enGetCuerentSoundId() != nBUZ_EngineOilPressureLow))
	    {
	    	u32EngineOilPressureLowtime ++;
	    }
		
		if(u32EngineOilPressureLowtime == TIMECOUNT_5s)
		{
			 BUZ_SET(nBUZ_EngineOilPressureLow, nBuz_Inactive);
			 u32EngineOilPressureLowtime = 0;
		}
    }
    else
    {
        FISD_u32OilPressureLowTime = 0u;
		u32EngineOilPressureLowtime = 0u;
    }

    if((FISD_boIndFuncValid[nIND_EngineOilPressureLow] == TRUE) && \
            (TRUE == FISD_boInstrumentIsInSelfCheckSts()) &&\
            (FALSE == Rte_FISD_enGetEngineOilPressureSwitch())&&\
            sys_fullon)
    {
        IND_SET(nIND_EngineOilPressureLow, nInd_On);
    }

	/*PHEV3.0 机油压力报警指示*/
	if((Rte_FISD_u8GetCONFIG_DrivingPowerType() == nPowerType_PHEV) && (Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3))
	{
		if((Rte_boGetOilPressureWarningLampTimeout() == False) && (FISD_u16SignalValue[nOilPressureWarningLamp] == 1u) && sys_fullon)
		{
        	IND_SET(nIND_EngineOilPressureLow, nInd_On);
		}

		/*PHEV3.0 机油压力报警指示自检*/
		if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
	    {
	        IND_SET(nIND_EngineOilPressureLow, nInd_On);
	    }
    }

#ifdef ENABLE_TEXT_SOUND_MIN_SHOW_2S
    /*deal with the sound warning connect to textwarn which is cancelled in 2s.*/
    if(u16TachoSpeed >= FISD_u16SelfCheckEngineSpeedThreshold)
    {
        if(nDIC_ShutEngineCheckEngineOil == TextMgr_enGetDisplayText())
        {
            BUZ_SET(nBUZ_EngineOilPressureLow,nBuz_Active);
        }
    }
#endif
    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        FISD_u32OilPressureLowTime = 0u;
		u32EngineOilPressureLowtime = 0u;
        FISD_vEngineOilPressureClear();
    }
}

static void   FISD_vEngineOilPressureClear(void)
{
    IND_SET(nIND_EngineOilPressureLow, nInd_Off);
    DIC_SET(nDIC_ShutEngineCheckEngineOil, nDic_Inactive);
    BUZ_SET(nBUZ_EngineOilPressureLow, nBuz_Inactive);
}
#undef FISD_nActive

/*******************************************************************************************
* Function: FISD_vTurnLeftDetect(void)
* Description: TurnLeftDetect
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nActive            1u

static void  FISD_vTurnBuzDiagnose(void);
static void  FISD_vTurnBuzNormal(void);
static void  FISD_vTurnBuzActive(U16 u16Left,U16 u16Right,tenBuzzerId enTick,tenBuzzerId enTock);

static void  FISD_vTurnStateDetect(void)
{
    static U16 u8TurnlightCnt = TIMECOUNT_30ms;  //IO的拉高需要比a006的状态有效信号慢才可以触发双闪，看log需要延迟30ms
    static BOOL last_flag = false;
    if((TRUE == Rte_FISD_boGetOemControlSoundSts()) && (3u == Rte_FISD_u32GetOemControlSoundData()))
    {
        FISD_vTurnBuzDiagnose();
    }
    else
    {
        FISD_vTurnBuzNormal();
    }

    if((IlGetRxBCM_4_RHTurnlightSts() == 1u) || (IlGetRxBCM_4_LHTurnlightSts() == 1u) )
    {
        if (!last_flag)
        {
            last_flag = true;
            u8TurnlightCnt = TIMECOUNT_30ms;
        }
        if(u8TurnlightCnt > 0)
        {
            u8TurnlightCnt --;
        }
        else
        {
            PW_FADELIGHT_SYNC_ON();
        }
    }
    else
    {
        if (last_flag)
        {
            last_flag = false;
            u8TurnlightCnt = TIMECOUNT_30ms;
        }
        if(u8TurnlightCnt > 0)
        {
            u8TurnlightCnt --;
        }
        else
        {
            PW_FADELIGHT_SYNC_OFF();
        }
    }
}

static void  FISD_vTurnBuzNormal(void)
{
    FISD_vTurnBuzActive(FISD_u16SignalValue[nLHTurnlightSts],FISD_u16SignalValue[nRHTurnlightSts],nBUZ_TurningTick,nBUZ_TurningTock);
}

static void FISD_vTurnBuzDiagnose(void)
{
    U16 u16Wave = Rte_FISD_boGetEntireFlashStatus(nFlash400msOn400msOff);
    FISD_vTurnBuzActive(u16Wave,u16Wave,nBUZ_TurningTick_Test,nBUZ_TurningTock_Test);
}

static BOOL boTurnTickHasActive = FALSE;

static void FISD_vTurnBuzActive(U16 u16Left,U16 u16Right,tenBuzzerId enTick,tenBuzzerId enTock)
{
    
    static U16 su16TockRestPlayCnt = TIMECOUNT_200ms;

    IND_SET(nIND_TurnLeft, nInd_Off);

    if(FISD_nActive == u16Left)
    {
        IND_SET(nIND_TurnLeft, nInd_On);
    }

    IND_SET(nIND_TurnRight, nInd_Off);

    if(FISD_nActive == u16Right)
    {
        IND_SET(nIND_TurnRight, nInd_On);
    }

    BUZ_SET(enTick, nBuz_Inactive);
    BUZ_SET(enTock, nBuz_Inactive);


    if(su16TockRestPlayCnt > 0)
    {
        su16TockRestPlayCnt --;
    }

    if((1u == u16Left) ||(1u == u16Right))
    {
        BUZ_SET(enTick, nBuz_Active);/*滴*/
        boTurnTickHasActive = TRUE;
        su16TockRestPlayCnt = TIMECOUNT_500ms;
    }
    else if((0u == u16Left) ||(0u == u16Right))
    {
        if(TRUE == boTurnTickHasActive)
        {
            BUZ_SET(enTock, nBuz_Active);/*嗒*/
        }

        if(su16TockRestPlayCnt == 0u)
        {
            boTurnTickHasActive = FALSE;
        }
    }
    else
    {
    }

    if(Rte_FISD_boIsStayInAnimationPlayStatus() == TRUE)
    {
        BUZ_SET(enTick, nBuz_Inactive);
        BUZ_SET(enTock, nBuz_Inactive);
    }
}

BOOL FISD_vTurnTickHasActive(void)
{
	return boTurnTickHasActive;
}

#undef FISD_nActive

/*******************************************************************************************
* Function: FISD_vRearFogDetect(void)
* Description: RearFogDetect
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nActive            1u
static void  FISD_vRearFogDetect(void)
{
    IND_SET(nIND_RearFog, nInd_Off);

    if(TRUE == Rte_boGetRearFogLightStsTimeout())
    {
        IND_SET(nIND_RearFog, nInd_On);
    }

    if(FISD_nActive == FISD_u16SignalValue[nRearFogLightSts])
    {
        IND_SET(nIND_RearFog, nInd_On);
    }
	#if 0
    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        IND_SET(nIND_RearFog, nInd_Off);
    }
	#endif
}
#undef FISD_nActive

/*******************************************************************************************
* Function: FISD_vCoolantStatusDetect(void)
* Description: CoolantTempHighDetect
* Parameters: none
* Return: none
********************************************************************************************/


static void FISD_vCoolantStatusDetect(void)
{
    static U16 Warm_u16TimeCnt = 0;
    FISD_vCoolantStatusClear();
    IND_SET(nIND_CoolantWhite, nInd_Off);
    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    if(Warm_u16TimeCnt > 0)
    {
        Warm_u16TimeCnt --;
    }

    if((TRUE == Rte_FISD_enBacklightInput()) || \
            ((FALSE == Rte_FISD_enBacklightInput()) && \
             (TRUE == Rte_FISD_boAnimationPlayIsFinish()) && \
             (CoolantTemp_nRed != Rte_FISD_enGetCoolTempIndicatorColor()) && \
             (TRUE == Rte_FISD_boGetTempValid())) ||\
            ((Rte_FISD_u16GetEngineSpeedDisPlayValue() <  FISD_u16SelfCheckEngineSpeedThreshold)))
    {
        IND_SET(nIND_CoolantWhite, nInd_On);
    }

    if((TRUE == Rte_FISD_enGetCoolantTempHighWarn()) && (TRUE == Rte_FISD_boGetTempValid()) && sys_fullon)
    {
        IND_SET(nIND_CoolantTempHigh, nInd_Flash_1Hz);
        IND_SET(nIND_CoolantWhite, nInd_Off);
        DIC_SET(nDIC_ShutEngineCheckCoolant, nDic_Active);
        BUZ_SET(nBUZ_CoolantTempHigh, nBuz_Active);
    }
    
	if((Rte_u16GetEngineCoolantTemperatureFailSts() == TRUE) && sys_fullon)
	{
		DIC_SET(nDIC_ShutEngineCheckCoolant, nDic_Active);
	}
    
	if(can_diag_get_sw_conf(CONF_IDX_DrivingPowerType) == nPowerType_OIL)
	{
        /*暖机中*/
        if((CoolantTemp_nWarming == Rte_FISD_enGetVehicleWarmSts()) && \
                (TRUE == Rte_FISD_boGetTempValid()) && \
                (Rte_FISD_u16GetEngineSpeedDisPlayValue() >= FISD_u16SelfCheckEngineSpeedThreshold))
        {
            IND_SET(nIND_WarmingUp, nInd_On);
            IND_SET(nIND_CoolantWhite, nInd_Off);
            DIC_SET(nDIC_Warming, nDic_Active);
            FISD_boWarmingHappenFlag = TRUE;
        }
        else if((FALSE == Rte_FISD_boGetTempValid()) || \
                (Rte_FISD_u16GetEngineSpeedDisPlayValue() < FISD_u16SelfCheckEngineSpeedThreshold))
        {
            IND_SET(nIND_WarmingUp, nInd_Off);
            DIC_SET(nDIC_Warming, nDic_Inactive);
            FISD_boWarmingHappenFlag = FALSE;
        }
        else
        {
        }

        /*暖机完成*/
        if(FALSE == DIC_CHK(nDIC_Warmed, nDic_Active))
        {
            if((TRUE == FISD_boWarmingHappenFlag) && \
                    (CoolantTemp_nWarmed == Rte_FISD_enGetVehicleWarmSts()) && \
                    (TRUE == Rte_FISD_boGetTempValid()) && \
                    (Rte_FISD_u16GetEngineSpeedDisPlayValue() >= FISD_u16SelfCheckEngineSpeedThreshold))
            {
                IND_SET(nIND_WarmingUp, nInd_Off);
                IND_SET(nIND_CoolantWhite, nInd_On);
                DIC_SET(nDIC_Warming, nDic_Inactive);
                DIC_SET(nDIC_Warmed, nDic_Active);
    			BUZ_SET(nBUZ_Warming,nBuz_Active);
                Warm_u16TimeCnt = TIMECOUNT_5s;
                FISD_boWarmingHappenFlag = FALSE;
            }
            else if((CoolantTemp_nWarmNum == Rte_FISD_enGetVehicleWarmSts()) && \
                    (TRUE == Rte_FISD_boGetTempValid()) && \
                    (Rte_FISD_u16GetEngineSpeedDisPlayValue() >= FISD_u16SelfCheckEngineSpeedThreshold))
            {
                IND_SET(nIND_WarmingUp, nInd_Off);
                DIC_SET(nDIC_Warming, nDic_Inactive);
                FISD_boWarmingHappenFlag = FALSE;
            }
            else
            {
            }
        }
        else
        {
            if((CoolantTemp_nWarmed != Rte_FISD_enGetVehicleWarmSts()) || \
                    (FALSE == Rte_FISD_boGetTempValid()) || \
                    (Rte_FISD_u16GetEngineSpeedDisPlayValue() < FISD_u16SelfCheckEngineSpeedThreshold) || \
                    (TRUE == Rte_FISD_boGetTextShowFinishFlag(nDIC_Warmed)) || \
                    (Warm_u16TimeCnt == 0))
            {
                DIC_SET(nDIC_Warmed, nDic_Inactive);
            }

            if((CoolantTemp_nWarmNum == Rte_FISD_enGetVehicleWarmSts()) || \
                    (FALSE == Rte_FISD_boGetTempValid()) || \
                    (Rte_FISD_u16GetEngineSpeedDisPlayValue() < FISD_u16SelfCheckEngineSpeedThreshold) || \
                    (TRUE == Rte_FISD_boGetTextShowFinishFlag(nDIC_Warmed)))
            {
                IND_SET(nIND_WarmingUp, nInd_Off);
                IND_SET(nIND_CoolantWhite, nInd_On);
            }
        }

	}
    if((FISD_u16SignalValue[nEngineCoolantTemperature] == 0xFF) || \
            (FISD_u16SignalValue[nEngineCoolantTemperatureFailSts] == 1u))
    {
        IND_SET(nIND_CoolantTempHigh, nInd_Flash_1Hz);
        IND_SET(nIND_CoolantWhite, nInd_Off);
        IND_SET(nIND_WarmingUp, nInd_Off);
        //DIC_SET(nDIC_ShutEngineCheckCoolant, nDic_Active);
    }
    if(TRUE == Rte_boGetEngineCoolantTemperatureTimeout())
    {
        IND_SET(nIND_CoolantTempHigh, nInd_Flash_1Hz);
        IND_SET(nIND_WarmingUp, nInd_Off);
        IND_SET(nIND_CoolantWhite, nInd_Off);
        DIC_SET(nDIC_ShutEngineCheckCoolant, nDic_Inactive);
    }
    if((FISD_boIndFuncValid[nIND_CoolantTempHigh] == TRUE) && (TRUE == FISD_boInstrumentIsInSelfCheckSts()))
    {
        IND_SET(nIND_CoolantTempHigh, nInd_On);
        IND_SET(nIND_WarmingUp, nInd_Off);
        IND_SET(nIND_CoolantWhite, nInd_Off);
    }
    
    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        FISD_vCoolantStatusClear();
        IND_SET(nIND_WarmingUp, nInd_Off);
        IND_SET(nIND_CoolantWhite, nInd_Off);
        DIC_SET(nDIC_Warming, nDic_Inactive);
        DIC_SET(nDIC_Warmed, nDic_Inactive);
        FISD_boWarmingHappenFlag = FALSE;
    }
}

static void FISD_vCoolantStatusClear(void)
{
    IND_SET(nIND_CoolantTempHigh, nInd_Off);
    DIC_SET(nDIC_ShutEngineCheckCoolant, nDic_Inactive);

    BUZ_SET(nBUZ_CoolantTempHigh, nBuz_Inactive);
	BUZ_SET(nBUZ_Warming,nBuz_Inactive);
}

/*******************************************************************************************
* Function: FISD_vFuelLevelLowDetect(void)
* Description: FuelLevelLowDetect
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nFuelLevelLowRewarningDistance 10u/*km*/

static void  FISD_vFuelLevelLowIndDetect(void);
static void  FISD_vFuelLevelLowDicDetect(void);
static void  FISD_vFuelLevelLowDetect(void)
{
    FISD_vFuelLevelLowIndDetect();
	if(FG_boGetFuelFailureSts() == FALSE)
	{
    	FISD_vFuelLevelLowDicDetect();
	}
	else
	{
		DIC_SET(nDIC_FillFuel, nDic_Inactive);
    	BUZ_SET(nBUZ_FillFuel, nBuz_Inactive);
	}
}
static void  FISD_vFuelLevelLowIndDetect(void)
{
    IND_SET(nIND_FuelLevelLow, nInd_Off);

    if((TRUE== Rte_FISD_enBacklightInput()) || \
            ((FALSE == Rte_FISD_enBacklightInput()) && \
             (TRUE == Rte_FISD_boAnimationPlayIsFinish()) ))
    {
        IND_SET(nIND_FuelWhite, nInd_On);
    }
    else
    {
        IND_SET(nIND_FuelWhite, nInd_Off);
    }

    if(FuelCircuit_Normal != Rte_FISD_GetFuelCircuit_DTC_condition())
    {
        IND_SET(nIND_FuelLevelLow, nInd_Off);
        IND_SET(nIND_FuelWhite, nInd_On);
    }
    else
    {
#if 0
        if((TRUE == Rte_FISD_boGetFuelLevelLow())&&(mc_is_polling_active(MMI_DISPLAY_FULLON_FUEL)))
#else
		if((TRUE == FG_boGetFuelLowIconWarnSts())&&(mc_is_polling_active(MMI_DISPLAY_FULLON_FUEL))&&(FG_boGetFuelFailureSts() ==FALSE))
#endif
	    {
            IND_SET(nIND_FuelLevelLow, nInd_On);
            IND_SET(nIND_FuelWhite, nInd_Off);
        }
    }

    if((FISD_boIndFuncValid[nIND_FuelLevelLow] == TRUE) && \
            (TRUE == FISD_boInstrumentIsInSelfCheckSts()))
    {
        IND_SET(nIND_FuelLevelLow, nInd_On);
        IND_SET(nIND_FuelWhite, nInd_Off);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        IND_SET(nIND_FuelLevelLow, nInd_Off);
    }
}

static void  FISD_vFuelLevelLowDicDetect(void)
{
    if(FALSE == FISD_boFuelLevelLowFlag)
    {
        DIC_SET(nDIC_FillFuel, nDic_Inactive);
        BUZ_SET(nBUZ_FillFuel, nBuz_Inactive);

    #if 0
        if(TRUE == Rte_FISD_boGetFuelLevelLow())
    #else
        if((TRUE == FG_boGetFuelLowIconWarnSts()) && (mc_is_polling_active(MMI_DISPLAY_FULLON_FUEL)))
    #endif
        {
            FISD_boFuelLevelLowFlag = TRUE;
            FISD_u32FuelLevelLowOdo = Rte_FISD_u32GetOdometer();
        }
    }
    else
    {
        DIC_SET(nDIC_FillFuel, nDic_Active);
        BUZ_SET(nBUZ_FillFuel, nBuz_Active);

    #if 0
        if(FALSE == Rte_FISD_boGetFuelLevelLow())
    #else
        if(FALSE == FG_boGetFuelLowIconWarnSts())
    #endif
        {
            FISD_boFuelLevelLowFlag = FALSE;
        }

        if(Rte_FISD_u32GetOdometer() >= FISD_u32FuelLevelLowOdo)
        {
            if((Rte_FISD_u32GetOdometer() - FISD_u32FuelLevelLowOdo) >= FISD_nFuelLevelLowRewarningDistance)
            {
                FISD_boFuelLevelLowFlag = FALSE;
            }
        }
        else
        {
            FISD_u32FuelLevelLowOdo = Rte_FISD_u32GetOdometer();
        }
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        FISD_boFuelLevelLowFlag = FALSE;
        FISD_u32FuelLevelLowOdo = Rte_FISD_u32GetOdometer();
        DIC_SET(nDIC_FillFuel, nDic_Inactive);
        BUZ_SET(nBUZ_FillFuel, nBuz_Inactive);
    }
}

/*******************************************************************************************
* Function: FISD_vStartStopSysDetect(void)
* Description: StartStopSysDetect
* Parameters: none
* Return: none
********************************************************************************************/
#define ISS_IND_MAX_FLASH_CNT   500   /*5s*/
static void FISD_vClearIssSts(void);

static void  FISD_vStartStopSysDetect(void)
{
    U8 u8IssStsGs = FISD_u16SignalValue[nISS_Sts_GS];
    static BOOL sboIssFlashActiveFlag = FALSE;
    static U16 su16IssFlashTimeCnt = 0;
    FISD_vClearIssSts();

    BOOL startstop_condition = TRUE;
    if((TRUE == Rte_FISD_boAnimationPlayIsFinish())&&startstop_condition)
    {
        switch(u8IssStsGs)
        {
        case 1:
            IND_SET(nIND_StartStopSysWork, nInd_Flash_1Hz);
            DIC_SET(nDIC_IssOfflineCheckSts, nDic_Active);
		    BUZ_SET(nBUZ_IssOfflineCheckSts,nBuz_Active);
            break;
        case 2:
            IND_SET(nIND_StartStopSysWork, nInd_On);
            break;
        case 14:
            IND_SET(nIND_StartStopSysFault, nInd_On);
            DIC_SET(nDIC_IssUnstartPlsSwitchToGearN, nDic_Active);
			BUZ_SET(nBUZ_IssUnstartPlsSwitchToGearN, nBuz_Active);
            break;
        case 15:
            IND_SET(nIND_StartStopSysFault, nInd_On);
            DIC_SET(nDIC_IssUnstartPlsStartEngineByHand, nDic_Active);
			BUZ_SET(nBUZ_IssUnstartPlsStartEngineByHand, nBuz_Active);
            break;
        case 16:
            IND_SET(nIND_StartStopSysFault, nInd_Flash_1Hz);
            DIC_SET(nDIC_IssPlsCheckIss, nDic_Active);
            break;
        case 17:
            IND_SET(nIND_StartStopSysFault, nInd_On);
            DIC_SET(nDIC_IssHasStop, nDic_Active);
			BUZ_SET(nBUZ_IssHasStop, nBuz_Active);
            break;
        case 18:
            DIC_SET(nDIC_IssHasStart, nDic_Active);
			BUZ_SET(nBUZ_IssHasStart, nBuz_Active);
            break;
        case 19:
            DIC_SET(nDIC_ISS_MHEVFailure, nDic_Active);
            IND_SET(nIND_StartStopSysFault, nInd_Flash_1Hz);
            break;
        default:
            break;
        }
#if 0
        if(9 == u8IssStsGs)
        {
            if(sboIssFlashActiveFlag == FALSE)
            {
                sboIssFlashActiveFlag = TRUE;
                su16IssFlashTimeCnt = ISS_IND_MAX_FLASH_CNT;
            }

            if(su16IssFlashTimeCnt > 0)
            {
                su16IssFlashTimeCnt --;
                IND_SET(nIND_StartStopSysWork, nInd_Flash_1Hz); /*1Hz,show 5s*/
            }
        }
        else
        {
            su16IssFlashTimeCnt = 0u;
            sboIssFlashActiveFlag = FALSE;
        }
#endif
    }
    else
    {
        su16IssFlashTimeCnt = 0u;
        sboIssFlashActiveFlag = FALSE;
    }
}

static void FISD_vClearIssSts(void)
{
    IND_SET(nIND_StartStopSysFault, nInd_Off);
    IND_SET(nIND_StartStopSysWork, nInd_Off);

    DIC_SET(nDIC_IssOfflineCheckSts, nDic_Inactive);
    DIC_SET(nDIC_IssUnstartPlsSwitchToGearN, nDic_Inactive);
    DIC_SET(nDIC_IssUnstartPlsStartEngineByHand, nDic_Inactive);
    DIC_SET(nDIC_IssPlsCheckIss, nDic_Inactive);
    DIC_SET(nDIC_IssHasStop, nDic_Inactive);
    DIC_SET(nDIC_IssHasStart, nDic_Inactive);
    DIC_SET(nDIC_ISS_MHEVFailure, nDic_Inactive);

	BUZ_SET(nBUZ_IssOfflineCheckSts,nBuz_Inactive);
	BUZ_SET(nBUZ_IssUnstartPlsSwitchToGearN,nBuz_Inactive);
	BUZ_SET(nBUZ_IssUnstartPlsStartEngineByHand, nBuz_Inactive);
	BUZ_SET(nBUZ_IssHasStop, nBuz_Inactive);
	BUZ_SET(nBUZ_IssHasStart, nBuz_Inactive);
	
}

/*******************************************************************************************
* Function: FISD_vEcoWorkDetect(void)
* Description: EcoWorkDetect
* Parameters: none
* Return: none
********************************************************************************************/
void FISD_SetDestDriveMode(U8 mode)
{
    FISD_driveMode_SetByIHU = mode;
}

U8 FISD_GetDestDriveMode(void)
{
    return FISD_driveMode_SetByIHU;
}

static void FISD_vClearDriveModeSts(void)
{
	DIC_SET(nDIC_EcoWork, nDic_Inactive);
    DIC_SET(nDIC_SportWork, nDic_Inactive);
    DIC_SET(nDIC_NormalWork, nDic_Inactive);
    DIC_SET(nDIC_SnowWork, nDic_Inactive);
    DIC_SET(nDIC_MudWork, nDic_Inactive);
    DIC_SET(nDIC_SandWork, nDic_Inactive);
	DIC_SET(nDIC_OffRoadWork, nDic_Inactive);
	//DIC_SET(nDIC_EcoPlusWork, nDic_Inactive);

    BUZ_SET(nBUZ_EcoWork, nBuz_Inactive);
    BUZ_SET(nBUZ_SportWork, nBuz_Inactive);
    BUZ_SET(nBUZ_NormalWork, nBuz_Inactive);
    BUZ_SET(nBUZ_SnowWork, nBuz_Inactive);
    BUZ_SET(nBUZ_MudWork, nBuz_Inactive);
    BUZ_SET(nBUZ_SandWork, nBuz_Inactive);
	BUZ_SET(nBUZ_OffRoadWork, nBuz_Inactive);
	//BUZ_SET(nBUZ_EcoPlusWork, nBuz_Inactive);

    IND_SET(nIND_EcoWork, nInd_Off);
    IND_SET(nIND_SportWork, nInd_Off);
    IND_SET(nIND_NormalWork, nInd_Off);	
	IND_SET(nIND_SnowWork, nInd_Off);
	IND_SET(nIND_MudWork, nInd_Off);
	IND_SET(nIND_SandWork, nInd_Off);
	IND_SET(nIND_OffRoadWork, nInd_Off);
	IND_SET(nIND_EcoPlusWork, nInd_Off);
}


static void FISD_vDriveWorkModeDetect(void)
{
    //FISD_vClearDriveModeSts();
	static U8 FISD_driveMode_SetByIHU_pre;
	static BOOL ModeChangeFlag = FALSE ;
    static U16 u16TimerCnt = 0;
    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
    if(sys_fullon == FALSE)
    {
        u16TimerCnt=0;
    }
    else
    {
        if (u16TimerCnt<TIMECOUNT_100ms)
        {
            u16TimerCnt++;
        }
    }
	if(FISD_u16DebounceCount_D1 < TIMECOUNT_100ms)
	{
		ModeChangeFlag = FALSE;
	}
	else if(FISD_driveMode_SetByIHU_pre != FISD_driveMode_SetByIHU)
	{
		ModeChangeFlag = TRUE;
	}

	if((FISD_enIgnState == TRUE) && (ModeChangeFlag == TRUE))
	{
        if(u16TimerCnt>=TIMECOUNT_100ms)
        {
            FISD_vClearDriveModeSts();
            FISD_enSetTTSBroadcastFlag(TRUE);
    		switch(FISD_driveMode_SetByIHU)
    		{
    		case enMode_ECO:
    			DIC_SET(nDIC_EcoWork, nDic_Active);
    			BUZ_SET(nBUZ_EcoWork, nBuz_Active);
    			FISD_enWorkMode = enMode_ECO;
    			break;
    		case enMode_NORMAL: 			
    			DIC_SET(nDIC_NormalWork, nDic_Active);
    			BUZ_SET(nBUZ_NormalWork, nBuz_Active);
    			FISD_enWorkMode = enMode_NORMAL;
    			break;
    		case enMode_SPORT:
    			DIC_SET(nDIC_SportWork, nDic_Active);
    			BUZ_SET(nBUZ_SportWork, nBuz_Active);
    			FISD_enWorkMode = enMode_SPORT;
    			break;
    		case enMode_SNOW:
    			DIC_SET(nDIC_SnowWork, nDic_Active);
    			BUZ_SET(nBUZ_SnowWork, nBuz_Active);
    			FISD_enWorkMode = enMode_SNOW;
    			break;
    		case enMode_MUD:
    			DIC_SET(nDIC_MudWork, nDic_Active);
    			BUZ_SET(nBUZ_MudWork, nBuz_Active);
    			FISD_enWorkMode = enMode_MUD;
    			break;
    		case enMode_SAND:
    			DIC_SET(nDIC_SandWork, nDic_Active);
    			BUZ_SET(nBUZ_SandWork, nBuz_Active);
    			FISD_enWorkMode = enMode_SAND;
    			break;
    		case enMode_OffRoad:
    			DIC_SET(nDIC_OffRoadWork, nDic_Active);
    			BUZ_SET(nBUZ_OffRoadWork, nBuz_Active);
    			FISD_enWorkMode = enMode_OffRoad;
    			break;
			case enMode_EcoPlus:
    			//DIC_SET(nDIC_EcoPlusWork, nDic_Active);
    			//BUZ_SET(nBUZ_EcoPlusWork, nBuz_Active);
    			FISD_enWorkMode = enMode_EcoPlus;
    			break;
            }
            ModeChangeFlag = False;
		}
	}
	else
	{
        if(FISD_enIgnState == False)
        {
            FISD_vClearDriveModeSts();
			FISD_enWorkMode = FISD_driveMode_SetByIHU;
        }
	}

    if(FISD_enIgnState == TRUE)
	{
		switch(FISD_driveMode_SetByIHU)
		{
		case 0:
			IND_SET(nIND_EcoWork, nInd_On);
			break;
		case 1: 			
			IND_SET(nIND_NormalWork, nInd_On);
			break;
		case 2:
			IND_SET(nIND_SportWork, nInd_On);
			break;
		case 3:
			IND_SET(nIND_SnowWork, nInd_On);
			break;
		case 4:
			IND_SET(nIND_MudWork, nInd_On);
			break;
		case 5:
			IND_SET(nIND_SandWork, nInd_On);
			break;
		case 6:
			IND_SET(nIND_OffRoadWork, nInd_On);
			break;
		case 7:
			IND_SET(nIND_EcoPlusWork, nInd_On);
			break;
		}
	}
    
	if((Rte_FISD_boAnimationPlayIsFinish() == FALSE) || (can_diag_get_sw_conf(CONF_IDX_DriveMode) == 0) ||\
		(can_diag_get_sw_conf(CONF_IDX_DriveMode) == 1) ||(can_diag_get_sw_conf(CONF_IDX_DriveMode) == 2))
	{
		FISD_vClearDriveModeSts();
		if((can_diag_get_sw_conf(CONF_IDX_DriveMode) == 1) && (Rte_FISD_boAnimationPlayIsFinish() == TRUE))
		{
			IND_SET(nIND_EcoWork, nInd_On);
		}

		if((can_diag_get_sw_conf(CONF_IDX_DriveMode) == 2) && (Rte_FISD_boAnimationPlayIsFinish() == TRUE))
		{
			IND_SET(nIND_SportWork, nInd_On);
		}
	}

	FISD_driveMode_SetByIHU_pre = FISD_driveMode_SetByIHU;
}
U8 FISD_enGetEcoSportWorkMode(void)
{
	return FISD_enWorkMode;
}
void FISD_enSetTTSBroadcastFlag(BOOL TTSBroadcastFlag)
{
    need_TTS_broadcast = TTSBroadcastFlag;
}

BOOL FISD_enGetTTSBroadcastFlag(void)
{
    return need_TTS_broadcast;
}
/*******************************************************************************************
* Function: FISD_vOverspeedDetect
* Description: Detect overspeed warning is active or not
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vClearOverspeedStatus(void);

static void FISD_vOverspeedDetect(void)
{
    U8 enOverspeedWarn = Rte_FISD_enGetOverspeedWarn();

    /*Clear Status*/
    FISD_vClearOverspeedStatus();
    if(TRUE == Rte_FISD_boAnimationPlayIsFinish())
    {
        /*Redetect Status*/
		if(1 != FISD_Get_boSlaOnOffStatus())
		{
			if(SOS_nWarnSLD == enOverspeedWarn)/*Adjustable Overspeed*/
	        {
	            IND_SET(nIND_Overspeed, nInd_On);
	            IND_SET(nIND_OverSpeedWarn,nInd_On);        //红色圆圈跳动，效果如超速报警显示方案
	            BUZ_SET(nBUZ_OVS_1, nBuz_Active);
	        }
		}
		
        if(SOS_nWarnGSO == enOverspeedWarn)/*GSO Overspeed*/
        {
            IND_SET(nIND_Overspeed, nInd_On);
            IND_SET(nIND_OverSpeedWarn,nInd_On);        //红色圆圈跳动，效果如超速报警显示方案
            BUZ_SET(nBUZ_OVS_2, nBuz_Active);
        }
        else if(SOS_nWarn120kmph == enOverspeedWarn)/*Nonadjustable Overspeed*/
        {
            IND_SET(nIND_Overspeed, nInd_On);
            IND_SET(nIND_OverSpeedWarn,nInd_On);        //红色圆圈跳动，效果如超速报警显示方案
            BUZ_SET(nBUZ_OVS_1, nBuz_Active);
        }
        else
        {
        }
    }
}

static void FISD_vClearOverspeedStatus(void)
{
    /*Clear Status*/
    IND_SET(nIND_Overspeed, nInd_Off);
    //IND_SET(nIND_OverSpeedWarn,nInd_Off);
    BUZ_SET(nBUZ_OVS_1, nBuz_Inactive);
    BUZ_SET(nBUZ_OVS_2, nBuz_Inactive);
}

/*******************************************************************************************
* Function: FISD_vDoorOpenDetect
* Description: Door open or not
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nClose  0u
#define FISD_nOpen   1u
#define FISD_nShutPower  1u
#define FISD_nDoorOpenSoundWarningThreshold  30u/*Unit:0.1km/h*/
#define FISD_nDampingFactor 10u

static void FISD_vDoorBuzDiagnose(void);
static void FISD_vCalulatePlgAngle(void);

static void FISD_vDoorOpenDetect(void)
{
    BUZ_SET(nBUZ_DoorOpen_Test, nBuz_Inactive);
    if((TRUE == Rte_FISD_boGetOemControlSoundSts()) && (2u == Rte_FISD_u32GetOemControlSoundData()))
    {
        FISD_vDoorBuzDiagnose();
    }
    else
    {
        FISD_vDoorStsDetect();
    }
}

static void FISD_vCalulatePlgAngle(void)
{
    U16 u16PlgAngleValue = 0u;

    static U8 su8TimeCount = 0u;
    static U8 su8PlgNotChangeCnt = 0u;
    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
    U16 delta = 0;\
	
    u16PlgAngleValue = FISD_u16SignalValue[nPLG_PositionSts] > 100u ? 100u : FISD_u16SignalValue[nPLG_PositionSts];
    u16PlgAngleValue = u16PlgAngleValue * 67 / 10;// 670/100
    su8TimeCount ++;

    if(su8TimeCount > 10u)
    {
        su8TimeCount = 0u;
    }
	if((FISD_u16PlgOpenSignal_Pre != FISD_u16SignalValue[nPLG_PositionSts]) || (sys_fullon == FALSE))
	{
		FISD_boPlgOpenNotChange2sFlag = FALSE;
		su8PlgNotChangeCnt = 0u;
	}
  	if((nDIC_PLG_Open == TextMgr_enGetDisplayText()) && (su8TimeCount == 10u))
    {
        if(FISD_u16VehicleSpeed >= FISD_nDoorOpenSoundWarningThreshold)
        {
            if(FISD_u16PlgHmiShowAngle_Pre < FISD_nDampingFactor)
            {
                FISD_u16PlgHmiShowAngle_Pre = u16PlgAngleValue;
            }
            else
            {
                delta = (FISD_u16PlgHmiShowAngle_Pre - 0u)/FISD_nDampingFactor;
                if(FISD_u16PlgHmiShowAngle_Pre<1*delta)
                {
                    FISD_u16PlgHmiShowAngle_Pre = 0;
                }
				else
				{
                    FISD_u16PlgHmiShowAngle_Pre -= 1*delta;
				}
            }
        }
        else
        {
            if(abs(FISD_u16PlgHmiShowAngle_Pre - u16PlgAngleValue) < FISD_nDampingFactor)
            {
                if(sys_fullon)
                {
                    if(/*FISD_u16PlgHmiShowAngle_Pre == u16PlgAngleValue &&*/                               \
                            FISD_u16PlgOpenSignal_Pre == FISD_u16SignalValue[nPLG_PositionSts])
                    {
                        if(su8PlgNotChangeCnt < TIMECOUNT_200ms) // 2s
                        {
                            su8PlgNotChangeCnt ++;
                        }
                        else
                        {
                            DIC_SET(nDIC_PLG_Open, nDic_Inactive);
                            FISD_boPlgOpenNotChange2sFlag = TRUE;
                        }
                    }
                    else
                    {
                        su8PlgNotChangeCnt = 0u;
						
                    }
                }
				else
				{
					su8PlgNotChangeCnt = 0;
				}
                FISD_u16PlgHmiShowAngle_Pre = u16PlgAngleValue;
            }
            else
            {
                su8PlgNotChangeCnt = 0u;
                
                if(u16PlgAngleValue > FISD_u16PlgHmiShowAngle_Pre)
                {
                    FISD_u16PlgHmiShowAngle_Pre += (u16PlgAngleValue - FISD_u16PlgHmiShowAngle_Pre) / FISD_nDampingFactor;
                }
                else
                {
                    FISD_u16PlgHmiShowAngle_Pre -= (FISD_u16PlgHmiShowAngle_Pre - u16PlgAngleValue) / FISD_nDampingFactor;
                }
            }
        }
    }
    else if(nDIC_PLG_Open == TextMgr_enGetDisplayText())
    {
    }
    else
    {
        FISD_u16PlgHmiShowAngle_Pre = u16PlgAngleValue;
    }

    FISD_u16PlgOpenSignal_Pre = FISD_u16SignalValue[nPLG_PositionSts];
	
}

static void FISD_vClrDoorSts(void)
{
	IND_SET(nIND_DoorOpen,nInd_Off);
    IND_SET(nIND_HoodOpen, nInd_Off);
    IND_SET(nIND_TrunkOpen, nInd_Off);
    IND_SET(nIND_DriverDoorOpen, nInd_Off);
    IND_SET(nIND_PsngrDoorOpen, nInd_Off);
    IND_SET(nIND_RLDoorOpen, nInd_Off);
    IND_SET(nIND_RRDoorOpen, nInd_Off);

    BUZ_SET(nBUZ_DoorOpen, nBuz_Inactive);
	
    DIC_SET(nDIC_CloseDoor_Avh_Epb, nDic_Inactive);
	DIC_SET(nDIC_Seatbelt_avh_epb, nDic_Inactive);
	DIC_SET(nDIC_CloseDoorSeatbelt_Avh_epb, nDic_Inactive);
	DIC_SET(nDIC_CloseDoorSeatbelt, nDic_Inactive);
	DIC_SET(nDIC_DoorOpenWithText, nDic_Inactive);
    DIC_SET(nDIC_PLG_Open, nDic_Inactive);
	DIC_SET(nDIC_DoorOpenWithNoText, nDic_Inactive);
	
}


static void FISD_SingleDoorStsDetect(tenDoorIndex doorIndex)
{
	BOOL sys_fullon = boSystemOOMFullOnMode();
	U16 speedTemp = SpeedGauge_DisplayValueGet();
  
	BOOL Dow_Active = (((FISD_u16SignalValue[nDOWWarn_L] == 2u) || (FISD_u16SignalValue[nDOWWarn_R] == 2u))?TRUE:FALSE);
	if((Rte_boGetDOWWarn_LTimeout() == TRUE) || (Rte_boGetDOWWarn_RTimeout() == TRUE))
	{
		Dow_Active = FALSE;
	}
	if(FISD_u16SignalValue[g_doorMap[doorIndex][0]] == 1)
	{
		if(Rte_FISD_boAnimationPlayIsFinish() == TRUE)
		{
			DIC_SET(nDIC_DoorOpenWithNoText, nDic_Active);

		}
		else if(sys_fullon == FALSE)
		{
        	DIC_SET(nDIC_DoorOpenWithText, nDic_Active);

             //IND_SET(nIND_DoorOpen,nInd_Off);
             if(Rte_boGetPLG_PositionStsTimeout() == TRUE)
		    {
			    IND_SET(nIND_DoorOpen,nInd_Off);
			    //IND_SET(nIND_TrunkOpen, nInd_Flash_1Hz);
		    }

        }
        
		IND_SET(g_doorMap[doorIndex][1], nInd_On);
        
        if(Rte_FISD_GetPowerState() == nPowerState_D1)  
        {
		    IND_SET(nIND_DoorOpen,nInd_On);
         
        }

		if(speedTemp >= 3)
		{
			BUZ_SET(nBUZ_DoorOpen, nBuz_Active);
		}
		if((speedTemp >= 3) || (Dow_Active == TRUE))
		{
			IND_SET(g_doorMap[doorIndex][1], nInd_Flash_1Hz);
		}
		if(((FISD_u16SignalValue[nEPBWarningMessage] == 2) || \
		(FISD_u16SignalValue[nAVHWarningMessage] == 1)) && (sys_fullon ==TRUE))
		{
			IND_SET(g_doorMap[doorIndex][1], nInd_Flash_1Hz);
		}
    }
}

static void FISD_vTrunkDoorStsDetect(void)
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	U16 speedTemp = SpeedGauge_DisplayValueGet();
	BOOL Dow_Active = (((FISD_u16SignalValue[nDOWWarn_L] == 2u) || (FISD_u16SignalValue[nDOWWarn_R] == 2u))?TRUE:FALSE);
	if((Rte_boGetDOWWarn_LTimeout() == TRUE) || (Rte_boGetDOWWarn_RTimeout() == TRUE))
	{
		Dow_Active = FALSE;
	}
	static U16 PlgDicShowCnt = 0;
	FISD_vCalulatePlgAngle();
	if(TRUE == Rte_FISD_u8GetCONFIG_PLG())/*电动尾门*/
	{
		if((FISD_u16SignalValue[nPLG_PositionSts] > 0) && (FISD_u16SignalValue[nPLG_PositionSts] <= 100))
		{
			DIC_SET(nDIC_PLG_Open, nDic_Active);
            if(sys_fullon)
            {
			    IND_SET(nIND_DoorOpen,nInd_On);
			    IND_SET(nIND_TrunkOpen, nInd_On);
            }
			if(sys_fullon)
			{
				if(speedTemp < 3)
				{
					if(FISD_boPlgOpenNotChange2sFlag == TRUE)
					{
						DIC_SET(nDIC_PLG_Open, nDic_Inactive);
					}
				}
				if((speedTemp >= 3) || (Dow_Active == TRUE) ||\
				(FISD_u16SignalValue[nEPBWarningMessage] == 2) || \
				(FISD_u16SignalValue[nAVHWarningMessage] == 1))
				{
					 IND_SET(nIND_TrunkOpen, nInd_Flash_1Hz);
				}

				
			}
			else
			{
				 PlgDicShowCnt = 0;
			}
		}
		else
		{
			PlgDicShowCnt = 0;
		}
		
		FISD_u16PlgOpenSignal_Pre = FISD_u16SignalValue[nPLG_PositionSts];
		if(Rte_boGetPLG_PositionStsTimeout() == TRUE)
		{
			IND_SET(nIND_DoorOpen,nInd_On);
			IND_SET(nIND_TrunkOpen, nInd_Flash_1Hz);
		}
	}
	else
	{
		FISD_SingleDoorStsDetect(nIndex_trunk);
	}
}

static void FISD_vOtherDoorStsDetect(void)
{
	tenDoorIndex eDoorIndex = nIndex_hood;
	
	for(;eDoorIndex < nindex_Doormax; eDoorIndex++)
	{
		if(eDoorIndex == nIndex_trunk)
		{
			continue;
		}
		
		FISD_SingleDoorStsDetect(eDoorIndex);
	}
}
#define DoorOpenWithSeatbeltWarn_Count 6
#define SeatbeltWarn_Count 4

U16 DoorOpenWithSeatbeltWarn[DoorOpenWithSeatbeltWarn_Count]= {
																nDIC_DoorOpenWithText,
																nDIC_DoorOpenWithNoText,
																nDIC_CloseDoorSeatbelt,
																nDIC_CloseDoor_Avh_Epb,
																nDIC_CloseDoorSeatbelt_Avh_epb,
																nDIC_DOW_CloseDoor,
															  };
U16 SeatbeltWarn[SeatbeltWarn_Count]= {
										nDIC_BeltWarnFastenSeatbeltWithText,
										nDIC_CloseDoorSeatbelt,
										nDIC_Seatbelt_avh_epb,
										nDIC_CloseDoorSeatbelt_Avh_epb,
									  };																
static void FISD_vDoorStsRedetect(void)
{
	U8 allDoorSts = 0;
	U8 nindex_Warn = 0;
	U8 nindex_SeatbeltWarn = 0;
	static U8 PreAllDoorSts = 0;
	static BOOL DriverSeatbeltBuckle_pre = TRUE;
	static BOOL PsngrSeatBelt_pre = FALSE; 
	BOOL DriverSeatbeltBuckle = Rte_FISD_enGetDriverSeatbeltBuckle();
	BOOL PsngrSeatBelt = ((IlGetABM_1_PsngrSeatBeltWarningRxTimeout() == FALSE) && (IlGetRxABM_1_PsngrSeatBeltWarning() == 2));  
	tenDoorIndex eDoorIndex = nIndex_hood;
	
	for(;eDoorIndex < nindex_Doormax; eDoorIndex++)
	{
        /*配置了电动车尾门后，对于TrunkSts信号不做处理，不重新触发报警*/
        if((TRUE == Rte_FISD_u8GetCONFIG_PLG()) && (eDoorIndex == nIndex_trunk))
        {
            continue;
        }
		allDoorSts |= ((1 == FISD_u16SignalValue[g_doorMap[eDoorIndex][0]]) << eDoorIndex);
	}

	if(PreAllDoorSts != allDoorSts)
	{
	
		/*4门两盖有变化，重新触发报警显示*/
		for(; nindex_Warn < DoorOpenWithSeatbeltWarn_Count; nindex_Warn++)
		{
			if(TRUE == Rte_FISD_boGetTextShowFinishFlag(DoorOpenWithSeatbeltWarn[nindex_Warn]))
	        {
	            DIC_SET(DoorOpenWithSeatbeltWarn[nindex_Warn], nDic_Inactive);/*重新触发门开显示*/
	        }
	        else
	        {
	            FISD_vSetTextShowTimeResetFlag(DoorOpenWithSeatbeltWarn[nindex_Warn]);/*延长文字的显示时间*/
	        }
		}
		
		PreAllDoorSts = allDoorSts;
	}
	if((DriverSeatbeltBuckle == FALSE) || (PsngrSeatBelt == TRUE))
	{
		if((DriverSeatbeltBuckle_pre != DriverSeatbeltBuckle) ||\
		(PsngrSeatBelt_pre != PsngrSeatBelt))
		{
			for(; nindex_SeatbeltWarn < SeatbeltWarn_Count; nindex_SeatbeltWarn++)
			{
				if(TRUE == Rte_FISD_boGetTextShowFinishFlag(SeatbeltWarn[nindex_SeatbeltWarn]))
		        {
		            DIC_SET(SeatbeltWarn[nindex_SeatbeltWarn], nDic_Inactive);/*重新触安全带显示*/
		        }
		        else
		        {
		            FISD_vSetTextShowTimeResetFlag(SeatbeltWarn[nindex_SeatbeltWarn]);/*延长安全带文字的显示时间*/
		        }
			}
			DriverSeatbeltBuckle_pre = DriverSeatbeltBuckle;
			PsngrSeatBelt_pre = PsngrSeatBelt;
		}
	}

	
}

static void FISD_vDoorStsDetect(void)
{
	FISD_vClrDoorSts();
	FISD_vTrunkDoorStsDetect();
	FISD_vOtherDoorStsDetect();
	if((Rte_FISD_boAnimationPlayIsFinish() == FALSE) && (boSystemOOMPartOnMode() == FALSE))
	{
		FISD_vClrDoorSts();
	}
}


static void FISD_vSeatbeltWithTextDetect(void)
{
	U16 speedTemp = SpeedGauge_DisplayValueGet();
	BOOL doorSts = DIC_CHK(nDIC_DoorOpenWithNoText, nDic_Active);

	if((FALSE == Rte_FISD_enGetDriverSeatbeltBuckle()) || \
	((2 == IlGetRxABM_1_PsngrSeatBeltWarning())&&\
    (FALSE==IlGetABM_1_PsngrSeatBeltWarningRxTimeout())))
	{
		if(speedTemp >=  3)
		{
			if(doorSts == FALSE)
			{
				DIC_SET(nDIC_BeltWarnFastenSeatbeltWithText, nDic_Active);
			}
		}
	}
	else
	{
		DIC_SET(nDIC_BeltWarnFastenSeatbeltWithText, nDic_Inactive);
	}
	
	if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        DIC_SET(nDIC_BeltWarnFastenSeatbeltWithText, nDic_Inactive);
    }
}



static void FISD_vDoorSeatbeltSync(void)
{
	BOOL seatbeltSts = DIC_CHK(nDIC_BeltWarnFastenSeatbeltWithText, nDic_Active);
	BOOL doorSts = DIC_CHK(nDIC_DoorOpenWithNoText, nDic_Active);
	BOOL FrontSeatbeltSts = ((!IND_CHK(nIND_Seatbelt, nInd_Off)) || (!IND_CHK(nIND_PsngerSeatBlt, nInd_Off)));
	U16 speedTemp = SpeedGauge_DisplayValueGet();
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	BOOL sys_parton = boSystemOOMPartOnMode();
	DIC_SET(nDIC_DOW_CloseDoor, nDic_Inactive);
	BUZ_SET(nBUZ_DOW_CloseDoor, nBuz_Inactive);
	
	FISD_vSeatbeltWithTextDetect();

	BOOL Dow_Active = (((FISD_u16SignalValue[nDOWWarn_L] == 2u) || (FISD_u16SignalValue[nDOWWarn_R] == 2u))?TRUE:FALSE);
	if((Rte_boGetDOWWarn_LTimeout() == TRUE) || (Rte_boGetDOWWarn_RTimeout() == TRUE))
	{
		Dow_Active = FALSE;
	}
	if(Dow_Active == TRUE)
	{
		if((FISD_u16SignalValue[nDOWWarn_L] != 3u) && (FISD_u16SignalValue[nDOWWarn_R] != 3u))
		{
			BUZ_SET(nBUZ_DOW_CloseDoor, nBuz_Active);
		}
	}
	
	if((Dow_Active == TRUE) && ((doorSts == TRUE) || ( DIC_CHK(nDIC_DoorOpenWithText, nDic_Active) == TRUE)) &&\
		((TRUE == Rte_FISD_boAnimationPlayIsFinish()) || (sys_parton == TRUE)))
	{
		DIC_SET(nDIC_DOW_CloseDoor, nDic_Active);
		DIC_SET(nDIC_DoorOpenWithNoText, nDic_Inactive);
		DIC_SET(nDIC_DoorOpenWithText, nDic_Inactive);
	}
	else if(speedTemp >= 3)
	{
		//normal door seatbelt sync
		if((doorSts == TRUE) && (FrontSeatbeltSts == FALSE))
		{
			DIC_SET(nDIC_DoorOpenWithText, nDic_Active);
			DIC_SET(nDIC_DoorOpenWithNoText, nDic_Inactive);
			doorSts = TRUE;
		}
		else if((doorSts == FALSE) && (FrontSeatbeltSts == TRUE))
		{
			/*安全带检测处会触发*/
		}
		else if((doorSts == TRUE) && (FrontSeatbeltSts == TRUE))
		{
			DIC_SET(nDIC_DoorOpenWithText, nDic_Inactive);
			DIC_SET(nDIC_DoorOpenWithNoText, nDic_Inactive);
			DIC_SET(nDIC_BeltWarnFastenSeatbeltWithText, nDic_Inactive);
			
			DIC_SET(nDIC_CloseDoorSeatbelt, nDic_Active);
		}
	}
	else
	{
		//avh issue sync
		if(((FISD_u16SignalValue[nEPBWarningMessage] == 2) || \
		(FISD_u16SignalValue[nAVHWarningMessage] == 1)) && (sys_fullon == TRUE))
		{
			//DIC_SET(nDIC_DoorOpenWithNoText, nDic_Inactive);
			if(((doorSts == FALSE) && (FrontSeatbeltSts == FALSE)) || \
			((doorSts == TRUE) && (FrontSeatbeltSts == FALSE)))
			{
				DIC_SET(nDIC_CloseDoor_Avh_Epb, nDic_Active);
			}
			else if((doorSts == FALSE) && (FrontSeatbeltSts == TRUE))
			{
				DIC_SET(nDIC_Seatbelt_avh_epb, nDic_Active);
			}
			else
			{
				DIC_SET(nDIC_CloseDoorSeatbelt_Avh_epb, nDic_Active);
			}
		}
	}
	if(sys_parton == TRUE)
	{
		if((TextMgr_enGetDisplayText() != nDIC_DoorOpenWithText) &&\
		(TextMgr_enGetDisplayText() != nDIC_PLG_Open) &&\
		(TextMgr_enGetDisplayText() != nDIC_DOW_CloseDoor)&&\
		(TextMgr_enGetDisplayText() != nDIC_Invalid))
		{
		   int index=0;
            for(index=0;index<6;index++)
            {
                if(FISD_u16SignalValue[g_doorMap[index][0]] == 1)
                {
                    IND_SET(nIND_DoorOpen,nInd_On);
                    break;
                }
            }
		}
	}
	FISD_vDoorStsRedetect();
}

static void FISD_vDoorBuzDiagnose(void)
{
    IND_SET(nIND_HoodOpen, nInd_On);
    BUZ_SET(nBUZ_DoorOpen_Test, nBuz_Active);
    DIC_SET(nDIC_DoorOpenWithText, nDic_Active);
}

/*******************************************************************************************
* Function: FISD_vRadarDetect
* Description: Detect radar warning and its sound warning
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nActive  1u


static void FISD_vRadarTextDetect(void);
static void FISD_vRadarTextClear(void);
static void FISD_vRadarSoundDetect(void);
static void FISD_vRadarSoundClear(void);

static void FISD_vRadarDetect(void)
{
    FISD_vRadarTextDetect();
    FISD_vRadarSoundDetect();
}

#define RAdARWARN_NUM 12
static void FISD_vRadarTextDetect(void)
{
	tenTextId FISD_enRadarWarnEnum[RAdARWARN_NUM] = {
		nDIC_RL_RadarProbe,nDIC_RLM_RadarProbe,nDIC_RRM_RadarProbe,nDIC_RR_RadarProbe,
		nDIC_FL_RadarProbe,nDIC_FLM_RadarProbe,nDIC_FRM_RadarProbe,nDIC_FR_RadarProbe,
		nDIC_RightRear_RadarProbe,nDIC_LeftRear_RadarProbe,nDIC_RightFront_RadarProbe,nDIC_LeftFront_RadarProbe
		};
		
    U32 u32Num = 0u;

    /*Clear Status*/
    for(u32Num = 0u; u32Num < RAdARWARN_NUM; u32Num ++)
    {
        DIC_SET(FISD_enRadarWarnEnum[u32Num],nDic_Inactive);
    }
    /*Redetect Status*/
    for(u32Num = 0u; u32Num < RAdARWARN_NUM; u32Num ++)
    {
    	if(u32Num == RADAR_enGetProbeFailureId() - 1)
    	{
            DIC_SET(FISD_enRadarWarnEnum[u32Num], nDic_Active);
        }
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        //FISD_vRadarTextClear();
	    for(u32Num = 0u; u32Num < RAdARWARN_NUM; u32Num ++)
	    {
	        DIC_SET(FISD_enRadarWarnEnum[u32Num],nDic_Inactive);
	    }
    }
}

static void FISD_vRadarTextClear(void)
{
    U32 u32Num = 0u;
    /*Clear Status*/
    for(u32Num = 0u; u32Num < RADAR_nWarn_Num; u32Num ++)
    {
        DIC_SET(nDIC_RL_RadarProbe + (u32Num-1), nDic_Inactive);
    }
}

static void FISD_vRadarSoundDetect(void)
{
    /*1 will enable normal function*/
    /*Clear Status*/
    FISD_vRadarSoundClear();

    /*Redetect Status*/
    if((1u <= RADAR_u16GetSoundInterval()) && (RADAR_u16GetSoundInterval() <= 4u))
    {
        BUZ_SET(nBUZ_Radar_PDC_1 + RADAR_u16GetSoundInterval() - 1u, nBuz_Active);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        FISD_vRadarSoundClear();
    }
}

static void FISD_vRadarSoundClear(void)
{
    BUZ_SET(nBUZ_Radar_PDC_1, nBuz_Inactive);
    BUZ_SET(nBUZ_Radar_PDC_2, nBuz_Inactive);
    BUZ_SET(nBUZ_Radar_PDC_3, nBuz_Inactive);
    BUZ_SET(nBUZ_Radar_PDC_4, nBuz_Inactive);
}

#undef FISD_nActive

/*******************************************************************************************
* Function: FISD_vBsdAndRctaDetect
* Description: BSD and RCTA Detection
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nOn  1u
#define FISD_nTemporaryError  2u
#define FISD_nSystemError  3u
#define FISD_nWarning  1u
#define FISD_nCriticalWarning 2u

static void FISD_vRctaDetect(void);
static void FISD_ClearBsdandRctaState(void);
static void FISD_vDowDetect(void);
static void FISD_vRcwDetect(void);

static void FISD_vBsdAndRctaDetect(void)
{
    FISD_ClearBsdandRctaState();

    FISD_vRcwDetect();
    //FISD_vBsdSystemDetect();
    FISD_vRctaDetect();
    FISD_vBsdSystemDetect_Ver4();
    FISD_vDowDetect();

    FISD_vRctbRaebDetect();
    FISD_vCornerRadarDetect();
    if(!IND_CHK(nIND_BlindYellow, nInd_Off))
    {
        IND_SET(nIND_BlindGreen, nInd_Off);
    }

    if(((TRUE == Rte_boGetBSDWarningLeftTimeout()) && (Rte_FISD_u8GetCONFIG_RLCR() == TRUE)) || \
		((TRUE == Rte_boGetBSDWarningRightTimeout()) && (Rte_FISD_u8GetCONFIG_RRCR() == TRUE)))
    {
        IND_SET(nIND_BlindGreen, nInd_Off);
		IND_SET(nIND_BlindYellow, nInd_Off);
		IND_SET(nIND_BlindYellow, nInd_On);
    }

    if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
    {
    	if((Rte_FISD_u8GetCONFIG_RLCR() == TRUE) || (Rte_FISD_u8GetCONFIG_RRCR() == TRUE))
    	{
        	IND_SET(nIND_BlindYellow, nInd_On);
    	}
		IND_SET(nIND_BlindGreen, nInd_Off);
    }

    
	if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
	{
        IND_SET(nIND_BlindGreen, nInd_Off);
	    IND_SET(nIND_BlindYellow, nInd_Off);
	    IND_SET(nIND_LeftBlindAreaYellow, nInd_Off);
	    IND_SET(nIND_LeftBlindLineYellow, nInd_Off);
	    IND_SET(nIND_RightBlindAreaYellow, nInd_Off);
	    IND_SET(nIND_RightBlindLineYellow, nInd_Off);
		IND_SET(nIND_BSDWarn_Left, nInd_Off);
		IND_SET(nIND_BSDWarn_Right, nInd_Off);
		DIC_SET(nDIC_CheckBlindSystem, nDic_Inactive);
	}


	if((TRUE != Rte_FISD_boAnimationPlayIsFinish()) || (boSystemOOMPartOnMode() == TRUE))
	{
		
	    DIC_SET(nDIC_WarnSwitch2ADAS, nDic_Inactive);
	    BUZ_SET(nBUZ_RCTA, nBuz_Inactive);
	    BUZ_SET(nBUZ_BSD, nBuz_Inactive);
	    BUZ_SET(nBUZ_BSD_1, nBuz_Inactive);
	    BUZ_SET(nBUZ_BSD_2, nBuz_Inactive);
	}

	
}

static void FISD_vRcwDetect(void)
{
    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    DIC_SET(nDIC_CareForRearCar, nDic_Inactive);
    BUZ_SET(nBUZ_RCW, nBuz_Inactive);
	BUZ_SET(nBUZ_CareForRearCar, nBuz_Inactive);
	IND_SET(nIND_RCWWarn, nInd_Off);
    if((Rte_FISD_boAnimationPlayIsFinish() ==TRUE)&&sys_fullon)
    {
        if(FISD_u16SignalValue[nRcwWarning] == 1u)
        {
            /*double flash(instrument doesnot need to judge)*/
			BUZ_SET(nBUZ_RCW, nBuz_Active);
			IND_SET(nIND_RCWWarn, nInd_On);
        }
        else if(FISD_u16SignalValue[nRcwWarning] == 2u)
        {
            DIC_SET(nDIC_CareForRearCar, nDic_Active);
			BUZ_SET(nBUZ_CareForRearCar, nBuz_Active);
			IND_SET(nIND_RCWWarn, nInd_Flash_1Hz);
        }
        else
        {
        }
    }
	if((IlGetRLCR_1_RCWWarnRxTimeout() == TRUE) || (IlGetRRCR_1_RCTAWarnRxTimeout() == TRUE))
	{
		BUZ_SET(nBUZ_RCW, nBuz_Inactive);
		DIC_SET(nDIC_CareForRearCar, nDic_Inactive);
		BUZ_SET(nBUZ_CareForRearCar, nBuz_Inactive);
	}
}

static void FISD_vDowDetect(void)
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	BOOL sys_parton = boSystemOOMPartOnMode();
	BOOL Dow_Active = (((FISD_u16SignalValue[nDOWWarn_L] == 2u) || (FISD_u16SignalValue[nDOWWarn_R] == 2u))?TRUE:FALSE);
	BOOL Dow_DoorLock = (((FISD_u16SignalValue[nDOWWarn_L] == 3u) || (FISD_u16SignalValue[nDOWWarn_R] == 3u))?TRUE:FALSE);
	if((Rte_boGetDOWWarn_LTimeout() == TRUE) || (Rte_boGetDOWWarn_RTimeout() == TRUE))
	{
		Dow_Active = FALSE;
		Dow_DoorLock = FALSE;
	}
	BUZ_SET(nBUZ_DOW, nBuz_Inactive);//work in d1 d2 d3
	DIC_SET(nDIC_DoorLock, nDic_Inactive);

	if((TRUE ==  Dow_DoorLock) && ((TRUE == Rte_FISD_boAnimationPlayIsFinish()) || (sys_parton == TRUE)))
	{
		DIC_SET(nDIC_DoorLock, nDic_Active);
		BUZ_SET(nBUZ_DOW, nBuz_Active);
	}
	if((IlGetRLCR_1_RCWWarnRxTimeout() == TRUE) || (IlGetRRCR_1_RCTAWarnRxTimeout() == TRUE))
	{
		DIC_SET(nDIC_DoorLock, nDic_Inactive);
		BUZ_SET(nBUZ_DOW, nBuz_Inactive);
	}
}

static void FISD_vRctaDetect(void)
{
    BOOL BsdSystemActive = (((FISD_u16SignalValue[nSysSt_L] == FISD_nOn)&&(FISD_u16SignalValue[nSysSt_R] == FISD_nOn))? TRUE:FALSE);

    BUZ_SET(nBUZ_RCTA, nBuz_Inactive);
 
    if((FISD_u16SignalValue[nRCTAWarningLeft] == FISD_nWarning) || \
      (FISD_u16SignalValue[nRCTAWarningRight] == FISD_nWarning))
    {
    	if(BsdSystemActive == TRUE)
    	{
			IND_SET(nIND_BlindYellow, nInd_Flash_2Hz);
    	}
		if(TRUE == Rte_FISD_boAnimationPlayIsFinish())
		{
        	BUZ_SET(nBUZ_RCTA, nBuz_Active);
		}
    }
	if((IlGetRLCR_1_RCWWarnRxTimeout() == TRUE) || (IlGetRRCR_1_RCTAWarnRxTimeout() == TRUE))
	{
		BUZ_SET(nBUZ_RCTA, nBuz_Inactive);
	}
}


static void FISD_ClearBsdandRctaState(void)
{
    /*Clear Status*/
    IND_SET(nIND_BlindGreen, nInd_Off);
    IND_SET(nIND_BlindYellow, nInd_Off);
    IND_SET(nIND_LeftBlindAreaYellow, nInd_Off);
    IND_SET(nIND_LeftBlindLineYellow, nInd_Off);
    IND_SET(nIND_RightBlindAreaYellow, nInd_Off);
    IND_SET(nIND_RightBlindLineYellow, nInd_Off);
	IND_SET(nIND_BSDWarn_Left, nInd_Off);
	IND_SET(nIND_BSDWarn_Right, nInd_Off);
    DIC_SET(nDIC_CheckBlindSystem, nDic_Inactive);
    DIC_SET(nDIC_WarnSwitch2ADAS, nDic_Inactive);
    BUZ_SET(nBUZ_RCTA, nBuz_Inactive);
    BUZ_SET(nBUZ_BSD, nBuz_Inactive);
    BUZ_SET(nBUZ_BSD_1, nBuz_Inactive);
    BUZ_SET(nBUZ_BSD_2, nBuz_Inactive);
}

#undef FISD_nOn
#undef FISD_nTemporaryError
#undef FISD_nSystemError
#undef FISD_nWarning
#undef FISD_nCriticalWarning

/*******************************************************************************************
* Function: FISD_vPepsWarningDetect
* Description: PEPS warning
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vBsdSystemDetect_Ver4(void)
{
	BOOL BsdSystemActive = (((FISD_u16SignalValue[nSysSt_L] == 1u)&&(FISD_u16SignalValue[nSysSt_R] == 1u))? TRUE:FALSE);
	static U8 u8BSDWarnDelayCount = 0u;
	U8 u8RL_SysSt = FISD_u16SignalValue[nSysSt_L];
	U8 u8RR_SysSt = FISD_u16SignalValue[nSysSt_R];

	if (True == FISD_u16SignalValue[nBlkSts_L] == 1 || FISD_u16SignalValue[nBlkSts_R] == 1) 
    {
        IND_SET(nIND_BlindYellow,nInd_On);
    }
    else if((0x01 == u8RL_SysSt) && (0x01 == u8RR_SysSt))
    {
        if((FISD_u16SignalValue[nBSDWarningLeft] != 2u) && (FISD_u16SignalValue[nBSDWarningLeft] != 2u))
        {
            IND_SET(nIND_BlindGreen, nInd_On);
        }
        else 
        {
            IND_SET(nIND_BlindGreen, nInd_Flash_2Hz);
        }
    }
    else if(((0x00 == u8RL_SysSt)&&(0x01 == u8RR_SysSt)) || \
			((0x01 == u8RL_SysSt)&&(0x00 == u8RR_SysSt)))
	{
		if(u8BSDWarnDelayCount++ >= TIMECOUNT_400ms)
		{
			IND_SET(nIND_BlindYellow, nInd_On);
			u8BSDWarnDelayCount = TIMECOUNT_400ms;
		}
	}
	else if(((0u == FISD_u16SignalValue[nSysSt_L])&&(0u != FISD_u16SignalValue[nSysSt_R])) || \
		   ((1u == FISD_u16SignalValue[nSysSt_L])&&(1u != FISD_u16SignalValue[nSysSt_R])) ||  \
		   (((2u == FISD_u16SignalValue[nSysSt_L])||(3u == FISD_u16SignalValue[nSysSt_L]))))
	{
		u8BSDWarnDelayCount = 0u;
		IND_SET(nIND_BlindYellow, nInd_On);
		//DIC_SET(nDIC_CheckBlindSystem, nDic_Active);
	}
    else 
    {

    }
	if((TRUE == BsdSystemActive))
	{
		if((FISD_u16SignalValue[nBSDWarningLeft] == 2u) && \
				(FISD_u16SignalValue[nBSDWarningRight] == 2u))
		{
			DIC_SET(nDIC_WarnSwitch2ADAS, nDic_Active);

			IND_SET(nIND_LeftBlindLineYellow, nInd_Off);
			IND_SET(nIND_LeftBlindAreaYellow, nInd_Flash_2Hz);

			IND_SET(nIND_RightBlindLineYellow, nInd_Off);
			IND_SET(nIND_RightBlindAreaYellow, nInd_Flash_2Hz);

			BUZ_SET(nBUZ_BSD, nBuz_Active);
            
			IND_SET(nIND_BSDWarn_Left, nInd_Flash_1Hz);
			IND_SET(nIND_BSDWarn_Right, nInd_Flash_1Hz);
            
			FISD_enCurAdasWarn = enAdasWarn_Bsd;
		}
		else if(FISD_u16SignalValue[nBSDWarningLeft] == 2u)
		{
			DIC_SET(nDIC_WarnSwitch2ADAS, nDic_Active);

			IND_SET(nIND_LeftBlindLineYellow, nInd_Off);
			IND_SET(nIND_LeftBlindAreaYellow, nInd_Flash_2Hz);

			IND_SET(nIND_RightBlindLineYellow, nInd_Off);
			IND_SET(nIND_RightBlindAreaYellow, nInd_Off);

			BUZ_SET(nBUZ_BSD, nBuz_Active);
            
		    IND_SET(nIND_BSDWarn_Left, nInd_Flash_1Hz);
            
			FISD_enCurAdasWarn = enAdasWarn_Bsd;
		}
		else if(FISD_u16SignalValue[nBSDWarningRight] == 2u)
		{
			DIC_SET(nDIC_WarnSwitch2ADAS, nDic_Active);

			IND_SET(nIND_LeftBlindLineYellow, nInd_Off);
			IND_SET(nIND_LeftBlindAreaYellow, nInd_Off);

			IND_SET(nIND_RightBlindLineYellow, nInd_Off);
			IND_SET(nIND_RightBlindAreaYellow, nInd_Flash_2Hz);

			BUZ_SET(nBUZ_BSD, nBuz_Active);
            
			IND_SET(nIND_BSDWarn_Right, nInd_Flash_1Hz);
            
			FISD_enCurAdasWarn = enAdasWarn_Bsd;
		}
		else
		{
			
		}

		if(FISD_u16SignalValue[nBSDWarningLeft] == 1u)
		{
			IND_SET(nIND_BSDWarn_Left, nInd_On);
		}

		if(FISD_u16SignalValue[nBSDWarningRight] == 1u)
		{
			IND_SET(nIND_BSDWarn_Right, nInd_On);
		}
	}
	else if(((0x00 == u8RL_SysSt)&&(0x01 == u8RR_SysSt)) || \
			((0x01 == u8RL_SysSt)&&(0x00 == u8RR_SysSt)))
	{
		if(u8BSDWarnDelayCount++ >= TIMECOUNT_400ms)
		{
			DIC_SET(nDIC_CheckBlindSystem, nDic_Active);
			u8BSDWarnDelayCount = TIMECOUNT_400ms;
		}
	}
	else if(((0u == FISD_u16SignalValue[nSysSt_L])&&(0u != FISD_u16SignalValue[nSysSt_R])) || \
		   ((1u == FISD_u16SignalValue[nSysSt_L])&&(1u != FISD_u16SignalValue[nSysSt_R])) ||  \
		   (((2u == FISD_u16SignalValue[nSysSt_L])||(3u == FISD_u16SignalValue[nSysSt_L]))))
	{
		u8BSDWarnDelayCount = 0u;
		//IND_SET(nIND_BlindYellow, nInd_On);
		//DIC_SET(nDIC_CheckBlindSystem, nDic_Active);
	}
	else
	{
		u8BSDWarnDelayCount = 0u;
	}
	if(((0x02 == u8RL_SysSt)&&((0x02 == u8RR_SysSt) || (0x03 == u8RR_SysSt))) ||\
	((0x03 == u8RL_SysSt)&&((0x02 == u8RR_SysSt) || (0x03 == u8RR_SysSt))))
	{
		DIC_SET(nDIC_CheckBlindSystem, nDic_Active);
	}

	if((IlGetRLCR_1_RCWWarnRxTimeout() == TRUE) || (IlGetRRCR_1_RCTAWarnRxTimeout() == TRUE))
	{
		u8BSDWarnDelayCount = 0u;
		DIC_SET(nDIC_CheckBlindSystem, nDic_Inactive);
	}
#ifdef ENABLE_TEXT_SOUND_MIN_SHOW_2S
	/*deal with the sound warning connect to textwarn which is cancelled in 2s.*/
	if(nDIC_WarnSwitch2ADAS == TextMgr_enGetDisplayText())
	{
		//BUZ_SET(nBUZ_BSD,nBuz_Active);
	}
#endif

	if(TRUE == Rte_FISD_boAnimationPlayIsFinish())
	{
		/*盲区监测车道线转到ADAS车道线上*/
		if(FALSE == IND_CHK(nIND_LeftBlindLineYellow,nInd_Off))
		{
			IND_SET(nIND_LDW_LL_White, nInd_Off);
			IND_SET(nIND_LDW_LL_Grey, nInd_Off);
			IND_SET(nIND_LDW_LL_Green, nInd_Off);
			IND_SET(nIND_LDW_LL_Yellow, nInd_On);
		}

		if(FALSE == IND_CHK(nIND_RightBlindLineYellow,nInd_Off))
		{
			IND_SET(nIND_LDW_RL_White, nInd_Off);
			IND_SET(nIND_LDW_RL_Grey, nInd_Off);
			IND_SET(nIND_LDW_RL_Green, nInd_Off);
			IND_SET(nIND_LDW_RL_Yellow, nInd_On);
		}
	}
}

static void FISD_vRctbRaebDetect(void)
{
    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	BOOL BsdSystemActive = (((FISD_u16SignalValue[nSysSt_L] == 1)&&(FISD_u16SignalValue[nSysSt_R] == 1))? TRUE:FALSE);

    DIC_SET(nDIC_ReabRctWarning, nDic_Inactive);


    if((Rte_FISD_boAnimationPlayIsFinish() ==TRUE)&&(sys_fullon == TRUE))
    {
        if((FISD_u16SignalValue[nRAEBWarn] == 1u)||(FISD_u16SignalValue[nRCTBWarn] == 1u))
        {
            DIC_SET(nDIC_ReabRctWarning, nDic_Active);
        }
        else
        {
        }
    }
}

static void FISD_vCornerRadarDetect(void)
{
    BOOL sys_parton = boSystemOOMPartOnMode();

    DIC_SET(nDIC_BlkStsLeft, nDic_Inactive);
    DIC_SET(nDIC_BlkStsRight, nDic_Inactive);


    if(Rte_FISD_boAnimationPlayIsFinish() ==TRUE)
    {
        if(FISD_u16SignalValue[nBlkSts_L] == 1u)
        {
            DIC_SET(nDIC_BlkStsLeft, nDic_Active);
        }

        if(FISD_u16SignalValue[nBlkSts_R] == 1u)
        {
            DIC_SET(nDIC_BlkStsRight, nDic_Active);
        }
    }
}

#define TSISgnLongStayDisp_NUM  8
U16 TSISgnLongStayDispBuz[TSISgnLongStayDisp_NUM] = { nBUZ_NoOvertake,    /*禁止超车*/
													  nBUZ_EndOfNoOvertake,    /*解除禁止超车*/
													  nBUZ_NoParking,                       /*交通标识识别禁止停车，info一声*/
													  nBUZ_ProhibStandAndPark,              /*禁止临时及长时停车*/
    												  nBUZ_AustralianNoOvertak,             /*澳大利亚禁止超车*/
    											      nBUZ_AustralianProhibitStandAndPark,  /*澳大利亚禁止临时及长时停车*/
    												  nBUZ_AustralianNoPark,                /*澳大利亚禁止停车*/
    												  nBUZ_MalaysiaNoOvertak,               /*马来西亚禁止超车*/
													};


static void FISD_vNotParkingDetect(void)
{
    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	U8 u8_NUM;
	U16 TSISgnLongStayDisp = FISD_u16SignalValue[nTSISgnLongStayDisp];
	for(u8_NUM = 0 ; u8_NUM < TSISgnLongStayDisp_NUM ;u8_NUM++)
	{
		 BUZ_SET(TSISgnLongStayDispBuz[u8_NUM],nBuz_Inactive);
	}
   


    if((Rte_FISD_boAnimationPlayIsFinish() ==TRUE)&&sys_fullon)
    {
        if((1u == FISD_u16SignalValue[nTSINoParkWarn]) && \
           ((( TSISgnLongStayDisp > 0))&&((TSISgnLongStayDisp < 9))))
        {
            BUZ_SET(TSISgnLongStayDispBuz[TSISgnLongStayDisp -1],nBuz_Active);
        }
    }
}


/*******************************************************************************************
* Function: FISD_vPepsWarningDetect
* Description: PEPS warning
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vClearPepsStatus(void);
static void FISD_vCheckPepsStatus(void);

static void FISD_vPepsWarningDetect(void)
{
    FISD_vClearPepsStatus();
    FISD_vCheckPepsStatus();
}

static void FISD_vCheckPepsStatus(void)
{
		U32 u32Num = 0u;
	if(((Rte_FISD_boAnimationPlayIsFinish() == TRUE) || (boSystemOOMPartOnMode() == TRUE)) && (Rte_boGetSmartSystemWarning_2_2Timeout() == FALSE))
	{
		#if 0
		for(u32Num = 0u; u32Num < PEPS_nWarnNum; u32Num ++)
		{
			if(u32Num == Rte_FISD_enGetWarnId())
			{
				switch(u32Num)
				{
				case PEPS_nNoSmartKeyDetected2_2:
					DIC_SET(nDIC_NoSmartKeyDetected2_2, nDic_Active);
					break;
				case PEPS_nNoSmartKeyDetected1_1:
					DIC_SET(nDIC_NoSmartKeyDetected1_1, nDic_Active);
					break;
				case PEPS_nShiftToP_N:
					DIC_SET(nDIC_ShiftToP_N, nDic_Active);
					break;
				case PEPS_nStartStepClutchPedal:
					DIC_SET(nDIC_StartStepClutchPedal, nDic_Active);
					break;
				case PEPS_nStartStepBrakePedal:
					DIC_SET(nDIC_StartStepBrakePedal, nDic_Active);
					break;
				case PEPS_nShiftToP:
					DIC_SET(nDIC_ShiftToP, nDic_Active);
					break;
				case PEPS_nCautionSmartKeyInCar:
					DIC_SET(nDIC_CautionSmartKeyInCar, nDic_Active);
					break;
				case PEPS_nShutPower:
					DIC_SET(nDIC_ShutPower, nDic_Active);
					break;
				case PEPS_nNoSmartKeyDetected2_0:
					DIC_SET(nDIC_NoSmartKeyDetected2_0, nDic_Active);
					break;
				case PEPS_nSmartKeyPowerLow:
					DIC_SET(nDIC_SmartKeyPowerLow, nDic_Active);
					break;
				default:
					break;
				}
			}
		}
		#endif
		
		if(Rte_boGetSmartSystemWarning_2_2Timeout() == FALSE)
		{
			if(Rte_PEPS_u16GetSmartSystemWarning2_2() == 1u)
			{		
				BUZ_SET(nBUZ_NoSmartKeyDetected2_2, nBuz_Active);
				DIC_SET(nDIC_NoSmartKeyDetected2_2, nDic_Active);
			}
			if(Rte_PEPS_u16GetSmartSystemWarning1_3() == 1u)
			{
				BUZ_SET(nBUZ_CautionSmartKeyInCar, nBuz_Active);
				DIC_SET(nDIC_CautionSmartKeyInCar, nDic_Active);
			}
			if(Rte_PEPS_u16GetSmartSystemWarning1_1() == 1u)
			{
				BUZ_SET(nBUZ_NoSmartKeyDetected1_1, nBuz_Active);
				DIC_SET(nDIC_NoSmartKeyDetected1_1, nDic_Active);
			}
			if(Rte_PEPS_u16GetSmartSystemWarning2_0() == 1u)
			{
				BUZ_SET(nBUZ_NoSmartKeyDetected2_0, nBuz_Active);

				DIC_SET(nDIC_NoSmartKeyDetected2_0, nDic_Active);
			}
			if(Rte_PEPS_u16GetSmartSystemWarning4_3() == 1u)
	    	{
				BUZ_SET(nBUZ_ShiftToP, nBuz_Active);
				DIC_SET(nDIC_ShiftToP, nDic_Active);
	   		}
		    /*启动请踩离合/刹车踏板*/
		    if(Rte_PEPS_u16GetSmartSystemWarning4_2() == 1u)
		    {
		        if(True == Rte_PEPS_u8GetCONFIG_NoTcu1NoTcu2NoTcu3NoTcu4())/*(MT))*/
		        {
					BUZ_SET(nBUZ_StartStepClutchPedal, nBuz_Active);
					DIC_SET(nDIC_StartStepClutchPedal, nDic_Active);
		        }
		        if(True == Rte_PEPS_u8GetCONFIG_Tcu1OrTcu2OrTcu3OrTcu4())/*(AT)*/
		        {
					BUZ_SET(nBUZ_StartStepBrakePedal, nBuz_Active);
					DIC_SET(nDIC_StartStepBrakePedal, nDic_Active);
		        }

		    }
		    /*请挂入P或N档启动PEPS3_2*/
		    if(Rte_PEPS_u16GetSmartSystemWarning3_2() == 1u)
		    {
				BUZ_SET(nBUZ_ShiftToP_N, nBuz_Active);
				DIC_SET(nDIC_ShiftToP_N, nDic_Active);
		    }
			 /*智能钥匙电量低*/
		    if(Rte_PEPS_u16GetSmartSystemWarning3_1() == 1u)
		    {
		    	DIC_SET(nDIC_SmartKeyPowerLow, nDic_Active);
				BUZ_SET(nBUZ_SmartKeyPowerLow, nBuz_Active);
		    }


		    /*请关闭电源PEPS1.0*/
		    if(Rte_PEPS_u16GetSmartSystemWarning1_0() == 1u)
		    {
				BUZ_SET(nBUZ_ShutPower, nBuz_Active);
				DIC_SET(nDIC_ShutPower, nDic_Active);
		    }
		}
	
	}


		#if 0
	//#ifdef ENABLE_TEXT_SOUND_MIN_SHOW_2S
		/*deal with the sound warning connect to textwarn which is cancelled in 2s.*/
		if(nDIC_NoSmartKeyDetected2_2 == TextMgr_enGetDisplayText())
		{
			BUZ_SET(nBUZ_NoSmartKeyDetected2_2,nBuz_Active);
		}
		else
		{
			if(nDIC_NoSmartKeyDetected1_1  == TextMgr_enGetDisplayText())
			{
				BUZ_SET(nBUZ_NoSmartKeyDetected1_1, nBuz_Active);
			}
			else if(nDIC_ShiftToP_N  == TextMgr_enGetDisplayText())
			{
				BUZ_SET(nBUZ_ShiftToP_N, nBuz_Active);
			}
			else if(nDIC_StartStepClutchPedal  == TextMgr_enGetDisplayText())
			{
				BUZ_SET(nBUZ_StartStepClutchPedal, nBuz_Active);
			}
			else if(nDIC_StartStepBrakePedal  == TextMgr_enGetDisplayText())
			{
				BUZ_SET(nBUZ_StartStepBrakePedal, nBuz_Active);
			}
			else if(nDIC_ShiftToP  == TextMgr_enGetDisplayText())
			{
				BUZ_SET(nBUZ_ShiftToP, nBuz_Active);
			}
			else if(nDIC_CautionSmartKeyInCar  == TextMgr_enGetDisplayText())
			{
				BUZ_SET(nBUZ_CautionSmartKeyInCar, nBuz_Active);
			}
			else if(nDIC_ShutPower  == TextMgr_enGetDisplayText())
			{
				BUZ_SET(nBUZ_ShutPower, nBuz_Active);
			}
			else if(nDIC_NoSmartKeyDetected2_0  == TextMgr_enGetDisplayText())
			{
				BUZ_SET(nBUZ_NoSmartKeyDetected2_0, nBuz_Active);
			}
			else if(nDIC_SmartKeyPowerLow  == TextMgr_enGetDisplayText())
			{
				BUZ_SET(nBUZ_SmartKeyPowerLow, nBuz_Active);
			}
		}
	   #endif
		if(Rte_FISD_boIsStayInAnimationPlayStatus() == TRUE)
		{
			FISD_vClearPepsStatus();
		}
	
		if(TRUE == Rte_boGetSmartSystemWarning_1_0Timeout())
		{
			//DIC_SET(nDIC_PEPS_Offline, nDic_Active);
		}
	
		if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
		{
			//DIC_SET(nDIC_PEPS_Offline, nDic_Inactive);
		}
}


static void FISD_vClearPepsStatus(void)
{
    DIC_SET(nDIC_CautionSmartKeyInCar, nDic_Inactive);
    DIC_SET(nDIC_NoSmartKeyDetected2_2, nDic_Inactive);
    DIC_SET(nDIC_ShiftToP, nDic_Inactive);
    DIC_SET(nDIC_StartStepClutchPedal, nDic_Inactive);
    DIC_SET(nDIC_StartStepBrakePedal, nDic_Inactive);
    DIC_SET(nDIC_ShiftToP_N, nDic_Inactive);
    DIC_SET(nDIC_SmartKeyPowerLow, nDic_Inactive);
    DIC_SET(nDIC_ShutPower, nDic_Inactive);
    DIC_SET(nDIC_NoSmartKeyDetected2_0, nDic_Inactive);
    DIC_SET(nDIC_NoSmartKeyDetected1_1, nDic_Inactive);

    BUZ_SET(nBUZ_NoSmartKeyDetected2_0, nBuz_Inactive);
    BUZ_SET(nBUZ_NoSmartKeyDetected1_1, nBuz_Inactive);
    BUZ_SET(nBUZ_CautionSmartKeyInCar, nBuz_Inactive);
    BUZ_SET(nBUZ_NoSmartKeyDetected2_2, nBuz_Inactive);
    BUZ_SET(nBUZ_ShiftToP, nBuz_Inactive);
    BUZ_SET(nBUZ_StartStepClutchPedal, nBuz_Inactive);
    BUZ_SET(nBUZ_StartStepBrakePedal, nBuz_Inactive);
    BUZ_SET(nBUZ_ShiftToP_N, nBuz_Inactive);
    BUZ_SET(nBUZ_SmartKeyPowerLow, nBuz_Inactive);

    BUZ_SET(nBUZ_ShutPower, nBuz_Inactive);
}

/*******************************************************************************************
* Function: FISD_vFatigueDetect
* Description: Fatigue detection
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vFatigueDetect(void)
{
    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    /*Clear Status*/
    IND_SET(nIND_FatigueReminder, nInd_Off);
    DIC_SET(nDIC_FatigueReminder, nDic_Inactive);
    BUZ_SET(nBUZ_FatigueReminder, nBuz_Inactive);

    /*Redetect Status*/
    if(sys_fullon&&Rte_FISD_boGetFatigueFlag())
    {
        IND_SET(nIND_FatigueReminder, nInd_On);
        DIC_SET(nDIC_FatigueReminder, nDic_Active);
        BUZ_SET(nBUZ_FatigueReminder, nBuz_Active);
    }
	
	if(can_diag_get_sw_conf(CONF_IDX_Fatiguemonitoring) == 0x01)
	{
		IND_SET(nIND_FatigueReminder, nInd_Off);
    	DIC_SET(nDIC_FatigueReminder, nDic_Inactive);
   		BUZ_SET(nBUZ_FatigueReminder, nBuz_Inactive);
	}
}
/*******************************************************************************************
* Function: FISD_vAvmWarningDetect
* Description: Detect AVM warning
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nFailure  1u
static void FISD_vAvmWarningClear(void);

static void FISD_vAvmWarningDetect(void)
{	
	extern U8 avm_front_camera_state;
	extern U8 avm_rear_camera_state;
	extern U8 avm_left_camera_state;
	extern U8 avm_right_camera_state;
	U8 u8avmType = can_diag_get_sw_conf(CONF_IDX_AVMIntergatedMethod);
    /*Clear Status*/
    FISD_vAvmWarningClear();

    if((RPC_GetAVM_Front_Camera_state()) || ((Rte_boGetAVMFaultStatusCameraFrontTimeout() == FALSE) && (FISD_u16SignalValue[nAVMFaultStatusCameraFront])))
    {
    	DIC_SET(nDIC_CheckFrontCamera, nDic_Active);
    }

	if((RPC_GetAVM_Left_Camera_state()) || ((Rte_boGetAVMFaultStatusCameraFrontTimeout() == FALSE) && (FISD_u16SignalValue[nAVMFaultStatusCameraLeft])))
	{
    	DIC_SET(nDIC_CheckLeftCamera, nDic_Active);
    }


	if((RPC_GetAVM_Right_Camera_state()) || ((Rte_boGetAVMFaultStatusCameraFrontTimeout() == FALSE) && (FISD_u16SignalValue[nAVMFaultStatusCameraRight])))
    {
    	DIC_SET(nDIC_CheckRightCamera, nDic_Active);
    }

	if((RPC_GetAVM_Rear_Camera_state()) || ((Rte_boGetAVMFaultStatusCameraFrontTimeout() == FALSE) && (FISD_u16SignalValue[nAVMFaultStatusCameraRear])))
    {
    	DIC_SET(nDIC_CheckRearCamera, nDic_Active);
    }
        
	if((Rte_boGetAVMFaultStatusCameraFrontTimeout() == FALSE) && (FISD_u16SignalValue[nAVMStatusFault]))
	{
		DIC_SET(nDIC_CheckAvmSystem, nDic_Active);	
	}
#if 0 
    /*Redetect Status*/
    for(u32Num = 0u; u32Num <= u32Interval; u32Num ++)
    {
        if(FISD_u16SignalValue[nAVMStatusFault + u32Num] == FISD_nFailure)
        {
            DIC_SET(nDIC_CheckAvmSystem + u32Num, nDic_Active);
        }
    }
	
#endif

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        FISD_vAvmWarningClear();
    }

	if(FISD_u16DebounceCount_D1 < TIMECOUNT_5s)
	{
		FISD_vAvmWarningClear();
	}
	
	if((FISD_u16DebounceCount_D1 > TIMECOUNT_5s)&& (u8avmType == 2))
	{
		rte_can_sig_send_AVMFaultStatusCameraRight_Infocan(&avm_right_camera_state, 1);
		rte_can_sig_send_AVMFaultStatusCameraRear_Infocan(&avm_rear_camera_state, 1);
		rte_can_sig_send_AVMFaultStatusCameraLeft_Infocan(&avm_left_camera_state, 1);
		rte_can_sig_send_AVMFaultStatusCameraFront_Infocan(&avm_front_camera_state, 1);
	}
}

static void FISD_vAvmWarningClear(void)
{
    U32 u32Num = 0u;
    U32 u32Interval = nDIC_CheckRearCamera - nDIC_CheckAvmSystem;

    for(u32Num = 0u; u32Num <= u32Interval; u32Num ++)
    {
        DIC_SET(nDIC_CheckAvmSystem + u32Num, nDic_Inactive);
    }
}

#undef FISD_nFailure

/*******************************************************************************************
* Function: FISD_vFollowMeDetect
* Description: Detect follow me
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nFollowMeFactor  3000u /*Unit:10ms*/
#define FISD_nTurnOffLcdTime  2000u /*Unit:10ms*/

static void FISD_vFollowMeDetect(void)
{
    U32 u32FollowMeTime = 0u;
    U8 is_key_off = (FALSE == IO_GET_PS2uP_ON_OFF_LOGIC());
    /*Redetect Status*/
    if((FISD_enIgnState == FALSE) && \
            (1u <= FISD_u16SignalValue[nFollowMeTimeSts]) && \
            (FISD_u16SignalValue[nFollowMeTimeSts] <= 8u) && \
            (is_key_off))
    {
 
		
        /*剩余倒计时时间的判断*/
        if(FISD_u16SignalValue[nFollowMeTimeSts] < FISD_u16FollowMeLastValue)
        {
            FISD_u32FollowMeElapseTime = 0u;
        }

        u32FollowMeTime = FISD_u16SignalValue[nFollowMeTimeSts] * FISD_nFollowMeFactor;
		
		

        if(FISD_u32FollowMeElapseTime < u32FollowMeTime)
        {
            FISD_u32FollowMeElapseTime ++;
        }
        else if(FISD_u32FollowMeElapseTime > u32FollowMeTime)
        {
            FISD_u32FollowMeElapseTime = 0u;
        }
        else
        {
        }

		FISD_u32FollowMeRestTime = u32FollowMeTime - FISD_u32FollowMeElapseTime; /*Unit:10ms*/
        

        /*LCD显示时间的判断*/
        if(FISD_u16SignalValue[nFollowMeTimeSts] >= (FISD_u16FollowMeLastValue + 1u))
        {
            //FISD_u32TurnOffLcdTime = FISD_nTurnOffLcdTime;
            FISD_u32TurnOffLcdTime += (FISD_u16SignalValue[nFollowMeTimeSts] - FISD_u16FollowMeLastValue)*FISD_nFollowMeFactor;
        }
        else if(FISD_u16SignalValue[nFollowMeTimeSts] < FISD_u16FollowMeLastValue)
        {
            //FISD_u32TurnOffLcdTime = FISD_nTurnOffLcdTime;
            FISD_u32TurnOffLcdTime = FISD_u16SignalValue[nFollowMeTimeSts]*FISD_nFollowMeFactor;
        }
        else
        {
        }

        if(FALSE == FISD_boFollowMeActiveFlag)
        {
            DIC_SET(nDIC_FollowMeHome, nDic_Inactive);

            if(FISD_u16SignalValue[nFollowMeTimeSts] != FISD_u16FollowMeLastValue)
            {
                FISD_boFollowMeActiveFlag = TRUE;
                DIC_SET(nDIC_FollowMeHome, nDic_Active);
            }
        }
        else
        {
            DIC_SET(nDIC_FollowMeHome, nDic_Active);/*显示FollowMeHome界面*/

            FISD_u32TurnOffLcdTime --;

            if(0u == FISD_u32TurnOffLcdTime)
            {
                FISD_boFollowMeActiveFlag = FALSE;
                DIC_SET(nDIC_FollowMeHome, nDic_Inactive);
            }
        }

        FISD_u16FollowMeLastValue = FISD_u16SignalValue[nFollowMeTimeSts];
    }
    else
    {
        //FISD_u32FollowMeRestTime = 0u;
		FISD_u32FollowMeElapseTime = 0u;
        FISD_u32TurnOffLcdTime = 0u;
        FISD_boFollowMeActiveFlag = FALSE;
        FISD_u16FollowMeLastValue = 0u;
        DIC_SET(nDIC_FollowMeHome, nDic_Inactive);
    }
}

#undef FISD_nFollowMeFactor

/*******************************************************************************************
* Function: FISD_vKeyInCarDetect
* Description: Detect key is in car or not.
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nActive  1u

static void FISD_vKeyInCarDetect(void)
{
    /*Clear Status*/
    BUZ_SET(nBUZ_KeyInCar, nBuz_Inactive);

    /*Redetect Status*/
    if((FISD_u16SignalValue[nKeyRemindWarning] == FISD_nActive) && \
            (FISD_enIgnState == FALSE) && \
            (nOFF == FISD_u16SignalValue[nKeySts]))
    {
        BUZ_SET(nBUZ_KeyInCar, nBuz_Active);
    }
}

#undef FISD_nActive

/*******************************************************************************************
* Function: FISD_vMaintenanceClear
* Description: 
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vMaintenanceClear(void)
{
    DIC_SET(nDIC_IntervalTime_Warning, nDic_Inactive);
	DIC_SET(nDIC_IntervalDistance_Warning, nDic_Inactive);
	DIC_SET(nDIC_Interval_Warning, nDic_Inactive);
	DIC_SET(nDIC_MaintainVehicle, nDic_Inactive);
	IND_SET(nIND_Mainten, nInd_Off);
	BUZ_SET(nBUZ_MaintainVehicle, nBuz_Inactive);
}

/*******************************************************************************************
* Function: FISD_vLdwLkaDetect
* Description: Detect LDW warning is happened or not.
* Parameters: none
* Return: none
********************************************************************************************/
//static void FISD_vLdwDelay(void);
static void FISD_vLdwLkaLaneStatusDetect(void);
static void FISD_vPleaseTakeOverTurn(void);
static void FISD_vLdwIndicatorDetect(void);
static void FISD_vLkaIndicatorDetect(void);
static void FISD_vClearLdwStatus(void);
static void FISD_vLdwLkaDetect(void)
{
    BOOL sys_fullon = boSystemOOMFullOnMode();

    BUZ_SET(nBUZ_LDWLKA_StatusChange, nBuz_Inactive);
    BUZ_SET(nBUZ_TJAICA_StatusChange, nBuz_Inactive);

    /*Redetect Status*/
    if(TRUE == Rte_FISD_boAnimationPlayIsFinish()  &&  TRUE == Rte_FISD_u8GetCONFIG_FCM())
    {
        /*IND*/
        switch(FISD_u16SignalValue[nReserved_LDWLKA_LaneAssitTypefeedback])
        {
        case 1u:                             //车道偏离
            FISD_vClearLdwStatus();
            FISD_vLdwIndicatorDetect();
            break;
        case 2u:                            //车道保持
            FISD_vClearLdwStatus();
            FISD_vLkaIndicatorDetect();
            break;
        default:                            // all not show on
            FISD_vClearLdwStatus();
            break;

        }

        /*DIC*/
        FISD_vLdwLkaLaneStatusDetect();

        FISD_vPleaseTakeOverTurn();
        if((FISD_u8LeftAreaValue != 0u) || (FISD_u8RightAreaValue != 0u))
        {
            if(FISD_enCurAdasWarn != enAdasWarn_Bsd)
            {
                FISD_enCurAdasWarn = enAdasWarn_LdwLka;
            }
        }
        if(((FISD_u16DebounceCount_D1 < TIMECOUNT_3s)) ||
                (TRUE == Rte_boGetHMA_StatusTimeout() && TRUE == Rte_boGetLDW_LKA_StatusTimeout()))
        {
            /*LDW*/
			if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
			{
				IND_SET(nIND_LDW_Green, nInd_Off);
                /*配置 FCM 存在或 IDCU 存在时自检*/
                if(Rte_FISD_u8GetCONFIG_FCM()!= 0 || Rte_FISD_u8GetCONFIG_IDCU()!=0)
                {
				    IND_SET(nIND_LDW_Yellow, nInd_On);
                }
                else
                {
                    IND_SET(nIND_LDW_Yellow, nInd_Off);
                }
				IND_SET(nIND_LDW_Grey, nInd_Off);
				IND_SET(nIND_LDW_OFF, nInd_Off);
			
				/*LKA*/
				IND_SET(nIND_LDW_LKA_Green, nInd_Off);
                /*配置 (FCM 存在且 (ELK /LDP 存在)) 或 IDCU 存在时自检*/
                if((Rte_FISD_u8GetCONFIG_FCM()!= 0 
                    && (Rte_FISD_u8GetCONFIG_LDP() != 0 || Rte_FISD_u8GetCONFIG_ELK() != 0) )
                   ||Rte_FISD_u8GetCONFIG_IDCU()!= 0)
                {
				    IND_SET(nIND_LDW_LKA_Yellow, nInd_On);
                }
                else
                {
                    IND_SET(nIND_LDW_LKA_Yellow, nInd_Off);
                }
				IND_SET(nIND_LDW_LKA_Grey, nInd_Off);
				IND_SET(nIND_LDP_ELK_OFF, nInd_Off);
			}
            else if(((FISD_u16DebounceCount_D1 >= TIMECOUNT_3s)))
            {
                if((TRUE == Rte_boGetHMA_StatusTimeout() && TRUE == Rte_boGetLDW_LKA_StatusTimeout()))
                {
                	if(Rte_FISD_u8GetCONFIG_LDW()&&Rte_FISD_u8GetCONFIG_FCM())
                	{
                    	IND_SET(nIND_LDW_Yellow, nInd_On);
                	}
					if((Rte_FISD_u8GetCONFIG_LDP() == TRUE ) || (Rte_FISD_u8GetCONFIG_ELK() == TRUE))
					{
						IND_SET(nIND_LDW_LKA_Yellow, nInd_On);
					}
                    IND_SET(nIND_LDW_LL_White, nInd_Off);
                    IND_SET(nIND_LDW_LL_Grey, nInd_Off);
                    IND_SET(nIND_LDW_LL_Yellow, nInd_On);
                    IND_SET(nIND_LDW_RL_White, nInd_Off);
                    IND_SET(nIND_LDW_RL_Grey, nInd_Off);
                    IND_SET(nIND_LDW_RL_Yellow, nInd_On);
                    IND_SET(nIND_LDW_LL_Green,nInd_Off);
                    IND_SET(nIND_LDW_RL_Green,nInd_Off);
                }
                else
                {
                    FISD_vClearLdwStatus();
                }
            }
        }
    }
    else
    {
        FISD_vClearLdwStatus();
    }
}

static void FISD_vPleaseTakeOverTurn(void)
{

}

static void FISD_vLdwLkaLaneStatusDetect(void)
{
    U8 u8LdwLka_LeftVisual = FISD_u16SignalValue[nLDW_LKA_LeftVisualization];
    U8 u8LdwLka_RightVisual = FISD_u16SignalValue[nLDW_LKA_RightVisualization];

    if(u8LdwLka_LeftVisual == 0x03 || u8LdwLka_RightVisual == 0x03)
    {
        if(u8LdwLka_LeftVisual == 0x03)
        {
            IND_SET(nIND_LDW_LL_Yellow,nInd_Flash_2Hz);
            FISD_u8LeftAreaValue = 3u;
            switch(u8LdwLka_RightVisual)
            {
            case 0:
                IND_SET(nIND_LDW_RL_Grey,nInd_On);
                FISD_u8RightAreaValue = 0u;
                break;
            case 1:
                IND_SET(nIND_LDW_RL_White,nInd_On);
                FISD_u8RightAreaValue = 1u;
                break;
            case 2:
                IND_SET(nIND_LDW_RL_Green,nInd_On);
                FISD_u8RightAreaValue = 2u;
                break;
            case 3:
                IND_SET(nIND_LDW_RL_Yellow,nInd_Flash_2Hz);
                FISD_u8RightAreaValue = 3u;
                break;
            default:
                break;
            }
        }
        else
        {
            IND_SET(nIND_LDW_RL_Yellow,nInd_Flash_2Hz);
            FISD_u8RightAreaValue = 3u;

            switch(u8LdwLka_LeftVisual)
            {
            case 0:
                IND_SET(nIND_LDW_LL_Grey,nInd_On);
                FISD_u8LeftAreaValue = 0u;
                break;
            case 1:
                IND_SET(nIND_LDW_LL_White,nInd_On);
                FISD_u8LeftAreaValue = 1u;
                break;
            case 2:
                IND_SET(nIND_LDW_LL_Green,nInd_On);
                FISD_u8LeftAreaValue = 2u;
                break;
            case 3:
                IND_SET(nIND_LDW_LL_Yellow,nInd_Flash_2Hz);
                FISD_u8LeftAreaValue = 3u;
                break;
            default:
                break;
            }
        }

        DIC_SET(nDIC_WarnSwitch2ADAS,nDic_Active);
        BUZ_SET(nBUZ_LDW, nBuz_Active);
        if(FISD_enCurAdasWarn != enAdasWarn_Bsd)
        {
            FISD_enCurAdasWarn = enAdasWarn_LdwLka;
        }
    }
    else if(u8LdwLka_LeftVisual == 0x02 || u8LdwLka_RightVisual == 0x02)
    {
        IND_SET(nIND_LDW_LL_Green,nInd_On);
        IND_SET(nIND_LDW_RL_Green,nInd_On);
        FISD_u8LeftAreaValue = 2u;
        FISD_u8RightAreaValue = 2u;
    }
    else
    {
        switch(u8LdwLka_LeftVisual)
        {
        case 0:
            IND_SET(nIND_LDW_LL_Grey,nInd_On);
            FISD_u8LeftAreaValue = 0u;
            break;
        case 1:
            IND_SET(nIND_LDW_LL_White,nInd_On);
            FISD_u8LeftAreaValue = 1u;
            break;
        default:
            break;
        }

        switch(u8LdwLka_RightVisual)
        {
        case 0:
            IND_SET(nIND_LDW_RL_Grey,nInd_On);
            FISD_u8RightAreaValue = 0u;
            break;
        case 1:
            IND_SET(nIND_LDW_RL_White,nInd_On);
            FISD_u8RightAreaValue = 1u;
            break;
        default:
            break;
        }

    }
#ifdef ENABLE_TEXT_SOUND_MIN_SHOW_2S
    /*deal with the sound warning connect to textwarn which is cancelled in 2s.*/
    if(nDIC_WarnSwitch2ADAS == TextMgr_enGetDisplayText())
    {
        BUZ_SET(nBUZ_LDW,nBuz_Active);
    }
#endif
}

static void FISD_vLkaIndicatorDetect(void)
{
    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    static U8 ldw_lka_previous_status = 0;

    /**IND Detect**/
    switch(FISD_u16SignalValue[nLDW_LKA_Status])
    {
    case 1: /*Standby*/
        if(sys_fullon)
        {
            IND_SET( nIND_LDW_LKA_Grey,nInd_On);
        }
        break;
    case 2:/*active*/
        if(sys_fullon)
        {
            IND_SET( nIND_LDW_LKA_Green,nInd_On);
        }
        break;
    case 3:/*Fault*/
        if(sys_fullon)
        {
            IND_SET( nIND_LDW_LKA_Yellow,nInd_On);
        }
        break;
    default:
        break;
    }

    // Check LDW/LKA TJA/ICA status change
    static U8 ldw_lka_keep_timer = 0;

    if((ldw_lka_previous_status==2)&&((FISD_u16SignalValue[nLDW_LKA_Status]==0)||\
                                      (FISD_u16SignalValue[nLDW_LKA_Status]==1)||(FISD_u16SignalValue[nLDW_LKA_Status]==3)))
    {
        ldw_lka_keep_timer = 20;
        ldw_lka_previous_status = FISD_u16SignalValue[nLDW_LKA_Status];
    }
    else
    {
        // since it is triggered, we just stay 200ms for buzzering
        if(ldw_lka_keep_timer>0)
        {
            BUZ_SET(nBUZ_LDWLKA_StatusChange, nBuz_Active);
            ldw_lka_keep_timer--;
        }
        if(ldw_lka_keep_timer==0)
        {
            ldw_lka_previous_status = FISD_u16SignalValue[nLDW_LKA_Status];
        }
    }
}

static void FISD_vLDPELKIndicatorDetect(void)
{
    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    static U8 ldp_previous_status = 0;
    static U8 elk_previous_status = 0;

    IND_SET(nIND_LDW_LKA_Green, nInd_Off);
    IND_SET(nIND_LDW_LKA_Yellow, nInd_Off);
    IND_SET(nIND_LDW_LKA_Grey, nInd_Off);
	IND_SET(nIND_LDP_ELK_OFF, nInd_Off);
    IND_SET(nIND_LDW_LKA_Red, nInd_Off);
    DIC_SET(nDIC_LdpFault, nDic_Inactive);
    DIC_SET(nDIC_ElkFault, nDic_Inactive);

    /**IND Detect**/
    if(sys_fullon == TRUE )
    {
		if(((4u == FISD_u16SignalValue[nELKSysSts]))||\
			((4u == FISD_u16SignalValue[nLDPSysSts]))) /*Fault*/
        {
            IND_SET( nIND_LDW_LKA_Yellow,nInd_On);
            if(4u == FISD_u16SignalValue[nELKSysSts])
            {
                DIC_SET(nDIC_ElkFault, nDic_Active);
            }
            if(4u == FISD_u16SignalValue[nLDPSysSts])
            {
                DIC_SET(nDIC_LdpFault, nDic_Active);
            }
        }
		else if(((3u == FISD_u16SignalValue[nELKSysSts]))||\
			((3u == FISD_u16SignalValue[nLDPSysSts])))
		{
			IND_SET( nIND_LDW_LKA_Green,nInd_Flash_1Hz);
		}
        else if(((2u == FISD_u16SignalValue[nELKSysSts]))||\
			((2u == FISD_u16SignalValue[nLDPSysSts])))
        {
			 IND_SET( nIND_LDW_LKA_Green,nInd_On);
		}
		else if(((1u == FISD_u16SignalValue[nELKSysSts]))||\
			   ((1u == FISD_u16SignalValue[nLDPSysSts])))
		{
			IND_SET( nIND_LDW_LKA_Grey,nInd_On);
		}
		else if(((0u == FISD_u16SignalValue[nELKSysSts]))||\
			   ((0u == FISD_u16SignalValue[nLDPSysSts])))
		{
			IND_SET( nIND_LDP_ELK_OFF,nInd_On);
		}
    }
    #if 0
	 else if(sys_fullon == TRUE)
	 {	
		if(4u == FISD_u16SignalValue[nLDPSysSts]) /*Fault*/
		{
		    IND_SET(nIND_LDW_LKA_Yellow,nInd_On);
		    DIC_SET(nDIC_LdpFault, nDic_Active); 
		}
		else if(3u == FISD_u16SignalValue[nLDPSysSts])
		{
		  IND_SET( nIND_LDW_LKA_Green,nInd_Flash_1Hz);
		}
        else if(2u == FISD_u16SignalValue[nLDPSysSts])
		{
		   IND_SET( nIND_LDW_LKA_Green,nInd_On);
		}
		else if(1u == FISD_u16SignalValue[nLDPSysSts])
		{
		   IND_SET( nIND_LDW_LKA_Grey,nInd_On);
		}
	 }
     #endif
    // Check LDW/LKA TJA/ICA status change
    static U8 ldp_keep_timer = 0;
    static U8 elk_keep_timer = 0;

    if((ldp_previous_status==2)&&((FISD_u16SignalValue[nLDPSysSts]==0)||\
      (FISD_u16SignalValue[nLDPSysSts]==1)||(FISD_u16SignalValue[nLDPSysSts]==4)))
    {
        ldp_keep_timer = 300;
        ldp_previous_status = FISD_u16SignalValue[nLDPSysSts];
    }
    else
    {
        // since it is triggered, we just stay 200ms for buzzering
        if(ldp_keep_timer>0)
        {
            //DIC_SET(nDIC_LDW_LKA_DeACTIVE, nDic_Active);
            //BUZ_SET(nBUZ_LDWLKA_StatusChange, nBuz_Active);
            ldp_keep_timer--;
        }
        if(ldp_keep_timer==0)
        {
            ldp_previous_status = FISD_u16SignalValue[nLDPSysSts];
        }
    }
    // ELK
    if((elk_previous_status==2)&&((FISD_u16SignalValue[nELKSysSts]==0)||\
                                      (FISD_u16SignalValue[nELKSysSts]==1)||(FISD_u16SignalValue[nELKSysSts]==4)))
    {
        elk_keep_timer = 20;
        elk_previous_status = FISD_u16SignalValue[nELKSysSts];
    }
    else
    {
        // since it is triggered, we just stay 200ms for buzzering
        if(elk_keep_timer>0)
        {
            //DIC_SET(nDIC_LDW_LKA_DeACTIVE, nDic_Active);
            //BUZ_SET(nBUZ_LDWLKA_StatusChange, nBuz_Active);
            elk_keep_timer--;
        }
        if(elk_keep_timer==0)
        {
            elk_previous_status = FISD_u16SignalValue[nELKSysSts];
        }
    }
}

static void FISD_vClearLDWLDPEKLIndicatorStatus(void)
{	
    IND_SET(nIND_LDW_LKA_Green, nInd_Off);
    IND_SET(nIND_LDW_LKA_Yellow, nInd_Off);
    IND_SET(nIND_LDW_LKA_Grey, nInd_Off);
	IND_SET(nIND_LDP_ELK_OFF, nInd_Off);
    IND_SET(nIND_LDW_LKA_Red, nInd_Off);
}

static void FISD_vLdwLDPELKIndicatorDetect(void)
{
    
    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
    
    IND_SET(nIND_LDW_LKA_Green, nInd_Off);
    IND_SET(nIND_LDW_LKA_Yellow, nInd_Off);
    IND_SET(nIND_LDW_LKA_Grey, nInd_Off);
	IND_SET(nIND_LDP_ELK_OFF, nInd_Off);
    IND_SET(nIND_LDW_LKA_Red, nInd_Off);
    DIC_SET(nDIC_LdpFault, nDic_Inactive);
    DIC_SET(nDIC_ElkFault, nDic_Inactive);
    DIC_SET(nDIC_LdwFault, nDic_Inactive);
    if((sys_fullon==TRUE) && (TRUE == Rte_FISD_boAnimationPlayIsFinish()))
    {
        switch(FISD_u16SignalValue[nELKSysSts])
        {
            case 1: /*Standby*/
                {
                    IND_SET( nIND_LDW_LKA_Grey,nInd_On);
                }
                break;
            case 2:/*active standby*/
                {
                    IND_SET( nIND_LDW_LKA_Green,nInd_On);
                }
                break;
            case 4:/*Fault*/
                {
                    IND_SET( nIND_LDW_LKA_Yellow,nInd_On);
                    DIC_SET(nDIC_ElkFault, nDic_Active);
                }
                break;
            case 3:/*active startup*/
                {
                    IND_SET( nIND_LDW_LKA_Green,nInd_Flash_1Hz);
                }
                break;
        	 case 0:/*off*/
                
                {
                    IND_SET( nIND_LDP_ELK_OFF,nInd_On);
                }
                break;
            default:
                break;
        }
        
		if(((4u == FISD_u16SignalValue[nLDWSysSts]))||\
			((4u == FISD_u16SignalValue[nLDPSysSts]))) /*Fault*/
        {
            FISD_vClearLDWLDPEKLIndicatorStatus();
            IND_SET( nIND_LDW_LKA_Yellow,nInd_On);
        
            if(4u == FISD_u16SignalValue[nLDWSysSts])
            {
                DIC_SET(nDIC_LdwFault, nDic_Active);
            }
            if(4u == FISD_u16SignalValue[nLDPSysSts])
            {
                DIC_SET(nDIC_LdpFault, nDic_Active);
            }
        }
		else if(((3u == FISD_u16SignalValue[nLDWSysSts]))||\
			((3u == FISD_u16SignalValue[nLDPSysSts])))
		{
            FISD_vClearLDWLDPEKLIndicatorStatus();
			IND_SET( nIND_LDW_LKA_Green,nInd_Flash_1Hz);
		}
        else if(((2u == FISD_u16SignalValue[nLDWSysSts]))||\
			((2u == FISD_u16SignalValue[nLDPSysSts])))
        {
            FISD_vClearLDWLDPEKLIndicatorStatus();
			IND_SET( nIND_LDW_LKA_Green,nInd_On);
    		if(3u == FISD_u16SignalValue[nELKSysSts])
            {
                IND_SET( nIND_LDW_LKA_Green,nInd_Flash_1Hz);
            }
		}
		else if(((1u == FISD_u16SignalValue[nLDWSysSts]))||\
			   ((1u == FISD_u16SignalValue[nLDPSysSts])))
		{
		    FISD_vClearLDWLDPEKLIndicatorStatus();
			IND_SET( nIND_LDW_LKA_Grey,nInd_On);
            if(((2u == FISD_u16SignalValue[nELKSysSts])))
            {
                IND_SET( nIND_LDW_LKA_Grey,nInd_Off);
                IND_SET( nIND_LDW_LKA_Green,nInd_On);
            }
            if(((3u == FISD_u16SignalValue[nELKSysSts])))
            {
                IND_SET( nIND_LDW_LKA_Grey,nInd_Off);
                IND_SET( nIND_LDW_LKA_Green,nInd_Flash_1Hz);
            }
		}
		else if(((0u == FISD_u16SignalValue[nLDWSysSts]))||\
			   ((0u == FISD_u16SignalValue[nLDPSysSts])))
		{
			IND_SET( nIND_LDP_ELK_OFF,nInd_On);
		}
    }
    
}


U8 FISD_GetLeftLaneValue(void)
{
    return FISD_u8LeftAreaValue;
}

U8 FISD_GetRightLaneValue(void)
{
    return FISD_u8RightAreaValue;
}


static void FISD_vLdwIndicatorDetect(void)
{
    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    static U8 ldw_lka_previous_status = 0;

    IND_SET(nIND_LDW_Green, nInd_Off);
    IND_SET(nIND_LDW_Yellow, nInd_Off);
    IND_SET(nIND_LDW_Grey, nInd_Off);
	IND_SET(nIND_LDW_OFF, nInd_Off);
    DIC_SET(nDIC_LdwFault, nDic_Inactive);
    
    /**IND Detect**/
    if(sys_fullon == TRUE)
    {
        switch(FISD_u16SignalValue[nLDWSysSts])
        {
        case 1: /*Standby*/
            {
                IND_SET( nIND_LDW_Grey,nInd_On);
            }
            break;
        case 2:/*active standby*/
            {
                IND_SET( nIND_LDW_Green,nInd_On);
            }
            break;
        case 4:/*Fault*/
            {
                IND_SET( nIND_LDW_Yellow,nInd_On);
                DIC_SET(nDIC_LdwFault, nDic_Active);
            }
            break;
        case 3:/*active startup*/
            {
                IND_SET( nIND_LDW_Green,nInd_Flash_1Hz);
            }
            break;
    	 case 0:/*off*/
            {
                IND_SET( nIND_LDW_OFF,nInd_On);
            }
            break;
        default:
            break;
        }
    }

    // Check LDW/LKA TJA/ICA status change
    static U8 ldw_lka_keep_timer = 0;

    if((ldw_lka_previous_status==2)&&((FISD_u16SignalValue[nLDWSysSts]==0)||\
                                      (FISD_u16SignalValue[nLDWSysSts]==1)||(FISD_u16SignalValue[nLDWSysSts]==4)))
    {
        ldw_lka_keep_timer = 20;
        ldw_lka_previous_status = FISD_u16SignalValue[nLDWSysSts];
    }
    else
    {
        // since it is triggered, we just stay 200ms for buzzering
        if(ldw_lka_keep_timer>0)
        {
            //DIC_SET(nDIC_LDW_LKA_DeACTIVE, nDic_Active);
            //BUZ_SET(nBUZ_LDWLKA_StatusChange, nBuz_Active);
            ldw_lka_keep_timer--;
        }
        if(ldw_lka_keep_timer==0)
        {
            ldw_lka_previous_status = FISD_u16SignalValue[nLDWSysSts];
        }
    }
}

static void FISD_vClearLdwStatus(void)
{
    /*LDW*/
    IND_SET(nIND_LDW_Green, nInd_Off);
    IND_SET(nIND_LDW_Yellow, nInd_Off);
    IND_SET(nIND_LDW_Grey, nInd_Off);
	IND_SET(nIND_LDW_OFF, nInd_Off);

    /*LKA*/
    IND_SET(nIND_LDW_LKA_Green, nInd_Off);
    IND_SET(nIND_LDW_LKA_Yellow, nInd_Off);
    IND_SET(nIND_LDW_LKA_Grey, nInd_Off);
	IND_SET(nIND_LDP_ELK_OFF, nInd_Off);

    IND_SET(nIND_LDW_LL_White, nInd_Off);
    IND_SET(nIND_LDW_LL_Grey, nInd_Off);
    IND_SET(nIND_LDW_LL_Yellow, nInd_Off);
    IND_SET(nIND_LDW_RL_White, nInd_Off);
    IND_SET(nIND_LDW_RL_Grey, nInd_Off);
    IND_SET(nIND_LDW_RL_Yellow, nInd_Off);
    IND_SET(nIND_LDW_LL_Green,nInd_Off);
    IND_SET(nIND_LDW_RL_Green,nInd_Off);

    BUZ_SET(nBUZ_LDW, nBuz_Inactive);
    DIC_SET(nDIC_WarnSwitch2ADAS, nDic_Inactive);
    BUZ_SET(nBUZ_TakeOverTurnReq,nBuz_Inactive);

    FISD_u8LeftAreaValue = 0u;
    FISD_u8RightAreaValue = 0u;
    FISD_enCurAdasWarn = enAdasWarn_Off;
}

/*******************************************************************************************
* Function: FISD_vDrlDetect
* Description: Detect DRL warning is happened or not.
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vClearLDWLDPEKLStatus(void)
{
	IND_SET(nIND_LDW_Green, nInd_Off);
    IND_SET(nIND_LDW_Yellow, nInd_Off);
    IND_SET(nIND_LDW_Grey, nInd_Off);
	IND_SET(nIND_LDW_OFF, nInd_Off);
    DIC_SET(nDIC_LdwFault, nDic_Inactive);
	
    IND_SET(nIND_LDW_LKA_Green, nInd_Off);
    IND_SET(nIND_LDW_LKA_Yellow, nInd_Off);
    IND_SET(nIND_LDW_LKA_Grey, nInd_Off);
	IND_SET(nIND_LDP_ELK_OFF, nInd_Off);
    IND_SET(nIND_LDW_LKA_Red, nInd_Off);
    DIC_SET(nDIC_LdpFault, nDic_Inactive);
    DIC_SET(nDIC_ElkFault, nDic_Inactive);

	BUZ_SET(nBUZ_LdpLdwWarning, nBuz_Inactive);
	BUZ_SET(nBUZ_ElkSafty, nBuz_Inactive);
	BUZ_SET(nBUZ_ElkSafty_Always, nBuz_Inactive);
	BUZ_SET(nBUZ_ELKOncoming, nBuz_Inactive);
	BUZ_SET(nBUZ_ELK_EV, nBuz_Inactive);
    BUZ_SET(nBUZ_ELK_EV_1, nBuz_Inactive);
}


static void FISD_vLdwLdpEklDetect(void)
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

	FISD_vClearLDWLDPEKLStatus();
	BUZ_SET(nBUZ_LDWLKA_StatusChange, nBuz_Inactive);
	if((TRUE == Rte_FISD_boAnimationPlayIsFinish())&&(TRUE == Rte_FISD_boGetPowerDependent()))
	{
		/* 指示灯及文言报警 */
        if(Rte_FISD_u8GetCONFIG_T18FL4() == 0)   /* 非T18FL4国内、T18FL4&T28国际项目 */
        {
            FISD_vLdwIndicatorDetect();
            FISD_vLDPELKIndicatorDetect();
        }
        else
        {
            FISD_vLdwLDPELKIndicatorDetect();   /* T18FL4国内、T18FL4&T28国际 */
        }

		/* 声音报警 */
		if((1u == FISD_u16SignalValue[nWarnModSts])||(3u == FISD_u16SignalValue[nWarnModSts]))
		{
			if(3u == FISD_u16SignalValue[nLDWSysSts])
			{
				BUZ_SET(nBUZ_LdpLdwWarning, nBuz_Active);
			}
			if(3u == FISD_u16SignalValue[nELKSysSts])
			{
				if(1u == FISD_u16SignalValue[nWarnModSts])
				{
					BUZ_SET(nBUZ_ElkSafty, nBuz_Active);
				}
				if(3u == FISD_u16SignalValue[nWarnModSts])
				{
					BUZ_SET(nBUZ_ElkSafty_Always, nBuz_Active);
				}
			}
		}
		if(2u == FISD_u16SignalValue[nELKIntvMod])
		{
			BUZ_SET(nBUZ_ELKOncoming, nBuz_Active);
			BUZ_SET(nBUZ_ELK_EV, nBuz_Active);
		}
        if (3u == FISD_u16SignalValue[nELKIntvMod])
        {
            BUZ_SET(nBUZ_ELKOncoming, nBuz_Active);
            BUZ_SET(nBUZ_ELK_EV_1, nBuz_Active);
        }
		if((Rte_boGetLDWSysStsTimeout() == TRUE)||(Rte_boGetTJA_ICA_modeTimeout() == TRUE))
		{
			IND_SET(nIND_LDW_Green, nInd_Off);
    		IND_SET(nIND_LDW_Yellow, nInd_Off);
       		IND_SET(nIND_LDW_Grey, nInd_Off);
			IND_SET(nIND_LDW_OFF, nInd_Off);
			IND_SET(nIND_LDW_LKA_Green, nInd_Off);
       	 	IND_SET(nIND_LDW_LKA_Yellow, nInd_Off);
       		IND_SET(nIND_LDW_LKA_Grey, nInd_Off);
			IND_SET(nIND_LDP_ELK_OFF, nInd_Off);
			if((Rte_boGetTJA_ICA_modeTimeout() ==TRUE)&&(Rte_FISD_u8GetCONFIG_FCM() == TRUE))
			{
				if((Rte_FISD_u8GetCONFIG_LDP() == TRUE) || (Rte_FISD_u8GetCONFIG_ELK() == TRUE))
				{
					IND_SET(nIND_LDW_LKA_Yellow, nInd_On);
				}

				if(Rte_FISD_u8GetCONFIG_LDW() == TRUE)
				{					
					IND_SET(nIND_LDW_Yellow, nInd_On);
				}
			}

		}
	}
	else
	{
		FISD_vClearLDWLDPEKLStatus();
	}
	if(Rte_boGetLDWSysStsTimeout() == TRUE)
	{
		BUZ_SET(nBUZ_LDWLKA_StatusChange, nBuz_Inactive);
	}
	if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
    {
    	IND_SET(nIND_LDW_Green, nInd_Off);
		if((Rte_FISD_u8GetCONFIG_FCM() == TRUE) || (Rte_FISD_u8GetCONFIG_IDCU() == TRUE))
		{
    		IND_SET(nIND_LDW_Yellow, nInd_On);
		}
       	IND_SET(nIND_LDW_Grey, nInd_Off);
		IND_SET(nIND_LDW_OFF, nInd_Off);

        /*LKA*/
        IND_SET(nIND_LDW_LKA_Green, nInd_Off);
		if((Rte_FISD_u8GetCONFIG_FCM() == TRUE && (Rte_FISD_u8GetCONFIG_LDP() == TRUE || Rte_FISD_u8GetCONFIG_ELK() == TRUE)) ||\
          (Rte_FISD_u8GetCONFIG_IDCU() == TRUE))
		{
        	IND_SET(nIND_LDW_LKA_Yellow, nInd_On);
		}
        IND_SET(nIND_LDW_LKA_Grey, nInd_Off);
		IND_SET(nIND_LDP_ELK_OFF, nInd_Off);
    }
}

static void FISD_vTakeOverDetect(void)
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	BOOL TakeOver_ind_condition = TRUE;

	DIC_SET(nDIC_TakeOverWarning, nDic_Inactive);
	DIC_SET(nDIC_TakeOverSafty, nDic_Inactive);
	DIC_SET(nDIC_TakeOverFunExit, nDic_Inactive);
	
	BUZ_SET(nBUZ_TakeOverWarning, nBuz_Inactive);
	BUZ_SET(nBUZ_TakeOverSafty, nBuz_Inactive);
	if((TRUE == Rte_FISD_boAnimationPlayIsFinish())&&(TRUE == Rte_FISD_boGetPowerDependent()))
	{
		if(1u == FISD_u16SignalValue[nLDPTJATakeoverReq])
		{
			DIC_SET(nDIC_TakeOverWarning, nDic_Active);
			BUZ_SET(nBUZ_TakeOverWarning, nBuz_Active);
		}
		else if(2u == FISD_u16SignalValue[nLDPTJATakeoverReq])
		{
			DIC_SET(nDIC_TakeOverSafty, nDic_Active);
			BUZ_SET(nBUZ_TakeOverSafty, nBuz_Active);
		}
		else if(3u == FISD_u16SignalValue[nLDPTJATakeoverReq])
		{
			DIC_SET(nDIC_TakeOverFunExit, nDic_Active);
		}
	}
}

/*******************************************************************************************
* Function: FISD_vDrlDetect
* Description: Detect DRL warning is happened or not.
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nActive  1u
static void FISD_vDrlDetect(void)
{
    /*Clear Status*/
    IND_SET(nIND_DrvDaylight, nInd_Off);

    /*Redetect Status*/

    if(FISD_nActive == FISD_u16SignalValue[nDRLSts])
    {
        IND_SET(nIND_DrvDaylight, nInd_On);
    }

    if((TRUE != Rte_FISD_boAnimationPlayIsFinish()) && (Rte_FISD_boGetPowerDependent() == TRUE))
    {
        IND_SET(nIND_DrvDaylight, nInd_Off);
    }
}
#undef FISD_nActive

/*******************************************************************************************
* Function: FISD_vFrontFogDetect
* Description: Detect Front fog warning is happened or not.
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nActive  1u
static void FISD_vFrontFogDetect(void)
{
    /*Clear Status*/
    IND_SET(nIND_FrontFog, nInd_Off);

    /*Redetect Status*/

    if(FISD_nActive == FISD_u16SignalValue[nFrontFogLightSts])
    {
        IND_SET(nIND_FrontFog, nInd_On);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        IND_SET(nIND_FrontFog, nInd_Off);
    }
}
#undef FISD_nActive

/*******************************************************************************************
* Function: FISD_vHdcDetect
* Description: Detect HDC warning is happened or not.
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vHdcDetect(void)
{
    static U16 hdc_dic_timer = 0;
    static U8 previous_hdc_ctrl_sts = 0;
    U8 hdc_ctrl_sts = FISD_u16SignalValue[nHDCCtrlSts];
	static U8 hdc_ctrl_sts_pre = 0;
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
    /*Clear Status*/
    IND_SET(nIND_HDC, nInd_Off);
    IND_SET(nIND_HDC_Fault,nInd_Off);
    DIC_SET(nDIC_HDC_Failure, nDic_Inactive);
//	DIC_SET(nDIC_HDC_Active, nDic_Inactive);
	//BUZ_SET(nBUZ_HDC_Active, nBuz_Inactive);
	if((sys_fullon == TRUE)&&\
        (((Rte_FISD_u8GetCONFIG_M1E() == 1) && (Rte_FISD_u8GetCONFIG_DrivingPowerType() == nPowerType_OIL)) == FALSE))
	{
		if(hdc_ctrl_sts == 1)
		{
			IND_SET(nIND_HDC, nInd_On);
			if((hdc_ctrl_sts_pre == 0) || (hdc_ctrl_sts_pre == 1))
			{
				BUZ_SET(nBUZ_HDC_Active, nBuz_Active);
				DIC_SET(nDIC_HDC_Active, nDic_Active);
			}
			
		}
		else if(hdc_ctrl_sts == 2)
		{
			IND_SET(nIND_HDC, nInd_Flash_1Hz);
		}
		if((hdc_ctrl_sts == 0) || (hdc_ctrl_sts == 2) || (hdc_ctrl_sts == 3) || \
			((hdc_ctrl_sts == 1) && (hdc_ctrl_sts_pre == 2))||\
			((hdc_ctrl_sts == 1) && (hdc_ctrl_sts_pre == 3)))
		{
			BUZ_SET(nBUZ_HDC_Active, nBuz_Inactive);
			DIC_SET(nDIC_HDC_Active, nDic_Inactive);
		}
		if(hdc_ctrl_sts != 1)
		{
			hdc_ctrl_sts_pre = hdc_ctrl_sts;
		}
		

		if((1u == FISD_u16SignalValue[nHDCFailSts]))
	    {
	    	IND_SET(nIND_HDC_Fault,nInd_On);
	        DIC_SET(nDIC_HDC_Failure, nDic_Active);
			IND_SET(nIND_HDC, nInd_Off);
	    }

		if(Rte_boGetHDCCtrlStsTimeout() == TRUE)
		{
			IND_SET(nIND_HDC, nInd_Off);
    		IND_SET(nIND_HDC_Fault,nInd_Off);
			BUZ_SET(nBUZ_HDC_Active, nBuz_Inactive);
			DIC_SET(nDIC_HDC_Failure, nDic_Inactive);
			DIC_SET(nDIC_HDC_Active, nDic_Inactive);
		}

	}

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        IND_SET(nIND_HDC, nInd_Off);
        IND_SET(nIND_HDC_Fault,nInd_Off);
        DIC_SET(nDIC_HDC_Failure, nDic_Inactive);
        DIC_SET(nDIC_HDC_Active, nDic_Inactive);
		BUZ_SET(nBUZ_HDC_Active, nBuz_Inactive);
    }
	
}

/*******************************************************************************************
* Function: FISD_vHhcDetect
* Description: Detect HHC warning is happened or not.
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nFailPresent 1u
static void FISD_vHhcDetect(void)
{
    BOOL hhc_ind_condition = TRUE;

    /*Clear Status*/
    DIC_SET(nDIC_HHC_Failure, nDic_Inactive);

    /*Redetect Status*/

    if((FISD_nFailPresent == FISD_u16SignalValue[nHHCFailSts])&&hhc_ind_condition)
    {
        DIC_SET(nDIC_HHC_Failure, nDic_Active);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        DIC_SET(nDIC_HHC_Failure, nDic_Inactive);
    }
}
#undef FISD_nFailPresent

/*******************************************************************************************
* Function: FISD_vRainDetect
* Description: Detect HDC warning is happened or not.
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_nFailPresent 1u
static void FISD_vRainSensorFailure(void);
static void FISD_vRainDetect(void)
{
    FISD_vRainSensorFailure();
}

static void FISD_vRainSensorFailure(void)
{
    /*Clear Status*/
    DIC_SET(nDIC_CheckRainSensor, nDic_Inactive);

    /*Redetect Status*/

    if(FISD_nFailPresent == FISD_u16SignalValue[nRainSensorFailSts])
    {
        DIC_SET(nDIC_CheckRainSensor, nDic_Active);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        DIC_SET(nDIC_CheckRainSensor, nDic_Inactive);
    }
}

#undef FISD_nFailPresent

/*******************************************************************************************
* Function: FISD_vDebounceThreeSecond
* Description: Debounce 3 second
* Parameters: none
* Return: none
********************************************************************************************/
#define SelfCheckDelayCount 100
static void FISD_vDebounceThreeSecond(void)
{
	U8 u8SocSts = SYN_SOC_WorkingStateGet();
	static BOOL FristIgnOn = FALSE;
	static U16 self_check_delay_cnt = 0;
    /*D1 debounce*/
    if(FALSE == FISD_boIgnOnToOffFlag)
    {
        FISD_u16DebounceCount_D1 = 0u;
    }

    if(TRUE == FISD_enIgnState)
    {
        if((FISD_u16DebounceCount_D1 < TIMECOUNT_6s) && \
            (TRUE == Rte_FISD_boAnimationPlayIsFinish()) && \
            ((u8SocSts >= SOC_WORKING_ANIMATION_ENDED) && (u8SocSts <= SOC_WORKING_NORMAL))
          )
        {
        	if(self_check_delay_cnt < SelfCheckDelayCount)
			{
				self_check_delay_cnt++;
			}
			else
			{
				FristIgnOn = TRUE;
			}
			if(FristIgnOn == TRUE)
			{
            	FISD_u16DebounceCount_D1 ++;
			}
        }
        else
        {
        }
		
    }
    else if(FALSE == FISD_enIgnState)
    {
    	//self_check_delay_cnt = SelfCheckDelayCount;
    }
    else
    {
    }

    /*D1 and Crank debounce*/
    if((FALSE == FISD_boIgnOnToOffFlag) || \
            ((nCRANK_ON != FISD_enLastKeySts) && (nCRANK_ON == FISD_u16SignalValue[nKeySts])))
    {
        FISD_boDebounceFlag_Crank_D1 = FALSE;
        FISD_u16DebounceCount_Crank_D1 = 0u;
    }

    if(FALSE == FISD_enIgnState)
    {
        FISD_boDebounceFlag_Crank_D1 = TRUE;
    }
    else if((TRUE == FISD_enIgnState) || (nCRANK_ON == FISD_u16SignalValue[nKeySts]))
    {
        if(FISD_u16DebounceCount_Crank_D1 < TIMECOUNT_3s)
        {
            FISD_u16DebounceCount_Crank_D1 ++;
        }
        else
        {
            FISD_boDebounceFlag_Crank_D1 = TRUE;
        }
    }
    else
    {
    }

    /*D1 , Crank and Acc debounce*/
    if((FALSE == FISD_boIgnOnToOffFlag) || \
            ((nCRANK_ON != FISD_enLastKeySts) && (nCRANK_ON == FISD_u16SignalValue[nKeySts]))|| \
            ((OomStandbyMode != FISD_enLastPowerMode) && (OomStandbyMode == Rte_FISD_enGetCurrentPowerMode())))
    {
        FISD_u16DebounceCount_Crank_D1_D2  = 0u;
    }

    if((TRUE == FISD_enIgnState) || (nCRANK_ON == FISD_u16SignalValue[nKeySts]) || (OomStandbyMode == Rte_FISD_enGetCurrentPowerMode()))
    {
        if(FISD_u16DebounceCount_Crank_D1_D2  < TIMECOUNT_3s)
        {
            FISD_u16DebounceCount_Crank_D1_D2 ++;
        }
        else
        {
        }
    }

    FISD_enLastKeySts = (tenKeySts)FISD_u16SignalValue[nKeySts];
    FISD_enLastPowerMode = Rte_FISD_enGetCurrentPowerMode();
}

void FISD_vSetBacklightControlRecord(BOOL blSts, U8 stsCode)
{
	FISD_BackLightStsRecord = stsCode;
	FISD_BackLightStsRecord <<= 1;
	FISD_BackLightStsRecord |= blSts;	
}

U8 FISD_vGetBacklightControlRecord(void)
{
	return FISD_BackLightStsRecord;
}

BOOL FISD_bHasIndToShow(void)
{
	BOOL bRtnVal = FALSE;
	
	if((!IND_CHK(nIND_TurnLeft, nInd_Off)) || (!IND_CHK(nIND_TurnRight, nInd_Off)) || \
	(!IND_CHK(nIND_LittleLamp, nInd_Off)) || (!IND_CHK(nIND_HighBeam, nInd_Off)) || \
	(!IND_CHK(nIND_DrvDaylight, nInd_Off)) || (!IND_CHK(nIND_RearFog, nInd_Off)) || \
	(!IND_CHK(nIND_DoorOpen, nInd_Off)) || (!IND_CHK(nIND_ESCLFault, nInd_Off)) || \
	(!IND_CHK(nIND_ESCLSerFault, nInd_Off)) || (!IND_CHK(nIND_HandBrake_EpbWork_Red_P, nInd_Off)) || \
	(!IND_CHK(nIND_HMA_White, nInd_Off)) || (!IND_CHK(nIND_HMA_Yellow, nInd_Off))
	)
	{
		bRtnVal = TRUE;
	}
	
	return bRtnVal;
}

BOOL FISD_bHasDicToShow(void)
{
	BOOL bRtnVal = FALSE;
	tenTextId tenTextWarnId = TextMgr_enGetDisplayText();
	
	if(tenTextWarnId < nDIC_MaxNum)
	{
		bRtnVal = TRUE;
	}
	
	return bRtnVal;
}
void FISD_log_set_duty()
{
#if defined(MCU_LOG_OPTION)
	extern U8 g_outputTftDuty;
	LOG_APP_UPDATE_LOG_DATA_TIMING(eLOG_RTE_DAT_SYSTEM_TSK_02_OUTPUT_TFT_DUTY,g_outputTftDuty);
#endif
}
/*******************************************************************************************
* Function: FISD_vCalendarAndCarSts
* Description: Show calender and car after ignition off with any other warning
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_ArmingSuccessful    1u
#define FISD_DisarmingSuccessful 2u
static void FISD_vCalendarAndCarSts(void)
#if 1
{
    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
    BOOL sys_parton = boSystemOOMPartOnMode();
    BOOL acc_off = ((FALSE == IO_GET_PS2uP_ON_OFF_LOGIC()) && (nACC != Rte_u16GetKeySts()));

    BOOL boNmStatus = Rte_FISD_boGetNMMsgSleepStatus();
    
    BOOL rvs_mode = ((FALSE==IlGetBCM_4_RVSModeRxTimeout())&&(1==IlGetRxBCM_4_RVSMode()));

    BOOL driver_door_status = ((FALSE==IlGetBCM_4_DriverDoorStsRxTimeout())&&(1==IlGetRxBCM_4_DriverDoorSts()));

    static BOOL IsdriverDoorOpenedSinceRVS = FALSE;

#if defined(MCU_LOG_OPTION)
    static U16 last_OTAsts = 0u;

    LOG_APP_UPDATE_LOG_DATA_TIMING(eLOG_RTE_DAT_IPC_TSK_01_NM_SLEEP_STATE, boNmStatus);

  #if defined(SV_DEBUG)  
    LOG_APP_UPDATE_LOG_DATA_TIMING(eLOG_RTE_DAT_IPC_TSK_01_SV_DEBUG_MODE, 1);
  #endif
#endif
    
    if(sys_fullon)
    {
        if((rvs_mode == TRUE) && (0 == driver_door_status) && (IsdriverDoorOpenedSinceRVS == FALSE))
        {
            FISD_boCalendarAndCarSts = FALSE;
            FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 1);
        }
        else 
        {
        	FISD_boCalendarAndCarSts = TRUE;
        	FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 2);

        	if(TRUE == driver_door_status)
        	{
        		IsdriverDoorOpenedSinceRVS = TRUE;
        		FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 3);
        	}
        }

        FISD_u16CalendarAndCarStsDelay = TIMECOUNT_5s;  
    }
    else if(sys_parton && (acc_off == FALSE))
    {
        if(can_diag_get_sw_conf(CONF_IDX_One_ClickStartSwitch) == nOneClickStartSwitch_Not_Present)
        {
            if((rvs_mode == TRUE) && (0 == driver_door_status) && (IsdriverDoorOpenedSinceRVS == FALSE))
            {
                FISD_boCalendarAndCarSts = FALSE;
                FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 4);
            }
            else 
            {
                FISD_boCalendarAndCarSts = TRUE;
                FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 5);
            
                if(TRUE == driver_door_status)
                {
                    IsdriverDoorOpenedSinceRVS = TRUE;
                    FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 6);
                }
            }
        }
        else
        {
            FISD_boCalendarAndCarSts = TRUE;
            IsdriverDoorOpenedSinceRVS = FALSE;
            FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 7);
        }
        FISD_u16CalendarAndCarStsDelay = TIMECOUNT_5s;
    }
    else if(sys_parton && acc_off)
    {
        if(TRUE == boNmStatus)
        {
            FISD_boCalendarAndCarSts = FALSE;
            FISD_u16CalendarAndCarStsDelay = 0u;
            FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 8);
        }
        else
        {
        	if(((Rte_FISD_u8GetCONFIG_DrivingPowerType() == nPowerType_EV) || \
                ((Rte_FISD_u8GetCONFIG_DrivingPowerType() == nPowerType_PHEV)&&(Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3)))&&\
                (u8GetChargePageState() == nIND_ChargePage))
        	{
        		if((TextMgr_enGetDisplayText() == nDIC_EnterChargingPage) || (driver_door_status == True))
        		{
        			FISD_boCalendarAndCarSts = TRUE;
					FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 9);
        		}
				else
				{
					FISD_boCalendarAndCarSts = FALSE;
					FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 10);
				}
				FISD_u16CalendarAndCarStsDelay = TIMECOUNT_5s;
        	}
			else
			{
	            if((TRUE == FISD_bHasIndToShow()) || (TRUE == FISD_bHasDicToShow()))
	            {
	                FISD_boCalendarAndCarSts = TRUE;
	                FISD_u16CalendarAndCarStsDelay = TIMECOUNT_5s;
	                FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 11);
	            }

	            if(FISD_u16CalendarAndCarStsDelay > 0u)
	            {
	                FISD_u16CalendarAndCarStsDelay --;
	            }
	            else
	            {
	                FISD_boCalendarAndCarSts = FALSE;
	                FISD_u16CalendarAndCarStsDelay = TIMECOUNT_5s;
	                FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 12);
	            }
			}
        }
        
        IsdriverDoorOpenedSinceRVS = FALSE;
    }
    else
    {
        FISD_boCalendarAndCarSts = FALSE;
        FISD_u16CalendarAndCarStsDelay = 0u;
        IsdriverDoorOpenedSinceRVS = FALSE;
        FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 13);
    }
    
    if((FISD_u16SignalValue[nOTASts] == 1) || (OTAGet_OTASts() == 1))
	{
        FISD_boCalendarAndCarSts = FALSE;
        FISD_u16CalendarAndCarStsDelay = 0u;
        FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 14);
    }

    if(power_mana_get_appointment_to_upgrade_enter_flag() == 1)
    {
		/*预约升级熄灭屏幕*/
        FISD_boCalendarAndCarSts = FALSE;
        FISD_u16CalendarAndCarStsDelay = 0u;
        FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 15);
    }

    if (TRUE == power_mana_get_reset_start_flag())
    {
        /*进入复位流程熄灭屏幕*/
        FISD_boCalendarAndCarSts = FALSE;
        FISD_u16CalendarAndCarStsDelay = 0u;
        FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 16);
    }
    
#if defined(MCU_LOG_OPTION)
	if(FISD_BackLightStsRecord_BACK!=FISD_BackLightStsRecord)
	{
		LOG_APP_UPDATE_LOG_DATA_AT_ONCE(eLOG_RTE_DAT_SYSTEM_TSK_02_FISD_BACKLIGHTSTS_RECORD,FISD_BackLightStsRecord);
	}
	FISD_BackLightStsRecord_BACK=FISD_BackLightStsRecord;

    if (last_OTAsts != FISD_u16SignalValue[nOTASts])
    {
        last_OTAsts = FISD_u16SignalValue[nOTASts];
        if (FISD_u16SignalValue[nOTASts] == 1u)
        {
            LOG_APP_UPDATE_LOG_DATA_AT_ONCE(eLOG_RTE_DAT_SYSTEM_TSK_02_IHU_OTA_STS,1);
        }
        else
        {
            LOG_APP_UPDATE_LOG_DATA_AT_ONCE(eLOG_RTE_DAT_SYSTEM_TSK_02_IHU_OTA_STS,0);
        }
    }
#endif
}	
#else
{
    BOOL sys_fullon = boSystemOOMFullOnMode();
    BOOL sys_parton = boSystemOOMPartOnMode();
    BOOL acc_off = (FALSE == IO_GET_PS2uP_ON_OFF_LOGIC());

    U8 u8ArmingSts = FISD_u16SignalValue[nArmingSts];
    static U8 u8LastArmingsts = 0;

    BOOL boNmStatus = Rte_FISD_boGetNMMsgSleepStatus();

#if defined(MCU_LOG_OPTION)
    LOG_APP_UPDATE_LOG_DATA_TIMING(eLOG_RTE_DAT_IPC_TSK_01_NM_SLEEP_STATE, boNmStatus);

  #if defined(SV_DEBUG)  
    LOG_APP_UPDATE_LOG_DATA_TIMING(eLOG_RTE_DAT_IPC_TSK_01_SV_DEBUG_MODE, 1);
  #endif
#endif
    BOOL rvs_mode = ((FALSE==IlGetBCM_4_RVSModeRxTimeout())&&(1==IlGetRxBCM_4_RVSMode()));
    static BOOL boLastRVSmode = 0;

    BOOL driver_door_status = ((FALSE==IlGetBCM_4_DriverDoorStsRxTimeout())&&(1==IlGetRxBCM_4_DriverDoorSts()));
    static BOOL last_driver_door_status = 0;
    static U8 parton_to_fullon_entry_counter = 0;

    BOOL passenger_door_status = ((FALSE==IlGetBCM_4_PsngrDoorStsRxTimeout())&&(1==IlGetRxBCM_4_PsngrDoorSts()));
    static BOOL last_passenger_door_status = 0;

    BOOL rr_door_status = ((FALSE==IlGetBCM_4_RHRDoorStsRxTimeout())&&(1==IlGetRxBCM_4_RHRDoorSts()));
    static BOOL last_rr_door_status = 0;

    BOOL rl_door_status = ((FALSE==IlGetBCM_4_LHRdoorStsRxTimeout())&&(1==IlGetRxBCM_4_LHRdoorSts()));
    static BOOL last_rl_door_status = 0;

    static U16 rvsmode_back2normal_counter = 0;
    if(sys_fullon)
    {
        if((driver_door_status==TRUE)&&(last_driver_door_status==FALSE))
        {
            FISD_u16CalendarAndCarStsDelay = TIMECOUNT_5s;
            FISD_boCalendarAndCarSts = TRUE;
            rvsmode_back2normal_counter = 0;
            FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 1);
        }
        else if((passenger_door_status==TRUE)&&(last_passenger_door_status==FALSE))
        {
            FISD_u16CalendarAndCarStsDelay = TIMECOUNT_5s;
            FISD_boCalendarAndCarSts = TRUE;
            rvsmode_back2normal_counter = 0;
            FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 2);
        }
        else if((rr_door_status==TRUE)&&(last_rr_door_status==FALSE))
        {
            FISD_u16CalendarAndCarStsDelay = TIMECOUNT_5s;
            FISD_boCalendarAndCarSts = TRUE;
            rvsmode_back2normal_counter = 0;
            FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 3);
        }
        else if((rl_door_status==TRUE)&&(last_rl_door_status==FALSE))
        {
            FISD_u16CalendarAndCarStsDelay = TIMECOUNT_5s;
            FISD_boCalendarAndCarSts = TRUE;
            rvsmode_back2normal_counter = 0;
            FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 4);
        }
        else if(rvs_mode==FALSE)
        {
            if(rvsmode_back2normal_counter)
            {
                rvsmode_back2normal_counter--;
            }
            if(rvsmode_back2normal_counter==0)
            {
                FISD_u16CalendarAndCarStsDelay = TIMECOUNT_5s;
            }
        }
        else if(((rvs_mode==TRUE)&&(boLastRVSmode==FALSE))||((rvs_mode==TRUE)&&(parton_to_fullon_entry_counter==1)))
        {
            FISD_u16CalendarAndCarStsDelay = 0;
            FISD_boCalendarAndCarSts = FALSE;
            rvsmode_back2normal_counter = 200;
            FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 5);
        }
        else
        {
            rvsmode_back2normal_counter = 200;
        }

        if(FISD_u16CalendarAndCarStsDelay > 0u)
        {
            FISD_u16CalendarAndCarStsDelay --;

        }
        if(FISD_u16CalendarAndCarStsDelay>0)
        {
            FISD_boCalendarAndCarSts = TRUE;
            FISD_u16CalendarAndCarStsDelay = TIMECOUNT_5s;
            FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 6);
        }
        parton_to_fullon_entry_counter = 0;
        boLastRVSmode = rvs_mode;
		last_driver_door_status = driver_door_status;
		last_passenger_door_status = passenger_door_status;
		last_rr_door_status = rr_door_status;
		last_rl_door_status = rl_door_status;
    }
    else if(sys_parton&&(acc_off==FALSE))
    {
        FISD_boCalendarAndCarSts = TRUE;
        FISD_u16CalendarAndCarStsDelay = TIMECOUNT_5s;
        parton_to_fullon_entry_counter = 1;
        FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 7);
    }
    else if(sys_parton&&acc_off)
    {
        if((((driver_door_status==TRUE)&&(last_driver_door_status==FALSE))||\
        ((passenger_door_status==TRUE)&&(last_passenger_door_status==FALSE))||\
        ((rr_door_status==TRUE)&&(last_rr_door_status==FALSE))||\
        ((rl_door_status==TRUE)&&(last_rl_door_status==FALSE))) && (FALSE == boNmStatus))
        {
            FISD_u16CalendarAndCarStsDelay = TIMECOUNT_5s;
            last_driver_door_status = driver_door_status;
		    last_passenger_door_status = passenger_door_status;
		    last_rr_door_status = rr_door_status;
		    last_rl_door_status = rl_door_status;
        }
        else if((u8ArmingSts==FISD_ArmingSuccessful)&&(u8LastArmingsts==0))
        {
            FISD_u16CalendarAndCarStsDelay = 0;
            FISD_boCalendarAndCarSts = FALSE;
            u8LastArmingsts = u8ArmingSts;
            FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 8);
        }
        else if(TRUE == boNmStatus)
        {
            FISD_boCalendarAndCarSts = FALSE;
            FISD_u16CalendarAndCarStsDelay = 0u;
            FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 9);
        }
        else
        {
        }

        if(FISD_u16CalendarAndCarStsDelay > 0u)
        {
            FISD_u16CalendarAndCarStsDelay --;

        }
        if(FISD_u16CalendarAndCarStsDelay>0)
        {
            FISD_boCalendarAndCarSts = TRUE;
            FISD_u16CalendarAndCarStsDelay = TIMECOUNT_5s;
            FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 10);
        }
        parton_to_fullon_entry_counter = 1;
    }
    else
    {
        FISD_boCalendarAndCarSts = FALSE;
        FISD_u16CalendarAndCarStsDelay = 0u;
        FISD_vSetBacklightControlRecord(FISD_boCalendarAndCarSts, 11);
    }
    
}
#endif
#undef FISD_ArmingSuccessful
#undef FISD_DisarmingSuccessful


/*******************************************************************************************
* Function: FISD_vAccAebFcwDetect
* Description: Show Adaptive cruise control,Automatic Emergency Braking,and Pre-collision warning system warning infomation
* Parameters: none
* Return: none
********************************************************************************************/
//static U16 u16TarObjValPre = 0u;
static BOOL boCurTimeGap_preIsZero;

static void Clear_vAccAebFcwStatus(void);
static void FISD_vACCGepIndicator(void);
static void FISD_vAEB_FCWGepIndicator(void);
static void FISD_vTimeGapIndicator(void);
static void FISD_vTextInfo(void);
static void FISD_vACCSpeedText(void);

static void FISD_vAccAebFcwDetect(void)
{
    BOOL accaeb_condition = TRUE;

    /*Clear Status*/
    Clear_vAccAebFcwStatus();
	DIC_SET(nDIC_TimeGap, nDic_Inactive);
	DIC_SET(nDIC_ACCSpeed, nDic_Inactive);
	DIC_SET(nDIC_TimeGapSetICM_1, nDic_Inactive);
    if((TRUE == Rte_FISD_boAnimationPlayIsFinish())&&accaeb_condition)
    {
        /*indicator*/
        FISD_vACCGepIndicator();/*ACC Gep IND*/
        FISD_vAEB_FCWGepIndicator();  /*AEB,FCW Gep IND*/
        FISD_vTimeGapIndicator();

        /*text*/
		FISD_vACCSpeedText();
        if(!Rte_boGetTextinfoTimeout())
        {
            FISD_vTextInfo();
        }

        if((TRUE == Rte_boGetFCWModeTimeout()) ||(TRUE == Rte_boGetAEBModeTimeout()) ||\
                (TRUE == Rte_boGetTimeGapSet_ICMTimeout()) ||(TRUE == Rte_boGetACCModeTimeout()))
        {
            Clear_vAccAebFcwStatus();
        }

        if((TRUE == Rte_boGetACCModeTimeout()) && ((Rte_FISD_u8GetCONFIG_FRM() ==TRUE)||(Rte_FISD_u8GetCONFIG_FCM() ==TRUE)))
        {
            //IND_SET( nIND_ACC_YellowEmpty,nInd_On);  8155没有空心，不知道这个空心是啥
            IND_SET( nIND_ACC_YellowSolid,nInd_On); // 8155改成实心
            IND_SET( nIND_FCW_AEB_Yellow,nInd_On);
        }

        if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
        {
            IND_SET(nIND_ACC_GreyBlack,nInd_Off);
            IND_SET(nIND_ACC_Greywhite,nInd_Off);

            IND_SET(nIND_ACC_GreenBlack,nInd_Off);
            IND_SET(nIND_ACC_GreenGreen,nInd_Off);

            IND_SET( nIND_ACC_YellowSolid,nInd_Off);
          
            IND_SET( nIND_FCW_AEB_Yellow,nInd_Off);
            IND_SET( nIND_FCW_Red,nInd_Off);
            IND_SET(nIND_AEB_OFF_Yellow,nInd_Off);
			if((Rte_FISD_u8GetCONFIG_FRM() == TRUE) || (Rte_FISD_u8GetCONFIG_FCM() == TRUE))
			{
            	IND_SET( nIND_FCW_AEB_Yellow,nInd_On);
				//IND_SET( nIND_ACC_YellowEmpty,nInd_On);
				IND_SET( nIND_ACC_YellowSolid,nInd_On); //8155改成实心
			}
        }

        /*proipty:ACC > Curise > LimitSpd > Cruisetrl*/
        if((!IND_CHK(nIND_ACC_GreyBlack,nInd_Off)) ||\
                (!IND_CHK(nIND_ACC_Greywhite,nInd_Off)) ||\
                (!IND_CHK(nIND_ACC_GreenBlack,nInd_Off)) ||\
                (!IND_CHK(nIND_ACC_GreenGreen,nInd_Off)) ||\
                (!IND_CHK(nIND_ACC_YellowSolid,nInd_Off)) ||
                (!IND_CHK(nIND_ACC_YellowEmpty,nInd_Off)))
        {
            /*Curise*/
            IND_SET(nIND_Cruise, nInd_Off);

            /*LimitSpd*/
            IND_SET(nIND_LimitOn, nInd_Off);
            IND_SET(nIND_LimitCancel, nInd_Off);
            IND_SET(nIND_LimitDefault, nInd_Off);
			IND_SET(nIND_LimitOn_Green, nInd_Off);//EV
			IND_SET(nIND_LimitOn_Grey, nInd_Off);//EV
            IND_SET(nIND_LimitSpdFlash, nInd_Off);
            BUZ_SET(nBUZ_OVS_3, nBuz_Inactive);
			DIC_SET(nDIC_LimitSpeedCancel, nDic_Inactive);
			BUZ_SET(nBUZ_LimitSpeedCancel, nBuz_Inactive);

            /*CruiseCtrl*/
            IND_SET(nIND_CruiseOn, nInd_Off);
            IND_SET(nIND_CruiseCancel, nInd_Off);
            IND_SET(nIND_CancelDisplay, nInd_Off);
            IND_SET(nIND_CruiseSpdFlash, nInd_Off);
        }
        else if((FALSE == IND_CHK(nIND_LimitOn,nInd_Off)) ||\
                (FALSE == IND_CHK(nIND_LimitCancel,nInd_Off)) ||\
                (FALSE == IND_CHK(nIND_LimitDefault,nInd_Off)) ||\
                (FALSE == IND_CHK(nIND_LimitSpdFlash,nInd_Off)) ||\
                (FALSE == IND_CHK(nIND_LimitOn_Green,nInd_Off)) ||\
                (FALSE == IND_CHK(nIND_LimitOn_Grey,nInd_Off)))
        {
            /*Cruise*/
            IND_SET(nIND_CruiseOn, nInd_Off);
            IND_SET(nIND_CruiseCancel, nInd_Off);
            IND_SET(nIND_CancelDisplay, nInd_Off);
            IND_SET(nIND_CruiseSpdFlash, nInd_Off);
        }
        else
        {
        }

    }
    else
    {
        //u16TarObjValPre = 0u;
        boCurTimeGap_preIsZero = FALSE;
        Clear_vAccAebFcwStatus();
        IND_SET(nIND_NotFindCar_OneBar, nInd_Off);
        IND_SET(nIND_NotFindCar_TwoBar, nInd_Off);
        IND_SET(nIND_NotFindCar_ThreeBar, nInd_Off);
        IND_SET(nIND_FindCar_OneBar, nInd_Off);
        IND_SET(nIND_FindCar_TwoBar, nInd_Off);
        IND_SET(nIND_FindCar_ThreeBar, nInd_Off);
    }
    if((FISD_boIndFuncValid[nIND_ACC_YellowEmpty] == TRUE) && \
            (TRUE == FISD_boInstrumentIsInSelfCheckSts()))
    {
    	if((Rte_FISD_u8GetCONFIG_FRM() == TRUE) || (Rte_FISD_u8GetCONFIG_FCM() == TRUE))
    	{
        	//IND_SET(nIND_ACC_YellowEmpty, nInd_On);
        	IND_SET(nIND_ACC_YellowSolid, nInd_On);
			IND_SET(nIND_FCW_AEB_Yellow, nInd_On);
    	}
       
    }

}

static void FISD_vTimeGapIndicator(void)
{
    U8 u8CurTimeGap = FISD_u16SignalValue[nTimeGapSet_ICM];
    U8 u8ObjValid = FISD_u16SignalValue[nObjValid];
	BOOL sys_fullon = boSystemOOMFullOnMode();
	static U8 u8CurTimeGap_pre;
	
	if(u8CurTimeGap == 0)
	{
		boCurTimeGap_preIsZero = FALSE;
	}
	else if(u8CurTimeGap_pre != u8CurTimeGap)
	{
		if(u8CurTimeGap_pre != 0)
		{
			boCurTimeGap_preIsZero = TRUE;
		}
		else
		{
			boCurTimeGap_preIsZero = FALSE;	
		}
	}
	if(((1 <= u8CurTimeGap) && (u8CurTimeGap <= 5)) && (boCurTimeGap_preIsZero == TRUE))
	{
		DIC_SET(nDIC_TimeGap, nDic_Active);
		if(u8CurTimeGap_pre != u8CurTimeGap)
		{
			if(TRUE == Rte_FISD_boGetTextShowFinishFlag(nDIC_TimeGap))
			{
			   	DIC_SET(nDIC_TimeGap, nDic_Inactive);/*重新触发门开显示*/
			}
			else
			{
			    FISD_vSetTextShowTimeResetFlag(nDIC_TimeGap);/*延长文字的显示时间*/
			}
		}
	}
	switch(u8CurTimeGap)
	{
		case 1u:                                                     /*tauGap_1*/
	        IND_SET(nIND_NotFindCar_OneBar, nInd_On);
	        break;
	    case 2u:                                                     /*tauGap_2*/
	        IND_SET(nIND_NotFindCar_TwoBar, nInd_On);
	        break;
	    case 3u:                                                     /*tauGap_3*/
	        IND_SET(nIND_NotFindCar_ThreeBar, nInd_On);
	        break;
	    case 4u:                                                     /*tauGap_4*/
	        IND_SET(nIND_TimeGap_Fourbar, nInd_On);
	        break;
	    case 5u:                                                     /*tauGap_5*/
	        IND_SET(nIND_TimeGap_Fivebar, nInd_On);
	        break;

	    default:
	        break;
	}
	u8CurTimeGap_pre = u8CurTimeGap;
}

static void FISD_vAEB_FCWGepIndicator(void)
{
	if(False == Rte_FISD_u8GetCONFIG_T18FL4())
	{
	    if(FISD_u16SignalValue[nFCWMode] == 0x02)
	    {
	        IND_SET( nIND_FCW_Red,nInd_On);
	    }
		else if(FISD_u16SignalValue[nFCWMode] == 0x03)
		{
			IND_SET( nIND_FCW_AEB_Yellow,nInd_On);
		}
	    else if(FISD_u16SignalValue[nAEBMode] == 0x02)
	    {
	        IND_SET( nIND_FCW_Red,nInd_On);
	    }
	    else if(FISD_u16SignalValue[nAEBMode] == 0x00)
	    {
	        IND_SET(nIND_AEB_OFF_Yellow,nInd_On);
	    }
	    else if(FISD_u16SignalValue[nAEBMode] == 0x03)
	    {
	        IND_SET(nIND_FCW_AEB_Yellow,nInd_On);
	    }
	    else
	    {
	    }
    }
    else
    {
        if(FISD_u16SignalValue[nAEBMode] == 0x02)
	    {
	        IND_SET( nIND_FCW_Red,nInd_On);
	    }
	    else if(FISD_u16SignalValue[nAEBMode] == 0x00)
	    {
	        IND_SET(nIND_AEB_OFF_Yellow,nInd_On);
	    }
	    else if(FISD_u16SignalValue[nAEBMode] == 0x03)
	    {
	        IND_SET(nIND_FCW_AEB_Yellow,nInd_On);
	    }
        else if(FISD_u16SignalValue[nFCWMode] == 0x02)
	    {
	        IND_SET( nIND_FCW_Red,nInd_On);
	    }
		else if(FISD_u16SignalValue[nFCWMode] == 0x03)
		{
			IND_SET( nIND_FCW_AEB_Yellow,nInd_On);
		}
        else if(FISD_u16SignalValue[nFCWMode] == 0x00) 
		{
			IND_SET( nIND_AEB_OFF_Yellow,nInd_On);
		}
	    else
	    {
	    }
        
    }

    /*国际项目工作指示FCM_FRM_6_HMI_Indicator*/
    if(IlGetFCM_FRM_6_HMI_IndicatorRxTimeout() == FALSE)
    {
        if(IlGetRxFCM_FRM_6_HMI_Indicator() == 1)
        {
            IND_SET(nIND_AEB_OFF_Yellow,nInd_On);
        }
        else if(IlGetRxFCM_FRM_6_HMI_Indicator() == 2)
        {
            IND_SET(nIND_FCW_AEB_Yellow,nInd_On);
        }
        else if(IlGetRxFCM_FRM_6_HMI_Indicator() == 3)
        {
            IND_SET( nIND_FCW_Red,nInd_On);
        }
        else
        {

        }
    }
}

static void FISD_vACCGepIndicator(void)
{
    if((FISD_u16SignalValue[nACCMode] == 0x06) ||(FISD_u16SignalValue[nACCMode] ==0x07) ||\
      (FISD_u16SignalValue[nACCMode] == 0x02))
    {
        IND_SET(nIND_ACC_Greywhite,nInd_On);
    }
    else if((FISD_u16SignalValue[nACCMode] >= 0x03) && (FISD_u16SignalValue[nACCMode] <= 0x05))
    {
        IND_SET(nIND_ACC_GreenGreen,nInd_On);
    }
    else if(FISD_u16SignalValue[nACCMode] == 0x09)
    {
        IND_SET(nIND_ACC_YellowSolid,nInd_On);
    }
    else /*include ACC_mode is 0x00,0x08,or 0x10-0x15*/
    {
    }
}

static void FISD_vACCSpeedText(void)
{
	U16 u16VSetDis;
	static U16 u16VSetDis_pre = 0;
	U8 u8TimeGapSet_ICM = FISD_u16SignalValue[nTimeGapSet_ICM];
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	static U8 u8TimeGapSet_ICM_pre = 0;
	boACCSpeedSts = FALSE;
	if(Rte_boGetVSetDisTimeout() == FALSE)
	{
		u16VSetDis = FISD_u16SignalValue[nVSetDis];
	}
	else
	{
		u16VSetDis = 0;
	}
	
	if(((FISD_u16SignalValue[nACCMode] == 0x03) && (Rte_boGetACCModeTimeout() == FALSE)) ||\
		((FISD_u16SignalValue[nTJA_ICA_mode] == 0x02) && (Rte_boGetTJA_ICA_modeTimeout() == FALSE)) ||\
		((FISD_u16SignalValue[nCPMod] == 0x03) && (Rte_boGetCPModTimeout() == FALSE))||\
		((FISD_u16SignalValue[nCPMod] == 0x06) && (Rte_boGetCPModTimeout() == FALSE)))
	{
		if(u16VSetDis > 0)
		{
			boACCSpeedSts = TRUE;
			DIC_SET(nDIC_ACCSpeed,nDic_Active);
			
			if((u16VSetDis != u16VSetDis_pre) && (u16VSetDis > 0))
			{
				if(TRUE == Rte_FISD_boGetTextShowFinishFlag(nDIC_ACCSpeed))
				{
				   	DIC_SET(nDIC_ACCSpeed, nDic_Inactive);/*重新触发门开显示*/
				}
				else
				{
				    FISD_vSetTextShowTimeResetFlag(nDIC_ACCSpeed);/*延长文字的显示时间*/
				}
			}
			
		}

		if(u8TimeGapSet_ICM == 0)
		{
			boCurTimeGapSet_ICMpreIsZero = FALSE;
		}
		else if(u8TimeGapSet_ICM_pre != u8TimeGapSet_ICM)
		{
			if(u8TimeGapSet_ICM_pre != 0)
			{
				boCurTimeGapSet_ICMpreIsZero = TRUE;
			}
			else
			{
				boCurTimeGapSet_ICMpreIsZero = FALSE;	
			}
		}
		
		if(((1 <= u8TimeGapSet_ICM) && (u8TimeGapSet_ICM <= 5)) && (boCurTimeGapSet_ICMpreIsZero == TRUE))
		{
			DIC_SET(nDIC_TimeGapSetICM_1, nDic_Active);
			if(u8TimeGapSet_ICM_pre != u8TimeGapSet_ICM)
			{
				if(TRUE == Rte_FISD_boGetTextShowFinishFlag(nDIC_TimeGapSetICM_1))
				{
				   	DIC_SET(nDIC_TimeGapSetICM_1, nDic_Inactive);/*重新触发门开显示*/
				}
				else
				{
				    FISD_vSetTextShowTimeResetFlag(nDIC_TimeGapSetICM_1);/*延长文字的显示时间*/
				}
			}
		}
	}

	u16VSetDis_pre = u16VSetDis;
	u8TimeGapSet_ICM_pre = u8TimeGapSet_ICM;
}


static void FISD_vTextInfo(void)
{
    switch(FISD_u16SignalValue[nTextinfo] )
    {
    case 0x00:
        break;
    case 0x01:
        DIC_SET(nDIC_AccCancel, nDic_Active);
        break;
    case 0x02:
        DIC_SET(nDIC_AccActiveFalse, nDic_Active);
		BUZ_SET(nBUZ_AccActiveFalse, nBuz_Active);
        break;
    case 0x03:
        DIC_SET(nDIC_AccStandWait, nDic_Active);
		BUZ_SET(nBUZ_AccStandWait, nBuz_Active);
        break;
    case 0x04:
        DIC_SET(nDIC_DrivewayInformationActive, nDic_Active);
        BUZ_SET(nBUZ_DriveAway, nBuz_Active);/* 新增一个报警，info一声 */
        break;
    case 0x05:
        DIC_SET(nDIC_AebSwitchOff_Textinfo, nDic_Active);
		BUZ_SET(nBUZ_AebSwitchOff_Textinfo, nBuz_Active);
        break;
    case 0x06:
        DIC_SET(nDIC_AebSwitchOn, nDic_Active);
		BUZ_SET(nBUZ_AebSwitchOn, nBuz_Active);
        break;
    case 0x07:
        DIC_SET(nDIC_AebInactive, nDic_Active);
		BUZ_SET(nBUZ_AebInactive, nBuz_Active);
        break;
    case 0x08:
        DIC_SET(nDIC_AebActive, nDic_Active);
        break;
    case 0x09:
        DIC_SET(nDIC_FcwSwitchOff, nDic_Active);
		BUZ_SET(nBUZ_FcwSwitchOff, nBuz_Active);
        break;
    case 0x0A:
        DIC_SET(nDIC_FcwSwitchOn, nDic_Active);
		BUZ_SET(nBUZ_FcwSwitchOn, nBuz_Active);
        break;
    case 0x0B:
        DIC_SET(nDIC_FcwActiveFalse, nDic_Active);
		BUZ_SET(nBUZ_FcwActiveFalse, nBuz_Active);
        break;
    case 0x0C:
        DIC_SET(nDIC_FcwActive, nDic_Active);
		BUZ_SET(nBUZ_FCWActive, nBuz_Active);
        break;
	case 0x0D:
        DIC_SET(nDIC_SCFSwitchedOn, nDic_Active);
        break;
	case 0x0E:
        DIC_SET(nDIC_SCFSwitchedOff, nDic_Active);
        break;
	case 0x0F:
        DIC_SET(nDIC_UnableToSwitchedOnSCF, nDic_Active);
        break;
    default:
        break;
    }
    
    switch(FISD_u16SignalValue[nTextinfoWarn] )
    {
    case 0x00:
        break;
    case 0x01:
        DIC_SET(nDIC_TakeOverReq, nDic_Active);
        BUZ_SET(nBUZ_DriverTakeOver, nBuz_Active);
        break;
    case 0x02:
        DIC_SET(nDIC_AccErr, nDic_Active);
        break;
    case 0x03:
        DIC_SET(nDIC_AebErr, nDic_Active);
        break;
    case 0x04:
        DIC_SET(nDIC_RadarCover, nDic_Active);
        break;
    case 0x05:
        DIC_SET(nDIC_FcwErr, nDic_Active);
        break;
    case 0x06:
        DIC_SET(nDIC_RadarErr, nDic_Active);
        break;
    case 0x07:
        //DIC_SET(nDIC_AebInactive, nDic_Active);
		
        break;
    default:
        break;
    }

#ifdef ENABLE_TEXT_SOUND_MIN_SHOW_2S
    if(nDIC_TakeOverReq == TextMgr_enGetDisplayText())
    {
        BUZ_SET(nBUZ_DriverTakeOver, nBuz_Active);
    }
    else
    {
    }
#endif
}

static void Clear_vAccAebFcwStatus(void)
{
    /*ACC Gep*/
    IND_SET(nIND_ACC_GreyBlack,nInd_Off);
    IND_SET(nIND_ACC_Greywhite,nInd_Off);

    IND_SET(nIND_ACC_GreenBlack,nInd_Off);
    IND_SET(nIND_ACC_GreenGreen,nInd_Off);

    IND_SET(nIND_ACC_YellowSolid,nInd_Off);
    IND_SET(nIND_ACC_YellowEmpty,nInd_Off);
    /*AEB and FCW Gep*/
    IND_SET(nIND_FCW_AEB_Yellow,nInd_Off);
    IND_SET(nIND_FCW_Red,nInd_Off);
    IND_SET(nIND_AEB_OFF_Yellow,nInd_Off);

    /*Obj Car*/
    BUZ_SET(nBUZ_DriverTakeOver, nBuz_Inactive);
    BUZ_SET(nBUZ_FCWActive, nBuz_Inactive);

    DIC_SET(nDIC_TakeOverReq, nDic_Inactive);

    DIC_SET(nDIC_AccCancel, nDic_Inactive);
    //DIC_SET(nDIC_AccSwitchOff, nDic_Inactive);
    DIC_SET(nDIC_AccActiveFalse, nDic_Inactive);
	BUZ_SET(nBUZ_AccActiveFalse, nBuz_Inactive);
    DIC_SET(nDIC_AccErr, nDic_Inactive);

    DIC_SET(nDIC_AebSwitchOff_Textinfo, nDic_Inactive);
	BUZ_SET(nBUZ_AebSwitchOff_Textinfo, nBuz_Inactive);
    DIC_SET(nDIC_AebSwitchOn, nDic_Inactive);
	BUZ_SET(nBUZ_AebSwitchOn, nBuz_Inactive);
    DIC_SET(nDIC_AebInactive, nDic_Inactive);
	BUZ_SET(nBUZ_AebInactive, nBuz_Inactive);
    DIC_SET(nDIC_AebActive, nDic_Inactive);
    DIC_SET(nDIC_AebErr, nDic_Inactive);

    DIC_SET(nDIC_RadarCover, nDic_Inactive);
    DIC_SET(nDIC_FcwSwitchOff, nDic_Inactive);
	BUZ_SET(nBUZ_FcwSwitchOff, nBuz_Inactive);
    DIC_SET(nDIC_FcwActiveFalse, nDic_Inactive);
	BUZ_SET(nBUZ_FcwActiveFalse, nBuz_Inactive);
    DIC_SET(nDIC_FcwSwitchOn, nDic_Inactive);
	BUZ_SET(nBUZ_FcwSwitchOn, nBuz_Inactive);
    DIC_SET( nDIC_FcwActive, nDic_Inactive);
	BUZ_SET(nBUZ_FCWActive, nBuz_Inactive);
    DIC_SET(nDIC_FcwErr, nDic_Inactive);
    DIC_SET(nDIC_RadarErr, nDic_Inactive);
    DIC_SET(nDIC_AccStandWait,nDic_Inactive);
	BUZ_SET(nBUZ_AccStandWait, nBuz_Inactive);
    DIC_SET(nDIC_DrivewayInformationActive,nDic_Inactive);
	BUZ_SET(nBUZ_DriveAway, nBuz_Inactive);
	DIC_SET(nDIC_SCFSwitchedOn, nDic_Inactive);
	DIC_SET(nDIC_SCFSwitchedOff, nDic_Inactive);
	DIC_SET(nDIC_UnableToSwitchedOnSCF, nDic_Inactive);

	IND_SET(nIND_NotFindCar_OneBar, nInd_Off);
    IND_SET(nIND_NotFindCar_TwoBar, nInd_Off);
    IND_SET(nIND_NotFindCar_ThreeBar, nInd_Off);
    IND_SET(nIND_FindCar_OneBar, nInd_Off);
    IND_SET(nIND_FindCar_TwoBar, nInd_Off);
    IND_SET(nIND_FindCar_ThreeBar, nInd_Off);
    IND_SET(nIND_TimeGap_Fourbar, nInd_Off);
    IND_SET(nIND_TimeGap_Fivebar, nInd_Off);
}


/*******************************************************************************************
* Function: FISD_vHmaDetect
* Description: Show
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vHmaDetect(void)
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	BOOL sys_parton = boSystemOOMPartOnMode();
	U8 u8HmaStsValue = FISD_u16SignalValue[nHMA_Status];
	BOOL boHmaSigTimeOut = Rte_boGetHMA_StatusTimeout();
	
	U8 u8AdbStsValue = FISD_u16SignalValue[nADBSts];
	BOOL boAdbSigTimeOut = Rte_boGetADBStsTimeout();

	BOOL boWhiteFlag = FALSE;
	BOOL boYellowFlag = FALSE;
	
    /*Clear Status*/
    IND_SET(nIND_HMA_Grey, nInd_Off);
    IND_SET(nIND_HMA_White, nInd_Off);
    IND_SET(nIND_HMA_Yellow, nInd_Off);

    DIC_SET(nDIC_CameraFault, nDic_Inactive);
    DIC_SET(nDIC_CameraDirty, nDic_Inactive);
    DIC_SET(nDIC_OpenAirXConditioning, nDic_Inactive);
	DIC_SET(nDIC_ADBFault, nDic_Inactive);
	BUZ_SET(nBUZ_ADBFault, nBuz_Inactive);


    /*status detect*/
	if((!boHmaSigTimeOut) && ((TRUE == Rte_FISD_boAnimationPlayIsFinish()) || sys_parton))
	{
		if(0x02 == u8HmaStsValue)
		{
			boWhiteFlag = TRUE;
		}
		else if(0x03 == u8HmaStsValue)
		{
			boYellowFlag = TRUE;
		}
		if(sys_fullon == TRUE)
		{
		    if(FISD_u16SignalValue[nCamera_textinfo] == 0x01)
		    {
		        DIC_SET(nDIC_CameraFault, nDic_Active);
		    }
		    else if(FISD_u16SignalValue[nCamera_textinfo] == 0x02)
		    {
		        DIC_SET( nDIC_CameraDirty, nDic_Active);
		    }
		    else if(FISD_u16SignalValue[nCamera_textinfo] == 0x03)
		    {
		        DIC_SET( nDIC_OpenAirXConditioning, nDic_Active);
		    }
		    else
		    {
		    }
		}
	}

	if((!boAdbSigTimeOut)&&((TRUE == Rte_FISD_boAnimationPlayIsFinish()) || sys_parton))
	{
		if(0x01 == u8AdbStsValue)
		{
			boWhiteFlag = TRUE;
		}
		else if(0x02 == u8AdbStsValue)
		{
			boWhiteFlag = TRUE;
			IND_SET(nIND_HighBeam, nInd_On);
		}
		else if(0x03 == u8AdbStsValue)
		{
			boYellowFlag = TRUE;
			DIC_SET(nDIC_ADBFault, nDic_Active);
			BUZ_SET(nBUZ_ADBFault, nBuz_Active);
		}
	}
	
	if(boYellowFlag)
	{
		IND_SET(nIND_HMA_Yellow, nInd_On);
	}
	else if(boWhiteFlag)
	{
		IND_SET(nIND_HMA_White, nInd_On);		
	}

	if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
	{
	    //IND_SET(nIND_HMA_Grey, nInd_Off);
	    //IND_SET(nIND_HMA_White, nInd_Off);
	    //IND_SET(nIND_HMA_Yellow, nInd_Off);

	    DIC_SET(nDIC_CameraFault, nDic_Inactive);
	    DIC_SET(nDIC_CameraDirty, nDic_Inactive);
		DIC_SET(nDIC_ADBFault, nDic_Inactive);
        DIC_SET(nDIC_OpenAirXConditioning, nDic_Inactive);
		
	}


}



/*******************************************************************************************
* Function: FISD_vSlaDetect
* Description: Show
* Parameters: none
* Return: none
********************************************************************************************/
volatile U8 SLA_vu8LimitSpd = 0u;
static void FISD_vClearSlaSts(void);

static void FISD_vSlaDetect(void)
{
    //update later  5.29
    FISD_vClearSlaSts();
    BUZ_SET(nBUZ_SCF,nBuz_Inactive);
    DIC_SET(nDIC_DriverconfirmRequest, nDic_Inactive);

    /*redetect*/
    if((TRUE == Rte_FISD_boAnimationPlayIsFinish()) && (1 == FISD_u16SignalValue[nSLAOnOffsts]))
    {
        if(FISD_u16SignalValue[nSLAState] == 4u)
        {
            IND_SET(nIND_SLASpd_Defect,nInd_On);
        }
        else if((FISD_u16SignalValue[nSLAState] > 0u) && (FISD_u16SignalValue[nSLAState] <= 3u))
        {
            if(FISD_u16SignalValue[nSLASpdlimit] == 0u)
            {
                //IND_SET(nIND_SLASpd_Defect,nInd_On);
                SLA_vu8LimitSpd = 0u;
            }
            else if(FISD_u16SignalValue[nSLASpdlimit] == 1u)
            {
                IND_SET(nIND_SLASpd_40,nInd_On);
            }
            else if((FISD_u16SignalValue[nSLASpdlimit] > 1u) && (FISD_u16SignalValue[nSLASpdlimit] < 28u))
            {
                IND_SET(nIND_SLASpd_120,nInd_On);
                SLA_vu8LimitSpd = (U8)((FISD_u16SignalValue[nSLASpdlimit]-1u) * 5u);
            }
            else
            {
            }
            SLA_vu8LimitSpd = (SLA_vu8LimitSpd > 130) ? 130 : SLA_vu8LimitSpd;
        }
        else
        {
        }

        if(FISD_u16SignalValue[nSLASpdlimitWarning] == 1u)
        {
            IND_SET(nIND_SlaSpdLimitWarn,nInd_On);        //红色圆圈跳动，效果如超速报警显示方案
        }

    }
	if(TRUE == Rte_FISD_boAnimationPlayIsFinish())
	{
		if(FISD_u16SignalValue[nSLASpdlimitWarning] == 1u)
		{
			BUZ_SET(nBUZ_SLA,nBuz_Active);
		}
		
        if(FISD_u16SignalValue[nSCF_PopoverReq] == 1u)
        {
            BUZ_SET(nBUZ_SCF,nBuz_Active);
            DIC_SET(nDIC_DriverconfirmRequest, nDic_Active);
        }
	}
	if((Rte_FISD_enGetOverspeedWarn() == SOS_nWarnGSO) ||\
	(Rte_FISD_enGetOverspeedWarn() == SOS_nWarn120kmph))
	{
		 FISD_vClearSlaSts();
	}
}

BOOL FISD_Get_boSlaOnOffStatus(void)
{
    return FISD_u16SignalValue[nSLAOnOffsts];
}
static void FISD_vClearSlaSts(void)
{
    IND_SET(nIND_SLASpd_Defect,nInd_Off);
    IND_SET(nIND_SLASpd_40,nInd_Off);
    IND_SET(nIND_SLASpd_120,nInd_Off);
	IND_SET(nIND_SlaSpdLimitWarn,nInd_Off);
    IND_SET(nIND_OverSpeedWarn,nInd_Off);
    BUZ_SET(nBUZ_SLA,nBuz_Inactive);
    //BUZ_SET(nBUZ_SCF,nBuz_Inactive);
    //DIC_SET(nDIC_DriverconfirmRequest, nDic_Inactive);
}

static void FISD_vClearIsaSts(void)
{
    FISD_vClearSlaSts();
     
    IND_SET(nIND_ISA_SpdLimtyp1, nInd_Off);
    IND_SET(nIND_ISA_SpdLimtyp2, nInd_Off);
    IND_SET(nIND_ISA_SPLUnconfirmed, nInd_Off);
    IND_SET(nIND_ISA_FuncErr, nInd_Off);  
    IND_SET(nIND_ISA_UnknownWarning, nInd_Off);
	IND_SET(nIND_SCF_PopoverReq, nInd_Off);
	IND_SET(nIND_ISA_bell, nInd_Off);
	IND_SET(nIND_ISA_horn, nInd_Off);
	IND_SET(nIND_ISA_OFF, nInd_Off);
    
    BUZ_SET(nBUZ_SCFSpdlimWarn,nBuz_Inactive);
    BUZ_SET(nBUZ_SLASpdlimitWarningAud,nBuz_Inactive);
    //BUZ_SET(nBUZ_SCF,nBuz_Inactive);
    //DIC_SET(nDIC_DriverconfirmRequest, nDic_Inactive);
}


static void FISD_vIsaDetect(void)
{
    //update later  5.29
    FISD_vClearIsaSts();
    IND_SET(nIND_SCF_PopoverReq, nInd_Off);
    BUZ_SET(nBUZ_SCF,nBuz_Inactive);
    BUZ_SET(nBUZ_ISA_SCF_PopoverReq,nBuz_Inactive);
    DIC_SET(nDIC_DriverconfirmRequest, nDic_Inactive);

    /*redetect*/
    if((TRUE == Rte_FISD_boAnimationPlayIsFinish()) && (1 == FISD_u16SignalValue[nSLAOnOffsts]))
    {
        if(FISD_u16SignalValue[nSLAState] == 4u)
        {
            IND_SET(nIND_ISA_FuncErr,nInd_On);
            IND_SET(nIND_SLASpd_Defect,nInd_On);
        }
        else if((FISD_u16SignalValue[nSLAState] > 0u) && (FISD_u16SignalValue[nSLAState] <= 3u))
        {
            if(FISD_u16SignalValue[nSLASpdlimit] == 0u)
            {
                //IND_SET(nIND_SLASpd_Defect,nInd_On);
                SLA_vu8LimitSpd = 0u;
            }
            else if(FISD_u16SignalValue[nSLASpdlimit] == 1u)
            {
                IND_SET(nIND_SLASpd_40,nInd_On);
            }
            else if((FISD_u16SignalValue[nSLASpdlimit] > 1u) && (FISD_u16SignalValue[nSLASpdlimit] < 38u))
            {
                switch(FISD_u16SignalValue[nSpdLimtyp])
                {
                case 0u:
                    IND_SET(nIND_SLASpd_120,nInd_On);
                break;
                case 1u:
                    IND_SET(nIND_ISA_SpdLimtyp1,nInd_On);
                break;
                case 2u:
                    IND_SET(nIND_ISA_SpdLimtyp2,nInd_On);
                break;
                }
                SLA_vu8LimitSpd = (U8)((FISD_u16SignalValue[nSLASpdlimit]-1u) * 5u);
            }
            else
            {
                if((FISD_u16SignalValue[nSpdLimtyp] == 1u) && (FISD_u16SignalValue[nSLASpdlimit] == 0xFF))
                {
                    IND_SET(nIND_ISA_SPLUnconfirmed,nInd_On);
                }
            }
            SLA_vu8LimitSpd = (SLA_vu8LimitSpd > 130) ? 130 : SLA_vu8LimitSpd;
        }
        else
        {
        }

        if((FISD_u16SignalValue[nSLASpdlimitWarning] == 1u) || (FISD_u16SignalValue[nSLASpdlimitWarningVis] == 1u))
        {
            IND_SET(nIND_OverSpeedWarn,nInd_On);        //红色圆圈跳动，效果如超速报警显示方案
        }

        if(FISD_u16SignalValue[nSLASpdlimitUnknownWarning] == 1u)
        {
            FISD_vClearIsaSts();
            IND_SET(nIND_ISA_UnknownWarning, nInd_On);
        }

        if(FISD_u16SignalValue[nSCF_PopoverReq] == 1u)
        {
            FISD_vClearIsaSts();
            IND_SET(nIND_SCF_PopoverReq, nInd_On);
        }

    }
	if(TRUE == Rte_FISD_boAnimationPlayIsFinish())
	{
		if(FISD_u16SignalValue[nSLASpdlimitWarning] == 1u)
		{
			BUZ_SET(nBUZ_SLA,nBuz_Active);
		}

        if(FISD_u16SignalValue[nSLASpdlimitWarningAud] == 1u)
		{
			BUZ_SET(nBUZ_SLASpdlimitWarningAud,nBuz_Active);
		}

        if(FISD_u16SignalValue[nSCFSpdlimWarn] == 1u)
		{
			BUZ_SET(nBUZ_SCFSpdlimWarn,nBuz_Active);
		}
        
        if(FISD_u16SignalValue[nSCF_PopoverReq] == 1u)
        {
            //BUZ_SET(nBUZ_SCF,nBuz_Active);
            BUZ_SET(nBUZ_ISA_SCF_PopoverReq,nBuz_Active);
            DIC_SET(nDIC_DriverconfirmRequest, nDic_Active);
        }
	}
	if((Rte_FISD_enGetOverspeedWarn() == SOS_nWarnGSO) ||\
	(Rte_FISD_enGetOverspeedWarn() == SOS_nWarn120kmph))
	{
		 FISD_vClearIsaSts();
	}
}

static void FISD_vIsaIsaDetect(void)
{
    //update later  5.29
    FISD_vClearIsaSts();    
    BUZ_SET(nBUZ_SCF,nBuz_Inactive);
    BUZ_SET(nBUZ_ISA_SCF_PopoverReq,nBuz_Inactive);
    DIC_SET(nDIC_DriverconfirmRequest, nDic_Inactive);

    /*redetect*/
    if(TRUE == Rte_FISD_boAnimationPlayIsFinish())
    {
    	if(FISD_u16SignalValue[nISAState] == 1u)
    	{
    		IND_SET(nIND_ISA_OFF, nInd_On);
    	}
		else if(FISD_u16SignalValue[nISAState] < 4u)
		{
	        if(FISD_u16SignalValue[nSLAState] == 4u)
	        {
	            IND_SET(nIND_ISA_FuncErr,nInd_On);
                IND_SET(nIND_SLASpd_Defect,nInd_On);
	        }
	        else if((FISD_u16SignalValue[nSLAState] > 0u) && (FISD_u16SignalValue[nSLAState] <= 3u))
	        {
	            if((FISD_u16SignalValue[nSLASpdlimit] == 0u) ||(FISD_u16SignalValue[nSpdLimtyp] == 3u))
	            {
	                //IND_SET(nIND_SLASpd_Defect,nInd_On);
	                SLA_vu8LimitSpd = 0u;
	            }
	            else if(FISD_u16SignalValue[nSLASpdlimit] == 1u)
	            {
	                IND_SET(nIND_SLASpd_40,nInd_On);
	            }
	            else if((FISD_u16SignalValue[nSLASpdlimit] > 1u) && (FISD_u16SignalValue[nSLASpdlimit] < 38u))
	            {
	                switch(FISD_u16SignalValue[nSpdLimtyp])
	                {
	                case 0u:
	                    IND_SET(nIND_SLASpd_120,nInd_On);
	                break;
	                case 1u:
	                    IND_SET(nIND_ISA_SpdLimtyp1,nInd_On);
	                break;
	                case 2u:
	                    IND_SET(nIND_ISA_SpdLimtyp2,nInd_On);
	                break;
	                }
	                SLA_vu8LimitSpd = (U8)((FISD_u16SignalValue[nSLASpdlimit]-1u) * 5u);
	            }
	            else
	            {
	                if((FISD_u16SignalValue[nSpdLimtyp] == 1u) && (FISD_u16SignalValue[nSLASpdlimit] == 0xFF))
	                {
	                    IND_SET(nIND_ISA_SPLUnconfirmed,nInd_On);
	                }
	            }
	            SLA_vu8LimitSpd = (SLA_vu8LimitSpd > 130) ? 130 : SLA_vu8LimitSpd;
	        }
	        else
	        {
	        }

			/*ISA_bell ISA_horn*/
			if((IND_CHK(nIND_SLASpd_40, nInd_On))||(IND_CHK(nIND_SLASpd_120, nInd_On))||\
				(IND_CHK(nIND_ISA_SpdLimtyp1, nInd_On))||(IND_CHK(nIND_ISA_SpdLimtyp2, nInd_On))||(IND_CHK(nIND_ISA_SPLUnconfirmed, nInd_On)))
			{
				if(FISD_u16SignalValue[nISAState] == 2u)
				{
					IND_SET(nIND_ISA_bell, nInd_On);
				}
				else if(FISD_u16SignalValue[nISAState] == 3u)
				{
					IND_SET(nIND_ISA_horn, nInd_On);
				}
			}
		}

		if((IND_CHK(nIND_ISA_bell, nInd_Off))&&(IND_CHK(nIND_ISA_horn, nInd_Off)))
		{
			if(FISD_u16SignalValue[nSLASpdlimitUnknownWarning] == 1u)
			{
				FISD_vClearIsaSts();
				IND_SET(nIND_ISA_UnknownWarning, nInd_On);
			}

			if(FISD_u16SignalValue[nSCF_PopoverReq] == 1u)
			{
				FISD_vClearIsaSts();
				IND_SET(nIND_SCF_PopoverReq, nInd_On);
				//BUZ_SET(nBUZ_SCF,nBuz_Active);
				BUZ_SET(nBUZ_ISA_SCF_PopoverReq,nBuz_Active);
				DIC_SET(nDIC_DriverconfirmRequest, nDic_Active);
			}
		}

		if((FISD_u16SignalValue[nSLASpdlimitWarning] == 1u) || (FISD_u16SignalValue[nSLASpdlimitWarningVis] == 1u))
		{
			IND_SET(nIND_SlaSpdLimitWarn,nInd_On);		  //红色圆圈跳动，效果如超速报警显示方案
		}

		if(FISD_u16SignalValue[nSLASpdlimitWarning] == 1u)
		{
			BUZ_SET(nBUZ_SLA,nBuz_Active);
		}

		if(FISD_u16SignalValue[nSLASpdlimitWarningAud] == 1u)
		{
			BUZ_SET(nBUZ_SLASpdlimitWarningAud,nBuz_Active);
		}

		if(FISD_u16SignalValue[nSCFSpdlimWarn] == 1u)
		{
			BUZ_SET(nBUZ_SCFSpdlimWarn,nBuz_Active);
		}
	}
}

static void FISD_vSLASpdLimChgAud(void)
{
	BUZ_SET(nBUZ_SLASpdLimChgAud,nBuz_Inactive);

	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    if((Rte_boGetSLASpdLimChgAudTimeout() == False) &&\
        (FISD_u16SignalValue[nSLASpdLimChgAud] == 1u) && sys_fullon)
    {
        BUZ_SET(nBUZ_SLASpdLimChgAud,nBuz_Active);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
    	BUZ_SET(nBUZ_SLASpdLimChgAud,nBuz_Inactive);
    }
}

/*******************************************************************************************
* Function: FISD_vTJA_ICADetect
* Description: Show
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vClearTJA_ICAStatus(void);
static void FISD_vTJA_ICADetect(void)
{
    BOOL tja_ind_condition = TRUE;
    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
    static U8 tja_ica_previous_status = 0;
    static U8 tja_ica_keep_timer = 0;

    FISD_vClearTJA_ICAStatus();

    if(TRUE == Rte_FISD_boAnimationPlayIsFinish())
    {
        switch(FISD_u16SignalValue[nTJA_ICA_mode])
        {
        case 1:
            if(tja_ind_condition&&sys_fullon)
                IND_SET(nIND_FuckTraffic_Grey,nInd_On);
            break;
        case 2:
            if(tja_ind_condition&&sys_fullon)
                IND_SET(nIND_FuckTraffic_Green,nInd_On);
            break;
        case 3:
            if(tja_ind_condition&&sys_fullon)
                IND_SET(nIND_FuckTraffic_Yellow,nInd_On);
            break;
        default:
            break;
        }

        switch(FISD_u16SignalValue[nTJA_ICA_Textinfo])
        {
        case 0:
            if(tja_ind_condition&&sys_fullon)
                
            break;
        case 1:
            if(tja_ind_condition&&sys_fullon)
            {
				DIC_SET( nDIC_TJA_ICA_Stop, nDic_Active);
				BUZ_SET( nBUZ_TJA_ICA_Stop, nBuz_Active);
            }    
            break;
        case 2:
            if(tja_ind_condition&&sys_fullon)
            {
				DIC_SET( nDIC_TJA_ICA_On, nDic_Active);
				BUZ_SET(nBUZ_TJA_ICA_On , nBuz_Active);
            }    	
            break;
        case 3:
            if(tja_ind_condition&&sys_fullon)
            {
				DIC_SET(nDIC_TJA_ICA_FailActive, nDic_Active);
				BUZ_SET(nBUZ_TJA_ICA_FailActive , nBuz_Active);		
            }
            break;
        case 5:
            if(tja_ind_condition&&sys_fullon)
            {
            	DIC_SET( nDIC_TJA_ICA_Check, nDic_Active);
            }
			break;
		case 6:
	        if(tja_ind_condition&&sys_fullon)
	        {
	        	DIC_SET( nDIC_TJA_ICA_IsExitReminder, nDic_Active);
	            BUZ_SET(nBUZ_TJAICA_isexitreminder, nBuz_Active);
	        }
            break;
        default:
            break;
       }

       if(1u == FISD_u16SignalValue[nTJAICAEnaRcmend])
       {
           if(tja_ind_condition&&sys_fullon)
                DIC_SET( nDIC_TJA_ICA_Demand, nDic_Active);
		   		BUZ_SET(nBUZ_TJA_ICA_Demand , nBuz_Active);		   		   
       }

        if((Rte_boGetTJA_ICA_modeTimeout() == TRUE) && (tja_ind_condition&&sys_fullon))
        {
            //FISD_vClearTJA_ICAStatus();
            if(Rte_FISD_u8GetCONFIG_TJA() == TRUE)
            {
            	IND_SET(nIND_FuckTraffic_Yellow,nInd_On);
            }
			IND_SET(nIND_FuckTraffic_Grey,nInd_Off);
			IND_SET(nIND_FuckTraffic_Green,nInd_Off);
        }

		if((Rte_boGetTJA_ICA_TextinfoTimeout() == TRUE) && (tja_ind_condition&&sys_fullon))
		{
			DIC_SET( nDIC_TJA_ICA_Demand, nDic_Inactive);
			DIC_SET( nDIC_TJA_ICA_Stop, nDic_Inactive);
			DIC_SET( nDIC_TJA_ICA_On, nDic_Inactive);
			DIC_SET(nDIC_TJA_ICA_FailActive, nDic_Inactive);
			DIC_SET( nDIC_TJA_ICA_Check, nDic_Inactive);
			DIC_SET( nDIC_TJA_ICA_IsExitReminder, nDic_Inactive);

			
			BUZ_SET( nBUZ_TJA_ICA_Stop, nBuz_Inactive);
			BUZ_SET(nBUZ_TJA_ICA_On , nBuz_Inactive);
			BUZ_SET(nBUZ_TJA_ICA_FailActive , nBuz_Inactive);
			BUZ_SET(nBUZ_TJAICA_isexitreminder, nBuz_Inactive);
			BUZ_SET(nBUZ_TJA_ICA_Demand , nBuz_Inactive); 			   
			
		}
        if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
        {
            IND_SET(nIND_FuckTraffic_Green,nInd_Off);
            IND_SET(nIND_FuckTraffic_Grey,nInd_Off);
			if((Rte_FISD_u8GetCONFIG_FCM() == TRUE) && (Rte_FISD_u8GetCONFIG_TJA() == TRUE))
			{
            	IND_SET(nIND_FuckTraffic_Yellow,nInd_On);
			}
        }
    }
    else
    {
        FISD_vClearTJA_ICAStatus();
    }

}

static void FISD_vClearTJA_ICAStatus(void)
{
    IND_SET(nIND_FuckTraffic_Green,nInd_Off);
    IND_SET(nIND_FuckTraffic_Grey,nInd_Off);
    IND_SET(nIND_FuckTraffic_Yellow,nInd_Off);
    DIC_SET( nDIC_TJA_ICA_Stop, nDic_Inactive);
    DIC_SET( nDIC_TJA_ICA_On, nDic_Inactive);
    DIC_SET(nDIC_TJA_ICA_FailActive, nDic_Inactive);
    DIC_SET( nDIC_TJA_ICA_Check, nDic_Inactive);

    DIC_SET( nDIC_TJA_ICA_IsExitReminder, nDic_Inactive);
    DIC_SET( nDIC_TJA_ICA_Demand, nDic_Inactive);
    BUZ_SET(nBUZ_TJAICA_isexitreminder, nBuz_Inactive);
	BUZ_SET( nBUZ_TJA_ICA_Stop, nBuz_Inactive);
	BUZ_SET(nBUZ_TJA_ICA_On , nBuz_Inactive);
	BUZ_SET(nBUZ_TJA_ICA_FailActive , nBuz_Inactive);
	BUZ_SET(nBUZ_TJA_ICA_Demand , nBuz_Inactive); 			   
}
/*******************************************************************************************
* Function: FISD_vESCLDetect
* Description: Show
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vClearESCLSts(void);
static void FISD_vESCLDetect(void)
{
    /*clear falg*/
    FISD_vClearESCLSts();
	if((TRUE == Rte_FISD_boAnimationPlayIsFinish()) || (boSystemOOMPartOnMode() == TRUE))
	{
	    /*Redetect */
	    /*work in D2,D3 mode*/
		if(1u == FISD_u16SignalValue[nESCLCriticalFailure])
		{
			IND_SET(nIND_ESCLSerFault,nInd_On);
			DIC_SET( nDIC_ESCLCritiFailure, nDic_Active);
			BUZ_SET(nBUZ_ESCLSerFault,nBuz_Active);
		}
		else if(1u == FISD_u16SignalValue[nESCLFunctionFailure])
		{
			IND_SET(nIND_ESCLFault,nInd_On);		
			BUZ_SET(nBUZ_ESCLFault,nBuz_Active);
		}
		
		if(1u == FISD_u16SignalValue[nESCLUnlockJamming])
		{
			DIC_SET(nDIC_ESCLUnlocklamming, nDic_Active);
			BUZ_SET(nBUZ_ESCLUnlocklamming, nBuz_Active);
		}

		
		if(1u == FISD_u16SignalValue[nESCLFunctionFailure])
		{
			DIC_SET( nDIC_ESCLFuncFailure, nDic_Active);
		}


	#ifdef ENABLE_TEXT_SOUND_MIN_SHOW_2S
	    /*deal with the sound warning connect to textwarn which is cancelled in 2s.*/
	    if(nDIC_ESCLCritiFailure == TextMgr_enGetDisplayText() && TRUE == Rte_FISD_boAnimationPlayIsFinish())
	    {
	        BUZ_SET(nBUZ_ESCLSerFault,nBuz_Active);
	    }
	    else if(nDIC_ESCLFuncFailure == TextMgr_enGetDisplayText() && TRUE == Rte_FISD_boAnimationPlayIsFinish())
	    {
	        BUZ_SET(nBUZ_ESCLFault,nBuz_Active);
	    }
	    else if(nDIC_ESCLUnlocklamming == TextMgr_enGetDisplayText())
	    {
	       // BUZ_SET(nBUZ_ESCLUnlocklamming,nBuz_Active);
	    }
	    else
	    {
	    }
	#endif
	}
}

static void FISD_vClearESCLSts(void)
{
    IND_SET(nIND_ESCLFault,nInd_Off);
    IND_SET(nIND_ESCLUnlock,nInd_Off);
    IND_SET(nIND_ESCLSerFault,nInd_Off);
    DIC_SET( nDIC_ESCLCritiFailure, nDic_Inactive);
    DIC_SET( nDIC_ESCLFuncFailure, nDic_Inactive);
    DIC_SET( nDIC_ESCLUnlocklamming, nDic_Inactive);

    BUZ_SET(nBUZ_ESCLSerFault,nBuz_Inactive);
    BUZ_SET(nBUZ_ESCLFault,nBuz_Inactive);
    BUZ_SET(nBUZ_ESCLUnlocklamming,nBuz_Inactive);
    BUZ_SET(nBUZ_ESCLNotLocked,nBuz_Inactive);
}

/*******************************************************************************************
* Function: FISD_vAPADetects
* Description: Show
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vAPADetect(void)
{
    DIC_SET(nDIC_APAFailureSts,nDic_Inactive);
    DIC_SET(nDIC_APANotDisplay,nDic_Inactive);
    BUZ_SET(nBUZ_APANotDisplay,nBuz_Inactive);

    if(TRUE == Rte_FISD_boAnimationPlayIsFinish())
    {
        if(1u == FISD_u16SignalValue[nAPAFailureSts])
        {
            DIC_SET(nDIC_APAFailureSts,nDic_Active);
        }
        if(1u == FISD_u16SignalValue[nReasonForDVDnotDisplyAPA])
        {
            DIC_SET(nDIC_APANotDisplay,nDic_Active);
            BUZ_SET(nBUZ_APANotDisplay,nBuz_Active);
        }

#ifdef ENABLE_TEXT_SOUND_MIN_SHOW_2S
        /*deal with the sound warning connect to textwarn which is cancelled in 2s.*/
        if(nDIC_APANotDisplay == TextMgr_enGetDisplayText())
        {
            BUZ_SET(nBUZ_APANotDisplay,nBuz_Active);
        }
#endif
    }
}
/*******************************************************************************************
* Function: FISD_vElecShiftFaultDetect
* Description: Show
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vElecShiftFaultDetect(void)
{
	static U16 u16GearShiftFailCount = 0;
    DIC_SET( nDIC_GearShiftFailToStop, nDic_Inactive);
	DIC_SET(nDIC_GearShiftFailToSlow, nDic_Inactive);
    DIC_SET( nDIC_GearShiftPosition, nDic_Inactive);
    BUZ_SET(nBUZ_GearShiftFailToStop,nBuz_Inactive);
	BUZ_SET(nBUZ_GearShiftFailToSlow, nBuz_Inactive);
	BUZ_SET(nBUZ_GearShiftPosition, nBuz_Inactive);

	DIC_SET(nDIC_PleaseShiftTheGearLeverToTheRight, nDic_Inactive);
    DIC_SET(nDIC_GearLeverFailurePleaseRepairInTime, nDic_Inactive);
    BUZ_SET(nBUZ_GearLeverFailurePleaseRepairInTime,nBuz_Inactive);
	BUZ_SET(nBUZ_PleaseShiftTheGearLeverToTheRight, nBuz_Inactive);

	//0224 电子换挡器报警增加超时判断
    if(TRUE == Rte_FISD_boAnimationPlayIsFinish() && \
		(FALSE ==Rte_boGetAllWarningInfoTimeout()))
    {
        if(1u == FISD_u16SignalValue[nAllWarningInfo])
        {
            DIC_SET(nDIC_GearShiftFailToStop, nDic_Active);
			DIC_SET(nDIC_GearLeverFailurePleaseRepairInTime, nDic_Active);
			if(u16GearShiftFailCount < TIMECOUNT_5s)
			{
            	BUZ_SET(nBUZ_GearShiftFailToStop,nBuz_Active);
                BUZ_SET(nBUZ_GearLeverFailurePleaseRepairInTime,nBuz_Active);
				if((SoundMgr_enGetCuerentSoundId() == nBUZ_GearShiftFailToStop)
                    ||(SoundMgr_enGetCuerentSoundId() == nBUZ_GearLeverFailurePleaseRepairInTime))
				{
					u16GearShiftFailCount++;
				}
				else
				{
					u16GearShiftFailCount = 0;
				}
			}

        }
        else
        {
			u16GearShiftFailCount = 0;
		}
		if(2u == FISD_u16SignalValue[nAllWarningInfo])
        {
            DIC_SET(nDIC_GearShiftFailToSlow, nDic_Active);
			BUZ_SET(nBUZ_GearShiftFailToSlow, nBuz_Active);
        }
        else if(4u == FISD_u16SignalValue[nAllWarningInfo])
        {
            DIC_SET(nDIC_GearShiftPosition, nDic_Active);
			BUZ_SET(nBUZ_GearShiftPosition, nBuz_Active);
			DIC_SET(nDIC_PleaseShiftTheGearLeverToTheRight, nDic_Active);
			BUZ_SET(nBUZ_PleaseShiftTheGearLeverToTheRight, nBuz_Active);
        }
        else
        {
        }

#ifdef ENABLE_TEXT_SOUND_MIN_SHOW_2S
        /*deal with the sound warning connect to textwarn which is cancelled in 2s.*/
        if(nDIC_GearShiftFailToStop == TextMgr_enGetDisplayText())
        {
            BUZ_SET(nBUZ_GearShiftFailToStop,nBuz_Active);
        }
#endif
    }
	else
	{
		u16GearShiftFailCount = 0;
	}
}

/*******************************************************************************************
* Function: FISD_vWarnIconDetect
* Description: Show
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vWarnIconDetect(void)
{
	//U16 u16TextBrowseNum = 0;
    //u16TextBrowseNum = TextMgr_u32GetBrowseTotalNum();

    IND_SET(nIND_WarningIcon,nInd_Off);

    

    if(TextMgr_u32GetBrowseTotalNum() > 0u)
    {
        IND_SET(nIND_WarningIcon,nInd_On);
    }
}

/*******************************************************************************************
* Function: FISD_vLanguageSetDetect
* Description: Show
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vLanguageSetDetect(void)
{
#ifdef G6SH_TBD
    if(2u == FISD_u16SignalValue[nLanguageSet])
    {
        Menu_vSetLanguageFromHmi(1u); /*English*/
    }
    else if(0u != FISD_u16SignalValue[nLanguageSet])
    {
        Menu_vSetLanguageFromHmi(0u); /*Chinese*/
    }
#endif /* G6SH_TBD */
}

/*******************************************************************************************
* Function: FISD_vLdwLkaSwitchDetect
* Description: Show
* Parameters: none
* Return: none
********************************************************************************************/
volatile BOOL boLdwLkaTextShowFinish = FALSE;
volatile U16 u16DicLdwLkaRestShowTime = 0;
#define DIC_LDW_LKA_SHOWTIME_MAX       300u
static BOOL FISD_boLdwLkaSwitchIsPressed(void);

static void  FISD_vLdwLkaSwitchDetect(void)
{
#define LANE_DEFAULT      0
#define LANE_LDW          1
#define LANE_LKA          2
#define LANE_OFF          3
#define LDW_LKA_SWITCH_DELAY_TIME_CNT  330
#define LDW_LKA_SWITCH_KEEP_TIME_CNT   300

#define LDWLKA_TIPS_ONLY_USE_FEEDBACK                      0

    uint8 last_setting_to_ldw_lka = LANE_DEFAULT;
    uint8 ldw_lka_setting_destination = LANE_DEFAULT;
    static BOOL boLdwLkaPressed = FALSE; /*judge the key of LdwLka has been pressed down*/
    U16 u8LdwLkaFeedBackVal_Pre = FISD_u16SignalValue[nReserved_LDWLKA_LaneAssitTypefeedback];

    static uint8 aipm_ldw_lka_switch_val = 0;
    uint8 aipm_ldw_lka_switch = IlGetRxAIPM_LDWLKA_LaneAssitTypeReq();
    U8 ldw_config = Rte_FISD_u8GetCONFIG_LDW();
    U8 lka_config = Rte_FISD_u8GetCONFIG_LKA();
	U16 tenTextWarnId = TextMgr_enGetDisplayText();

    static U16 ldw_lka_switch_delay_cnt = 0;

#if(LDWLKA_TIPS_ONLY_USE_FEEDBACK)
    U16 ldwlka_feedbackval = FISD_u16SignalValue[nReserved_LDWLKA_LaneAssitTypefeedback];
    static U16 ldwlka_feedbackval_previous = 0;;
#endif

    CfgMgr_u8ConfigValueGet(CONFIG_LDWLKA_OUTPUT_STATUS, &last_setting_to_ldw_lka);
    DIC_SET(nDIC_LDW_LKA_Switch_Not_Use,nDic_Inactive);
    if(TRUE == Rte_FISD_boAnimationPlayIsFinish())
    {
        /*PIN14 is low*/
        if(0) //(TRUE == FISD_boLdwLkaSwitchIsPressed()), do not detect LDW/LKA hardware switch
        {
            boLdwLkaPressed = TRUE;
            IND_SET(nIND_LDW_LKA_Switch,nInd_On);

            // User delay counter to trigger LDW/LKA switch
            //DIC_SET(nDIC_LDW_LKA_Switch_Not_Use,nDic_Active);
            ldw_lka_switch_delay_cnt = LDW_LKA_SWITCH_DELAY_TIME_CNT;

            u16DicLdwLkaRestShowTime = DIC_LDW_LKA_SHOWTIME_MAX;
            boLdwLkaTextShowFinish = FALSE;
        }
        else
        {
            if(1)
            {
                if(aipm_ldw_lka_switch_val!=aipm_ldw_lka_switch&&aipm_ldw_lka_switch!=0)
                {
                    CfgMgr_u8ConfigValueSet(CONFIG_LDWLKA_OUTPUT_STATUS, aipm_ldw_lka_switch);

#if(LDWLKA_TIPS_ONLY_USE_FEEDBACK==0)
                    if(lka_config)
                    {
                        // User delay counter to trigger LDW/LKA switch
                        //DIC_SET(nDIC_LDW_LKA_Switch_Not_Use,nDic_Active);
                        if(tenTextWarnId==nDIC_LDW_LKA_Switch_Not_Use)
                        {
                            ldw_lka_switch_delay_cnt = LDW_LKA_SWITCH_KEEP_TIME_CNT;
                        }
						else
                        {
                            ldw_lka_switch_delay_cnt = LDW_LKA_SWITCH_DELAY_TIME_CNT;
                        }
                    }
                    else if(ldw_config)
                    {
                        if((aipm_ldw_lka_switch==1)||aipm_ldw_lka_switch==3)
                        {
                            // User delay counter to trigger LDW/LKA switch
                            //DIC_SET(nDIC_LDW_LKA_Switch_Not_Use,nDic_Active);
							if(tenTextWarnId==nDIC_LDW_LKA_Switch_Not_Use)
							{
								ldw_lka_switch_delay_cnt = LDW_LKA_SWITCH_KEEP_TIME_CNT;
							}
							else
							{
								ldw_lka_switch_delay_cnt = LDW_LKA_SWITCH_DELAY_TIME_CNT;
							}
                        }
                    }
                    else
                    {
                        //do not display
                    }
                }
#endif
                aipm_ldw_lka_switch_val = aipm_ldw_lka_switch;
            }

#if(LDWLKA_TIPS_ONLY_USE_FEEDBACK)
            if(((ldwlka_feedbackval==1)||(ldwlka_feedbackval==2))&&\
                    (ldwlka_feedbackval_previous!=ldwlka_feedbackval))
            {
                // ldw/lka entry, should show tips info
                ldw_lka_switch_delay_cnt = LDW_LKA_SWITCH_DELAY_TIME_CNT;
            }
            else if(((ldwlka_feedbackval==0)||(ldwlka_feedbackval==3))&&\
                    ((ldwlka_feedbackval_previous==1)||(ldwlka_feedbackval_previous==2)))
            {
                // ldw/lka exit, should show tips info
                ldw_lka_switch_delay_cnt = LDW_LKA_SWITCH_DELAY_TIME_CNT;
            }
            else
            {
                // nothing to do in this case
            }
#endif
        }
        if(ldw_lka_switch_delay_cnt)
        {
            ldw_lka_switch_delay_cnt--;
            if(ldw_lka_switch_delay_cnt<=LDW_LKA_SWITCH_KEEP_TIME_CNT&&ldw_lka_switch_delay_cnt>0)
            {
                DIC_SET(nDIC_LDW_LKA_Switch_Not_Use,nDic_Active);
            }
        }

        if(boLdwLkaPressed == TRUE)
        {
            boLdwLkaPressed = FALSE;

            if((0u == u8LdwLkaFeedBackVal_Pre) || (3u == u8LdwLkaFeedBackVal_Pre))
            {
                switch(last_setting_to_ldw_lka)
                {
                case LANE_OFF:
                    ldw_lka_setting_destination = LANE_LDW;
                    break;
                case LANE_LDW:
                    ldw_lka_setting_destination = LANE_LKA;
                    break;
                case LANE_LKA:
                    ldw_lka_setting_destination = LANE_OFF;
                    break;
                default:
                    ldw_lka_setting_destination = LANE_OFF;
                    break;
                }
                CfgMgr_u8ConfigValueSet(CONFIG_LDWLKA_OUTPUT_STATUS, ldw_lka_setting_destination);
            }
            else if(1u == u8LdwLkaFeedBackVal_Pre)
            {
                switch(last_setting_to_ldw_lka)
                {
                case LANE_LDW:
                    ldw_lka_setting_destination = LANE_OFF;
                    break;
                case LANE_OFF:
                    ldw_lka_setting_destination = LANE_LKA;
                    break;
                case LANE_LKA:
                    ldw_lka_setting_destination = LANE_LDW;
                    break;
                default:
                    ldw_lka_setting_destination = LANE_OFF;
                    break;
                }
                CfgMgr_u8ConfigValueSet(CONFIG_LDWLKA_OUTPUT_STATUS, ldw_lka_setting_destination);
            }
            else if(2u == u8LdwLkaFeedBackVal_Pre)
            {
                switch(last_setting_to_ldw_lka)
                {
                case LANE_LKA:
                    ldw_lka_setting_destination = LANE_OFF;
                    break;
                case LANE_OFF:
                    ldw_lka_setting_destination = LANE_LDW;
                    break;
                case LANE_LDW:
                    ldw_lka_setting_destination = LANE_LKA;
                    break;
                default:
                    ldw_lka_setting_destination = LANE_OFF;
                    break;
                }
                CfgMgr_u8ConfigValueSet(CONFIG_LDWLKA_OUTPUT_STATUS, ldw_lka_setting_destination);
            }
            else
            {
            }
        }


        if(0)//(EM_MENU_SET != Menu_u8TransmitHmiPageValue())
        {
            if(u16DicLdwLkaRestShowTime > 0)
            {
                u16DicLdwLkaRestShowTime --;
                boLdwLkaTextShowFinish = FALSE;
                IND_SET(nIND_LDW_LKA_Switch,nInd_On);
            }
            else
            {
                boLdwLkaTextShowFinish = TRUE;
                IND_SET(nIND_LDW_LKA_Switch,nInd_Off);
            }
        }
        else
        {
            boLdwLkaTextShowFinish = FALSE;
            IND_SET(nIND_LDW_LKA_Switch,nInd_Off);
            u16DicLdwLkaRestShowTime = 0;
        }
    }
    else
    {
        boLdwLkaTextShowFinish = FALSE;
        IND_SET(nIND_LDW_LKA_Switch,nInd_Off);
        u16DicLdwLkaRestShowTime = 0;
    }

#if(LDWLKA_TIPS_ONLY_USE_FEEDBACK)
    ldwlka_feedbackval_previous = ldwlka_feedbackval;
#endif

#undef LANE_DEFAULT
#undef LANE_LDW
#undef LANE_LKA
#undef LANE_OFF
}

static BOOL FISD_boLdwLkaSwitchIsPressed(void)
{
    static U16 u16LdwLkaPressTime = 0;
    BOOL enLdwLkaSts = Rte_FISD_enGetDio_LawLda_Status();
    BOOL boRteVal = FALSE;

    if(enLdwLkaSts == FALSE)/*Press*/
    {
        if(u16LdwLkaPressTime < TIMECOUNT_5s)
        {
            u16LdwLkaPressTime ++;
        }
    }
    else /*Not press or release*/
    {
        if(u16LdwLkaPressTime > TIMECOUNT_100ms)
        {
            boRteVal = TRUE;
        }
        u16LdwLkaPressTime = 0;
    }

    return boRteVal;
}

#undef DIC_LDW_LKA_SHOWTIME_MAX
/*******************************************************************************************
* Function: FISD_vWarnIconDetect
* Description: Show
* Parameters: none
* Return: none
********************************************************************************************/
#define FISD_SeatMirror_NoActive 0u
#define FISD_SeatMirror_Success  1u
#define FISD_SeatMirror_Fail  2u
#define FISD_RememberKeepSetPeriod 100u
static void FISD_vSeatRearmirrorDedect(void)
{
	U8 sys_fullon = Rte_FISD_boGetPowerDependent();
	U8 sys_parton = boSystemOOMPartOnMode();
	static U8 sys_power_status;
    static U16 su16SeatRearTextShowTime = 0u;
	static U16 su16DisplayText_pre = nDIC_Invalid;
    U8 u8SeatExtMirror_Cur = FISD_u16SignalValue[nSeat_ExtMirror_MemorySts];
    static U16 RememberKeepSetTimer = 0;
	static U16 RememberKeepSetTimer_failure = 0;
	BUZ_SET(nBUZ_RememberKeepSet,nBuz_Inactive);
	BUZ_SET(nBUZ_RememberKeepSet_failure,nBuz_Inactive);

	
	if(FALSE == sboSeatRearMirrorInit)
	{
		sboSeatRearMirrorInit = TRUE;
		DIC_SET(nDIC_RememberSetSuccess,nDic_Inactive);
		DIC_SET(nDIC_RememberSetFailure,nDic_Inactive);
	}


    if((TRUE == Rte_FISD_boAnimationPlayIsFinish() || sys_parton == TRUE) && (SYN_SOC_WorkingStateGet() >= SOC_WORKING_ANIMATION_ENDED))
    {
        if((u8SeatExtMirror_Cur != u8SeatExtMirror_Pre) && (FISD_SeatMirror_Success == u8SeatExtMirror_Cur))
        {
            DIC_SET(nDIC_RememberSetSuccess,nDic_Active);
			DIC_SET(nDIC_RememberSetFailure,nDic_Inactive);
            su16SeatRearTextShowTime = TIMECOUNT_3s;
            if(nDIC_RememberSetSuccess == TextMgr_enGetDisplayText())
            {
                FISD_aboTextShowTimeResetFlag[nDIC_RememberSetSuccess] = TRUE;/*延长弹窗显示时间*/
            }
            RememberKeepSetTimer = FISD_RememberKeepSetPeriod;
        }
        else if((u8SeatExtMirror_Cur != u8SeatExtMirror_Pre) && (FISD_SeatMirror_Fail == u8SeatExtMirror_Cur))
        {
            DIC_SET(nDIC_RememberSetFailure,nDic_Active);
			DIC_SET(nDIC_RememberSetSuccess,nDic_Inactive);
            su16SeatRearTextShowTime = TIMECOUNT_3s;
            if(nDIC_RememberSetFailure == TextMgr_enGetDisplayText())
            {
                FISD_aboTextShowTimeResetFlag[nDIC_RememberSetFailure] = TRUE;
            }
            RememberKeepSetTimer_failure = FISD_RememberKeepSetPeriod;
        }
        else
        {
            if((TRUE == FISD_aboTextShowTimeResetFlag[nDIC_RememberSetSuccess]) || \
                    (TRUE == FISD_aboTextShowTimeResetFlag[nDIC_RememberSetFailure]))
            {
                FISD_aboTextShowTimeResetFlag[nDIC_RememberSetSuccess] = FALSE;
                FISD_aboTextShowTimeResetFlag[nDIC_RememberSetFailure] = FALSE;
            }
					
			if((TextMgr_enGetDisplayText() == nDIC_RememberSetSuccess) ||\
				(TextMgr_enGetDisplayText() == nDIC_RememberSetFailure))
			{
			
	            if(su16SeatRearTextShowTime > 0u)
	            {
	                su16SeatRearTextShowTime --;
	            }
	            else
	            {
	                sboSeatRearMirrorInit = FALSE;
					su16SeatRearTextShowTime = 0u;
	            }
			}
			else if(su16SeatRearTextShowTime > 0u)
			{
				su16SeatRearTextShowTime = TIMECOUNT_3s;
			}
			
        }

        if(RememberKeepSetTimer>0)
        {
            RememberKeepSetTimer--;
            BUZ_SET(nBUZ_RememberKeepSet,nBuz_Active);
        }
        else
        {
            BUZ_SET(nBUZ_RememberKeepSet,nBuz_Inactive);
        }
		if(RememberKeepSetTimer_failure>0)
        {
            RememberKeepSetTimer_failure--;
            BUZ_SET(nBUZ_RememberKeepSet_failure,nBuz_Active);
        }
        else
        {
            BUZ_SET(nBUZ_RememberKeepSet_failure,nBuz_Inactive);
        }
		if(sys_power_status != sys_fullon)
		{
			if(FISD_u16SignalValue[nSeat_ExtMirror_MemorySts] == 1)
			{
				DIC_SET(nDIC_RememberSetSuccess,nDic_Active);
				DIC_SET(nDIC_RememberSetFailure,nDic_Inactive);
				su16SeatRearTextShowTime = TIMECOUNT_3s;
			}
			else if(FISD_u16SignalValue[nSeat_ExtMirror_MemorySts] == 2)
			{
				DIC_SET(nDIC_RememberSetFailure,nDic_Active);
				DIC_SET(nDIC_RememberSetSuccess,nDic_Inactive);
				su16SeatRearTextShowTime = TIMECOUNT_3s;
			}
		}

		if(FISD_u16SignalValue[nSeat_ExtMirror_MemorySts] == 1)
		{
			BUZ_SET(nBUZ_RememberKeepSet,nBuz_Active);
		}
		else if(FISD_u16SignalValue[nSeat_ExtMirror_MemorySts] == 2)
		{
			BUZ_SET(nBUZ_RememberKeepSet_failure,nBuz_Active);
		}
		u8SeatExtMirror_Pre = u8SeatExtMirror_Cur;
    }
    else
    {
        DIC_SET(nDIC_RememberSetSuccess,nDic_Inactive);
        DIC_SET(nDIC_RememberSetFailure,nDic_Inactive);
        BUZ_SET(nBUZ_RememberKeepSet,nBuz_Inactive);
		BUZ_SET(nBUZ_RememberKeepSet_failure,nBuz_Inactive);
    }
	
	if(sys_fullon == FALSE)
	{

		//BUZ_SET(nBUZ_RememberKeepSet,nBuz_Inactive);
	}
	//#endif
	
    
	sys_power_status = sys_fullon;
}

#undef FISD_SeatMirror_NoActive
#undef FISD_SeatMirror_Success
#undef FISD_SeatMirror_Fail


#define FISD_MemoryKey_NoActive 0u
#define FISD_MemoryKey_Active    1u
static void FISD_vMemoryKeyStsDetect(void)
{
    static U8 u8MemoryKeySts_Pre = FISD_MemoryKey_NoActive;
    static BOOL sboMemoryKeyStsInit = FALSE;
    static U16 su16MemoryKeyPlayTime = 0u;
    U8 u8MemoryKeySts_Cur = (U8)FISD_u16SignalValue[nMemoryKeySts];

    if(FALSE == sboMemoryKeyStsInit)
    {
        sboMemoryKeyStsInit = TRUE;
        BUZ_SET(nBUZ_MenorySetKey,nBuz_Inactive);
    }

    if(TRUE != Rte_FISD_boIsStayInAnimationPlayStatus())
    {
        if(u8MemoryKeySts_Pre == FISD_MemoryKey_NoActive && u8MemoryKeySts_Cur == FISD_MemoryKey_Active)
        {
            BUZ_SET(nBUZ_MenorySetKey,nBuz_Active);
            su16MemoryKeyPlayTime =  TIMECOUNT_2s;
        }
        else
        {
            if(su16MemoryKeyPlayTime > 0)
            {
                su16MemoryKeyPlayTime --;
            }
            else
            {
                sboMemoryKeyStsInit = FALSE;
            }
        }
    }
    else
    {
        BUZ_SET(nBUZ_MenorySetKey,nBuz_Inactive);
    }
    u8MemoryKeySts_Pre = u8MemoryKeySts_Cur;
}

#undef FISD_MemoryKey_NoActive
#undef FISD_MemoryKey_Active

/*******************************************************************************************
* Function: FISD_vChildLockDedect
* Description: Show
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vClearChildLockStatus(void);
static void FISD_vChildLockDedect(void)
{
    U16 u16ChildSafetySts = FISD_u16SignalValue[nChildSafetySts];
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
    static U16 u16Timer = 0;
    static BOOL last_sys_fullon;
    /*添加首次上电1s延时判断*/
	if(sys_fullon == TRUE && last_sys_fullon == FALSE)
	{
		u16Timer = TIMECOUNT_1s;
	}

    if(u16Timer>0)
    {
        u16Timer--;
    }

    FISD_vClearChildLockStatus();
    if(TRUE == Rte_FISD_boAnimationPlayIsFinish() && u16Timer==0)
    {
        switch(u16ChildSafetySts)
        {
        case 1:
            DIC_SET(nDIC_ChildSaftyUnlock, nDic_Active);
           // BUZ_SET(nBUZ_ChildSaftyUnlock, nBuz_Active);
            break;
        case 2:
            DIC_SET(nDIC_ChildSaftyLock, nDic_Active);
           // BUZ_SET(nBUZ_ChildSaftyLock, nBuz_Active);
            break;
        case 3:
            DIC_SET(nDIC_ChildSaftyFault, nDic_Active);
            BUZ_SET(nBUZ_ChildSaftyFault, nBuz_Active);
            break;
        default:
            break;
        }

#ifdef ENABLE_TEXT_SOUND_MIN_SHOW_2S
        /*deal with the sound warning connect to textwarn which is cancelled in 2s.*/
        if(nDIC_ChildSaftyUnlock == TextMgr_enGetDisplayText())
        {
          //  BUZ_SET(nBUZ_ChildSaftyUnlock,nBuz_Active);
        }
        else if(nDIC_ChildSaftyLock == TextMgr_enGetDisplayText())
        {
          //  BUZ_SET(nBUZ_ChildSaftyLock,nBuz_Active);
        }
        else if(nDIC_ChildSaftyFault == TextMgr_enGetDisplayText())
        {
            BUZ_SET(nBUZ_ChildSaftyFault,nBuz_Active);
        }
        else
        {
        }
#endif
    }
    else
    {

    }
    last_sys_fullon = sys_fullon;
}
static void FISD_vClearChildLockStatus(void)
{
    DIC_SET(nDIC_ChildSaftyLock, nDic_Inactive);
    DIC_SET(nDIC_ChildSaftyUnlock, nDic_Inactive);
    DIC_SET(nDIC_ChildSaftyFault, nDic_Inactive);

  //  BUZ_SET(nBUZ_ChildSaftyLock, nBuz_Inactive);
  //  BUZ_SET(nBUZ_ChildSaftyUnlock, nBuz_Inactive);
    BUZ_SET(nBUZ_ChildSaftyFault, nBuz_Inactive);
}

/*******************************************************************************************
* Function: FISD_vRemoteStartModeDetect
* Description: Show
* Parameters: none
* Return: none
********************************************************************************************/

static void FISD_vRemoteStartModeDetect(void)
{
    DIC_SET(nDIC_RemoteModeActive_AT, nDic_Inactive);
    DIC_SET(nDIC_RemoteModeActive_MT, nDic_Inactive);

	if(can_diag_get_sw_conf(CONF_IDX_One_ClickStartSwitch) == nOneClickStartSwitch_Not_Present)
	{
		if((FISD_u16SignalValue[nRVSMode] == 1u) && (Rte_FISD_GetPowerState() == nPowerState_D2))
        {
            DIC_SET(nDIC_RemoteModeActive_AT, nDic_Active);
            DIC_SET(nDIC_RemoteModeActive_MT, nDic_Active);
        }
	}
	else
	{
	    if(TRUE == Rte_FISD_boAnimationPlayIsFinish() &&
	            (FALSE == FISD_boInstrumentIsInSelfCheckSts()))
	    {
	        if(FISD_u16SignalValue[nRVSMode] == 1u)
	        {
	            DIC_SET(nDIC_RemoteModeActive_AT, nDic_Active);
	            DIC_SET(nDIC_RemoteModeActive_MT, nDic_Active);
	        }
	    }
	}
}

/*******************************************************************************************
* Function: FISD_vCarLampFaultDetect
* Description: Show
* Parameters: none
* Return: none
********************************************************************************************/
static void FISD_vCarLampFaultDetect(void)
{
    U8 sys_fullon = Rte_FISD_boGetPowerDependent();
	U8 sys_parton = boSystemOOMPartOnMode();

    DIC_SET(nDIC_BrakeLampFault, nDic_Inactive);
    DIC_SET(nDIC_ReverseLampFault, nDic_Inactive);
    DIC_SET(nDIC_RearFogFault, nDic_Inactive);

    BUZ_SET(nBUZ_BrakeLampFault, nBuz_Inactive);
    BUZ_SET(nBUZ_ReverseLampFault, nBuz_Inactive);
    BUZ_SET(nBUZ_RearFogFault, nBuz_Inactive);

    if((sys_parton == TRUE) || (TRUE == Rte_FISD_boAnimationPlayIsFinish()))
    {
        if(1 == FISD_u16SignalValue[nBrakelampFaultSts])
        {
            DIC_SET(nDIC_BrakeLampFault, nDic_Active);
            BUZ_SET(nBUZ_BrakeLampFault, nBuz_Active);
        }

        if(1 == FISD_u16SignalValue[nReversinglampFaultSts])
        {
            DIC_SET(nDIC_ReverseLampFault, nDic_Active);
            BUZ_SET(nBUZ_ReverseLampFault, nBuz_Active);
        }

        if(1 == FISD_u16SignalValue[nRearFogFaultSts])
        {
            DIC_SET(nDIC_RearFogFault, nDic_Active);
            BUZ_SET(nBUZ_RearFogFault, nBuz_Active);
        }
    }
}

/*******************************************************************************************
* Function: FISD_vGpfFuncDetect
* Description: Show
* Parameters: none
* Return: none
********************************************************************************************/
static void	FISD_vGpfFuncDetect(void)
{
    U8 u8GpfWarn = FISD_u16SignalValue[nGPFWarning];
    IND_SET(nIND_GpfFull_Green, nInd_Off);
    IND_SET(nIND_GpfOverLimited_Yellow, nInd_Off);

    DIC_SET(nDIC_GpfParticleTrapFullPlsClear,nDic_Inactive);
    DIC_SET(nDIC_GpfParticleTrapOverPlsRepair,nDic_Inactive);

    BUZ_SET(nBUZ_GpfParticleTrapFullPlsClear, nBuz_Inactive);
    BUZ_SET(nBUZ_GpfParticleTrapOverPlsRepair, nBuz_Inactive);

    if(TRUE == Rte_FISD_boAnimationPlayIsFinish())
    {
        if(1u == u8GpfWarn)
        {
            IND_SET(nIND_GpfFull_Green, nInd_On);
            DIC_SET(nDIC_GpfParticleTrapFullPlsClear,nDic_Active);
            BUZ_SET(nBUZ_GpfParticleTrapFullPlsClear, nBuz_Active);
        }
        else if(2u == u8GpfWarn)
        {
            IND_SET(nIND_GpfOverLimited_Yellow, nInd_On);
            DIC_SET(nDIC_GpfParticleTrapOverPlsRepair,nDic_Active);
            BUZ_SET(nBUZ_GpfParticleTrapOverPlsRepair, nBuz_Active);
        }
    }

#ifdef ENABLE_TEXT_SOUND_MIN_SHOW_2S
    /*deal with the sound warning connect to textwarn which is cancelled in 2s.*/
    if(nDIC_GpfParticleTrapFullPlsClear == TextMgr_enGetDisplayText())
    {
        //BUZ_SET(nBUZ_GpfParticleTrapFullPlsClear,nBuz_Active);
    }
    else if(nDIC_GpfParticleTrapOverPlsRepair== TextMgr_enGetDisplayText())
    {
        //BUZ_SET(nBUZ_GpfParticleTrapOverPlsRepair,nBuz_Active);
    }
    else
    {
    }
#endif
}

static void FISD_vBattEnergyWarningDetect(void)
{
    uint8 u8sig_afs_faultsts = FISD_u16SignalValue[nAFS_FaultSts];
    uint8 u8sig_battery_temp_sts = FISD_u16SignalValue[nBattery_Temp_Sts];

    IND_SET(nIND_AFS_Fault, nInd_Off);
    DIC_SET(nDIC_BattOverHot, nDic_Inactive);
    DIC_SET(nDIC_BattRelayOpen, nDic_Inactive);
    DIC_SET(nDIC_AFSFault, nDic_Inactive);

    if(TRUE == Rte_FISD_boAnimationPlayIsFinish())
    {
        if(u8sig_afs_faultsts&&(FALSE==Rte_boGetAFS_FaultStsTimeout()))
        {
            IND_SET(nIND_AFS_Fault, nInd_On);
            DIC_SET(nDIC_AFSFault, nDic_Active);
        }

        if((u8sig_battery_temp_sts==1)&&(FALSE==Rte_boGetBattery_Temp_StsTimeout()))
        {
            DIC_SET(nDIC_BattOverHot, nDic_Active);
            DIC_SET(nDIC_BattRelayOpen, nDic_Inactive);
        }
        else if((u8sig_battery_temp_sts==2)&&(FALSE==Rte_boGetBattery_Temp_StsTimeout()))
        {
            DIC_SET(nDIC_BattOverHot, nDic_Inactive);
            DIC_SET(nDIC_BattRelayOpen, nDic_Active);
        }
        else
        {
            DIC_SET(nDIC_BattOverHot, nDic_Inactive);
            DIC_SET(nDIC_BattRelayOpen, nDic_Inactive);
        }
    }

}

static void FISD_vTakeYourPhoneDetect(void)
{
    BOOL cwc_timeout = IlGetCWC_PhoneforgottenRxTimeout();
    BOOL cwc_forgotphone = IlGetRxCWC_Phoneforgotten();

    BUZ_SET(nBUZ_TakeYourPhone,nBuz_Inactive);

    if((cwc_timeout==FALSE)&&(cwc_forgotphone==1))
    {
        BUZ_SET(nBUZ_TakeYourPhone,nBuz_Active);
    }
}

static U8 camera_shot_trigger_timer = 0;
void FISD_vCameraShotTrigger(void)
{
    camera_shot_trigger_timer = 50;
}

static void FISD_vCameraShotDetect(void)
{
    BUZ_SET(nBUZ_CameraShot, nBuz_Inactive);
    if(camera_shot_trigger_timer>0)
    {
        camera_shot_trigger_timer--;
    }
    if(camera_shot_trigger_timer>0)
    {
        BUZ_SET(nBUZ_CameraShot, nBuz_Active);
    }
}

static void FISD_vAWD_FaultDetect(void)
{
	U8 u8Sig = FISD_u16SignalValue[nAWDoutserviceDispRq];
	BOOL btimeout = Rte_boGetAWDoutserviceDispRqTimeout();
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	
	BUZ_SET(nBUZ_AwdFault, nBuz_Inactive);
	IND_SET(nIND_AWD_Red, nInd_Off);
	IND_SET(nIND_AWD_Yellow, nInd_Off);
	DIC_SET(nDIC_AwdFault, nDic_Inactive);

	if((!btimeout) && sys_fullon)
	{
		if(1u == u8Sig)
		{
			IND_SET(nIND_AWD_Yellow, nInd_On);
		}
		else if(2u == u8Sig)
		{
			BUZ_SET(nBUZ_AwdFault, nBuz_Active);
			IND_SET(nIND_AWD_Red, nInd_On);
			DIC_SET(nDIC_AwdFault, nDic_Active);
		}
	}

	if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
	{
		BUZ_SET(nBUZ_AwdFault, nBuz_Inactive);
		IND_SET(nIND_AWD_Red, nInd_Off);
		IND_SET(nIND_AWD_Yellow, nInd_Off);
		DIC_SET(nDIC_AwdFault, nDic_Inactive);
	}
}

static float32 u8GetDispalyTemperature_C(void)
{
	U8 u8Temperature_F = FISD_u16SignalValue[nExternalTemperature_F];
	U8 u8Temperature_C = FISD_u16SignalValue[nExternalTemperature_C];
	U8 u8TemperatureFailSts = FISD_u16SignalValue[nExternalTemperatureFailSts];
	U8 u8TemperatureUnit = FISD_u16SignalValue[nTemperatureUnit];
	static float32 u8ActualTemperature_F = 0x6C;
	static float32 u8ActualTemperature_C = 0x46;

	if(u8TemperatureUnit == 0)
	{
		if((u8TemperatureFailSts == 0) && (u8Temperature_C <= 0XFA))
		{
			u8ActualTemperature_C = (float)((u8Temperature_C))/2-55;
		}
		else
		{
			u8ActualTemperature_C = 0x46;
		}
	}
	else if(u8TemperatureUnit == 1)
	{
		if((u8TemperatureFailSts == 0) && (u8Temperature_F <= 0XE1))
		{
			u8ActualTemperature_F = u8Temperature_F - 67;
			u8ActualTemperature_C = 5*((float)(u8ActualTemperature_F) - 32)/9;
		}
		else
		{
			u8ActualTemperature_C = 0x46;
		}
	}
	return u8ActualTemperature_C;
}

#define TEMPERATURE_DELAY_COUNT 50
static void FISD_vIceRoadDetect(void)
{
	static float32 u8DispalyTemperature_C = 0x46;
	static BOOL bMultipleTriggerFlag = FALSE;
	BOOL btimeout = Rte_boGetTemperatureUnitTimeout();
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	BOOL sys_parton = boSystemOOMPartOnMode();

	if(sys_fullon == FALSE)
	{
		bMultipleTriggerFlag = FALSE;
	}
	if((!btimeout) && sys_fullon &&(Rte_FISD_boAnimationPlayIsFinish()==TRUE) && \
		(TRUE == IlGetCLM_1_ExternalTemperature_CFirstvalue()))
	{
		u8DispalyTemperature_C = u8GetDispalyTemperature_C();
		if(u8DispalyTemperature_C <= 3)
		{
			IND_SET(nIND_IceRoad, nInd_On);
			if(bMultipleTriggerFlag == FALSE)
			{
				BUZ_SET(nBUZ_IceRoad, nBuz_Active);
				DIC_SET(nDIC_IceRoad, nDic_Active);
				bMultipleTriggerFlag = TRUE;
			}
		}
		else if(u8DispalyTemperature_C >= 5)
		{
			BUZ_SET(nBUZ_IceRoad, nBuz_Inactive);
			IND_SET(nIND_IceRoad, nInd_Off);
			DIC_SET(nDIC_IceRoad, nDic_Inactive);
		}
	}

	if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
	{
		BUZ_SET(nBUZ_IceRoad, nBuz_Inactive);
		IND_SET(nIND_IceRoad, nInd_Off);
		DIC_SET(nDIC_IceRoad, nDic_Inactive);
	}
	
	if(btimeout && sys_fullon)
	{
		BUZ_SET(nBUZ_IceRoad, nBuz_Inactive);
		IND_SET(nIND_IceRoad, nInd_Off);
		DIC_SET(nDIC_IceRoad, nBuz_Inactive);
	}
}

static void FISD_vWashingWaterDetect(void)
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	static U16 u16ActiveCnt = 0U;
	static U16 u16DeactiveCnt = 0U;

	if(TRUE == Rte_FISD_boAnimationPlayIsFinish())
	{
		if(FALSE == Rte_FISD_enGetWashingWaterSensor())
		{
			u16DeactiveCnt = 0U;
			
			if(u16ActiveCnt > WASHING_WATER_ALARM_ACTIVE_CNT)
			{
				IND_SET(nIND_WashingWater, nInd_On);
				DIC_SET(nDIC_WashingWater, nDic_Active);
				BUZ_SET(nBUZ_WashingWater, nBuz_Active);
			}
			else
			{
				u16ActiveCnt++;
			}
		}
		else
		{
			u16ActiveCnt = 0U;
			
			if(u16DeactiveCnt > WASHING_WATER_ALARM_DEACTIVE_CNT)
			{
				IND_SET(nIND_WashingWater, nInd_Off);
				DIC_SET(nDIC_WashingWater, nDic_Inactive);
				BUZ_SET(nBUZ_WashingWater, nBuz_Inactive);
			}
			else
			{
				u16DeactiveCnt++;
			}
		}
	}
	else
	{
	
		u16ActiveCnt = 0U;
		u16DeactiveCnt = 0U;
		IND_SET(nIND_WashingWater, nInd_Off);
		DIC_SET(nDIC_WashingWater, nDic_Inactive);
		BUZ_SET(nBUZ_WashingWater, nBuz_Inactive);
	}
}

BOOL FISD_vGetWashingWaterWarn(void)
{
	BOOL boWashingWaterWarn = FALSE;
	if(Rte_FISD_u8GetCONFIG_T18P() == 0)
	{
		if(IND_CHK(nIND_WashingWater, nInd_Off) == FALSE)
		{
			boWashingWaterWarn = TRUE;
		}
	}
	return boWashingWaterWarn;
}


static void FISD_vActiveSafetyBeltDetect(void)
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	BOOL btimeout = Rte_boGetPPMIStTimeout();
	U8 u8SafetyBeltFailureFlag = FISD_u16SignalValue[nPPMISt];
	U8 u8SafetyBeltExpireFlag = FISD_u16SignalValue[nPPMICounter];

	DIC_SET(nDIC_ActiveSeatBeltFailure, nDic_Inactive);
	DIC_SET(nDIC_ActiveSeatBeltsExpire, nDic_Inactive);
	
	if((!btimeout) && sys_fullon)
	{
		if(u8SafetyBeltFailureFlag == 0x02)
		{
			DIC_SET(nDIC_ActiveSeatBeltFailure, nDic_Active);
		}
		else
		{
			DIC_SET(nDIC_ActiveSeatBeltFailure, nDic_Inactive);
		}
		
		if(u8SafetyBeltExpireFlag == 0x01)
		{
			DIC_SET(nDIC_ActiveSeatBeltsExpire, nDic_Active);
		}
		else 
		{
			DIC_SET(nDIC_ActiveSeatBeltsExpire, nDic_Inactive);
		}
	}

	
	if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
	{
		DIC_SET(nDIC_ActiveSeatBeltFailure, nDic_Inactive);
		DIC_SET(nDIC_ActiveSeatBeltsExpire, nDic_Inactive);
	}
}


static void FISD_vHUDOverHotDetect(void)
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	BOOL btimeout = Rte_boGetHUD_1_HUDHiTWarnTimeout();
	U8 u8HUDOverHotFlag = FISD_u16SignalValue[nHUD_1_HUDHiTWarn];
	
	BUZ_SET(nBUZ_HUDOverHot, nBuz_Inactive);
	DIC_SET(nDIC_HUDOverHot, nDic_Inactive);

	if((!btimeout) && sys_fullon)
	{
		if(u8HUDOverHotFlag == 0x01)
		{
			BUZ_SET(nBUZ_HUDOverHot, nBuz_Active);
			DIC_SET(nDIC_HUDOverHot, nDic_Active);
		}
	}
	
	if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
	{
		BUZ_SET(nBUZ_HUDOverHot, nBuz_Inactive);
		DIC_SET(nDIC_HUDOverHot, nDic_Inactive);
	}
	
}


static void FISD_vClothesHookedDetect(void)
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	BOOL btimeout = Rte_boGetDriverDoorSIllStsTimeout();
	U8 u8ClothesHookedFlag = FISD_u16SignalValue[nDriverDoorSIllSts];
	
	DIC_SET(nDIC_ClothesHooked, nDic_Inactive);
	BUZ_SET(nBUZ_ClothesHooked, nBuz_Inactive);
	
	if(btimeout && sys_fullon)
	{
		DIC_SET(nDIC_ClothesHooked, nDic_Inactive);
		BUZ_SET(nBUZ_ClothesHooked, nBuz_Inactive);
	}


	if((!btimeout) && sys_fullon)
	{
		if(u8ClothesHookedFlag == 0x01)
		{
			DIC_SET(nDIC_ClothesHooked, nDic_Active);
			BUZ_SET(nBUZ_ClothesHooked, nBuz_Active);
		}
		else
		{
			DIC_SET(nDIC_ClothesHooked, nDic_Inactive);
		}
	}

	if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
	{
		DIC_SET(nDIC_ClothesHooked, nDic_Inactive);
		BUZ_SET(nBUZ_ClothesHooked, nBuz_Inactive);
	}
}

static void FISD_vElectricDoorOpenDetect(void)
{
    DIC_SET(nDIC_DoorOpenErr, nDic_Inactive);
    DIC_SET(nDIC_DrvrStall, nDic_Inactive);
    DIC_SET(nDIC_PassStall, nDic_Inactive);
    DIC_SET(nDIC_LHRStall, nDic_Inactive);
    DIC_SET(nDIC_RHRStall, nDic_Inactive);
	if((IlGetDCM_1_DrvrStallStsRxTimeout() == FALSE) &&\
	((TRUE == Rte_FISD_boAnimationPlayIsFinish()) || \
	 (boSystemOOMPartOnMode() == TRUE)))
	{
		if(1u == FISD_u16SignalValue[nPwrDoorErr])
		{
	    	DIC_SET(nDIC_DoorOpenErr, nDic_Active);
		}
		if(1u == FISD_u16SignalValue[nDrvrStallSts])
		{
	    	DIC_SET(nDIC_DrvrStall, nDic_Active);
		}
		if(1u == FISD_u16SignalValue[nPassStallSts])
		{
	    	DIC_SET(nDIC_PassStall, nDic_Active);
		}
		if(1u == FISD_u16SignalValue[nLHRStallSts])
		{
	    	DIC_SET(nDIC_LHRStall, nDic_Active);
		}
		if(1u == FISD_u16SignalValue[nRHRStallSts])
		{
	    	DIC_SET(nDIC_RHRStall, nDic_Active);
		}
	}
}

static void FISD_vCloseLightDetect()
{
    DIC_SET(nDIC_CloseLight, nDic_Inactive);
    BUZ_SET(nBUZ_CloseLight, nBuz_Inactive);
    
    if((False == IlGetBCM_4_DriverDoorStsRxTimeout()) && \
       (FISD_u16SignalValue[nParkLightSts] == 1) && \
       (FISD_u16SignalValue[nDriverDoorSts] == 1) && \
       (((FISD_enIgnState == FALSE) && \
       (nOFF == FISD_u16SignalValue[nKeySts]))||(nPowerState_D3 == Rte_FISD_GetPowerState())))
    {
        DIC_SET(nDIC_CloseLight, nDic_Active);
        BUZ_SET(nBUZ_CloseLight, nBuz_Active);
    }
}


#define SOC_10_UPPER_LIMIT 1049u
#define SOC_11_LOWER_LIMIT 1050u
#define SOC_15_LOWER_LIMIT 1450u


static void FISD_vPHEV_SOCDisp_Detect(void) //PHEV相关功能
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	BOOL sig_timeout = Rte_FISD_boGetSOCDispTimeout();
	U16 sigPercentage = Rte_FISD_GetSocDisplayVal();
    static BOOL lowWarn = FALSE;
    static BOOL HaveWarnedFlag = FALSE;
	
	IND_SET(nIND_BatLowAlarm_Yellow, nInd_Off);
	//DIC_SET(nDIC_BatPowerLow, nDic_Inactive);
	//BUZ_SET(nBUZ_BatPowerLow, nBuz_Inactive);
	IND_SET(nIND_BatLowAlarm_White, nInd_Off);
	IND_SET(nIND_SocDisRed, nInd_Off);

    if (nPowerState_D1 != Rte_FISD_GetPowerState())
    {
        HaveWarnedFlag = FALSE;
    }
    
	if(!sig_timeout)
    {
    	if(sigPercentage <= SOC_10_UPPER_LIMIT)         /*Low battery alarm: When the battery meter shows that the battery is<=10%*/
    	{
    		lowWarn = TRUE;
    	}
    	
    	if(lowWarn)
    	{
    		if((HaveWarnedFlag == FALSE) && (sys_fullon == TRUE))
            {
        		DIC_SET(nDIC_BatPowerLow, nDic_Active);
        		BUZ_SET(nBUZ_BatPowerLow, nBuz_Active);
                HaveWarnedFlag = TRUE;
            }
            if((((nPowerType_PHEV==can_diag_get_sw_conf(CONF_IDX_DrivingPowerType) 
                    && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2)
                    ||(nPowerType_EV==can_diag_get_sw_conf(CONF_IDX_DrivingPowerType) ))
                    &&(nPowerState_D1 == Rte_FISD_GetPowerState()))
                    ||((nPowerType_PHEV==can_diag_get_sw_conf(CONF_IDX_DrivingPowerType) 
                    && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3)
                    &&(nPowerState_D1 == Rte_FISD_GetPowerState() 
                    || nPowerState_D2 == Rte_FISD_GetPowerState())))
            {
                IND_SET(nIND_BatLowAlarm_Yellow, nInd_On);
            }
			IND_SET(nIND_SocDisRed, nInd_On);
    	}
    	else
    	{
    		IND_SET(nIND_BatLowAlarm_White, nInd_On);
    	}
    }
    else
    {
        DIC_SET(nDIC_BatPowerLow, nDic_Inactive);
        BUZ_SET(nBUZ_BatPowerLow, nBuz_Inactive);
        lowWarn = FALSE;
        IND_SET(nIND_BatLowAlarm_White, nInd_On);
        HaveWarnedFlag = FALSE;
    }
 
	if(lowWarn == TRUE)
    {
        /*After the low battery alarm is triggered, when the battery power in (10%, 15%), 
        the progress bar indicator will display red and the indicator light will display yellow*/
        if((sigPercentage > SOC_10_UPPER_LIMIT)&&(sigPercentage < SOC_15_LOWER_LIMIT))
        {
            if(sys_fullon)
            {
                IND_SET(nIND_BatLowAlarm_Yellow, nInd_On);
                DIC_SET(nDIC_BatPowerLow, nDic_Inactive);
                BUZ_SET(nBUZ_BatPowerLow, nBuz_Inactive);
            }
            IND_SET(nIND_SocDisRed, nInd_On);
        }
        /*After the low battery alarm is triggered, the alarm can only be cleared and the 
        normal battery display can be restored when the battery power is>=15%.*/
        else if(sigPercentage >= SOC_15_LOWER_LIMIT)
        {
            IND_SET(nIND_BatLowAlarm_Yellow, nInd_Off);
            DIC_SET(nDIC_BatPowerLow, nDic_Inactive);
            BUZ_SET(nBUZ_BatPowerLow, nBuz_Inactive);
            IND_SET(nIND_SocDisRed, nInd_Off);
            lowWarn = FALSE;
            HaveWarnedFlag = FALSE;
        }
    }
	
	if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
    {
    	IND_SET(nIND_BatLowAlarm_Yellow, nInd_On);
    }

	if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
		DIC_SET(nDIC_BatPowerLow, nDic_Inactive);
		BUZ_SET(nBUZ_BatPowerLow, nBuz_Inactive);
        if ((can_diag_get_sw_conf(CONF_IDX_DrivingPowerType) != nPowerType_PHEV)
            || (Rte_FISD_u8GetCONFIG_PHEV_TYPE() != nIND_PHEV_TYPE_3))
        {
            IND_SET(nIND_BatLowAlarm_Yellow, nInd_Off);
            IND_SET(nIND_BatLowAlarm_White, nInd_Off);
        }
    }
}

static void FISD_vPHEV_LowSOCDetect(void)
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	BOOL sig_timeout = IlGetBMSH_LowSOCRxTimeout();
	
	DIC_SET(nDIC_BatSeriousLoss,nDic_Inactive);
    DIC_SET(nDIC_BatSeriousLowSoc,nDic_Inactive);
	BUZ_SET(nBUZ_BatSeriousLoss,nBuz_Inactive);
	IND_SET(nIND_BatLowAlarm_Red, nInd_Off);

	if(sys_fullon && (!sig_timeout) && (FISD_u16SignalValue[nBMSH_LowSOC] == 0x3u))
	{
        DIC_SET(nDIC_BatSeriousLoss,nDic_Active);
        DIC_SET(nDIC_BatSeriousLowSoc,nDic_Active);
		BUZ_SET(nBUZ_BatSeriousLoss,nBuz_Active);
		IND_SET(nIND_BatLowAlarm_Red, nInd_On);
		IND_SET(nIND_BatLowAlarm_White, nInd_Off);
		IND_SET(nIND_BatLowAlarm_Yellow, nInd_Off);
	}

	if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
    	DIC_SET(nDIC_BatSeriousLoss,nDic_Inactive);
        DIC_SET(nDIC_BatSeriousLowSoc,nDic_Inactive);
		BUZ_SET(nBUZ_BatSeriousLoss,nBuz_Inactive);
		IND_SET(nIND_BatLowAlarm_Red, nInd_Off);
    }
}

static void FISD_vPHEVSystemDetect(void)
{
    BOOL timeout_hcu = IlGetHCU_HVReadyRxTimeout();
    U8 hv_ready_sig = IlGetRxHCU_HVReady();
    U16 pt_ready_sig = Rte_FISD_u16GetPTReadySts();
	static BOOL bMultipleTriggerFlag = FALSE;
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
    static U16 u16Timercnt = 0;
	if(sys_fullon == FALSE)
	{
		bMultipleTriggerFlag = FALSE;
        u16Timercnt = 0;
	}

    if(bMultipleTriggerFlag == TRUE)
    {
        if(u16Timercnt>=TIMECOUNT_5s)
        {
            DIC_SET(nDIC_PT_Ready, nDic_Inactive);
            BUZ_SET(nBUZ_PT_Ready, nBuz_Inactive);
        }
        else
        {
        	if(TextMgr_enGetDisplayText() == nDIC_PT_Ready)
			{
				u16Timercnt++;
			}
			else
			{
				u16Timercnt = 0;
			}
        }
    }

    if((FALSE==Rte_FISD_boGetPTReadyStsTimeout())&&pt_ready_sig&&sys_fullon &&(Rte_FISD_boAnimationPlayIsFinish()==TRUE))
    {
        IND_SET(nIND_PTReady, nInd_On);
		if(bMultipleTriggerFlag == FALSE)
		{
            DIC_SET(nDIC_PT_Ready, nDic_Active);
            BUZ_SET(nBUZ_PT_Ready, nBuz_Active);
            bMultipleTriggerFlag = TRUE;
        }
    }
    else
    {
        IND_SET(nIND_PTReady, nInd_Off);
        DIC_SET(nDIC_PT_Ready, nDic_Inactive);
        BUZ_SET(nBUZ_PT_Ready, nBuz_Inactive);
    }
    if((FALSE==IlGetHCU_BrkPsdRxTimeout())&&(Rte_u16GetBrkPsd() == 1)&&sys_fullon &&(Rte_FISD_boAnimationPlayIsFinish()==TRUE))
    {
        DIC_SET(nDIC_HcuBrkPsd, nDic_Active);
        BUZ_SET(nBUZ_HcuBrkPsd, nBuz_Active);
    }
    else
    {
        DIC_SET(nDIC_HcuBrkPsd, nDic_Inactive);
        BUZ_SET(nBUZ_HcuBrkPsd, nBuz_Inactive);
    }
}

static void FISD_vPHEV_PowerModeDetect(void)
{
	U8 sigpowermode = FISD_u16SignalValue[nPowerModeFed];
	U8 sigchangefail = IlGetRxHCU_PowerModeChangeFail();//FISD_u16SignalValue[nPowerModeChangeFail];
    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	BOOL timeout_changefail = IlGetHCU_PowerModeChangeFailRxTimeout();
	static BOOL boIsWarnOn = False;
    static U8 sigchangefail_Pre = 0u;
	
	IND_SET(nIND_PowerModeEV,nInd_Off);
	IND_SET(nIND_PowerModeHEV, nInd_Off);
	IND_SET(nIND_PowerModeEVplus, nInd_Off);
	DIC_SET(nDIC_Unable2ChangeMode, nDic_Inactive);
    BUZ_SET(nBUZ_Unable2ChangeMode, nBuz_Inactive);

    if(boIsWarnOn)
	{
		DIC_SET(nDIC_Unable2ChangeMode, nDic_Active);
        BUZ_SET(nBUZ_Unable2ChangeMode, nBuz_Active);
	}
	else
	{
		DIC_SET(nDIC_Unable2ChangeMode, nDic_Inactive);
        BUZ_SET(nBUZ_Unable2ChangeMode, nBuz_Inactive);
	}

    if(sys_fullon)
    {
#if 0	/*客户要求不关联ptready信号*/
    	if(IND_CHK(nIND_PTReady,nInd_On))
#endif
    	{
    		if(sigpowermode == 1)
    		{
    			IND_SET(nIND_PowerModeEV,nInd_On);
    		}
    		else if(sigpowermode == 2)
    		{
    			IND_SET(nIND_PowerModeHEV, nInd_On);
    		}
			else if(sigpowermode == 3)/*3代PHEV*/
    		{
    			IND_SET(nIND_PowerModeEVplus, nInd_On);
    		}

    		if(timeout_changefail == False)
    		{
    		    if((sigchangefail == 1)&&(sigchangefail_Pre == 0))
                {      
    		    	boIsWarnOn = True;
                    DIC_SET(nDIC_Unable2ChangeMode, nDic_Inactive);
        	        BUZ_SET(nBUZ_Unable2ChangeMode, nBuz_Inactive);
                }
    		}
			else
			{
    			boIsWarnOn = False;
    		}
    	}
    }
	else
	{
		boIsWarnOn = False;
	}
    sigchangefail_Pre = sigchangefail;

	if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
    	IND_SET(nIND_PowerModeEV,nInd_Off);
		IND_SET(nIND_PowerModeHEV, nInd_Off);
		IND_SET(nIND_PowerModeEVplus, nInd_Off);
        DIC_SET(nDIC_Unable2ChangeMode, nDic_Inactive);
        BUZ_SET(nBUZ_Unable2ChangeMode, nBuz_Inactive);
    }
}

static void FISD_vPHEV_eletricityModeDetect(void)
{
	U8 sigmode = FISD_u16SignalValue[nSocManageFed];
	BOOL sigtimeout = Rte_boGetSocManageFedTimeout();
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
    static U8 lastsigmode = 0;
    IND_SET(nIND_ElectricityModeDefault, nInd_Off);
	IND_SET(nIND_ElectricityModeAuto, nInd_Off);
	IND_SET(nIND_ElectricityModeSave, nInd_Off);
	IND_SET(nIND_ElectricityModeCharge, nInd_Off);

    if(IND_CHK(nIND_PTReady,nInd_On))
    {
        sigmode = FISD_u16SignalValue[nSocManageFed];
    }
    else
    {
        sigmode = lastsigmode;
    }

	if(sys_fullon && (!sigtimeout) && Rte_FISD_boAnimationPlayIsFinish())
	{
		if(sigmode == 0)
		{
			IND_SET(nIND_ElectricityModeDefault, nInd_On);
		}
		else if(sigmode == 1)
		{
			IND_SET(nIND_ElectricityModeSave, nInd_On);
		}
		else if(sigmode == 2)
		{
			IND_SET(nIND_ElectricityModeAuto, nInd_On);
		}
		else if(sigmode == 3)
		{
			IND_SET(nIND_ElectricityModeCharge, nInd_On);
		}
	}
    else
    {
        IND_SET(nIND_ElectricityModeDefault, nInd_Off);
        IND_SET(nIND_ElectricityModeAuto, nInd_Off);
        IND_SET(nIND_ElectricityModeSave, nInd_Off);
        IND_SET(nIND_ElectricityModeCharge, nInd_Off);
    }
    lastsigmode = sigmode;

}

static void FISD_vPHEV_RGCdetect(void)
{
	BOOL sigtimeout = Rte_boGetFuelTankLidSystemFailureStsTimeout();
	U8 sigcondition = FISD_u16SignalValue[nFuelTankPressureCR];
	U16 u16CurSpeed = SpeedGauge_DisplayValueGet();

	DIC_SET(nDIC_CheckRGC, nDic_Inactive);
	BUZ_SET(nBUZ_CheckRGC, nBuz_Inactive);
	DIC_SET(nDIC_ReadyForRefuel, nDic_Inactive);
    BUZ_SET(nBUZ_ReadyForRefuel, nBuz_Inactive);
	DIC_SET(nDIC_CompleteForRefuel, nDic_Inactive);
    BUZ_SET(nBUZ_CompleteForRefuel, nBuz_Inactive);
	/*PHEV3.0*/
	DIC_SET(nDIC_FuelTankLidSystemFailureSts, nDic_Inactive);
	BUZ_SET(nBUZ_FuelTankLidSystemFailureSts, nBuz_Inactive);

	if(!Rte_boGetFuelTankLidStsTimeout() && FISD_u16SignalValue[nFuelTankLidSts] == 1u)
	{
		if(u16CurSpeed >= 3u)
		{
			DIC_SET(nDIC_CloseGasCapAlways, nDic_Active);
			DIC_SET(nDIC_CloseGasCap, nDic_Inactive);
			BUZ_SET(nBUZ_CloseGasCapAlways, nBuz_Active);
			/*PHEV3.0*/
			DIC_SET(nDIC_FuelTankLidSts, nDic_Active);
			BUZ_SET(nBUZ_FuelTankLidSts, nBuz_Active);
		}
		else if(u16CurSpeed < 3u)
		{
			DIC_SET(nDIC_CloseGasCap, nDic_Active);
			DIC_SET(nDIC_CloseGasCapAlways, nDic_Inactive);
			BUZ_SET(nBUZ_CloseGasCapAlways, nBuz_Inactive);
			/*PHEV3.0*/
			DIC_SET(nDIC_FuelTankLidSts, nDic_Inactive);
			BUZ_SET(nBUZ_FuelTankLidSts, nBuz_Inactive);
		}
		else
		{
			/*keep curent state*/
		}
	}
	else
	{
		DIC_SET(nDIC_CloseGasCap, nDic_Inactive);
		DIC_SET(nDIC_CloseGasCapAlways, nDic_Inactive);
		BUZ_SET(nBUZ_CloseGasCapAlways, nBuz_Inactive);
		/*PHEV3.0*/
		DIC_SET(nDIC_FuelTankLidSts, nDic_Inactive);
		BUZ_SET(nBUZ_FuelTankLidSts, nBuz_Inactive);
	}

	if(!sigtimeout)
	{
		if(FISD_u16SignalValue[nFuelTankLidSystemFailureSts] == 1u)
		{
			DIC_SET(nDIC_CheckRGC, nDic_Active);
			BUZ_SET(nBUZ_CheckRGC, nBuz_Active);

			/*PHEV3.0*/
			DIC_SET(nDIC_FuelTankLidSystemFailureSts, nDic_Active);
			BUZ_SET(nBUZ_FuelTankLidSystemFailureSts, nBuz_Active);
		}

		if(sigcondition == 1u)
		{
			DIC_SET(nDIC_ReadyForRefuel, nDic_Active);
			BUZ_SET(nBUZ_ReadyForRefuel, nBuz_Active);
		}
		else if(sigcondition == 2u)
		{
			DIC_SET(nDIC_CompleteForRefuel, nDic_Active);
			BUZ_SET(nBUZ_CompleteForRefuel, nBuz_Active);
		}
	}
}

	static void FISD_vPHEV_RegenerateLevelDetect(void)
{
	BOOL sigtimeout = Rte_FISD_boGetRegenerateLevelTimeout();
	U16 sigcondition = Rte_FISD_u16GetRegenerateLevel();
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
    U8 u8Val = can_diag_get_sw_conf(CONF_IDX_DrivingPowerType);
	static U8 PreSigcondition = 0u;
	static BOOL bFirstin = TRUE;
	
	DIC_SET(nDIC_ShiftGear2Right, nDic_Inactive);
	IND_SET(nIND_RegenerateLevel, nInd_Off);
	DIC_SET(nDIC_EnterRegenerationSetting, nDic_Inactive);
	BUZ_SET(nBUZ_EnterRegenerationSetting,nBuz_Inactive);
    //BUZ_SET(nBUZ_RegenerateLevelLow,nBuz_Inactive);
    //BUZ_SET(nBUZ_RegenerateLevelMid,nBuz_Inactive);
    //BUZ_SET(nBUZ_RegenerateLevelHigh,nBuz_Inactive);
    BUZ_SET(nBUZ_ShiftGear2Right, nBuz_Inactive);

	if((!sigtimeout) && sys_fullon)
	{
		if(bFirstin)
		{
			PreSigcondition = sigcondition;
			bFirstin = FALSE;
		}
		else 
		{
			if(PreSigcondition != sigcondition)
			{
                if(u8Val == nPowerType_PHEV)
                {
    				if(sigcondition == 0u)
    				{
    					DIC_SET(nDIC_Recovery_Level_High, nDic_Active);
    					DIC_SET(nDIC_Recovery_Level_Middle, nDic_Inactive);
    					DIC_SET(nDIC_Recovery_Level_Low, nDic_Inactive);

                        BUZ_SET(nBUZ_RegenerateLevelLow,nBuz_Inactive);
                        BUZ_SET(nBUZ_RegenerateLevelMid,nBuz_Inactive);
                        BUZ_SET(nBUZ_RegenerateLevelHigh,nBuz_Active);
    				}
    				else if(sigcondition == 1u)
    				{
    					DIC_SET(nDIC_Recovery_Level_Middle, nDic_Active);
    					DIC_SET(nDIC_Recovery_Level_Low, nDic_Inactive);
    					DIC_SET(nDIC_Recovery_Level_High, nDic_Inactive);

                        BUZ_SET(nBUZ_RegenerateLevelLow,nBuz_Inactive);
                        BUZ_SET(nBUZ_RegenerateLevelMid,nBuz_Active);
                        BUZ_SET(nBUZ_RegenerateLevelHigh,nBuz_Inactive);
    				}
    				else if(sigcondition == 2u)
    				{
    					DIC_SET(nDIC_Recovery_Level_Low, nDic_Active);
    					DIC_SET(nDIC_Recovery_Level_High, nDic_Inactive);
    					DIC_SET(nDIC_Recovery_Level_Middle, nDic_Inactive);
                       
                        BUZ_SET(nBUZ_RegenerateLevelLow,nBuz_Active);
                        BUZ_SET(nBUZ_RegenerateLevelMid,nBuz_Inactive);
                        BUZ_SET(nBUZ_RegenerateLevelHigh,nBuz_Inactive);
    				}
    				else
    				{
    					DIC_SET(nDIC_Recovery_Level_High, nDic_Inactive);
    					DIC_SET(nDIC_Recovery_Level_Middle, nDic_Inactive);
    					DIC_SET(nDIC_Recovery_Level_Low, nDic_Inactive);
                        BUZ_SET(nBUZ_RegenerateLevelLow,nBuz_Inactive);
                        BUZ_SET(nBUZ_RegenerateLevelMid,nBuz_Inactive);
                        BUZ_SET(nBUZ_RegenerateLevelHigh,nBuz_Inactive);
    				}
                }
                else if(u8Val == nPowerType_EV)
                {
    				if(sigcondition == 0u)
    				{
    					DIC_SET(nDIC_Recovery_Level_Low, nDic_Active);
    					DIC_SET(nDIC_Recovery_Level_Middle, nDic_Inactive);
    					DIC_SET(nDIC_Recovery_Level_High, nDic_Inactive);

                        BUZ_SET(nBUZ_RegenerateLevelLow,nBuz_Active);
                        BUZ_SET(nBUZ_RegenerateLevelMid,nBuz_Inactive);
                        BUZ_SET(nBUZ_RegenerateLevelHigh,nBuz_Inactive);
    				}
    				else if(sigcondition == 1u)
    				{
    					DIC_SET(nDIC_Recovery_Level_Middle, nDic_Active);
    					DIC_SET(nDIC_Recovery_Level_Low, nDic_Inactive);
    					DIC_SET(nDIC_Recovery_Level_High, nDic_Inactive);

                        BUZ_SET(nBUZ_RegenerateLevelMid,nBuz_Active);
                        BUZ_SET(nBUZ_RegenerateLevelLow,nBuz_Inactive);
                        BUZ_SET(nBUZ_RegenerateLevelHigh,nBuz_Inactive);
    				}
    				else if(sigcondition == 2u)
    				{
    					DIC_SET(nDIC_Recovery_Level_High, nDic_Active);
    					DIC_SET(nDIC_Recovery_Level_Low, nDic_Inactive);
    					DIC_SET(nDIC_Recovery_Level_Middle, nDic_Inactive);

                        BUZ_SET(nBUZ_RegenerateLevelHigh,nBuz_Active);
                        BUZ_SET(nBUZ_RegenerateLevelLow,nBuz_Inactive);
                        BUZ_SET(nBUZ_RegenerateLevelMid,nBuz_Inactive);                        
    				}
    				else
    				{
    					DIC_SET(nDIC_Recovery_Level_High, nDic_Inactive);
    					DIC_SET(nDIC_Recovery_Level_Middle, nDic_Inactive);
    					DIC_SET(nDIC_Recovery_Level_Low, nDic_Inactive);
                        BUZ_SET(nBUZ_RegenerateLevelLow,nBuz_Inactive);
                        BUZ_SET(nBUZ_RegenerateLevelMid,nBuz_Inactive);
                        BUZ_SET(nBUZ_RegenerateLevelHigh,nBuz_Inactive);
    				}
                }
				PreSigcondition = sigcondition;
			}
		}

		if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
	    {
	    
	    }
	    else if(sigcondition != 3u)
		{
			IND_SET(nIND_RegenerateLevel, nInd_On);
		}

		if(FISD_u16SignalValue[nGearReturnWarn] == 1)
		{
			DIC_SET(nDIC_ShiftGear2Right, nDic_Active);
			BUZ_SET(nBUZ_ShiftGear2Right, nBuz_Active);	
		}
	}
	else
	{
		DIC_SET(nDIC_Recovery_Level_High, nDic_Inactive);
		DIC_SET(nDIC_Recovery_Level_Middle, nDic_Inactive);
		DIC_SET(nDIC_Recovery_Level_Low, nDic_Inactive);
	}
	if((FISD_u16SignalValue[nHCU_EgyReGenIndcrB] == 1) && (Rte_FISD_boAnimationPlayIsFinish() == TRUE))
	{
		DIC_SET(nDIC_EnterRegenerationSetting, nDic_Active);
		BUZ_SET(nBUZ_EnterRegenerationSetting, nBuz_Active);
	}

	if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
		DIC_SET(nDIC_Recovery_Level_High, nDic_Inactive);
		DIC_SET(nDIC_Recovery_Level_Middle, nDic_Inactive);
		DIC_SET(nDIC_Recovery_Level_Low, nDic_Inactive);
		DIC_SET(nDIC_ShiftGear2Right, nDic_Inactive);
		IND_SET(nIND_RegenerateLevel, nInd_Off);
		DIC_SET(nDIC_EnterRegenerationSetting, nDic_Inactive);
    }
}

static void FISD_vPowerLimitDetect(void)
{
	BOOL sigtimeout = Rte_FISD_boGetPowerLimitTimeout();
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

	IND_SET(nIND_LimitPower_Yellow, nInd_Off);
    DIC_SET(nDIC_PowerLimit, nDic_Inactive);

	if(sys_fullon && (!sigtimeout) && (Rte_FISD_u16GetPowerLimit() == 1u))
	{
		IND_SET(nIND_LimitPower_Yellow, nInd_On);
        DIC_SET(nDIC_PowerLimit, nDic_Active);
	}

	if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
    	IND_SET(nIND_LimitPower_Yellow, nInd_Off);
        DIC_SET(nDIC_PowerLimit, nDic_Inactive);        
    }
}

static void FISD_vPHEV_AVAS_Detect(void)
{
	BOOL sigtimeout = Rte_boGetFunctionStsTimeout();
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
    U8 u8AVAS_Switch = RteCanGet_AVAS_Switch_Internal();
    
	IND_SET(nIND_AVAS_Yellow, nInd_Off);
	
	if(TRUE == sys_fullon)
	{
        if (((FALSE == sigtimeout) && (FISD_u16SignalValue[nFunctionSts] == 1u))
            ||(AVAS_INTERNAL_SWITCH_OFF == u8AVAS_Switch))
        {
            IND_SET(nIND_AVAS_Yellow, nInd_On);
        }		
	}

	if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
	    IND_SET(nIND_AVAS_Yellow, nInd_Off);
    }
}

/*
static void FISD_vPHEV_IPB_detect(void)
{
	BOOL sigtimeout = Rte_boGetBrakeFluidStsTimeout();
	BOOL sys_fullon = boSystemOOMFullOnMode();
	
	DIC_SET(nDIC_BrakeDiscOverHeat, nDic_Inactive);
	DIC_SET(nDIC_CheckBrakeFluid, nDic_Inactive);
	DIC_SET(nDIC_DontBrakeAndAccelerator, nDic_Inactive);

	if(sys_fullon && (!sigtimeout))
	{
		if(FISD_u16SignalValue[nFWAWarningSts] == 0x2u)
		{
			DIC_SET(nDIC_BrakeDiscOverHeat, nDic_Active);
		}

		if(FISD_u16SignalValue[nBrakeFluidSts] == 0x1u)
		{
			DIC_SET(nDIC_CheckBrakeFluid, nDic_Active);
		}
		
		if(FISD_u16SignalValue[nTwofeetBrake] == 0x1u)
		{
			DIC_SET(nDIC_DontBrakeAndAccelerator, nDic_Active);
		}

		if(FISD_u16SignalValue[nBrakesystemFailSts] == 0x1u)
		{
			IND_SET(nIND_BrakeFluidLevelLow_EbdFault, nInd_On);
		}
	}

	if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
    	DIC_SET(nDIC_BrakeDiscOverHeat, nDic_Inactive);
		DIC_SET(nDIC_CheckBrakeFluid, nDic_Inactive);
		DIC_SET(nDIC_DontBrakeAndAccelerator, nDic_Inactive);
    }
}
*/
static void FISD_vPHEV_BMSH_InsulationStsDetect(void)
{
	BOOL sigtimeout = Rte_boGetBMSH_InsulationStsTimeout();
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	U8 sigsts = FISD_u16SignalValue[nBMSH_InsulationSts];
	
	IND_SET(nIND_BMSH_InsulationSts_Yellow, nInd_Off);
	IND_SET(nIND_BMSH_InsulationSts_Red, nInd_Off);
	DIC_SET(nDIC_RiskOfLeakage,nDic_Inactive);

	if(sys_fullon && (!sigtimeout))
	{
		if(sigsts == 1u)
		{
			IND_SET(nIND_BMSH_InsulationSts_Yellow, nInd_On);
		}
		else if(sigsts == 2u)
		{
			if(Rte_FISD_u8GetCONFIG_DrivingPowerType() == nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2)
			{
			    IND_SET(nIND_BMSH_InsulationSts_Red, nInd_On);
			}
            else
            {
                IND_SET(nIND_BMSH_InsulationSts_Yellow, nInd_On);
                DIC_SET(nDIC_RiskOfLeakage,nDic_Active);
			}
		}
		else if(sigsts == 3u)
		{
			if(Rte_FISD_u8GetCONFIG_DrivingPowerType() == nPowerType_PHEV && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2)
			{
			    IND_SET(nIND_BMSH_InsulationSts_Yellow, nInd_On);
			    DIC_SET(nDIC_RiskOfLeakage,nDic_Active);
            }
            else
            {
                IND_SET(nIND_BMSH_InsulationSts_Red, nInd_On);
            }
		}
	}

	if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
    	IND_SET(nIND_BMSH_InsulationSts_Yellow, nInd_Off);
		IND_SET(nIND_BMSH_InsulationSts_Red, nInd_Off);
    }
}

static void FISD_vPHEV_TurnOffIgnDetect(void)
{
	BOOL sigtimeout = Rte_FISD_boGetPTReadyStsTimeout();
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

	DIC_SET(nDIC_Turnoff_Ign, nDic_Inactive);
	BUZ_SET(nBUZ_Turnoff_Ign, nBuz_Inactive);

    if(IlGetBCM_4_DriverDoorStsRxTimeout() == FALSE
        && Rte_FISD_boGetPTReadyStsTimeout() == FALSE
        && IlGetTCU_G_STAT_LeverModeDispRxTimeout() == FALSE)
    {
    	if(sys_fullon && (1u == Rte_FISD_u16GetPTReadySts()) && \
    	(FISD_u16SignalValue[nDriverDoorSts] == 1u) && \
    	(FISD_u16SignalValue[nSTAT_LeverModeDisp] == 1u))
    	{
    		if((Rte_FISD_u8GetCONFIG_PHEV_TYPE()== nIND_PHEV_TYPE_2) ||\
				((Rte_FISD_u8GetCONFIG_PHEV_TYPE()== nIND_PHEV_TYPE_3) && (can_diag_get_sw_conf(CONF_IDX_One_ClickStartSwitch)== nOneClickStartSwitch_Present)))
    		{
	    		DIC_SET(nDIC_Turnoff_Ign, nDic_Active);
	    		BUZ_SET(nBUZ_Turnoff_Ign, nBuz_Active);
			}
    	}
    }

	if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
    	DIC_SET(nDIC_Turnoff_Ign, nDic_Inactive);
		BUZ_SET(nBUZ_Turnoff_Ign, nBuz_Inactive);
    }
}

static void FISD_vPHEV_BATPower_Red_Detect(void)
{
    BOOL bBattFaultLampStsTimeout = Rte_FISD_boGetBattFaultLampStsTimeout();
    BOOL bPackThermalRunawayLightTimeout = Rte_FISD_boGetPackThermalRunawayLightTimeout();
	BOOL sigtimeout3 = Rte_boGetPowertrainFaultTimeout();
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
    U16 u16BattFaultLampSts = Rte_FISD_u16GetBattFaultLampSts();
    U16 u16PackThermalRunawayLight = Rte_FISD_u16GetPackThermalRunawayLight();
	
	DIC_SET(nDIC_Check_BMS, nDic_Inactive);
	BUZ_SET(nBUZ_Check_BMS, nDic_Inactive);
	IND_SET(nIND_BATPower_Red, nInd_Off);
    IND_SET(nIND_BATPower_Red_PHEV3, nInd_Off);

	DIC_SET(nDIC_BAT_OverHeatAndGetAway, nDic_Inactive);
	BUZ_SET(nBUZ_BAT_OverHeatAndGetAway, nBuz_Inactive);

	DIC_SET(nDIC_Check_eMcu, nDic_Inactive);
	BUZ_SET(nBUZ_Check_eMcu, nBuz_Inactive);

	DIC_SET(nDIC_Charging_Failure, nDic_Inactive);
	BUZ_SET(nBUZ_Charging_Failure, nBuz_Inactive);

    DIC_SET(nDIC_Check_BMS_PHEV3, nDic_Inactive);
	BUZ_SET(nBUZ_Check_BMS_PHEV3, nDic_Inactive);
	DIC_SET(nDIC_Check_eMcu_PHEV3, nDic_Inactive);
	BUZ_SET(nBUZ_Check_eMcu_PHEV3, nBuz_Inactive);

	DIC_SET(nDIC_HVILSts, nDic_Inactive);
    BUZ_SET(nBUZ_HVILSts, nBuz_Inactive);
	
	if((sys_fullon && (!bBattFaultLampStsTimeout)&&(u16BattFaultLampSts == 1u)&&(Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2)&&(Rte_FISD_u8GetCONFIG_DrivingPowerType() == nPowerType_PHEV))|| \
		((!bPackThermalRunawayLightTimeout) && (u16PackThermalRunawayLight == 1u)&&(FISD_u16SignalValue[nKeySts] != 0u)&&(Rte_FISD_u8GetCONFIG_DrivingPowerType() != nPowerType_EV))|| \
		((False == Rte_boGetISGFSysFaultTimeout()) && (FISD_u16SignalValue[nISGFSysFault] == 1u)&& sys_fullon ) || \
		((False == Rte_boGetTMFSysFaultTimeout()) && (FISD_u16SignalValue[nTMFSysFault] == 1u) && sys_fullon) || \
		((False == Rte_boGetTMRSysFaultTimeout()) && (FISD_u16SignalValue[nTMRSysFault] == 1u) && sys_fullon) || \
		((False == Rte_boGetVehicleSystemFailureTimeout()) &&  (FISD_u16SignalValue[nVehicleSystemFailure] == 1u) && sys_fullon) ||\
		((FALSE == Rte_boGetChargeFaultTimeout()) && (FISD_u16SignalValue[nChargeFault] == 1u) && ((TRUE == Rte_FISD_boAnimationPlayIsFinish()) || (boSystemOOMPartOnMode() == TRUE))) || \
		((!sigtimeout3) && (FISD_u16SignalValue[nPowertrainFault] == 1u) && (sys_fullon)))
	{
		IND_SET(nIND_BATPower_Red, nInd_On);
	}

	/*PHEV3.0 环路互锁故障*/
	if((Rte_boGetHVILStsTimeout() == False) && (FISD_u16SignalValue[nHVILSts] == 1u) &&\
		((Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3) && (Rte_FISD_u8GetCONFIG_DrivingPowerType() == nPowerType_PHEV)) &&\
		sys_fullon)
	{
		IND_SET(nIND_BATPower_Red, nInd_On);
		DIC_SET(nDIC_HVILSts, nDic_Active);
    	BUZ_SET(nBUZ_HVILSts, nBuz_Active);
	}
		
    /*PHEV3.0/EV 动力电池系统故障指示灯*/
    if (sys_fullon && (!bBattFaultLampStsTimeout)&&(u16BattFaultLampSts == 1u)&&\
		(((Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3) && (Rte_FISD_u8GetCONFIG_DrivingPowerType() == nPowerType_PHEV)) ||\
		(Rte_FISD_u8GetCONFIG_DrivingPowerType() == nPowerType_EV)))
    {
        IND_SET(nIND_BATPower_Red_PHEV3, nInd_On);
    }

	if(sys_fullon && (!bBattFaultLampStsTimeout)&&(u16BattFaultLampSts == 1u))
	{
		/*2代PHEV  and EV*/
    	DIC_SET(nDIC_Check_BMS, nDic_Active);
    	BUZ_SET(nBUZ_Check_BMS, nBuz_Active);
		
		/*3代PHEV*/
        DIC_SET(nDIC_Check_BMS_PHEV3, nDic_Active);
    	BUZ_SET(nBUZ_Check_BMS_PHEV3, nBuz_Active);
	}
	
	if((!bPackThermalRunawayLightTimeout) && (u16PackThermalRunawayLight == 1u))
	{
		DIC_SET(nDIC_BAT_OverHeatAndGetAway, nDic_Active);
		BUZ_SET(nBUZ_BAT_OverHeatAndGetAway, nBuz_Active);
	}
		
	if(sys_fullon)
	{
		if(((False == Rte_boGetISGFSysFaultTimeout()) && (FISD_u16SignalValue[nISGFSysFault] == 1u)) || \
		   ((False == Rte_boGetTMFSysFaultTimeout()) && (FISD_u16SignalValue[nTMFSysFault] == 1u)) || \
		   ((False == Rte_boGetTMRSysFaultTimeout()) && (FISD_u16SignalValue[nTMRSysFault] == 1u)))
		{
            if(Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2
                || Rte_FISD_u8GetCONFIG_DrivingPowerType() == nPowerType_EV)
            {
    			DIC_SET(nDIC_Check_eMcu, nDic_Active);
    			BUZ_SET(nBUZ_Check_eMcu, nBuz_Active);
            }
            if(Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3)
            {
    			DIC_SET(nDIC_Check_eMcu_PHEV3, nDic_Active);
    			BUZ_SET(nBUZ_Check_eMcu_PHEV3, nBuz_Active);
            }
		}
	}

	if((FALSE == Rte_boGetChargeFaultTimeout()) 
        && (FISD_u16SignalValue[nChargeFault] == 1u)
        && ((TRUE == Rte_FISD_boAnimationPlayIsFinish()) || (boSystemOOMPartOnMode() == TRUE)))
    {
        DIC_SET(nDIC_Charging_Failure, nDic_Active);
        BUZ_SET(nBUZ_Charging_Failure, nBuz_Active);
    }
    else
    {
        DIC_SET(nDIC_Charging_Failure, nDic_Inactive);
        BUZ_SET(nBUZ_Charging_Failure, nBuz_Inactive);
    }

	if(TRUE != Rte_FISD_boAnimationPlayIsFinish() && sys_fullon)
    {
    	DIC_SET(nDIC_Check_BMS, nDic_Inactive);
		BUZ_SET(nBUZ_Check_BMS, nDic_Inactive);
		IND_SET(nIND_BATPower_Red, nInd_Off);
		DIC_SET(nDIC_Check_eMcu, nDic_Inactive);
		BUZ_SET(nBUZ_Check_eMcu, nBuz_Inactive);
        IND_SET(nIND_BATPower_Red_PHEV3, nInd_Off);
        DIC_SET(nDIC_Check_BMS_PHEV3, nDic_Inactive);
		BUZ_SET(nBUZ_Check_BMS_PHEV3, nDic_Inactive);
		DIC_SET(nDIC_Check_eMcu_PHEV3, nDic_Inactive);
		BUZ_SET(nBUZ_Check_eMcu_PHEV3, nBuz_Inactive);
		DIC_SET(nDIC_HVILSts, nDic_Inactive);
    	BUZ_SET(nBUZ_HVILSts, nBuz_Inactive);
    }


}

static void FISD_vPHEV_DisChargeFaultDetect(void)
{
	DIC_SET(nDIC_DisCharging_Failure, nDic_Inactive);
	BUZ_SET(nBUZ_DisCharging_Failure, nBuz_Inactive);

    if((FALSE == IlGetHCU_2_G_DchaFaultRxTimeout()) 
        && (FISD_u16SignalValue[DchaFault] == 1u)
        && ((TRUE == Rte_FISD_boAnimationPlayIsFinish()) || (boSystemOOMPartOnMode() == TRUE)))
    {
         DIC_SET(nDIC_DisCharging_Failure, nDic_Active);
         BUZ_SET(nBUZ_DisCharging_Failure, nBuz_Active);
    }
    else
    {
        DIC_SET(nDIC_DisCharging_Failure, nDic_Inactive);
        BUZ_SET(nBUZ_DisCharging_Failure, nBuz_Inactive);
    }
}

static void FISD_vPHEV_ChargingStsDetect(void)
{
    U8   u8Val = 0;
	IND_SET(nIND_OBC_CC_ConnectSts_Red, nInd_Off);
	IND_SET(nIND_Reserve_Charging_Open, nInd_Off);
    IND_SET(nIND_OBC_CC_ConnectSts_Blue, nInd_Off);
    IND_SET(nIND_PHEV_CC2_Charging, nInd_Off);
    DIC_SET(nDIC_EnterChargingPage, nDic_Inactive);
    static BOOL sbD3ShowChargePage = FALSE;
    static U16 suD3ShowCnt = 0;
    static U8 sbLastChargePage = nIND_HidePage;
    u8Val = can_diag_get_sw_conf(CONF_IDX_DrivingPowerType);

	if(u8GetChargePageState() 
        && ((u8GetBatteryStatus() != nIND_Hide) 
        || (u16GetEvBatteryStatus() != nIND_Hide)))
	{
        if((TRUE == Rte_FISD_boAnimationPlayIsFinish() && nPowerState_D1==Rte_FISD_GetPowerState()) 
            || (nPowerState_D2==Rte_FISD_GetPowerState()))
        {
            DIC_SET(nDIC_EnterChargingPage, nDic_Active);
            if(Rte_FISD_u16GetCc2_ConnectSts() == 1
                && Rte_FISD_boGetCc2_ConnectStsTimeout() == FALSE)
            {
                IND_SET(nIND_PHEV_CC2_Charging, nInd_On);
            }
            /*非D3模式复位显示标志和计时器*/
            sbD3ShowChargePage = TRUE;
            suD3ShowCnt = 0;
        }
        else if(nPowerState_D3==Rte_FISD_GetPowerState())
        {
            /*D3模式下充放电界面显示30s;
            充放电界面切换后重新计时;
            信号中断再恢复后重新计时;*/
            if(sbLastChargePage != u8GetChargePageState())
            {
                sbD3ShowChargePage = TRUE;
                suD3ShowCnt = 0;
            }
        
            if (sbD3ShowChargePage == TRUE)
            {
                DIC_SET(nDIC_EnterChargingPage, nDic_Active);
                if(Rte_FISD_u16GetCc2_ConnectSts() == 1
                    && Rte_FISD_boGetCc2_ConnectStsTimeout() == FALSE)
                {
                    IND_SET(nIND_PHEV_CC2_Charging, nInd_On);
                }
                if(suD3ShowCnt>=TIMECOUNT_30s)
                {
                    sbD3ShowChargePage = FALSE;
                }
                else
                {
                    suD3ShowCnt++;
                }
            }
            else
            {
                DIC_SET(nDIC_EnterChargingPage, nDic_Inactive);
                IND_SET(nIND_PHEV_CC2_Charging, nInd_Off);
            }
            sbLastChargePage = u8GetChargePageState();
        }
        else
        {
            /*非D3模式复位显示标志和计时器*/
            sbD3ShowChargePage = TRUE;
            suD3ShowCnt = 0;
        }
	}
    else
    {
        /*无充放电界面复位显示标志和计时器*/
        sbD3ShowChargePage = TRUE;
        suD3ShowCnt=0;
        DIC_SET(nDIC_EnterChargingPage, nDic_Inactive);
        IND_SET(nIND_PHEV_CC2_Charging, nInd_Off);
    }

    if((TRUE == Rte_FISD_boAnimationPlayIsFinish()) || (boSystemOOMPartOnMode() == TRUE))
    {
        if(u8Val == nPowerType_PHEV)
        {
            if((Rte_FISD_u16GetCc2_ConnectSts() == 1
                && Rte_FISD_boGetCc2_ConnectStsTimeout() == FALSE)
                || (Rte_FISD_u16GetCc_ConnectSts() == 1
                    && Rte_FISD_boGetCc_ConnectStsTimeout() == FALSE))
            {
                IND_SET(nIND_OBC_CC_ConnectSts_Red, nInd_On);
                IND_SET(nIND_OBC_CC_ConnectSts_Blue, nInd_Off);
            }
            else if(Rte_FISD_u16GetCc_ConnectSts() == 2
                    && Rte_FISD_boGetCc_ConnectStsTimeout() == FALSE)
            {
                IND_SET(nIND_OBC_CC_ConnectSts_Red, nInd_Off);
                IND_SET(nIND_OBC_CC_ConnectSts_Blue, nInd_On);
            }
            else
            {
                IND_SET(nIND_OBC_CC_ConnectSts_Red, nInd_Off);
                IND_SET(nIND_OBC_CC_ConnectSts_Blue, nInd_Off);
            }
        }
        else if(u8Val == nPowerType_EV)
        {
            if((Rte_FISD_u16GetCc2_ConnectSts() == 1
                && Rte_FISD_boGetCc2_ConnectStsTimeout() == FALSE)
                || (Rte_FISD_u16GetCc_ConnectSts() == 1
                    && Rte_FISD_boGetCc_ConnectStsTimeout() == FALSE))
            {
                IND_SET(nIND_OBC_CC_ConnectSts_Red, nInd_On);
                IND_SET(nIND_OBC_CC_ConnectSts_Blue, nInd_Off);
            }
            else if(Rte_FISD_u16GetCc_ConnectSts() == 2
                    && Rte_FISD_boGetCc_ConnectStsTimeout() == FALSE)
            {
                IND_SET(nIND_OBC_CC_ConnectSts_Red, nInd_Off);
                IND_SET(nIND_OBC_CC_ConnectSts_Blue, nInd_On);
            }
            else
            {
                IND_SET(nIND_OBC_CC_ConnectSts_Red, nInd_Off);
                IND_SET(nIND_OBC_CC_ConnectSts_Blue, nInd_Off);
            }
        }
    }
}

static void FISD_vPHEV_WarmerRemindDetect(void)
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	DIC_SET(nDIC_WarmerRemind, nDic_Inactive);
	BUZ_SET(nBUZ_WarmerRemind, nBuz_Inactive);

	if((Rte_boGetGBWarmUpReqTimeout() == FALSE) && (FISD_u16SignalValue[nGBWarmUpReq] == 1u)&&(sys_fullon))
	{
		DIC_SET(nDIC_WarmerRemind, nDic_Active);
		BUZ_SET(nBUZ_WarmerRemind, nBuz_Active);
	}

	if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
	{
		DIC_SET(nDIC_WarmerRemind, nDic_Inactive);
		BUZ_SET(nBUZ_WarmerRemind, nBuz_Inactive);
	}
}


static void FISD_vPHEV_HvSysFltStopReq_Detect(void)
{
	BOOL sigtimeout = Rte_boGetHCU_HvSysFltStopReqTimeout();
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

	DIC_SET(nDIC_SevereHigVolFault,nDic_Inactive);
	BUZ_SET(nBUZ_SevereHigVolFault,nBuz_Inactive);

	if((!sigtimeout) && (FISD_u16SignalValue[nHCU_HvSysFltStopReq] == 1u) && (sys_fullon))
	{
		DIC_SET(nDIC_SevereHigVolFault,nDic_Active);
		BUZ_SET(nBUZ_SevereHigVolFault,nBuz_Active);
	}
}

static void FISD_vPHEV_BatteryTempLightSts_Detect(void)
{
	BOOL sigtimeout = Rte_boGetBMSH_BatteryTempLightStsTimeout();
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    BOOL bPackThermalRunawayLightTimeout = Rte_FISD_boGetPackThermalRunawayLightTimeout();
    U16 u16PackThermalRunawayLight = Rte_FISD_u16GetPackThermalRunawayLight();

    IND_SET(nIND_BatteryTempLight, nInd_Off);
	DIC_SET(nDIC_BAT_temp_over_High,nDic_Inactive);
	DIC_SET(nDIC_BAT_temp_over_Low,nDic_Inactive);
	BUZ_SET(nBUZ_BAT_Temp_Over_High,nBuz_Inactive);
    BUZ_SET(nBUZ_BAT_Temp_Over_Low,nBuz_Inactive);

	if((!sigtimeout) && (FISD_u16SignalValue[nBMSH_BatteryTempLightSts] == 1u) && (sys_fullon))
	{
	    IND_SET(nIND_BatteryTempLight, nInd_On);
		DIC_SET(nDIC_BAT_temp_over_High,nDic_Active);
		DIC_SET(nDIC_BAT_temp_over_Low,nDic_Inactive);
    	BUZ_SET(nBUZ_BAT_Temp_Over_High,nBuz_Active);
        BUZ_SET(nBUZ_BAT_Temp_Over_Low,nBuz_Inactive);
	}
	else if((!sigtimeout) && (FISD_u16SignalValue[nBMSH_BatteryTempLightSts] == 2u) && (sys_fullon))
	{
	    IND_SET(nIND_BatteryTempLight, nInd_On);
		DIC_SET(nDIC_BAT_temp_over_Low,nDic_Active);
		DIC_SET(nDIC_BAT_temp_over_High,nDic_Inactive);
		BUZ_SET(nBUZ_BAT_Temp_Over_Low,nBuz_Active);
    	BUZ_SET(nBUZ_BAT_Temp_Over_High,nBuz_Inactive);        
	}

	/*EV 动力电池热失控报警*/
	if((!bPackThermalRunawayLightTimeout) && (u16PackThermalRunawayLight == 1u)&&(nPowerState_D1==Rte_FISD_GetPowerState() || nPowerState_D2==Rte_FISD_GetPowerState())&&\
		(Rte_FISD_u8GetCONFIG_DrivingPowerType() == nPowerType_EV))
	{
		IND_SET(nIND_BatteryTempLight, nInd_On);
	}
}

static void FISD_vPHEV3_LowBatteryInfo_Detect(void)
{
    DIC_SET(nDIC_LowBatteryInfo,nDic_Inactive);
	DIC_SET(nDIC_LowBatteryLowFuelInfo,nDic_Inactive);
	BUZ_SET(nBUZ_LowBatteryInfo,nBuz_Inactive);
	BUZ_SET(nBUZ_LowBatteryLowFuelInfo,nBuz_Inactive);

    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    if((Rte_boGetLowBatteryInfoTimeout() == False) &&\
        (FISD_u16SignalValue[nLowBatteryInfo] == 1u) && sys_fullon)
    {
		DIC_SET(nDIC_LowBatteryInfo,nDic_Active);
		BUZ_SET(nBUZ_LowBatteryInfo,nBuz_Active);
    }
	if((Rte_boGetLowBatteryLowFuelInfoTimeout() == False) &&\
        (FISD_u16SignalValue[nLowBatteryLowFuelInfo] == 1u) && sys_fullon)
    {
		DIC_SET(nDIC_LowBatteryLowFuelInfo,nDic_Active);
		BUZ_SET(nBUZ_LowBatteryLowFuelInfo,nBuz_Active);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
    	DIC_SET(nDIC_LowBatteryInfo,nDic_Inactive);
		DIC_SET(nDIC_LowBatteryLowFuelInfo,nDic_Inactive);
		BUZ_SET(nBUZ_LowBatteryInfo,nBuz_Inactive);
		BUZ_SET(nBUZ_LowBatteryLowFuelInfo,nBuz_Inactive);
    }
}

static void FISD_vPHEV3_HighTempLimit_Detect(void)
{
    DIC_SET(nDIC_HighTempLimit,nDic_Inactive);
	BUZ_SET(nBUZ_HighTempLimit,nBuz_Inactive);

    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    if((Rte_boGetHCU_2_G_HighTempLimitTimeout() == False) &&\
        (FISD_u16SignalValue[nHCU_2_G_HighTempLimit] == 1u) && sys_fullon)
    {
		DIC_SET(nDIC_HighTempLimit,nDic_Active);
		BUZ_SET(nBUZ_HighTempLimit,nBuz_Active);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
		DIC_SET(nDIC_HighTempLimit,nDic_Inactive);
		BUZ_SET(nBUZ_HighTempLimit,nBuz_Inactive);
    }
}

static void FISD_vPHEV3_ChrgFillerAjar_Detect(void)
{
    DIC_SET(nDIC_ChrgFillerAjar1,nDic_Inactive);
	DIC_SET(nDIC_ChrgFillerAjar2,nDic_Inactive);
	BUZ_SET(nBUZ_ChrgFillerAjar,nBuz_Inactive);

	if((TRUE == Rte_FISD_boAnimationPlayIsFinish()) || (boSystemOOMPartOnMode() == TRUE))
	{
	    if((Rte_boGetChrgFillerAjarTimeout() == False) &&\
	        (FISD_u16SignalValue[nChrgFillerAjar] == 1u))
	    {
			if(SpeedGauge_DisplayValueGet() >= 3u)
			{
				DIC_SET(nDIC_ChrgFillerAjar1,nDic_Active);
				BUZ_SET(nBUZ_ChrgFillerAjar,nBuz_Active);
			}
			else
			{
				DIC_SET(nDIC_ChrgFillerAjar2,nDic_Active);
			}
	    }
	}
}

static void FISD_vPHEV3_EDrvOnly_Detect(void)
{
    DIC_SET(nDIC_EDrvOnly,nDic_Inactive);
	BUZ_SET(nBUZ_EDrvOnly,nBuz_Inactive);

    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    if((Rte_boGetEDrvOnlyTimeout() == False) &&\
        (FISD_u16SignalValue[nEDrvOnly] == 1u) && sys_fullon)
    {
        DIC_SET(nDIC_EDrvOnly,nDic_Active);
	    BUZ_SET(nBUZ_EDrvOnly,nBuz_Active);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
    	DIC_SET(nDIC_EDrvOnly,nDic_Inactive);
	    BUZ_SET(nBUZ_EDrvOnly,nBuz_Inactive);
    }
}

static void FISD_vPHEV3_ForcedPrkgChrg_Detect(void)
{
    DIC_SET(nDIC_ForcedPrkgChrg,nDic_Inactive);

    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    if((Rte_boGetForcedPrkgChrgTimeout() == False) &&\
        (FISD_u16SignalValue[nForcedPrkgChrg] == 1u) && sys_fullon)
    {
        DIC_SET(nDIC_ForcedPrkgChrg,nDic_Active);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
    	DIC_SET(nDIC_ForcedPrkgChrg,nDic_Inactive);
    }
}

static void FISD_vPHEV3_EngSelfMaiTin_Detect(void)
{
    DIC_SET(nDIC_EngSelfMaiTin,nDic_Inactive);

    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    if((Rte_boGetEngSelfMaiTinTimeout() == False) &&\
        (FISD_u16SignalValue[nEngSelfMaiTin] == 1u) && sys_fullon)
    {
        DIC_SET(nDIC_EngSelfMaiTin,nDic_Active);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
    	DIC_SET(nDIC_EngSelfMaiTin,nDic_Inactive);
    }
}

static void FISD_vPHEV3_VehStrtWarn_Detect(void)
{
	static U16 u16VehStrtWarnTime = 0u;
    DIC_SET(nDIC_VehStrtWarn,nDic_Inactive);
	BUZ_SET(nBUZ_VehStrtWarn,nBuz_Inactive);

    if((Rte_boGetVehStrtWarnTimeout() == False) &&\
        (FISD_u16SignalValue[nVehStrtWarn] == 1u) && (Rte_FISD_GetPowerState() == nPowerState_D2))
    {
    	BUZ_SET(nBUZ_VehStrtWarn,nBuz_Active);
		if(u16VehStrtWarnTime < TIMECOUNT_10s)
		{
			if(TextMgr_enGetDisplayText() == nDIC_VehStrtWarn)
			{
				u16VehStrtWarnTime++;
			}
			else
			{
				u16VehStrtWarnTime = 0;
			}
			DIC_SET(nDIC_VehStrtWarn, nDic_Active);
			//BUZ_SET(nBUZ_VehStrtWarn,nBuz_Active);
		}
    }
	else
	{
		u16VehStrtWarnTime = 0;
	}
}


static void FISD_vEV_VehicleSystemFailure_Detect(void)
{
    DIC_SET(nDIC_VehicleSystemFailure,nDic_Inactive);
	BUZ_SET(nBUZ_VehicleSystemFailure,nBuz_Inactive);

    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    if((Rte_boGetVehicleSystemFailureTimeout() == False) &&\
        (FISD_u16SignalValue[nVehicleSystemFailure] == 1u) && sys_fullon)
    {
        DIC_SET(nDIC_VehicleSystemFailure,nDic_Active);
	    BUZ_SET(nBUZ_VehicleSystemFailure,nBuz_Active);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
    	DIC_SET(nDIC_VehicleSystemFailure,nDic_Inactive);
	    BUZ_SET(nBUZ_VehicleSystemFailure,nBuz_Inactive);
    }
}

static void FISD_vEV_InvldCdnToDrvr_Detect(void)
{
    DIC_SET(nDIC_InvldCdnToDrvr_1,nDic_Inactive);
    DIC_SET(nDIC_InvldCdnToDrvr_2,nDic_Inactive);
    DIC_SET(nDIC_InvldCdnToDrvr_3,nDic_Inactive);
    DIC_SET(nDIC_InvldCdnToDrvr_4,nDic_Inactive);
    DIC_SET(nDIC_InvldCdnToDrvr_5,nDic_Inactive);
    DIC_SET(nDIC_InvldCdnToDrvr_6,nDic_Inactive);
    DIC_SET(nDIC_InvldCdnToDrvr_7,nDic_Inactive);

    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    if((Rte_boGetInvldCdnToDrvrTimeout() == False) && sys_fullon)
    {
        if(FISD_u16SignalValue[nInvldCdnToDrvr] == 1u)
        {
            DIC_SET(nDIC_InvldCdnToDrvr_1,nDic_Active);
        }
        else if(FISD_u16SignalValue[nInvldCdnToDrvr] == 2u)
        {
            DIC_SET(nDIC_InvldCdnToDrvr_2,nDic_Active);
        }
        else if(FISD_u16SignalValue[nInvldCdnToDrvr] == 3u)
        {
            DIC_SET(nDIC_InvldCdnToDrvr_3,nDic_Active);
        }
        else if(FISD_u16SignalValue[nInvldCdnToDrvr] == 4u)
        {
            DIC_SET(nDIC_InvldCdnToDrvr_4,nDic_Active);
        }
        else if(FISD_u16SignalValue[nInvldCdnToDrvr] == 5u)
        {
            DIC_SET(nDIC_InvldCdnToDrvr_5,nDic_Active);
        }
        else if(FISD_u16SignalValue[nInvldCdnToDrvr] == 6u)
        {
            DIC_SET(nDIC_InvldCdnToDrvr_6,nDic_Active);
        }
        else if(FISD_u16SignalValue[nInvldCdnToDrvr] == 7u)
        {
            DIC_SET(nDIC_InvldCdnToDrvr_7,nDic_Active);
        }
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
    	DIC_SET(nDIC_InvldCdnToDrvr_1,nDic_Inactive);
        DIC_SET(nDIC_InvldCdnToDrvr_2,nDic_Inactive);
        DIC_SET(nDIC_InvldCdnToDrvr_3,nDic_Inactive);
        DIC_SET(nDIC_InvldCdnToDrvr_4,nDic_Inactive);
        DIC_SET(nDIC_InvldCdnToDrvr_5,nDic_Inactive);
        DIC_SET(nDIC_InvldCdnToDrvr_6,nDic_Inactive);
        DIC_SET(nDIC_InvldCdnToDrvr_7,nDic_Inactive);
    }
}

static void FISD_vEV_LowSOCCLMLimitSts_Detect(void)
{
    DIC_SET(nDIC_LowSOCCLMLimitSts,nDic_Inactive);

    if((Rte_boGetLowSOCCLMLimitStsTimeout() == False) &&\
        (FISD_u16SignalValue[nLowSOCCLMLimitSts] == 1u) && \
        ((TRUE == Rte_FISD_boAnimationPlayIsFinish()) || (boSystemOOMPartOnMode() == TRUE)))
    {
        DIC_SET(nDIC_LowSOCCLMLimitSts,nDic_Active);
    }
}

static void FISD_vEV_TowMode_Detect(void)
{
    DIC_SET(nDIC_TowMode,nDic_Inactive);

    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    if((Rte_boGetTowModeTimeout() == False) &&\
        (FISD_u16SignalValue[nTowMode] == 2u) && sys_fullon)
    {
        DIC_SET(nDIC_TowMode,nDic_Active);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
    	DIC_SET(nDIC_TowMode,nDic_Inactive);
    }
}

static void FISD_vEV_V2LFuncSts_Detect(void)
{
    DIC_SET(nDIC_V2LSts_FuncSts,nDic_Inactive);
    DIC_SET(nDIC_V2LFuncNotOpen,nDic_Inactive);
    DIC_SET(nDIC_V2LFuncDisable,nDic_Inactive);

	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    BOOL bCc_ConnectStsRxTimeout = Rte_FISD_boGetCc_ConnectStsTimeout();
    U16  u16Cc_ConnectSts = Rte_FISD_u16GetCc_ConnectSts();
    U16 u16SOCDisp = Rte_FISD_u16GetSOCDisp();
    u16SOCDisp *= 0.01;

    if((TRUE == Rte_FISD_boAnimationPlayIsFinish()) || (boSystemOOMPartOnMode() == TRUE))
    {
        if((bCc_ConnectStsRxTimeout == False) &&\
            (IlGetV2L_FunctionStsRxTimeout() == False) &&\
            (u16Cc_ConnectSts == 2u) &&
            (FISD_u16SignalValue[nV2L_FunctionSts] == 1u) &&\
            (u16SOCDisp >= FISD_u16SignalValue[nDischargeStopSOC]))
        {
            DIC_SET(nDIC_V2LSts_FuncSts,nDic_Active);
        }

        if((bCc_ConnectStsRxTimeout == False) &&\
            (IlGetV2L_FunctionStsRxTimeout() == False) &&\
            (u16Cc_ConnectSts == 2u) &&
            (FISD_u16SignalValue[nV2L_FunctionSts] == 0u) && sys_fullon)
        {
            DIC_SET(nDIC_V2LFuncNotOpen,nDic_Active);
        }

        if((bCc_ConnectStsRxTimeout == False) &&\
            (IlGetV2L_FunctionStsRxTimeout() == False) &&\
            (u16Cc_ConnectSts == 2u) &&
            (FISD_u16SignalValue[nV2L_FunctionSts] == 1u) &&\
            (u16SOCDisp < FISD_u16SignalValue[nDischargeStopSOC]))
        {
            DIC_SET(nDIC_V2LFuncDisable,nDic_Active);
        }
    }
}

static void FISD_vEV_VCU_HVReady_Detect(void)
{
    IND_SET(nIND_VCU_HVReady, nInd_Off);

    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    if((Rte_boGetHVReadyTimeout() == False) &&\
        (FISD_u16SignalValue[nHVReady] == 0u) && sys_fullon)
    {
        IND_SET(nIND_VCU_HVReady, nInd_On);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
    	IND_SET(nIND_VCU_HVReady, nInd_Off);
    }
}

static void FISD_vEV_TMF_MILSts_Detect(void)
{
    IND_SET(nIND_TMF_MILSts, nInd_Off);

    BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

    if((Rte_boGetTMF_MILStsTimeout() == False) &&\
        (FISD_u16SignalValue[nTMF_MILSts] == 2u) && sys_fullon)
    {
        IND_SET(nIND_TMF_MILSts, nInd_On);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
    	IND_SET(nIND_TMF_MILSts, nInd_Off);
    }
}

static void FISD_vEV_BMSH_PreWarmDis_Detect(void)
{
    DIC_SET(nDIC_BatteryPreHeat,nDic_Inactive);
    DIC_SET(nDIC_BatteryKeepWarm,nDic_Inactive);

    if((Rte_boGetPreWarmDisTimeout() == False) &&\
        ((TRUE == Rte_FISD_boAnimationPlayIsFinish()) || (boSystemOOMPartOnMode() == TRUE)))
    {
        if(FISD_u16SignalValue[nPreWarmDis] == 1u)
        {
            DIC_SET(nDIC_BatteryPreHeat,nDic_Active);
        }
        else if(FISD_u16SignalValue[nPreWarmDis] == 2u)
        {
            DIC_SET(nDIC_BatteryKeepWarm,nDic_Active);
        }        
    }
}

static void FISD_vEV_TrailerConnectSts_Detect(void)
{
    IND_SET(nIND_TrailerConnectSts, nInd_Off);
	DIC_SET(nDIC_TrailerConnectSts,nDic_Inactive);
	DIC_SET(nDIC_TrailerConnectSts_EV,nDic_Inactive);
    
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();

	if((Rte_boGetTrailerConnectStsTimeout() == False) &&\
        (FISD_u16SignalValue[nTrailerConnectSts] == 1u) && sys_fullon)
    {
        IND_SET(nIND_TrailerConnectSts, nInd_On);
        DIC_SET(nDIC_TrailerConnectSts,nDic_Active);
        DIC_SET(nDIC_TrailerConnectSts_EV,nDic_Active);
    }

    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        IND_SET(nIND_TrailerConnectSts, nInd_Off);
    	DIC_SET(nDIC_TrailerConnectSts,nDic_Inactive);
        DIC_SET(nDIC_TrailerConnectSts_EV,nDic_Inactive);
    }
}

static void FISD_vClearSmartADASStatus()
{
	DIC_SET(nDIC_ACC_Exit,nDic_Inactive);
	DIC_SET(nDIC_AEB_Unused,nDic_Inactive);
	DIC_SET(nDIC_AEB_Working,nDic_Inactive);
	DIC_SET(nDIC_CleanRadar,nDic_Inactive);
	DIC_SET(nDIC_CleanCamera,nDic_Inactive);
	DIC_SET(nDIC_CheckCamera,nDic_Inactive);
	DIC_SET(nDIC_CheckRadar,nDic_Inactive);
	DIC_SET(nDIC_FCW_Unused,nDic_Inactive);
	DIC_SET(nDIC_SlowDownImmediately,nDic_Inactive);
	DIC_SET(nDIC_FrontCarDriveAway,nDic_Inactive);
	DIC_SET(nDIC_ACC_Recover,nDic_Inactive);
	DIC_SET(nDIC_ADAS_UnusedCuzCamera,nDic_Inactive);
	DIC_SET(nDIC_ADAS_UnsedCuzRadar,nDic_Inactive);
	DIC_SET(nDIC_CP_Exit,nDic_Inactive);
	DIC_SET(nDIC_CP_Unsed,nDic_Inactive);
	DIC_SET(nDIC_ChangLaneUnableCuzSpd,nDic_Inactive);
	DIC_SET(nDIC_NotTime2ChangeL,nDic_Inactive);
	DIC_SET(nDIC_NOC_GoingtoExit,nDic_Inactive);
	DIC_SET(nDIC_TakeOver_1,nDic_Inactive);
	DIC_SET(nDIC_TakeOver_2,nDic_Inactive);
	DIC_SET(nDIC_TakeOver_3,nDic_Inactive);
	DIC_SET(nDIC_LongTimeTakeOver,nDic_Inactive);
	DIC_SET(nDIC_BSD_Error,nDic_Inactive);
	DIC_SET(nDIC_LDW_Error,nDic_Inactive);
	DIC_SET(nDIC_AEB_Off,nDic_Inactive);
	DIC_SET(nDIC_AEB_On,nDic_Inactive);
	DIC_SET(nDIC_AEB_Error,nDic_Inactive);
	DIC_SET(nDIC_ACC_Error,nDic_Inactive);
	DIC_SET(nDIC_FCW_Error,nDic_Inactive);
	DIC_SET(nDIC_EntranceRamp,nDic_Inactive);
	DIC_SET(nDIC_NOC_Turn2LeftLane,nDic_Inactive);
	DIC_SET(nDIC_NOC_Turn2RightLane,nDic_Inactive);
	DIC_SET(nDIC_Turn2LeftLane,nDic_Inactive);
	DIC_SET(nDIC_Turn2RightLane,nDic_Inactive);
	DIC_SET(nDIC_FCW_Off,nDic_Inactive);
	DIC_SET(nDIC_FCW_On,nDic_Inactive);
	DIC_SET(nDIC_CP_Error,nDic_Inactive);
	DIC_SET(nDIC_LDP_Error,nDic_Inactive);
	DIC_SET(nDIC_ELK_Error,nDic_Inactive);
	DIC_SET(nDIC_LongitudinalOverride,nDic_Inactive);
	DIC_SET(nDIC_Confirm2ChaneLeft,nDic_Inactive);
	DIC_SET(nDIC_Confirm2ChaneRight,nDic_Inactive);

	BUZ_SET(nBUZ_ACC_Exit,nBuz_Inactive);
	//BUZ_SET(nBUZ_AEB_Working,nBuz_Inactive);
	BUZ_SET(nBUZ_SlowDownImmediately,nBuz_Inactive);
	BUZ_SET(nBUZ_FrontCarDriveAway,nBuz_Inactive);
	BUZ_SET(nBUZ_ACC_Recover,nBuz_Inactive);
	BUZ_SET(nBUZ_ADAS_UnusedCuzCamera,nBuz_Inactive);
	BUZ_SET(nBUZ_ADAS_UnsedCuzRadar,nBuz_Inactive);
	BUZ_SET(nBUZ_CP_Exit,nBuz_Inactive);
	BUZ_SET(nBUZ_NOC_GoingtoExit,nBuz_Inactive);
	BUZ_SET(nBUZ_TakeOver_1,nBuz_Inactive);
	BUZ_SET(nBUZ_TakeOver_2,nBuz_Inactive);
	BUZ_SET(nBUZ_TakeOver_3,nBuz_Inactive);
	BUZ_SET(nBUZ_LongTimeTakeOver,nBuz_Inactive);
	BUZ_SET(nBUZ_EntranceRamp,nBuz_Inactive);
	BUZ_SET(nBUZ_NOC_Turn2LeftLane,nBuz_Inactive);
	BUZ_SET(nBUZ_NOC_Turn2RightLane,nBuz_Inactive);
	BUZ_SET(nBUZ_LateralOverride,nBuz_Inactive);
	BUZ_SET(nBUZ_LongitudinalOverride,nBuz_Inactive);
	BUZ_SET(nBUZ_Confirm2ChaneLeft,nBuz_Inactive);
	BUZ_SET(nBUZ_Confirm2ChaneRight,nBuz_Inactive);
}

#define RmninfoSet 1
#define RmninfoReset 2
#define Rmninfo_NUM 47
U8 RmninfoActiveNum = 0u;
U8 FISD_u8RmninfoIndicator2beDisp[Rmninfo_NUM] ={0u};
U16 RmninfoWarnText[Rmninfo_NUM] = {nDIC_ACC_Exit,    nDIC_ACC_Unused,       nDIC_AEB_Unused,    nDIC_AEB_Working,
	                           nDIC_CleanRadar,  nDIC_CleanCamera,      nDIC_CheckCamera,   nDIC_CheckRadar,
							   nDIC_FCW_Unused,  nDIC_SlowDownImmediately,  nDIC_FrontCarDriveAway,  nDIC_ACC_Recover,
							   nDIC_ADAS_UnusedCuzCamera,  nDIC_ADAS_UnsedCuzRadar,  nDIC_CP_Exit,   nDIC_CP_Unsed,
							   nDIC_ChangLaneUnableCuzSpd,  nDIC_NotTime2ChangeL,  nDIC_NOC_GoingtoExit,  nDIC_TakeOver_1,
							   nDIC_TakeOver_2,  nDIC_TakeOver_3,  nDIC_LongTimeTakeOver,  nDIC_BSD_Error,
							   nDIC_LDW_Error,  nDIC_AEB_Off,  nDIC_AEB_On,  nDIC_AEB_Error,
							   nDIC_ACC_Error,  nDIC_FCW_Error,  nDIC_EntranceRamp, nDIC_NOC_Turn2LeftLane,
							   nDIC_NOC_Turn2RightLane,  nDIC_Turn2LeftLane,   nDIC_Turn2RightLane,  nDIC_MaxNum,
							   nDIC_FCW_Off,   nDIC_FCW_On,   nDIC_CP_Error,   nDIC_LDP_Error,
							   nDIC_ELK_Error,  nDIC_LongitudinalOverride,  nDIC_Confirm2ChaneLeft,  nDIC_Confirm2ChaneRight,
							   nDIC_ACC_Active,  nDIC_CP_Active,  nDIC_DCLC_BadState};


U16 RmninfoWarnBuz[Rmninfo_NUM] = {nBUZ_ACC_Exit,   nBUZ_ACC_Unused,   nBUZ_AEB_Unused,   nBUZ_AEB_Working,
								   nBUZ_MaxNum, 	nBUZ_MaxNum,   nBUZ_MaxNum,   nBUZ_MaxNum,   
								   nBUZ_FCW_Unused,     nBUZ_SlowDownImmediately,   nBUZ_FrontCarDriveAway,  nBUZ_ACC_Recover,
								   nBUZ_ADAS_UnusedCuzCamera,  nBUZ_ADAS_UnsedCuzRadar,  nBUZ_CP_Exit,  nBUZ_CP_Unsed,
								    nBUZ_ChangLaneUnableCuzSpd,     nBUZ_NotTime2ChangeL,    nBUZ_NOC_GoingtoExit,     nBUZ_TakeOver_1,
								    nBUZ_TakeOver_2,   nBUZ_TakeOver_3,   nBUZ_LongTimeTakeOver,  nBUZ_MaxNum,
								    nBUZ_MaxNum,   nBUZ_AEB_Off,   nBUZ_AEB_On,   nBUZ_MaxNum,
								    nBUZ_ACC_Error,   nBUZ_MaxNum,   nBUZ_EntranceRamp,    nBUZ_NOC_Turn2LeftLane,
								    nBUZ_NOC_Turn2RightLane,   nBUZ_Turn2LeftLane,   nBUZ_Turn2RightLane,   nBUZ_LateralOverride,
								    nBUZ_FCW_Off,    nBUZ_FCW_On,   nBUZ_CP_Error,   nBUZ_MaxNum,
								    nBUZ_MaxNum,   nBUZ_LongitudinalOverride,    nBUZ_Confirm2ChaneLeft,  nBUZ_Confirm2ChaneRight,
								    nBUZ_ACC_Active,  nBUZ_CP_Active,  nBUZ_DCLC_BadState};
U16  FISD_u16RmninfoCostTimeFromLastRefresh[Rmninfo_NUM] = {0u};
BOOL FISD_boWarnRmninfoSts_Pre[Rmninfo_NUM] ={2u,2u,2u,2u,2u,2u,2u,2u,
											  2u,2u,2u,2u,2u,2u,2u,2u,
											  2u,2u,2u,2u,2u,2u,2u,2u,
											  2u,2u,2u,2u,2u,2u,2u,2u,
											  2u,2u,2u,2u,2u,2u,2u,2u,
											  2u,2u,2u,2u,2u,2u,2u};
static void FISD_vDeleteRmninfoFromIndicatorDispLine(U8 u8Ind_ID);
static void FISD_vRmninfoClearCostTimeFromLastRefresh(U8 Rmninfo_2Clear)
{
    FISD_u16RmninfoCostTimeFromLastRefresh[Rmninfo_2Clear - 1u] = 0u;
}
static void FISD_vRmninfoAccumulateCostTimeFromLastRefresh(void)
{
    U8 u8i;
    for(u8i = 0u; u8i < Rmninfo_NUM; u8i ++)
    {
        if((RmninfoSet == FISD_boWarnRmninfoSts_Pre[u8i]) && \
                (TIMECOUNT_15s > FISD_u16RmninfoCostTimeFromLastRefresh[u8i]))
        {
            FISD_u16RmninfoCostTimeFromLastRefresh[u8i] ++;
        }

        if(FISD_u16RmninfoCostTimeFromLastRefresh[u8i] >= TIMECOUNT_15s)
        {
            FISD_vDeleteRmninfoFromIndicatorDispLine(u8i + 1u); //need 验证
            FISD_boWarnRmninfoSts_Pre[u8i] = RmninfoReset;
        }
    }
}

static U8 FISD_u8LookUpWhichOneIndcatorRmninfo2Display(void)
{
    int8_T int8i = Rmninfo_NUM - 1u;
    U8 u8Need2Disp = 0xFF; /*none to display*/

    /*look for the last one*/
    for( ; int8i >= 0; int8i --)
    {
        if(0u != FISD_u8RmninfoIndicator2beDisp[int8i])
        {
            u8Need2Disp = FISD_u8RmninfoIndicator2beDisp[int8i];
            break;
        }
    }

    return u8Need2Disp;
}
static void FISD_vDeleteRmninfoFromIndicatorDispLine(U8 u8Ind_ID)
{
    U8 u8i;
    BOOL boFindVal = FALSE;

    for(u8i = 0u; u8i < Rmninfo_NUM; u8i ++)
    {
        if(FISD_u8RmninfoIndicator2beDisp[u8i] == u8Ind_ID)
        {
            FISD_u8RmninfoIndicator2beDisp[u8i] = 0u;
            boFindVal = TRUE;
            u8i ++;
            break;
        }
    }

    if(boFindVal == TRUE)
    {
        for( ; u8i < Rmninfo_NUM; u8i++)
        {
            if(0u != FISD_u8RmninfoIndicator2beDisp[u8i])
            {
                FISD_u8RmninfoIndicator2beDisp[u8i -1] = FISD_u8RmninfoIndicator2beDisp[u8i];
                FISD_u8RmninfoIndicator2beDisp[u8i] = 0u;
            }
        }
    }
}

static void  FISD_vInsertRmninfo2IndicatorDispLine(U8 u8Ind_ID)
{
    U8 u8i;
    BOOL boInsertSts = FALSE;
    U8 *pIndVisitID = FISD_u8RmninfoIndicator2beDisp;

    /*the one to be inserted is the one current displayed*/
    if(u8Ind_ID == FISD_u8LookUpWhichOneIndcatorRmninfo2Display())
    {
        return;
    }
    else
    {
        for(u8i = 0u; u8i < Rmninfo_NUM; u8i ++)
        {
            if(*pIndVisitID != 0u)
            {
                pIndVisitID ++;
            }
            else
            {
                *pIndVisitID = u8Ind_ID;
                boInsertSts = TRUE;
                break;
            }
        }

        /*the inserted line is full*/
        if(FALSE == boInsertSts)
        {
            FISD_vDeleteRmninfoFromIndicatorDispLine(u8Ind_ID);
            FISD_u8RmninfoIndicator2beDisp[Rmninfo_NUM - 1u] = u8Ind_ID;
        }
    }
}
void FISD_vClearRmninfoFaultStatus(void)
{
	U8 i;
	for(i=0;i<Rmninfo_NUM;i++)
	{	
		if(RmninfoWarnText[i] < nDIC_MaxNum)
		{
			DIC_SET(RmninfoWarnText[i], nDic_Inactive);
		}
		if(RmninfoWarnBuz[i] < nBUZ_MaxNum)
		{
			BUZ_SET(RmninfoWarnBuz[i], nBuz_Inactive);
		}
	}
	
}

static void FISD_vSmartADASDetect(void)
{
	U8 FISD_boGetWarnRmninfoTrig_Cur = RmninfoReset;
    U8 u8i,FISD_u8WarnRmninfoGet_Cur = 0;
	(U8)FISD_u16SignalValue[nRmninfo];
	/*init*/
	if(IlGetIDCU_12_RmninfoTrigRxTimeout() == FALSE)
	{
		FISD_boGetWarnRmninfoTrig_Cur = FISD_u16SignalValue[nRmninfoTrig];
		FISD_u8WarnRmninfoGet_Cur = (U8)FISD_u16SignalValue[nRmninfo];
	}
	FISD_vClearRmninfoFaultStatus();
	if(TRUE == WarningRmninfoneedInitFlag)
    {
        WarningRmninfoneedInitFlag = FALSE;
        for(u8i = 0u; u8i < Rmninfo_NUM; u8i ++)
        {
            FISD_boWarnRmninfoSts_Pre[u8i] = RmninfoReset;
            FISD_u8RmninfoIndicator2beDisp[u8i] = 0u;
            FISD_u16RmninfoCostTimeFromLastRefresh[u8i] = 0u;
        }
        WarnMessageIDActiveNum = 0u;
    }
	if(FALSE == WarningRmninfoneedInitFlag)/*Init success,and work normally*/
	{
		if((FISD_u8WarnRmninfoGet_Cur > 0u) && (FISD_u8WarnRmninfoGet_Cur < Rmninfo_NUM + 1) )
		{
			if(RmninfoSet == FISD_boGetWarnRmninfoTrig_Cur)            /*value == set*/
			{
				if(RmninfoReset == FISD_boWarnRmninfoSts_Pre[FISD_u8WarnRmninfoGet_Cur - 1u]) /*pre status == reset*/
				{
					FISD_boWarnRmninfoSts_Pre[FISD_u8WarnRmninfoGet_Cur - 1u] = RmninfoSet;
					FISD_vInsertRmninfo2IndicatorDispLine(FISD_u8WarnRmninfoGet_Cur);
					FISD_vRmninfoClearCostTimeFromLastRefresh(FISD_u8WarnRmninfoGet_Cur);
				}
				 else if(RmninfoSet == FISD_boWarnRmninfoSts_Pre[FISD_u8WarnRmninfoGet_Cur - 1u]) /*pre status == set*/
                {
                    FISD_vRmninfoClearCostTimeFromLastRefresh(FISD_u8WarnRmninfoGet_Cur);
                }
				else
                {
                }
			}
			else if(RmninfoReset == FISD_boGetWarnRmninfoTrig_Cur)  /*value == reset*/
            {
                FISD_boWarnRmninfoSts_Pre[FISD_u8WarnRmninfoGet_Cur - 1u] = RmninfoReset;
                FISD_vDeleteWMIDFromIndicatorDispLine(FISD_u8WarnRmninfoGet_Cur);
                FISD_vRmninfoClearCostTimeFromLastRefresh(FISD_u8WarnRmninfoGet_Cur);
            }
			else
			{
			}
		}
		else
		{
		}
	}
	FISD_vRmninfoAccumulateCostTimeFromLastRefresh();

	 /*display on*/
    switch(FISD_u8LookUpWhichOneIndcatorRmninfo2Display())
    {

	}
	for(u8i = 0u; u8i < Rmninfo_NUM; u8i ++)
	{
        if((RmninfoSet == FISD_boWarnRmninfoSts_Pre[u8i]))
        {	
        	if(RmninfoWarnText[u8i] < nDIC_MaxNum)
        	{
				DIC_SET(RmninfoWarnText[u8i], nDic_Active);
        	}

			if(RmninfoWarnBuz[u8i] < nBUZ_MaxNum)
        	{
				BUZ_SET(RmninfoWarnBuz[u8i], nBuz_Active);
        	}
		}
	}
	if((FALSE == Rte_FISD_boAnimationPlayIsFinish()) ||(IlGetIDCU_12_RmninfoTrigRxTimeout() == TRUE))
	{
		FISD_vClearRmninfoFaultStatus();
	}
}

static void FISD_vDMS_TipsDtetct(void)
{	
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	U8   alarmSts = RPC_GetDMS_FaceID_Alarm_State();
	DIC_SET(nDIC_LookAhead, nDic_Inactive);
	DIC_SET(nDIC_LookCamara, nDic_Inactive);
	DIC_SET(nDIC_KeepFaceClean, nDic_Inactive);
	DIC_SET(nDIC_KeepEyeClean, nDic_Inactive);
	DIC_SET(nDIC_KeepMouthClean, nDic_Inactive);
	DIC_SET(nDIC_PleaseTurnRight, nDic_Inactive);
	DIC_SET(nDIC_PleaseTurnLeft, nDic_Inactive);
	DIC_SET(nDIC_PleaseTurnDown, nDic_Inactive);
	DIC_SET(nDIC_PleaseTurnUp, nDic_Inactive);
	DIC_SET(nDIC_FaceIDRegistered, nDic_Inactive);

	DIC_SET(nDIC_FaceRecognizing, nDic_Inactive);
	DIC_SET(nDIC_RegistrationFailedCameraAbnormal, nDic_Inactive);
	DIC_SET(nDIC_RegistrationFailedPleaseAgain, nDic_Inactive);
	DIC_SET(nDIC_FaceUndetected, nDic_Inactive);
	DIC_SET(nDIC_FaceUnregistered, nDic_Inactive);
	DIC_SET(nDIC_RegistrationFailedDuplicateFace, nDic_Inactive);
	DIC_SET(nDIC_RegistrationTimedOutPleaseAgain, nDic_Inactive);
	DIC_SET(nDIC_LoginTimedOutPleaseLoginAgain, nDic_Inactive);
	DIC_SET(nDIC_RegistrationCloudErr, nDic_Inactive);

	if(sys_fullon)
	{
		switch(alarmSts)
		{
			case 0U: 
			{
				DIC_SET(nDIC_LookAhead, nDic_Active);
			}
			break;
			
			case 1U: 
			{
				DIC_SET(nDIC_LookCamara, nDic_Active);
			}
			break;
			
			case 2U: 
			{
				DIC_SET(nDIC_KeepFaceClean, nDic_Active);
			}
			break;
			
			case 3U: 
			{
				DIC_SET(nDIC_KeepEyeClean, nDic_Active);
			}
			break;
			
			case 4U:
			{
				DIC_SET(nDIC_KeepMouthClean, nDic_Active);
			}
			break;
			
			case 5U:
			{
				DIC_SET(nDIC_PleaseTurnRight, nDic_Active);
			}
			break;
			
			case 6U:
			{
				DIC_SET(nDIC_PleaseTurnLeft, nDic_Active);
			}
			break;
			
			case 7U:
			{
				DIC_SET(nDIC_PleaseTurnDown, nDic_Active);
			}
			break;

			case 8U:
			{
				DIC_SET(nDIC_PleaseTurnUp, nDic_Active);
			}
			break;
			
			case 9U:
			{
				DIC_SET(nDIC_FaceIDRegistered, nDic_Active);
			}
			break;

			case 10U:
			{
				DIC_SET(nDIC_FaceRecognizing, nDic_Active);
			}
			break;

			case 11U:
			{
				DIC_SET(nDIC_RegistrationFailedCameraAbnormal, nDic_Active);
			}
			break;
			
			case 12U:
			{
				DIC_SET(nDIC_RegistrationFailedPleaseAgain, nDic_Active);
			}
			break;
			
			case 13U:
			{
				DIC_SET(nDIC_FaceUndetected, nDic_Active);
			}
			break;
			
			case 14U:
			{
				DIC_SET(nDIC_FaceUnregistered, nDic_Active);
			}
			break;

			case 15U:
			{
				DIC_SET(nDIC_RegistrationFailedDuplicateFace, nDic_Active);
			}
			break;

			case 16U:
			{
				DIC_SET(nDIC_RegistrationTimedOutPleaseAgain, nDic_Active);
			}
			break;
			
			case 17U:
			{
				DIC_SET(nDIC_LoginTimedOutPleaseLoginAgain, nDic_Active);
			}
			break;
			
			case 18U:
			{
				DIC_SET(nDIC_RegistrationCloudErr, nDic_Active);
			}
			break;
		}
	}
}

static void FISD_vIPB_WarnDtetct(void)
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	if(sys_fullon == FALSE)
	{
		DIC_SET(nDIC_FWAWarningSts, nDic_Inactive);
		DIC_SET(nDIC_BrakeFluidSts, nDic_Inactive);
		DIC_SET(nDIC_TwofeetBrake, nDic_Inactive);
		BUZ_SET(nBUZ_TwofeetBrake, nBuz_Inactive);
	}
	if((TRUE == Rte_FISD_boAnimationPlayIsFinish()) &&(Rte_boGetBrakeFluidStsTimeout() == FALSE))
	{
		if(FISD_u16SignalValue[nBrakesystemFailSts] == 0x01)
		{
			IND_SET(nIND_BrakeFluidLevelLow_EbdFault, nInd_On);
		}

		if(FISD_u16SignalValue[nFWAWarningSts] == 0x02)
		{
			DIC_SET(nDIC_FWAWarningSts, nDic_Active);
		}
		else
		{
			DIC_SET(nDIC_FWAWarningSts, nDic_Inactive);
		}
		
		
		if(FISD_u16SignalValue[nBrakeFluidSts] == 0x01)
		{
			DIC_SET(nDIC_BrakeFluidSts, nDic_Active);
		}
		else
		{
			DIC_SET(nDIC_BrakeFluidSts, nDic_Inactive);
		}

		
		if(FISD_u16SignalValue[nTwofeetBrakeSts] == 0x01)
		{
			DIC_SET(nDIC_TwofeetBrake, nDic_Active);
			//BUZ_SET(nBUZ_TwofeetBrake, nBuz_Active);
		}
		else
		{
			DIC_SET(nDIC_TwofeetBrake, nDic_Inactive);
			BUZ_SET(nBUZ_TwofeetBrake, nBuz_Inactive);
		}
	}
	else
	{
		DIC_SET(nDIC_FWAWarningSts, nDic_Inactive);
		DIC_SET(nDIC_BrakeFluidSts, nDic_Inactive);
		DIC_SET(nDIC_TwofeetBrake, nDic_Inactive);
		BUZ_SET(nBUZ_TwofeetBrake, nBuz_Inactive);
	}

}

#define    TIMECOUNT_10Minute    60000u
static void FISD_vRearChildDetect(void)
{
	 static uint32	u32RearDoorOpenTime = 0;
	 static BOOL boDetectActive_1 = FALSE;
	 static BOOL boDetectActive_2 = FALSE;
	 static BOOL boDisplayDic = FALSE;
	 static BOOL DoorOpen = FALSE;
	 BOOL acc_off = (FALSE == IO_GET_PS2uP_ON_OFF_LOGIC());
	 static uint16	u16DicDisplayTime = 0;
	 DIC_SET(nDIC_PleaseCheckRearSeats, nDic_Inactive);
	 BUZ_SET(nBUZ_REARCHILDDET_SEATBELT, nBuz_Inactive);
	 if(((TRUE == FISD_u16SignalValue[nLHRdoorSts]) || (TRUE == FISD_u16SignalValue[nRHRDoorSts])) && (boSystemOOMPartOnMode() == TRUE))
	 {
		 DoorOpen = TRUE;
	 }
	 
	 if(DoorOpen == TRUE)
	 {
		u32RearDoorOpenTime++;
		if(u32RearDoorOpenTime < TIMECOUNT_10Minute)
		{
			if((TRUE == Rte_FISD_boGetPowerDependent())||(TRUE == FISD_u16SignalValue[nEngineSts]))
			{
				DoorOpen = FALSE;
				boDetectActive_1 = TRUE;
				u32RearDoorOpenTime = 0;
			}
		}
		else
		{
			DoorOpen = FALSE;
			boDetectActive_1 = FALSE;
		}
	 }
	 else
	 {
		u32RearDoorOpenTime = 0;
	 }

	 
	 
	 if((TRUE == Rte_FISD_boGetPowerDependent())||(TRUE == FISD_u16SignalValue[nEngineSts]))
	 {
		 if((TRUE == FISD_u16SignalValue[nLHRdoorSts]) || (TRUE == FISD_u16SignalValue[nRHRDoorSts]))
		 {
			boDetectActive_2 = TRUE;
		 }
		 else
		 {
			boDetectActive_2 = FALSE;
		 }
	 }

	 if(((IlGetBCM_4_KeyStsRxTimeout() == FALSE) && (IlGetRxBCM_4_KeySts() == 0)))
	 {
		 if((TRUE == boDetectActive_1) || (TRUE == boDetectActive_2))
		 {
			 DIC_SET(nDIC_PleaseCheckRearSeats, nDic_Active);
			 BUZ_SET(nBUZ_REARCHILDDET_SEATBELT, nBuz_Active);
		 }
	 }

	if((IlGetBCM_4_RHRDoorStsRxTimeout() == TRUE) || (IlGetEMS_1_G_EngineStsRxTimeout() == TRUE))
	{
		DIC_SET(nDIC_PleaseCheckRearSeats, nDic_Inactive);
		BUZ_SET(nBUZ_REARCHILDDET_SEATBELT, nBuz_Inactive);
	}
}


static void FISD_vChildProtectionDetect(void)
{
	static BOOL boIsWarnOn = False;
	static U16 u16ClsdWarnTime = 0;
	BOOL acc_line = IO_GET_PS2uP_ON_OFF_LOGIC();
	static BOOL acc_line_pre;

	DIC_SET(nDIC_FltySts, nDic_Inactive);
	DIC_SET(nDIC_ClsdWarn, nDic_Inactive);
	if((TRUE == Rte_FISD_boAnimationPlayIsFinish()) || (boSystemOOMPartOnMode() == TRUE))
	{
		if(Rte_boGetFltyStsTimeout() == FALSE)
		{
		
			if(FISD_u16SignalValue[nFltySts] == 1)
			{
				DIC_SET(nDIC_FltySts, nDic_Active);
			}

			if(FISD_u16SignalValue[nClsdWarn] == 1)
			{
				DIC_SET(nDIC_ClsdWarn, nDic_Active);
			}
				
		}
	}

	/*儿童保护间接检测*/
	DIC_SET(nDIC_ClsdWarn_BCM5, nDic_Inactive);
	if(Rte_boGetBCM5_clsdwarnTimeout() == FALSE)
	{
		if(FISD_u16SignalValue[nBCM5_clsdwarn] == 1)
		{
			boIsWarnOn = True;
		}
		else if(FISD_u16SignalValue[nBCM5_clsdwarn] == 2)
		{
			boIsWarnOn = False;
		}
	}
	else
	{
		boIsWarnOn = False;
	}
	
	if((TRUE == Rte_FISD_boAnimationPlayIsFinish()) || (boSystemOOMPartOnMode() == TRUE))
	{
		if(boIsWarnOn)
		{
			if(IO_GET_PS2uP_ON_OFF_LOGIC() == TRUE)
			{
				if(u16ClsdWarnTime < TIMECOUNT_11s)
				{
					if(TextMgr_enGetDisplayText() == nDIC_ClsdWarn_BCM5)
					{
						u16ClsdWarnTime++;
					}
					else
					{
						u16ClsdWarnTime = 0;
					}
					DIC_SET(nDIC_ClsdWarn_BCM5, nDic_Active);
				}
			}
			else
			{
				if(u16ClsdWarnTime < TIMECOUNT_6s)
				{
					if(TextMgr_enGetDisplayText() == nDIC_ClsdWarn_BCM5)
					{
						u16ClsdWarnTime++;
					}
					else
					{
						u16ClsdWarnTime = 0;
					}
					DIC_SET(nDIC_ClsdWarn_BCM5, nDic_Active);
				}
			}
		}
		else
		{
			u16ClsdWarnTime = 0;
		}
	}
	else
	{
		u16ClsdWarnTime = 0;
	}
	
	if(acc_line_pre != acc_line)
	{
		u16ClsdWarnTime = 0;
	}
	acc_line_pre = acc_line;

}


#if 0
static void FISD_vSmartADASDetect(void)
{
	BOOL sys_fullon = boSystemOOMFullOnMode();
	BOOL sigtimeout = Rte_boGetRmninfoTimeout();
	U8 	u8SigValue = FISD_u16SignalValue[nRmninfo];
	static U16 SetCount = 0; 
	U8 	u8RmninfoTrig = FISD_u16SignalValue[nRmninfoTrig];
	static U8 u8SigCount = 0u;
	FISD_vClearSmartADASStatus();

	if((!sigtimeout) && sys_fullon)
	{
			switch(u8SigValue)
			{
				case 0x01:
					DIC_SET(nDIC_ACC_Exit,nDic_Active);
					BUZ_SET(nBUZ_ACC_Exit,nBuz_Active);
					break;
				case 0x02:
					DIC_SET(nDIC_ACC_Unused,nDic_Active);
					break;
				case 0x03:
					DIC_SET(nDIC_AEB_Unused,nDic_Active);
					break;
				case 0x04:
					DIC_SET(nDIC_AEB_Working,nDic_Active);
					BUZ_SET(nBUZ_AEB_Working,nBuz_Active);
					break;
				case 0x05:
					DIC_SET(nDIC_CleanRadar,nDic_Active);
					break;
				case 0x06:
					DIC_SET(nDIC_CleanCamera,nDic_Active);
					break;
				case 0x07:
					DIC_SET(nDIC_CheckCamera,nDic_Active);
					break;
				case 0x08:
					DIC_SET(nDIC_CheckRadar,nDic_Active);
					break;
				case 0x09:
					DIC_SET(nDIC_FCW_Unused,nDic_Active);
					break;
				case 0x0A:
					DIC_SET(nDIC_SlowDownImmediately,nDic_Active);
					BUZ_SET(nBUZ_SlowDownImmediately,nBuz_Active);
					break;
				case 0x0B:
					DIC_SET(nDIC_FrontCarDriveAway,nDic_Active);
					BUZ_SET(nBUZ_FrontCarDriveAway,nBuz_Active);
					break;
				case 0x0C:
					DIC_SET(nDIC_ACC_Recover,nDic_Active);
					BUZ_SET(nBUZ_ACC_Recover,nBuz_Active);
					break;
				case 0x0D:
					DIC_SET(nDIC_ADAS_UnusedCuzCamera,nDic_Active);
					BUZ_SET(nBUZ_ADAS_UnusedCuzCamera,nBuz_Active);
					break;
				case 0x0E:
					DIC_SET(nDIC_ADAS_UnsedCuzRadar,nDic_Active);
					BUZ_SET(nBUZ_ADAS_UnsedCuzRadar,nBuz_Active);
					break;
				case 0x0F:
					DIC_SET(nDIC_CP_Exit,nDic_Active);
					BUZ_SET(nBUZ_CP_Exit,nBuz_Active);
					break;
				case 0x10:
					DIC_SET(nDIC_CP_Unsed,nDic_Active);
					BUZ_SET(nBUZ_NOC_GoingtoExit,nBuz_Active);
					break;
				case 0x11:
					DIC_SET(nDIC_ChangLaneUnableCuzSpd,nDic_Active);
					break;
				case 0x12:
					DIC_SET(nDIC_NotTime2ChangeL,nDic_Active);
					break;
				case 0x13:
					DIC_SET(nDIC_NOC_GoingtoExit,nDic_Active);
					break;
				case 0x14:
					DIC_SET(nDIC_TakeOver_1,nDic_Active);
					BUZ_SET(nBUZ_TakeOver_1,nBuz_Active);
					break;
				case 0x15:
					DIC_SET(nDIC_TakeOver_2,nDic_Active);
					BUZ_SET(nBUZ_TakeOver_2,nBuz_Active);
					break;
				case 0x16:
					DIC_SET(nDIC_TakeOver_3,nDic_Active);
					BUZ_SET(nBUZ_TakeOver_3,nBuz_Active);
					break;
				case 0x17:
					DIC_SET(nDIC_LongTimeTakeOver,nDic_Active);
					BUZ_SET(nBUZ_LongTimeTakeOver,nBuz_Active);
					break;
				case 0x18:
					DIC_SET(nDIC_BSD_Error,nDic_Active);
					break;
				case 0x19:
					DIC_SET(nDIC_LDW_Error,nDic_Active);
					break;
				case 0x1A:
					DIC_SET(nDIC_AEB_Off,nDic_Active);
					break;
				case 0x1B:
					DIC_SET(nDIC_AEB_On,nDic_Active);
					break;
				case 0x1C:
					DIC_SET(nDIC_AEB_Error,nDic_Active);
					break;
				case 0x1D:
					DIC_SET(nDIC_ACC_Error,nDic_Active);
					break;
				case 0x1E:
					DIC_SET(nDIC_FCW_Error,nDic_Active);
					break;
				case 0x1F:
					DIC_SET(nDIC_EntranceRamp,nDic_Active);
					BUZ_SET(nBUZ_EntranceRamp,nBuz_Active);
					break;
				case 0x20:
					DIC_SET(nDIC_NOC_Turn2LeftLane,nDic_Active);
					BUZ_SET(nBUZ_NOC_Turn2LeftLane,nBuz_Active);
					break;
				case 0x21:
					DIC_SET(nDIC_NOC_Turn2RightLane,nDic_Active);
					BUZ_SET(nBUZ_NOC_Turn2RightLane,nBuz_Active);
					break;
				case 0x22:
					DIC_SET(nDIC_Turn2LeftLane,nDic_Active);
					break;
				case 0x23:
					DIC_SET(nDIC_Turn2RightLane,nDic_Active);
					break;
				case 0x24:
					BUZ_SET(nBUZ_LateralOverride,nBuz_Active);
					break;
				case 0x25:
					DIC_SET(nDIC_FCW_Off,nDic_Active);
					break;
				case 0x26:
					DIC_SET(nDIC_FCW_On,nDic_Active);
					break;
				case 0x27:
					DIC_SET(nDIC_CP_Error,nDic_Active);
					break;
				case 0x28:
					DIC_SET(nDIC_LDP_Error,nDic_Active);
					break;
				case 0x29:
					DIC_SET(nDIC_ELK_Error,nDic_Active);
					break;
				case 0x2A:
					DIC_SET(nDIC_LongitudinalOverride,nDic_Active);
					BUZ_SET(nBUZ_LongitudinalOverride,nBuz_Active);
					break;
				case 0x2B:
					DIC_SET(nDIC_Confirm2ChaneLeft,nDic_Active);
					BUZ_SET(nBUZ_Confirm2ChaneLeft,nBuz_Active);
					break;
				case 0x2C:
					DIC_SET(nDIC_Confirm2ChaneRight,nDic_Active);
					BUZ_SET(nBUZ_Confirm2ChaneRight,nBuz_Active);
					break;
				default:
					break;
			}
		if(BUZ_CHK(nBUZ_AEB_Working,nBuz_Active))
		{
			u8SigCount++;
			if(u8SigCount > TIMECOUNT_2s)
			{
				u8SigCount=0u;
				BUZ_SET(nBUZ_AEB_Working,nBuz_Inactive);
			}
		}
		else
		{
			u8SigCount = 0u;
			BUZ_SET(nBUZ_AEB_Working,nBuz_Inactive);
		}
	}
}
#endif

static void FISD_vMaintainWarningDetect(void)
{
	U8 u8IntervalTime = Rte_ICMT_u16GetSlaInterTime();
	U16 u16IntervalDistance = Rte_ICMT_u16GetIntervalDistance();
	BOOL boNowIgnstatus = Rte_FISD_boGetPowerDependent();
	U8 u8SlaveVehMod = FISD_u16SignalValue[nSlaveVehMod];

	BOOL IsGpsTime = calendar_is_gps_or_internet_type();
	static U8 LastGpsSts = 0xff;

	BOOL IsReadyToAlarmTime = FALSE;
    static U16 u16TimerCnt = 0;
	IsReadyToAlarmTime = (IsGpsTime && (u8SlaveVehMod == 2U)/*normal mode*/) ? TRUE : FALSE;
    
    if(Rte_FISD_GetPowerState() != nPowerState_D1)//解决D2状态下保养里程清零回到D会触发文字报警
  	{        
  		u16TimerCnt = 0;    
	}    
    else    
  	{        
  		if (u16TimerCnt < TIMECOUNT_5s)       
		{            
			u16TimerCnt++;      
		}   
	}
    
	if((LastGpsSts == FALSE) && (IsGpsTime == TRUE))
	{
		MTN_vSetMaintain_time_data();/*非网络时间切换为网络时间，记录保养时间基准*/
	}
    u8IntervalTime = Rte_ICMT_u16GetSlaInterTime();

	if((RteCanGet_MaintenanceSwitch() == TRUE) && boNowIgnstatus && \
		(TRUE == Rte_FISD_boAnimationPlayIsFinish()) && (u16TimerCnt>= TIMECOUNT_5s))
	{
		if(((u8IntervalTime == 0) && IsReadyToAlarmTime) || (u16IntervalDistance == 0))
		{
			DIC_SET(nDIC_MaintainVehicle, nDic_Active);
			IND_SET(nIND_Mainten, nInd_On);
			BUZ_SET(nBUZ_MaintainVehicle, nBuz_Active);
		}
		else if((u8IntervalTime <= 10) && (u16IntervalDistance <= 500) && (IsReadyToAlarmTime))
		{
			DIC_SET(nDIC_Interval_Warning,nDic_Active);
			IND_SET(nIND_Mainten, nInd_On);
			BUZ_SET(nBUZ_MaintainVehicle, nBuz_Active);
		}
		else if((u8IntervalTime <= 10) && (IsReadyToAlarmTime))
		{
			DIC_SET(nDIC_IntervalDistance_Warning,nDic_Active);
			IND_SET(nIND_Mainten, nInd_On);
			BUZ_SET(nBUZ_MaintainVehicle, nBuz_Active);
		}				
		else if(u16IntervalDistance <= 500)
		{
			DIC_SET(nDIC_IntervalTime_Warning,nDic_Active);
			IND_SET(nIND_Mainten, nInd_On);
			BUZ_SET(nBUZ_MaintainVehicle, nBuz_Active);
		}
	}

	
	LastGpsSts = IsGpsTime;
}

static void FISD_vMaintainDetect(void)
{
	FISD_vMaintenanceClear();
	FISD_vMaintainWarningDetect();
}

static void FISD_vPilotDetect(void)
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	BOOL sigtimeout = Rte_boGetCPModTimeout();
	U8 	u8SigValue = FISD_u16SignalValue[nCPMod];
	
	IND_SET(nIND_CheryPilot_Grey, nInd_Off);
	IND_SET(nIND_CheryPilot_Green, nInd_Off);
	IND_SET(nIND_CheryPilot_Red, nInd_Off);
	IND_SET(nIND_CheryPilot_Yellow, nInd_Off);
	DIC_SET(nDIC_CPMod_StopSafety, nDic_Inactive);
	BUZ_SET(nBUZ_CPMod_StopSafety, nBuz_Inactive);

	DIC_SET(nDIC_AttentionVehiclesRight, nDic_Inactive);
	BUZ_SET(nBUZ_AttentionVehiclesRight, nBuz_Inactive);

	DIC_SET(nDIC_AttentionVehiclesLeft, nDic_Inactive);
	BUZ_SET(nBUZ_AttentionVehiclesLeft, nBuz_Inactive);
	if(sys_fullon && !sigtimeout && \
	(TRUE == Rte_FISD_boAnimationPlayIsFinish()))
	{
		switch(u8SigValue)
		{
			case 0x01:
			case 0x02:
				IND_SET(nIND_CheryPilot_Grey,nInd_On);
				break;
			case 0x03:
				IND_SET(nIND_CheryPilot_Green,nInd_On);
				break;
			case 0x04:
			case 0x05:
				IND_SET(nIND_CheryPilot_Grey,nInd_On);
				break;
			case 0x06:
				IND_SET(nIND_CheryPilot_Green,nInd_On);
				break;
			case 0x07:
			case 0x08:
				IND_SET(nIND_CheryPilot_Grey,nInd_On);
				break;
			case 0x09:
				IND_SET(nIND_CheryPilot_Red,nInd_On);
				DIC_SET(nDIC_CPMod_StopSafety, nDic_Active);
				BUZ_SET(nBUZ_CPMod_StopSafety, nBuz_Active);
				break;
			case 0x0A:
				IND_SET(nIND_CheryPilot_Yellow,nInd_On);
				break;
			default:
				break;
		}

		if(FISD_u16SignalValue[nFCTAWarnLe] == 0x01)
		{
			DIC_SET(nDIC_AttentionVehiclesLeft, nDic_Active);
			BUZ_SET(nBUZ_AttentionVehiclesLeft, nBuz_Active);	
		}
		if(FISD_u16SignalValue[nFCTAWarnRi] == 0x01)
		{
			DIC_SET(nDIC_AttentionVehiclesRight, nDic_Active);
			BUZ_SET(nBUZ_AttentionVehiclesRight, nBuz_Active);	
		}
	}
	if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
	{
		IND_SET(nIND_CheryPilot_Yellow, nInd_Off);
		if(can_diag_get_sw_conf(CONF_IDX_IDCU) == TRUE)
		{
			IND_SET(nIND_CheryPilot_Yellow, nInd_On);
		}
		IND_SET(nIND_CheryPilot_Grey, nInd_Off);
		IND_SET(nIND_CheryPilot_Green, nInd_Off);
		IND_SET(nIND_CheryPilot_Red, nInd_Off);
	}
	
}

static void FISD_vBatLow_Detect(void)
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	BOOL sigtimeout = Rte_boGetLbatipTimeout();
	U8 u8SigValue = FISD_u16SignalValue[nLbatip];
	U16 power_voltage = adc_async_get_pwr_value();
	static U16 BatLowWarnTime = 0;
	static U16 BatLowNoWarnTime = 0;
	U16 u16NextEngineSpeed = Rte_FISD_u16GetEngineSpeedValue();	

	if(FALSE == Rte_FISD_u8GetCONFIG_EBS())
	{
		//硬线ACC
		DIC_SET(nDIC_BatLowPlsStartEngine,nDic_Inactive);
        DIC_SET(nDIC_BatLowEntertainmentLimit,nDic_Inactive);
		if((u16NextEngineSpeed == 0) && (nPowerState_D1 == Rte_FISD_GetPowerState() || nPowerState_D2 == Rte_FISD_GetPowerState()))
		{
			if(power_voltage < 11400) //11.4V
			{
				BatLowWarnTime ++;
				BatLowNoWarnTime = 0;
			}
			else
			{
			}

			if(BatLowWarnTime >= TIMECOUNT_30s)
			{
				DIC_SET(nDIC_BatLowPlsStartEngine,nDic_Active);		
				BatLowWarnTime = TIMECOUNT_30s;
			}

			if(DIC_CHK(nDIC_BatLowPlsStartEngine,nDic_Active) == TRUE)
			{
				if(power_voltage > 11700) //11.7V
				{
					BatLowNoWarnTime ++;
				}
			}
			
			if(BatLowNoWarnTime >= TIMECOUNT_30s)
			{
				DIC_SET(nDIC_BatLowPlsStartEngine,nDic_Inactive);
				BatLowWarnTime = 0;
				BatLowNoWarnTime = TIMECOUNT_30s;
			}
		}
	}
	else
	{
		DIC_SET(nDIC_BatLowPlsStartEngine,nDic_Inactive);
		DIC_SET(nDIC_BatLowEntertainmentLimit,nDic_Inactive);
		if(!sigtimeout)
		{
			if(1u == u8SigValue)
			{
				DIC_SET(nDIC_BatLowPlsStartEngine,nDic_Active);
			}
			if(3u == u8SigValue)
			{
				DIC_SET(nDIC_BatLowEntertainmentLimit,nDic_Active);
			}
		}
	}

	if(FALSE == Rte_FISD_boAnimationPlayIsFinish())
	{
		//DIC_SET(nDIC_BatLowPlsStartEngine,nDic_Inactive);
		DIC_SET(nDIC_BatLowEntertainmentLimit,nDic_Inactive);
	}
}

static void FISD_vSlaveVehModDetect(void)
{
	U8 u8SlaveVehMod = FISD_u16SignalValue[nSlaveVehMod];
	static U8 Last_VehMode = 0xFF;
	BOOL sigtimeout = Rte_boGetSlaveVehModTimeout();
	DIC_SET(nDIC_FactoryMode , nDic_Inactive);
	DIC_SET(nDIC_TransportMode , nDic_Inactive);
	DIC_SET(nDIC_DynoMode , nDic_Inactive);
	DIC_SET(nDIC_CrashMode , nDic_Inactive);

	if(sigtimeout == FALSE)
	{
		switch(u8SlaveVehMod)
		{
			case 0:
				DIC_SET(nDIC_FactoryMode , nDic_Active);
				break;
			case 1:
				DIC_SET(nDIC_TransportMode , nDic_Active);
				break;
			case 2:
				break;
			case 3:
				DIC_SET(nDIC_DynoMode , nDic_Active);
				break;
			case 4:
				DIC_SET(nDIC_CrashMode , nDic_Active);
				break;
			case 5:
				DIC_SET(nDIC_FactoryMode , nDic_Active);
				break;
			case 6:
				DIC_SET(nDIC_TransportMode , nDic_Active);
				break;
			default :
				break;
		}

		if((Last_VehMode != 0xFF) && (Last_VehMode != 2) && (u8SlaveVehMod == 2))
		{
			MTN_vSetMaintain_time_data();
		}

		Last_VehMode = u8SlaveVehMod;
	}

	
}


static void FISD_vASUFaultDetect(void)
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	BUZ_SET(nBUZ_ASUFault , nBuz_Inactive);
	DIC_SET(nDIC_ASUFault , nDic_Inactive);
	if(((FISD_u16SignalValue[nASUSysFailrSts] == 0x01) || (FISD_u16SignalValue[nASUSysFailrSts] == 0x02))&& \
		(Rte_boGetASUSysFailrStsTimeout() == FALSE)&&\
		sys_fullon)

	{
		BUZ_SET(nBUZ_ASUFault , nBuz_Active);
		DIC_SET(nDIC_ASUFault , nDic_Active);
	}
}

static void FISD_vOUTDMS_TipsDetect(void)
{
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
	IND_SET(nIND_DMSSts_Green, 	nInd_Off);
	IND_SET(nIND_DMSSts_Yellow, nInd_Off);
	IND_SET(nIND_DrvCameraSts, 	nInd_Off);
	IND_SET(nIND_DrvAbnormStAla,nInd_Off);

	DIC_SET(nDIC_DMSSts , 			nDic_Inactive);
	DIC_SET(nDIC_DrvCameraSts , 	nDic_Inactive);
	DIC_SET(nDIC_DrvAbnormStAla_1 , nDic_Inactive);
	DIC_SET(nDIC_DrvAbnormStAla_2 , nDic_Inactive);
	DIC_SET(nDIC_DrvAbnormStAla_3 , nDic_Inactive);
	DIC_SET(nDIC_DrvAbnormStAla_4 , nDic_Inactive);
	DIC_SET(nDIC_DrvAbnormStAla_5 , nDic_Inactive);
	DIC_SET(nDIC_DrvAbnormStAla_6 , nDic_Inactive);
	DIC_SET(nDIC_DrvAbnormStAla_7 , nDic_Inactive);

	BUZ_SET(nBUZ_DrvAbnormStAla_1 , nBuz_Inactive);
	BUZ_SET(nBUZ_DrvAbnormStAla_2 , nBuz_Inactive);
	BUZ_SET(nBUZ_DrvAbnormStAla_3 , nBuz_Inactive);
	BUZ_SET(nBUZ_DrvAbnormStAla_4 , nBuz_Inactive);
	BUZ_SET(nBUZ_DrvAbnormStAla_5 , nBuz_Inactive);
	BUZ_SET(nBUZ_DrvAbnormStAla_6 , nBuz_Inactive);
	BUZ_SET(nBUZ_DrvAbnormStAla_7 , nBuz_Inactive);
	BUZ_SET(nBUZ_DMSSts 		  , nBuz_Inactive);
	BUZ_SET(nBUZ_DrvCameraSts 	  , nBuz_Inactive);

    if((Rte_FISD_boAnimationPlayIsFinish() ==TRUE)&&(Rte_boGetDMSStatusTimeout() == FALSE) &&\
		(Rte_boGetDrvCameraStsTimeout() == FALSE)&&(Rte_boGetDrvAbnormStAlaTimeout() == FALSE) &&\
		sys_fullon)	//开机动画完成,电源状态D1
    {
		if(FISD_u16SignalValue[nDMSStatus] == 3u)
		{
			IND_SET(nIND_DMSSts_Green, nInd_On);
		}
		else if(FISD_u16SignalValue[nDMSStatus] == 4u)
		{
			IND_SET(nIND_DMSSts_Yellow, nInd_On);
			DIC_SET(nDIC_DMSSts , nDic_Active);
			//BUZ_SET(nBUZ_DMSSts , nBuz_Active);
		}

		if(FISD_u16SignalValue[nDrvCameraSts] == 3u)
		{
			IND_SET(nIND_DMSSts_Yellow, nInd_On);
			DIC_SET(nDIC_DrvCameraSts , nDic_Active);
			//BUZ_SET(nBUZ_DrvCameraSts , nBuz_Active);
		}

		if((FISD_u16SignalValue[nDrvAbnormStAla] == 4u) || (FISD_u16SignalValue[nDrvAbnormStAla] == 5u)\
			||(FISD_u16SignalValue[nDrvAbnormStAla] == 6u))
		{
			IND_SET(nIND_DrvAbnormStAla, nInd_On);
		}
		else if((FISD_u16SignalValue[nDrvAbnormStAla] == 1u) || (FISD_u16SignalValue[nDrvAbnormStAla] == 2u)\
			||(FISD_u16SignalValue[nDrvAbnormStAla] == 7u) || (FISD_u16SignalValue[nDrvAbnormStAla] == 8u))
		{
			IND_SET(nIND_DrvCameraSts, nInd_On);
		}

		switch (FISD_u16SignalValue[nDrvAbnormStAla])
        {
        	case 1u:
            	DIC_SET(nDIC_DrvAbnormStAla_1 , nDic_Active);
				BUZ_SET(nBUZ_DrvAbnormStAla_1 , nBuz_Active);
                break;
				
            case 2u:
                DIC_SET(nDIC_DrvAbnormStAla_2 , nDic_Active);
				BUZ_SET(nBUZ_DrvAbnormStAla_2 , nBuz_Active);
                break;
				
			case 4u:
				DIC_SET(nDIC_DrvAbnormStAla_3 , nDic_Active);	
				BUZ_SET(nBUZ_DrvAbnormStAla_3 , nBuz_Active);
				break;

			case 5u:
				DIC_SET(nDIC_DrvAbnormStAla_4 , nDic_Active);	
				BUZ_SET(nBUZ_DrvAbnormStAla_4 , nBuz_Active);
				break;

			case 6u:
				DIC_SET(nDIC_DrvAbnormStAla_5 , nDic_Active);	
				BUZ_SET(nBUZ_DrvAbnormStAla_5 , nBuz_Active);
				break;

			case 7u:
				DIC_SET(nDIC_DrvAbnormStAla_6 , nDic_Active);
				BUZ_SET(nBUZ_DrvAbnormStAla_6 , nBuz_Active);
				break;

			case 8u:
				DIC_SET(nDIC_DrvAbnormStAla_7 , nDic_Active);	
                //BUZ_SET(nBUZ_DrvAbnormStAla_7 , nBuz_Active);	
				break;
            default:
                break;
        }		
	}
}

static void FISD_vINTDMS_TipsDetect(void)
{
    IND_SET(nIND_DMSSts_Green, 	nInd_Off);
	IND_SET(nIND_DMSSts_Yellow, nInd_Off);
	IND_SET(nIND_DrvCameraSts, 	nInd_Off);
	IND_SET(nIND_DrvAbnormStAla,nInd_Off);

				
	BOOL sys_fullon = Rte_FISD_boGetPowerDependent();
    if((Rte_FISD_boAnimationPlayIsFinish() ==TRUE) && (sys_fullon == TRUE))	
    {
          if(RPC_Get_warningflag() == TRUE)
          {
    		  if(RPC_Get_DRV_ABNORM_ST_ALRM_LED1()== nIND_DrvCameraSts)  //驾驶员异常告警状态报警
    		  {
                  IND_SET(nIND_DrvCameraSts, 	nInd_On);  
    		  }
    		  if(RPC_Get_DRV_ABNORM_ST_ALRM_LED2()== nIND_DrvAbnormStAla)
    		  {
                  IND_SET(nIND_DrvAbnormStAla, 	nInd_On);
    		  }
          }
          
          if(RPC_Get_CAMERA_ST_warning() == nIND_DMSSts_Yellow) //摄像头状态报警
     	  {
              IND_SET(nIND_DMSSts_Yellow, 	nInd_On);
          }
       
        switch (RPC_Get_ST_DMS_LED())
        {
        	case nIND_DMSSts_Green:
                if(RPC_Get_CAMERA_ST_warning() == nIND_DMSSts_Green)  //DMS服务正常且摄像头正常的时候显示绿灯
                {
                    IND_SET(nIND_DMSSts_Green, 	nInd_On);
                }
              break;
            case nIND_DMSSts_Yellow:
                IND_SET(nIND_DMSSts_Yellow, 	nInd_On);
              break;	
            default:
              break;
        }
            
	}
    if(TRUE != Rte_FISD_boAnimationPlayIsFinish())
    {
        IND_SET(nIND_DMSSts_Green, 	nInd_Off);
        IND_SET(nIND_DMSSts_Yellow, nInd_Off);
        IND_SET(nIND_DrvCameraSts, 	nInd_Off);
        IND_SET(nIND_DrvAbnormStAla,nInd_Off);
        
        DIC_SET(nDIC_DMSSts , 			nDic_Inactive);
        DIC_SET(nDIC_DrvCameraSts , 	nDic_Inactive);
        DIC_SET(nDIC_DrvAbnormStAla_1 , nDic_Inactive);
        DIC_SET(nDIC_DrvAbnormStAla_2 , nDic_Inactive);
        DIC_SET(nDIC_DrvAbnormStAla_3 , nDic_Inactive);
        DIC_SET(nDIC_DrvAbnormStAla_4 , nDic_Inactive);
        DIC_SET(nDIC_DrvAbnormStAla_5 , nDic_Inactive);
        DIC_SET(nDIC_DrvAbnormStAla_6 , nDic_Inactive);
        DIC_SET(nDIC_DrvAbnormStAla_7 , nDic_Inactive);

        BUZ_SET(nBUZ_DrvAbnormStAla_1 , nBuz_Inactive);
        BUZ_SET(nBUZ_DrvAbnormStAla_2 , nBuz_Inactive);
        BUZ_SET(nBUZ_DrvAbnormStAla_3 , nBuz_Inactive);
        BUZ_SET(nBUZ_DrvAbnormStAla_4 , nBuz_Inactive);
        BUZ_SET(nBUZ_DrvAbnormStAla_5 , nBuz_Inactive);
        BUZ_SET(nBUZ_DrvAbnormStAla_6 , nBuz_Inactive);
        BUZ_SET(nBUZ_DrvAbnormStAla_7 , nBuz_Inactive);
        BUZ_SET(nBUZ_DMSSts 		  , nBuz_Inactive);
        BUZ_SET(nBUZ_DrvCameraSts 	  , nBuz_Inactive);
    }

}

/*******************************************************************************************
* Function: FISD_vWarnIconDetect
* Description: Show
* Parameters: none
* Return: none
********************************************************************************************/

/*******************************************************************************************
* EOF: FISD.c
********************************************************************************************/
#endif
#define FinishCount 150
static void FISD_vCrankOnAndSelfCheckDetect(void)
{
	static U16 u16CrankOn_time = 0;
	static BOOL boCrankOnFlag = FALSE;
	//static U16 KeySts_pre = 0;
	static U16 u16AnimationPlayIsFinishCount = 0;
	if(Rte_FISD_boAnimationPlayIsFinish() == TRUE)
	{
		if(u16AnimationPlayIsFinishCount < FinishCount)
		{
			u16AnimationPlayIsFinishCount++;
		}
	}
	else
	{
		u16AnimationPlayIsFinishCount = 0;
	}
	
	if(IlGetBCM_4_KeyStsRxTimeout() == FALSE)
	{
		if(IlGetRxBCM_4_KeySts() == 3)//&&\
		//(KeySts_pre == 2))
		{
			boCrankOnFlag = TRUE;
		}
		
		if((IlGetRxBCM_4_KeySts() != 3) && (IlGetRxBCM_4_KeySts() != 2))
		{
			boCrankOnFlag = FALSE;
		}
	}
	else
	{
		boCrankOnFlag = FALSE;
	}
	
	if(boCrankOnFlag == TRUE)
	{
		if(u16CrankOn_time < TIMECOUNT_3s)
		{
			u16CrankOn_time++;
		}
	}
	else
	{
		u16CrankOn_time = 0;
	}
	if((TRUE == FISD_boInstrumentIsInSelfCheckSts()) ||\
		((boCrankOnFlag == TRUE) && (u16CrankOn_time < TIMECOUNT_3s)))
	{
		DIC_SET(nDIC_FillBrakeFluid,nDic_Inactive);
		DIC_SET(nDIC_CheckEspSystem,nDic_Inactive);
		DIC_SET(nDIC_CheckAbsSystem,nDic_Inactive);
		DIC_SET(nDIC_HDC_Failure,nDic_Inactive);
		DIC_SET(nDIC_HHC_Failure,nDic_Inactive);
		DIC_SET(nDIC_AutoHoldFailure,nDic_Inactive);
		DIC_SET(nDIC_AutoHoldActive,nDic_Inactive);
		DIC_SET(nDIC_HDC_Active,nDic_Inactive);

		BUZ_SET(nBUZ_AutoHoldActive,nBuz_Inactive);
		BUZ_SET(nBUZ_HDC_Active,nBuz_Inactive);
	}

	if(TRUE == FISD_boInstrumentIsInSelfCheckSts())
	{
		DIC_SET(nDIC_TakeOverFunExit,nDic_Inactive);
		DIC_SET(nDIC_TransTempHighPleaseStop,nDic_Inactive);
		DIC_SET(nDIC_TransTempHighHold5Minute_7DCT,nDic_Inactive);
		DIC_SET(nDIC_ShutEngineCheckCoolant,nDic_Inactive);
		DIC_SET(nDIC_TransmissionTempHigh,nDic_Inactive);
		DIC_SET(nDIC_GearboxSeriousFault,nDic_Inactive);
		DIC_SET(nDIC_TransmissionTempHighHold5Minute,nDic_Inactive);
		DIC_SET(nDIC_CautionParkOnHighSlope,nDic_Inactive);
		DIC_SET(nDIC_TakeOverWarning,nDic_Inactive);
		DIC_SET(nDIC_TakeOverSafty,nDic_Inactive);
		DIC_SET(nDIC_CDP_Failure,nDic_Inactive);
		DIC_SET(nDIC_TransmissionCheck,nDic_Inactive);
		DIC_SET(nDIC_TransmissionCheck_1,nDic_Inactive);
		DIC_SET(nDIC_TransmissionCheck_2,nDic_Inactive);
		DIC_SET(nDIC_CheckPas,nDic_Inactive);
		DIC_SET(nDIC_BlkStsLeft,nDic_Inactive);
		DIC_SET(nDIC_BlkStsRight,nDic_Inactive);
		DIC_SET(nDIC_LdpFault,nDic_Inactive);
		DIC_SET(nDIC_LdwFault,nDic_Inactive);
		DIC_SET(nDIC_ElkFault,nDic_Inactive);
		DIC_SET(nDIC_CheckBlindSystem,nDic_Inactive);
		DIC_SET(nDIC_ShutEngineCheckEngineOil,nDic_Inactive);
		DIC_SET(nDIC_GearFault_P,nDic_Inactive);
		DIC_SET(nDIC_CheckAirbag,nDic_Inactive);
		DIC_SET(nDIC_CheckTpms,nDic_Inactive);
		DIC_SET(nDIC_FillFuel,nDic_Inactive);
		DIC_SET(nDIC_Seatbelt_avh_epb,nDic_Inactive);
		DIC_SET(nDIC_CloseDoorSeatbelt_Avh_epb,nDic_Inactive);
		DIC_SET(nDIC_EPS_FailureToInitAngle,nDic_Inactive);
		DIC_SET(nDIC_EotNotLearnt,nDic_Inactive);
		DIC_SET(nDIC_PleaseStampBraking,nDic_Inactive);
		DIC_SET(nDIC_CarMovePleaseSwitch_P,nDic_Inactive);
		DIC_SET(nDIC_StepBrakePedalReleaseHandbrake,nDic_Inactive);
		DIC_SET(nDIC_NotInGearP,nDic_Inactive);
		DIC_SET(nDIC_SwitchtoP_AfterStopped,nDic_Inactive);
		DIC_SET(nDIC_BrakingTillShowGear_P,nDic_Inactive);
		DIC_SET(nDIC_OperateGearPWhenEngineRun,nDic_Inactive);
		DIC_SET(nDIC_Warming,nDic_Inactive);
		DIC_SET(nDIC_Warmed,nDic_Inactive);

		BUZ_SET(nBUZ_TransTempHighPleaseStop,nBuz_Inactive);
		BUZ_SET(nBUZ_TransTempHighHold5Minute_7DCT,nBuz_Inactive);
		BUZ_SET(nBUZ_CoolantTempHigh,nBuz_Inactive);
		BUZ_SET(nBUZ_TransmissionTempHigh, nBuz_Inactive);
		BUZ_SET(nBUZ_GearboxSeriousFault,nBuz_Inactive);
		BUZ_SET(nBUZ_TransmissionTempHighHold5Minute,nBuz_Inactive);
		BUZ_SET(nBUZ_TakeOverWarning,nBuz_Inactive);
		BUZ_SET(nBUZ_TakeOverSafty,nBuz_Inactive);
		BUZ_SET(nBUZ_EngineOilPressureLow,nBuz_Inactive);
		BUZ_SET(nBUZ_GearFault_P,nBuz_Inactive);
		BUZ_SET(nBUZ_FillFuel,nBuz_Inactive);
		BUZ_SET(nBUZ_PleaseStampBraking,nBuz_Inactive);
		BUZ_SET(nBUZ_CarMovePleaseSwitch_P,nBuz_Inactive);
		BUZ_SET(nBUZ_StepBrakePedalReleaseHandbrake,nBuz_Inactive);
		BUZ_SET(nBUZ_NotInGearP,nBuz_Inactive);
		BUZ_SET(nBUZ_SwitchtoP_AfterStopped,nBuz_Inactive);
		BUZ_SET(nBUZ_BrakingTillShowGear_P,nBuz_Inactive);
		BUZ_SET(nBUZ_OperateGearPWhenEngineRun,nBuz_Inactive);
		BUZ_SET(nBUZ_Warming,nBuz_Inactive);
		DIC_SET(nDIC_AebErr, nDic_Inactive);
		
	}
	//KeySts_pre = IlGetRxBCM_4_KeySts();
}

#define T19C_INT_MODELCODE      (0x07)
#define T1EJ_INT_MODELCODE      (0x09)

static void FISD_vVINComparisonDetect(void)
{
    DIC_SET(nDIC_VINIncorrect,nDic_Inactive);
    
    Rte_FISD_SetVINCompCorrectFlag(FALSE);
    
    /*T19C和3代PHEV执行*/
#if 1   /*T19C和T1EJ开启*/
    if ((Rte_FISD_u8GetCONFIG_MODEL_CODE() == T19C_INT_MODELCODE)
        ||((Rte_FISD_u8GetCONFIG_CHN_TIEJ() == TRUE)
        &&((nPowerType_PHEV==can_diag_get_sw_conf(CONF_IDX_DrivingPowerType) 
            && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3))))
#else   /*3代PHEV待开启*/
    if ((Rte_FISD_u8GetCONFIG_MODEL_CODE() == T19C_INT_MODELCODE)
        ||((nPowerType_PHEV==can_diag_get_sw_conf(CONF_IDX_DrivingPowerType) 
            && Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3)))
#endif            
    {
        if (nPowerState_D1==Rte_FISD_GetPowerState() && TRUE == Rte_FISD_boAnimationPlayIsFinish())
        {
            if (IlGetBDM_VINcode_10RxTimeout() == TRUE)
            {
                DIC_SET(nDIC_VINIncorrect,nDic_Active);
            }
            else
            {
                if (UDS_GetVinCodeByIndex(9) != IlGetRxBDM_VINcode_10()
                 || UDS_GetVinCodeByIndex(10) != IlGetRxBDM_VINcode_11()
                 || UDS_GetVinCodeByIndex(11) != IlGetRxBDM_VINcode_12()
                 || UDS_GetVinCodeByIndex(12) != IlGetRxBDM_VINcode_13()
                 || UDS_GetVinCodeByIndex(13) != IlGetRxBDM_VINcode_14()
                 || UDS_GetVinCodeByIndex(14) != IlGetRxBDM_VINcode_15()
                 || UDS_GetVinCodeByIndex(15) != IlGetRxBDM_VINcode_16()
                 || UDS_GetVinCodeByIndex(16) != IlGetRxBDM_VINcode_17())
                {
                    DIC_SET(nDIC_VINIncorrect,nDic_Active);
                }
                else
                {
                    Rte_FISD_SetVINCompCorrectFlag(TRUE);
                }
            }
        }
    }
}

/*主机语音交互：燃油
第一位：燃油低指示灯点亮
第二位：燃油低指示灯点亮 && 续航里程＜50
第三位：燃油短路与断路失效判断策略*/
U8 u8GetFuel_Information(void)
{
	U8 u8Fuel_Information = 0;
	extern BOOL remain_km_low_flag_to_ivi;
	if(IND_CHK(nIND_FuelLevelLow, nInd_Off) != FALSE)
	{
		u8Fuel_Information |= 0x01;
		if(remain_km_low_flag_to_ivi == TRUE)
		{
			u8Fuel_Information |= 0X02;
		}
		
	}
	if(FG_boGetFuelFailureSts() == TRUE)
	{
		u8Fuel_Information |= 0x04;
	}
	return u8Fuel_Information;
}


/*主机语音交互：胎压信息
第一位：胎压正常
第二位：胎压异常
第三位：BCM_5超时*/
U8 u8GetTirePressure_Information(void)
{
	U8 u8TirePressure_Information = 0;
	if(IlGetBCM_5_LHFTirePressureRxTimeout() == FALSE)
	{
		if(((IlGetRxBCM_5_TirePosWarning_LHFTire() >= 1) &&\ 
		(IlGetRxBCM_5_TirePosWarning_LHFTire() <= 6) &&\
		(IlGetRxBCM_5_TirePosWarning_LHFTire() != 5)) ||\
		((IlGetRxBCM_5_TirePosWarning_RHFTire() >= 1) &&\ 
		(IlGetRxBCM_5_TirePosWarning_RHFTire() <= 6) &&\
		(IlGetRxBCM_5_TirePosWarning_RHFTire() != 5)) ||\
		((IlGetRxBCM_5_TirePosWarning_LHRTire() >= 1) &&\ 
		(IlGetRxBCM_5_TirePosWarning_LHRTire() <= 6) &&\
		(IlGetRxBCM_5_TirePosWarning_LHRTire() != 5)) ||\
		((IlGetRxBCM_5_TirePosWarning_RHRTire() >= 1) &&\ 
		(IlGetRxBCM_5_TirePosWarning_RHRTire() <= 6) &&\
		(IlGetRxBCM_5_TirePosWarning_RHRTire() != 5)))
		{
			u8TirePressure_Information |= 0x02;
		}
		else
		{
			u8TirePressure_Information |= 0x01;
		}
	}
	else
	{
		u8TirePressure_Information |= 0x04;
	}
	return u8TirePressure_Information;
}

/*主机语音交互：水温信息
第一位：水温正常
第二位：水温异常
第三位：EMS_1_G超时*/
U8 u8GetCoolantTemp_Information(void)
{
	U8 u8CoolantTemp_Information = 0;
	if(IlGetEMS_1_G_EngineCoolantTemperatureFailStsRxTimeout() == FALSE)
	{
		if((CoolantTemp_enIndicatorColor() == TRUE) ||\
		(IlGetRxEMS_1_G_EngineCoolantTemperatureFailSts() == 1))
		{
			u8CoolantTemp_Information |=0x02;
		}
		else
		{
			u8CoolantTemp_Information |=0x01;
		}
	}
	else
	{
		u8CoolantTemp_Information |=0x04;
	}
	return u8CoolantTemp_Information;
}

/*主机语音交互：刹车/制动信息
第一位：刹车/制动正常
第二位：刹车/制动异常
第三位：CGW_ABS_ESP_G超时||  ESP_15超时*/
U8 u8GetBrakesystemAndEBDFailSts_Information(void)
{
	U8 BrakesystemAndEBDFailSts_Information = 0;
	if((IlGetABS_ESP_G_EBDFailStsRxTimeout() == FALSE) &&\
	(IlGetIPB_ESP_15_BrakesystemFailStsRxTimeout()== FALSE)
)
	{
		if((IlGetRxABS_ESP_G_EBDFailSts() == 1) ||\
		(IlGetRxIPB_ESP_15_BrakesystemFailSts() == 1))
		{
			BrakesystemAndEBDFailSts_Information |= 0x02;
		}
		else
		{
			BrakesystemAndEBDFailSts_Information |= 0x01;
		}
	} 
	else
	{
		BrakesystemAndEBDFailSts_Information |=0x04;
	}
	return BrakesystemAndEBDFailSts_Information;
}

BOOL boTextWarnIcon_Flag(void)
{
	BOOL Warn_Flag = FALSE;
	if(TextMgr_u32GetBrowseTotalNum() > 0)
	{
		Warn_Flag = TRUE;
	}
	return Warn_Flag;
}

#define FaultIndicatorIDCount 34
tenIndicatorId FaultIndicatorID[FaultIndicatorIDCount] = {nIND_HMA_Yellow, nIND_AFS_Fault,  nIND_EngineOilPressureLow,  nIND_ABS_Fault,  nIND_ESP_Fault,
									   nIND_MIL_Lamp,  nIND_EpcIndicator,  nIND_BrakeFluidLevelLow_EbdFault,  nIND_TyrePressureLow,  nIND_Airbag,
									   nIND_Charger_Iem,  nIND_EpbFault_Yellow_P,  nIND_StartStopSysFault,  nIND_AutoHold_Yellow_A,  nIND_EPS_Fault_Yellow,
									   nIND_EPS_Fault_Red,  nIND_ESCLSerFault,  nIND_ESCLFault,  nIND_GearboxFault_Red,  nIND_TransmissionFault_CVT_DCT,
									   nIND_TransmisionFault_Red,  nIND_TransmisionFault_Yellow,  nIND_HDC_Fault,  nIND_LDW_Yellow,  nIND_LDW_LKA_Yellow,
									   nIND_SLASpd_Defect,  nIND_FCW_AEB_Yellow,  nIND_ACC_YellowEmpty,  nIND_FuckTraffic_Yellow,  nIND_CheryPilot_Yellow,
									   nIND_CheryPilot_Red, nIND_GpfFull_Green, nIND_GpfOverLimited_Yellow,  nIND_WarningIcon
									  };
			
BOOL boFaultIndicatorFlag(void)
{
	U8 u8Index;
	BOOL boFlag = FALSE;
	for(u8Index = 0; u8Index< 34; u8Index++)
	{
		if(IND_CHK(FaultIndicatorID[u8Index], nInd_Off) != FALSE)
		{
			boFlag = TRUE;
			break;
		}
	}
	return boFlag;
}

U8 u8ChargePageState(void)
{
    U8 PageStatus = nIND_HidePage;
    U8   u8Val = u8GetBatteryStatus();
    switch(u8Val) 
    {
       case nIND_ChargeFinished:
       case nIND_Charging:
       case nIND_Waiting:
       case nIND_Heating:
       case nIND_ReserCharging:
       case nIND_TerminateCharge:
       case nIND_ChargingHeating:
       case nIND_ChargingCooling:
       case nIND_TherManagError:
       case nIND_BalanceTemp:
       case nIND_Cooling:
          PageStatus = nIND_ChargePage;
          break;
       case nIND_DisCharging:
       case nIND_DisChargeFinished:
          PageStatus = nIND_DisChargePage;
          break;
       default:
          PageStatus = nIND_HidePage;
    }

    return PageStatus;
}


U8 u8GetChargePageState(void)
{
    U8  PageStatus = nIND_HidePage;

    if((Rte_FISD_boGetCc2_ConnectStsTimeout() == FALSE && Rte_FISD_u16GetCc2_ConnectSts() == 1) ||\
        (Rte_FISD_boGetCc_ConnectStsTimeout() == FALSE && Rte_FISD_u16GetCc_ConnectSts() == 1))
    {
        /*充电界面显示*/
        PageStatus = nIND_ChargePage;
    }
    /*放电界面显示*/
    if (Rte_FISD_boGetCc_ConnectStsTimeout() == FALSE)
    {
        if (Rte_FISD_u16GetCc_ConnectSts() == 2)
        {
            if (PageStatus == nIND_HidePage)
            {
                PageStatus = nIND_DisChargePage;
            }
        }
    }
    
    return PageStatus;
}


U16 u16GetEvBatteryStatus(void)
{
    U8 Battery1 = nIND_Hide;
    U8 Battery2 = nIND_Hide;
    U16 u16RetVal = nIND_Hide;

    U8 u8SigVal1 = 0;
    U8 u8SigVal2 = 0;
    BOOL bCc2_ConnectStsRxTimeout = Rte_FISD_boGetCc2_ConnectStsTimeout();
    U16  u16Cc2_ConnectSts = Rte_FISD_u16GetCc2_ConnectSts();
    BOOL bPackChargingStsRxTimeout = Rte_FISD_boGetPackChargingStsTimeout();
    U16  u16PackChargingSts = Rte_FISD_u16GetPackChargingSts();

    if (can_diag_get_sw_conf(CONF_IDX_DrivingPowerType) == nPowerType_EV)
    {
        if(((bCc2_ConnectStsRxTimeout == FALSE)
            && (u16Cc2_ConnectSts == 1)
            &&(IlGetRxTBOX_ChargeModeSts() != 2))
            ||((Rte_FISD_boGetCc_ConnectStsTimeout() == FALSE)
            && (Rte_FISD_u16GetCc_ConnectSts() == 1)
            &&(IlGetRxTBOX_ChargeModeSts() != 2)))
        {
            if(Rte_FISD_boGetPackChargingStsTimeout() == FALSE)
            {
                u8SigVal1 = u16PackChargingSts;
                if(u8SigVal1 == 0)
                {
                    Battery1 = nIND_Waiting;
                }
                else if(u8SigVal1 == 1)
                {
                    Battery1 = nIND_Charging;
                }
                else if(u8SigVal1 == 2)
                {
                    Battery1 = nIND_ChargeFinished;
                }
                else if(u8SigVal1 == 3
                    || u8SigVal1 == 5)
                {
                    Battery1 = nIND_TerminateCharge;
                }
                else if(u8SigVal1 == 4)
                {
                    Battery1 = nIND_ChargingHeating;
                }
            }

            if(IlGetBMSH_PackChargingThermalStRxTimeout() == FALSE)
            {
                u8SigVal2 = IlGetRxBMSH_PackChargingThermalSt();
                
                if(u8SigVal2 == 1)
                {
                    Battery2 = nIND_Heating;
                }
                else if(u8SigVal2 == 2)
                {
                    Battery2 = nIND_ChargingCooling;
                }
                else if(u8SigVal2 == 3)
                {
                    Battery2 = nIND_Charging;
                }
                else if(u8SigVal2 == 4)
                {
                    Battery2 = nIND_ChargeFinished;
                }
                else if(u8SigVal2 == 5)
                {
                    Battery2 = nIND_TherManagError;
                }
                else if(u8SigVal2 == 6)
                {
                    Battery2 = nIND_BalanceTemp;
                }
                else if(u8SigVal2 == 7)
                {
                    Battery2 = nIND_Cooling;
                }
            }
            u16RetVal = (((U16)Battery2)<<8) + (U16)Battery1;
        }
        
        /*预约充电条件判断*/
        if(((Rte_FISD_boGetCc_ConnectStsTimeout() == FALSE)
             &&(Rte_FISD_u16GetCc_ConnectSts() == 1)
             &&(IlGetBMSH_PackChargingThermalStRxTimeout() == FALSE)
             &&(IlGetRxBMSH_PackChargingThermalSt() == 0))
         ||((bCc2_ConnectStsRxTimeout == FALSE)
             &&(u16Cc2_ConnectSts == 1)
             &&(bPackChargingStsRxTimeout == FALSE)
             &&(u16PackChargingSts == 0)))
        {
            if((IlGetTBOX_ChargeModeStsRxTimeout() == FALSE)
                &&(IlGetRxTBOX_ChargeModeSts() == 2))
            {
                u16RetVal = nIND_ReserCharging;
            }
        }
        else if(((Rte_FISD_boGetCc_ConnectStsTimeout() == FALSE)
             &&(Rte_FISD_u16GetCc_ConnectSts() == 1)
             &&(IlGetBMSH_PackChargingThermalStRxTimeout() == FALSE)
             &&(IlGetRxBMSH_PackChargingThermalSt() != 0))
         ||((bCc2_ConnectStsRxTimeout == FALSE)
             &&(u16Cc2_ConnectSts == 1)
             &&(bPackChargingStsRxTimeout == FALSE)
             &&(u16PackChargingSts != 0)))
        {
            if((IlGetTBOX_ChargeModeStsRxTimeout() == FALSE)
                &&(IlGetRxTBOX_ChargeModeSts() == 2))
            {
                u16RetVal = nIND_Charging;
            }
        }
        
        if(Rte_FISD_boGetCc_ConnectStsTimeout() == FALSE)
        {
            if(Rte_FISD_u16GetCc_ConnectSts() == 2)
            {
                if(IlGetV2L_FunctionStsRxTimeout() == FALSE
                    && IlGetV2L_DischargeStopSOC_IHURxTimeout() == FALSE
                    && Rte_FISD_boGetSOCDispTimeout() == FALSE
                    && (IlGetRxV2L_FunctionSts() == 1)
                    && (Rte_FISD_u16GetSOCDisp()>=(IlGetRxV2L_DischargeStopSOC_IHU()*100)))
                {
                    if (u16RetVal == nIND_Hide)
                    {
                        u16RetVal = nIND_DisChargeEnable;
                    }
                }
            }
        }
    }

	return u16RetVal;
}

U8 u8GetBatteryStatus(void)
{
	U8 Battery = nIND_Hide;
    U8 u8SigVal = 0;
    BOOL bCc2_ConnectStsRxTimeout = Rte_FISD_boGetCc2_ConnectStsTimeout();
    U16  u16Cc2_ConnectSts = Rte_FISD_u16GetCc2_ConnectSts();
    BOOL bPackChargingStsRxTimeout = Rte_FISD_boGetPackChargingStsTimeout();
    U16  u16PackChargingSts = Rte_FISD_u16GetPackChargingSts();

    if(can_diag_get_sw_conf(CONF_IDX_DrivingPowerType) == nPowerType_PHEV)
    {
        if(bCc2_ConnectStsRxTimeout == FALSE)
        {
            if((u16Cc2_ConnectSts == 1)
                &&(IlGetRxTBOX_ChargeModeSts() != 2))
            {
                u8SigVal = u16PackChargingSts;
                if(u8SigVal == 0 && (Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2))
                {
                    Battery = nIND_Waiting;
                }
                else if(u8SigVal == 1)
                {
                    if (Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3)
                    {
                        Battery = nIND_PHEV3_Heating;
                    }
                    else
                    {
                        Battery = nIND_Heating;
                    }
                }
                else if(u8SigVal == 2
                    || u8SigVal == 3
                    || u8SigVal == 6
                    || u8SigVal == 7
                    || u8SigVal == 8)
                {
                    Battery = nIND_Charging;
                }
                else if(u8SigVal == 4)
                {
                    Battery = nIND_ChargeFinished;
                }                
            }
        }
        
        if(Rte_FISD_boGetCc_ConnectStsTimeout() == FALSE)
        {
            if((Rte_FISD_u16GetCc_ConnectSts() == 1)
                &&(IlGetRxTBOX_ChargeModeSts() != 2))
            {
                u8SigVal = IlGetRxBMSH_PackChargingThermalSt();
                if(u8SigVal == 0 && (Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2))
                {
                    if (Battery == nIND_Hide)
                    {
                        Battery = nIND_Waiting;
                    }
                }
                else if(u8SigVal == 1)
                {
                    if (Battery == nIND_Hide)
                    {   
                        if (Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3)
                        {
                            Battery = nIND_PHEV3_Heating;
                        }
                        else
                        {
                            Battery = nIND_Heating;
                        }                        
                    }
                }
                else if(u8SigVal == 2 || u8SigVal == 3)
                {
                    if (Battery == nIND_Hide)
                    {
                        Battery = nIND_Charging;
                    }
                }
                else if(u8SigVal == 4)
                {
                    if (Battery == nIND_Hide)
                    {
                        Battery = nIND_ChargeFinished;
                    }
                }
                else if(u8SigVal == 6 && (Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3))
                {
                    if (Battery == nIND_Hide)
                    {
                        Battery = nIND_Charging;
                    }
                }
                else if(u8SigVal == 7 && (Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3))
                {
                    if (Battery == nIND_Hide)
                    {
                        Battery = nIND_Charging;
                    }
                }
            }
        }

        /*预约充电条件判断*/
        if((IlGetTBOX_ChargeModeStsRxTimeout() == FALSE)
            &&(IlGetRxTBOX_ChargeModeSts() == 2))
        {
            if((bCc2_ConnectStsRxTimeout == FALSE)
                 &&(u16Cc2_ConnectSts == 1))
            {
                if(bPackChargingStsRxTimeout == FALSE)
                {
                    if (u16PackChargingSts == 0)
                    {
                        if (Battery == nIND_Hide)
                        {
                            Battery = nIND_ReserCharging;
                        }
                    }
                    else
                    {
                        u8SigVal = Rte_FISD_u16GetPackChargingSts();
                        if(u8SigVal == 0 && (Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2))
                        {
                            Battery = nIND_Waiting;
                        }
                        else if(u8SigVal == 1)
                        {
                            if (Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3)
                            {
                                Battery = nIND_PHEV3_Heating;
                            }
                            else
                            {
                                Battery = nIND_Heating;
                            }
                        }
                        else if(u8SigVal == 2
                            || u8SigVal == 3
                            || u8SigVal == 6
                            || u8SigVal == 7
                            || u8SigVal == 8)
                        {
                            Battery = nIND_Charging;
                        }
                        else if(u8SigVal == 4)
                        {
                            Battery = nIND_ChargeFinished;
                        }      
                    }
                }
             }
             else if((Rte_FISD_boGetCc_ConnectStsTimeout() == FALSE)
                 &&(Rte_FISD_u16GetCc_ConnectSts() == 1))
             {
                if(IlGetBMSH_PackChargingThermalStRxTimeout() == FALSE)
                {
                    if(IlGetRxBMSH_PackChargingThermalSt() == 0)
                    {
                        if (Battery == nIND_Hide)
                        {
                            Battery = nIND_ReserCharging;
                        }
                    }
                    else
                    {
                        u8SigVal = IlGetRxBMSH_PackChargingThermalSt();
                        if(u8SigVal == 0 && (Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_2))
                        {
                            if (Battery == nIND_Hide)
                            {
                                Battery = nIND_Waiting;
                            }
                        }
                        else if(u8SigVal == 1)
                        {
                            if (Battery == nIND_Hide)
                            {   
                                if (Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3)
                                {
                                    Battery = nIND_PHEV3_Heating;
                                }
                                else
                                {
                                    Battery = nIND_Heating;
                                }                        
                            }
                        }
                        else if(u8SigVal == 2 || u8SigVal == 3)
                        {
                            if (Battery == nIND_Hide)
                            {
                                Battery = nIND_Charging;
                            }
                        }
                        else if(u8SigVal == 4)
                        {
                            if (Battery == nIND_Hide)
                            {
                                Battery = nIND_ChargeFinished;
                            }
                        }
                        else if(u8SigVal == 6 && (Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3))
                        {
                            if (Battery == nIND_Hide)
                            {
                                Battery = nIND_Charging;
                            }
                        }
                        else if(u8SigVal == 7 && (Rte_FISD_u8GetCONFIG_PHEV_TYPE() == nIND_PHEV_TYPE_3))
                        {
                            if (Battery == nIND_Hide)
                            {
                                Battery = nIND_Charging;
                            }
                        }
                    }
                }
             }

        }

        if(Rte_FISD_boGetCc_ConnectStsTimeout() == FALSE)
        {
            if(Rte_FISD_u16GetCc_ConnectSts() == 2)
            {
                u8SigVal = Rte_FISD_u16GetOBC_ChgSts_PHEV();
                if(Rte_FISD_boGetOBC_ChgSts_PHEVTimeout() == FALSE)
                {
                    if(u8SigVal == 4)
                    {
                        if (Battery == nIND_Hide)
                        {
                            Battery = nIND_DisCharging;
                        }
                    }
                    else if(u8SigVal == 5)
                    {
                        if (Battery == nIND_Hide)
                        {
                            Battery = nIND_DisChargeFinished;
                        }
                    }
                }            
            }
        }
    }

	return Battery;
}

