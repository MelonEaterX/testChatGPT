	//SCURemind_Init();
    if(0 ==CFG_SCU)//怀挡
    {
        LED_SCURemind();
        SCURemindBeepPaidangMask = 0;
    }
    else//排挡
    {
        LED_SCURemindPD();
        SCURemindBeepMask = 0;
    }


static void LED_SCURemind(void)
{
	VUINT8 flag;
    WarnMgr_Del(Warn_PressunlockbuttonforrangechangePAIDANG_SCU1);
    WarnMgr_Del(Warn_PressbrakepadelforrangechangePAIDANG_SCU2);
    WarnMgr_Del(Warn_SpeedtoohighforrangechangePAIDANG_SCU3);
    WarnMgr_Del(Warn_EnginespeedtoohighforrangechangePAIDANG_SCU4);
    WarnMgr_Del(Warn_ParkrangeappliedPAIDANG_SCU5);
    WarnMgr_Del(Warn_ForwardchangetokeepneutralrangePAIDANG_SCU6);
    WarnMgr_Del(Warn_NeutralrangemodeappliedkeepsafePAIDANG_SCU7);
    WarnMgr_Del(Warn_ETRSfaultneedrepairPAIDANG_SCU8);
    WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediatelyPAIDANG_SCU9);
	if(FALSE == RxSigFullOnLEDSts[LEDSig_SCU_09Dh_SCURequestToIPK].TimeoutFlag)
	{
		flag = SCURemindBeepMask;
		if(LED_CHECK_END == LEDCheck_GetState())
		{
       		switch(RxSigFullOnLEDSts[LEDSig_SCU_09Dh_SCURequestToIPK].SignalData.Data8U)
	       	{
	            		case SCU_Pressunlockbuttonforrangechange:
	           		{
	                		WarnMgr_Add(Warn_Pressunlockbuttonforrangechange_SCU1);
					WarnMgr_Del(Warn_Pressbrakepadelforrangechange_SCU2);
					WarnMgr_Del(Warn_Speedtoohighforrangechange_SCU3);
					WarnMgr_Del(Warn_Enginespeedtoohighforrangechange_SCU4);
					WarnMgr_Del(Warn_Parkrangeapplied_SCU5);
					WarnMgr_Del(Warn_Forwardchangetokeepneutralrange_SCU6);
					WarnMgr_Del(Warn_Neutralrangemodeappliedkeepsafe_SCU7);
					WarnMgr_Del(Warn_ETRSfaultneedrepair_SCU8);
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediately_SCU9);
			              MMIBeep_LCDWarnBeepOn(kLCDWarn_SCUPressunlockbuttonforrangechange);
				       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
					MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
	            		}
				break;
				
	            		case SCU_Pressbrakepadelforrangechange:
		            	{
					WarnMgr_Add(Warn_Pressbrakepadelforrangechange_SCU2);
	                		WarnMgr_Del(Warn_Pressunlockbuttonforrangechange_SCU1);		
					WarnMgr_Del(Warn_Speedtoohighforrangechange_SCU3);
					WarnMgr_Del(Warn_Enginespeedtoohighforrangechange_SCU4);
					WarnMgr_Del(Warn_Parkrangeapplied_SCU5);
					WarnMgr_Del(Warn_Forwardchangetokeepneutralrange_SCU6);
					WarnMgr_Del(Warn_Neutralrangemodeappliedkeepsafe_SCU7);
					WarnMgr_Del(Warn_ETRSfaultneedrepair_SCU8);
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediately_SCU9);

					MMIBeep_LCDWarnBeepOn(kLCDWarn_SCUPressbrakepadelforrangechange);
			              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);     
					MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
	            		}
				break;

				case SCU_Speedtoohighforrangechange:
		            	{
					WarnMgr_Add(Warn_Speedtoohighforrangechange_SCU3);
	                		WarnMgr_Del(Warn_Pressunlockbuttonforrangechange_SCU1);
					WarnMgr_Del(Warn_Pressbrakepadelforrangechange_SCU2);
					WarnMgr_Del(Warn_Enginespeedtoohighforrangechange_SCU4);
					WarnMgr_Del(Warn_Parkrangeapplied_SCU5);
					WarnMgr_Del(Warn_Forwardchangetokeepneutralrange_SCU6);
					WarnMgr_Del(Warn_Neutralrangemodeappliedkeepsafe_SCU7);
					WarnMgr_Del(Warn_ETRSfaultneedrepair_SCU8);
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediately_SCU9);

					MMIBeep_LCDWarnBeepOn(kLCDWarn_SCUSpeedtoohighforrangechange );
			              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
				       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
	            		}
				break;

				case SCU_Enginespeedtoohighforrangechange:
		            	{
					WarnMgr_Add(Warn_Enginespeedtoohighforrangechange_SCU4);
	                		WarnMgr_Del(Warn_Pressunlockbuttonforrangechange_SCU1);
					WarnMgr_Del(Warn_Pressbrakepadelforrangechange_SCU2);
					WarnMgr_Del(Warn_Speedtoohighforrangechange_SCU3);
					WarnMgr_Del(Warn_Parkrangeapplied_SCU5);
					WarnMgr_Del(Warn_Forwardchangetokeepneutralrange_SCU6);
					WarnMgr_Del(Warn_Neutralrangemodeappliedkeepsafe_SCU7);
					WarnMgr_Del(Warn_ETRSfaultneedrepair_SCU8);

					MMIBeep_LCDWarnBeepOn (kLCDWarn_SCUEnginespeedtoohighforrangechange );
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediately_SCU9);
			              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
				       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
					MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
			       }
				break;

				case SCU_Parkrangeapplied:
	           		{
					WarnMgr_Add(Warn_Parkrangeapplied_SCU5);   //显示怀挡，无文字提示
	                		WarnMgr_Del(Warn_Pressunlockbuttonforrangechange_SCU1);
					WarnMgr_Del(Warn_Pressbrakepadelforrangechange_SCU2);
					WarnMgr_Del(Warn_Speedtoohighforrangechange_SCU3);
					WarnMgr_Del(Warn_Enginespeedtoohighforrangechange_SCU4);
					
					WarnMgr_Del(Warn_Forwardchangetokeepneutralrange_SCU6);
					WarnMgr_Del(Warn_Neutralrangemodeappliedkeepsafe_SCU7);
					WarnMgr_Del(Warn_ETRSfaultneedrepair_SCU8);
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediately_SCU9);

					MMIBeep_LCDWarnBeepOn (kLCDWarn_SCUParkrangeapplied );
			              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
				       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
					MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );	
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
	            		}
				break;
				
	            		case SCU_Forwardchangetokeepneutralrange:
		            	{
					WarnMgr_Add(Warn_Forwardchangetokeepneutralrange_SCU6);
	                		WarnMgr_Del(Warn_Pressunlockbuttonforrangechange_SCU1);
					WarnMgr_Del(Warn_Pressbrakepadelforrangechange_SCU2);
					WarnMgr_Del(Warn_Speedtoohighforrangechange_SCU3);
					WarnMgr_Del(Warn_Enginespeedtoohighforrangechange_SCU4);
					WarnMgr_Del(Warn_Parkrangeapplied_SCU5);				
					WarnMgr_Del(Warn_Neutralrangemodeappliedkeepsafe_SCU7);
					WarnMgr_Del(Warn_ETRSfaultneedrepair_SCU8);
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediately_SCU9);

					MMIBeep_LCDWarnBeepOn (kLCDWarn_SCUForwardchangetokeepneutralrange );
			              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
				       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
					MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );	
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
	            		}
				break;

				case SCU_Neutralrangemodeappliedkeepsafe:
	           		{
					WarnMgr_Add(Warn_Neutralrangemodeappliedkeepsafe_SCU7);
	                		WarnMgr_Del(Warn_Pressunlockbuttonforrangechange_SCU1);
					WarnMgr_Del(Warn_Pressbrakepadelforrangechange_SCU2);
					WarnMgr_Del(Warn_Speedtoohighforrangechange_SCU3);
					WarnMgr_Del(Warn_Enginespeedtoohighforrangechange_SCU4);
					WarnMgr_Del(Warn_Parkrangeapplied_SCU5);
					WarnMgr_Del(Warn_Forwardchangetokeepneutralrange_SCU6);			
					WarnMgr_Del(Warn_ETRSfaultneedrepair_SCU8);
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediately_SCU9);

					MMIBeep_LCDWarnBeepOn (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
			              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
				       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
					MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );	            		
				}
				break;
				
	            		case SCU_ETRSfaultneedrepair:
		            	{
					WarnMgr_Add(Warn_ETRSfaultneedrepair_SCU8);
	                		WarnMgr_Del(Warn_Pressunlockbuttonforrangechange_SCU1);
					WarnMgr_Del(Warn_Pressbrakepadelforrangechange_SCU2);
					WarnMgr_Del(Warn_Speedtoohighforrangechange_SCU3);
					WarnMgr_Del(Warn_Enginespeedtoohighforrangechange_SCU4);
					WarnMgr_Del(Warn_Parkrangeapplied_SCU5);
					WarnMgr_Del(Warn_Forwardchangetokeepneutralrange_SCU6);
					WarnMgr_Del(Warn_Neutralrangemodeappliedkeepsafe_SCU7);				
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediately_SCU9);
					MMIBeep_LCDWarnBeepOn (kLCDWarn_SCUETRSfaultneedrepair );
			              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
				       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
					MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );				
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
	            		}
				break;

				case SCU_ETRSfaultstopsafelyrepairimmediately:
		            	{
						WarnMgr_Add(Warn_ETRSfaultstopsafelyrepairimmediately_SCU9);
	                			WarnMgr_Del(Warn_Pressunlockbuttonforrangechange_SCU1);
						WarnMgr_Del(Warn_Pressbrakepadelforrangechange_SCU2);
						WarnMgr_Del(Warn_Speedtoohighforrangechange_SCU3);
						WarnMgr_Del(Warn_Enginespeedtoohighforrangechange_SCU4);
						WarnMgr_Del(Warn_Parkrangeapplied_SCU5);
						WarnMgr_Del(Warn_Forwardchangetokeepneutralrange_SCU6);
						WarnMgr_Del(Warn_Neutralrangemodeappliedkeepsafe_SCU7);
						WarnMgr_Del(Warn_ETRSfaultneedrepair_SCU8);	
						//MMIBeep_LCDWarnBeepOn (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
				              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
					       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
						MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
						MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
						MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );
						MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
						MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
						MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );	
	            		}
				break;

				default:
					WarnMgr_Del(Warn_Pressunlockbuttonforrangechange_SCU1);
					WarnMgr_Del(Warn_Pressbrakepadelforrangechange_SCU2);
					WarnMgr_Del(Warn_Speedtoohighforrangechange_SCU3);
					WarnMgr_Del(Warn_Enginespeedtoohighforrangechange_SCU4);
					WarnMgr_Del(Warn_Parkrangeapplied_SCU5);
					WarnMgr_Del(Warn_Forwardchangetokeepneutralrange_SCU6);
					WarnMgr_Del(Warn_Neutralrangemodeappliedkeepsafe_SCU7);
					WarnMgr_Del(Warn_ETRSfaultneedrepair_SCU8);
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediately_SCU9);
			              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
				       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
					MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
				break;
	       	}
		}

		if(SCU_ETRSfaultstopsafelyrepairimmediately==RxSigFullOnLEDSts[LEDSig_SCU_09Dh_SCURequestToIPK].SignalData.Data8U)
		{
			SCURemindBeepMask |= 0x01;
		}
		else
		{
			SCURemindBeepMask &= ~0x01;
		}
		
     		if(SCURemindBeepMask > flag)
        	{
            		MMIBeepOn(SPEAKER_SCU);
            		Chime(CHIME_SCU);
        	}
        	else if(0 == SCURemindBeepMask)
        	{
            		MMIBeepOff(SPEAKER_SCU);
            		Chime_Off(CHIME_SCU);
        	}
        	else
        	{
        	}	
    	}
    	else
    	{
             	WarnMgr_Del(Warn_Pressunlockbuttonforrangechange_SCU1);
		WarnMgr_Del(Warn_Pressbrakepadelforrangechange_SCU2);
		WarnMgr_Del(Warn_Speedtoohighforrangechange_SCU3);
		WarnMgr_Del(Warn_Enginespeedtoohighforrangechange_SCU4);
		WarnMgr_Del(Warn_Parkrangeapplied_SCU5);
		WarnMgr_Del(Warn_Forwardchangetokeepneutralrange_SCU6);
		WarnMgr_Del(Warn_Neutralrangemodeappliedkeepsafe_SCU7);
		WarnMgr_Del(Warn_ETRSfaultneedrepair_SCU8);
		WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediately_SCU9);
              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
	       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
		MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUSpeedtoohighforrangechange );
		MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
		MMIBeep_LCDWarnBeepOff( kLCDWarn_SCUParkrangeapplied );
		MMIBeep_LCDWarnBeepOff( kLCDWarn_SCUForwardchangetokeepneutralrange );
		MMIBeep_LCDWarnBeepOff( kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
		MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
		//MMIBeep_LCDWarnBeepOff( kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
		MMIBeepOff(SPEAKER_SCU);
            	Chime_Off(CHIME_SCU);
		SCURemindBeepMask &= ~0x01;
    	}
}


static void LED_SCURemindPD(void)
{
	VUINT8 flag;
    WarnMgr_Del(Warn_Pressunlockbuttonforrangechange_SCU1);
    WarnMgr_Del(Warn_Pressbrakepadelforrangechange_SCU2);
    WarnMgr_Del(Warn_Speedtoohighforrangechange_SCU3);
    WarnMgr_Del(Warn_Enginespeedtoohighforrangechange_SCU4);
    WarnMgr_Del(Warn_Parkrangeapplied_SCU5);
    WarnMgr_Del(Warn_Forwardchangetokeepneutralrange_SCU6);
    WarnMgr_Del(Warn_Neutralrangemodeappliedkeepsafe_SCU7);
    WarnMgr_Del(Warn_ETRSfaultneedrepair_SCU8);
    WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediately_SCU9);
	if(FALSE == RxSigFullOnLEDSts[LEDSig_SCU_09Dh_SCURequestToIPK].TimeoutFlag)
	{
		flag = SCURemindBeepPaidangMask;
		if(LED_CHECK_END == LEDCheck_GetState())
		{
       		switch(RxSigFullOnLEDSts[LEDSig_SCU_09Dh_SCURequestToIPK].SignalData.Data8U)
	       	{
	            		case SCU_Pressunlockbuttonforrangechange:
	           		{
	                		WarnMgr_Add(Warn_PressunlockbuttonforrangechangePAIDANG_SCU1);
					WarnMgr_Del(Warn_PressbrakepadelforrangechangePAIDANG_SCU2);
					WarnMgr_Del(Warn_SpeedtoohighforrangechangePAIDANG_SCU3);
					WarnMgr_Del(Warn_EnginespeedtoohighforrangechangePAIDANG_SCU4);
					WarnMgr_Del(Warn_ParkrangeappliedPAIDANG_SCU5);
					WarnMgr_Del(Warn_ForwardchangetokeepneutralrangePAIDANG_SCU6);
					WarnMgr_Del(Warn_NeutralrangemodeappliedkeepsafePAIDANG_SCU7);
					WarnMgr_Del(Warn_ETRSfaultneedrepairPAIDANG_SCU8);
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediatelyPAIDANG_SCU9);
			              MMIBeep_LCDWarnBeepOn(kLCDWarn_SCUPressunlockbuttonforrangechange);
				       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
					MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
	            		}
				break;
				
	            		case SCU_Pressbrakepadelforrangechange:
		            	{
					WarnMgr_Add(Warn_PressbrakepadelforrangechangePAIDANG_SCU2);
	                		WarnMgr_Del(Warn_PressunlockbuttonforrangechangePAIDANG_SCU1);		
					WarnMgr_Del(Warn_SpeedtoohighforrangechangePAIDANG_SCU3);
					WarnMgr_Del(Warn_EnginespeedtoohighforrangechangePAIDANG_SCU4);
					WarnMgr_Del(Warn_ParkrangeappliedPAIDANG_SCU5);
					WarnMgr_Del(Warn_ForwardchangetokeepneutralrangePAIDANG_SCU6);
					WarnMgr_Del(Warn_NeutralrangemodeappliedkeepsafePAIDANG_SCU7);
					WarnMgr_Del(Warn_ETRSfaultneedrepairPAIDANG_SCU8);
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediatelyPAIDANG_SCU9);

					MMIBeep_LCDWarnBeepOn(kLCDWarn_SCUPressbrakepadelforrangechange);
			              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);     
					MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
	            		}
				break;

				case SCU_Speedtoohighforrangechange:
		            	{
					WarnMgr_Add(Warn_SpeedtoohighforrangechangePAIDANG_SCU3);
	                		WarnMgr_Del(Warn_PressunlockbuttonforrangechangePAIDANG_SCU1);
					WarnMgr_Del(Warn_PressbrakepadelforrangechangePAIDANG_SCU2);
					WarnMgr_Del(Warn_EnginespeedtoohighforrangechangePAIDANG_SCU4);
					WarnMgr_Del(Warn_ParkrangeappliedPAIDANG_SCU5);
					WarnMgr_Del(Warn_ForwardchangetokeepneutralrangePAIDANG_SCU6);
					WarnMgr_Del(Warn_NeutralrangemodeappliedkeepsafePAIDANG_SCU7);
					WarnMgr_Del(Warn_ETRSfaultneedrepairPAIDANG_SCU8);
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediatelyPAIDANG_SCU9);

					MMIBeep_LCDWarnBeepOn(kLCDWarn_SCUSpeedtoohighforrangechange );
			              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
				       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
	            		}
				break;

				case SCU_Enginespeedtoohighforrangechange:
		            	{
					WarnMgr_Add(Warn_EnginespeedtoohighforrangechangePAIDANG_SCU4);
	                		WarnMgr_Del(Warn_PressunlockbuttonforrangechangePAIDANG_SCU1);
					WarnMgr_Del(Warn_PressbrakepadelforrangechangePAIDANG_SCU2);
					WarnMgr_Del(Warn_SpeedtoohighforrangechangePAIDANG_SCU3);
					WarnMgr_Del(Warn_ParkrangeappliedPAIDANG_SCU5);
					WarnMgr_Del(Warn_ForwardchangetokeepneutralrangePAIDANG_SCU6);
					WarnMgr_Del(Warn_NeutralrangemodeappliedkeepsafePAIDANG_SCU7);
					WarnMgr_Del(Warn_ETRSfaultneedrepairPAIDANG_SCU8);

					MMIBeep_LCDWarnBeepOn (kLCDWarn_SCUEnginespeedtoohighforrangechange );
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediately_SCU9);
			              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
				       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
					MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
			       }
				break;

				case SCU_Parkrangeapplied:
	           		{
					WarnMgr_Add(Warn_ParkrangeappliedPAIDANG_SCU5);   //显示怀挡，无文字提示
	                		WarnMgr_Del(Warn_PressunlockbuttonforrangechangePAIDANG_SCU1);
					WarnMgr_Del(Warn_PressbrakepadelforrangechangePAIDANG_SCU2);
					WarnMgr_Del(Warn_SpeedtoohighforrangechangePAIDANG_SCU3);
					WarnMgr_Del(Warn_EnginespeedtoohighforrangechangePAIDANG_SCU4);
					
					WarnMgr_Del(Warn_ForwardchangetokeepneutralrangePAIDANG_SCU6);
					WarnMgr_Del(Warn_NeutralrangemodeappliedkeepsafePAIDANG_SCU7);
					WarnMgr_Del(Warn_ETRSfaultneedrepairPAIDANG_SCU8);
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediatelyPAIDANG_SCU9);

					MMIBeep_LCDWarnBeepOn (kLCDWarn_SCUParkrangeapplied );
			              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
				       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
					MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );	
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
	            		}
				break;
				
	            		case SCU_Forwardchangetokeepneutralrange:
		            	{
					WarnMgr_Add(Warn_ForwardchangetokeepneutralrangePAIDANG_SCU6);
	                		WarnMgr_Del(Warn_PressunlockbuttonforrangechangePAIDANG_SCU1);
					WarnMgr_Del(Warn_PressbrakepadelforrangechangePAIDANG_SCU2);
					WarnMgr_Del(Warn_SpeedtoohighforrangechangePAIDANG_SCU3);
					WarnMgr_Del(Warn_EnginespeedtoohighforrangechangePAIDANG_SCU4);
					WarnMgr_Del(Warn_ParkrangeappliedPAIDANG_SCU5);				
					WarnMgr_Del(Warn_NeutralrangemodeappliedkeepsafePAIDANG_SCU7);
					WarnMgr_Del(Warn_ETRSfaultneedrepairPAIDANG_SCU8);
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediatelyPAIDANG_SCU9);

					MMIBeep_LCDWarnBeepOn (kLCDWarn_SCUForwardchangetokeepneutralrange );
			              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
				       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
					MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );	
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
	            		}
				break;

				case SCU_Neutralrangemodeappliedkeepsafe:
	           		{
					WarnMgr_Add(Warn_NeutralrangemodeappliedkeepsafePAIDANG_SCU7);
	                		WarnMgr_Del(Warn_PressunlockbuttonforrangechangePAIDANG_SCU1);
					WarnMgr_Del(Warn_PressbrakepadelforrangechangePAIDANG_SCU2);
					WarnMgr_Del(Warn_SpeedtoohighforrangechangePAIDANG_SCU3);
					WarnMgr_Del(Warn_EnginespeedtoohighforrangechangePAIDANG_SCU4);
					WarnMgr_Del(Warn_ParkrangeappliedPAIDANG_SCU5);
					WarnMgr_Del(Warn_ForwardchangetokeepneutralrangePAIDANG_SCU6);			
					WarnMgr_Del(Warn_ETRSfaultneedrepairPAIDANG_SCU8);
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediatelyPAIDANG_SCU9);

					MMIBeep_LCDWarnBeepOn (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
			              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
				       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
					MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );	            		
				}
				break;
				
	            		case SCU_ETRSfaultneedrepair:
		            	{
					WarnMgr_Add(Warn_ETRSfaultneedrepairPAIDANG_SCU8);
	                		WarnMgr_Del(Warn_PressunlockbuttonforrangechangePAIDANG_SCU1);
					WarnMgr_Del(Warn_PressbrakepadelforrangechangePAIDANG_SCU2);
					WarnMgr_Del(Warn_SpeedtoohighforrangechangePAIDANG_SCU3);
					WarnMgr_Del(Warn_EnginespeedtoohighforrangechangePAIDANG_SCU4);
					WarnMgr_Del(Warn_ParkrangeappliedPAIDANG_SCU5);
					WarnMgr_Del(Warn_ForwardchangetokeepneutralrangePAIDANG_SCU6);
					WarnMgr_Del(Warn_NeutralrangemodeappliedkeepsafePAIDANG_SCU7);				
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediatelyPAIDANG_SCU9);
					MMIBeep_LCDWarnBeepOn (kLCDWarn_SCUETRSfaultneedrepair );
			              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
				       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
					MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );				
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
	            		}
				break;

				case SCU_ETRSfaultstopsafelyrepairimmediately:
		            	{
						WarnMgr_Add(Warn_ETRSfaultstopsafelyrepairimmediatelyPAIDANG_SCU9);
	                			WarnMgr_Del(Warn_PressunlockbuttonforrangechangePAIDANG_SCU1);
						WarnMgr_Del(Warn_PressbrakepadelforrangechangePAIDANG_SCU2);
						WarnMgr_Del(Warn_SpeedtoohighforrangechangePAIDANG_SCU3);
						WarnMgr_Del(Warn_EnginespeedtoohighforrangechangePAIDANG_SCU4);
						WarnMgr_Del(Warn_ParkrangeappliedPAIDANG_SCU5);
						WarnMgr_Del(Warn_ForwardchangetokeepneutralrangePAIDANG_SCU6);
						WarnMgr_Del(Warn_NeutralrangemodeappliedkeepsafePAIDANG_SCU7);
						WarnMgr_Del(Warn_ETRSfaultneedrepairPAIDANG_SCU8);	
						//MMIBeep_LCDWarnBeepOn (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
				              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
					       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
						MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
						MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
						MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );
						MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
						MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
						MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );	
	            		}
				break;

				default:
					WarnMgr_Del(Warn_PressunlockbuttonforrangechangePAIDANG_SCU1);
					WarnMgr_Del(Warn_PressbrakepadelforrangechangePAIDANG_SCU2);
					WarnMgr_Del(Warn_SpeedtoohighforrangechangePAIDANG_SCU3);
					WarnMgr_Del(Warn_EnginespeedtoohighforrangechangePAIDANG_SCU4);
					WarnMgr_Del(Warn_ParkrangeappliedPAIDANG_SCU5);
					WarnMgr_Del(Warn_ForwardchangetokeepneutralrangePAIDANG_SCU6);
					WarnMgr_Del(Warn_NeutralrangemodeappliedkeepsafePAIDANG_SCU7);
					WarnMgr_Del(Warn_ETRSfaultneedrepairPAIDANG_SCU8);
					WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediatelyPAIDANG_SCU9);
			              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
				       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
					MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUSpeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUParkrangeapplied );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUForwardchangetokeepneutralrange );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
					MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
					//MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
				break;
	       	}
		}

		if(SCU_ETRSfaultstopsafelyrepairimmediately==RxSigFullOnLEDSts[LEDSig_SCU_09Dh_SCURequestToIPK].SignalData.Data8U)
		{
			SCURemindBeepPaidangMask |= 0x01;
		}
		else
		{
			SCURemindBeepPaidangMask &= ~0x01;
		}
		
     		if(SCURemindBeepPaidangMask > flag)
        	{
            		MMIBeepOn(SPEAKER_SCU);
            		Chime(CHIME_SCU);
        	}
        	else if(0 == SCURemindBeepPaidangMask)
        	{
            		MMIBeepOff(SPEAKER_SCU);
            		Chime_Off(CHIME_SCU);
        	}
        	else
        	{
        	}	
    	}
    	else
    	{
             	WarnMgr_Del(Warn_PressunlockbuttonforrangechangePAIDANG_SCU1);
		WarnMgr_Del(Warn_PressbrakepadelforrangechangePAIDANG_SCU2);
		WarnMgr_Del(Warn_SpeedtoohighforrangechangePAIDANG_SCU3);
		WarnMgr_Del(Warn_EnginespeedtoohighforrangechangePAIDANG_SCU4);
		WarnMgr_Del(Warn_ParkrangeappliedPAIDANG_SCU5);
		WarnMgr_Del(Warn_ForwardchangetokeepneutralrangePAIDANG_SCU6);
		WarnMgr_Del(Warn_NeutralrangemodeappliedkeepsafePAIDANG_SCU7);
		WarnMgr_Del(Warn_ETRSfaultneedrepairPAIDANG_SCU8);
		WarnMgr_Del(Warn_ETRSfaultstopsafelyrepairimmediatelyPAIDANG_SCU9);
              MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressunlockbuttonforrangechange);
	       MMIBeep_LCDWarnBeepOff(kLCDWarn_SCUPressbrakepadelforrangechange);
		MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUSpeedtoohighforrangechange );
		MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUEnginespeedtoohighforrangechange );
		MMIBeep_LCDWarnBeepOff( kLCDWarn_SCUParkrangeapplied );
		MMIBeep_LCDWarnBeepOff( kLCDWarn_SCUForwardchangetokeepneutralrange );
		MMIBeep_LCDWarnBeepOff( kLCDWarn_SCUNeutralrangemodeappliedkeepsafe );
		MMIBeep_LCDWarnBeepOff (kLCDWarn_SCUETRSfaultneedrepair );
		//MMIBeep_LCDWarnBeepOff( kLCDWarn_SCUETRSfaultstopsafelyrepairimmediately );
		MMIBeepOff(SPEAKER_SCU);
            	Chime_Off(CHIME_SCU);
		SCURemindBeepPaidangMask &= ~0x01;
    	}
}

