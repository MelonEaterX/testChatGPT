static void LED_BatteryLow_Process(void)
{
    	if(MmiDisplayBattery_GetLowBattery())
    	{
        	LED_SetOn(LED_SOCLow);
		if(LED_CHECK_END == LEDCheck_GetState())
		{
			WarnMgr_Add(Warn_BatteryLow);
			MMIBeep_LCDWarnBeepOn(kLCDWarn_BatteryLow);
		}
    	}
    	else
    	{
        	LED_SetOff(LED_SOCLow);
		WarnMgr_Del(Warn_BatteryLow);
		MMIBeep_LCDWarnBeepOff(kLCDWarn_BatteryLow);
    	}
}
