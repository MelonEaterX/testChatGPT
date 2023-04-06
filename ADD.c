
static void LED_Ready_Process(void)
{
    	if(FALSE == RxSigFullOnLEDSts[LEDSig_HCU_0C9h_PrplsnSysAtv].TimeoutFlag)
    	{
        	if (1 == RxSigFullOnLEDSts[LEDSig_HCU_0C9h_PrplsnSysAtv].SignalData.Data8U)
        	{
                	LED_SetOn(LED_PowerStatusReady);
                }
                else
                {
                	LED_SetOff(LED_PowerStatusReady);
                }
        }
        else
        {
        	LED_SetOff(LED_PowerStatusReady);
        }
}
