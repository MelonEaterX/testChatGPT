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
