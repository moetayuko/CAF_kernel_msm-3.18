
#if CONFIG_BATTERY_STC3117

int null_fn(void)
{
	return 0;                // for discharging status
}

int Temperature_fn(void)
{
	return (25);
}

static struct stc311x_platform_data stc3117_data = {
	.battery_online = NULL,
	.charger_online = null_fn, 		// used in stc311x_get_status()
	.charger_enable = null_fn,		// used in stc311x_get_status()
	.power_supply_register = NULL,
	.power_supply_unregister = NULL,

	//-------------------------------------------------------------------------
	
	//Battery specific, Mandatory:
	.Cnom = 1500,       /* nominal battery capacity [in mAh] */
	.Rint = 200,		/* nominal battery internal impedance [mOhms]*/
	.CC_cnf = 303,      /* nominal CC_CNF (Coulomb mode Conf), computed from formula */
	                    //REG_CC_CNF: (Rsense x Nominal battery capacity) / 49.556
	.VM_cnf = 307,      /* nominal VM_CNF (Voltage Mode Conf), computed from formula*/
	                    //REG_VM_CNF: (Internal battery impedance x Nominal battery capacity) / 977.78
	
	//Hardware specific, Mandatory:
	.Rsense = 10,       /* sense resistor [mOhms]*/
	
	//-------------------------------------------------------------------------
	
	//Gas gauge specific, Default:
	.Vmode= 0,       /* REG_MODE, BIT_VMODE 1=Voltage mode, 0=mixed mode */
	.Alm_SOC = 10,      /* SOC alm level %*/
	.Alm_Vbat = 3600,   /* Vbat alm level mV*/
	.RelaxCurrent = 150, /* current for relaxation in mA (< C/20) */
	.Adaptive = 1,     /* 1=Adaptive mode enabled, 0=Adaptive mode disabled */

	//Battery specific, default:
	.CapDerating[6] = 0,   /* capacity de-rating in 0.1%, for temp = -20°C */
	.CapDerating[5] = 0,   /* capacity de-rating in 0.1%, for temp = -10°C */
	.CapDerating[4] = 0,   /* capacity de-rating in 0.1%, for temp = 0°C */
	.CapDerating[3] = 0,   /* capacity de-rating in 0.1%, for temp = 10°C */
	.CapDerating[2] = 0,   /* capacity de-rating in 0.1%, for temp = 25°C */
	.CapDerating[1] = 0,   /* capacity de-rating in 0.1%, for temp = 40°C */
	.CapDerating[0] = 0,   /* capacity de-rating in 0.1%, for temp = 60°C */

	.SOCValue[0] = 0x00,    /* SOC=0% - default SOC axis for battery OCV curve */
	.SOCValue[1] = 0x06,    /* SOC=3% - default SOC axis for battery OCV curve */
	.SOCValue[2] = 0x0C,    /* SOC=6% - default SOC axis for battery OCV curve */
	.SOCValue[3] = 0x14,    /* SOC=10% - default SOC axis for battery OCV curve */
	.SOCValue[4] = 0x1E,    /* SOC=15% - default SOC axis for battery OCV curve */
	.SOCValue[5] = 0x28,    /* SOC=20% - default SOC axis for battery OCV curve */
	.SOCValue[6] = 0x32,    /* SOC=25% - default SOC axis for battery OCV curve */
	.SOCValue[7] = 0x3C,    /* SOC=30% - default SOC axis for battery OCV curve */
	.SOCValue[8] = 0x50,    /* SOC=40% - default SOC axis for battery OCV curve */
	.SOCValue[9] = 0x64,    /* SOC=50% - default SOC axis for battery OCV curve */
	.SOCValue[10] = 0x78,    /* SOC=60% - default SOC axis for battery OCV curve */
	.SOCValue[11] = 0x82,    /* SOC=65% - default SOC axis for battery OCV curve */
	.SOCValue[12] = 0x8C,    /* SOC=70% - default SOC axis for battery OCV curve */
	.SOCValue[13] = 0xA0,    /* SOC=80% - default SOC axis for battery OCV curve */
	.SOCValue[14] = 0xB4,    /* SOC=90% - default SOC axis for battery OCV curve */
	.SOCValue[15] = 0xC8,    /* SOC=100% - default SOC axis for battery OCV curve */
	
	.OCVValue[0] = 3300,    /* default battery OCV curve (at 0%) : OCV=3.3V */
	.OCVValue[1] = 3541,    /* default battery OCV curve (at 3%) : OCV=3.54V */
	.OCVValue[2] = 3618,    /* default battery OCV curve (at x%) : OCV=...V */
	.OCVValue[3] = 3658,    /* default battery OCV curve (at x%) : OCV=...V */
	.OCVValue[4] = 3695,    /* default battery OCV curve (at x%) : OCV=...V */
	.OCVValue[5] = 3721,    /* default battery OCV curve (at x%) : OCV=...V */
	.OCVValue[6] = 3747,    /* default battery OCV curve (at x%) : OCV=...V */
	.OCVValue[7] = 3761,    /* default battery OCV curve (at x%) : OCV=...V */
	.OCVValue[8] = 3778,    /* default battery OCV curve (at x%) : OCV=...V */
	.OCVValue[9] = 3802,    /* default battery OCV curve (at x%) : OCV=...V */
	.OCVValue[10]= 3863,    /* default battery OCV curve (at x%) : OCV=...V */
	.OCVValue[11]= 3899,    /* default battery OCV curve (at x%) : OCV=...V */
	.OCVValue[12]= 3929,    /* default battery OCV curve (at x%) : OCV=...V */
	.OCVValue[13]= 3991,    /* default battery OCV curve (at x%) : OCV=...V */
	.OCVValue[14]= 4076,    /* default battery OCV curve (at 90%) : OCV=4.076V */
	.OCVValue[15]= 4176,    /* default battery OCV curve (at 100%) : OCV=4.176V */

	
	//-------------------------------------------------------------------------
	//Platform specific:
	
	/* if the application temperature data is preferred than the STC3117 temperature */
	.ExternalTemperature = Temperature_fn, /*External temperature function, return °C*/
	.ForceExternalTemperature = 0, /* 1=External temperature, 0=STC3117 temperature */

};
#endif


static struct i2c_board_info __initdata beagle_i2c2_boardinfo[] = {

#if CONFIG_BATTERY_STC3117
	{
		I2C_BOARD_INFO("stc3117", 0x70),
			.platform_data = &stc3117_data,
	},
#endif

};
