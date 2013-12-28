  //=========================================================================
  //	21.2.2013 - ...
  //	Jan Ondras + father + anybody who helped and advised
  //	Robot-Warehouseman for Istrobot
  //========================================================================
  // all my functions, variables begins with j...
  // 0 % duty cycle means max. speed
  // Initially, robot moves forward: (A is left side, B is right side of the robot)
  // A:	GPIO2 - LOW,	GPIO3 - HIGH
  // B:	GPIO6 - HIGH,	GPIO7 - LOW
  
  // ADC:
  // ID - 0 - B0 - AdcResult.ADCRESULT8
  // ID - 1 - B4 - AdcResult.ADCRESULT12
  // ID - 2 - A5 - AdcResult.ADCRESULT5
  // ID - 3 - B2 - AdcResult.ADCRESULT10
  
  //	AdcResults[0] = AdcResult.ADCRESULT0;
  //	AdcResults[1] = AdcResult.ADCRESULT1;
  //	AdcResults[2] = AdcResult.ADCRESULT2;
  //	AdcResults[3] = 0;						// ADC-A3 NOT AVAILABLE on controlSTICK
  //	AdcResults[4] = AdcResult.ADCRESULT4;
  //	AdcResults[5] = AdcResult.ADCRESULT5;
  //	AdcResults[6] = AdcResult.ADCRESULT6;
  //	AdcResults[7] = 0;						// ADC-A7 NOT AVAILABLE on controlSTICK
  //	AdcResults[8] = AdcResult.ADCRESULT8;
  //	AdcResults[9] = AdcResult.ADCRESULT9;
  //	AdcResults[10] = AdcResult.ADCRESULT10;
  //	AdcResults[11] = 0;						// ADC-B3 NOT AVAILABLE on controlSTICK
  //	AdcResults[12] = AdcResult.ADCRESULT12;
  //	AdcResults[13] = 0;						// ADC-B5 NOT AVAILABLE on controlSTICK
  //	AdcResults[14] = AdcResult.ADCRESULT14;
  //	AdcResults[15] = 0;						// ADC-B7 NOT AVAILABLE on controlSTICK

//----------------------------------------------------------------------------------

#include "PeripheralHeaderIncludes.h"
#include "F2806x_EPwm_defines.h" 	    // useful defines for initialization
																		 

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// FUNCTION PROTOTYPES
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void DeviceInit(void);
void InitFlash(void);
void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// VARIABLE DECLARATIONS - GENERAL
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Used for running BackGround in flash, and ISR in RAM
extern Uint16 RamfuncsLoadStart, RamfuncsLoadEnd, RamfuncsRunStart;

float jTest1, jTest2, jTest0, jTest3;

float jMinVelocity = 0.32;	// 30.00 %
float jMaxVelocity = 0.65;	// 65.00 %

//Uint16 AdcResults[16];
Uint16 jSensMin[4];			// min values of sensors
Uint16 jSensMax[4];			// max values of sensors

Uint16 jPos[3] = {1, 3, 1};				// information about position[x,y] from {1, ... 5} and rotation[r] from {1, ... 4}
										// x coordinate 
										// y coordinate
										// r ...	1 is along the positive x axis
										//			2 is along the positive y axis
										//			3 is along the negative x axis
										//			4 is along the negative y axis

Uint16 duty_cycle_A = 8000;	// Set duty 50% initially
Uint16 duty_cycle_B = 8000;	// Set duty 50% initially

typedef enum {false,true} bool;

// jTime in milliseconds
void jDelay(Uint16 jTime)
{
	Uint16 jCnt = 0;
	for( ; ; )
	{
		if(CpuTimer0Regs.TCR.bit.TIF == 1)
		{
			CpuTimer0Regs.TCR.bit.TIF = 1;	// clear flag
			
			if(jCnt >= jTime)
				break;
				
			jCnt++;
		}
	}	
}

float jSensGetPercent(Uint16 jSensID)
{
	switch(jSensID)
	{
		case 0:
			return ((100.00*(AdcResult.ADCRESULT8 - jSensMin[jSensID]))/(jSensMax[jSensID] - jSensMin[jSensID]));
		case 1:
			return ((100.00*(AdcResult.ADCRESULT12 - jSensMin[jSensID]))/(jSensMax[jSensID] - jSensMin[jSensID]));
		case 2:
			return ((100.00*(AdcResult.ADCRESULT5 - jSensMin[jSensID]))/(jSensMax[jSensID] - jSensMin[jSensID]));
		case 3:
			return ((100.00*(AdcResult.ADCRESULT10 - jSensMin[jSensID]))/(jSensMax[jSensID] - jSensMin[jSensID]));
	}
}

bool jSensRange(Uint16 jSID, float jStartPercent, float jEndPercent)
{
	if( (jSensGetPercent(jSID) >= jStartPercent) && (jSensGetPercent(jSID) <= jEndPercent) )
		return true;
	else
		return false;
}

void jAsetDirection(bool jAforward)
{
	if(jAforward == true)	{
		GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
		GpioDataRegs.GPASET.bit.GPIO3 = 1;
	}
	else	{
		GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
		GpioDataRegs.GPASET.bit.GPIO2 = 1;
	}
}

void jBsetDirection(bool jBforward)
{
	if(jBforward == true)	{
		GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;
		GpioDataRegs.GPASET.bit.GPIO6 = 1;
	}
	else	{
		GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
		GpioDataRegs.GPASET.bit.GPIO7 = 1;
	}
}

void jAsetVelocity(float jAvel)
{
	//EPwm3Regs.CMPA.half.CMPA = (int)(EPwm3Regs.TBPRD * (1 -  (jAvel/100) ));
	EPwm3Regs.CMPA.half.CMPA = (int)(EPwm3Regs.TBPRD*(1 - jMinVelocity - ((jMaxVelocity - jMinVelocity)*(jAvel/100)) ) );
}

void jBsetVelocity(float jBvel)
{
	//EPwm3Regs.CMPB = (int)(EPwm3Regs.TBPRD * (1 -  (jBvel/100) ));
	EPwm3Regs.CMPB = (int)(EPwm3Regs.TBPRD*(1 - jMinVelocity - ((jMaxVelocity - jMinVelocity)*(jBvel/100)) ) );
}

void jABbreak()
{
	GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;
		
	EPwm3Regs.CMPA.half.CMPA = EPwm3Regs.TBPRD;
	EPwm3Regs.CMPB = EPwm3Regs.TBPRD;
}

// velecity parameters are from 0% to 100% = from 0 to 100
// jDuration, jBreakDuration in milliseconds
void jMove(bool jAforward, bool jBforward, float jAvelocity, float jBvelocity, Uint16 jDuration, bool jBreak, Uint16 jBreakDuration)
{
	// direction part
	jAsetDirection(jAforward);
	jBsetDirection(jBforward);
	
	// velocity part
	jAsetVelocity(jAvelocity);
	jBsetVelocity(jBvelocity);
	
	// timing part
	jDelay(jDuration);
	
	// breaking part
	if(jBreak == true)	{
		jABbreak();
		jDelay(jBreakDuration);
	}
}

void jLineFollower(bool jForward)
{
	jAsetDirection(true);
	jBsetDirection(true);
	jAsetVelocity(0);
	jBsetVelocity(0);
	jDelay(350);
	jABbreak();
	
	while( (jSensRange(0, 99.0, 150) == true) && (jSensRange(3, 99.0, 150) == true) )
	{
			jTest1 = jSensGetPercent(1);
	jTest2 = jSensGetPercent(2);
		jTest0 = jSensGetPercent(0);
		jTest3 = jSensGetPercent(3);
		
		if( (jSensRange(1, 99.0, 150) == true) && (jSensRange(2, 99.0, 150) == true) )
		{
			jAsetDirection(true);
			jBsetDirection(true);
			jAsetVelocity(0);
			jBsetVelocity(0);
			jDelay(50);
			jABbreak();
		}
		else
		{
			//jAsetDirection(false);
			//jBsetDirection(false);
			
			//jABbreak();

			if(jSensRange(1, 0, 99.0) == true)
			{
				//while(jSensRange(1, 99, 150) == true);
				
				jAsetDirection(false);
				jBsetDirection(true);				
			}
			else
			{
				//while(jSensRange(2, 99, 150) == true);
				
				jAsetDirection(true);
				jBsetDirection(false);
			}
			jAsetVelocity(50);
			jBsetVelocity(50);
			jDelay(50);
			jABbreak();
		}
	}
	
	jABbreak();
}

void jRotate90(bool jClockwise)
{
	if(jClockwise == true)
		jMove(true, false, 30.00, 30.00, 0, false, 0);
	else
		jMove(false, true, 30.00, 30.00, 0, false, 0);
	
	for( ; ; )
		if( (jSensRange(0, 98.00, 150.00) == true) || (jSensRange(3, 98.00, 150.00) == true))
			break;
			
	for( ; ; )
		if( (jSensRange(0, 0.00, 95.00) == true) || (jSensRange(3, 0.00, 95.00) == true) )
			break;
			
	jABbreak();
	
	// switch status
	if(jClockwise == true)
	{
		if(jPos[2] == 1)
			jPos[2] = 4;
		else
			jPos[2]--;
	}
	else
	{
		if(jPos[2] == 4)
			jPos[2] = 1;
		else
			jPos[2]++;
	}
}

void jRotate180(bool jClockwise)
{
	Uint16 jI = 0;
	
	if(jClockwise == true)
		jMove(true, false, 30.00, 30.00, 0, false, 0);
	else
		jMove(false, true, 30.00, 30.00, 0, false, 0);
	
	for( ; jI < 2 ; jI++ )
	{
		for( ; ; )
			if( (jSensRange(0, 95.00, 150.00) == true) || (jSensRange(3, 95.00, 150.00) == true))
				break;
				
		for( ; ; )
			if( (jSensRange(0, 0.00, 95.00) == true) || (jSensRange(3, 0.00, 95.00) == true) )
				break;
				
		// switch status
		if( (jPos[2] == 1) || (jPos[2] == 2) )
			jPos[2]++;
		else
			jPos[2]--;
	}
			
	jABbreak();
}

void jTransferX(int jCnt)
{
	int jI = 0;
	
	if(jCnt > 0)
	{
		if(jPos[2] == 1)
		{
			for( ; jI < jCnt ; jI++)
			{
				jLineFollower(true);
				jPos[0]++;
			}
		}
		else
		{
			for( ; jI < jCnt ; jI++)
			{
				jLineFollower(false);
				jPos[0]++;
			}
		}
	}
	else if(jCnt < 0)
	{
		if(jPos[2] == 3)
		{
			for(jI = jCnt ; jI < 0 ; jI++)
			{
				jLineFollower(true);
				jPos[0]--;
			}
		}
		else
		{
			for(jI = jCnt ; jI < 0 ; jI++)
			{
				jLineFollower(false);
				jPos[0]--;
			}
		}
	}
}

void jTransferY(int jCnt)
{
	int jI = 0;
	
	if(jCnt > 0)
	{
		if(jPos[2] == 2)
		{
			for( ; jI < jCnt ; jI++)
			{
				jLineFollower(true);
				jPos[1]++;
			}
		}
		else
		{
			for( ; jI < jCnt ; jI++)
			{
				jLineFollower(false);
				jPos[1]++;
			}
		}
	}
	else if(jCnt < 0)
	{
		if(jPos[2] == 4)
		{
			for(jI = jCnt ; jI < 0 ; jI++)
			{
				jLineFollower(true);
				jPos[1]--;
			}
		}
		else
		{
			for(jI = jCnt ; jI < 0 ; jI++)
			{
				jLineFollower(false);
				jPos[1]--;
			}
		}
	}
}

void jTransferQuickest(Uint16 jX, Uint16 jY)
{
	if( (jPos[2] == 1) || (jPos[2] == 3) )
	{
		jTransferX(jX - jPos[0]);
		jRotate90(true);
		jTransferY(jY - jPos[1]);
	}
	else
	{
		jTransferY(jY - jPos[1]);
		jRotate90(true);
		jTransferX(jX - jPos[0]);
	}
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// MAIN CODE - starts here
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void main(void)
{
	Uint16 i = 0;

//=================================
//	INITIALISATION - General
//=================================

	DeviceInit();	// Device Life support & GPIO mux settings 

// Only used if running from FLASH
// Note that the variable FLASH is defined by the compiler with -d FLASH
// (see TwoChannelBuck.pjt file)
#ifdef FLASH		
// Copy time critical code and Flash setup code to RAM
// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
// symbols are created by the linker. Refer to the linker files. 
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
	InitFlash();	// Call the flash wrapper init function
#endif //(FLASH)


//-------------------------------------------------------------
// Initialise Period of Cpu Timers
// Timer period definitions found in PeripheralHeaderIncludes.h
	CpuTimer0Regs.PRD.all =  mSec1;	 // 500ms * 2(# of LED states) = 1s blink rate
//	CpuTimer1Regs.PRD.all =  mSec1000;	
//	CpuTimer2Regs.PRD.all =  mSec5000;	
//-------------------------------------------------------------

#define period 8000							  // 80kHz when PLL is set to 0x10 (80MHz) 
//      period 250   			    	      // 160kHz when PLL is set to 0x10 (80MHz)
  
  // Time-base registers

   	EPwm3Regs.TBPRD = period;       		   // Set timer period, PWM frequency = 1 / period
   	EPwm3Regs.TBPHS.all = 0;				   // Time-Base Phase Register
   	EPwm3Regs.TBCTR = 0;					   // Time-Base Counter Register	
    EPwm3Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;  // Set Immediate load
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count-updown mode: used for symmetric PWM
	EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;	   // Disable phase loading
	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
	EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   	// Setup shadow register load on ZERO

   	EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   	EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;	// load on CTR=Zero
   	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;	// load on CTR=Zero

   	// Set Compare values

   	EPwm3Regs.CMPA.half.CMPA = duty_cycle_A;    // Set duty 50% initially
   	EPwm3Regs.CMPB = duty_cycle_B;	            // Set duty 50% initially

   	// Set actions

   	EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM2A on event A, up count
   	EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM2A on event A, down count

   	EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;            // Set PWM2B on event B, up count
   	EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;          // Clear PWM2B on event B, down count


//=================================
//	INITIALISATION - Peripherals
//=================================

// ADC INITIALISATION
    EALLOW;
	AdcRegs.ADCCTL1.bit.ADCREFSEL	= 0;	// Use internal bandgap
   	AdcRegs.ADCCTL1.bit.ADCBGPWD	= 1;	// Power up band gap
   	AdcRegs.ADCCTL1.bit.ADCREFPWD	= 1;	// Power up reference
   	AdcRegs.ADCCTL1.bit.ADCPWDN 	= 1;	// Power up rest of ADC
	AdcRegs.ADCCTL1.bit.ADCENABLE	= 1;	// Enable ADC
    for(i=0; i<5000; i++){}					// wait 60000 cycles = 1ms (each iteration is 12 cycles)

	AdcRegs.ADCCTL1.bit.INTPULSEPOS	= 1;	// create int pulses 1 cycle prior to output latch

	// set S/H window to 6 clk cycles (112.5ns)
	AdcRegs.ADCSOC0CTL.bit.ACQPS = 8;
   	AdcRegs.ADCSOC1CTL.bit.ACQPS = 8;
   	AdcRegs.ADCSOC2CTL.bit.ACQPS = 8;
   	AdcRegs.ADCSOC4CTL.bit.ACQPS = 8;
   	AdcRegs.ADCSOC5CTL.bit.ACQPS = 8;
   	AdcRegs.ADCSOC6CTL.bit.ACQPS = 8;
   	AdcRegs.ADCSOC9CTL.bit.ACQPS = 8;
   	AdcRegs.ADCSOC8CTL.bit.ACQPS = 8;
   	AdcRegs.ADCSOC10CTL.bit.ACQPS = 8;
   	AdcRegs.ADCSOC12CTL.bit.ACQPS = 8;
   	AdcRegs.ADCSOC14CTL.bit.ACQPS = 8;
   	

	AdcRegs.INTSEL1N2.bit.INT1SEL = 12;		// ADCCH12 (ADC-B4) EOC causes ADCInterrupt 1
	AdcRegs.INTSEL1N2.bit.INT1CONT = 1;		// set ADCInterrupt 1 to auto clr
	AdcRegs.INTSEL1N2.bit.INT1E = 1;		// enable ADC interrupt 1

// Note that SOC3, 7, 11, 13 & 15 are valid, but these SOCs are not configured 
// since these ADC outputs do not exist on the controlSTICK. The configuration
// is configured as it is for readability.

	//EOC = end of conversion event; SOC = start of conversion event
   	AdcRegs.ADCINTSOCSEL1.bit.SOC0 = 1;		// ADCInterrupt 1 causes SOC0
   	AdcRegs.ADCINTSOCSEL1.bit.SOC1 = 1;
   	AdcRegs.ADCINTSOCSEL1.bit.SOC2 = 1;
   	AdcRegs.ADCINTSOCSEL1.bit.SOC4 = 1;
   	AdcRegs.ADCINTSOCSEL1.bit.SOC5 = 1;
   	AdcRegs.ADCINTSOCSEL1.bit.SOC6 = 1;
   	AdcRegs.ADCINTSOCSEL2.bit.SOC8 = 1;
   	AdcRegs.ADCINTSOCSEL2.bit.SOC9 = 1;
   	AdcRegs.ADCINTSOCSEL2.bit.SOC10 = 1;
   	AdcRegs.ADCINTSOCSEL2.bit.SOC12 = 1;
   	AdcRegs.ADCINTSOCSEL2.bit.SOC14 = 1;   	

// Select the channel to be converted when SOCx is received
	AdcRegs.ADCSOC0CTL.bit.CHSEL= 0;	// convert ADC-A0 (CH0) when SOC0 is received
	AdcRegs.ADCSOC1CTL.bit.CHSEL= 1;	// convert ADC-A1 (CH1) when SOC1 is received
	AdcRegs.ADCSOC2CTL.bit.CHSEL= 2;
	AdcRegs.ADCSOC4CTL.bit.CHSEL= 4;
	AdcRegs.ADCSOC5CTL.bit.CHSEL= 5;
	AdcRegs.ADCSOC6CTL.bit.CHSEL= 6;
	AdcRegs.ADCSOC8CTL.bit.CHSEL= 8;
	AdcRegs.ADCSOC9CTL.bit.CHSEL= 9;	// convert ADC-B1 (CH9) when SOC9 is received
	AdcRegs.ADCSOC10CTL.bit.CHSEL= 10;
	AdcRegs.ADCSOC12CTL.bit.CHSEL= 12;
	AdcRegs.ADCSOC14CTL.bit.CHSEL= 14;
	

	EDIS;

	AdcRegs.ADCSOCFRC1.all = 0x1000;  	// kick start ADC by causing a SOC12 event
	
//====================================================================================
//CALIBRATION
//===================================================================================
// 
	jDelay(500);
	// black
	jSensMin[0] = AdcResult.ADCRESULT8;
	jSensMin[1] = AdcResult.ADCRESULT12;
	jSensMin[2] = AdcResult.ADCRESULT5;
	jSensMin[3] = AdcResult.ADCRESULT10;
	
	jDelay(500);
	jMove(false, false, 50.00, 50.00, 150, true, 500);
	
	//white
	jSensMax[0] = AdcResult.ADCRESULT8;
	jSensMax[1] = AdcResult.ADCRESULT12;
	jSensMax[2] = AdcResult.ADCRESULT5;
	jSensMax[3] = AdcResult.ADCRESULT10;
	
	jDelay(500);
	jMove(true, true, 50.00, 50.00, 150, true, 1000);

	
//=================================================================================
//jMove(true, true, 0.00, 0.00, 0, false, 0);
// 1. zistenie funkcie....
/*for(;;)
{
	jTest1 = jSensGetPercent(1);
	jTest2 = jSensGetPercent(2);
		jTest0 = jSensGetPercent(0);
		jTest3 = jSensGetPercent(3);
}*/


jLineFollower(true);
jRotate90(true);
jLineFollower(true);

} //END MAIN CODE
