/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * [File Name]     main.c
 * [Platform]      FRDM-K64F
 * [Project]       DataLog
 * [Version]       1.00
 * [Author]        b22385
 * [Date]          12/24/2015
 * [Language]      'C'
 * [History]       1.00 - Original Release
 *
 */

//-----------------------------------------------------------------------
// Standard C/C++ Includes
//-----------------------------------------------------------------------

//-----------------------------------------------------------------------
// KSDK Includes
//-----------------------------------------------------------------------
#include "main.h"
#include "DataLogger.h"
#include "fsl_adc16_driver.h"
#include "fsl_pmc_hal.h"
//-----------------------------------------------------------------------
// Application Includes
//-----------------------------------------------------------------------

#define ADCR_VDD                (65535U)    // Maximum value when use 16b resolution

#define V_BG                    (1000U)     // BANDGAP voltage in mV (trim to 1.0V)

#define V_TEMP25                (716U)      // Typical converted value at 25 oC in mV

#define M                       (1620U)     // Typical slope:uV/oC

#define STANDARD_TEMP           (25)

#define ADC16_INSTANCE                (0)   // ADC instacne

#define ADC16_TEMPERATURE_CHN         (kAdc16Chn26) // Temperature Sensor Channel

#define ADC16_BANDGAP_CHN             (kAdc16Chn27) // ADC channel of BANDGAP

#define ADC16_CHN_GROUP               (0)   // ADC group configuration selection

#define HALL_SENSOR_CHN		          (kAdc16Chn0) // ADC channel of BANDGAP

#define SAMPLE_INTERVAL				  (5) //seconds

#define CONT_SAMPLE_INTERVAL		  (0) //seconds

#define TOTAL_MEASUREMENTS			  (50)
//-----------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------
static void SensorCharacterization_ADCInit(void);

static int16_t SensorCharacterization_Measure(uint8_t AdcChannel);

static void SensorCharacterization_SetSampleAlarm(uint32_t AlarmInSeconds);
//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------

//-----------------------------------------------------------------------
// Typedefs
//-----------------------------------------------------------------------

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------

 static const gpio_input_pin_user_config_t inputPin[] = {
 		{
 				.pinName	= kGpioSW2,
 				.config.isPullEnable = true,
 				.config.interrupt = kPortIntFallingEdge,
 				.config.pullSelect = kPortPullUp,
 		},
 		{
 				.pinName	= kGpioSW3,
 				.config.isPullEnable = true,
 				.config.interrupt = kPortIntFallingEdge,
 				.config.pullSelect = kPortPullUp,
 		},
 		{
 				.pinName	= GPIO_PINS_OUT_OF_RANGE,
 		}

 };


 uint32_t adcValue = 0;               // ADC value

 uint32_t adcrTemp25 = 0;             // Calibrated ADCR_TEMP25

 uint32_t adcr100m = 0;               // calibrated conversion value of 100mV

 adc16_converter_config_t adcUserConfig;   // structure for user config

 uint16_t DataToLog[2];

 volatile bool StartMeasurement = false;

 volatile bool ContinuousMeasurement = false;
//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------

//-----------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------

int16_t HallSensorData;

int main(void)
{
	int16_t TempSensorData;
	int32_t Temperature;
//	int16_t HallSensorData;
	uint16_t Measures = TOTAL_MEASUREMENTS;
	uint16_t ContinuousMeasures = false;
	uint8_t LogMeasurements = false;
	// Configure board specific pin muxing
    hardware_init();

    OSA_Init();



	/* Configure safe remove pin */
	GPIO_DRV_Init(&switchPins[0], &ledPins[0]);

    SensorCharacterization_ADCInit();

    (void)DataLogger_Init();

    GPIO_DRV_ClearPinOutput(kGpioLED1);

    for (;;)
    {

    	 if(LogMeasurements == true)
    	 {
			 if ((RTC_DRV_IsAlarmPending(0)) && Measures)
			 {
				 HallSensorData = SensorCharacterization_Measure(HALL_SENSOR_CHN);

				 TempSensorData = SensorCharacterization_Measure(ADC16_TEMPERATURE_CHN);

				 Temperature = (int32_t)(STANDARD_TEMP - ((int32_t)TempSensorData - (int32_t)adcrTemp25) * 100000 /(int32_t)(adcr100m*M));

				 DataToLog[0] = (uint16_t)HallSensorData;

				 DataToLog[1] = ((uint16_t)(Temperature&0xFFFF));

				 DataLogger_PostEvent(NULL, &DataToLog[0], sizeof(DataToLog)/sizeof(int16_t));



				 if(ContinuousMeasures == false)
				 {
					 Measures--;
					 SensorCharacterization_SetSampleAlarm(SAMPLE_INTERVAL);
				 }
				 else
				 {
					 SensorCharacterization_SetSampleAlarm(CONT_SAMPLE_INTERVAL);
				 }

				 GPIO_DRV_TogglePinOutput(kGpioLED2);

			 }
			 else if(!Measures)
			 {
				 GPIO_DRV_ClearPinOutput(kGpioLED3);
				 LogMeasurements = false;
			 }
    	 }

    	 if(StartMeasurement == true)
    	 {
    		 StartMeasurement = false;

    		 if(LogMeasurements != true)
    		 {
    			 LogMeasurements = true;
    			 ContinuousMeasures = false;
    			 Measures = TOTAL_MEASUREMENTS;
    			 GPIO_DRV_SetPinOutput(kGpioLED3);
    			 SensorCharacterization_SetSampleAlarm(SAMPLE_INTERVAL);
    		 }
    	 }

    	 if(ContinuousMeasurement == true)
    	 {
    		 ContinuousMeasurement = false;

    		 if(LogMeasurements != true)
    		 {
    			 LogMeasurements = true;
    			 ContinuousMeasures = true;
    			 Measures = TOTAL_MEASUREMENTS;
    			 GPIO_DRV_SetPinOutput(kGpioLED3);
    			 SensorCharacterization_SetSampleAlarm(CONT_SAMPLE_INTERVAL);
    		 }
    	 }
    }


}

static void SensorCharacterization_ADCInit(void)
{
    adc16_chn_config_t adcChnConfig;
    adc16_hw_average_config_t userHwAverageConfig;
    pmc_bandgap_buffer_config_t pmcBandgapConfig = {
        .enable = true,
        .enableInLowPower = false,

    };

    uint32_t bandgapValue = 0;  // ADC value of BANDGAP
    uint32_t vdd = 0;           // VDD in mV
    adc16_calibration_param_t adcCalibraitionParam;

    // Initialization ADC for
	// 16bit resolution.
	// interrupt mode and hw trigger disabled,
	// normal convert speed, VREFH/L as reference,
	// disable continuous convert mode.
	ADC16_DRV_StructInitUserConfigDefault(&adcUserConfig);
	// Use 16bit resolution if enable.
	adcUserConfig.resolution = kAdc16ResolutionBitOf16;
	adcUserConfig.lowPowerEnable = false;
	adcUserConfig.asyncClkEnable = false;
	adcUserConfig.clkSrc = kAdc16ClkSrcOfBusClk;
	adcUserConfig.highSpeedEnable = false;
	adcUserConfig.longSampleTimeEnable =true;
	adcUserConfig.longSampleCycleMode =kAdc16LongSampleCycleOf24;
	adcUserConfig.hwTriggerEnable = false;
	adcUserConfig.refVoltSrc = kAdc16RefVoltSrcOfVref;
	adcUserConfig.continuousConvEnable = false;
	adcUserConfig.dmaEnable = false;

	ADC16_DRV_Init(ADC16_INSTANCE, &adcUserConfig);

    // Use hardware average to increase stability of the measurement.
    userHwAverageConfig.hwAverageEnable = true;
    userHwAverageConfig.hwAverageCountMode = kAdc16HwAverageCountOf32;
    ADC16_DRV_ConfigHwAverage(ADC16_INSTANCE, &userHwAverageConfig);

    // Auto calibration
    ADC16_DRV_GetAutoCalibrationParam(ADC16_INSTANCE, &adcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(ADC16_INSTANCE, &adcCalibraitionParam);

    // Enable BANDGAP reference voltage
    PMC_HAL_BandgapBufferConfig(PMC_BASE_PTR, &pmcBandgapConfig);



    // Configure the conversion channel
    // differential and interrupt mode disable.
    adcChnConfig.chnIdx                  = (adc16_chn_t)ADC16_BANDGAP_CHN;

    adcChnConfig.diffConvEnable          = false;

    adcChnConfig.convCompletedIntEnable  = false;
    ADC16_DRV_ConfigConvChn(ADC16_INSTANCE, ADC16_CHN_GROUP, &adcChnConfig);

    // Wait for the conversion to be done
    ADC16_DRV_WaitConvDone(ADC16_INSTANCE, ADC16_CHN_GROUP);

    // Get current ADC BANDGAP value and format it.
    bandgapValue = ADC16_DRV_GetConvValueSigned(ADC16_INSTANCE, ADC16_CHN_GROUP);
    // Calculates bandgapValue in 16bit resolution
    // from 12bit resolution to calibrate.

    // ADC stop conversion
    ADC16_DRV_PauseConv(ADC16_INSTANCE, ADC16_CHN_GROUP);

    // Get VDD value measured in mV
    // VDD = (ADCR_VDD x V_BG) / ADCR_BG
    vdd = ADCR_VDD * V_BG / bandgapValue;
    // Calibrate ADCR_TEMP25
    // ADCR_TEMP25 = ADCR_VDD x V_TEMP25 / VDD
    adcrTemp25 = ADCR_VDD * V_TEMP25 / vdd;
    // Calculate conversion value of 100mV.
    // ADCR_100M = ADCR_VDD x 100 / VDD
    adcr100m = ADCR_VDD*100/ vdd;

    // Disable BANDGAP reference voltage
    pmcBandgapConfig.enable = false;
    PMC_HAL_BandgapBufferConfig(PMC_BASE_PTR, &pmcBandgapConfig);
}


static int16_t SensorCharacterization_Measure(uint8_t AdcChannel)
{
    adc16_chn_config_t chnConfig;
    int16_t AdcData;

    // Configure the conversion channel
    // differential and interrupt mode disable.
    chnConfig.chnIdx     = (adc16_chn_t)AdcChannel;
    chnConfig.diffConvEnable = false;
    chnConfig.convCompletedIntEnable  = false;

    // Software trigger the conversion.
    ADC16_DRV_ConfigConvChn(ADC16_INSTANCE, ADC16_CHN_GROUP, &chnConfig);

    // Wait for the conversion to be done.
    ADC16_DRV_WaitConvDone(ADC16_INSTANCE, ADC16_CHN_GROUP);

    // Fetch the conversion value.
    AdcData = ADC16_DRV_GetConvValueRAW(ADC16_INSTANCE, ADC16_CHN_GROUP);

    // Pause the conversion.
    ADC16_DRV_PauseConv(ADC16_INSTANCE, ADC16_CHN_GROUP);

    return(AdcData);

}

static void SensorCharacterization_SetSampleAlarm(uint32_t AlarmInSeconds)
{
	rtc_datetime_t CurrentTime;
	uint32_t CurrentTimeInSeconds;

	RTC_DRV_GetDatetime(0, &CurrentTime);
	// Convert current date time to seconds
	RTC_HAL_ConvertDatetimeToSecs(&CurrentTime, &CurrentTimeInSeconds);

	CurrentTimeInSeconds += AlarmInSeconds;

	// Convert sec to date type
	RTC_HAL_ConvertSecsToDatetime(&CurrentTimeInSeconds, &CurrentTime);

	RTC_DRV_SetAlarm(0, &CurrentTime, false);
}

/* gpio IRQ handler with the same name in startup code. */
void PORTC_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    PORT_HAL_ClearPortIntFlag(PORTC_BASE_PTR);
    StartMeasurement = true;
}

/* gpio IRQ handler with the same name in startup code. */
void PORTA_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    PORT_HAL_ClearPortIntFlag(PORTA_BASE_PTR);
    ContinuousMeasurement = true;
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
