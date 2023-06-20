#include "main.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "main.h"

//******************
// CLASS: sensor
//
// DESCRIPTION:
//  distance optical sensor class
//
// CREATED: 03.03.2021, by Sergey
//
// FILE: sensor.h
//
class sensor{
public:
	sensor() = default;
	~sensor() = default;

	void SetOffsetMin(uint16_t offset);
	void SetTrigger(uint16_t offset);
	void SetCallChanel(uint16_t ch);
	void SetOffsetMax(uint16_t offset);
	void SetTimeCall(uint32_t time);
	void SetOffsetTime(uint32_t time);
	void SetTimeoutRasing(uint32_t time);
	void SetSensorType(SensorType type);

	uint16_t GetOffsetMin();
	uint16_t GetOffsetMax();
	uint16_t GetResult();
	uint16_t GetCallChanel();
	uint16_t GetTrigger();
	uint32_t GetOffsetTime();
	uint32_t GetTimeoutRasing();
	SensorType GetSensorType();
	bool Getdetect();
	bool StatusCalibration();


	uint32_t DataProcessing(uint16_t *data);
	void CallDistance(); // считывает и сохраняет данные расстояния (расстояние до ленты)
	int CallTime(); // считывает и сохраняет данные времени(размер ребер на ленте)
	bool DetectPoll(uint32_t tRising = 0, uint32_t tFalling = 0);
	void PwrSet(uint16_t r);
	void Init(settings_t *set, TIM_TypeDef* tim, uint32_t triggerChannel, uint32_t echoChannel, GPIO_TypeDef* GPIO_pwr, uint16_t Pin_pwr, int ID, float soundSpeed = 343000.0f); // скорость звука указана в милиметрах/сек
	void Init(settings_t *set, osSemaphoreId *ADC_endHandle, ADC_HandleTypeDef *hadc, uint16_t *adc_buffer, GPIO_TypeDef* GPIO_pwr, uint16_t Pin_pwr, int ID);

	//uint32_t timOutRising = 200;
	//uint32_t timOutFalling = 10;
	bool change_settings = false;
	uint16_t Depth = 10;

	//for hcsr04

	/*This method should be invoked when timer update event occurs*/
	void _acknowledgeTimerUpdate();

	/*This method should be invoked when channel input capture event occurs*/
	void _acknowledgeChannelCapture();

	/*Negative values will be returned if there is no object before sensor. */
	float GetDistance();

	/*Negative values will be returned if there is no object before sensor. */
	float GetDistanceInSeconds();


private:

	enum EchoPulseState {
		NOT_STARTED,
		STARTED,
		ENDED,
	};

	float ExpRunningAvgAdaptive(float newVal);

	settings_t *setttings;
	bool detect;                        // в зоне сенсора что то есть
	uint16_t result = 0;				// входное напряжение от датчика
	int id = 0;
	bool calibrationInProgress = false; // состаяние калибровки

	osSemaphoreId *ADC_endHandle;
	ADC_HandleTypeDef *hadc;
	uint16_t *adc_buffer;
	GPIO_TypeDef* GPIO_pwr;
	uint16_t Pin_pwr;

	uint32_t oldTimeRising = 0;
	uint32_t timeRising = 0;
	uint32_t oldTimeFalling = 0;
	uint32_t timeFalling = 0;


	//for hcsr04

	float soundSpeed;

	// base timer structure and channel codes
	TIM_HandleTypeDef hTim;
	uint32_t triggerChannel;
	uint32_t echoChannel;

	// start and end of echo pulse
	uint32_t echoPulseStart;
	uint32_t echoPulseEnd;

	// echo pulse state
	EchoPulseState echoPulseState;

	// calculated echo delay
	int32_t echoDelay;

	// control parameters
	const uint32_t PROBING_FREQUNCY = 20; // 20 Hz
	const uint32_t COUNTER_FREQUNCY = 100000; // 100 kHz
	const uint32_t TRIGGER_PULSE_LEN = 10; // 10 us
	const uint32_t INTERRUPT_PRIOIRY = 8;
	const uint32_t INTERRUPT_SUBPRIOIRY = 8;

};
