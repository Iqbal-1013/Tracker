// ---------------------------------------------------------------------------
// ------------------------- INCLUDES ----------------------------------------
// ---------------------------------------------------------------------------


//Version two 


#include "lib\HyperVTS.h"


// ---------------------------------------------------------------------------
// ------------------------- GLOBALS -----------------------------------------
// ---------------------------------------------------------------------------

const unsigned long baudrate = 9600;

const uint8_t serviceString_BufferLength = 255;
char serviceString[serviceString_BufferLength] = { '\0' };


const uint8_t geofenceViolation_BufferLength = 255;
char geofenceViolation[geofenceViolation_BufferLength] = { '\0' };

const uint16_t fixed_loopTime = 5000;

boolean http_success = false, check_engine_force_off = false;
boolean checkFifteenMin = false, checkFiveDay = false, checkFuelData = false, onceCalled = false;
const unsigned long limit_http_failed = 20000;

unsigned long http_time = 0;

#define AT_CHECK_GPRS "AT+CGATT?"
#define RESPONSE_GPRS_CHECK "+CGATT: 1"
#define AT_POWER_OF_SHIELD "AT+CSCLK=1" 
#define AT_POWER_ON_SHIELD "AT+CSCLK=0"


void updateHttpAfterEngineOn();



// Configuration
Config config;



//Motion Manager
MotionManager motionManager;


// Geofence Manager
GeofenceManager geofenceManager;


// USSD Manager
USSD_Manager ussdManager;



// Engine Test

// Engine On Service
uint8_t engineOnServiceID = 2;
uint8_t engineOnServicePin = 16;
uint8_t engineOnServiceType = 1;


// Engine Off Service
uint8_t engineOffServiceID = 3;
uint8_t engineOffServicePin = 15;
uint8_t engineOffServiceType = 2;

uint8_t shut_down_pin = 21;
uint8_t shield_wakeup_pin = 22;

Service engineOnService(engineOnServiceID, engineOnServicePin, engineOnServiceType);
Service engineOffService(engineOffServiceID, engineOffServicePin, engineOffServiceType);


// Engine Manager
EngineManager engineManager(&engineOnService, &engineOffService, &motionManager);


// Fuel Test

// Oil
uint8_t oilServiceID = 16;
uint8_t oilServicePin = 30;
uint8_t oilServiceType = 0;

// Gas
uint8_t gasServiceID = 17;
uint8_t gasServicePin = 31;
uint8_t gasServiceType = 0;

Service serviceOil(oilServiceID, oilServicePin, oilServiceType);
Service serviceGas(gasServiceID, gasServicePin, gasServiceType);

FuelManager fuelManager(&serviceOil, &serviceGas);




// Flags
boolean FLAG_NEW_SMS = false;
boolean FLAG_NEW_GPS = false;

boolean pseudoSleepEnabled = false;
boolean deepSleepEnabled = false;


unsigned long timerStartPseudoSleep = 0;
unsigned long timerStartDeepSleep = 0;

//10 minutes sleep after car is off mode. 
//unsigned long pseudoSleepTime = 300000; //600000;
unsigned long pseudoSleepTime = 60000;
unsigned long fixed_five_day = 432000000;
//unsigned long fixed_five_day = 180000;
unsigned long fixed_fifteen_min = 900000;
unsigned long deepSleepTime = 120000; //604800000;
unsigned long remainingNightTime = 0;
unsigned long start = 0;
unsigned long five_day_counter = 0;
boolean sleepEnabled = false;
static int counter = 0;

const uint16_t HTTP_LOC_BUFFER_LENGTH = 100;
uint16_t counter_for_http = 0;
unsigned long http_failed_timer = 0;


// ---------------------------------------------------------------------------
// ------------------------- FUCTIONS ----------------------------------------
// ---------------------------------------------------------------------------


void Update();
unsigned long GetRemainingNightTime();
void SendStableFuelValue(unsigned long start);

struct points {
	float lat;
	float lng;
};
struct points latlong[10];


void reSetup()
{
	 //Watchdog
	Watchdog::Disable();

	// Timer
	Timer::Start();


	CONSOLE.begin(baudrate);		// Arduino baud rate
	SIM808.begin(baudrate);			// the SIM808 module baud rate


	Log(F("\n[ reSETUP BEGIN ]\n"));


	//// Engine Manager
	Log(F("\n\n[ CHECKING LOCK STATUS FROM EEPROM ]"));
	if (engineManager.IsLocked_EEPROM())
	{
		Log(F("[ LOCKING ENGINE ]"));
		engineManager.Lock();
	}
	else
	{
		Log(F("[ UNLOCKING ENGINE ]"));
		engineManager.Unlock();
	}


	// Starting GPS
	Log(F("\n[ INITIALIZING GPS ]"));
	GPS_Start();
	Log(F("Done."));


	Log(F("\n\n[ INITIALIZING SIM808 ]"));
	Time();
	SIM808_Init();
	Log(F("Done."));
	Time();


	Log(F("\n\n[ INITIALIZING SMS ]"));
	SMS_Init();
	Log(F("Done."));


	Log(F("\n\n[ INITIALIZING GPRS ]"));
	Time();
	if (!GPRS_Init()) {
		while (sendATcommand("AT", "OK", 2000) != 1);
		reSetup();
	}
	Log(F("Done."));
	Time();


	Log(F("\n[ reSETUP COMPLETE ]\n"));


	Diagnostics::time_setup = Timer::Duration();
	Log(F("\n\n[ SETUP TIME ] : "), Diagnostics::time_setup / 1000.0);

	//Watchdog::Enable();
}


void setup()
{
	// Watchdog
	Watchdog::Disable();

	pinMode(shut_down_pin, OUTPUT);
	pinMode(shield_wakeup_pin, OUTPUT);

	digitalWrite(shut_down_pin, LOW);
	digitalWrite(shield_wakeup_pin, LOW);

	// Timer
	Timer::Start();


	CONSOLE.begin(baudrate);		// Arduino baud rate
	SIM808.begin(baudrate);			// the SIM808 module baud rate

	Log(F("\n[ SETUP BEGIN ]\n"));

	//// Engine Manager
	Log(F("\n\n[ CHECKING LOCK STATUS FROM EEPROM ]"));
	if (engineManager.IsLocked_EEPROM())
	{
		Log(F("[ LOCKING ENGINE ]"));
		engineManager.Lock();
	}
	else
	{
		Log(F("[ UNLOCKING ENGINE ]"));
		engineManager.Unlock();
	}

	while (sendATcommand("AT+CMEE=1", "OK", 5000) != 1);

	// Starting GPS
	Log(F("\n[ INITIALIZING GPS ]"));
	GPS_Start();
	Log(F("Done."));


	Log(F("\n\n[ INITIALIZING SIM808 ]"));
	Time();
	SIM808_Init();
	Log(F("Done."));
	Time();

	Log(F("\n\n[ INITIALIZING SMS ]"));
	SMS_Init();
	Log(F("Done."));

	Log(F("\n\n[ INITIALIZING GPRS ]"));
	Time();
	if (!GPRS_Init()) {
		while (sendATcommand("AT", "OK", 2000) != 1);
		reSetup();
	}
	
	Log(F("Done."));
	Time();


	Log(F("\n[ SETUP COMPLETE ]\n"));


	Diagnostics::time_setup = Timer::Duration();
	Log(F("\n\n[ SETUP TIME ] : "), Diagnostics::time_setup / 1000.0);

	Timer::Start(five_day_counter);

	//Watchdog::Enable();
}



void loop()
{
	//check if engine is on/off
	if (engineManager.IsEngineOn())
	{
		Log("\nEngine ON");
		++counter;
		if (onceCalled) {

			digitalWrite(shield_wakeup_pin, HIGH);
			memset(AT_Buffer, '\0', AT_BufferLength);
			sprintf(AT_Buffer, "%s", AT_POWER_ON_SHIELD);
			delay(500);
			while (!AT_Send(SIM808, "OK", 5000))
			{
				memset(AT_Buffer, '\0', AT_BufferLength);
				sprintf(AT_Buffer, "%s", AT_POWER_ON_SHIELD);
			}
			delay(500);
			digitalWrite(shield_wakeup_pin, LOW);


			onceCalled = false;
			reSetup();
		}

		if (counter == 1)
		{
			updateHttpAfterEngineOn();
			Timer::Start(http_failed_timer);
			counter_for_http = 0;
		}
		Log("\n\n[ NORMAL OPERATION MODE ]");


		pseudoSleepEnabled = false;
		deepSleepEnabled = false;
		
		Update();
	}
	else
	{
		Timer::Start(five_day_counter);
		counter = 0;
		Log("\n\n[ PSEUDO SLEEP MODE ]");

		//Watchdog::Enable();

		//check for time from server
		//remainingNightTime = GetRemainingNightTime();
		//Log("\n\n [ REMAINING NIGHT TIME : ", remainingNightTime);

		Timer::Start(start);

		

		boolean flag_MidNightFuelSending = false;

		//if (!pseudoSleepEnabled) {
			SendStableFuelValue(start);
			//pseudoSleepEnabled = true;
		//}
		Watchdog::Disable();

		// check for sms always until it's night 
		while (true)
		{

			if (engineManager.IsEngineOn())
			{
				checkFiveDay = true;
				pseudoSleepEnabled = false;
				deepSleepEnabled = false;
				//Update();

				break;
			}if (Timer::Duration(five_day_counter) >= fixed_five_day && !onceCalled) {
				//GPRS_Stop();
				Log("\n\nFive day stopping");
				GPS_Stop();
				GPRS_Stop();
				delay(5000);
				//digitalWrite(shut_down_pin, HIGH);
				//delay(3000);
				//digitalWrite(shut_down_pin, LOW);
				memset(AT_Buffer, '\0', AT_BufferLength);
				sprintf(AT_Buffer, "%s", AT_POWER_OF_SHIELD);

				while(!AT_Send(SIM808, "OK", 5000));
				
				//break;

				checkFiveDay = true;
				onceCalled = true;

			}
			//else if (!flag_MidNightFuelSending && (Timer::Duration(start)) > remainingNightTime*1000)
			//{
			//	Log("\n\n[ MID NIGHT ]\n\n");
			//	flag_MidNightFuelSending = true;
			//	SendStableFuelValue(millis());
			//	updateHttpAfterEngineOn();
			//}
			else if (!onceCalled){
				delay(1000);
				Log("\n\n[ reading sms ]\n\n");
				//if (millis() - five_day_counter >= fixed_fifteen_min && !checkFifteenMin) {
				//	GPS_Stop();
				//	checkFifteenMin = true;
				//}

				// check if new SMS is available.  
				//Log(F("\n[ CHECKING FOR NEW SMS ]"));
				//FLAG_NEW_SMS = SMS_IsAvailable(hyperMSISDN);
				//Time();

				//if (FLAG_NEW_SMS)
				//{
				//	// Engine Manager
				//	Log(F("\n[ INVOKING ENGINE MANAGER EVENT HANDLER ]"));
				//	engineManager.EventHandler();
				//	Log(F("\nDone."));
				//	Time();

				//	SMS_Delete(1, 1);
				//	FLAG_NEW_SMS = false;
				//}
			}
			//Watchdog::Reset();

		}
	}
}


unsigned long GetRemainingNightTime()
{
	//http current time from server.
	memset(HTTP_Buffer, '\0', HTTP_BufferLength);
	sprintf(HTTP_Buffer, "%s=%s", "com_id", COMMERCIAL_ID);

	HTTP_Post(URL_REMAINING_NIGHT);
	HTTP_Parse();
	Log("\n\n [ REMAINING NIGHT TIME : ", atoi(HTTP_Buffer));
	return atoi(HTTP_Buffer);
	//return deepSleepTime;
}

void SendStableFuelValue(unsigned long start)
{
	Log("\n\n[ SENDING STABLE FUEL AND GAS VALUES ]");
	//Watchdog::Disable();

	// update fuel and gas for 1 minute after the car is off. 
	while (Timer::Duration(start) < pseudoSleepTime)
	{
		if (engineManager.IsEngineOn())
		{
			pseudoSleepEnabled = false;
			deepSleepEnabled = false;
			break;
		}

		memset(serviceString, '\0', serviceString_BufferLength);
		sprintf(serviceString, "%s", COMMERCIAL_ID);

		//engineManager.Update();
		engineManager.AppendToString(serviceString, '&');

		//HTTP fuel
		//fuelManager.Update();
		fuelManager.onlyFuel(serviceString, '&');

		Log(F("\n\n[ SENDING SERVICE STRING ]"));
		HTTP_Flush();
		sprintf(HTTP_Buffer, "%s=%s", "ss", serviceString);
		Log(F("\n\nHTTP Buffer : "), HTTP_Buffer);

		 
		send_HTTP(serviceURL);
		//Watchdog::Reset();
		delay(2000);
	}
}


void Update()
{
	Timer::Start();

	Log(F("\n[ LOOP BEGIN ]\n"));

	// Service String Send Logic
	Log(F("\n[ CREATING SERVICE STRING ]\n"));

	memset(serviceString, '\0', serviceString_BufferLength);
	memset(geofenceViolation, '\0', geofenceViolation_BufferLength);


	Log(F("\n[ COPYING COM ID ]"));
	sprintf(serviceString, "%s=%s","ss", COMMERCIAL_ID);
	Log(F("\nDone."));
	Time();


	// GPS 

	// Actual code
	Log(F("\n[ READING GPS ]"));
	FLAG_NEW_GPS = GPS_Read();
	Log(F("\nDone."));
	Time();


	// Getting time for motion manager. Required for calculating distance
	//motionManager.time = Timer::Duration(motionManager.timerStart);
	//Timer::Start(motionManager.timerStart);

	//// If GPS has problem fix
	//Log(F("\n[ GPS INDEX ] : "), gpsStringIndex);
	//Log(F("[ GPS VALUE ] : "), gpsStrings[gpsStringIndex]);

	//FLAG_NEW_GPS = true;
	//sprintf(AT_Buffer, "%s", gpsStrings[gpsStringIndex]);
	//sprintf(AT_Buffer, "%s", "+CGNSINF: 1,1,20170709102949.000,23.837925,90.369528,74.200,0");
	// Update index
	//gpsStringIndex = ++gpsStringIndex % gpsStringCount;


	// Fuel
	fuelManager.Update();
	fuelManager.AppendToString(serviceString, '&');


	if (FLAG_NEW_GPS)
	{
		//GPS_Parse();


		if (counter_for_http > 0)
		{
			Log("\n[resending latlong]\n");
			sprintf(serviceString, "%s&%s=%s", serviceString, "lt", Math::FloatToString(latlong[0].lat));
			sprintf(serviceString, "%s,%s|", serviceString, Math::FloatToString(latlong[0].lng));
			Log(F("\n[Before number 1 ]\n"),serviceString);

			for (int i = 1; i < counter_for_http; i++)
			{
				Log("\n[Inside Loop....]\n");
				sprintf(serviceString, "%s%s", serviceString, Math::FloatToString(latlong[i].lat));
				sprintf(serviceString, "%s,%s|", serviceString, Math::FloatToString(latlong[i].lng));
			}
			sprintf(serviceString, "%s%s", serviceString, Math::FloatToString(GPS_Latitude));
			sprintf(serviceString, "%s,%s", serviceString, Math::FloatToString(GPS_Longitude));
		}
		else
		{
			sprintf(serviceString, "%s&%s=%s", serviceString, "lt", Math::FloatToString(GPS_Latitude));
			sprintf(serviceString, "%s,%s", serviceString, Math::FloatToString(GPS_Longitude));
		}

		geofenceManager.Update(geofenceViolation);

		// Motion manager
		motionManager.AppendToServiceString(serviceString, '&');
	}

	// Engine status
	if (check_engine_force_off)
	{
		sprintf(serviceString, "%s%c%s=%d", serviceString, '&', "es",0);
	}
	else 
	{
		engineManager.AppendToString(serviceString, '&');
	}

	//adding speed to SS
	motionManager.Update(serviceString);

	// HTTP the Service String
	Time();
	Log(F("\n\n[ SERVICE STRING ] : "), serviceString);
	Log(F("[ GEOFENCE VIOLATION ] : "), geofenceViolation);


	Log(F("\n\n[ SENDING SERVICE STRING ]"));
	HTTP_Flush();
	sprintf(HTTP_Buffer, "%s&%s=%s", serviceString, "gv", geofenceViolation);
	
	//sprintf(HTTP_Buffer, "1234%s=%s&%s=%s", "ss", serviceString, "gv", geofenceViolation);
	Log(F("\n\nHTTP Buffer : "), HTTP_Buffer);


	memset(AT_Buffer, '\0', AT_BufferLength);
	sprintf(AT_Buffer, "%s", AT_CHECK_GPRS);

	if (GPRS_IsBearerConnected())
	{
		int http_return = 0;
		delay(1000);

		Watchdog::Reset();
		http_return = send_HTTP(serviceURL);
			
			if (http_return == 1)
			{
				counter_for_http = 0;
				Timer::Start(http_failed_timer);
				engineManager.HttpEvent();
				
				check_engine_force_off = engineManager.Update();
				Watchdog::Reset();
				//delay(2000);
			}
			else
			{
				
				if (FLAG_NEW_GPS) 
				{
					Log("\n[Storing Lat long for future use]\n");
					latlong[counter_for_http].lat = GPS_Latitude;
					latlong[counter_for_http].lng = GPS_Longitude;
					Log(F("\nLattitude error http : "), latlong[counter_for_http].lat);
					Log(F("\nLongitude error http : "), latlong[counter_for_http].lng);
				}
				++counter_for_http;
				
				Watchdog::Reset();

				if (http_return == -2) 
				{
					Watchdog::Reset();
					while (sendATcommand("AT", "OK", 2000) != 1);
					reSetup();
					counter_for_http = 0;
					//Update();
				}
				Log("Http failed ", counter_for_http);
				if (counter_for_http >= 5) 
				{
					counter_for_http = 0;
					if (!GPRS_Init()) { 
						while (sendATcommand("AT", "OK", 2000) != 1);
						reSetup(); 
						//Update(); 
					}
				}
				delay(2000);
			}
	}
	else {
		GPRS_Stop();
		delay(1000);
		if (!GPRS_Init())
		{
			while (sendATcommand("AT", "OK", 2000) != 1);
			reSetup();
			//Update();
		}
	}

	// Diagnostics
	Diagnostics::Update(Timer::Duration());
	Diagnostics::Print();
	//}


	if (Diagnostics::count_loop == 1 || Diagnostics::count_loop % Diagnostics::sendLoopCount == 0)
	{
		HTTP_Flush();
		Diagnostics::ToString(HTTP_Buffer, HTTP_BufferLength);
		send_HTTP(URL_DIAGNOSTICS);
		Diagnostics::Reset();
	}

	//int16_t timeDiff = fixed_loopTime - Timer::Duration();
	//Log("\n\n[ MILLIS ] : ", millis() / 1000);
	//Log("\n\n[ START TIME ] : ", Timer::start / 1000);
	//Log("\n\n[ DURATION ] : ", Timer::Duration() / 1000);
	//Log("\n\n[ DELAY TIME ] : ", timeDiff / 1000);
	//if (timeDiff > 0 && http_success)
	//{
	//	delay(timeDiff);
	//}
	//Log("\n\n[ LOOP TIME ] : ", (millis() - Timer::start) / 1000);


	// Watchdog
	Watchdog::Reset();
}


void updateHttpAfterEngineOn() 
{

	memset(latlong, '\0', 10);
	Watchdog::Disable();
	//GPS CHECK AND RECONNECT
	if(sendATcommand("AT+CGNSPWR?", "+CGNSPWR: 1", 5000) == 1)
	{
		Log("\n\n[reseting gps]\n\n");
		while(sendATcommand("AT+CGPSRST=1", "OK", 5000) != 1);
	}
	else
	{
		Log("\n\n[reseting gps]\n\n");
		GPS_Start();
	}

	memset(AT_Buffer, '\0', AT_BufferLength);
	sprintf(AT_Buffer, "%s", AT_CHECK_GPRS);

	////GPRS
	//if (sendATcommand(AT_CHECK_GPRS, RESPONSE_GPRS_CHECK, 5000) == 0) {
	//	GPRS_Stop();
	//	delay(1000);
	//	if (!GPRS_Init()) {
	//		while (sendATcommand("AT", "OK", 2000) != 1);
	//		reSetup();
	//	}
	//}
	//else {
	//	if (sendATcommand(AT_CHECK_GPRS, "OK", 5000) == 0) {
	//		GPRS_Stop();
	//		delay(1000);
	//		if (!GPRS_Init()) {
	//			while (sendATcommand("AT", "OK", 2000) != 1);
	//			reSetup(); //Update();
	//		}
	//	}
	//}

	if (!GPRS_IsBearerConnected())
	{
		GPRS_Stop();
		delay(1000);
		if (!GPRS_Init()) {
			while (sendATcommand("AT", "OK", 2000) != 1);
			reSetup();
		}
	}

	// Config
	Log(F("\n\n[ UPDATING CONFIGURATION ]"));
	Time();
	config.Update();
	Log(F("Done."));
	Time();


	// Update the system time
	Time_UpdateSystemTime(config);


	//// Engine
	Log(F("\n\n[ INITIALIZING ENGINE ]"));
	Time();
	engineManager.Init(config);
	Log(F("Done."));
	Time();
	if (!config.lockStatus) {
		check_engine_force_off = false;
	}


	SMS_Init();


	// Geofence Manager
	Log(F("\n\n[ INITIALIZING GEOFENCE MANAGER ]"));
	Time();
	geofenceManager.Init(config);
	Log(F("Done."));
	Time();

	Watchdog::Enable();
}
