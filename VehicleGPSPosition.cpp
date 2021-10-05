/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "VehicleGPSPosition.hpp"

#include <px4_platform_common/log.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/takeoff_status.h>
#include <unistd.h>
#include <poll.h>
#include <iostream>
#include <fstream>
#include <limits>
#include <cmath>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <math.h>


//////////////////////////////////////// added for fault injection
double injected_gps_lat = std::numeric_limits<double>::quiet_NaN();
double injected_gps_lon = std::numeric_limits<double>::quiet_NaN();
double injected_gps_alt = std::numeric_limits<double>::quiet_NaN();
double mission_deviation = std::numeric_limits<double>::quiet_NaN();
double start_injection_time = std::numeric_limits<double>::quiet_NaN();
double end_injection_time = std::numeric_limits<double>::quiet_NaN();
double delay_value = std::numeric_limits<double>::quiet_NaN();
//1 = Fixed, 2 = Freeze, 3 = Delay
int fault_mode = 0;
//Varibles for takeoff detection
bool takeoff_detected = false;
bool valueFreezed = false;
uint64_t takeoff_start_timestamp;
int takeoff_value = 0, aux_takeoff_value = 0;
bool goldrun = true;


///////////////////////////////////////////////////////////////////
std::string filename = "/home/naghmeh/bubbles/PX4-Autopilot/faults/FaultData.fc";

std::string FaultType = "Software";
std::string FaultSubType = "FreezeValue";
std::string FaultTarget = "GPS";
double InjectionTimeStart = std::numeric_limits<double>::quiet_NaN();
double InjectionTimeEnd = std::numeric_limits<double>::quiet_NaN();
std::string Values[6]= {"","","","","",""};

///////////////////////////////////////////////////////////////////



namespace sensors
{

using namespace matrix;  /////////////////// added for fault injection
using math::constrain;   /////////////////// added for fault injection

VehicleGPSPosition::VehicleGPSPosition() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

VehicleGPSPosition::~VehicleGPSPosition()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleGPSPosition::Start()
{
	// force initial updates
	ParametersUpdate(true);
	ReadConfigFile();  /////////////////// added for fault injection
	ScheduleNow();

	return true;
}

void VehicleGPSPosition::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_gps_sub) {
		sub.unregisterCallback();
	}
}


//////////////////////////////////////////// added for fault injection

void VehicleGPSPosition::ReadConfigFile(){
	std::ifstream ConfigFile;
	std::string text;
	std::string tempString, tempString2;
	std::string tempValue, tempValue2;
	std::vector<std::string> file_contents;
	
	ConfigFile.open(filename.c_str());
	
	if (ConfigFile.is_open()){
		goldrun=false;
		PX4_INFO("not a goldrun");
	}
		


	while (getline (ConfigFile, text)){
		PX4_INFO("Line is %s", text.c_str());
		file_contents.push_back(text);
	}

	ConfigFile.close();

	for(std::size_t i = 0; i < file_contents.size(); i++){
		tempString = file_contents[i];
		PX4_INFO("parsing ... %s", tempString.c_str());

		//////////////////////////////////////////////////////////////////////////
		if(tempString.find("FaultType") != std::string::npos){
			FaultType = tempString.substr(12, tempString.size());
			PX4_INFO("FaultType %s\n", FaultType.c_str());

		} else if (tempString.find("FaultSubType") != std::string::npos) {
			FaultSubType = tempString.substr(15, tempString.size());
			PX4_INFO("FaultSubType %s\n", FaultSubType.c_str());

		} else if (tempString.find("FaultTarget") != std::string::npos){
			FaultTarget = tempString.substr(14, tempString.size());
			PX4_INFO("FaultTarget %s\n", FaultTarget.c_str());

		} else if (tempString.find("InjectionTimeStart") != std::string::npos){
			tempString = tempString.substr(21, tempString.size());
			if(tempString.compare("nan") != 0){
				//end_injection_time = atof(tempValue2.c_str()) * 1000000;
				InjectionTimeStart = std::stod(tempString) * 1000000;
				PX4_INFO("Injected start time is %f\n", InjectionTimeStart);
			}
		
		} else if (tempString.find("InjectionTimeEnd") != std::string::npos){
			tempString = tempString.substr(19, tempString.size());
			if(tempString.compare("nan") != 0){
				//end_injection_time = atof(tempValue2.c_str()) * 1000000;
				InjectionTimeEnd = std::stod(tempString) * 1000000;
				PX4_INFO("Injected end time is %f\n", InjectionTimeEnd);
			}

		} else if (tempString.find("Values") != std::string::npos){
			int s = tempString.size() -1;
			tempString = tempString.substr(10, s);
			PX4_INFO("FaultTarget %s\n", tempString.c_str());

			std::string delimiter = ", ";

			size_t pos = 0;
			std::string token;
			int counter=0;
			while ((pos = tempString.find(delimiter)) != std::string::npos) {
    				token = tempString.substr(0, pos);
				Values[counter++]=token;
    				//std::cout << token << std::endl;
				//PX4_INFO("Value Token %s\n", token.c_str());
    				tempString.erase(0, pos + delimiter.length());
				PX4_INFO("Values %s\n", Values[counter-1].c_str());
			}
			delimiter = "]";
			pos = tempString.find(delimiter);
			token = tempString.substr(0, pos);
			Values[counter]=token;
			PX4_INFO("Values %s\n", Values[counter].c_str());


		}

		//////////////////////////////////////////////////////////////////////////


		/*

		if(tempString.find("Mission Deviation") != std::string::npos){
			tempValue = tempString.substr(19, tempString.size());
			if(tempValue.compare("nan") != 0){
				//mission_deviation = atof(tempValue.c_str()) * 1000000;
				mission_deviation = std::stod(tempValue) * 1000000;
			}
			PX4_INFO("Mission Deviation is %f\n", mission_deviation);
		}
		if(tempString.find("Component") != std::string::npos){
			tempValue = tempString.substr(12, tempString.size());
			PX4_INFO("Component is %s\n", tempValue.c_str());
			if(tempValue == "GPS Failure"){
				tempString2 = file_contents[++i];
				tempValue2 = tempString2.substr(13, tempString2.size());
				PX4_INFO("Fault value is %s\n", tempValue2.c_str());
				//ADD OPTIONS FOR GPS FAILURE HERE
				if(tempValue2 == "Fixed Value"){
					fault_mode = 1;
					tempString2 = file_contents[++i];
					tempValue2 = tempString2.substr(10, tempString2.size());
					if(tempValue2.compare("nan") != 0){
						//injected_gps_lat = atof(tempValue2.c_str()) * 1.0E7;
						injected_gps_lat = std::stod(tempValue2) * 1.0E7;
						PX4_INFO("Injected GPS Lat is %f\n", injected_gps_lat);
					}
					tempString2 = file_contents[++i];
					tempValue2 = tempString2.substr(10, tempString2.size());
					if(tempValue2.compare("nan") != 0){
						//injected_gps_lon = atof(tempValue2.c_str()) * 1.0E7;
						injected_gps_lon = std::stod(tempValue2) * 1.0E7;
						PX4_INFO("Injected GPS Lon is %f\n", injected_gps_lon);
					}
					tempString2 = file_contents[++i];
					tempValue2 = tempString2.substr(10, tempString2.size());
					if(tempValue2.compare("nan") != 0){
						//injected_gps_alt = atof(tempValue2.c_str()) * 1.0E7;
						injected_gps_alt = std::stod(tempValue2) * 1.0E7;
						PX4_INFO("Injected GPS Alt is %f\n", injected_gps_alt);
					}
					tempString2 = file_contents[++i];
					tempValue2 = tempString2.substr(13, tempString2.size());
					if(tempValue2.compare("nan") != 0){
						//start_injection_time = atof(tempValue2.c_str()) * 1000000;
						start_injection_time = std::stod(tempValue2) * 1000000;
						PX4_INFO("Injected start time is %f\n", start_injection_time);
					}
					tempString2 = file_contents[++i];
					tempValue2 = tempString2.substr(11, tempString2.size());
					if(tempValue2.compare("nan") != 0){
						//end_injection_time = atof(tempValue2.c_str()) * 1000000;
						end_injection_time = std::stod(tempValue2) * 1000000;
						PX4_INFO("Injected end time is %f\n", end_injection_time);
					}
				}else if(tempValue2 == "Freeze Value"){
					fault_mode = 2;
					tempString2 = file_contents[++i];
					tempValue2 = tempString2.substr(13, tempString2.size());
					if(tempValue2.compare("nan") != 0){
						//start_injection_time = atof(tempValue2.c_str()) * 1000000;
						start_injection_time = std::stod(tempValue2) * 1000000;
						PX4_INFO("Injected start time is %f\n", start_injection_time);
					}
					tempString2 = file_contents[++i];
					tempValue2 = tempString2.substr(11, tempString2.size());
					if(tempValue2.compare("nan") != 0){
						//end_injection_time = atof(tempValue2.c_str()) * 1000000;
						end_injection_time = std::stod(tempValue2) * 1000000;
						PX4_INFO("Injected end time is %f\n", end_injection_time);
					}
				}else if(tempValue2 == "Delay Value"){
					fault_mode = 3;
					tempString2 = file_contents[++i];
					tempValue2 = tempString2.substr(14, tempString2.size());
					if(tempValue2.compare("nan") != 0){
						//delay_value = atof(tempValue2.c_str());
						delay_value = std::stod(tempValue2);
						PX4_INFO("Delay value is %f\n", delay_value);
					}
					tempString2 = file_contents[++i];
					tempValue2 = tempString2.substr(13, tempString2.size());
					if(tempValue2.compare("nan") != 0){
						//start_injection_time = atof(tempValue2.c_str()) * 1000000;
						start_injection_time = std::stod(tempValue2) * 1000000;
						PX4_INFO("Injected start time is %f\n", start_injection_time);
					}
					tempString2 = file_contents[++i];
					tempValue2 = tempString2.substr(11, tempString2.size());
					if(tempValue2.compare("nan") != 0){
						//end_injection_time = atof(tempValue2.c_str()) * 1000000;
						end_injection_time = std::stod(tempValue2) * 1000000;
						PX4_INFO("Injected end time is %f\n", end_injection_time);
					}
				}
			}
		}*/
	} 
	//changed
	/*srand(time(NULL));
	if(start_injection_time > 0 && end_injection_time > 0){
		int temp_start_time = round(start_injection_time);
		int temp_end_time = round(end_injection_time);
		PX4_INFO("Injected temp start time is %d\n", temp_start_time);
		PX4_INFO("Injected temp end time is %d\n", temp_end_time);
		do{
			start_injection_time = rand() % (temp_end_time-temp_start_time) + temp_start_time;
			end_injection_time = rand() % (temp_end_time-temp_start_time) + temp_start_time;
		}while(start_injection_time >= end_injection_time);
		PX4_INFO("Injected start time is %f\n", start_injection_time);
		PX4_INFO("Injected end time is %f\n", end_injection_time);
	}*/
}

///////////////////////////////////////////////////////////


void VehicleGPSPosition::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		if (_param_sens_gps_mask.get() == 0) {
			_sensor_gps_sub[0].registerCallback();

		} else {
			for (auto &sub : _sensor_gps_sub) {
				sub.registerCallback();
			}
		}

		_gps_blending.setBlendingUseSpeedAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_SPD_ACC);
		_gps_blending.setBlendingUseHPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_HPOS_ACC);
		_gps_blending.setBlendingUseVPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_VPOS_ACC);
		_gps_blending.setBlendingTimeConstant(_param_sens_gps_tau.get());
		_gps_blending.setPrimaryInstance(_param_sens_gps_prime.get());
	}
}

void VehicleGPSPosition::Run()
{
	perf_begin(_cycle_perf);
	ParametersUpdate();

	// GPS blending
	ScheduleDelayed(500_ms); // backup schedule

	// Check all GPS instance
	bool any_gps_updated = false;
	bool gps_updated = false;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		gps_updated = _sensor_gps_sub[i].updated();

		sensor_gps_s gps_data;

		if (gps_updated) {
			any_gps_updated = true;

			_sensor_gps_sub[i].copy(&gps_data);
			_gps_blending.setGpsData(gps_data, i);

			if (!_sensor_gps_sub[i].registered()) {
				_sensor_gps_sub[i].registerCallback();
			}
		}
	}

	if (any_gps_updated) {
		_gps_blending.update(hrt_absolute_time());

		if (_gps_blending.isNewOutputDataAvailable()) {
			Publish(_gps_blending.getOutputGpsData(), _gps_blending.getSelectedGps());
		}
	}

	perf_end(_cycle_perf);
}




void VehicleGPSPosition::Publish(const sensor_gps_s &gps, uint8_t selected)
{
	vehicle_gps_position_s gps_output{};


	///////////////////////////////////////////////////////////////////// added for fault injection
	gps_output.timestamp = gps.timestamp;
	gps_output.time_utc_usec = gps.time_utc_usec;



	if (!goldrun){

		int takeoff_sub = orb_subscribe(ORB_ID(takeoff_status));
		if(!takeoff_detected){
			PX4_INFO("get takeoff value ........ : %d", takeoff_value);
			orb_set_interval(takeoff_sub, 20);
		
			struct takeoff_status_s takeoff;
			orb_copy(ORB_ID(takeoff_status), takeoff_sub, &takeoff);

			takeoff_value = takeoff.takeoff_state;

			PX4_INFO("tookoff value is : %d", takeoff_value);
			PX4_INFO("takeoff detected is: %d", takeoff_detected);
		}	
		
		if((takeoff_value >= 5 && aux_takeoff_value == 1) || takeoff_detected == true){

			if(takeoff_detected == false){
				PX4_INFO("takeoff detected");
				//start counting
				takeoff_start_timestamp = gps.timestamp;
				takeoff_detected = true;
			}
			
			////////////////////////////////////////////////////////////////// GPS Software failures
			if(FaultType.find("Software") != std::string::npos){
				if (FaultTarget.find("GPS") != std::string::npos){
					
					if (!std::isnan(InjectionTimeStart) && !std::isnan(InjectionTimeEnd) && (gps.timestamp - takeoff_start_timestamp >= InjectionTimeStart) && (gps.timestamp - takeoff_start_timestamp <= InjectionTimeEnd)){

						if(FaultSubType.find("FixedValue") != std::string::npos){

							PX4_INFO("injecting gps fixed value");
							gps_output.lat = (int32_t) std::stod(Values[0]); 
							gps_output.lon = (int32_t) std::stod(Values[1]); 
							gps_output.alt = (int32_t) std::stod(Values[2]); 


						}else if(FaultSubType.find("FreezeValue") != std::string::npos){
							//Freeze Value
							PX4_INFO("injecting gps freeze value");
							if (!valueFreezed){
								injected_gps_lat = gps.lat;
								injected_gps_lon = gps.lon;
								injected_gps_alt = gps.alt;
								valueFreezed = true;
							}
							gps_output.lat = injected_gps_lat;
							gps_output.lon = injected_gps_lon;
							gps_output.alt = injected_gps_alt;


						}else if(FaultSubType.find("DelayValue") != std::string::npos){
							//Delay Value
							PX4_INFO("injecting gps delay value");
							std::this_thread::sleep_for(std::chrono::seconds((int)std::stod(Values[0])));
							gps_output.lat = gps.lat;
							gps_output.lon = gps.lon;
							gps_output.alt = gps.alt;

						}else if (FaultSubType.find("RandomValue") != std::string::npos){
							PX4_INFO("injecting gps random value");
							// (rand() % (max-min) ) + min
							gps_output.lat = std::stod(Values[0]) + rand() % (int)(std::stod(Values[1])-std::stod(Values[0]));
							gps_output.lon = std::stod(Values[2]) + rand() % (int)(std::stod(Values[3])-std::stod(Values[2]));
							gps_output.alt = std::stod(Values[4]) + rand() % (int)(std::stod(Values[5])-std::stod(Values[4]));

						
						}else if (FaultSubType.find("MinLatitude") != std::string::npos){
							gps_output.lat = -90;
							gps_output.lon = gps.lon;
							gps_output.alt = gps.alt;
						
						}else if (FaultSubType.find("MinLongtitude") != std::string::npos){
							gps_output.lat = gps.lat;
							gps_output.lon = -180;
							gps_output.alt = gps.alt;
						
						}else if (FaultSubType.find("MinAltitude") != std::string::npos){
							gps_output.lat = gps.lat;
							gps_output.lon = gps.lon;
							gps_output.alt = 0;
						
						}else if (FaultSubType.find("MaxLatitude") != std::string::npos){
							gps_output.lat = 90;
							gps_output.lon = gps.lon;
							gps_output.alt = gps.alt;
						
						}else if (FaultSubType.find("MaxLongtitude") != std::string::npos){
							gps_output.lat = gps.lat;
							gps_output.lon = 180;
							gps_output.alt = gps.alt;
						
						}else if (FaultSubType.find("MaxAltitude") != std::string::npos){
							gps_output.lat = gps.lat;
							gps_output.lon = gps.lon;
							gps_output.alt = 18000;
						
						}else{
							gps_output.lat = gps.lat;
							gps_output.lon = gps.lon;
							gps_output.alt = gps.alt;
						}

					}

					
				}else {
					gps_output.lat = gps.lat;
					gps_output.lon = gps.lon;
					gps_output.alt = gps.alt;
				}

				PX4_INFO("Software Failure Injected");

				PX4_INFO("Fault Sub Type %s", FaultSubType.c_str());
				PX4_INFO("Fault Target %s", FaultTarget.c_str());

				PX4_INFO("Injected start time is %f", InjectionTimeStart);
				PX4_INFO("Injected end time is %f", InjectionTimeEnd);

				PX4_INFO("Latitude Before Injection: %d", gps_output.lat);
				PX4_INFO("Longitude Before Injection: %d", gps_output.lon);
				PX4_INFO("Altitude Before Injection: %d", gps_output.alt);

				PX4_INFO("Latitude After Injection: %d", gps_output.lat);
				PX4_INFO("Longitude After Injection: %d", gps_output.lon);
				PX4_INFO("Altitude After Injection: %d", gps_output.alt);

			////////////////////////////////////////////////////////////////// GPS Security Issues: GPS Spoofing
			}else if (FaultType.find("Security") != std::string::npos){
				if (FaultTarget.find("GPS") != std::string::npos){

					if (!std::isnan(InjectionTimeStart) && !std::isnan(InjectionTimeEnd) && (gps.timestamp - takeoff_start_timestamp >= InjectionTimeStart) && (gps.timestamp - takeoff_start_timestamp <= InjectionTimeEnd)){

						if(FaultSubType.find("HijackByFixedPosition") != std::string::npos){

							//Fixed position of attacker is used to hijack a UAV
							PX4_INFO("injecting gps fixed value");
							gps_output.lat = (int32_t) std::stod(Values[0]); 
							gps_output.lon = (int32_t) std::stod(Values[1]); 
							gps_output.alt = (int32_t) std::stod(Values[2]); 


						}else if(FaultSubType.find("HijackByUAV") != std::string::npos){
							// assuming that the other UAV is close to the UAV
							gps_output.lat = gps.lat + std::stod(Values[0]);
							gps_output.lon = gps.lon + std::stod(Values[1]);
							gps_output.alt = gps.alt + std::stod(Values[2]);
							

						}else if(FaultSubType.find("DelayValue") != std::string::npos){
							//Delay Value
							//Delay Value
							PX4_INFO("injecting gps delay value");
							std::this_thread::sleep_for(std::chrono::seconds((int)std::stod(Values[0])));
							gps_output.lat = gps.lat;
							gps_output.lon = gps.lon;
							gps_output.alt = gps.alt;

						}else if (FaultSubType.find("RandomValue") != std::string::npos){
							PX4_INFO("injecting gps random value");
							// (rand() % (max-min) ) + min
							gps_output.lat = std::stod(Values[0]) + rand() % (int)(std::stod(Values[1])-std::stod(Values[0]));
							gps_output.lon = std::stod(Values[2]) + rand() % (int)(std::stod(Values[3])-std::stod(Values[2]));
							gps_output.alt = std::stod(Values[4]) + rand() % (int)(std::stod(Values[5])-std::stod(Values[4]));
						
						}else if (FaultSubType.find("ForceLanding") != std::string::npos){
							// tampers th altitude values with slightly higher values than real one, trying to force an unplanned landing
							gps_output.lat = gps.lat;
							gps_output.lon = gps.lon;
							gps_output.alt = gps.alt + 1; // 1 meter for instance

						
						}else if (FaultSubType.find("RandomLongtitude") != std::string::npos){ // -180   ... 180
							gps_output.lat = gps.lat;
							gps_output.lon = rand() % 360 -180 ;
							gps_output.alt = gps.alt;
						
						}else if (FaultSubType.find("RandomLatitude") != std::string::npos){ // -90     90
							gps_output.lat =  rand() % 180 -90 ;
							gps_output.lon = gps.lon;
							gps_output.alt = gps.alt;

						
						}else{
							gps_output.lat = gps.lat;
							gps_output.lon = gps.lon;
							gps_output.alt = gps.alt;
						}


					}
				
				}else {
					gps_output.lat = gps.lat;
					gps_output.lon = gps.lon;
					gps_output.alt = gps.alt;
				}


				PX4_INFO("Security Issues Injected");

				PX4_INFO("Fault Sub Type %s", FaultSubType.c_str());
				PX4_INFO("Fault Target %s", FaultTarget.c_str());

				PX4_INFO("Injected start time is %f", InjectionTimeStart);
				PX4_INFO("Injected end time is %f", InjectionTimeEnd);

				PX4_INFO("Latitude Before Injection: %d", gps_output.lat);
				PX4_INFO("Longitude Before Injection: %d", gps_output.lon);
				PX4_INFO("Altitude Before Injection: %d", gps_output.alt);

				PX4_INFO("Latitude After Injection: %d", gps_output.lat);
				PX4_INFO("Longitude After Injection: %d", gps_output.lon);
				PX4_INFO("Altitude After Injection: %d", gps_output.alt);




			} else {
				gps_output.lat = gps.lat;
				gps_output.lon = gps.lon;
				gps_output.alt = gps.alt;
			}

			
			/*}else{
				if(fault_mode == 1 && !std::isnan(start_injection_time) && !std::isnan(end_injection_time) && (gps.timestamp - takeoff_start_timestamp >= start_injection_time) && (gps.timestamp - takeoff_start_timestamp < end_injection_time)){
					//Fixed Value
					PX4_INFO("injecting gps fixed value");
					if(!std::isnan(injected_gps_lat))  gps_output.lat = (int32_t) injected_gps_lat; else gps_output.lat = gps.lat;
					if(!std::isnan(injected_gps_lon))  gps_output.lon = (int32_t) injected_gps_lon; else gps_output.lon = gps.lon;
					if(!std::isnan(injected_gps_alt))  gps_output.alt = (int32_t) injected_gps_alt; else gps_output.alt = gps.alt;
				}else if(fault_mode == 2 && !std::isnan(start_injection_time) && !std::isnan(end_injection_time) && (gps.timestamp - takeoff_start_timestamp >= start_injection_time) && (gps.timestamp - takeoff_start_timestamp <= end_injection_time)){
					//Freeze Value
					if(hasFreezeValue){
						PX4_INFO("injecting gps freeze value");
						gps_output.lat = injected_gps_lat;
						gps_output.lon = injected_gps_lon;
						gps_output.alt = injected_gps_alt;
					}else{
						PX4_INFO("Defining freeze value");
						//changed (double) on first three (int32_t in the bottom 3)
						injected_gps_lat = gps.lat;
						injected_gps_lon = gps.lon;
						injected_gps_alt = gps.alt;
						gps_output.lat = injected_gps_lat;
						gps_output.lon = injected_gps_lon;
						gps_output.alt = injected_gps_alt;
						hasFreezeValue = true;
					}
				}else if(fault_mode == 3 && !std::isnan(start_injection_time) && !std::isnan(end_injection_time) && (gps.timestamp - takeoff_start_timestamp >= start_injection_time) && (gps.timestamp - takeoff_start_timestamp <= end_injection_time)){
					//Delay Value
					//Don't send any data
					PX4_INFO("injecting gps delay value");
					gps_output.lat = gps.lat;
					gps_output.lon = gps.lon;
					gps_output.alt = gps.alt;
					std::this_thread::sleep_for(std::chrono::seconds((int)delay_value));
				}else{
					gps_output.lat = gps.lat;
					gps_output.lon = gps.lon;
					gps_output.alt = gps.alt;
					//PX4_INFO("GPS Timestamp: %" PRIu64 "", gps.timestamp);
					//PX4_INFO("Takeoff start timestamp: %" PRIu64 "", takeoff_start_timestamp);
				}
			} */	
		}else{
			gps_output.lat = gps.lat;
			gps_output.lon = gps.lon;
			gps_output.alt = gps.alt;
		}
		
		aux_takeoff_value = takeoff_value;
		//PX4_INFO("timestamp: %" PRIu64 "\n", gps.timestamp);
		//PX4_INFO("time utc usec: %" PRIu64 "\n", gps.time_utc_usec);
		/////////////////////////////////////////////////////////////////////////////////
	}else {

		gps_output.lat = gps.lat;
		gps_output.lon = gps.lon;
		gps_output.alt = gps.alt;
	}

	//////////////////////////////////////////////////////removed for fault injection
	//gps_output.timestamp = gps.timestamp;
	//gps_output.time_utc_usec = gps.time_utc_usec;
	//gps_output.lat = gps.lat;
	//gps_output.lon = gps.lon;
	//gps_output.alt = gps.alt;
        ///////////////////////////////////////////////////////////////////


	gps_output.alt_ellipsoid = gps.alt_ellipsoid;
	gps_output.s_variance_m_s = gps.s_variance_m_s;
	gps_output.c_variance_rad = gps.c_variance_rad;
	gps_output.eph = gps.eph;
	gps_output.epv = gps.epv;
	gps_output.hdop = gps.hdop;
	gps_output.vdop = gps.vdop;
	gps_output.noise_per_ms = gps.noise_per_ms;
	gps_output.jamming_indicator = gps.jamming_indicator;
	gps_output.jamming_state = gps.jamming_state;
	gps_output.vel_m_s = gps.vel_m_s;
	gps_output.vel_n_m_s = gps.vel_n_m_s;
	gps_output.vel_e_m_s = gps.vel_e_m_s;
	gps_output.vel_d_m_s = gps.vel_d_m_s;
	gps_output.cog_rad = gps.cog_rad;
	gps_output.timestamp_time_relative = gps.timestamp_time_relative;
	gps_output.heading = gps.heading;
	gps_output.heading_offset = gps.heading_offset;
	gps_output.fix_type = gps.fix_type;
	gps_output.vel_ned_valid = gps.vel_ned_valid;
	gps_output.satellites_used = gps.satellites_used;

	gps_output.selected = selected;

	_vehicle_gps_position_pub.publish(gps_output);
}

void VehicleGPSPosition::PrintStatus()
{
	//PX4_INFO("selected GPS: %d", _gps_select_index);
	


}

}; // namespace sensors
