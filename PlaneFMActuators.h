#ifndef __PlaneFMACTUATORS__
#define __PlaneFMACTUATORS__

#include "../stdafx.h"
#include <stdio.h>
#include <string>
#include <math.h>
#include "../UtilityFunctions.h"

namespace PlaneFM
{
	namespace ACTUATORS
	{
		double	elevatorPosition_DEG = 0.0;
		double	elevatorRate_DEGPERSEC = 0.0;
		double	aileronPosition_DEG = 0.0;
		double	aileronRate_DEGPERSEC = 0.0;
		double	flapPosition_DEG = 0.0;
		double	flapRate_DEGPERSEC = 0.0;
		double	throttle_state = 0.0;
		double	throttle_rate = 0.0;
		double	gear_state = 0.0;
		double	gear_rate = 0.0;
		double	airbrake_state = 0.0;
		double	airbrake_rate = 0.0;
		double  internal_fuel;

		bool	simInitialized = false;

		double  elevator_actuator(double elevator_DEG_commanded, double frameTime)
		{
			if (!simInitialized)
			{
				elevatorPosition_DEG = elevator_DEG_commanded;
				return elevatorPosition_DEG;
			}

			elevatorRate_DEGPERSEC = 20.2 * (elevator_DEG_commanded - elevatorPosition_DEG);

			elevatorRate_DEGPERSEC = limit(elevatorRate_DEGPERSEC, -0.5, 0.5);

			elevatorPosition_DEG += (elevatorRate_DEGPERSEC * frameTime);

			elevatorPosition_DEG = limit(elevatorPosition_DEG, -20.0, 20.0);

			return elevatorPosition_DEG;
		}

		double  aileron_actuator(double roll_cmd, double frameTime)
		{
			if (!simInitialized)
			{
				aileronPosition_DEG = roll_cmd;
				return aileronPosition_DEG;
			}

			aileronRate_DEGPERSEC = 15.0 * (roll_cmd - aileronPosition_DEG);

			aileronRate_DEGPERSEC = limit(aileronRate_DEGPERSEC, -20.0, 20.0);

			aileronPosition_DEG += (aileronRate_DEGPERSEC * frameTime);

			aileronPosition_DEG = limit(aileronPosition_DEG, -20.0, 20.0);

			return aileronPosition_DEG;
		}

		float	rudderPosition_DEG = 0.0;
		float	rudderRate_DEGPERSEC = 0.0;

		float  rudder_actuator(float rudderCommanded_DEG, double frameTime)
		{
			if (!simInitialized)
			{
				rudderPosition_DEG = rudderCommanded_DEG;
				return rudderPosition_DEG;
			}

			rudderRate_DEGPERSEC = 20.2 * (rudderCommanded_DEG - rudderPosition_DEG);

			rudderRate_DEGPERSEC = limit(rudderRate_DEGPERSEC, -3.0, 3.0);

			rudderPosition_DEG += (rudderRate_DEGPERSEC * frameTime);

			rudderPosition_DEG = limit(rudderPosition_DEG, -1.0, 1.0);

			return rudderPosition_DEG;
		}

		double  throttle_actuator(double throttleInput, double frameTime)
		{
			if (!simInitialized)
					{
						throttle_state = throttleInput;
						return throttle_state;
					}
					throttle_rate = 20.2 * (throttleInput - throttle_state);
					throttle_rate = limit(throttle_rate, -25.0, 20.0);
					throttle_state += (throttle_rate * frameTime);
					throttle_state = limit(throttle_state, 0.0, 100.0);
			return throttle_state;
		}
		double  gear_actuator(double GearCommand, double frameTime)
		{
			if (!simInitialized)
			{
				gear_state = GearCommand;
				return gear_state;
			}
			gear_rate = 20.2 * (GearCommand - gear_state);
			gear_rate = limit(gear_rate, -0.5, 0.5);
			gear_state += (gear_rate * frameTime);
			gear_state = limit(gear_state, 0.0, 100.0);

			return gear_state;
		}
		double  flaps_actuator(double flap_command, double frameTime)
		{
			if (!simInitialized)
			{
				flapPosition_DEG = flap_command;
				return flapPosition_DEG;
			}
			flapRate_DEGPERSEC = 20.2 * (flap_command - flapPosition_DEG);
			flapRate_DEGPERSEC = limit(flapRate_DEGPERSEC, -1.0, 1.0);
			flapPosition_DEG += (flapRate_DEGPERSEC * frameTime);
			flapPosition_DEG = limit(flapPosition_DEG, 0.0, 1.0);

			return flapPosition_DEG;
		}

		float elev_pos = 0.0;
		float elev_rate = 0.0;

		float  elev_actuator(float longStickInput, double frameTime)
		{
			if (!simInitialized)
			{
				elev_pos = longStickInput;
				return elev_pos;
			}

			elev_rate = 20.2 * (longStickInput - elev_pos);
			elev_rate = limit(elev_rate, -4.0, 4.0);
			elev_pos += (elev_rate * frameTime);
			elev_pos = limit(elev_pos, -1.0, 1.0);

			return elev_pos;
		}

		double  airbrake_actuator(double airbrake_command, double frameTime)
		{
			if (!simInitialized)
			{
				airbrake_state = airbrake_command;
				return airbrake_state;
			}
			airbrake_rate = 20.2 * (airbrake_command - airbrake_state);
			airbrake_rate = limit(airbrake_rate, -0.75, 0.75);
			airbrake_state += (airbrake_rate * frameTime);
			airbrake_state = limit(airbrake_state, 0.0, 1.0);

			return airbrake_state;
		}

		float misc_pos = 0.0;
		float misc_rate = 0.0;

		float  misc_actuator(float misc_cmd, double frameTime)
		{
			if (!simInitialized)
			{
				misc_pos = misc_cmd;
				return misc_pos;
			}
			misc_rate = 20.2 * (misc_cmd - misc_pos);
			misc_rate = limit(misc_rate, -0.5, 0.5);
			misc_pos += (misc_rate * frameTime);
			misc_pos = limit(misc_pos, 0.0, 1.0);

			return misc_pos;
		}
	};


}

#endif