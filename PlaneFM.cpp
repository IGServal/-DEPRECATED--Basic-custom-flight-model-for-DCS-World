//--------------------------------------------------------------------------
// External Flight Model for DCS World
// 
// Author: Aiyke/ Sérvalpilot
// Original author: CptSmiley 
// You can get the original source code at https://forums.eagle.ru/topic/79927-high-fidelity-flight-dynamics-and-techacademic-initial-demonstration/
//
// Use Only for Non-Commercial Purposes
// 
// This was originally meant for an F-16 module. I tried changing the namespaces
// to make it more general but it didn't work. Also, the original code used imperial
// units (feet, pounds, etc); again, I tried converting these to metric (sensible)
// units and it resulted in very weird flight behaviour.
//
//--------------------------------------------------------------------------
// F-16Demo.cpp : Defines the exported functions for the DLL application.
// Control the main portion of the discrete simulation event
//
// This project will compile a DLL.  This DLL needs to be compiled with the
// same machine type of your machine (x86 or x64).  This DLL then needs to
// be placed within the bin directory in your mod/aircraft/airplane name/ 
// directory within DCS World.  
//
// See associated entry.lua for how to tell the mod to use the DLL flight
// model
//--------------------------------------------------------------------------
// IMPORTANT!  COORDINATE CONVENTION:
//
// DCS WORLD Convention:
// Xbody: Out the front of the nose
// Ybody: Out the top of the aircraft
// Zbody: Out the right wing
//
// Normal Aerodynamics/Control Convention:
// Xbody: Out the front of the nose
// Ybody: Out the right wing
// Zbody: Out the bottom of the aircraft (???)
//
// This means that if you are referincing from any aerodynamic, stabilty, or control document
// they are probably using the second set of directions.  Which means you always need to switch
// the Y and the Z and reverse the Y prior to output to DCS World
//---------------------------------------------------------------------------
// TODO List:
// - Left/right engine stuff
//---------------------------------------------------------------------------
// KNOWN Issues:
//---------------------------------------------------------------------------
#include "stdafx.h"
#include "PlaneFM.h"
#include "ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include <Math.h>
#include <stdio.h>
#include <string>
#include <math.h>
#include "UtilityFunctions.h"	// Utility help functions
#include "Inputs.h"

#include "include/Cockpit/CockpitAPI_Declare.h" // Provides param handle interfacing for use in lua
#include "include/FM/API_Declare.h"	// Provides all DCS related functions in this cpp file
// Model headers
#include "Actuators/PlaneFMActuators.h"				//Actuators model functions
#include "Atmosphere/PlaneFMAtmosphere.h"			//Atmosphere model functions
#include "Aerodynamics/PlaneFMAero.h"				//Aerodynamic model functions
#include "FlightControls/FlightControls.h"	//Flight Controls model functions
#include "Engine/PlaneFMEngine.h"					//Engine model functions
#include "param_functions.h"

using namespace PlaneFM;

//-----------------------------------------------------------------
// This variable is very important.  Make sure you set this
// to 0 at the beginning of each frame time or else the moments
// will accumulate.  For each frame time, add up all the moments
// acting on the air vehicle with this variable using th
//
// Units = Newton * meter
//-----------------------------------------------------------------
Vec3	common_moment;			

Vec3	common_force;

Vec3    center_of_gravity;

Vec3	inertia;

Vec3	wind;

Vec3	velocity_world_cs;

//-------------------------------------------------------
// Start of aircraft Simulation Variables
//-------------------------------------------------------
namespace PlaneFM // I tried to convert the imperial units to metric, but it resulted in very bizarre behaviour.
{
	double		meterToFoot	= 3.28084;					// Meter to foot conversion factor
	double		ambientTemperature_DegK = 0.0;			// Ambient temperature (kelvin)
	double		ambientDensity_KgPerM3	= 0.0;			// Ambient density (kg/m^3)
	double		wingSpan_FT				= 32.667;		// F-16 wing-span (ft)
	double		wingArea_FT2			= 300.0;		// F-16 wing area (ft^2)
	double		meanChord_FT			= 11.32;		// F-16 mean aerodynamic chord (ft)
	double		referenceCG_PCT			= 0.35;			// Reference center of mass as a % of wing chord
	double		actualCG_PCT			= 0.30;			// Actual center of mass as a % of wing chord
	double		pi						= acos(-1.0);	// Pi (3.14159....)
	double		radiansToDegrees		= 180.0/pi;		// Conversion factor from radians to degrees
	double		inertia_Ix_KGM2			= 12874.0;		// Reference moment of inertia (kg/m^2)
	double		inertia_Iy_KGM2			= 75673.6;		// Reference moment of inertia (kg/m^2)
	double		inertia_Iz_KGM2			= 85552.1;		// Reference moment of inertia (kg/m^2)
	double		temp[9];								// Temporary array for holding look-up table results
	double		altitude_m				= 0.0;			// Absolute altitude above sea level (metres)
	double		altitude_FT				= 0.0;			// Absolute altitude above sea level (ft)
	double		totalVelocity_FPS		= 0.0;			// Total velocity (always positive) (ft/s)
	double		totalVelocity_MS		= 0.0;			// Total velocity (m/s)
	double		Forward_Velocity		= 0.0;			// Total forward velocity (m/s)
	double		alpha_DEG				= 0.0;			// Angle of attack (deg)
	double		beta_DEG				= 0.0;			// Slideslip angle (deg)
	double		rollRate_RPS			= 0.0;			// Body roll rate (rad/sec)
	double		rollspeed				= 0.0;			// Absolute roll rate
	double		pitchRate_RPS			= 0.0;			// Body pitch rate (rad/sec)
	double		yawRate_RPS				= 0.0;			// Body yaw rate (rad/sec)
	double		thrust_N				= 0.0;			// Engine thrust (Newtons)

	double		elevator_DEG			= 0.0;			// Elevator deflection (deg)
	double		aileron_DEG				= 0.0;			// Aileron deflection (deg)
	double		rudder_DEG				= 0.0;			// Rudder deflection (deg)
	double		elevator_DEG_commanded	= 0.0;			// Commanded elevator deflection from control system (deg)
	double		aileron_DEG_commanded	= 0.0;			// Commanded aileron deflection from control system (deg)
	double		rudder_DEG_commanded	= 0.0;			// Commanded rudder deflection from control system (deg)
	double		pitchTrim				= 0.0;			// Pitch trim
	double		rollTrim				= 0.0;			// Roll trim
	double		yawTrim					= 0.0;			// Yaw trim
	double		pitch_cmd				= 0.0;			// Elevator/stabilator command
	double		roll_cmd				= 0.0;			// Aileron command
	double		yaw_cmd					= 0.0;			// Rudder command
	double		throttle_state			= 0.2;			// Engine power state
	double		pedInput				= 0.0;			// Rudder pedal input command normalized (-1 to 1)
	double		throttleInput			= 0.2;			// Throttle input command normalized (-1 to 1)
	double		flap_DEG				= 0.0;			// Trailing edge flap deflection (deg)
	double		flap_PCT				= 0.0;			// Trailing edge flap deflection (0 to 1)
	double		aileron_PCT				= 0.0;			// Aileron deflection as a percent of maximum (-1 to 1)
	double		rudder_PCT				= 0.0;			// Rudder deflection as a percent of maximum (-1 to 1)
	double		elevator_PCT			= 0.0;			// Elevator deflection as a percent of maximum (-1 to 1)
	float		elev_pos				= 0.0;			// Elevator/stabilator deflection
	double		leadingEdgeFlap_DEG		= 0.0;			// Leading edge slat deflection (deg)
	double		leadingEdgeFlap_PCT		= 0.0;			// Leading edge slat deflection as a percent of maximum (0 to 1)

	float roll_pos = 0.0;
	float pitch_filter = 0.0;
	float roll_filter = 0.0;
	float yaw_filter = 0.0;
	int yaw_left = 0;
	int yaw_right = 0;
	int yaw_reversing = 0; // Is yaw going against input?

	double		pitch_cmd_discrete = 0.0;		// Discrete Elevator/stabilator command input (button)
	double		roll_cmd_discrete = 0.0;		// Discrete Aileron command input (button)
	double		yaw_cmd_discrete = 0.0;			// Discrete Rudder command input (button)
	double		yaw_combined = 0.0;
	double		yaw_cmd_axis = 0.0;				// Continuous rudder command (analog)
	double		pitch_cmd_axis = 0.0;			// Continuous elevator/stabilator command (analog)
	double		roll_cmd_axis = 0.0;			// Continuous aileron command (analog)
	double		roll_cmd_axis_norm = 0.0;		// Normalised continuous aileron command (analog)

	double		dynamicPressure_LBFT2	= 0.0;			// Dynamic pressure (lb/ft^2)
	double		mach					= 0.0;			// Air speed in Mach; 1 is the local speed of sound.
	double		ps_LBFT2				= 0.0;			// Ambient calculated pressure (lb/ft^2)
	bool		simInitialized			= false;		// Has the simulation gone through it's first run frame?
	double		gearDown				= 0.0;			// Is the gear currently down?
	double		gforce					= 0.0;			// This is the G force felt by the pilot, acting out the bottom of the aircraft (m/s^2), 1 is Earth's gravity.
	double		az						= 0.0;			// This is the G force felt by the pilot, acting out the bottom of the aircraft (m/s^2), 1 is Earth's gravity.
	double		ay						= 0.0;			// Ay (per normal direction convention) out the right wing (m/s^2)
	double		weight_N				= 0.0;			// Weight force of aircraft (N)
	double		ay_world				= 0.0;			// World referenced up/down acceleration (m/s^2)
	double		vy_world				= 0.0;			// World referenced up/down velocity (m/s), in orther words, vertical speed
	double		weight_on_wheels		= 0.0;			// Is there weight on the wheels? This checks if the plane is on the ground.

	double		rolling_friction		= 0.015;			// Wheel friction amount, I don't know what units. I don't know what this is exactly.
	double		WheelBrakeCommand		= 0.0;			// Commanded wheel brake
	double		GearCommand				= 0.0;			// Commanded gear lever
	double		airbrake_command		= 0.0;			// Air brakes/spoiler command
	double		airbrakes				= 0.0;			// Are the air brakes/spoilers deployed?
	double		flap_command			= 0.0;			// flaps command
	double		flaps					= 0.0;			// Are the flaps down?
	double		starter_state			= 0.0;			// Engine starter
	double		starter_command			= 0.0;			// Engine starter
	float		rudder_pos				= 0.0;			// Rudder(s) deflection
	float		misc_cmd				= 0.0;			// Misc actuator command either for tail hooks or weapon bays (F-22, Su-57, etc.)
	float		misc_state				= 0.0;

	double engine_damage = 0;			// Combined left and right engine damage
	double Lwing_damage = 0;			// Left wing damage
	double Rwing_damage = 0;			// Right wing damage
	double wing_damage = (Rwing_damage + Lwing_damage); // Combined wing damage
	double tail_damage = 0;				// Tail(s) and rudder(s) damage
	double cockpit_damage = 0;			// Cockpit avionics and pilot damage	
	float total_damage = 0;				// Combined damage, because individual damage effects don't seem to work.

	double		pitch_angle = 0.0;		// Pitch angle relateive to the horizon in degrees, -90 to +90.
	double		roll_angle = 0.0;		// Roll angle relateive to the horizon in degrees, -90 to +90.
	double		vspeed = 0.0;			// Vertical speed in metres per second.
	double		vx_world = 0;
	double		vz_world = 0;
	double		ax_body = 0;
	double		ay_body = 0;
	double		az_body = 0;
	double		vx_body = 0;
	double		vy_body = 0;
	double		vz_body = 0;
	double		roll_acc = 0.0;
	double		yawRate_world = 0.0;	// Yaw rate relative to the Earth.
	double		omegax_world = 0;
	double		omegaz_world = 0;

	int			alt_hold = 0;			// Autopilot altitude hold
	int			altroll_hold = 0;		// Autopilot altitude and bank hold
	int			horiz_hold = 0;			// Autopilot horizon hold
	
	double		DeltaTime				= 0.0;			// Delta time of the simulation, in seconds. I still haven't figured out how this works exactly
	bool		engineswitch			= false;		// Is the engine(s) on? If there are two engines they are treated as one.
	float		starter_switch			= 0.0;
	double		fuel_consumption_since_last_time = 0;
	double		internal_fuel;
	double		external_fuel;

	//double test_force = 0.0; //test downwards pitch or "nose down" force
	
	EDPARAM cockpitAPI;
	param_stuff param_class;
}

// Very important! This function sum up all the forces acting on
// the aircraft for this run frame.  It currently assume the force
// is acting at the center of mass.
void add_local_force(const Vec3 & Force, const Vec3 & Force_pos)
{
	common_force.x += Force.x;
	common_force.y += Force.y;
	common_force.z += Force.z;
}

// This is where the simulation send the accumulated forces to the DCS Simulation
// after each run frame
void ed_fm_add_local_force(double& x, double& y, double& z, double& pos_x, double& pos_y, double& pos_z)
{
	x = common_force.x;
	y = common_force.y;
	z = common_force.z;
	pos_x = center_of_gravity.x;
	pos_y = center_of_gravity.y;
	pos_z = center_of_gravity.z;
}

// Very important! This function sums up all the moments acting
// on the aircraft for this run frame.  It currently assumes the
// moment is acting at the center of mass.
void add_local_moment(const Vec3 & Moment)
{
	common_moment.x += Moment.x;
	common_moment.y += Moment.y;
	common_moment.z += Moment.z;
}


// Earth's equitorial circumference is around 40075.017 Km.
// This is where the simulation send the accumulated moments to the DCS Simulation
// after each run frame
void ed_fm_add_local_moment(double& x, double& y, double& z)
{						// Figure out what these represent (R,P,Y)
	x = common_moment.x;
	y = common_moment.y;
	z = common_moment.z;
}

//Fuel consumption
void simulate_fuel_consumption(double dt) // This doesn't seem to work. 

{
	PlaneFM::fuel_consumption_since_last_time =  10 * (PlaneFM::throttleInput * 1000) * engine_damage * dt; //10 kg persecond
		if (PlaneFM::fuel_consumption_since_last_time > internal_fuel)
			PlaneFM::fuel_consumption_since_last_time = internal_fuel;
		internal_fuel -= PlaneFM::fuel_consumption_since_last_time +10 +(10* throttleInput);
}

// Not used
void ed_fm_add_global_force(double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z)
{}

// Not used
void ed_fm_add_global_moment(double & x,double &y,double &z)
{
}

float aoa_filter = 0.0;
float aos_filter = 0.0;

//-----------------------------------------------------------------------
// The most important part of the entire EFM code.  This is where you code
// gets called for each run frame.  Each run frame last for a duration of
// "dt" (delta time).  This can be used to help time certain features such
// as filters and lags.
// dt is 6 milliseconds.
//-----------------------------------------------------------------------

void ed_fm_simulate(double dt)
{
	PlaneFM::DeltaTime = dt;


	/*
	if(PlaneFM::weight_on_wheels)
	{
		PlaneFM::alpha_DEG = 0.0;
		PlaneFM::az = 0.0;
	}
	*/

	// Very important! clear out the forces and moments before you start calculated
	// a new set for this run frame
	common_force = Vec3();
	common_moment = Vec3();

	// Get the total absolute velocity acting on the aircraft with wind included
	Vec3 airspeed;
	airspeed.x = velocity_world_cs.x - wind.x;
	airspeed.y = velocity_world_cs.y - wind.y;
	airspeed.z = velocity_world_cs.z - wind.z;

	// Total airspeed in meters per second because I'm stubborn like that.
	PlaneFM::totalVelocity_MS = sqrt(airspeed.x * airspeed.x + airspeed.y * airspeed.y + airspeed.z * airspeed.z);
	if (PlaneFM::totalVelocity_MS < 0.01)
	{
		PlaneFM::totalVelocity_MS = 0.01;
	}

	PlaneFM::totalVelocity_FPS = sqrt(airspeed.x * airspeed.x + airspeed.y * airspeed.y + airspeed.z * airspeed.z) * PlaneFM::meterToFoot;
	if (PlaneFM::totalVelocity_FPS < 0.01)
	{
		PlaneFM::totalVelocity_FPS = 0.01;
	}

	PlaneFM::Forward_Velocity = airspeed.x; // Total forward speed (m/s)

	// Call the atmosphere model to get mach and dynamic pressure
	// This was originally programmed with imperial units, so LB/FT^2 for the pressure.
	// I tried changing the units to the sensible system (metric), but the result was terrible.
	double* temp;
	temp = (double*)malloc(9 * sizeof(double));
	PlaneFM::ATMOS::atmos(PlaneFM::ambientTemperature_DegK, PlaneFM::ambientDensity_KgPerM3, PlaneFM::totalVelocity_FPS, temp);
	PlaneFM::dynamicPressure_LBFT2 = temp[0];
	PlaneFM::mach = temp[1];

	//---------------------------------------------
	//-----CONTROL DYNAMICS------------------------
	//---------------------------------------------
	// 

	// Autopilot
	if (cockpit_damage < 0.1)
	{
		if (alt_hold == 1) // Hold altitude
		{
			if (Forward_Velocity <= 250)
				pitchTrim = limit(vspeed, -5, 5) + (pitch_angle / 2);
			else
				pitchTrim = limit((vspeed), -5, 5) + (pitch_angle / 5); // FIX ALTITUDE HOLD!!
		};

		if (altroll_hold == 1) // Keep level and altitude
		{
			if (Forward_Velocity <= 250)
				pitchTrim = limit(vspeed, -5, 5) + (pitch_angle / 2);
			rollTrim = roll_angle / 2;
			yawTrim = yawRate_world;

			if (Forward_Velocity > 250)
				pitchTrim = limit((vspeed), -5, 5) + (pitch_angle / 5);
			rollTrim = roll_angle / 2;
			yawTrim = yawRate_world;
		}

		if (horiz_hold == 1) // Keep level to the horizon
		{
			pitchTrim = (pitch_angle * 2);
			rollTrim = roll_angle / 2;
			yawTrim = yawRate_world;
		}
	}
	if (cockpit_damage > 1)
	{
		PlaneFM::engine_damage = 0;
		PlaneFM::Lwing_damage = 0;
		PlaneFM::Rwing_damage = 0;
		PlaneFM::tail_damage = 0;
	}

	//	Fuel system

	if (engine_damage < 1)
		PlaneFM::fuel_consumption_since_last_time = ((PlaneFM::thrust_N / 40000) * dt * (engine_damage / 2));
	else
		PlaneFM::fuel_consumption_since_last_time = ((PlaneFM::thrust_N / 20000) * dt);

	PlaneFM::internal_fuel -= (PlaneFM::fuel_consumption_since_last_time)*PlaneFM::param_class.fuelvalue;

	// Leading edge flap/slat dynamics controller, this controller is based on dynamic pressure and angle of attack and is completely automatic
	PlaneFM::leadingEdgeFlap_DEG = PlaneFM::FLIGHTCONTROLS::leading_edge_flap_controller(PlaneFM::alpha_DEG, PlaneFM::dynamicPressure_LBFT2, PlaneFM::ps_LBFT2, dt);
	PlaneFM::leadingEdgeFlap_PCT = limit(PlaneFM::leadingEdgeFlap_DEG / 25.0, 0.0, 1.0);

	// These controls take the following inputs:
	// -Normalized longitudinal stick input
	// -Trimmed G offset
	// -Angle of attack (deg)
	// -Pitch rate (rad/sec)
	// -Experimental hard input limiter.

	// Stuff for added stability
	double pitch_combined = (pow((PlaneFM::pitch_cmd_discrete + (PlaneFM::pitch_cmd_axis * 2)), 2) / 4);
	double roll_axis_boost = (((sin(roll_cmd_axis_norm * 2) * 2) / (25 * roll_cmd_axis_norm) * 15) / roll_cmd_axis_norm * 2) / 5;
	double roll_combined = (pow((PlaneFM::roll_cmd_discrete + roll_cmd_axis), 2) / 4);

	 yaw_combined = (pow((PlaneFM::yaw_cmd_discrete + (PlaneFM::yaw_cmd_axis * 2)), 2) / 4);

	if (pitch_combined < 0.25 && wing_damage < 0.5)
		pitch_filter = (pitchRate_RPS * 25) * (1 - pitch_combined);
	else pitch_filter = 0;

	PlaneFM::elevator_DEG_commanded = -(PlaneFM::FLIGHTCONTROLS::fcs_pitch_controller(PlaneFM::FLIGHTCONTROLS::longStickInput, -1.0, PlaneFM::alpha_DEG, PlaneFM::pitchRate_RPS * PlaneFM::radiansToDegrees, (PlaneFM::az / 9.81), 0.0, PlaneFM::dynamicPressure_LBFT2, dt));
	if (PlaneFM::FLIGHTCONTROLS::longStickInput >= 0.01) // Pitch down
	// This is to make the controls a bit more responsive, especially when pitching the nose down.
		PlaneFM::elevator_DEG = PlaneFM::elevator_DEG_commanded + (PlaneFM::pitchTrim + (PlaneFM::FLIGHTCONTROLS::longStickInput * 25)) + pitch_filter - (PlaneFM::ACTUATORS::flapPosition_DEG * 5);
	
	if (PlaneFM::FLIGHTCONTROLS::longStickInput < 0.01 && gforce < 0) // Pitch up
		PlaneFM::elevator_DEG = PlaneFM::elevator_DEG_commanded + (PlaneFM::pitchTrim + (PlaneFM::FLIGHTCONTROLS::longStickInput * 15)) + pitch_filter - (PlaneFM::ACTUATORS::flapPosition_DEG * 5);
	if (PlaneFM::FLIGHTCONTROLS::longStickInput < 0.01 && gforce > 0) // Pitch up
		PlaneFM::elevator_DEG = PlaneFM::elevator_DEG_commanded + (PlaneFM::pitchTrim + (PlaneFM::FLIGHTCONTROLS::longStickInput * 15) / (1 + PlaneFM::gforce / 5)) + pitch_filter - (PlaneFM::ACTUATORS::flapPosition_DEG * 5);

	//{ For some reason, changing this causes DCS to crash.
	PlaneFM::elevator_DEG = limit(PlaneFM::elevator_DEG, -12.0, 12.0);

	if (roll_cmd_discrete < 0.5 && roll_cmd_axis_norm < 0.1 && cockpit_damage < 1)
		roll_filter = (rollRate_RPS * 30) / (1 + roll_combined) + (roll_cmd_axis * 100);
	else
		roll_filter = 0.0;

	PlaneFM::aileron_DEG_commanded = (PlaneFM::FLIGHTCONTROLS::fcs_roll_controller(PlaneFM::FLIGHTCONTROLS::latStickInput, PlaneFM::FLIGHTCONTROLS::longStickForce, PlaneFM::ay / 9.81, PlaneFM::rollRate_RPS * PlaneFM::radiansToDegrees, 0.0, PlaneFM::dynamicPressure_LBFT2, dt)) + (total_damage * 18446744073709551615);
	PlaneFM::aileron_DEG = PlaneFM::aileron_DEG_commanded + PlaneFM::rollTrim + roll_filter + (wing_damage);
	PlaneFM::aileron_DEG = limit(PlaneFM::aileron_DEG, -30, 30);

	if (yaw_combined < 0.25 && tail_damage == 0.0)
		yaw_filter = (yawRate_RPS * 30) * (1 - yaw_combined) + (beta_DEG + yawRate_RPS);
	else yaw_filter = 0;

	// Yaw anti-reversal filter

	int yaw_anti_reverse_left = 0.0;
	int yaw_anti_reverse_right = 0.0;

	if (PlaneFM::yaw_left == 1 && yawRate_RPS < 0.017453 && tail_damage == 0.0)
		yaw_anti_reverse_left += 100000000000;
	else yaw_anti_reverse_left = 0;

	if (PlaneFM::yaw_right == 1 && yawRate_RPS < 0.017453 && tail_damage == 0.0)
		yaw_anti_reverse_right -= 100000000000;
	else yaw_anti_reverse_right = 0;

	PlaneFM::rudder_DEG_commanded = PlaneFM::FLIGHTCONTROLS::fcs_yaw_controller(PlaneFM::pedInput, 0.0, PlaneFM::yawRate_RPS * (180.0 / 3.14159), ((PlaneFM::rollRate_RPS * PlaneFM::radiansToDegrees) / 45),
		PlaneFM::FLIGHTCONTROLS::alphaFiltered, PlaneFM::aileron_DEG_commanded, PlaneFM::ay / 1.56, dt);
	PlaneFM::rudder_DEG = PlaneFM::rudder_DEG_commanded + yaw_filter + PlaneFM::yawTrim + yaw_anti_reverse_left + yaw_anti_reverse_right * ( 1+ total_damage * 18446744073709551615);
	PlaneFM::rudder_DEG = limit(PlaneFM::rudder_DEG, -5 * (1 + wing_damage), 5 * (1 + wing_damage));

	PlaneFM::flap_DEG = PlaneFM::FLIGHTCONTROLS::fcs_flap_controller(PlaneFM::totalVelocity_FPS);

	PlaneFM::elev_pos = PlaneFM::ACTUATORS::elev_actuator(PlaneFM::FLIGHTCONTROLS::longStickInput + (PlaneFM::pitchTrim / 15), dt);

	PlaneFM::rudder_pos = PlaneFM::ACTUATORS::rudder_actuator(PlaneFM::pedInput, dt);

	PlaneFM::roll_pos = PlaneFM::ACTUATORS::aileron_actuator(PlaneFM::roll_cmd, dt);

	PlaneFM::gearDown = PlaneFM::ACTUATORS::gear_actuator(PlaneFM::GearCommand, dt);

	PlaneFM::airbrakes = PlaneFM::ACTUATORS::airbrake_actuator(PlaneFM::airbrake_command, dt);

	PlaneFM::flaps = PlaneFM::ACTUATORS::flaps_actuator(PlaneFM::flap_command, dt);

	PlaneFM::misc_state = PlaneFM::ACTUATORS::misc_actuator(PlaneFM::misc_cmd, dt);

	PlaneFM::starter_state = PlaneFM::ACTUATORS::engine_starter(PlaneFM::starter_command, dt);

	if (PlaneFM::ACTUATORS::starter_state >= 0.49)
	{
		PlaneFM::engineswitch = 1;
	}
	else PlaneFM::engineswitch = 0;

	if (PlaneFM::total_damage > 0.0)
	{
		PlaneFM::starter_command = 0;
	};

	//Throttle and thrust
	PlaneFM::throttle_state = PlaneFM::ACTUATORS::throttle_actuator(PlaneFM::throttleInput, dt) / (1 + PlaneFM::engine_damage);

	if (PlaneFM::throttleInput <= 64.9) //This is to make sure the plane holds still while at idle on the ground.
	{
		PlaneFM::thrust_N = PlaneFM::ENGINE::engine_dynamics((PlaneFM::throttleInput - 25), PlaneFM::mach, PlaneFM::altitude_FT, dt) / 1.2 + (engine_damage * 10);
	}

	if (PlaneFM::throttle_state >= 89.9 && PlaneFM::gearDown >= 0.25) // I can't think of a better way of getting ground acceleration more realistic.
	{
		PlaneFM::thrust_N = PlaneFM::ENGINE::engine_dynamics(PlaneFM::throttleInput, PlaneFM::mach, PlaneFM::altitude_FT, dt) / 1.125 + (engine_damage * 10);
	}

	if (PlaneFM::throttle_state > 65.0 && PlaneFM::throttle_state < 89.9)
	{
		PlaneFM::thrust_N = PlaneFM::ENGINE::engine_dynamics((PlaneFM::throttleInput - 25), PlaneFM::mach, PlaneFM::altitude_FT, dt) * 1.14 / (1+ total_damage * 10);
	}

	if (PlaneFM::throttle_state >= 90.0 && PlaneFM::gearDown >= 0.26 && weight_on_wheels == true) //simulating the increased thrust from afterburners.
	{
		PlaneFM::thrust_N = PlaneFM::ENGINE::engine_dynamics(PlaneFM::throttleInput, PlaneFM::mach, PlaneFM::altitude_FT, dt) * 1.1 / (1 + total_damage * 15);
	}
	if (PlaneFM::throttle_state >= 95.0 && PlaneFM::gearDown <= 0.26) //simulating the increased thrust from afterburners.
	{
		PlaneFM::thrust_N = PlaneFM::ENGINE::engine_dynamics(PlaneFM::throttleInput, PlaneFM::mach, PlaneFM::altitude_FT, dt) * 1.4 / (1 + total_damage * 20);
	}
	if (PlaneFM::internal_fuel < 5.0)
	PlaneFM::thrust_N = 0;

	PlaneFM::aileron_PCT = (PlaneFM::aileron_DEG + ((Lwing_damage * 2) + (Rwing_damage * 2))) / (25.5 + wing_damage);
	PlaneFM::elevator_PCT = PlaneFM::elevator_DEG / 25.0 / 1 + (wing_damage * 10);
	PlaneFM::rudder_PCT = (PlaneFM::rudder_DEG - (tail_damage)) / (30.0 + (tail_damage * 10));
	PlaneFM::flap_PCT = PlaneFM::flap_DEG / 20.0;

	// Aerodynamics stuff

	double alpha1_DEG_Limited = limit(PlaneFM::alpha_DEG, -20.0, 90.0);
	double beta1_DEG_Limited = limit(PlaneFM::beta_DEG, -30.0, 30.0);

	// Flaps

	double CLFlaps = 0.175 * PlaneFM::ACTUATORS::flapPosition_DEG;
	double CDFlaps = 0.025 * PlaneFM::ACTUATORS::flapPosition_DEG;
	double CzFlaps = -(CLFlaps * cos(PlaneFM::alpha_DEG * (PlaneFM::pi / 180.0)) + (CDFlaps * sin(PlaneFM::pi / 180.0)) / 10);
	double CxFlaps = -(-CLFlaps * sin(PlaneFM::alpha_DEG * (PlaneFM::pi / 180.0)) + (CDFlaps * cos(PlaneFM::pi / 180.0)) / 10);

	// Air brakes aero 

	double CDbrakes = 0.05 * (PlaneFM::ACTUATORS::airbrake_state);
	double Cxbrakes = -(CDbrakes * cos(PlaneFM::pi / 180.0) * PlaneFM::ACTUATORS::airbrake_state);

	// Gear aero
	double CDGear = (0.027 + WheelBrakeCommand) * PlaneFM::gearDown * 1.1;
	double CzGear = -(CDGear * sin(PlaneFM::pi / 180.0));
	double CxGear = -(CDGear * cos(PlaneFM::pi / 180.0));

	// These are the individual coefficients for the aerodynamics calculations.
	// Modifying these can help fine-tune flight behaviour. It's not realistic, but it works.

	PlaneFM::AERO::hifi_C(alpha1_DEG_Limited, beta1_DEG_Limited, PlaneFM::elevator_DEG, temp);
	PlaneFM::AERO::Cx = temp[0] * (1 + total_damage / 100); // I think this is drag when AOA is low.
	PlaneFM::AERO::Cz = temp[1];
	PlaneFM::AERO::Cm = temp[2];
	PlaneFM::AERO::Cy = temp[3];
	PlaneFM::AERO::Cn = temp[4] / (1 + (5 * PlaneFM::yaw_combined)); // This helps in yaw stability
	PlaneFM::AERO::Cl = temp[5];

	PlaneFM::AERO::hifi_damping(alpha1_DEG_Limited, temp);
	PlaneFM::AERO::Cxq = temp[0]; // This one's weird. It seems to turn angular momentum into speed.
	PlaneFM::AERO::Cyr = temp[1];
	PlaneFM::AERO::Cyp = temp[2];
	if (alpha_DEG > 0.01 || Lwing_damage < 0.2 || Rwing_damage < 0.2)
	{					// This might be a "brute force" approach to stall prevention, but that's because I couldn't get anything else to work.
		PlaneFM::AERO::Czq = temp[3] * limit(((alpha_DEG * alpha_DEG / 10 + (10 * PlaneFM::ACTUATORS::flapPosition_DEG)) * (Forward_Velocity / 2) + 1), 1, (10 + (PlaneFM::ACTUATORS::flapPosition_DEG / 5)));
	}
	else
		PlaneFM::AERO::Czq = temp[3] / (1 + total_damage / 100); // This is related to lift (force up from the dorsal side)
	PlaneFM::AERO::Clr = temp[4];
	PlaneFM::AERO::Clp = temp[5];
	PlaneFM::AERO::Cmq = temp[6];
	PlaneFM::AERO::Cnr = temp[7];
	PlaneFM::AERO::Cnp = temp[8];

	PlaneFM::AERO::hifi_C_lef(alpha1_DEG_Limited, beta1_DEG_Limited, temp);
	PlaneFM::AERO::Cx_delta_lef = temp[0];
	PlaneFM::AERO::Cz_delta_lef = temp[1];
	PlaneFM::AERO::Cm_delta_lef = temp[2];
	PlaneFM::AERO::Cy_delta_lef = temp[3];
	PlaneFM::AERO::Cn_delta_lef = temp[4];
	PlaneFM::AERO::Cl_delta_lef = temp[5];

	PlaneFM::AERO::hifi_damping_lef(alpha1_DEG_Limited, temp);
	PlaneFM::AERO::Cxq_delta_lef = temp[0];
	PlaneFM::AERO::Cyr_delta_lef = temp[1];
	PlaneFM::AERO::Cyp_delta_lef = temp[2];
	PlaneFM::AERO::Czq_delta_lef = temp[3];
	PlaneFM::AERO::Clr_delta_lef = temp[4];
	PlaneFM::AERO::Clp_delta_lef = temp[5];
	PlaneFM::AERO::Cmq_delta_lef = temp[6];
	PlaneFM::AERO::Cnr_delta_lef = temp[7];
	PlaneFM::AERO::Cnp_delta_lef = temp[8];

	PlaneFM::AERO::hifi_rudder(alpha1_DEG_Limited, beta1_DEG_Limited, temp);
	PlaneFM::AERO::Cy_delta_r30 = temp[0];
	PlaneFM::AERO::Cn_delta_r30 = temp[1] / (1 + total_damage / 100); // This seems to be the yaw damping effect 
	PlaneFM::AERO::Cl_delta_r30 = temp[2] / (1 + yaw_combined * 2); // This seems to translate yaw into roll

	PlaneFM::AERO::hifi_ailerons(alpha1_DEG_Limited, beta1_DEG_Limited, temp);
	PlaneFM::AERO::Cy_delta_a20 = temp[0] / (1 + total_damage / 100);
	PlaneFM::AERO::Cy_delta_a20_lef = temp[1];
	PlaneFM::AERO::Cn_delta_a20 = temp[2];
	PlaneFM::AERO::Cn_delta_a20_lef = temp[3];
	PlaneFM::AERO::Cl_delta_a20 = temp[4];
	PlaneFM::AERO::Cl_delta_a20_lef = temp[5];

	PlaneFM::AERO::hifi_other_coeffs(alpha1_DEG_Limited, PlaneFM::elevator_DEG, temp);
	PlaneFM::AERO::Cn_delta_beta = temp[0];
	PlaneFM::AERO::Cl_delta_beta = temp[1];
	PlaneFM::AERO::Cm_delta = temp[2];
	PlaneFM::AERO::eta_el = temp[3];
	PlaneFM::AERO::Cm_delta_ds = 0;        // ignore deep-stall effect
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
compute Cx_tot, Cz_tot, Cm_tot, Cy_tot, Cn_tot, and Cl_total
(as on NASA report p37-40)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* XXXXXXXX Cx_tot XXXXXXXX */
// Cx is drag
	PlaneFM::AERO::dXdQ = (PlaneFM::meanChord_FT / (2 * PlaneFM::totalVelocity_FPS)) * (PlaneFM::AERO::Cxq + PlaneFM::AERO::Cxq_delta_lef * PlaneFM::leadingEdgeFlap_PCT);

	PlaneFM::AERO::Cx_total = PlaneFM::AERO::Cx + PlaneFM::AERO::Cx_delta_lef * PlaneFM::leadingEdgeFlap_PCT + PlaneFM::AERO::dXdQ * PlaneFM::pitchRate_RPS;
	PlaneFM::AERO::Cx_total += CxFlaps + CxGear + Cxbrakes;

	/* ZZZZZZZZ Cz_tot ZZZZZZZZ */
	PlaneFM::AERO::dZdQ = (PlaneFM::meanChord_FT / (2 * PlaneFM::totalVelocity_FPS)) * (PlaneFM::AERO::Czq + PlaneFM::AERO::Cz_delta_lef * PlaneFM::leadingEdgeFlap_PCT);

	PlaneFM::AERO::Cz_total = PlaneFM::AERO::Cz + PlaneFM::AERO::Cz_delta_lef * PlaneFM::leadingEdgeFlap_PCT + PlaneFM::AERO::dZdQ * PlaneFM::pitchRate_RPS;
	PlaneFM::AERO::Cz_total += CzFlaps + CzGear;

	/* MMMMMMMM Cm_tot MMMMMMMM */
	PlaneFM::AERO::dMdQ = (PlaneFM::meanChord_FT / (2 * PlaneFM::totalVelocity_FPS)) * (PlaneFM::AERO::Cmq + PlaneFM::AERO::Cmq_delta_lef * PlaneFM::leadingEdgeFlap_PCT);

	PlaneFM::AERO::Cm_total = PlaneFM::AERO::Cm * PlaneFM::AERO::eta_el + PlaneFM::AERO::Cz_total * (PlaneFM::referenceCG_PCT - PlaneFM::actualCG_PCT) + PlaneFM::AERO::Cm_delta_lef * PlaneFM::leadingEdgeFlap_PCT + PlaneFM::AERO::dMdQ * PlaneFM::pitchRate_RPS + PlaneFM::AERO::Cm_delta + PlaneFM::AERO::Cm_delta_ds;

	/* YYYYYYYY Cy_tot YYYYYYYY */
	PlaneFM::AERO::dYdail = PlaneFM::AERO::Cy_delta_a20 + PlaneFM::AERO::Cy_delta_a20_lef * PlaneFM::leadingEdgeFlap_PCT;

	PlaneFM::AERO::dYdR = (PlaneFM::wingSpan_FT / (2 * PlaneFM::totalVelocity_FPS)) * (PlaneFM::AERO::Cyr + PlaneFM::AERO::Cyr_delta_lef * PlaneFM::leadingEdgeFlap_PCT);

	PlaneFM::AERO::dYdP = (PlaneFM::wingSpan_FT / (2 * PlaneFM::totalVelocity_FPS)) * (PlaneFM::AERO::Cyp + PlaneFM::AERO::Cyp_delta_lef * PlaneFM::leadingEdgeFlap_PCT);

	PlaneFM::AERO::Cy_total = PlaneFM::AERO::Cy + PlaneFM::AERO::Cy_delta_lef * PlaneFM::leadingEdgeFlap_PCT + PlaneFM::AERO::dYdail * PlaneFM::aileron_PCT + PlaneFM::AERO::Cy_delta_r30 * PlaneFM::rudder_PCT + PlaneFM::AERO::dYdR * PlaneFM::yawRate_RPS + PlaneFM::AERO::dYdP * PlaneFM::rollRate_RPS;

	/* NNNNNNNN Cn_tot NNNNNNNN */
	PlaneFM::AERO::dNdail = PlaneFM::AERO::Cn_delta_a20 + PlaneFM::AERO::Cn_delta_a20_lef * PlaneFM::leadingEdgeFlap_PCT;

	PlaneFM::AERO::dNdR = (PlaneFM::wingSpan_FT / (2 * PlaneFM::totalVelocity_FPS)) * (PlaneFM::AERO::Cnr + PlaneFM::AERO::Cnr_delta_lef * PlaneFM::leadingEdgeFlap_PCT);

	PlaneFM::AERO::dNdP = (PlaneFM::wingSpan_FT / (2 * PlaneFM::totalVelocity_FPS)) * (PlaneFM::AERO::Cnp + PlaneFM::AERO::Cnp_delta_lef * PlaneFM::leadingEdgeFlap_PCT);

	PlaneFM::AERO::Cn_total = PlaneFM::AERO::Cn + PlaneFM::AERO::Cn_delta_lef * PlaneFM::leadingEdgeFlap_PCT - PlaneFM::AERO::Cy_total * (PlaneFM::referenceCG_PCT - PlaneFM::actualCG_PCT) * (PlaneFM::meanChord_FT / PlaneFM::wingSpan_FT) + PlaneFM::AERO::dNdail * PlaneFM::aileron_PCT + PlaneFM::AERO::Cn_delta_r30 * PlaneFM::rudder_PCT + PlaneFM::AERO::dNdR * PlaneFM::yawRate_RPS + PlaneFM::AERO::dNdP * PlaneFM::rollRate_RPS + PlaneFM::AERO::Cn_delta_beta * PlaneFM::beta_DEG;

	/* LLLLLLLL Cl_total LLLLLLLL */
	// Cl is lift
	PlaneFM::AERO::dLdail = PlaneFM::AERO::Cl_delta_a20 + PlaneFM::AERO::Cl_delta_a20_lef * PlaneFM::leadingEdgeFlap_PCT;

	PlaneFM::AERO::dLdR = (PlaneFM::wingSpan_FT / (2 * PlaneFM::totalVelocity_FPS)) * (PlaneFM::AERO::Clr + PlaneFM::AERO::Clr_delta_lef * PlaneFM::leadingEdgeFlap_PCT);

	PlaneFM::AERO::dLdP = (PlaneFM::wingSpan_FT / (2 * PlaneFM::totalVelocity_FPS)) * (PlaneFM::AERO::Clp + PlaneFM::AERO::Clp_delta_lef * PlaneFM::leadingEdgeFlap_PCT);

	PlaneFM::AERO::Cl_total = PlaneFM::AERO::Cl + PlaneFM::AERO::Cl_delta_lef * PlaneFM::leadingEdgeFlap_PCT + PlaneFM::AERO::dLdail * PlaneFM::aileron_PCT + PlaneFM::AERO::Cl_delta_r30 * PlaneFM::rudder_PCT + PlaneFM::AERO::dLdR * PlaneFM::yawRate_RPS + PlaneFM::AERO::dLdP * PlaneFM::rollRate_RPS + PlaneFM::AERO::Cl_delta_beta * PlaneFM::beta_DEG;

	//----------------------------------------------------------------
	// All prior forces calculated in lbs, needs to be converted
	// to units.  All prior forces calculated in lb*ft, needs
	// to be converted into N*m
	//----------------------------------------------------------------

	// Here's where all the calculations get applied to the aircraft in 3-dimensional space.

	// Cy	(force out the right wing)
	Vec3 cy_force(0.0, 0.0, PlaneFM::AERO::Cy_total * PlaneFM::wingArea_FT2 * PlaneFM::dynamicPressure_LBFT2 * 4.44822162825);		// Output force in Newtons
	Vec3 cy_force_pos(0.0, 0, 0); //0.01437
	add_local_force(cy_force, cy_force_pos);

	// Cx (force out the nose)
	Vec3 cx_force(PlaneFM::AERO::Cx_total * PlaneFM::wingArea_FT2 * PlaneFM::dynamicPressure_LBFT2 * 4.44822162825, 0, 0);		// Output force in Newtons
	Vec3 cx_force_pos(0, 0.0, 0.0);
	add_local_force(cx_force, cx_force_pos);

	// Cz (force down the bottom of the aircraft)
	Vec3 cz_force(0.0, -PlaneFM::AERO::Cz_total * PlaneFM::wingArea_FT2 * PlaneFM::dynamicPressure_LBFT2 * 4.44822162825, 0.0);	// Output force in Newtons
	Vec3 cz_force_pos(0, 0, 0);
	add_local_force(cz_force, cz_force_pos);

	// Cl	(Output force in N/m)
	Vec3 cl_moment(PlaneFM::AERO::Cl_total * PlaneFM::wingArea_FT2 * PlaneFM::dynamicPressure_LBFT2 * PlaneFM::wingSpan_FT * 1.35581795, 0.0, 0.0); // Roll
	add_local_moment(cl_moment);																										//AAAAAAAAAAAAAAAAAAAAAAAAA

	if (alpha_DEG <= 15 || alpha_DEG >= -15) {
		// Cm	(Output force in N/m)
		Vec3 cm_moment(0.0, 0.0, PlaneFM::AERO::Cm_total * PlaneFM::wingArea_FT2 * PlaneFM::dynamicPressure_LBFT2 * 1.35581795 * PlaneFM::meanChord_FT); // Pitch
		add_local_moment(cm_moment);}

	if (alpha_DEG > 15 && total_damage < 1) {
		// Cm	(Output force in N/m)
		Vec3 cm_moment(0.0, 0.0, PlaneFM::AERO::Cm_total * PlaneFM::wingArea_FT2 * PlaneFM::dynamicPressure_LBFT2 * 1.35581795 * PlaneFM::meanChord_FT + (alpha_DEG * 500)); // Pitch
		add_local_moment(cm_moment);}
	
	if (alpha_DEG < -15 && total_damage < 1) {
		// Cm	(Output force in N/m)
		Vec3 cm_moment(0.0, 0.0, PlaneFM::AERO::Cm_total * PlaneFM::wingArea_FT2 * PlaneFM::dynamicPressure_LBFT2 * 1.35581795 * PlaneFM::meanChord_FT - (alpha_DEG * 500)); // Pitch
		add_local_moment(cm_moment);}



	// Experimental death spin prevention system
	// Cn	(Output force in N/m)
	if (beta_DEG <= 5 || beta_DEG >= -5) { // Normal flight
		// Cn	(Output force in N/m)
		Vec3 cn_moment(0.0, -PlaneFM::AERO::Cn_total * PlaneFM::wingArea_FT2 * PlaneFM::dynamicPressure_LBFT2 * PlaneFM::wingSpan_FT * 1.35581795, 0.0); //Yaw
		add_local_moment(cn_moment);
	}
	if (beta_DEG > 5 && total_damage < 1) { // If this doesn't seem like a "brute force" apporach to stopping death spirals, I don't know what would.
		// Cm	(Output force in N/m)
		Vec3 cn_moment(0.0, -PlaneFM::AERO::Cn_total * PlaneFM::wingArea_FT2 * PlaneFM::dynamicPressure_LBFT2 * PlaneFM::wingSpan_FT * 1.35581795 + (1000 * (5-beta_DEG) * (pedInput) / (1+tail_damage)), 0.0); //Yaw
		add_local_moment(cn_moment); // The moment/torque produced here can be up to 175000 N/m.
	}

	if (beta_DEG < -5 && total_damage < 1) {
		// Cn	(Output force in N/m)
		Vec3 cn_moment(0.0, -PlaneFM::AERO::Cn_total * PlaneFM::wingArea_FT2 * PlaneFM::dynamicPressure_LBFT2 * PlaneFM::wingSpan_FT * 1.35581795 - (1000 * (5+beta_DEG) * (pedInput) / (1 + tail_damage)), 0.0); //Yaw
		add_local_moment(cn_moment);
	}

	if (Forward_Velocity > -1.0) {
		// Thrust			 
		//Vec3 thrust_force(PlaneFM::thrust_N, 0.0, 0.0);	// Default; Output force is in Newtons
		Vec3 thrust_force(PlaneFM::thrust_N / (1+ (PlaneFM::total_damage * rand() * 1000)), 0.0, 0.0);	// Output force in Newtons
		Vec3 thrust_force_pos(0, 0, 0);
		add_local_force(thrust_force, thrust_force_pos);
	};
	
	
	if (Forward_Velocity < -1.0) {
		// Thrust	
		Vec3 thrust_force(PlaneFM::thrust_N + (1 - Forward_Velocity * 100) / (1 + (PlaneFM::total_damage * rand() * 1000)), 0.0, 0.0);	// Output force in Newtons
		Vec3 thrust_force_pos(0, 0, 0);
		add_local_force(thrust_force, thrust_force_pos);
	};

		// Tell the simulation that it has gone through the first frame
		PlaneFM::simInitialized = true;
		PlaneFM::ACTUATORS::simInitialized = true;
		PlaneFM::FLIGHTCONTROLS::simInitialized = true;

	// This is a fairly complex check to see if the aircraft is on the ground. I've yet to figure out altitude above ground.
		if ((PlaneFM::ay_world < 0.5) && (PlaneFM::pitch_angle < 1) && (PlaneFM::vy_world < 0.1) && (mach < 0.5) && (altitude_m < 4000) && (PlaneFM::ACTUATORS::gear_state >= 0.9))
		{
			PlaneFM::weight_on_wheels = true;
		}
		else 
		{ 
			PlaneFM::weight_on_wheels = false;
		}
		free(temp);
}

void ed_fm_set_atmosphere(	
	double h,//altitude above sea level			(meters)
	double t,//current atmosphere temperature   (Kelvin)
	double a,//speed of sound					(meters/sec)
	double ro,// atmosphere density				(kg/m^3)
	double p,// atmosphere pressure				(N/m^2)
	double wind_vx,//components of velocity vector, including turbulence in world coordinate system (meters/sec)
	double wind_vy,//components of velocity vector, including turbulence in world coordinate system (meters/sec)
	double wind_vz //components of velocity vector, including turbulence in world coordinate system (meters/sec)
	)
{
	PlaneFM::ambientTemperature_DegK = t;
	PlaneFM::ambientDensity_KgPerM3 = ro;
	PlaneFM::altitude_m = h;
	PlaneFM::altitude_FT = h * PlaneFM::meterToFoot;
	PlaneFM::ps_LBFT2 = p * 0.020885434273;
}

void ed_fm_set_current_mass_state ( double mass,
									double center_of_mass_x,
									double center_of_mass_y,
									double center_of_mass_z,
									double moment_of_inertia_x,
									double moment_of_inertia_y,
									double moment_of_inertia_z
									)
{
	center_of_gravity.x  = center_of_mass_x;
	center_of_gravity.y  = center_of_mass_y;
	center_of_gravity.z  = center_of_mass_z;

	inertia.x = moment_of_inertia_x; // These don't seem to do anything.
	inertia.y = moment_of_inertia_y;
	inertia.z = moment_of_inertia_z;

	PlaneFM::weight_N = mass * 9.98665002864;
}
/*
called before simulation to set up your environment for the next step
*/

void ed_fm_set_current_state (double ax,//linear acceleration component in world coordinate system
							double ay,//linear acceleration component in world coordinate system
							double az,//linear acceleration component in world coordinate system
							double vx,//linear velocity component in world coordinate system
							double vy,//linear velocity component in world coordinate system
							double vz,//linear velocity component in world coordinate system
							double px,//center of the body position in world coordinate system
							double py,//center of the body position in world coordinate system
							double pz,//center of the body position in world coordinate system
							double omegadotx,//angular accelearation components in world coordinate system
							double omegadoty,//angular accelearation components in world coordinate system
							double omegadotz,//angular accelearation components in world coordinate system
							double omegax,//angular velocity components in world coordinate system
							double omegay,//angular velocity components in world coordinate system
							double omegaz,//angular velocity components in world coordinate system
							double quaternion_x,//orientation quaternion components in world coordinate system
							double quaternion_y,//orientation quaternion components in world coordinate system
							double quaternion_z,//orientation quaternion components in world coordinate system
							double quaternion_w //orientation quaternion components in world coordinate system
							)
{

	PlaneFM::ay_world = ay;
	vx_world = vx;
	PlaneFM::vy_world = vy;
	vz_world = vz;

	yawRate_world = -omegay - 0.389; // Yaw rate relative to the world; minor adjustments to make autopilot maintain bearing.
	omegax_world = omegax;
	omegaz_world = omegaz;

	PlaneFM::vspeed = vy -3.25; // Vertical speed; with minor adjustments to make autopilot work.
}



void ed_fm_set_current_state_body_axis(	
	double ax,//linear acceleration component in body coordinate system (meters/sec^2)
	double ay,//linear acceleration component in body coordinate system (meters/sec^2)
	double az,//linear acceleration component in body coordinate system (meters/sec^2)
	double vx,//linear velocity component in body coordinate system (meters/sec)
	double vy,//linear velocity component in body coordinate system (meters/sec)
	double vz,//linear velocity component in body coordinate system (meters/sec)
	double wind_vx,//wind linear velocity component in body coordinate system (meters/sec)
	double wind_vy,//wind linear velocity component in body coordinate system (meters/sec)
	double wind_vz,//wind linear velocity component in body coordinate system (meters/sec)
	double omegadotx,//angular accelearation components in body coordinate system (rad/sec^2)
	double omegadoty,//angular accelearation components in body coordinate system (rad/sec^2)
	double omegadotz,//angular accelearation components in body coordinate system (rad/sec^2)
	double omegax,//angular velocity components in body coordinate system (rad/sec)
	double omegay,//angular velocity components in body coordinate system (rad/sec)
	double omegaz,//angular velocity components in body coordinate system (rad/sec)
	double yaw,  //radians (rad)
	double pitch,//radians (rad)
	double roll, //radians (rad)
	double common_angle_of_attack, //AoA  (rad)
	double common_angle_of_slide   //AoS  (rad)
	)

{
	ax_body = ax;
	ay_body = ay;
	az_body = az;
	/*PlaneFM::vx_body = vx;
	PlaneFM::vy_body = vy;
	PlaneFM::vz_body = vz; */

	velocity_world_cs.x = vx;
	velocity_world_cs.y = vy;
	velocity_world_cs.z = vz;

	wind.x = wind_vx;
	wind.y = wind_vy;
	wind.z = wind_vz;

	pitch_angle = (pitch * radiansToDegrees)-1;
	roll_angle = (roll * radiansToDegrees);
	

	//-------------------------------
	// Start of setting plane states
	//-------------------------------

	PlaneFM::alpha_DEG	= (common_angle_of_attack * PlaneFM::radiansToDegrees);
	PlaneFM::beta_DEG	= (common_angle_of_slide * PlaneFM::radiansToDegrees);
	PlaneFM::rollRate_RPS = omegax;   
	PlaneFM::roll_acc = omegadotx;
	PlaneFM::yawRate_RPS = -omegay;  
	PlaneFM::pitchRate_RPS = omegaz;  //


	if ((beta_DEG > 1 && beta_DEG < 30) &&PlaneFM::pedInput > 0.5)
		PlaneFM::yaw_left = 1; 
	else PlaneFM::yaw_left = 0;

	if ((beta_DEG < -1 && beta_DEG > -30) && PlaneFM::pedInput < -0.5)
		PlaneFM::yaw_right = 1;
	else PlaneFM::yaw_right = 0;

	if ((yawRate_RPS < -0.01 && PlaneFM::pedInput < -0.1) || (yawRate_RPS > 0.01 && PlaneFM::pedInput > 0.1))
		PlaneFM::yaw_reversing = 1;		// If the pilot is kicking the rudder to one side and the plane yaws to the other, this becomes 1.
	else PlaneFM::yaw_reversing = 0;

	if (alpha_DEG > 5) { PlaneFM::pitchRate_RPS = omegaz * (((alpha_DEG * alpha_DEG + 10000) / 180) - 54.5); }

	PlaneFM::az = ay;
	PlaneFM::gforce = ay / 10;
	PlaneFM::ay = az;
}

void ed_fm_set_command(int command, float value)	// Command = Command Index (See Export.lua), Value = Signal Value (-1 to 1 for Joystick Axis)
{
	//----------------------------------
	// Set Raw Inputs
	//----------------------------------
	switch (command)
	{
	//Flight contols
		//Roll
	case JoystickRoll:
		PlaneFM::FLIGHTCONTROLS::latStickInput = limit(value, -1.0, 1.0);
		PlaneFM::roll_cmd = limit(value, -1.0, 1.0);
		PlaneFM::roll_cmd_axis = limit(value, -1.0, 1.0);
		PlaneFM::roll_cmd_axis_norm = (limit(value, -1.0, 1.0) * limit(value, -1.0, 1.0)) * 100;
		break;

	case RollLeft:
		//PlaneFM::FLIGHTCONTROLS::latStickInput = (-value - 0.25) - (PlaneFM::FLIGHTCONTROLS::latStickInput);
		PlaneFM::FLIGHTCONTROLS::latStickInput = (-value - 1);
		PlaneFM::roll_cmd = ((-value - 0.025) / 2.0 * 100.0);
		PlaneFM::roll_cmd_discrete += 1;
		break;

	case RollLeftStop:
		PlaneFM::FLIGHTCONTROLS::latStickInput = 0.0;
		//PlaneFM::FLIGHTCONTROLS::latStickInput = rollspeed;
		PlaneFM::roll_cmd = 0.0;
		PlaneFM::roll_cmd_discrete = 0;
		break;

	case trimLeft:
		PlaneFM::rollTrim += 0.015;
		break;

	case RollRight:
		//PlaneFM::FLIGHTCONTROLS::latStickInput = (-value + 0.25) + (PlaneFM::FLIGHTCONTROLS::latStickInput);
		PlaneFM::FLIGHTCONTROLS::latStickInput = (-value + 1);	
		PlaneFM::roll_cmd = ((-value + 0.025) / 2.0 * 100.0);
		PlaneFM::roll_cmd_discrete += 1;
		break;

	case RollRightStop:
		PlaneFM::FLIGHTCONTROLS::latStickInput = 0.0;
		//PlaneFM::FLIGHTCONTROLS::latStickInput = rollspeed;
		PlaneFM::roll_cmd = 0.0;
		PlaneFM::roll_cmd_discrete = 0.0;
		break;

	case trimRight:
		PlaneFM::rollTrim -= 0.015;
		break;

	case JoystickPitch:
		//PlaneFM::FLIGHTCONTROLS::longStickInput = limit(-value, -1.0, 1.0);
		PlaneFM::FLIGHTCONTROLS::longStickInput = limit(-value, -1.0, 1.0);
		PlaneFM::pitch_cmd_axis = (limit(-value, -1.0, 1.0) * limit(-value, -1.0, 1.0));
		break;

	case PitchUp:
		//PlaneFM::FLIGHTCONTROLS::longStickInput = -value - (((alpha_DEG) / (alpha_DEG * alpha_DEG + 5 * 5) / alpha_DEG) * 120);
		PlaneFM::FLIGHTCONTROLS::longStickInput = -value - 1;
		PlaneFM::pitch_cmd_discrete = 1;
		break;
	case PitchUpStop:
		PlaneFM::FLIGHTCONTROLS::longStickInput = 0;
		PlaneFM::pitch_cmd_discrete = 0;
		break;
	case trimUp:
		PlaneFM::pitchTrim -= 0.075;
		break;

	case PitchDown:
		//PlaneFM::FLIGHTCONTROLS::longStickInput = -value + ((alpha_DEG) / (alpha_DEG * alpha_DEG + 5 * 10) / alpha_DEG) *1250;
		//PlaneFM::FLIGHTCONTROLS::longStickInput = -value + ((alpha_DEG) / (alpha_DEG * alpha_DEG + 5 * 10) / alpha_DEG) *2500;
		PlaneFM::FLIGHTCONTROLS::longStickInput = -value + 1;
		PlaneFM::pitch_cmd_discrete = 1;
		//PlaneFM::pitchTrim += 5;
		break;
	case PitchDownStop:
		PlaneFM::FLIGHTCONTROLS::longStickInput = 0;
		PlaneFM::pitch_cmd_discrete = 0;
		//PlaneFM::pitchTrim -= 5;
		break;

	case trimDown:
		PlaneFM::pitchTrim += 0.075;
		break;

		//Yaw
	case JoystickYaw:
		PlaneFM::pedInput = limit(-value * (((beta_DEG) / (beta_DEG * beta_DEG + 100 * 5) / beta_DEG) * 501), -1.0, 1.0);
		PlaneFM::yaw_cmd_axis = (limit(-value, -10.0, 10.0) * limit(-value, -10.0, 10.0));
		break;

	case rudderleft:
		//PlaneFM::pedInput = -value + (((beta_DEG) / (beta_DEG * beta_DEG + 5 * 20) / beta_DEG) * 101);
		PlaneFM::pedInput = -value + 1;
		PlaneFM::yaw_cmd_discrete += 1;
		break;
	case rudderleftend:
		PlaneFM::pedInput = 0.0;
		PlaneFM::yaw_cmd_discrete = 0;
		break;

	case ruddertrimLeft:
		PlaneFM::yawTrim += 0.05;
		break;

	case rudderright:
		//PlaneFM::pedInput = -value - (((beta_DEG) / (beta_DEG * beta_DEG + 5 * 20) / beta_DEG) * 101);
		PlaneFM::pedInput = -value - 1;
		PlaneFM::yaw_cmd_discrete += 1;
		break;
	case rudderrightend:
		PlaneFM::pedInput = 0.0;
		PlaneFM::yaw_cmd_discrete = 0;
		break;

	case ruddertrimRight:
		PlaneFM::yawTrim -= 0.05;
		break;

	//Engine and throttle commands
	case EnginesOff: //FIGURE THIS OUT!
		//PlaneFM::engineswitch = 0;
		PlaneFM::starter_command = 0;
		//PlaneFM::throttleInput = 0;
		PlaneFM::throttleInput = -100 + limit((value), 0.0, 0.0);
		break;
	case LeftEngineOff:
		PlaneFM::starter_command = 0;
		PlaneFM::throttleInput = -100 + limit((value), 0.0, 0.0);
		break;
	case RightEngineOff:
		PlaneFM::starter_command = 0;
		PlaneFM::throttleInput = -100 + limit((value), 0.0, 0.0);
		break;

	case EnginesOn:
		//PlaneFM::engineswitch = 1;
		PlaneFM::starter_command = 0.5;
		//PlaneFM::throttleInput = 50;
		//PlaneFM::throttleInput += 50;
		//PlaneFM::throttleInput = (limit((value), 0.0, 100.0));
		break;
	case LeftEngineOn:
		PlaneFM::starter_command = 0.5;
		break;
	case RightEngineOn:
		PlaneFM::starter_command = 0.5;
		break;
	
	case JoystickThrottle:
		if (PlaneFM::engineswitch = true)
		{
			PlaneFM::throttleInput = limit(((-value + 1.0) / 2.0) * 100.0, 25.0, 100.0);
		}
		break;

	case ThrottleIncrease:
		if (PlaneFM::engineswitch == 1)
		//if (PlaneFM::ACTUATORS::starter_state >= 50)
		{
			if (PlaneFM::internal_fuel >= 5.0)
			{
				if (PlaneFM::throttleInput < 100)
				{
					PlaneFM::throttleInput += 0.40;
				}
				if (PlaneFM::throttleInput <= 24.9)
				{
					PlaneFM::throttleInput = 25.0;
				}
			}
			if (PlaneFM::internal_fuel < 5.0)
			{
				//PlaneFM::throttleInput -= 100.0;
				PlaneFM::throttleInput = 0.0;
				//PlaneFM::throttleInput = 0 + limit((value), 0.0, 0.01);
			}
		}
		else
		{
			PlaneFM::throttleInput = 0;
		}
		break;

	case ThrottleDecrease:
		if (PlaneFM::engineswitch == 1)
		//if (PlaneFM::ACTUATORS::starter_state >= 50)
		{
			if (PlaneFM::internal_fuel >= 5.0)
			{
				if (PlaneFM::throttleInput <= 24.9)
				{
					PlaneFM::throttleInput += 0.01;
				}
				if (PlaneFM::throttleInput > 25.0)
				{
					PlaneFM::throttleInput -= 0.50;
				}
			}
			else
			{
				//PlaneFM::throttleInput -= 100.0;
				PlaneFM::throttleInput = 0.0;
				//PlaneFM::throttleInput = 0 + limit((value), 0.0, 0.01);
			
			}
		}
		else
		{
			PlaneFM::throttleInput = 0;
		}
		break;
		

		//flaps
	case flapsdown:
		PlaneFM::flap_command = 1.0;
		PlaneFM::pitchTrim -= 1;
		break;

	case flapsup:
		PlaneFM::flap_command = 0.0;
		PlaneFM::pitchTrim += 1;
		break;

	case flapstoggle: //toggle
		if (PlaneFM::ACTUATORS::flapPosition_DEG < 0.5)
		{ PlaneFM::flap_command = 1.0;
			PlaneFM::pitchTrim += 1; }
		else if (PlaneFM::ACTUATORS::flapPosition_DEG > 0.51)
		{ PlaneFM::flap_command = 0.0;
			PlaneFM::pitchTrim -= 1; }
		break;


		//Air brakes
	case AirBrakes: //toggle
		if (PlaneFM::ACTUATORS::airbrake_state < 0.25) 
			PlaneFM::airbrake_command = 1.0;
		else if (PlaneFM::ACTUATORS::airbrake_state > 0.75) 
			PlaneFM::airbrake_command = 0.0;
		break;
	case AirBrakesOff:
		PlaneFM::airbrake_command = 0.0;
	case AirBrakesOn:
		PlaneFM::airbrake_command = 1.0;
		break;

		// Gear commands
	case geardown:
		PlaneFM::GearCommand = 1.0;
		break; 
	case gearup:
		PlaneFM::GearCommand = 0.0;
		break;
	case geartoggle:
		if (PlaneFM::ACTUATORS::gear_state > 0.5) PlaneFM::GearCommand = 0.0;
		else if (PlaneFM::ACTUATORS::gear_state < 0.5) PlaneFM::GearCommand = 1.0;
	case WheelBrakeOn:
		PlaneFM::rolling_friction = 0.165;
		PlaneFM::WheelBrakeCommand = 1.0 * weight_on_wheels;
		break;
	case WheelBrakeOff:
		PlaneFM::rolling_friction = 0.015;
		PlaneFM::WheelBrakeCommand = 0.0 * weight_on_wheels;
		break;

		//Other commands
	case tailhook:  
		if (misc_state < 0.5) PlaneFM::misc_cmd = 1.0;
		if (misc_state > 0.5) PlaneFM::misc_cmd = 0.0;
		break;

		//Autopilot
	case autopilot_alt:

		horiz_hold = 0;
		altroll_hold = 0;

		if (alt_hold < 0.5) alt_hold = 1;
		else if (alt_hold > 0.5) alt_hold = 0;
		break;

	case autopilot_horiz:

		alt_hold = 0;
		altroll_hold = 0;

		if (horiz_hold < 0.5) horiz_hold = 1;
		else if (horiz_hold > 0.5) horiz_hold = 0;
		break;

	case autopilot_alt_roll:

		alt_hold = 0;
		horiz_hold = 0;

		if (altroll_hold < 0.5) altroll_hold = 1;
		else if (altroll_hold > 0.5) altroll_hold = 0;
		break;

	case autopilot_reset:

		horiz_hold = 0;
		alt_hold = 0;
		altroll_hold = 0;

		break;

	case resetTrim:
		PlaneFM::pitchTrim = 0.0;
		PlaneFM::rollTrim = 0.0;
		PlaneFM::yawTrim = 0.0;
		break;

	};
}

/*
	Mass handling 

	will be called  after ed_fm_simulate :
	you should collect mass changes in ed_fm_simulate 

	double delta_mass = 0;
	double x = 0;
	double y = 0; 
	double z = 0;
	double piece_of_mass_MOI_x = 0;
	double piece_of_mass_MOI_y = 0; 
	double piece_of_mass_MOI_z = 0;
 
	//
	while (ed_fm_change_mass(delta_mass,x,y,z,piece_of_mass_MOI_x,piece_of_mass_MOI_y,piece_of_mass_MOI_z))
	{
	//internal DCS calculations for changing mass, center of gravity, and moments of inertia
	}
*/
bool ed_fm_change_mass  (double & delta_mass,
						double & delta_mass_pos_x,
						double & delta_mass_pos_y,
						double & delta_mass_pos_z,
						double & delta_mass_moment_of_inertia_x,
						double & delta_mass_moment_of_inertia_y,
						double & delta_mass_moment_of_inertia_z
						)
{
	if((inertia.x != PlaneFM::inertia_Ix_KGM2) ||
	   (inertia.y != PlaneFM::inertia_Iz_KGM2) ||
	   (inertia.z != PlaneFM::inertia_Iy_KGM2))
	{
		delta_mass = 0.0;
		delta_mass_pos_x = 0.0;
		delta_mass_pos_y = 0.0;
		delta_mass_pos_z = 0.0;
		delta_mass_moment_of_inertia_x = PlaneFM::inertia_Ix_KGM2 - inertia.x;
		delta_mass_moment_of_inertia_y = PlaneFM::inertia_Ix_KGM2 - inertia.z;
		delta_mass_moment_of_inertia_z = PlaneFM::inertia_Ix_KGM2 - inertia.y;

		// Can't set to true...crashing right now :(
		return false;
	}
	else
	{
		return false;
	}
	if (PlaneFM::fuel_consumption_since_last_time > 0)
	{
		delta_mass		 = PlaneFM::fuel_consumption_since_last_time;
		delta_mass_pos_x = -1.0;
		delta_mass_pos_y =  1.0;
		delta_mass_pos_z =  0;

		delta_mass_moment_of_inertia_x	= 0;
		delta_mass_moment_of_inertia_y	= 0;
		delta_mass_moment_of_inertia_z	= 0;

		PlaneFM::fuel_consumption_since_last_time = 0; // set it 0 to avoid infinite loop, because it called in cycle 
		// better to use stack like structure for mass changing 
		return true;
	}
	else 
	{
		return false;
	}
}

/*
	set internal fuel volume , init function, called on object creation and for refueling , 
	you should distribute it inside at different fuel tanks
*/
void ed_fm_set_internal_fuel(double fuel)
{
	internal_fuel = fuel;
}
/*
	get internal fuel volume 
*/
double ed_fm_get_internal_fuel()
{
	return internal_fuel + external_fuel;
}
/*
	set external fuel volume for each payload station , called for weapon init and on reload
*/
void  ed_fm_set_external_fuel (int	 station,
								double fuel,
								double x,
								double y,
								double z)
{
}
/*
	get external fuel volume 
*/

double ed_fm_get_external_fuel ()
{
	return external_fuel;
}

double ed_fm_refueling_add_fuel()
{
	return internal_fuel + 100;
}

void ed_fm_set_draw_args(EdDrawArgument* drawargs, size_t size) 
//The things that move on the model, use the model viewer to learn what each "arg" corresponds to.
{
	if (PlaneFM::simInitialized)
	{
		PlaneFM::ACTUATORS::gear_state = drawargs[0].f;
		PlaneFM::ACTUATORS::gear_state = drawargs[3].f;
		PlaneFM::ACTUATORS::gear_state = drawargs[5].f;
	}
	else {
		drawargs[0].f = (float)PlaneFM::ACTUATORS::gear_state;
		drawargs[3].f = (float)PlaneFM::ACTUATORS::gear_state;
		drawargs[5].f = (float)PlaneFM::ACTUATORS::gear_state;
	}
	// I haven't quite figured this out.
	PlaneFM::weight_on_wheels = limit(drawargs[1].f + drawargs[4].f + drawargs[6].f, -1.0, 1.0);

	//control surfaces

	// Flaps
	drawargs[9].f = (float)ACTUATORS::flapPosition_DEG;
	drawargs[10].f =(float)ACTUATORS::flapPosition_DEG;

	// Ailerons
	/*drawargs[11].f = (float)limit((-aileron_PCT + (rollTrim / 10) / (PlaneFM::mach + 1)), -0.75, 0.75);
	drawargs[12].f = (float)limit((aileron_PCT + (rollTrim / 10) / (PlaneFM::mach + 1)), -0.75, 0.75);
	drawargs[1126].f = (float)limit((-aileron_PCT), -0.5, 0.5);
	drawargs[1128].f = (float)limit((aileron_PCT), -0.5, 0.5);*/

	drawargs[11].f = (float)limit((-ACTUATORS::aileronPosition_DEG), -0.75, 0.75);
	drawargs[12].f = (float)limit((ACTUATORS::aileronPosition_DEG), -0.75, 0.75);
	drawargs[1126].f = (float)limit((-ACTUATORS::aileronPosition_DEG), -0.5, 0.5);
	drawargs[1128].f = (float)limit((ACTUATORS::aileronPosition_DEG), -0.5, 0.5);

	// Slats
	drawargs[13].f = (float)leadingEdgeFlap_PCT;
	drawargs[14].f = (float)leadingEdgeFlap_PCT;
	drawargs[127].f =(float)leadingEdgeFlap_PCT;
	drawargs[129].f =(float)leadingEdgeFlap_PCT;
	drawargs[274].f =(float)leadingEdgeFlap_PCT;
	drawargs[275].f =(float)leadingEdgeFlap_PCT;

	// Elevators or stabilators
	drawargs[15].f = (float)limit(-elev_pos / (mach + 1), -0.6, 0.6);
	drawargs[16].f = (float)limit(-elev_pos / (mach + 1), -0.6, 0.6);

	// Wing geometry (automatic)
	drawargs[7].f = (float)limit(((PlaneFM::mach - 0.7) / 0.3), 0.0, 1.0);

	// Rudder(s)
	drawargs[17].f = (float)limit((rudder_pos + (yawTrim / 10)), -0.75, 0.75);
	drawargs[18].f = (float)limit((rudder_pos + (yawTrim / 10)), -0.75, 0.75);

	// Nose wheel steering
	drawargs[2].f = (float)limit((rudder_pos), -0.6, 0.6);

	// Air brakes or spoilers
	drawargs[21].f = (float)ACTUATORS::airbrake_state;
	drawargs[182].f = (float)ACTUATORS::airbrake_state;
	drawargs[184].f = (float)ACTUATORS::airbrake_state;

	// Weapon bay doors or tail hook
	drawargs[25].f = (float)limit((PlaneFM::misc_state), 0.0, 1.0); // This is usually the tail hook for most planes with them.
	//drawargs[26].f = (float)limit((PlaneFM::misc_state), 0.0, 1.0); // This is usually weapon bays for planes with them.
	
	// Engines and afterburners
	drawargs[28].f = (float)limit(((PlaneFM::throttle_state - 90.0) / 10.0), 0.0, 1.0);//This determines where the afterburners start
	drawargs[29].f = (float)limit(((PlaneFM::throttle_state - 90.0) / 10.0), 0.0, 1.0);
	drawargs[89].f = (float)limit(((PlaneFM::throttle_state - 90.0) / 10.0), 0.0, 1.0);
	drawargs[90].f = (float)limit(((PlaneFM::throttle_state - 90.0) / 10.0), 0.0, 1.0);

	if (size > 616)
	{
		drawargs[611].f = drawargs[0].f;
		drawargs[614].f = drawargs[3].f;
		drawargs[616].f = drawargs[5].f;
	}
}

// Cockpit controls (stick, rudder pedals, throttle) don't animate for some reason.
//void ed_fm_set_fc3_cockpit_draw_args(double* drawargs, size_t size)
void ed_fm_set_fc3_cockpit_draw_args(double* drawargs, size_t size)
{
	drawargs[71] = (float)limit((PlaneFM::FLIGHTCONTROLS::latStickInput), -1.0, 1.0); 
	drawargs[74] = (float)limit((-PlaneFM::FLIGHTCONTROLS::longStickInput), -1.0, 1.0);
	drawargs[104] = (float)limit((PlaneFM::throttleInput), 0.0, 1.0);
	drawargs[105] = (float)limit((PlaneFM::throttleInput), 0.0, 1.0);

	PlaneFM::FLIGHTCONTROLS::latStickInput = drawargs[71];
	PlaneFM::FLIGHTCONTROLS::longStickInput = drawargs[74];
	PlaneFM::throttleInput = drawargs[104];
	PlaneFM::throttleInput = drawargs[105];

	drawargs[0] = PlaneFM::GearCommand;
	drawargs[3] = PlaneFM::GearCommand;
	drawargs[5] = PlaneFM::GearCommand;
};

void ed_fm_configure(const char * cfg_path)
{
	// I'm not too sure what this does.
}

double ed_fm_get_param(unsigned index)
{	
	// Gear stuff
	switch (index)
	{
	case ED_FM_SUSPENSION_0_RELATIVE_BRAKE_MOMENT:
		return PlaneFM::rolling_friction;
	case ED_FM_SUSPENSION_1_RELATIVE_BRAKE_MOMENT:
		return PlaneFM::rolling_friction;
	case ED_FM_SUSPENSION_2_RELATIVE_BRAKE_MOMENT:
		return PlaneFM::rolling_friction;
		//break;
	case ED_FM_SUSPENSION_0_WHEEL_SELF_ATTITUDE:
		return 0.0;

	case ED_FM_SUSPENSION_0_WHEEL_YAW:
		return limit(PlaneFM::rudder_pos, -0.3, 0.3);

	case ED_FM_ANTI_SKID_ENABLE:
		return true;
	case ED_FM_SUSPENSION_0_GEAR_POST_STATE:
		return PlaneFM::ACTUATORS::gear_state;

	case ED_FM_SUSPENSION_1_GEAR_POST_STATE:
		return PlaneFM::ACTUATORS::gear_state;

	case ED_FM_SUSPENSION_1_WHEEL_SELF_ATTITUDE:
		return 0.0;
	
	case ED_FM_SUSPENSION_2_GEAR_POST_STATE:
		return PlaneFM::ACTUATORS::gear_state;

	case ED_FM_SUSPENSION_2_WHEEL_SELF_ATTITUDE:
		return 0.0;

	case ED_FM_SUSPENSION_0_DOWN_LOCK:
		return PlaneFM::ACTUATORS::gear_state;
		
	case ED_FM_FC3_GEAR_HANDLE_POS:
		return PlaneFM::GearCommand;
	}


	if (index <= ED_FM_END_ENGINE_BLOCK)
	{
		switch (index)
		{
			//Engine 0 is the auxillary power unit (APU)
		case ED_FM_ENGINE_0_RPM:
		case ED_FM_ENGINE_0_RELATED_RPM:
		case ED_FM_ENGINE_0_THRUST:
		case ED_FM_ENGINE_0_RELATED_THRUST:
			return 0; // APU

		case ED_FM_ENGINE_1_RPM:

			if (PlaneFM::engineswitch == false)
				return limit((PlaneFM::throttleInput), 0.0, 0.0);
			if (PlaneFM::engineswitch == true)
				return limit(((PlaneFM::throttle_state - 35) / 50.0), 0.0, 1.0);
			if (PlaneFM::internal_fuel < 5)
				return 0 + limit((PlaneFM::throttleInput), 0.0, 0.001);

		case ED_FM_ENGINE_1_RELATED_RPM:
			//return limit((PlaneFM::throttleInput), 0.0, 100.0);
			if (PlaneFM::engineswitch == false)
				return limit((PlaneFM::throttleInput), 0.0, 0.0);
			if (PlaneFM::engineswitch == true)
				return limit(((PlaneFM::throttle_state - 25) / 75), 0.25, 1.0);
			if (PlaneFM::internal_fuel < 5)
				return 0 + limit((PlaneFM::throttleInput), 0.0, 0.001);

		case ED_FM_ENGINE_1_THRUST:
			if (PlaneFM::engineswitch == false)
				return limit((PlaneFM::throttleInput), 0.0, 0.0);
			if (PlaneFM::engineswitch == true)
				return PlaneFM::thrust_N;
			if (PlaneFM::internal_fuel < 5)
				return 0 + limit((PlaneFM::throttleInput), 0.0, 0.001);

		case ED_FM_ENGINE_1_RELATED_THRUST: // This determines the heat blur effect and exhaust sound
			if (PlaneFM::engineswitch == false)
				return /*limit((PlaneFM::throttleInput), */PlaneFM::ACTUATORS::starter_state/*, 0.0)*/;
			if (PlaneFM::engineswitch == true && starter_state > 20)
				return limit(((PlaneFM::throttle_state - 25) / 75), 0.25, 1.0);
			if (PlaneFM::internal_fuel < 5)
				return 0 + limit((PlaneFM::throttleInput), 0.0, 0.001);

		case ED_FM_ENGINE_1_CORE_RELATED_RPM: // This shows up as "RPM" in-game.
			if (PlaneFM::engineswitch == false)
				return limit((PlaneFM::throttleInput), starter_state, 0.0);
			if (PlaneFM::engineswitch == true)
				return limit(((((PlaneFM::throttle_state - 25) / 75) + 1) / (2 + engine_damage)), starter_state, 1.0);
				//return limit((PlaneFM::ACTUATORS::throttle_state / 100), 0.4, 1.05);
			if (PlaneFM::internal_fuel < 5)
				return limit((PlaneFM::throttleInput), 0.0, 0.001) + (PlaneFM::throttleInput - 100);
			if (PlaneFM::engine_damage > 5)
				return limit((PlaneFM::throttleInput), 0.0, 0.001) + (PlaneFM::throttleInput - 100);

			/*if (PlaneFM::engineswitch == false)
				return limit((PlaneFM::throttleInput), 0.0, 0.0);
			if (PlaneFM::engineswitch == true)
				return limit(((((PlaneFM::throttle_state - 25) / 75) + 1) / (2 + engine_damage)), 0.5, 1.0);
				//return limit((PlaneFM::ACTUATORS::throttle_state / 100), 0.4, 1.05);
			if (PlaneFM::internal_fuel < 5)
				return limit((PlaneFM::throttleInput), 0.0, 0.001) + (PlaneFM::throttleInput - 100);
			if (PlaneFM::engine_damage > 5)
				return limit((PlaneFM::throttleInput), 0.0, 0.001) + (PlaneFM::throttleInput - 100);
			*/
			//return PlaneFM::ACTUATORS::starter_state;

		case	ED_FM_ENGINE_1_TEMPERATURE:
			return limit((PlaneFM::ACTUATORS::throttle_state / 100), 0.5, 1.05)*500;
		case	ED_FM_ENGINE_1_OIL_PRESSURE:
			return PlaneFM::ACTUATORS::throttle_state * 10;
			if (PlaneFM::internal_fuel < 5)
				return 0 + limit((PlaneFM::throttleInput), 0.0, 0.001);
		case	ED_FM_ENGINE_1_FUEL_FLOW:
			if (PlaneFM::internal_fuel > 5)
				return limit((PlaneFM::ACTUATORS::throttle_state / 100), 0.25, 1.05) * 100;
			if (PlaneFM::internal_fuel < 5)
				return 0 + limit((PlaneFM::throttleInput), 0.0, 0.001);

		case ED_FM_FC3_THROTTLE_LEFT:
			if (PlaneFM::engineswitch == false)
				return limit((PlaneFM::throttleInput), 0.0, 0.0);
			if (PlaneFM::engineswitch == true)
			return limit((PlaneFM::throttleInput), 0.25, 1.0);
			if (PlaneFM::internal_fuel < 5)
				return 0 + limit((PlaneFM::throttleInput), 0.0, 0.001);


		case ED_FM_ENGINE_2_RPM:

			if (PlaneFM::engineswitch == false)
				return limit((PlaneFM::throttleInput), 0.0, 0.0);
			if (PlaneFM::engineswitch == true)
				return limit((PlaneFM::throttleInput), 0.3, 1.0);
			if (PlaneFM::internal_fuel < 5)
				return 0 + limit((PlaneFM::throttleInput), 0.0, 0.001);

		case ED_FM_ENGINE_2_RELATED_RPM:
			if (PlaneFM::engineswitch == false)
				return limit((PlaneFM::throttleInput), 0.0, 0.0);
			if (PlaneFM::engineswitch == true)
				return limit(((PlaneFM::throttle_state - 25) / 75), 0.25, 1.0);
			if (PlaneFM::internal_fuel < 5)
				return 0 + limit((PlaneFM::throttleInput), 0.0, 0.001);

		case ED_FM_ENGINE_2_THRUST:
			if (PlaneFM::engineswitch == false)
				return limit((PlaneFM::throttleInput), 0.0, 0.0);
			if (PlaneFM::engineswitch == true)
				return PlaneFM::thrust_N;
			if (PlaneFM::internal_fuel < 5)
				return 0 + limit((PlaneFM::throttleInput), 0.0, 0.001);

		case ED_FM_ENGINE_2_RELATED_THRUST: // This determines the heat blur effect and exhaust sound
			if (PlaneFM::engineswitch == false)
				return /*limit((PlaneFM::throttleInput), */PlaneFM::ACTUATORS::starter_state/*, 0.0)*/;
			if (PlaneFM::engineswitch == true && starter_state > 20)
				return limit(((PlaneFM::throttle_state - 25) / 75), 0.25, 1.0);
			if (PlaneFM::internal_fuel < 5)
				return 0 + limit((PlaneFM::throttleInput), 0.0, 0.001);

		case ED_FM_ENGINE_2_CORE_RELATED_RPM: // This shows up as "RPM" in-game.
			if (PlaneFM::engineswitch == false)
				return limit((PlaneFM::throttleInput), starter_state, 0.0);
			if (PlaneFM::engineswitch == true)
				return limit(((((PlaneFM::throttle_state - 25) / 75) + 1) / (2 + engine_damage)), starter_state, 1.0);
			//return limit((PlaneFM::ACTUATORS::throttle_state / 100), 0.4, 1.05);
			if (PlaneFM::internal_fuel < 5)
				return limit((PlaneFM::throttleInput), 0.0, 0.001) + (PlaneFM::throttleInput - 100);
			if (PlaneFM::engine_damage > 5)
				return limit((PlaneFM::throttleInput), 0.0, 0.001) + (PlaneFM::throttleInput - 100);
			
			//return PlaneFM::ACTUATORS::starter_state;

		case	ED_FM_ENGINE_2_TEMPERATURE:
			return limit((PlaneFM::ACTUATORS::throttle_state / 100), 0.5, 1.05) * 500;
		case	ED_FM_ENGINE_2_OIL_PRESSURE:
			return PlaneFM::ACTUATORS::throttle_state * 10;
			if (PlaneFM::internal_fuel < 5)
				return 0 + limit((PlaneFM::throttleInput), 0.0, 0.001);
		case	ED_FM_ENGINE_2_FUEL_FLOW:
			if (PlaneFM::internal_fuel > 5)
				return limit((PlaneFM::ACTUATORS::throttle_state / 100), 0.25, 1.05) * 100;
			if (PlaneFM::internal_fuel < 5)
				return 0 + limit((PlaneFM::throttleInput), 0.0, 0.001);

		case ED_FM_FC3_THROTTLE_RIGHT:
			if (PlaneFM::engineswitch == false)
				return limit((PlaneFM::throttleInput), 0.0, 0.0);
			if (PlaneFM::engineswitch == true)
				return limit((PlaneFM::throttleInput), 0.25, 1.0);
			if (PlaneFM::internal_fuel < 5)
				return 0 + limit((PlaneFM::throttleInput), 0.0, 0.001);
		}

	//other stuff
	switch (index)
	{

	case ED_FM_FUEL_INTERNAL_FUEL:
		return (PlaneFM::internal_fuel)+(PlaneFM::external_fuel);
	case ED_FM_FUEL_TOTAL_FUEL:
		return (PlaneFM::internal_fuel) + (PlaneFM::external_fuel);

	// These don't seem to do anything.
	case ED_FM_OXYGEN_SUPPLY:
		return 1000;
	case ED_FM_FLOW_VELOCITY:
		return 100;
	case ED_FM_FC3_SPEED_BRAKE_HANDLE_POS:
		return PlaneFM::airbrake_command*100;
	case ED_FM_FC3_STICK_PITCH:
		return PlaneFM::FLIGHTCONTROLS::longStickInput;
	case ED_FM_FC3_STICK_ROLL:
		return PlaneFM::FLIGHTCONTROLS::latStickInput;
	case ED_FM_FC3_RUDDER_PEDALS:
		return PlaneFM::pedInput;
	}
	}
	return 0;	

	// Autopilot stuff, work in progress
	switch (index)
	{
	case ED_FM_FC3_AUTOPILOT_STATUS:
		return PlaneFM::alt_hold;
	//case ED_FM_FC3_AUTOPILOT_FAILURE_ATTITUDE_STABILIZATION:
	}
}

// This defines what is reset when the plane is destroyed or the player restarts or quits the mission.
void ed_fm_release()
{
	PlaneFM::simInitialized = false;
	PlaneFM::ACTUATORS::simInitialized = false;
	PlaneFM::FLIGHTCONTROLS::simInitialized = false;

	PlaneFM::engine_damage = 0;
	PlaneFM::Lwing_damage = 0;
	PlaneFM::Rwing_damage = 0;
	PlaneFM::tail_damage = 0;
	PlaneFM::cockpit_damage = 0;
	PlaneFM::total_damage = 0;

	PlaneFM::pedInput = 0;
	PlaneFM::throttleInput = 0.0;
	PlaneFM::elevator_DEG = 0;
	PlaneFM::aileron_DEG = 0;
	PlaneFM::rudder_DEG = 0;
	PlaneFM::elevator_DEG_commanded = 0;
	PlaneFM::rudder_DEG_commanded = 0;
	PlaneFM::throttle_state = 0;
	PlaneFM::rolling_friction = 0.015;
	PlaneFM::WheelBrakeCommand = 0.0;
	PlaneFM::pitchTrim = 0.0;
	PlaneFM::rollTrim = 0.0;
	PlaneFM::yawTrim = 0.0;
	PlaneFM::airbrake_command = 0.0;
	PlaneFM::airbrakes = 0.0;
	PlaneFM::flap_command = 0.0;
	PlaneFM::flaps = 0.0;
	PlaneFM::misc_cmd = 0.0;
	PlaneFM::misc_state = 0.0;
	PlaneFM::starter_command = 0.0;
	PlaneFM::starter_state = 0.0;
	PlaneFM::starter_switch = 0.0;

	PlaneFM::alt_hold = 0;
	PlaneFM::horiz_hold = 0;
	PlaneFM::altroll_hold = 0;

	PlaneFM::ACTUATORS::flapPosition_DEG = 0.0;
	PlaneFM::ACTUATORS::flapRate_DEGPERSEC = 0.0;
	PlaneFM::ACTUATORS::throttle_state;
	PlaneFM::ACTUATORS::throttle_rate;
	PlaneFM::ACTUATORS::starter_rate;
	PlaneFM::ACTUATORS::starter_state;
	PlaneFM::ACTUATORS::misc_pos = 0.0;
	PlaneFM::ENGINE::percentPower = 0.0;
	PlaneFM::FLIGHTCONTROLS::latStickInput = 0.0;
	PlaneFM::FLIGHTCONTROLS::longStickInput = 0.0;
	PlaneFM::FLIGHTCONTROLS::longStickForce = 0.0;
	
	//free(temp);
	
}

// Conditions to make the screen shake in first-person view. This is VERY useful for debugging and figuring out what does what.
double ed_fm_get_shake_amplitude() 
{/**/
	return total_damage / 100;

	if (PlaneFM::gforce > 8.0)  // If the plane is under 8 gs or more, the screen shakes.
	{return (PlaneFM::az / 30);}

	if (PlaneFM::ACTUATORS::airbrake_state > 0.2 && PlaneFM::mach >= 0.1)
	{
		return PlaneFM::ACTUATORS::airbrake_state * (mach / 10);
	}
	if (PlaneFM::ACTUATORS::flapPosition_DEG > 0.2 && PlaneFM::mach >= 0.1)
	{
		return PlaneFM::ACTUATORS::flapPosition_DEG * (mach / 50);
	}
	if (PlaneFM::alpha_DEG > 45 && PlaneFM::mach >= 0.1)
	{
		return PlaneFM::alpha_DEG / 100;
	}
	if (PlaneFM::beta_DEG > 15 && PlaneFM::mach >= 0.05)
	{
		return PlaneFM::beta_DEG / 100;
	}
	else
	{
		return 0;
	}; 
}

// What parameters should change when easy flight mode is on/off?
void ed_fm_set_easy_flight(bool value) 
{} // Nothing here. Flight behaviour for this FM mimics "game" flight mode as is.

// This defines what happens when the unlimited fuel option is on or off.
void ed_fm_unlimited_fuel(bool value) 
{
		PlaneFM::param_class.param_stuff::fuelparam(1-value);
}

// What parameters should be set to what in a cold start?
void ed_fm_cold_start() 
{
	PlaneFM::gearDown = 1;
	PlaneFM::GearCommand = 1;
	PlaneFM::throttleInput = 0;
	PlaneFM::WheelBrakeCommand = 0.0;
	PlaneFM::flap_command = 0.0;
	PlaneFM::flaps = 0.0;
	PlaneFM::rolling_friction = 0.015;
	PlaneFM::starter_command = 0.0;
	PlaneFM::starter_state = 0.0;
	PlaneFM::starter_switch = 0.0;
} 

// What parameters should be set to what in a hot start on the ground?
void ed_fm_hot_start() 
{
	PlaneFM::gearDown = 1;
	PlaneFM::GearCommand = 1;
	PlaneFM::flap_command = 0;
	PlaneFM::throttleInput = 25.0;
	PlaneFM::WheelBrakeCommand = 0.0;
	PlaneFM::flap_command = 0.0;
	PlaneFM::flaps = 0.0;
	PlaneFM::starter_command = 0.5;
	PlaneFM::rolling_friction = 0.015; 
	//PlaneFM::starter_state = 0.0;
	//PlaneFM::ACTUATORS::starter_state = 0.5;
}

// What parameters should be set to what in a hot start in the air?
void ed_fm_hot_start_in_air() 
{
	PlaneFM::gearDown = 0;
	PlaneFM::GearCommand = 0;
	PlaneFM::throttleInput = 75.0;
	PlaneFM::throttle_state = 75.0;
	PlaneFM::WheelBrakeCommand = 0.0;
	PlaneFM::flap_command = 0.0;
	PlaneFM::flaps = 0.0;
	PlaneFM::starter_command = 0.5;
	PlaneFM::rolling_friction = 0.015;
	//PlaneFM::starter_state = 0.0;
	//PlaneFM::ACTUATORS::starter_state = 0.5;
}

//What should be fixed when repairs are complete?
void ed_fm_repair()
{
	PlaneFM::engine_damage = 0;
	PlaneFM::Lwing_damage = 0;
	PlaneFM::Rwing_damage = 0;
	PlaneFM::tail_damage = 0;
	PlaneFM::cockpit_damage = 0;
	PlaneFM::total_damage = 0;
}

void ed_fm_set_immortal(bool value)
{
	PlaneFM::param_class.param_stuff::invincible(1 - value);
}

// Damage stuff, still a work in progress and not really a priority unless people ask for it. 
void ed_fm_on_damage(int Element, double element_integrity_factor)
{
	double Engines = 0.0;
	double L_wing = 0.0;
	double R_wing = 0.0;
	double Tail = 0.0;
	double Canopy = 0.0;

	if (PlaneFM::param_class.invincible_value = 1)
	{ 	
	/*
	if (Element == 103 || Element == 104 || Element == 11 || Element == 12) // Engines 1 and 2
		PlaneFM::engine_damage += (element_integrity_factor * 10);

	// Right wing damage
	if (Element == 22 || Element == 24 || Element == 26 || Element == 28 || Element == 30 || Element == 34 || Element == 36 || Element == 38 || Element == 46 || Element == 48 || Element == 50 || Element == 52) // Right wing damage
		PlaneFM::Rwing_damage += (element_integrity_factor * 10);

	// Left wing damage
	if (Element == 21 || Element == 23 || Element == 25 || Element == 27 || Element == 29 || Element == 35 || Element == 37 || Element == 39 || Element == 45 || Element == 47 || Element == 49 || Element == 51) // Left wing damage
		PlaneFM::Lwing_damage += (element_integrity_factor * 10);

	// Tail/rudder damage
	if (Element == 53 || Element == 54 || Element == 55 || Element == 100 || Element == 56 || Element == 57 || Element == 58) // Tail damage
		PlaneFM::tail_damage += (element_integrity_factor * 10);

	// Cockpit, canopy, avionics, and pilot damage
	if (Element == 0 || Element == 1 || Element == 2 || Element == 3 || Element == 4 || Element == 5 || Element == 6) // Cockpit/Avionics damage and damage to the pilot
		PlaneFM::cockpit_damage += (element_integrity_factor * 10);
	*/
	if (Element >= 1 && Element < 111) // Turns out this is the only thing that works for some reason
		PlaneFM::total_damage += (element_integrity_factor);
	}

}

double test()
{
	return 10.0;
}

#pragma once