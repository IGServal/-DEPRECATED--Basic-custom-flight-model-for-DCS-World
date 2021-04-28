#pragma once

//-------------------------------------------------------
// Start of Simulation Constants 
//-------------------------------------------------------
namespace Helicopter
{
	const double		wingSpan_FT				= 27.33;		//  wing-span (ft)
	const double		wingArea_FT2			= 96.9;			//  wing area (ft^2)
	const double		diskArea				= 586.778;		// main rotor disk area (ft^2)
	const double		pi						= acos(-1.0);	// Pi (3.14159....) - use value from math.h
	const double		degtorad				= pi/180.0;	// 
	const double		radiansToDegrees		= 180.0/pi;		// Conversion factor from radians to degrees - use value from math.h
	const double		inertia_Ix_KGM2			= 446.0;		// Reference moment of inertia (kg/m^2)
	const double		inertia_Iy_KGM2			= 1219.6;		// Reference moment of inertia (kg/m^2)
	const double		inertia_Iz_KGM2			= 979.1;		// Reference moment of inertia (kg/m^2)
	const double		meterToFoot				= 3.28084;		// Meter to foot conversion factor
	const double		FootToMeter				= 1/meterToFoot;	// foot to meter conversion factor
	const double		lbToNewton				= 4.44822162825;
	const double 		lbFootToNewtonMeter		= 1.35581795;
}