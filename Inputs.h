#pragma once
// Used in ed_fm_set_command() 
enum InputCommands
{
	// commands from command_defs.lua

	//General controls
	resetTrim = 97,
	FBW_switch = 121, //Toggle

	//Engine commands
	EnginesOn = 309,
	LeftEngineOn = 311,
	RightEngineOn = 312,
	/* left on is 311, right on is 312*/

	EnginesOff = 310,
	LeftEngineOff = 313,
	RightEngineOff = 314,
	/* left off is 313, right off is 314*/

	//Pitch
	PitchUp = 195,
	PitchUpStop = 196,
	PitchDown = 193,
	PitchDownStop = 194,
	trimUp = 95,
	trimDown = 96,

	//Roll
	RollLeft = 197,
	RollLeftStop = 198,
	RollRight = 199,
	RollRightStop = 200,
	trimLeft = 93,
	trimRight = 94,

	//Yaw
	rudderleft = 201,
	rudderleftend = 202,
	rudderright = 203,
	rudderrightend = 204,
	ruddertrimLeft = 98,
	ruddertrimRight = 99,

	//throttle,
	throttle = 3002,
	throttleAxis = 3006,
	throttleupLeft = 161,
	throttleupRight = 163,
	throttledownLeft = 162,
	throttledownRight = 164,
	ThrottleIncrease = 1032,
	ThrottleDecrease = 1033,
	ThrottleStop = 1034,

	//gear commands
	geartoggle = 68,
	gearup = 430,
	geardown = 431,
	WheelBrakeOn = 74,
	WheelBrakeOff = 75,
	tailhook = 69,

	//air brake commands

	AirBrakes = 73,
	AirBrakesOn = 147,
	AirBrakesOff = 148,

	//flap commands
	flapstoggle = 72,
	flapsup = 145,
	flapsdown = 146,

	// joystick axis commands
	JoystickPitch = 2001,
	JoystickRoll = 2002,
	JoystickYaw = 2003,
	JoystickThrottle = 2004,

	// modes
	nav		= 105,
	bvr		= 106,
	vs		= 107,
	bore	= 108,
	helm	= 109,
	fi0		= 110,
	a2g		= 111,
	gun		= 113,

	// autopilot modes
	//autopilot = 62, // [A]
	//route_follow = 429,// [Left alt + 6]
	autopilot_horiz = 62,  // [A]
	autopilot_alt = 59,  // [H]
	autopilot_alt_roll = 387,  // [H]
	autopilot_reset = 408,  // [Left alt + 9]

	Reserved // placeholder
};