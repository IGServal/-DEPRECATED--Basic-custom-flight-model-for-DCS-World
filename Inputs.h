#pragma once
// Used in ed_fm_set_command() 
enum InputCommands
{
	// commands from command_defs.lua

	//general controls
	resetTrim = 97,
	FBW_override = 121,
	EnginesOn = 309,
	EnginesOff = 310,

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

	Reserved // placeholder
};