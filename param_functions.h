#pragma once
// This file details what happens when various options in DCS are set on or off.
class param_stuff
{
protected:

public:
	bool fuelvalue;
	
	void fuelparam(bool value)
	{
		fuelvalue = value;
	};


	bool invincible_value;

	void invincible(bool value)
	{
		invincible_value = value;
	};


};
