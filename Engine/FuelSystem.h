#pragma once
 
// also calculation for the mass of the fuel and position of it

// JP8 weight 6.65lbs/gal ?


class fuelsystem
{
public:

	double volume; // capacity of tank
	double fuel; // amount of fuel in tank

	double x;
	double y;
	double z;

	fuelsystem(double _volume = 0, double _fuel = 0)
		: volume(_volume)
		, fuel(_fuel)
		, x(0)
		, y(0)
		, z(0)
	{}
	~fuelsystem() {}

	// add what is possible, return remaining if full
	double addFuel(const double addition)
	{
		double space = volume - fuel;
		if (space < addition)
		{
			fuel = volume; // set to max
			return (addition - space); // overflow
		}
		fuel += addition;
		return 0;
	}

	double decFuel(const double decrement)
	{
		if (fuel < decrement)
		{
			double tmp = decrement - fuel;
			fuel = 0; // set to min
			return tmp; // remaining
		}
		fuel -= decrement;
		return 0;
	}

	double getSpace() const
	{
		return (volume - fuel);
	}
};

class AH6FuelSystem
{
protected:
	bool is_unlimited_fuel;
	double previous_usage;

	fuelsystem MainTank;
	fuelsystem AuxTank;

public:

	bool isIdleCutoff; // true means no fuel flow

	bool isFuelFlow;

	AH6FuelSystem() 
		: is_unlimited_fuel(false)
		, previous_usage(0)
		, MainTank(400 / 2.2046) // lb to kg
		, AuxTank(400 / 2.2046)
		, isIdleCutoff(false)
		, isFuelFlow(false)
	{}
	~AH6FuelSystem() {}

	void initCold()
	{
		isIdleCutoff = true;
		isFuelFlow = false;
	}
	void initHot()
	{
		isIdleCutoff = false;
		isFuelFlow = true;
	}

	/* 
	// for weight-balance calculation,
	// we need amount of fuel in each tank and position
	double getFuelMass()
{
		// JP8 weight 6.65lbs/gal ?
		//return getInternalFuel() * weightconstant;
		return 0;
	}
	*/

	// is low fuel indication
	bool isLowFuel() const
	{
		// check remining fuel
		if (getInternalFuel() <= 45)
		{
			return true;
		}
		return false;
	}

	// called on initialization
	void setInternalFuel(const double fuel) // <- in kg
	{
		MainTank.fuel = 0;
		AuxTank.fuel = 0;
		refuelAdd(fuel);
	}
	double getInternalFuel() const
	{
		return (MainTank.fuel + AuxTank.fuel);
	}

	void refuelAdd(const double fuel) // <- in kg
	{		// distribute fuel to each tank for weight balance
		double addition = fuel;
		addition = MainTank.addFuel(addition);
		addition = AuxTank.addFuel(addition);
	}

	void setIdleCutOff(float value)
	{
		if (value == 1)
		{
			isIdleCutoff = true;
			return;
		}
		isIdleCutoff = false;
	}


	void setUnlimitedFuel(bool status)
	{
		is_unlimited_fuel = status;
	}

	double getUsageSinceLastFrame() const
	{
		return previous_usage;
	}
	void clearUsageSinceLastFrame()
	{
		previous_usage = 0;
	}

	void update(const double rpm, const double frameTime)
	{
		if (is_unlimited_fuel == true)
		{
			isFuelFlow = true;
			return;
		}

		double fueltmp =  rpm * 0.0364 * frameTime;
		previous_usage += fueltmp; // add to usage since last time updated
		// TODO: transfer of fuel between tanks

		fueltmp = AuxTank.decFuel(fueltmp);
		fueltmp = MainTank.decFuel(fueltmp);

		if (getInternalFuel() > 0 && isIdleCutoff == false)
		{
			isFuelFlow = true;
		}
		else
		{
			isFuelFlow = false;
		}
	}

};