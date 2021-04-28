#pragma once

class param_stuff
{
protected:
	//bool testvalue;

public:
	bool fuelvalue;
	double fuelnumber;

	/*test_stuff()
		: testvalue()
		, testnumber()
	{}
	~test_stuff() {}*/
	
	void fuelparam(bool value)
	{
		fuelvalue = value;
		/*if (testvalue = true)
		{
			testnumber = 1.5;
		}
		if (testvalue = false)
		{
			testnumber = 0.001;
		}*/
	};


};