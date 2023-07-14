#ifndef Statistic_h
#define Statistic_h
#define STAT_USE_STDEV

#ifdef STAT_USE_STDEV
#include <math.h>
#endif

class Statistic 
{
	public:
	Statistic();
	void clear();
	void add(float);
	long count();
	float sum();
	float average();
	float minimum();
	float maximum();


#ifdef STAT_USE_STDEV
	float pop_stdev();	    // population stdev
	float unbiased_stdev();
#endif

protected:
	long _cnt;
	float _store;           // store to minimise computation
	float _sum;
	float _min;
	float _max;
#ifdef STAT_USE_STDEV
	float _ssqdif;		    // sum of squares difference
#endif
};

#endif
// END OF FILE