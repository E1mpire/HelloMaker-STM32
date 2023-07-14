#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "config.h"

class Encoder {
	public:
		Encoder(Encoder_TypeDef _encoder, uint32_t _arr, uint32_t _psc, uint32_t _counts_per_rev);
		void init();
		int32_t read();
		int32_t getRPM();
		void set_pos(int32_t pos);

		/* total value since startup */
		int32_t position;
		/* last read of timer */
		uint16_t last_timer;
		/* last difference between timer reads */
		int16_t last_timer_diff;

	private:
		Encoder_TypeDef encoder;
		uint32_t arr;
		uint32_t psc;
		uint32_t counts_per_rev;
		uint32_t lcounts_per_rev;
		uint32_t rcounts_per_rev;
		unsigned long prev_update_time_;
		long prev_encoder_ticks_;
};

#endif // _ENCODER_H_
