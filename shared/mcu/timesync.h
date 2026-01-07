
#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdint.h>

/*
 * Time Sync
 * Determines a best-estimate time sync between two systems
 * This variant is tuned for embedded systems and low-rate radio protocols (e.g. ESB):
 * - 32Bit integer operation if possible
 * - expecting low latencies
 * - can handle longer periods of no / lower-rate communication
 */

typedef struct time_sync
{
	uint32_t measurements;
	uint32_t errors;
	uint64_t last_timestep;
	uint64_t last_timestamp;
	float expected_factor;
	float factor; // Includes conversion ratio and drift
	float correction_up;
	float correction_down;
	int const_offset_us;
	uint32_t outlier_min;
	uint32_t outlier_max;
	uint32_t outlier_drift;
	uint16_t init_period;
	uint16_t grace_period;
} time_sync_t;

static inline void init_time_sync(time_sync_t *time)
{
	time->measurements = 0;
	time->errors = 0;
	time->last_timestep = 0;
	time->last_timestamp = 0;
	time->factor = 1.0f; // Includes conversion ratio and drift
	time->correction_up = 0.02f;
	time->correction_down = 0.002f;
	time->const_offset_us = 0;
	time->outlier_min = 1000; // More than 100us latency is bad
	time->outlier_max = 1000; // More than 100us latency is bad
	time->outlier_drift = 500; // For long periods, additionally accept drift of X us per second
	time->init_period = 1; // Don't allow factor to be adjusted in init period (it's 1:1)
	time->grace_period = 10;
}

/**
 * Bias of 0 means using closest timestep within overflow, positive bias favors this timestep being after last stored timestep
 * e.g. if timestep is strictly after the last timestep passed into update_time_synced,
 * then a bias of overflow/2 guarantees it is so, and allows for the longest time delta before overflow occurs
 */
static inline uint64_t rebase_timestamp(const time_sync_t *time, uint32_t timestep, uint32_t overflow, int bias)
{
	// assert(overflow <= (1<<31)); // Else we need to use int64_t for e.g. bias
	if (time->measurements == 0)
		return timestep;
	uint32_t basestep = time->last_timestep&(overflow-1);
	//int32_t passed = (uint32_t)((int64_t)timestep - (int64_t)basestep);
	int32_t passed;
	if (basestep < timestep)
		passed = +(int32_t)(timestep - basestep);
	else
		passed = -(int32_t)(basestep - timestep);
	uint64_t rebased_step = time->last_timestep + passed;
	int32_t hi = +(int32_t)(overflow/2)+bias, lo = -(int32_t)(overflow/2)+bias;
	if (passed < lo) rebased_step += overflow;
	else if (passed > hi) rebased_step -= overflow;
	return rebased_step;
}

static inline uint64_t rebase_timestamp_reference(const time_sync_t *time, uint32_t timestep, uint32_t overflow, uint64_t reference)
{
	// assert(overflow <= (1<<31)); // Else we need to use int64_t for e.g. bias
	if (time->measurements == 0)
		return timestep;
	int correction = 0, bias = 0;
	uint32_t passed_us, passed;
	if (reference >= time->last_timestamp)
	{
		passed_us = (uint32_t)(reference - time->last_timestamp);
		passed = (uint32_t)((float)passed_us / time->factor);
		correction = (int)(passed/overflow);
		bias = (int)(passed%overflow);
	}
	else
	{
		passed_us = (uint32_t)(time->last_timestamp - reference);
		passed = (uint32_t)((float)passed_us / time->factor);
		correction = -(int)(passed/overflow);
		bias = -(int)(passed%overflow);
	}
	// Rebase with bias (to correct for passed < overflow)
	uint64_t rebased_step = rebase_timestamp(time, timestep, overflow, bias);
	// Then correct for passed > overflow
	rebased_step += (long)correction*overflow;	
	/* if (correction != 0)
		LOG_INF("Rebased %u to %llu using last %llu, passed %u (%uus with factor %f): correction %d, bias %d",
			timestep, rebased_step, time->last_timestep, passed, passed_us, time->factor, correction, bias); */
	return rebased_step;
}

static inline uint64_t get_time_synced_past(const time_sync_t *time, uint64_t timestep)
{
	uint32_t passed = (uint32_t)(time->last_timestep-timestep);
	passed = (uint32_t)((float)passed * time->factor);
	return time->last_timestamp - passed;
}

static inline uint64_t get_time_synced_future(const time_sync_t *time, uint64_t timestep)
{
	uint32_t passed = (uint32_t)(timestep - time->last_timestep);
	passed = (uint32_t)((float)passed * time->factor);
	return time->last_timestamp + passed;
}

static inline uint64_t get_time_synced(const time_sync_t *time, uint64_t timestep)
{
	if (timestep < time->last_timestep)
		return get_time_synced_past(time, timestep);
	else
		return get_time_synced_future(time, timestep);
}

static uint64_t update_time_synced(time_sync_t *time, uint64_t timestep, uint64_t measurement)
{ // Received packet timestep with best real-time estimation being measurement

	if (time->errors >= 10)
	{ // Too many errors, reset
		time->measurements = 0;
		time->errors = 0;
		time->factor = 1;
	}

	if (time->measurements++ < time->init_period)
	{ // Initialise first measurement
		float lerp = 1.0f - 1.0f/time->init_period;
		if (time->measurements > 2)
			time->factor = time->factor*lerp + (float)(measurement-time->last_timestamp)/(float)(timestep-time->last_timestep) * (1-lerp);
		time->last_timestamp = measurement;
		time->last_timestep = timestep;
		return measurement;
	}
	//LOG_INF("Received time sync measurement %lld (diff %lld) and steps %lld (diff %lld) with factor %f!",
	//	measurement, (int64_t)measurement-(int64_t)time->last_timestamp, timestep, (int64_t)timestep-(int64_t)time->last_timestep, time->factor);

	if (timestep < time->last_timestep)
	{ // Can't update past timestamp
		time->errors++;
		return get_time_synced_past(time, timestep);
	}

	// Predict real-time of timestep based on time-local estimation of time synchronisation
	uint32_t passed = (uint32_t)(timestep - time->last_timestep);
	uint32_t passed_us = (uint32_t)((float)passed * time->factor);
	uint64_t time_pred = time->last_timestamp + passed_us;
	//LOG_INF("Predicted update time to be %llu (last %llu, passed %u/%fus, factor %f)!", time_pred, time->last_timestamp, passed, passed_us, time->factor);

	uint32_t outlier_var = passed_us*time->outlier_drift/1000000;

	// Update time-local estimation of time synchronisation with new measurement
	if (measurement < time_pred)
	{ // New minimum, upper bound for the actual time sync, correct and adapt factor slowly
		uint32_t diff_us = (uint32_t)(time_pred-measurement);
		if (diff_us < time->outlier_min+outlier_var)
		{
			//LOG_INF("Timesync received correct diff -%uus (allowed %u) with measurement %llu (diff %lld) and steps %llu (diff %lld) with factor %f!",
			//	diff_us, time->outlier_min+outlier_var, measurement, (int64_t)measurement-(int64_t)time->last_timestamp, timestep, (int64_t)timestep-(int64_t)time->last_timestep, time->factor);
			time->factor -= (float)diff_us * time->correction_down / (float)passed;
			time_pred = measurement;
			time->errors = 0;
		}
		else if (time->measurements > time->grace_period)
		{
			time->errors++;
		}
	}
	else
	{ // Higher than predicted, assume not significantly delayed, but primarily factor, so adapt slowly
		uint32_t diff_us = (uint32_t)(measurement-time_pred);
		if (diff_us < time->outlier_max+outlier_var)
		{
			//LOG_INF("Timesync received correct diff +%uus (allowed %u) with measurement %llu (diff %lld) and steps %llu (diff %lld) with factor %f!",
			//	diff_us, time->outlier_max+outlier_var, measurement, (int64_t)measurement-(int64_t)time->last_timestamp, timestep, (int64_t)timestep-(int64_t)time->last_timestep, time->factor);
			float correction_us = (float)diff_us * time->correction_up;
			time->factor += correction_us / (float)passed;
			time_pred += (uint32_t)correction_us; // Probably unnecessary, will be small
			time->errors = 0;
		}
		else if (time->measurements > time->grace_period)
		{
			time->errors++;
		}
	}

	// Lerp to expected 1 - may be unnecessary
	time->factor = time->factor*0.95f + 0.05;

	//LOG_INF("Updated synced time from measurement %lld to synced %lld with factor %f", measurement, time_pred, time->factor);

	time->last_timestamp = time_pred;
	time->last_timestep = timestep;
	return time_pred + time->const_offset_us;
}

#endif // TIME_SYNC_H