#include <AP_HAL.h>
#include <AP_Notify.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "RCInput.h"
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>

using namespace PX4;

#define THR_FAILSAFE 1
#define THR_RC_FAILSAFE 920
#define THR_RC_LOST 910
#define THR_CHANNEL 2

extern const AP_HAL::HAL& hal;

void PX4RCInput::init(void* unused)
{
        memset(&_rcin, 0, sizeof(_rcin));
        memset(&_new_rcin, 0, sizeof(_rcin));
	_perf_rcin = perf_alloc(PC_ELAPSED, "APM_rcin");
	_rc_sub = orb_subscribe(ORB_ID(input_rc));
	if (_rc_sub == -1) {
		hal.scheduler->panic("Unable to subscribe to input_rc");		
	}
	clear_overrides();
        pthread_mutex_init(&rcin_mutex, NULL);
}

bool PX4RCInput::new_input() 
{
    pthread_mutex_lock(&rcin_mutex);
#if THR_FAILSAFE
    bool valid = _rcin.timestamp_last_signal != _last_read || _rcin.timestamp_publication != _last_read || _override_valid;
#else
    bool valid = _rcin.timestamp_last_signal != _last_read || _override_valid;
#endif
    pthread_mutex_unlock(&rcin_mutex);
    return valid;
}

uint8_t PX4RCInput::num_channels() 
{
    pthread_mutex_lock(&rcin_mutex);
    uint8_t n = _rcin.channel_count;
    pthread_mutex_unlock(&rcin_mutex);
    return n;
}

uint16_t PX4RCInput::read(uint8_t ch) 
{
	if (ch >= RC_INPUT_MAX_CHANNELS) {
		return 0;
	}
        pthread_mutex_lock(&rcin_mutex);
	_last_read = _rcin.timestamp_last_signal;
	_override_valid = false;
	if (_override[ch]) {
            uint16_t v = _override[ch];
            pthread_mutex_unlock(&rcin_mutex);
            return v;
	}
	if (ch >= _rcin.channel_count) {
            pthread_mutex_unlock(&rcin_mutex);
            return 0;
	}
	uint16_t v = _rcin.values[ch];
        pthread_mutex_unlock(&rcin_mutex);
        return v;
}

uint8_t PX4RCInput::read(uint16_t* periods, uint8_t len) 
{
	if (len > RC_INPUT_MAX_CHANNELS) {
		len = RC_INPUT_MAX_CHANNELS;
	}
	for (uint8_t i = 0; i < len; i++){
		periods[i] = read(i);
	}
	return len;
}

bool PX4RCInput::set_overrides(int16_t *overrides, uint8_t len) 
{
	bool res = false;
	for (uint8_t i = 0; i < len; i++) {
		res |= set_override(i, overrides[i]);
	}
	return res;
}

bool PX4RCInput::set_override(uint8_t channel, int16_t override) {
	if (override < 0) {
		return false; /* -1: no change. */
	}
	if (channel >= RC_INPUT_MAX_CHANNELS) {
		return false;
	}
	_override[channel] = override;
	if (override != 0) {
		_override_valid = true;
		return true;
	}
	return false;
}

void PX4RCInput::clear_overrides()
{
	for (uint8_t i = 0; i < RC_INPUT_MAX_CHANNELS; i++) {
		set_override(i, 0);
	}
}

void PX4RCInput::_timer_tick(void)
{
	perf_begin(_perf_rcin);

	// check for status and channel values and update them consistently
	bool rc_updated = orb_check(_rc_sub, &rc_updated) == 0 && rc_updated;
	if (rc_updated) {
		// double-buffer rc input to avoid overwriting last valid channel values with zeros from PX4IO
		if (orb_copy(ORB_ID(input_rc), _rc_sub, &_new_rcin) == OK) {
			pthread_mutex_lock(&rcin_mutex);
			if (!_new_rcin.rc_lost) {
				// rc ok or failsafe, copy full rcin structure
				memcpy(&_rcin, &_new_rcin, sizeof(_rcin));
#if THR_FAILSAFE
				if (_new_rcin.rc_failsafe) {
					// valid RC signal, but contains a failsafe flag (e.g. TX switched off or out of range)
					_rcin.values[THR_CHANNEL] = THR_RC_FAILSAFE;
				}
#endif
			}
			else {
#if THR_FAILSAFE
				// PX4IO or RC receiver failure, copy only timestamps and flags (values[] are zero now) 
				_rcin.timestamp_last_signal = _new_rcin.timestamp_publication;
				_rcin.timestamp_publication = _new_rcin.timestamp_publication;
				_rcin.rc_lost = _new_rcin.rc_lost;
				_rcin.rc_failsafe = _new_rcin.rc_failsafe;
				_rcin.values[THR_CHANNEL] = THR_RC_LOST;
#endif
			}
			pthread_mutex_unlock(&rcin_mutex);

			// notify about RC trouble
			AP_Notify::flags.failsafe_radio = _new_rcin.rc_failsafe;
			AP_Notify::flags.radio_lost = _new_rcin.rc_lost;
		}
		else {
			// note, we rely on the vehicle code checking new_input()
			// and a timeout for the last valid input to handle rcin timeout
			AP_Notify::flags.radio_lost = true;
		}
	}
	perf_end(_perf_rcin);
}

#endif
