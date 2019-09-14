#ifndef MAIN_H
#define MAIN_H
extern uint32_t blink_interval_ms;

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
  BLINK_ALWAYS_ON = UINT32_MAX
};


#endif