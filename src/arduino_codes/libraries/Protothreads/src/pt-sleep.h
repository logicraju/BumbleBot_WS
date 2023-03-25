#ifndef __PT_SLEEP_H__
#define __PT_SLEEP_H__

/**
 * Arduino sleep for protothreads
 *
 * Put the protothread to sleep. Yield to other protothreads while sleeping.
 *
 * \param pt A pointer to the protothread control structure.
 * \param delay Delay in milliseconds.
 *
 * \hideinitializer
 */
#define PT_SLEEP(pt, delay) \
{ \
  do { \
    static unsigned long protothreads_sleep; \
    protothreads_sleep = millis(); \
    PT_WAIT_UNTIL(pt, millis() - protothreads_sleep > delay); \
  } while(false); \
}

#endif /* __PT_SLEEP_H__ */