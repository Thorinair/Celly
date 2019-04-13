/********
 * Interval Configuration
 *******/

/* Duration of the micro tick in ms. */
#define TICK_MICRO_TIME 5

/* The duration of a measurement during a micro tick.
 * Will be subtracted from TICK_MICRO_TIME.
 */
#define TICK_MICRO_TIME_MEASURE 1

/* Number of micro ticks after which sample tick fires. */
#define TICK_SAMPLE_COUNT 200

/* Number of sample ticks after which upload tick fires. */
#define TICK_UPLOAD_COUNT 60



/********
 * Misc. Configuration
 *******/

/* Wait time before the device starts in ms. */
#define PRE_CONNECT_DELAY 200

/* Time it takes in ms for a button to activate when holding. */
#define BUTTON_HOLD_TIME 2000