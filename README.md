# FlowWrapper

Program flow: startup from no power, hangs in main loop checking for button presses. If start button pressed, run startup sequence and home all axis, wait 2s then run. Continuously keep checking for stop button presses. If stop button pressed, stop machine. If settings button pressed while stopped, set paper speed(untested, speed is hardcoded for this version). Continue checking for button presses.

This program is runnable. Homing is successful, startup is successful. All axis are rehomed as part of the startup process, and all VFDs are disabled/reenabled. Jaws run on a stop-go timing which limits to one speed. Need to add in a jaw encoder and control it through a PID. Also need to add checking for mis-aligned patties.

timingScalar was added to compensate for accumulated error in the encoder counts. Prior to that could only run about 30 patties through before placement error was too great(accumulated error, probably due to either missing encoder counts or more likely the fact I am using a floating point scalar to align the paper and feeder drives).

