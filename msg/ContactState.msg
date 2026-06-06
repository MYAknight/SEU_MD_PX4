# Contact detection state machine output
# Used to trigger perching grasping mechanisms
uint64 timestamp		# time since system start (microseconds)

uint8 STATE_NO_CONTACT = 0
uint8 STATE_POSSIBLE = 1
uint8 STATE_CONFIRMED = 2
uint8 STATE_STABLE = 3
uint8 STATE_SLIPPING = 4

uint8 state			# current contact state (see STATE_* constants)
bool should_close		# true when stable perching is confirmed
float32 contact_duration	# duration of current contact event [s]
float32[3] contact_force	# last measured contact force [N]
