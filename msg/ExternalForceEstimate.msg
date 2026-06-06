# Estimated external force/disturbance acting on the vehicle
# Computed using Generalized Momentum Observer (GMO)
uint64 timestamp		# time since system start (microseconds)

float32[3] force		# estimated external force in NED frame [N]
float32[3] force_body		# estimated external force in body frame [N]
float32 force_magnitude		# magnitude of estimated force [N]
float32 confidence		# detection confidence [0, 1]
