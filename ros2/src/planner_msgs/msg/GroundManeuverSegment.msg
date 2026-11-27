# Explicit stop-to-stop ground primitive. Distances are metres, angles radians.
uint8 STRAIGHT=0
uint8 PIVOT=1
uint8 kind
# Signed distance along heading, or signed yaw rotation about the reference q.
float64 signed_amount
# Inclusive indices into PathFollower.path; adjacent segments share a boundary.
uint32 start_index
uint32 end_index
bool stop_at_end true
# Ordered observed crossings inside this straight primitive, in positive
# metres from its start along signed_amount's direction. No stop is required
# at these distances; the end above still requires a stop. Empty for pivots.
# Each crossing must be observed within the existing intermediate pose limits.
float64[] pass_through_distances
