uint64 timestamp                  # time since system start (microseconds)

uint8 STATE_IDLE          = 0    # running, waiting for enable command from companion
uint8 STATE_EV_ONLY     = 1      # EV pos on, GPS off
uint8 STATE_BLEND_TO_GPS  = 2    # EV pos on, GPS on — EV degraded, GPS warming up
uint8 STATE_GPS_FALLBACK  = 3    # EV pos off, GPS on
uint8 STATE_BLEND_TO_EV = 4      # EV pos on, GPS on — EV recovering

uint8    state                    # current arbiter state (see STATE_* constants)
bool     ev_pipeline_alive      # vehicle_visual_odometry is fresh
bool     ev_being_fused         # EKF2 is actively accepting EV position measurements
float32  ev_noise_current         # EKF2_EVP_NOISE value currently in effect (m)
float32  blend_progress           # 0.0-1.0 fraction through current blend window (0 otherwise)
uint8    primary_ekf_instance     # index of the primary EKF2 estimator
