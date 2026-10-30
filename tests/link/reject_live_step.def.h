// Must not compile: a live step below zero. run_host_test.sh expands this file and expects the build to stop.
PARAM_LIVE(tuning_gain, "", fixture::TuningGroup, gain, -0.5)
