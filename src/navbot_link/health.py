"""Names of the signals and enum values that describe the robot's health. No I/O."""

HEALTH_SIGNALS = (
    "balance_state", "balance_fault", "overspeed_tripped", "motor_test_active", "rc_valid", "ch5_position",
    "voltage_filtered_v", "voltage_min_raw_v", "loop_interval_max_us",
    "link_tx_drop_uart", "link_tx_drop_usb", "link_rx_bad_uart", "link_rx_bad_usb",
    "link_req_dropped", "link_undecodable",
    "uptime_s", "heap_free_b", "heap_min_free_b", "stack_free_loop_b", "stack_free_link_b",
)

# balance_state values, in enum order (BalanceController.h).
BALANCE_STATES = ("Disabled", "Arming", "Run", "Commissioning", "Fault")

# balance_fault values, in enum order (BalanceController.h).
FAULT_REASONS = (
    "None", "StaleImu", "StaleWheels", "StaleBus", "InvalidInput", "BadDt", "Fall", "Overspeed",
    "InvalidEnvelope", "InvalidConfig", "AuthorityExhausted", "TiltSaturated",
)
