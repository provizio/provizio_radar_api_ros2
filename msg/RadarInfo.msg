int8 UNKNOWN_RANGE = -1
int8 SHORT_RANGE = 0
int8 MEDIUM_RANGE = 1
int8 LONG_RANGE = 2
int8 ULTRA_LONG_RANGE = 3
int8 HYPER_LONG_RANGE = 4

std_msgs/Header header
string serial_number            # Radar's serial number
int8 current_range              # One the range constants declared above
int8[] supported_ranges         # All ranges supported by the radar in the current multiplexing mode
int8 current_multiplexing_mode  # Reserved, currently always -1
