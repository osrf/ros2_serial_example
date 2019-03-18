# Response to a request for a manifest of serial types for the ros2_serial_example.
# Note that this is *not* intended to be sent over the ROS 2 network; we reuse
# the ROS 2 machinery to generate the CDR serialization/deserialization for this
# type so we can use it directly on the serial wire.

# Enum for the direction communication should take; from serial to ROS 2 or
# or vice-versa.  Used in the "direction" sequence below.
uint8 SERIALTOROS2=0
uint8 ROS2TOSERIAL=1

# Note that all sequences below *must* have the same size when serializing.
string[] topic_names     # The sequence of topic names to map to serial.
uint64[] serial_mappings # The serial number to map to the topic.  Even though
                         # it is a uint64, not all mappings may be valid; this
                         # is controlled by the value of topic_id_size_t.
string[] types           # The ROS 2 message type corresponding to the topic.
                         # Only types that have been compiled into the
                         # ros2_to_serial_bridge will be bridged.
uint8[] direction        # The direction (from serial to ROS 2 or vice-versa);
                         # one of the enums above.
