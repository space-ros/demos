#
# User data format for ROS2 app in using RACS2 Bridge
#
# --- Header data ---
uint32 cfs_message_id    # Message ID of cFS desitnation app
uint32 body_data_length  # Body data size in byte
# --- Body data (serializied data is preffered): array of byte ---
# Note:
#   - In Python: "byte" means bytes object
#   - In C++   : "byte" means uint8_t type
byte[] body_data