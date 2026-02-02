import pandas as pd
from datetime import datetime

# 1. Read the CSV, skipping the initial metadata rows
df = pd.read_csv('can_bus_messages.csv', skiprows=5)

# 2. Clean column names
df.columns = [c.replace('\n', ' ').strip() for c in df.columns]

# 3. Get current timestamp
timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

header_lines = [
    "/* AUTOMATICALLY GENERATED CAN ID DEFINITIONS */",
    f"/* Generated on: {timestamp} */",
    "#ifndef CAN_IDS_H",
    "#define CAN_IDS_H",
    "\n#include <stdint.h>\n",
    "/* Utility structure to map spreadsheet layout to C code */",
    "typedef struct __attribute__((packed)) {",
    "    uint32_t remote_id;  /* Bytes 0-3: Target Remote ID */",
    "    uint8_t  payload[4]; /* Bytes 4-7: Command Data */",
    "} CamperCanMsg_t;\n"
]

for _, row in df.iterrows():
    # Detect Priority headers to create sections in the .h file
    section = str(row.get('D0 B1', ''))
    if "priority" in section.lower():
        header_lines.append(f"\n/* --- {section.strip()} --- */")
        continue

    msg_id = str(row.get('Message ID', '')).strip()
    name = str(row.get('c def', '')).strip()
    comment = str(row.get('Comments', '')).strip()
    
    # Only process rows with a valid Hex ID and C definition
    if msg_id.startswith('0x') and name != 'nan' and name != '':
        line = f"#define {name:<32} ({msg_id})"
        if comment != 'nan' and comment != '':
            line += f" /* {comment} */"
        header_lines.append(line)

header_lines.append("\n#endif /* CAN_IDS_H */")

   
# Save the file
with open('can_ids.h', 'w') as f:
    f.write("\n".join(header_lines))

print(f"Success: 'can_ids.h' generated at {timestamp}")


# include <stdint.h>

# /* Standard Message Structure for the Camper CAN Protocol */
# typedef struct __attribute__((packed)) {
#     uint32_t remote_id;   /* Target device unique ID (Bytes 0-3) */
#     union {
#         uint8_t  u8[4];   /* Access data as individual bytes (Bytes 4-7) */
#         uint16_t u16[2];  /* Access data as half-words */
#         uint32_t u32;     /* Access data as a single word */
#         struct {          /* Example for Switch commands */
#             uint8_t switch_id;
#             uint8_t mode;
#             uint8_t reserved[2];
#         } sw;
#     } data;
# } CamperCanMsg_t;

# void StartCANTask(void const * argument) {
#     CAN_RxHeaderTypeDef Header;
#     uint8_t RxData[8];
#     CamperCanMsg_t incoming;

#     for(;;) {
#         if (osMessageGet(canQueueHandle, osWaitForever).status == osEventMessage) {
#             HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Header, RxData);

#             // Directly map the 8 bytes to our structure
#             memcpy(&incoming, RxData, 8);

#             // Byte swapping: CAN is typically Big Endian, 
#             // STM32 is Little Endian. We fix the ID once here:
#             incoming.remote_id = __REV(incoming.remote_id);

#             if (incoming.remote_id == MY_LOCAL_ID || incoming.remote_id == 0xFFFFFFFF) {
#                 switch(Header.StdId) {
#                     case SW_SET_ON:
#                         // Use the payload bytes as defined in your CSV
#                         uint8_t target_switch = incoming.payload[0]; 
#                         turn_on_light(target_switch);
#                         break;
                        
#                     case ERROR_OVER_CURRENT:
#                         display_fault(incoming.payload[0]); // Data 0 from your CSV
#                         break;
#                 }
#             }
#         }
#     }
# }