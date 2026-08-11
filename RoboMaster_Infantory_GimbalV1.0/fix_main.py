import re

with open('D:/STM32WorkSpace/RoboMaster_Infantory_GimbalV1.0/Core/Src/main.c', 'r', encoding='utf-8') as f:
    text = f.read()

# 1. Add #include <stddef.h>
text = text.replace('//frame protocol', '#include <stddef.h>\r\n//frame protocol')

# 2. Add #define GIMBAL_CRC_OFFSET
text = text.replace('}GimbalState_Send_t;', '}GimbalState_Send_t;\r\n#define GIMBAL_CRC_OFFSET  offsetof(GimbalState_Send_t, crc)')

# 3. Replace the function
old_fn = 'static void usb_print_quat(const float *quat, int len)\r\n{\r\n  char buf[128];\r\n  int n = snprintf(buf, sizeof(buf),\r\n      "Q:%.3f,%.3f,%.3f,%.3f\\r\\n",\r\n      quat[0], quat[1], quat[2], quat[3]);\r\n  if (n > 0) {\r\n    CDC_Transmit_HS((uint8_t *)buf, (uint16_t)n);\r\n  }\r\n  (void)len;'
new_fn = 'static void usb_send_gimbal_state(float roll, float pitch, float yaw)\r\n{\r\n    GimbalState_Send_t state;\r\n    uint32_t crc = 0;\r\n    uint8_t *p = (uint8_t *)&state;\r\n    state.header[0] = 0x51;\r\n    state.header[1] = 0x59;\r\n    state.mode = 0;\r\n    state.roll = roll;\r\n    state.pitch = pitch;\r\n    state.yaw = yaw;\r\n    state.crc = 0.0f;\r\n    for (int i = 0; i < GIMBAL_CRC_OFFSET; i++) crc += p[i];\r\n    state.crc = (float)crc;\r\n    CDC_Transmit_HS((uint8_t *)&state, sizeof(state));\r\n}'
if old_fn in text:
    text = text.replace(old_fn, new_fn)
    print('Function replaced')
else:
    # Try finding the function
    idx = text.find('usb_print_quat')
    if idx >= 0:
        # Find function start and end
        fn_start = text.rfind('\n', 0, idx) + 1
        # Find next function or marker
        fn_end = text.find('\n/* ', idx)
        if fn_end < 0:
            fn_end = text.find('\nint __io_putchar', idx)
        old_block = text[fn_start:fn_end]
        text = text[:fn_start] + new_fn + text[fn_end:]
        print('Function replaced (find method)')
    else:
        print('Function NOT found!')

# 4. Replace the call
old_call = 'usb_print_quat(Hipnuc_HI14.hi14data.quat, 4);'
new_call = 'usb_send_gimbal_state(Hipnuc_HI14.hi14data.roll, Hipnuc_HI14.hi14data.pitch, Hipnuc_HI14.hi14data.yaw);'
text = text.replace(old_call, new_call)

# Save
with open('D:/STM32WorkSpace/RoboMaster_Infantory_GimbalV1.0/Core/Src/main.c', 'w', encoding='utf-8') as f:
    f.write(text)

print('Done. File size:', len(text), 'chars')

# Quick verification
for item in ['GIMBAL_CRC_OFFSET', '<stddef.h>', 'usb_send_gimbal_state', 'usb_print_quat']:
    print('  ', item, 'present' if item in text else 'ABSENT')
