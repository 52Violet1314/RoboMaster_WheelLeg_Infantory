with open('D:\\STM32WorkSpace\\RoboMaster_Infantory_GimbalV1.0\\Core\\Src\\main.c', 'rb') as f:
    data = f.read()
try:
    text = data.decode('utf-8')
    print('OK utf8', len(data), 'bytes', len(text), 'chars')
except:
    print('BAD utf8')
checks = ['GimbalState_Send_t', 'GIMBAL_CRC_OFFSET', 'usb_send_gimbal_state', 'CDC_Transmit_HS', 'offsetof', '<stddef.h>']
for c in checks:
    print(c, 'OK' if c in text else 'MISSING')
# Check old things removed
for c in ['usb_print_quat', 'float header[2]', 'uint8_t mode = 0']:
    print(c, 'REMOVED' if c not in text else 'STILL THERE')
