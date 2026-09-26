import asyncio
import rospy
from sensor_msgs.msg import Joy
from evdev import InputDevice, categorize, ecodes, KeyEvent, list_devices
import signal
import argparse

joystick_data = Joy()
joystick_data.axes = [0] * 9
joystick_data.buttons = [0] * 12

# Check the event in the system
devices = [InputDevice(path) for path in list_devices()]
device_found = False

for device in devices:
    if device.name in ['Microsoft X-Box 360 pad', 'hongjingda Cosmic Byte Ares', 'Cosmic Byte Ares']:
        device_path = device.path
        device_found = True

if device_found: 
    gamepad = InputDevice(device_path)
else:
    print('Controller not found. Please connect gamepad controller')

max_value = [0.5] * 9
min_value = [0.1] * 9

joystick_calibrated = False

print("Calibration started. Rotate the sticks and triggers to their extremes, press all four direction buttons, and then press MODE to start.")

parser = argparse.ArgumentParser()
parser.add_argument('--robot', type=str, default='svanm2', help='Name of the robot')
args = parser.parse_args()

async def main():
    global joystick_calibrated
    rospy.init_node("joy_commands")
    joy_pub = rospy.Publisher(f'/{args.robot}/joystick_data', Joy, queue_size=1)
    
    async for event in gamepad.async_read_loop():
        if event.type == ecodes.EV_KEY:
            keyevent = categorize(event)
            if keyevent.keystate == KeyEvent.key_down:
                joystick_data.buttons = [0] * 12  # Reset all buttons to 0
                if keyevent.scancode == 304:  # A button
                    joystick_data.buttons[0] = 1
                elif keyevent.scancode == 307:  # X button
                    joystick_data.buttons[1] = 1
                elif keyevent.scancode == 305:  # B button
                    joystick_data.buttons[2] = 1
                elif keyevent.scancode == 308:  # Y button
                    joystick_data.buttons[3] = 1
                elif keyevent.scancode == 310:  # LB button
                    joystick_data.buttons[4] = 1
                elif keyevent.scancode == 311:  # RB button
                    joystick_data.buttons[5] = 1
                elif keyevent.scancode == 317:  # THUMBL button
                    joystick_data.buttons[6] = 1
                elif keyevent.scancode == 318:  # THUMBR button
                    joystick_data.buttons[7] = 1
                elif keyevent.scancode == 314:  # BACK button
                    joystick_data.buttons[8] = 1
                elif keyevent.scancode == 315:  # START button
                    joystick_data.buttons[9] = 1
                elif keyevent.scancode == 316:  # MODE button
                    # pressing MODE should be the last step of joystick calibration
                    for max_val, min_val in zip(max_value, min_value):
                        if not (max_val == 0.5 or min_val == 0.1):
                            max_value[8] = 0.0
                            min_value[8] = 0.0
                    joystick_data.buttons[10] = 1
                

        elif event.type == ecodes.EV_ABS:
            if event.code == 0:  # LX
                index = 0
                if (event.value > max_value[index]):
                    max_value[index] = event.value
                if (event.value < min_value[index]):
                    min_value[index] = event.value
                joystick_data.axes[0] = normalize(event.value, min_value[index], max_value[index])
            elif event.code == 1:  # LY
                index = 1
                if (event.value > max_value[index]):
                    max_value[index] = event.value
                if (event.value < min_value[index]):
                    min_value[index] = event.value
                joystick_data.axes[1] = -normalize(event.value, min_value[index], max_value[index])
            elif event.code == 3:  # RX
                index = 2
                if (event.value > max_value[index]):
                    max_value[index] = event.value
                if (event.value < min_value[index]):
                    min_value[index] = event.value
                joystick_data.axes[2] = normalize(event.value, min_value[index], max_value[index])
            elif event.code == 4:  # RY
                index = 3
                if (event.value > max_value[index]):
                    max_value[index] = event.value
                if (event.value < min_value[index]):
                    min_value[index] = event.value
                joystick_data.axes[3] = -normalize(event.value, min_value[index], max_value[index])
            elif event.code == 5:  # RT
                index = 4
                if (event.value > max_value[index]):
                    max_value[index] = event.value
                if (event.value < min_value[index]):
                    min_value[index] = event.value
                joystick_data.axes[4] = normalize(event.value, min_value[index], max_value[index])
            elif event.code == 2:  # LT
                index = 5
                if (event.value > max_value[index]):
                    max_value[index] = event.value
                if (event.value < min_value[index]):
                    min_value[index] = event.value
                joystick_data.axes[5] = normalize(event.value, min_value[index], max_value[index])
            elif event.code == 16:  # D-pad Left/Right
                index = 6
                if (event.value > max_value[index]):
                    max_value[index] = event.value
                if (event.value < min_value[index]):
                    min_value[index] = event.value
                joystick_data.axes[6] = float(event.value)
            elif event.code == 17:  # D-pad Up/Down
                index = 7
                if (event.value > max_value[index]):
                    max_value[index] = event.value
                if (event.value < min_value[index]):
                    min_value[index] = event.value
                joystick_data.axes[7] = float(-event.value)

        # print("max: ", max_value)
        # print("min: ", min_value)

        calibration_done = all(max_val != 0.5 and min_val != 0.1 for max_val, min_val in zip(max_value, min_value))

        if calibration_done and not joystick_calibrated:
            print("Joystick calibrated successfully! Sending commands over rostopic...")
            print("The neutral joystick data is: ", joystick_data)
            joystick_calibrated = True

        if joystick_calibrated:
            joy_pub.publish(joystick_data)

def normalize(value, min_val, max_val):
    return (2 * float(value - min_val)) / (max_val - min_val) - 1.0

def handler_function():
    print("Shutdown request received ... ")
    loop.stop()

loop = asyncio.get_event_loop()
loop.add_signal_handler(signal.SIGINT, handler_function)
try:
    loop.run_until_complete(main())
finally:
    loop.close()
