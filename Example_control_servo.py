import time

from pymavlink import mavutil


def set_servo_pwm(servo_n, microseconds):
    """ Sets AUX 'servo_n' output PWM pulse-width.

    Uses https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO

    'servo_n' is the AUX port to set (assumes port is configured as a servo).
        Valid values are 1-3 in a normal BlueROV2 setup, but can go up to 8
        depending on Pixhawk type and firmware.
    'microseconds' is the PWM pulse-width to set the output to. Commonly
        between 1100 and 1900 microseconds.

    """
    # master.set_servo(servo_n+8, microseconds) or:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,  # first transmission of this command
        servo_n,
        microseconds,  # PWM pulse-width
        0, 0, 0, 0, 0  # unused parameters
    )





if __name__ == "__main__":
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14770')
    master.wait_heartbeat()
    print('Connection success with navigator')
    for us in range(1100, 1500, 50):
        set_servo_pwm(3, us)
        time.sleep(0.125)