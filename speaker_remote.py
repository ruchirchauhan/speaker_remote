#!/usr/bin/python

""" Imports """
from decimal import *
from itertools import cycle
import time
import RPi.GPIO as GPIO
import lirc
import json
import traceback

""" Declare few important constants and global variables
    All the step counts defined here are the steps that rotor
    has to take in order to fulfill the function.
    Do not be misled by the names
"""
""" Stepper motor constants """
# total number of steps that rotor takes to complete 1 revolution
rotor_steps_tot = 32
# total number of steps rotor has to take to complete 1 revolution
# of output shaft. Reduction of 1/64. Total steps 32x64 = 2048
motor_steps_tot = rotor_steps_tot * 64

""" Volume/Bass levels """
# Will be used for mapping with knob rotation
level_min = 1
level_max = 100
levels_tot = level_max - level_min + 1

""" Volume/Bass knob constants """
# Total degree of rotation allowed. After which the knob can break
degrees_allowed = 300
knob_steps_tot = int(Decimal(motor_steps_tot)/360 * degrees_allowed)

""" Mapping level to rotor steps """
# Steps taken by the rotor per level : 17
# Every level change would mean moving these many steps
steps_in_level = knob_steps_tot/levels_tot
# Rounding off knob_steps_tot
knob_steps_tot -= knob_steps_tot % steps_in_level

""" GPIO pin setup """
# Volume control motor:
# Blue:   GPIO17 pin-11,
# Pink:   GPIO27 pin-13,
# Yellow: GPIO22 pin-15,
# Orange: GPIO23 pin-16
step_pins_vol = [17, 27, 22, 23]
# Bass control motor:
# Blue:   GPIO05 pin-29,
# Pink:   GPIO06 pin-31,
# Yellow: GPIO13 pin-33,
# Orange: GPIO19 pin-35
step_pins_bass = [5, 6, 13, 9]
tot_pins = len(step_pins_bass)
# Pre-configure steps
step = []
step = range(0, 4)
step[0] = [1, 0, 0, 0]
step[1] = [0, 1, 0, 0]
step[2] = [0, 0, 1, 0]
step[3] = [0, 0, 0, 1]
# This pool will take track of the last step
# that was ever taken in the lifetime of this program
step_pool = cycle(step)

""" Misc variables """
wait_time = .006
power_off = True
step_types = len(step)

""" Persistent information """
# These 2 inputs to be taken from file,
# for persistence across power cycles
persist_info = {
    "last_step_taken_vol":0,
    "last_step_taken_bass":0,
    "last_direction_vol":" ",
    "last_direction_bass":" ",
    "mute":False
}


def load_data():
    global persist_info
    f = open('speaker_remote_persistent.info', 'r')
    persist_info = json.load(f)
    f.close()


def persist_data():
    f = open('speaker_remote_persistent.info', 'w')
    json.dump(persist_info, f)
    f.close()


def gpio_setup():
    """ GPIO Setup """

    # Use BCM GPIO references
    # instead of physical pin numbers
    GPIO.setmode(GPIO.BCM)

    # Set all pins as output
    for pin in step_pins_vol:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, False)
    for pin in step_pins_bass:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, False)


def can_change(control_type, change_type, n):
    """ Decide if there is still scope to
    increase/decrease the output of the control
    :param control_type:
    :return:True/False
    """
    if control_type == 'volume' and change_type == 'down':
        if persist_info['last_step_taken_vol'] + n <= knob_steps_tot:
            return True
    elif control_type == 'volume' and change_type == 'up':
        if persist_info['last_step_taken_vol'] - n >= 0:
            return True
    elif control_type == 'bass' and change_type == 'down':
        if persist_info['last_step_taken_bass'] + n <= knob_steps_tot:
            return True
    elif control_type == 'bass' and change_type == 'up':
        if persist_info['last_step_taken_bass'] - n >= 0:
            return True
    elif control_type == 'mute':
        if persist_info['last_step_taken_vol'] + n <= knob_steps_tot:
            return True
    print 'Can not crank', control_type, change_type
    return False


def is_reversed(last_direction, curr_direction):
    """ Checks if the current direction for a control type
    is reverse of the previous direction for this control type
    :return: True/False
    """
    if last_direction and curr_direction != last_direction:
            return True
    return False


def clear_output_pins(step_pins):
    for pin in range(0, tot_pins):
        GPIO.output(step_pins[pin], False)  # Disable this pin


def move(step_pins, direction, start_step, is_reverse, n=steps_in_level):
    """
    :param n: number of steps to move
    :return: last step type and number of steps taken in this run
    """

    if direction == 'down':
        start_step = start_step + 1 if not is_reverse else start_step
        last_step = start_step + n
        dir = 1
    elif direction == 'up':
        start_step = start_step - 1 if not is_reverse else start_step
        last_step = start_step - n
        dir = -1
    else:
        return

    print 'start_step: ', start_step
    print 'last_step: ', last_step

    for curr_step in range(start_step, last_step, dir):
        curr_step_index = curr_step % step_types
        print 'Index ', curr_step, 'curr_step ', step[curr_step_index]
        for pin in range(0, tot_pins):
            xpin = step_pins[pin] # Get GPIO
            if step[curr_step_index][pin] != 0:
                GPIO.output(xpin, True) # Enable this pin
            else:
                GPIO.output(xpin, False) # Disable this pin

        time.sleep(wait_time)

    clear_output_pins(step_pins)

    return True, curr_step


def control_speaker(codeIR):
    """
    :param codeIR: IR code received from the remote, str type
    :return: None
    """
    global persist_info
    moved = False
    print codeIR, ' received'
    control_type, direction = codeIR.split('_')
    n = knob_steps_tot - persist_info['last_step_taken_vol'] \
        if control_type == 'mute' else steps_in_level

    if not can_change(control_type, direction, n):
        return

    if control_type == 'volume':
        moved, last_step_taken = move(
            step_pins_vol, direction, persist_info['last_step_taken_vol'],
            is_reversed(persist_info['last_direction_vol'], direction), n)
        persist_info['last_step_taken_vol'] = last_step_taken \
            if moved else persist_info['last_step_taken_vol']
        persist_info['last_direction_vol'] = direction
        persist_info["mute"] = False

    elif control_type == 'bass':
        moved, last_step_taken = move(
            step_pins_bass, direction, persist_info['last_step_taken_bass'],
            is_reversed(persist_info['last_direction_bass'], direction), n)
        persist_info['last_step_taken_bass'] = last_step_taken \
            if moved else persist_info['last_step_taken_bass']
        persist_info['last_direction_bass'] = direction

    elif control_type == 'mute':
        # Mute/Un-mute the device
        print 'Mute received'
        if persist_info["mute"]:
            return
        direction = 'down'
        moved, last_step_taken = move(
            step_pins_vol, direction, persist_info['last_step_taken_vol'],
            is_reversed(persist_info['last_direction_vol'], direction), n)
        persist_info['last_step_taken_vol'] = last_step_taken \
            if moved else persist_info['last_step_taken_vol']
        persist_info['last_direction_vol'] = direction
        persist_info["mute"] = True

def toggle_power():
    global power_off
    power_off = False if power_off else True

#############
# Main Code #
#############

# Setup GPIO pins
gpio_setup()
# Load persisted data
load_data()
# Wait some time to start
time.sleep(0.5)

""" Create a connection to lirc """
sock_id = lirc.init("speaker_remote", blocking = False)

# Continuously keep polling for any remote inputs
print 'Turn power on to start'
try:
    while True:
        control_type = lirc.nextcode()
        if control_type:
            control_type = control_type[0]

            if control_type == 'power':
                toggle_power()
                continue

            if power_off:
                # If power is off then do not serve any command
                # and continue polling
                continue

            control_speaker(control_type)

except KeyboardInterrupt:
    print 'Got KeyboardInterrupt'
except Exception as e:
    print 'Caught some error: ', e
    traceback.print_exc()
finally:
    persist_data()
    print 'Exiting program'
    exit(0)
