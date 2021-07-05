#!/usr/bin/env python3

# Import statements
from ev3dev2.sensor.lego import TouchSensor, ColorSensor
from ev3dev2.motor import MoveSteering, OUTPUT_B, OUTPUT_C, MoveTank, SpeedPercent
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sound import Sound
from time import sleep, time


# Functions are in the semi-chronological order that they get used

# Rotate X degrees in a given direction (clockwise by default) using a given
# unit (e.g. the time taken to do a 360) as a reference.
# E.g. rotate(90, final_time, False) rotates 90 degrees anticlockwise using
# the time taken to do a 360
def rotate(degrees, units, clockwise):
    if clockwise:
        left_speed = 10
        right_speed = -10
    else:
        left_speed = -10
        right_speed = 10

    factor = 360 / degrees

    # uses a pre-calculated degree value
    # (based on the wheel's dimensions) to perform a rotation
    drive.on_for_degrees(SpeedPercent(left_speed),
                         SpeedPercent(right_speed), units / factor)


# Drive across tiles (horizontally in diagram). If forwards is False,
# the robot can reverse the same distance.
def drive_across(forwards):
    if forwards:
        left_speed = 15
        right_speed = 15
    else:
        left_speed = -15
        right_speed = -15

    drive.on_for_rotations(SpeedPercent(left_speed),
                           SpeedPercent(right_speed), 0.58)


# Reports back the tile number after incrementing it
# Updates this depending on if it's going across or downwards
def report_tile(tile, across):
    if across:
        sound.beep()
        tile += 1
    else:
        sound.beep()
        sound.beep()
        tile += 15

    sound.speak(str(tile))
    return tile


# For less precise rotations when turning left
def rotate_left(rtn_spd, rtn):
    drive.on_for_rotations(SpeedPercent(-rtn_spd), SpeedPercent(rtn_spd), rtn)


# For less precise rotations when turning right
def rotate_right(rtn_spd, rtn):
    drive.on_for_rotations(SpeedPercent(rtn_spd), SpeedPercent(-rtn_spd), rtn)


# Check the left of the robot for something dark
def check_tile_left(rtn_spd, rtn):
    temp = 1 # to know how many eighths of a rotation the robot has rotated

    # Check the colour sensor every eighth of a rotation
    # to see if line was found
    while cl.value() >= 15 and temp < 8:
        rotate_left(rtn_spd, rtn * 0.125)
        temp += 1

    # Did we find the line?
    if cl.value() < 15:
        # Yes, return True and temp
        return True, temp
    else:
        # No, rotate ourselves back to the centre and return False
        rotate_right(rtn_spd, rtn * 0.125 * temp)
        return False, temp


# Check the right of the robot for something dark
def check_tile_right(rtn_spd, rtn):

    temp = 1 # to know how many eighths of a rotation the robot has rotated

    # Check the colour sensor every eighth of a rotation
    # to see if line was found
    while cl.value() >= 15 and temp < 8:
        rotate_right(rtn_spd, rtn * 0.125)
        temp += 1

    # Did we find the line?
    if cl.value() < 15:
        # Yes, return True
        return True, temp
    else:
        # No, rotate ourselves back to the centre and return False
        rotate_left(rtn_spd, rtn * 0.125 * temp)
        return False, temp


# Proceeds across hardcoded distances, scanning tiles as we encounter them,
# until we find the given "stop tile."
# E.g. if stop_tile is 10, stop at tile 10.
def follow_path_across(tile, stop_tile):
    global rtn, rtn_spd, rtn_default, rtn_change, found_left, found_right
    global temp, off_course, tries

    while tile < stop_tile:
        if cl.value() <= 15:  # tile found?
            tile = report_tile(tile, True)

            if off_course == 1:  # found on left rotation?
                # drive forwards, and try to center myself
                drive_across(True)
                rotate_right(rtn_spd, rtn * 0.125 * temp)
                if tile == stop_tile:
                    drive.on_for_rotations(SpeedPercent(-15),
                                           SpeedPercent(-15), 0.35)
            elif off_course == 2:  # found on right rotation?
                # drive forwards, and try to center myself
                drive_across(True)
                rotate_left(rtn_spd, rtn * 0.125 * temp)
                if tile == stop_tile:
                    drive.on_for_rotations(SpeedPercent(-15),
                                           SpeedPercent(-15), 0.35)

            # Reset variables
            tries = 0
            temp = 1
            rtn = rtn_default
            off_course = 0

            # After 2 tries if it can't find something dark
            # it will try and attempt to realign itself
        elif tries >= 2 and cl.value() > 15:
            sound.speak("Alas, I am lost...")

            # Perform course correction
            while True:

                # Use temp to know how much to rotate to centre ourself
                found_left, temp = check_tile_left(rtn_spd, rtn)

                found_right, temp = check_tile_right(rtn_spd, rtn)

                if found_left:
                    off_course = 1
                    break
                elif found_right:
                    off_course = 2
                    break
                rtn += rtn_change

                # If the robot is at risk of over-rotating
                if rtn >= 0.5:
                    # Reverse and reset rotation
                    drive_across(False)
                    rtn = rtn_default

        # While it has driven less than 10 dark tiles
        # it will keep driving until this condition
        # is met
        if tile < 10 and off_course == 0:
            drive_across(True)
            tries += 1
    return tile


# Proceed downwards, scanning tiles as we encounter them, until we find the
# given "stop tile." E.g. if stop_tile is 55, stop at tile 55.
def follow_path_down(tile, stop_tile):
    global rtn, rtn_spd, rtn_default, rtn_change
    start_time = time()

    # Turn on motors and sleep to avoid double counting a tile
    steer_pair.on(steering=0, speed=10)
    sleep(3)

    while tile < stop_tile:
        time_taken = time() - start_time
        steer_pair.on(steering=0, speed=10) # ensure robot is moving

        # If robot somehow missed a black or a white tile, down_fix is
        # activated via a time out
        if time_taken >= 8:
            steer_pair.on(steering=0, speed=0)
            sound.speak("Time out")

            tile = down_fix(1, 0, tile)
            start_time = time() # reset start time

        if cl.value() <= 14:  # found a black tile?
            # yes, stop and call report_tile
            steer_pair.on(steering=0, speed=0)

            tile = report_tile(tile, False)

            # Turn on motors if we are not at stop_tile
            if tile != stop_tile:
                steer_pair.on(steering=0, speed=10)
                sleep(3)

            start_time = time() # reset start time
        elif cl.value() >= 50:  # found a white tile?
            # yes, try to find a tile by rotating
            steer_pair.on(steering=0, speed=0)
            sound.speak("White tile")
            tile = down_fix(1, 0, tile)
            start_time = time() # reset start time

    return tile


# Down fix is used for the robot fails to follow the dark tile
# lines (after tile 10) and has to realign itself, this is done
# by going back to the last known dark tile and perform angle
# rotations to try and realign itself back on to the dark tile line.
def down_fix(tries, rotation, tile):

    while True:
        # Reverse and look for black tile
        drive.on_for_rotations(SpeedPercent(-10), SpeedPercent(-10), 2)
        sleep(8)

        sound.speak("Still on " + str(tile))

        # Stops once it gets back to previous tile

        # If uneven number of tries rotates right left
        if tries % 2 != 0:
            sound.speak("left")
            rotation += 0.03
            rotate_left(10, rotation)

        # If even number of tries rotates left right and
        # increases rotations
        elif tries % 2 == 0:
            sound.speak("right")
            rotation += 0.03
            rotate_right(10, rotation)

        start_time = time()
        time_taken = 0
        steer_pair.on(steering=0, speed=10)

        sleep(3)

        # Checks for tiles
        tile_found = 0

        # Drives forward with new rotation until it finds a tile
        while tile_found < 2:
            time_taken = time() - start_time
            if cl.value() <= 12:
                tile_found += 1
            elif cl.value() >= 50:
                tile_found += 1
            elif time_taken >= 7:
                tile_found += 1

        steer_pair.on(steering=0, speed=0)

        # Checks if the tile isn't black
        if cl.value() >= 50:
            sound.speak("A dud?!")
            tries += 1
        # If the tile is black
        elif cl.value() < 15:
            return tile
        # If we timed out
        elif time_taken >= 7:
            sound.speak("Missed tiles")
            tries += 1


# Rotate 90 degrees and scan the tiles in front for the bottle. If the
# ultrasonic sensor reads less than the max value, then the robot will try to
# guess the tile number based on the robot's read distance.
# else if nothing is found in the maximum range the robot will keep searching
# up to tile 100
def rotate_and_scan(searches):
    searches += 1

    drive.on_for_rotations(SpeedPercent(10), SpeedPercent(10), 1.25)
    sleep(6)

    # Rotate 90 degrees anticlockwise
    rotate(90, rtn_units, False)
    sleep(2)  # to avoid false distance readings

    # Scan distance from object
    distance = round(us.distance_centimeters, 2)

    if round(distance) <= 100:  # something was found
        sound.speak("Something was located " +
                    str(distance) + " centimeters away.")

        # Calculate tile number based on distance
        if distance < 40:
            # closest square
            tower_tile = 3 * searches - 2
        elif distance < 70:
            # middle square
            tower_tile = 3 * searches - 1
        else:
            # furthest square
            tower_tile = 3 * searches

        # Tower is found so it reports the towers location and
        # closes the program
        sound.speak("I think the tower is on tile " + str(tower_tile))
        sound.speak("Mission completed. Self destructing in 3")
        sound.speak("2")
        sound.speak("1")
        sleep(0.5)
        sound.speak("Just kidding.")
        sound.speak("Shutting down...")
        exit(0)
    else:
        # Keep looking
        sound.speak("Nope.")
        rotate(90, rtn_units, True)
    return searches


# Initialize objects
steer_pair = MoveSteering(OUTPUT_B, OUTPUT_C)
drive = MoveTank(OUTPUT_B, OUTPUT_C)
us = UltrasonicSensor()
sound = Sound()
cl = ColorSensor()
ts = TouchSensor()
cl.mode = 'COL-REFLECT'

# Wheel values in CM
wheel_radius = 2.8
wheel_distance = 5.3 # distance between the two wheels
angle_rotate = 360

# Use a derived formula to calculate degrees required for a 360
wheel_degrees = (wheel_distance * angle_rotate) / wheel_radius

# Decide which units to use for rotation
rtn_units = wheel_degrees

# Set rotation variables
rtn_spd = 25
rtn_default = 0.25
rtn = rtn_default
rtn_change = 0.125

# Set tile related variables
tile = 0
tries = 0
temp = 1
stop_tile_across = 10
stop_tile_down = 55
searches = 0

# Used to determine where the robot should turn to center itself
off_course = 0  # 0 for false, 1 for left, 2 for right

# =========================================================
# For easy testing: put code here and uncomment exit(0)
# =========================================================

# exit(0)

# =============
# Main routine starts here
# =============

# Announce start
sound.speak("Ready, set, go!")

# Cross initial gap
drive.on_for_rotations(SpeedPercent(10), SpeedPercent(10), 1)

# Turn right on tile 1
rotate(90, rtn_units, True)
sleep(2)

# Drive across from tile 1 to tile 10
tile = follow_path_across(tile, stop_tile_across)

# Rotate at tile 10 to face downwards
rotate(90, rtn_units, True)
sleep(2)

# Drive downwards from tile 10 to tile 55
tile = follow_path_down(tile, stop_tile_down)

# Rotate and scan when passing tiles 55, 70, 85 and 100
while searches < 4:
    stop_tile_down += 15
    searches = rotate_and_scan(searches)
    tile = follow_path_down(tile, stop_tile_down)

# Report failure if searches is greater than 3 and no tower is found
sound.speak("Mission failed. We will get them next time.")
sound.speak("Assuming that there is a next time.")
sound.speak("Shutting down...")
