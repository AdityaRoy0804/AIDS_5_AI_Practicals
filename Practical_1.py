# Create a simple reflex based agent that takes decision to turn on/off the light based on the enviornment.
# Author: Aditya Kumar Roy

is_day_light = input("Is the daylight there:?").lower()
is_human_there = input("Is the human there:?").lower()

if is_day_light == "no" or is_day_light == "n"  and is_human_there == "yes" or is_human_there == "y":
    print("LIGHT ON")
else:
    print("LIGHT OFF")