# Click the link to see demo (https://drive.google.com/file/d/1okYeeKvK2BzJnQWiH7ah6bS1l9RepO-X/view?usp=sharing)
#
# (Dynamic) Navigation Assist for KSP
#
# Kerbal Space Program script library that automates common rocket flight procedures such as:
# • Launch and precise steering (optimized using dynamically calculated gravity turn maneuver)
# • Orbit circularization (calculates least eccentric orbit using derived rocket equation, optimized processing time using ternary search algorithm)
# • Landing (executes hoverslam maneuver based on factors such as changing gravity, engine isp, and surface angle of landing surface)
#
# Implementation:
# 
# • Place NABoot.ks in (.\Ships\Script\boot)
# • Place NAFUnctions.ks and NA1.ks in (.\Ships\Script)
# • (Within KSP) Place any kOS-compatible CPU on selected ship
# • Select NABoot.ks as your boot file on the CPU
# • Launch your ship
