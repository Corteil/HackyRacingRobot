#!/bin/bash
# Launch the robot dashboard server in a terminal, then open the browser.
lxterminal --title="Robot Dashboard" -e python3 /home/pi/Code/HackyRacingRobot/robot_dashboard.py &
sleep 5
xdg-open http://localhost:5000
