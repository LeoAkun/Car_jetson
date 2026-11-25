#!/bin/bash

CRON_ENTRY="@reboot /home/dlinano/workspace/traffic-light/app_start.sh >> /home/dlinano/workspace/traffic-light/app_start.log 2>&1"
if crontab -l | grep -q "$CRON_ENTRY"; then
    echo "START already exists."
else
    (crontab -l ; echo "$CRON_ENTRY") | crontab -
    echo "START added successfully."
fi
# CRON_ENTRY="@reboot sudo sh -c 'echo 200 > /sys/devices/pwm-fan/target_pwm'"
# if crontab -l | grep -q "$CRON_ENTRY"; then
#     echo "FAN already exists."
# else
#     (crontab -l ; echo "$CRON_ENTRY") | crontab -
#     echo "FAN added successfully."
# fi