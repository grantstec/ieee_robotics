#!/bin/bash
# This script sets up GPIO permissions for running without sudo

# Create gpio group if it doesn't exist
sudo groupadd -f gpio

# Add current user to gpio group
sudo usermod -a -G gpio robot

# Create udev rules for GPIO access
echo 'Creating udev rule for GPIO access'
sudo tee /etc/udev/rules.d/99-gpio.rules > /dev/null << 'EOF'
SUBSYSTEM=="gpio", KERNEL=="gpiochip[0-9]*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:gpio /dev/%k; chmod 660 /dev/%k'"
EOF

# Apply the rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Direct GPIO permission for immediate use
sudo chmod a+rw /dev/gpiochip0

echo "GPIO permissions have been set up."
echo "You may need to log out and log back in for group changes to take effect."
