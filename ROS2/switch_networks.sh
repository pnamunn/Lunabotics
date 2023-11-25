#!/bin/bash
# Must run this bash script with sudo
# Takes in 2 args: <network_to_disconnect_from> <network_to_connect_to>

# Assumes that you have previously connected $2 and do not need to enter in its password again

# If number of args not equal to 2, exit script
if [ "$#" -ne 2 ]
then
    echo -e "Improper usage >> Script takes in 2 args & must be run in sudo to work.
    Example of proper usage: sudo bash switch_networks.sh <network_to_disconnect_from> <network_to_connect_to>"
    exit 1
fi

echo "Disconnecting from $1..."
nmcli con down $1

echo "Connecting to $2..."
nmcli dev wifi connect $2

echo "Here's your current connection statuses:"
nmcli c