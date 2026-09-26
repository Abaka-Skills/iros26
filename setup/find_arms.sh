#!/bin/bash
# Put the USB Ethernet adapter on the arms' subnet, then ping the four arms.
# Usage: ./find_arms.sh          (add address + ping)
#        ./find_arms.sh --remove (remove the address again)
IFACE=en14
MY_IP=192.168.10.200
ARMS="192.168.10.2 192.168.10.3 192.168.10.4 192.168.10.5"

if [ "$1" = "--remove" ]; then
  sudo ifconfig $IFACE -alias $MY_IP && echo "Removed $MY_IP from $IFACE"
  exit
fi

if ! ifconfig $IFACE | grep -q "inet $MY_IP "; then
  sudo ifconfig $IFACE alias $MY_IP 255.255.255.0 || exit 1
  echo "Added $MY_IP to $IFACE"
fi

for ip in $ARMS; do
  if ping -c1 -t1 $ip >/dev/null 2>&1; then
    echo "$ip  UP    $(arp -n $ip | awk '{print $4}')"
  else
    echo "$ip  DOWN"
  fi
done
