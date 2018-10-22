#!/bin/bash

# Bluetooth Flash for Linux
#
# Flashes a compiled binary to an STM32 board using the stm32flash utility
#
# Author: Georges Troulis


################################################################################
#   Command Packets for the Flash Mediator
################################################################################

RTF='\xf7\x14\x41\x7f'  # Resuest To Flash  (sent)
RDF='\x56\xa8\x8a\x65'  # Ready To Flash    (received)
FFL='\x93\x87\x78\x39'  # Flashing Finished (sent)

ACK='\xaa\xcc\xcc\xaa'  # Acknowledge       (bidirectional)

################################################################################
#   Other Variables/Constants
################################################################################

# The Serial Port to flash over
PORT=$1
if [ -z $1 ]; then
  PORT='/dev/ttyUSB0'
  (>&2 echo "Warning: no port specified; using default port $PORT")
else
  echo "Using serial port $PORT"
fi

# The flash baud rate
BAUD=57600

# The name of the program to execute to perform the flashing (varies per OS)
FLASHER='stm32flash'

################################################################################
#   Main Application
################################################################################

# TODO send RTF, receive ACK, and wait for RDF with Timeout

$FLASHER -w IARC_FirmwareDev.bin -b $BAUD $PORT

# TODO send FFL
