This program is a user interface for a TAPR TICC time interval counter running on a Raspberry Pi computer. 

The TAPR TICC is a small, high-resolution time interval counter suitable for measuring the relative phase
between two precision frequency sources.  It can compare the relative phase between two 10 MHz sinewave inputs
with a resolution below 100 ps.

The Raspberry Pi 3 Model B (RPi) [3] is a small and low cost Linux computer that can be run “headless”
without a monitor, keyboard or mouse via its built-in Wi-Fi interface and an SSH connection.  As such,
it is an ideal way to capture data from the TICC without tying up a more expensive “real” computer.

Operation of a TICC/RPi clock measurement system is supported by a the TestDB Linux interface program
that connects to the TICC through a virtual COM port, receives the TICC data stream, and captures it
in the form of MJD timetagged phase data to a file and an optional PostgreSQL database.  The latter is
a particularly convenient way to archive, monitor and retrieve the clock data at LAN-connected workstations.
