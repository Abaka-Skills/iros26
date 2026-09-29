"""The cameras' relay: the RealSense cameras on USB, served on localhost over the network device
protocol (the same one the simulator serves), so the backend reads cameras only by address and never
needs root. It runs as a daemon and follows the cameras: one plugged in is opened and served, one
unplugged is dropped until it comes back; nobody has to do anything."""
