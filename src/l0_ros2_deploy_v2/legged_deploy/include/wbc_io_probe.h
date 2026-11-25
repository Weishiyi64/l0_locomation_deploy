#pragma once

#include <csignal>

// Offline diagnostic mode of the existing rl_controller executable.
// Called before ROS initialization; never publishes commands or starts hardware.
int runWbcIoProbe(int argc, char** argv, const volatile sig_atomic_t& stop_requested);
