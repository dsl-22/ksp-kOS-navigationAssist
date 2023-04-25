// Name: Navigation Assist Functions
// Author: dsl-22
// Version: 0.1
// kOS Version: 1.4.0.0
// KSP Version: 1.12.5
// Description:
//      Boot script for NA1.

wait until ship:unpacked.
copyPath("0:/NAFunctions.ks","").
copyPath("0:/NA1.ks","").
core:doEvent("Open Terminal").
print("Navigation assist ready - Press Ctrl+C to abort.").
print("run NA1. (to start)").
