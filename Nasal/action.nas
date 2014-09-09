##
#  action-sim.nas   Updates various simulated features including:
#                    fuel pressure, oil pressure every frame
# $Id$
# 
# This file is based on Lionceau, which is based on PA-24-250

#   Initialize local variables
var rpm = nil;
var fuel_pres = 0.0;
var oil_pres = 0.0;
var ias = nil;
var fuel_flow = nil;

# set up filters for these actions

var fuel_pres_lowpass = aircraft.lowpass.new(0.5);
var oil_pres_lowpass = aircraft.lowpass.new(0.5);


var init_actions = func {
    setprop("engines/engine[0]/fuel-flow-gph", 0.0);
    setprop("/surface-positions/flap-pos-norm", 0.0);
    setprop("/instrumentation/airspeed-indicator/indicated-speed-kt", 0.0);
    setprop("/instrumentation/airspeed-indicator/pressure-alt-offset-deg", 0.0);
    setprop("/accelerations/pilot-g", 1.0);

    # Request that the update fuction be called next frame
    settimer(update_actions, 0);
}


var update_actions = func {
##
#  This is a convenient cludge to model fuel pressure and oil pressure
#  EGT must be between 1200 - 1700 deg-F, maybe around 1470 F (800 C)
#  Fuel pressure (fp)  is adjusted so fp = 0.5 @ 500rpm and 9.0 at 2700 rpm
#  oil pressure (op) is adjusted so op = 25 @ 500rpm, 50 @ 1500, and 90 @ 2700
#  actually oil pressure should be around 115 when starting and warm-up
#  cylinder temperature max is 500 F, but not emulated at this moment

    rpm = getprop("/engines/engine/rpm");
    if (rpm > 600.0) {
        fuel_pres = rpm / 258.82 - 1.43;
        oil_pres = rpm / 33.85 + 10.25; 
    } else {
        fuel_pres = 0.0;
        oil_pres = 0.0;
    }


##
# outputs
##
    setprop("/engines/engine/fuel-pressure-psi", fuel_pres_lowpass.filter(fuel_pres));
    setprop("/engines/engine/oil-pressure-psi", oil_pres_lowpass.filter(oil_pres));

    settimer(update_actions, 0);
}

# Setup listener call to start update loop once the fdm is initialized
#
setlistener("/sim/signals/fdm-initialized", init_actions);
