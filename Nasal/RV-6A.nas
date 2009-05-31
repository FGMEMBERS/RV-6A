# Only move flaps if voltage is sufficient
# $Id$

var flapsDown = controls.flapsDown;
controls.flapsDown = func(v){
	var volts = getprop("/systems/electrical/outputs/flaps");
        flapsDown(v);
}
