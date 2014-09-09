#
# adjuster.nas : for restricting unwanted sensitivity around zero on each control axis
# Tatsuhiro Nishioka

var powers = { aileron : 1.1, elevator : 1.1, rudder : 1.3 };

adjustControlAroundZero = func(axis, powerValue) {
  var src = getprop("/controls/flight/" ~ axis);
  var dest = src;
  var sign = 1.0;
  if (src < 0.0) {
    sign = -1.0;
    src = - src;
  }
  if (src != 0.0) {
    dest = sign * math.pow(src, powerValue);
#    print(axis ~ " : " ~ src ~ " -> " ~ dest);
  } 
  setprop("/controls/flight/adjusted/" ~ axis, dest);
}

#
# updateControl: update flight controls (aileron, elevator, and rudder);
# remove a control from the list if you don't want adjust it
# you can also change power for 
#
updateControl = func {
  foreach (var axis; ["aileron", "elevator", "rudder"]) {
#print(axis ~ " : " ~ powers[axis] ~ "\n");
    adjustControlAroundZero(axis, powers[axis]);
  }
  settimer(updateControl, 0);
}

setlistener("/sim/signals/fdm-initialized", updateControl);
