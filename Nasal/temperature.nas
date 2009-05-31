#
# temperature.nas
# Engine temperature simulation for Yasim models
# DO NOT USE THIS SCRIPT FOR JSBSim SINCE IT IS JUST REDUNDANT
# 
# Created on Feb-06-2009 by Tatsuhiro Nishioka
# $Id$
#

var DEGC2KELVIN = 273.15;
var INHG2PA = 3386.3886;
var PSI2PA = 6894.7573;
var LITTER2CUBICMETER = 0.001;

# 
# TemperatureUtil: Temperature Utility class among temperature related classes
# 
TemperatureUtil = {
    new : func() {
	var obj = {
	    p_amb_sea_level : getprop("/environment/pressure-sea-level-inhg") * INHG2PA,
	    t_amb_sea_level : getprop("/environment/temperature-sea-level-degc") + DEGC2KELVIN,
	    displacement : 5.4,
            maxRPM: 2700
	};
	setprop("/instrumentation/egt/egt-degc", getprop("/environment/temperature-degc"));
	return obj;
    },

    do_temperatures : func {
	me.p_amb_sea_level = getprop("/environment/pressure-sea-level-inhg") * INHG2PA;
	me.t_amb_sea_level = getprop("/environment/temperature-sea-level-degc") + DEGC2KELVIN;
	me.t_amb = getprop("/environment/temperature-degc") + DEGC2KELVIN;
	me.p_amb = getprop("/environment/pressure-inhg") * INHG2PA;
    },

    do_equivalence_ratio : func {
	var mixture = getprop("/controls/engines/engine/mixture");
	var thi_sea_level = 1.3 * mixture;
	me.equivalence_ratio = thi_sea_level * me.p_amb_sea_level / me.p_amb;
    },

    # approx. calculation of combustion efficiency
    do_combustion_efficiency : func {
	if (me.equivalence_ratio < 0.9) {
	    me.combustion_efficiency = 0.98
	} else {
	    me.combustion_efficiency = 0.98 - (0.577 * (me.equivalence_ratio - 0.9));
	    if (me.combustion_efficiency < 0.1) {
		me.combustion_efficiency = 0.1;
	    }
        }
    },

    do_air_flow : func {
	var r_air = 287.3;
	var volumetric_efficiency = 0.8;
	var map = getprop("/engines/engine/mp-osi") * PSI2PA;       # manifold pressure (Pa)
	var rpm = getprop("/engines/engine/rpm");
        if (rpm < 500) {
	    rpm = 0; 
	}
	me.rpm = rpm;
	me.rho_air = me.p_amb / (r_air * me.t_amb);
	var v_dot_air = (me.displacement * rpm / 60) / 2 * volumetric_efficiency;   
	me.m_dot_air = v_dot_air * map / (r_air * me.t_amb);
#	setprop("/engines/engine/m_dot_air", me.m_dot_air);
    },

    do_fuel_flow : func {
	me.m_dot_fuel = me.m_dot_air * me.equivalence_ratio / 14.7;
#	setprop("/engines/engine/m_dot_fuel", me.m_dot_fuel);
    },

    updateUtil : func {
	me.do_temperatures();
	me.do_equivalence_ratio();
	me.do_air_flow();
	me.do_fuel_flow();
	me.do_combustion_efficiency();
#	setprop("/engines/engine/combustion_efficiency", me.combustion_efficiency);
#	setprop("/engines/engine/equivalence_ratio", me.equivalence_ratio);
    }
};

#
# Cylinder Head Temperature - simulates the cylinder head temperature
# This class is based on doCHT in JSBSim's FGPiston.cpp
#
CylinderHeadTemperature = {
  #
  # constructor
  # Inputs
  #  displacement        : displacement of an engine in litter  
  #  maxPRM              : Maximum RPM of an engine
  #  specific_heat       : specific heat for the engine heat. 880 for alminium, and 450 for steel
  #  combustion_temp_coef: grater value results in lower cht temp, default is 0.22
  # 
  new: func(displacement, maxRPM, specific_heat=800, mass_cylhead = 10, combustion_temp_coef=0.22, heat_capacity_coef = 2.75) {
      var obj = TemperatureUtil.new();
      obj.parents = [CylinderHeadTemperature, TemperatureUtil];
      obj.maxRPM = maxRPM;
      obj.cht = getprop("/environment/temperature-degc") + DEGC2KELVIN;
      obj.displacement = displacement * LITTER2CUBICMETER;
      obj.specific_heat = specific_heat;
      obj.combustion_temp_coef = combustion_temp_coef;
      obj.heat_capacity_coef = heat_capacity_coef;
      obj.mass_cylhead = mass_cylhead;
      setprop("/engines/engine/cht-fix-degf", obj.cht);
      setprop("/instrumentation/cht/cht-degf", getprop("/environment/temperature-degf"));
# for adjusting CHT on-the-fly, uncomment these; development use only
#      setprop("/engines/engine/temp/cht-combustion-coef", combustion_temp_coef);
#      setprop("/engines/engine/temp/cht-heat-capacity-coef", heat_capacity_coef);
      return obj;
  },

  #
  # update: called every frame for updating CHT
  #
  update: func {
      me.updateUtil();
      var h1 = -95.0;
      var h2 = -3.95;
      var h3 = -0.05 * me.maxRPM;
      var calorific_value_fuel = 47300000;

# for development use only
#      me.combustion_temp_coef = getprop("/engines/engine/temp/cht-combustion-coef");
#      me.heat_capacity_coef = getprop("/engines/engine/temp/cht-heat_capacity_coef");

      var dt = 1 / 60 * getprop("/sim/speed-up");
      var ias = getprop("/velocities/airspeed-kt"); # Should this be "/instrumentation/airspeed-indicator/indicated-speed-kt"?
      var arbitary_area = 1.0;
      var temperature_difference = me.cht - me.t_amb;
      var v_apparent = ias * 0.5144444;
      var v_dot_cooling_air = arbitary_area * v_apparent;
      var m_dot_cooling_air = v_dot_cooling_air * me.rho_air;
      var dqdt_from_combustion = me.m_dot_fuel * calorific_value_fuel * me.combustion_efficiency * me.combustion_temp_coef;
      var dqdt_forced = (h2 * m_dot_cooling_air * temperature_difference) +
	  (h3 * me.rpm * temperature_difference / me.maxRPM);
      var dqdt_free = h1 * temperature_difference;
      var dqdt_cylinder_head = dqdt_from_combustion + dqdt_forced + dqdt_free;
      var heat_capacity = me.specific_heat * me.mass_cylhead * me.heat_capacity_coef;
      me.cht = me.cht + (dqdt_cylinder_head / heat_capacity) * dt;
      var cht_degf = (me.cht - DEGC2KELVIN) * 1.8 + 32;
# for development use only
#      setprop("/engines/engine/temp/dqdt_free", dqdt_free);
#      setprop("/engines/engine/temp/dqdt_forced", dqdt_forced);
#      setprop("/engines/engine/temp/dqdt_combustion", dqdt_from_combustion);
#      setprop("/engines/engine/temp/dqdt_cylhead", dqdt_cylinder_head);
#      setprop("/engines/engine/temp/dqdt_diff", dqdt_cylinder_head / heatCapacityCylinderHead);

      setprop("/engines/engine/cht-fix-degf", cht_degf);
      setprop("/instrumentation/cht/cht-degf", cht_degf);
  }
};

#
# Exhaust Gas Temperature observer (EGT)
# This is mainly for YASim since it doesn't seem showing egt correctly.
# If you use JSBSim, then you might not need this unless engine parameters are wrong.
#
# displacement: displacement of the engine in SI (1 litre = 0.001 SI)

ExhaustGasTemperature = {
#
# new(displacement);
# displacement : displacement of the engine (Liter)
#
    new : func(displacement) {
	var obj = TemperatureUtil.new();
	obj.parents = [ExhaustGasTemperature, TemperatureUtil];
	obj.p_amb_sea_level = getprop("/environment/pressure-sea-level-inhg") * INHG2PA;
	obj.combustion_efficiency = 0;
	obj.displacement = displacement * 0.001;
	setprop("/instrumentation/egt/egt-degc", getprop("/environment/temperature-degc"));
	setprop("/instrumentation/egt/egt-degf", getprop("/environment/temperature-degf"));
	return obj;
    },

    update : func {
	#
	# This function is almost the same as doEGT() in JSBSim
	# except this uses some approx. calculations
	me.updateUtil();
	var cp_air=1005; # Constant pressure for air's specific heat (J/Kg/K)
	var cp_fuel=1700; # for fuel (J/Kg/K)
	var calorific_value_fuel = 47300000;

	if (getprop("/engines/engine/running") == 1) {
	    var enthalpy_exhaust = me.m_dot_fuel * calorific_value_fuel * me.combustion_efficiency * 0.33;
	    var heat_capacity_exhaust = (cp_air * me.m_dot_air) + (cp_fuel * me.m_dot_fuel);
	    var delta_T_exhaust = enthalpy_exhaust / heat_capacity_exhaust;
	    var egt = me.t_amb + delta_T_exhaust - DEGC2KELVIN;
            var egt_degf = egt * 1.8 + 32;
	    setprop("/instrumentation/egt/egt-degc", egt);
	    setprop("/instrumentation/egt/egt-degf", egt_degf);
	    setprop("/engines/engine/egt-fix-degf", egt_degf);

	} else {
	    # goes back to the ambient temperature in 1 min
	    interpolate("/instrumentation/egt/egt-degc", me.t_amb - 273.15, 60);
	    interpolate("/instrumentation/egt/egt-degf", (me.t_amb - 273.15) * 1.8 + 32, 60);
	}
    }
};

#
# Automatic Mixture Controller
# This class automatically adjust the mixture value depending on
# current exhaust gas temperature (egt) and given target egt.
# In acutal aircraft, mixture is adjusted using air density but
# it is easier to adjust using current egt and target egt
# 
AutoMixtureControl = {
    #
    # new(target_egt_degc)
    # target_egt_degc : egt where engine can get maximum power (Celsius)
    #
    new : func(target_egt_degc) {
	var obj = {
	    parents : [AutoMixtureControl],
	    target_egt : target_egt_degc
	};
	setprop("/controls/engines/engine/manual-mixture-control", 0);
	return obj;
    },

    #
    # automatic mixture adjuster
    #
    update : func {
	if (getprop("/controls/engines/engine/manual-mixture-control") != 1 and 
	    getprop("/engines/engine/running") == 1) {
	    var mixture = getprop("/controls/engines/engine/mixture");
	    var axis = getprop("/controls/engines/engine/mixture");
	    var egt = getprop("/instrumentation/egt/egt-degc");
	    var delta = me.target_egt - egt;
	    delta = me.target_egt - egt;
	    mixture -= (delta / 1000); 
	    if (mixture > 1.0) {
		mixture = 1.0;
	    } elsif (mixture < 0.0) {
		mixture = 0.0;
	    }
	    interpolate("/controls/engines/engine/mixture", mixture, 0.5);
	}
    }
}; 

Observers = {
    new : func {
        var obj = { parents : [Observers],
		    observers : [] };
	setlistener("/sim/signals/fdm-initialized", func { obj.registerTimer(); });
	return obj;
    },

    #
    # addObserver(observer)
    # add an observer object to the JapaneseWarbird object
    # each observer must have a method named "update"
    # 
    addObserver : func(observer) {
        append(me.observers, observer);
    },

    # 
    # update()
    # update each observer in turn
    # 
    update : func {
        foreach (observer; me.observers) {
	    observer.update();
        }
	me.registerTimer();
    },
    
    #
    # timer driven function
    #
    registerTimer : func {
        settimer(func { me.update() }, 0);
    }
};

var obs = Observers.new();
obs.addObserver(CylinderHeadTemperature.new(5.4, 2700));
obs.addObserver(ExhaustGasTemperature.new(5.4));

#egt.update();
#cht.update();
#print(sprintf("%.2f, %.2f\n", getprop("/engines/engine/cht-fix-degf"), getprop("/engines/engine/egt-fix-degf")));
