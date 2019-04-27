// Launch script v0.3.0 by James Wienecke 2019
MAIN().

// Autostage
DECLARE FUNCTION init_AutoStage {
	PARAMETER persistant IS TRUE.

	DECLARE FUNCTION autoStage {
		WAIT UNTIL STAGE:READY.
		STAGE.
		IF persistant { init_autostage(). }
	}

	// if there exists no potential for thrust in current stage, proceed to stage the vessel.
	IF SHIP:MAXTHRUST = 0 {
		autoStage().
	}

	// create triggers to track various fuel types left in current stage
	IF STAGE:RESOURCES:CONTAINS(LiquidFuel) {
		WHEN STAGE:LIQUIDFUEL < 0.1 THEN {
			autoStage().
		}
	}
	IF STAGE:RESOURCES:CONTAINS(LqdHydrogen) {
		WHEN STAGE:LQDHYDROGEN < 0.1 THEN {
			autoStage().
		}
	}
	IF STAGE:RESOURCES:CONTAINS(SolidFuel) {
		WHEN STAGE:SOLIDFUEL < 0.1 THEN {
			autoStage().
		}
	}
}

DECLARE FUNCTION listStageEngines {
	PARAMETER knownEngines IS LIST().
	PARAMETER stageNumber IS STAGE:NUMBER.
	LIST ENGINES IN vesselEngines.
	SET stageEngines TO list().
	// compile list of engines active during specified stage
	FOR _engine IN vesselEngines {
		IF _engine:STAGE = stageNumber AND NOT knownEngines:CONTAINS(_engine) {
			stageEngines:ADD(_engine).
			}
		}
	}
	RETURN stageEngines.
}

DECLARE FUNCTION gravityTurnAngle {
	PARAMETER initialPitch.
	SET groundVector TO UP + SHIP:VELOCITY:SURFACE.
	WAIT UNTIL groundVector:PITCH < initialPitch.
	RETURN HEADING(launchCompassDirection, groundVector:PITCH).
}

DECLARE FUNCTION lerpAngle {
	PARAMETER initAngle.
	PARAMETER tgtAngle.
	PARAMETER initTime.
	PARAMETER time_limit IS 2.

	return ((TIME:SECONDS - initTime)/(initTime + time_limit)) * tgtAngle + initAngle.
}















// dynamically set the throttle to a value that locks to maintain a specific TWR
DECLARE FUNCTION CalcThrustProfile {
	PARAMETER idealTWR.

	LOCAL LOCK g TO KERBIN:MU / KERBIN:DISTANCE^2.
	LOCAL LOCK maxTWR TO SHIP:MAXTHRUSTAT(SHIP:DYNAMICPRESSURE) / (SHIP:MASS * g).
	IF idealTWR > maxTWR { return 1. }
	LOCAL LOCK idealThrust TO idealTWR * SHIP:MASS * g.
	LOCAL LOCK cookedThrottle TO idealThrust / SHIP:MAXTHRUSTAT(SHIP:DYNAMICPRESSURE).

	RETURN cookedThrottle.
}

DECLARE FUNCTION GravityTurn {
	
}

// deprecated pls delete
// continuously adjust steering towards (relatively arbitrary) ascent angle (currently an algabraeic function of the vessel's altitude)
DECLARE FUNCTION DeprecatedGravityTurn {
	SET ascentAngle TO 5.658E-9 * ALT:RADAR^2 - 0.00195543 * ALT:RADAR + 107.067.
	IF ascentAngle > 90 { SET ascentAngle TO 90. }
}

// check vessel engines to catch flameouts and automatically stage
//DECLARE FUNCTION AutoStage {
//	LIST ENGINES IN engineList.
//	FOR engine IN engineList {
//		IF engine:FLAMEOUT {
//			DoStage().
//			LIST ENGINES IN engineList.
//			BREAK.
//		}
//	}
//}

// adjust pitch above horizon by pid-modulated vertical speed factor to maintain altitude while circularizing
DECLARE FUNCTION ApsisPreservingCircularization {
	SET proportionalVal TO 0.05.
	SET integralVal TO 0.005.
	SET derivativeVal TO 0.005.
	SET vspeedEvaluator TO PIDLOOP(proportionalVal, integralVal, derivativeVal).
	SET vspeedEvaluator:SETPOINT TO 0.

	RETURN vspeedEvaluator:UPDATE(TIME:SECONDS, VERTICALSPEED).
}

DECLARE FUNCTION PreciseOrbitalSpeed {
	PARAMETER orbitingBody.
	PARAMETER givenApoapsis.
	PARAMETER givenPeriapsis.
	RETURN SQRT(orbitingBody:MU * (2 / (givenApoapsis + orbitingBody:RADIUS) - 1 / (givenApoapsis + givenPeriapsis / 2) + orbitingBody:RADIUS)).
}

DECLARE FUNCTION CircularizationNode {
	SET initialVelocity TO PreciseOrbitalSpeed(KERBIN, ALT:APOAPSIS, ALT:PERIAPSIS).
	SET targetVelovity TO PreciseOrbitalSpeed(KERBIN, targetApoapsis, targetPeriapsis).
	RETURN NODE(ETA:APOAPSIS, 0, 0, targetVelocity - initialVelocity).
}

DECLARE FUNCTION ExecuteMNode {
	PARAMETER mNode.
	LOCK STEERING TO mNode:BURNVECTOR.
	SET burnDuration TO mNode:DELTAV / (SHIP:MAXTHRUST / SHIP:MASS).
	
}
	
DECLARE FUNCTION ExecLaunchManeuver {
	SET ascentAngle TO 90.
	LOCK THROTTLE TO CalcThrustProfile(launchTWR).
	LOCK STEERING TO HEADING(launchCompassDirection, ascentAngle).
	STAGE.
	PRINT "Launch! Maneuver parameters: ".
	PRINT "Target Apoapsis: " + targetApoapsis.
	PRINT "Target Periapsis: " + targetPeriapsis.
	PRINT "Launch direction: " + launchCompassDirection.
	PRINT "Target TWR: " + launchTWR.
	
	PRINT "Beginning gravity turn...".
	UNTIL ALT:APOAPSIS > 70000 {
		IF VERTICALSPEED < 150 { BREAK. }
		GravityTurn().
		WAIT 0.1.
	}
	SET ascentAngle TO 5.
	PRINT "Transatmospheric trajectory established. Continuing burn to bring apoapsis to target suborbital trajectory...".
	UNTIL ALTITUDE > 70000 AND ALT:APOAPSIS >= targetApoapsis {
	}
	LOCK THROTTLE TO 0.
	PRINT "Target apoapsis aquired. Initializing orbit circularization maneuver...".
	SET directivePointer TO ExecCircularization@.
	RETURN true.
}

DECLARE FUNCTION ExecCircularization {
	LOCK adjustedPitch TO PROGRADE.
	LOCK STEERING TO adjustedPitch.
	WAIT UNTIL ETA:APOAPSIS < 1.
	PRINT "Circularization burn in progress...".
	LOCK THROTTLE TO CalcThrustProfile(3).

	UNTIL ALT:PERIAPSIS > targetPeriapsis {
		SET adjustedPitch TO HEADING(launchCompassDirection, ApsisPreservingCircularization()).
	}
	PRINT "Orbit established.".
	SET directivePointer TO Cleanup@.
	RETURN false.
}

DECLARE FUNCTION Cleanup {
	RETURN false.
}

DECLARE FUNCTION Main {
	SET scriptReady TO false.
	PRINT "Building gui...".
	runpath("liftoffgui.ks").
	SET directivePointer TO ExecLaunchManeuver@.
	init_autoStage().
	WAIT UNTIL scriptReady = true.
	UNTIL scriptReady = false {
		SET scriptReady TO directivePointer:CALL.
	}
}