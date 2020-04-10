// Launch script v0.3.0 by JW 2019

// Autostage
DECLARE FUNCTION init_AutoStage {
	DECLARE FUNCTION autoStage {
		IF disable_AutoStage {
			return.
		}
		WAIT UNTIL STAGE:READY.
		STAGE.

		GLOBAL stage_engines IS listStageEngines().
		GLOBAL stage_ISP IS calc_stageISP(stage_engines).

		autoStage_Triggers().
	}

	DECLARE FUNCTION autoStage_Triggers {
		// if there exists no potential for thrust in current stage, proceed to stage the vessel.
		IF SHIP:MAXTHRUST = 0 {
			autoStage().
		}
		// create triggers to track various fuel types left in current stage
		IF STAGE:RESOURCES:CONTAINS(LiquidFuel) {
			WHEN STAGE:LIQUIDFUEL < 0.1 OR disable_AutoStage THEN {
				autoStage().
			}
		}
		IF STAGE:RESOURCES:CONTAINS(LqdHydrogen) {
			WHEN STAGE:LQDHYDROGEN OR disable_AutoStage < 0.1 THEN {
				autoStage().
			}
		}
		IF STAGE:RESOURCES:CONTAINS(SolidFuel) {
			WHEN STAGE:SOLIDFUEL OR disable_AutoStage < 0.1 THEN {
				autoStage().
			}
		}
	}

	SET disable_AutoStage TO FALSE.
	autoStage_Triggers().
}

DECLARE FUNCTION listStageEngines {
	PARAMETER stageNumber IS STAGE:NUMBER.
	LIST ENGINES IN vesselEngines.
	SET stageEngines TO list().
	// compile list of engines active during specified stage
	FOR _engine IN vesselEngines {
		IF _engine:STAGE = stageNumber AND NOT stage_engines:CONTAINS(_engine) {
			stageEngines:ADD(_engine).
		}
	}
	RETURN stageEngines.
}

DECLARE FUNCTION calc_stageISP {
	PARAMETER _engines.

	LOCAL _isp IS 0.	
	FOR _eng IN _engines {
		SET _isp TO _isp + (_eng:AVAILABLETHRUSTAT(SHIP:Q) / _eng:ISPAT(SHIP:Q)).
	}
	RETURN _isp.
}

DECLARE FUNCTION calc_TWR {
	LOCAL _thrust IS SHIP:AVAILABLETHRUSTAT(SHIP:Q).
	LOCAL _weight IS SHIP:MASS.

	RETURN _thrust / _weight.
}

DECLARE FUNCTION calc_estBurnLength {
	// credit and thanks for this formula go to u/gisikw on reddit.com/r/Kos/
	PARAMETER _dV.
	PARAMETER _vessel IS SHIP.

	LOCAL initMass_kg IS _vessel:MASS * 1000.
	LOCAL vesselThrust_kg IS _vessel:AVAILABLETHRUSTAT(_vessel:Q) * 1000.
	
	RETURN CONSTANT:g0 * initMass_kg * stage_ISP * (1 - CONSTANT:e^(-_dv / (CONSTANT:g0 * stage_ISP))) / vesselThrust_kg. 
}

DECLARE FUNCTION gravityTurnAngle {
	PARAMETER levelOffAlt IS 42000.
	PARAMETER AoA_mod IS 0.5.
	RETURN (1 - (ALTITUDE / levelOffAlt) ^ AoA_mod).
}

DECLARE FUNCTION calc_visViva {
	PARAMETER altitude IS SHIP:ALTITUDE.
	PARAMETER orbitingBody IS SHIP:ORBIT:BODY.

	LOCAL radius IS calc_distanceFromBodyCenter(altitude).

	RETURN SQRT(orbitingBody:MU * ((2 / radius) - (1 / orbitingBody:ORBIT:SEMIMAJORAXIS))).
}

DECLARE FUNCTION calc_distanceFromBodyCenter {
	PARAMETER heightAboveSurface IS SHIP:ALTITUDE.
	PARAMETER planetaryRadius IS SHIP:ORBIT:BODY:RADIUS.
	RETURN planetaryRadius + heightabovesurface.
}

DECLARE FUNCTION verticalSpeedPID {
	SET proportionalVal TO 0.05.
	SET integralVal TO 0.005.
	SET derivativeVal TO 0.005.
	SET vspeedEvaluator TO PIDLOOP(proportionalVal, integralVal, derivativeVal).
	SET vspeedEvaluator:SETPOINT TO 0.
	LOCAL _limit IS 10.
	SET vspeedEvaluator:MAXOUTPUT TO _limit.
	SET vspeedEvaluator:MINOUTPUT TO -_limit.

	RETURN vspeedEvaluator:UPDATE(TIME:SECONDS, VERTICALSPEED).
}


// flightplans are a data structure (sort of) to enable easy, modular mission planning
// A flightplan will eventually be represented as a list of delegate functions (procedures) that are executed sequentially

// when a flightplan-based script runs, the flightplan initializes and locks the CPU Vessel's
// throttle and steering to variables "cooked_throttle" and "cooked_steering". The flightplan's Procedure functions
// modify these two values to produce the autopilot/assisted steering system.



DECLARE FUNCTION Flightplan_LaunchToOrbit {
	PARAMETER goal_apoapsis IS 85000.
	PARAMETER goal_periapsis IS goal_apoapsis.
	PARAMETER launchDirection IS 90.

	DECLARE FUNCTION Procedure_Head {

		SET PROCEDURE_PTR TO Procedure_Init@.
		LOCAL PROCEDURE_COMPLETE IS FALSE.

		UNTIL PROCEDURE_COMPLETE {
			PROCEDURE_PTR().
		}

		DECLARE FUNCTION Procedure_Init {
			SET COOKED_THROTTLE TO 0.
			LOCK THROTTLE TO COOKED_THROTTLE.
			SET COOKED_STEERING TO HEADING(launchDirection,90).
			LOCK STEERING TO COOKED_STEERING.

			SET PROCEDURE_PTR TO Procedure_Liftoff@.
		}

		DECLARE FUNCTION Procedure_Liftoff {
			SET COOKED_THROTTLE TO 1.
			init_AutoStage().

			SET PROCEDURE_PTR TO Procedure_GravityTurn@.
		}

		DECLARE FUNCTION Procedure_GravityTurn {
			SET COOKED_STEERING TO HEADING(launchDirection, gravityTurnAngle()).

			// APOAPSIS ABOVE ATMOSPHERE
			// TBD - calculate and compensate for drag bringing down apo whilst still in atmo

			// APOAPSIS AT GOAL APOAPSIS
			WHEN APOAPSIS >= goal_apoapsis THEN {
				SET COOKED_THROTTLE TO 0.
				SET PROCEDURE_PTR TO Procedure_ChangeOrbitTo@.
			}
			
		}

		DECLARE FUNCTION Procedure_ChangeOrbitTo {

			DECLARE FUNCTION throttleForBurn {
				SET COOKED_THROTTLE TO 1.
				WHEN est_burn_time < 0.2 THEN {
					SET COOKED_THROTTLE TO 0.
					SET PROCEDURE_PTR TO Procedure_End@.
				}
			}

			SET COOKED_STEERING TO PROGRADE.
			IF ETA:APOAPSIS < ETA:PERIAPSIS { SET nextApsis TO APOAPSIS. } ELSE { SET nextApsis TO PERIAPSIS. }
			LOCAL goal_velocity IS calc_visViva().
			LOCAL est_burn_dV IS goal_velocity - VELOCITYAT(SHIP, TIME:SECONDS + ETA:nextApsis).
			LOCK est_burn_time TO calc_estBurnLength(est_burn_dv).

			WHEN ETA:nextApsis - est_burn_time/2 < 5 THEN {
				SET COOKED_STEERING TO HEADING(launchDirection, -verticalSpeedPID()).
				throttleForBurn().
			}
		}

		DECLARE FUNCTION Procedure_End {
			UNLOCK STEERING.
			UNLOCK THROTTLE.
			PRINT "COMPLETE?".
			SET PROCEDURE_COMPLETE TO TRUE.
		}
		
		
	}

	Procedure_Head().

}

// DECLARE FUNCTION calc_HohmannTransfer_dV {
// 	// LOCAL v_inject.
// 	// LOCAL v_insert.
// 	LOCAL orbitingBody IS KERBIN.
// 	{
// 		SQRT(SQRT( (2 * KERBOL)))
// 	}
// }

DECLARE FUNCTION PreciseOrbitalSpeed {
	PARAMETER orbitingBody.
	PARAMETER givenApoapsis.
	PARAMETER givenPeriapsis.
	RETURN SQRT(orbitingBody:MU * ( (2 / (givenApoapsis + orbitingBody:RADIUS) ) - 1 / (givenApoapsis + givenPeriapsis + 2 * orbitingBody:Radius / 2) + orbitingBody:RADIUS)).
}


DECLARE FUNCTION vesselPitch {
	PARAMETER _vessel IS SHIP.
	RETURN 90 - VANG(_vessel:UP:VECTOR, _vessel:FACING:FOREVECTOR).
}

DECLARE FUNCTION surfaceVelocityVector {
	PARAMETER _vessel IS SHIP.
	RETURN 90 - VANG(_vessel:UP:VECTOR, _vessel:SRFPROGRADE).
}

DECLARE FUNCTION lerpAngle {
	PARAMETER initAngle.
	PARAMETER tgtAngle.
	PARAMETER initTime.
	PARAMETER time_limit IS 2.

	return ((TIME:SECONDS - initTime)/(initTime + time_limit)) * tgtAngle + initAngle.
}

DECLARE FUNCTION calc_accDueToGravity {
	PARAMETER body_1.
	PARAMETER body_2.

	RETURN CONSTANT:G * ((body_1:MASS * body_2:MASS) / calc_distanceFromBodyCenter(body_1:ALTITUDE)).
}

//SET TRACKED_DATA TO LEXICON().
//TRACKED_DATA:ADD("VERTICAL VELOCITY", SHIP:VERTICALSPEED).
//TRACKED_DATA:ADD("")

//CLEARSCREEN.
//PRINT AT
