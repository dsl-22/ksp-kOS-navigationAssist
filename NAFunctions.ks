// Name: Navigation Assist Functions
// Author: dsl-22
// Version: 0.1
// kOS Version: 1.4.0.0
// KSP Version: 1.12.5
// Description:
//      Functions used by NA1 to assist in orbital navigation.
//
// Todo:
//     - Prevent improveConverge from stepping backwards
//     - Separate into files with boot and copying (decomposition)
//     - Allow user to submit maneuver to be executed
//     - Add status logging (time to stopping distance, time to maneuver, etc.)
//     - Look into asynchronicity for control flow separation
//     - Look into unit testing
//     - Look into integration testing (*)
//     - Improve doTransfer to stop testing after first collision maneuver is found
//
// Notes:
//     - The Ideal Rocket Equation was derived using a summation of
//       engine impulse values in initial vessel stage. KSP provides 
//       impulse values for each engine, and kOS can read them.
//     - Equations used to derive Ideal Rocket Equation:
//          - (dV = isp * g0 * ln(m0 / mf))
//          - (mf = m0 - (fuelFlow * t))
//          - (F = isp * g0 * fuelFlow)

// ------+Initial Ascent+------- //
function doLaunch { // Launches vessel using doAutoStage
    lock throttle to 1.
    doSafeStage().
}

function doAscent{ // Pitches vessel to follow Ideal Rocket Equation motion (see notes)
    lock targetPitch to 88.963 - 1.03287 * alt:radar^0.409511.
    set targetDirection to 90.
    lock steering to heading(targetDirection, targetPitch).
}

function doSafeStage { // Only stages when stage is ready
    wait until stage:ready.
    stage.
}

function doAutoStage { // Automatically stages and deploys until stage has thrust
    if not(defined oldThrust) {
        global oldThrust is ship:availablethrust. // Creates var to store available thrust
    }
    if ship:availablethrust < (oldThrust - 10) { // Deploys stages until engine is available
        until false {
            doSafeStage(). wait 1.
            if ship:availableThrust > 0 {
                break.
            }
        }
        global oldThrust is ship:availablethrust.
    }
}

function doShutdown { // Locks throttle to 0, locks steering to prograde marker
    lock throttle to 0.
    lock steering to prograde.
}
// ----------------------------- //

// ---------+Orbits+------------ //
function doCircularization { // Generatively calculates orbital trajectory based on scoring system, plots non-eccentric 100km orbit
    local circ is list(0).
    set circ to improveConverge(circ, eccentricityScore@). // Calculates orbit based on second parameter (eccentricityScore@ leads to 100km circular with ~zero eccentricity)
    executeManeuver(list(time:seconds + eta:apoapsis, 0, 0, circ[0])). // Orients vessel to target and throttles 
}

function eccentricityScore { // Scores orbit based on eccentricity value
    parameter data.
    local mnv is node(time:seconds + eta:apoapsis, 0, 0, data[0]). // Maneuver node
    addManeuverToFlightPlan(mnv). // Adds maneuver node to flight plan
    local result is mnv:orbit:eccentricity. // Eccentricity of orbit
    removeManeuverFromFlightPlan(mnv). // Removes maneuver from flight plan
    return result.
}

function improveConverge { // Converges orbit to fit scoreFunction parameter
    parameter data, scoreFunction.
    for stepSize in list(100, 10, 1) { // Decreases search area over time using improveCandidate
        until false {
            local oldScore is scoreFunction(data). // Method used by improveCandidate to improve trajectory, stores last used value
            set data to improveCandidate(data, stepSize, scoreFunction). // Iterates manuever positions based on inputted parameter (improveCandidate)
            if oldScore <= scoreFunction(data) { // Breaks loop if score fits the desired score function
                break.
            }
        }
    }
    return data.
}

function improveCandidate { // Edits trajectory based on scoring system and selected scoring system, scores improve as they approach 0
    parameter data, stepSize, scoreFunction.
    local scoreToBeat is scoreFunction(data).
    local bestCandidate is data.
    local candidates is list().
    local index is 0.
    until index >= data:length { // Iterates manuever positions with increments and decrements
        local incCandidate is data:copy().
        local decCandidate is data:copy().
        set incCandidate[index] to incCandidate[index] + stepSize.
        set decCandidate[index] to decCandidate[index] - stepSize.
        candidates:add(incCandidate).
        candidates:add(decCandidate).
        set index to index + 1.
    }
    for candidate in candidates {
        local candidateScore is scoreFunction(candidate). // If score is low enough, sets candidate score to new value
        if candidateScore < scoreToBeat {
            set scoreToBeat to candidateScore.
            set bestCandidate to candidate.
        }
    }
    return bestCandidate.
}

function executeManeuver { // Interacts with navigation system by placing and removing maneuver nodes
    parameter mList.
    local mnv is node(mList[0], mList[1], mList[2], mList[3]). // Stores inputted maneuver
    addManeuverToFlightPlan(mnv). // Adds manuever to map
    local startTime is calculateStartTime(mnv). // Calculates when engines should be fired
    wait until time:seconds > startTime - 10.
    lockSteeringAtManeuverTarget(mnv). // Locks steering to maneuver target
    wait until time:seconds > startTime.
    lock throttle to 1.
    until isManeuverComplete(mnv) { // Checks if maneuver is complete, auto-stages until delta V requirement is met
        doAutoStage().
    }
    lock throttle to 0. //
    unlock steering.
    removeManeuverFromFlightPlan(mnv). // Removes maneuver node from map when complete
}

function addManeuverToFlightPlan { // Adds manuever to map
    parameter mnv.
    add mnv.
}

function calculateStartTime { // Calculates when engines should be fired
    parameter mnv.
    return time:seconds + mnv:eta - maneuverBurnTime(mnv) / 2.
}

function maneuverBurnTime { // Calculates actual burn time needed to execute maneuver
    parameter mnv.
    local dV is mnv:deltaV:mag.
    local g0 is 9.80665.
    local isp is 0.

    list engines in myEngines. // Calculations based on derived Ideal Rocket Equation (see notes)
    for en in myEngines {
        if en:ignition and not en:flameout {
            set isp to isp + (en:isp * (en:availableThrust / ship:availableThrust)).
        }
    }

    local mf is ship:mass / constant():e^(dV / (isp * g0)). 
    local fuelFlow is ship:availableThrust / (isp * g0).
    local t is (ship:mass - mf) / fuelFlow.

    return t.
}

function lockSteeringAtManeuverTarget { // Locks steering to maneuver target
    parameter mnv.
    lock steering to mnv:burnvector.
}

function isManeuverComplete { // Checks if maneuver requirements have been met
    parameter mnv.
    if not(defined originalVector) or originalVector = -1 {
        global originalVector is mnv:burnvector.
    }
    if vang(originalVector, mnv:burnvector) > 90 {
        
        global originalVector is -1. // Deletes originalVector (i.e. sets vector to unallowed value)
        return true.
    }
    return false.
}

function removeManeuverFromFlightPlan { // Removes maneuver node from map when complete
    parameter mnv.
    remove mnv.
}

function doTransfer { // Calculates closest collision with selected body using a ternary search algorithm
    local startSearchTime is ternarySearch(
        angleToMun@, // Selected body
        time:seconds + 30,
        time:seconds + 30 + orbit:period,
        1
    ).
    local transferVal is list(startSearchTime, 0, 0, 0).
    set transferVal to improveConverge(transferVal, protectFromPast(munTransferScore@)). // Converges collision orbit with improveConverge, redundant calculations avoided with protectFromPast
    executeManeuver(transferVal). // Executes calculated maneuver
    wait until body = mun.
    wait 1.
}

function protectFromPast { // Prevents redundant calculations from selected function
    parameter originalFunction.
    local replacementFunction is {
        parameter data.
        if data[0] < time:seconds + 15 { // Returns arbitrarily large number if redundancy
            return 2^64.
        } else {
            return originalFunction(data).
        }
    }.
    return replacementFunction@.
}

function munTransferScore { // Scores trajectory based on whether or not it collides with the moon
    parameter data.
    local mnv is node(data[0], data[1], data[2], data[3]).
    addManeuverToFlightPlan(mnv).
    
    local result is 0.
    if mnv:orbit:hasnextpatch { // If orbit has a future trajectory, return scalar value for scoring
        set result to mnv:orbit:nextpatch:periapsis.
    } else {
        set result to distanceToMunAtApoapsis(mnv). // Returns distance from current orbit apoapsis to Mun
    }
    removeManeuverFromFlightPlan(mnv). // Removes maneuver from map
    return result.
}

function distanceToMunAtApoapsis { // Calculates distance from current orbit apoapsis and Mun using ternary search
    parameter mnv.
    local apoapsisTime is ternarySearch(
        altitudeAt@,
        time:seconds + mnv:eta,
        time:seconds + mnv:eta + (mnv:orbit:period / 2),
        1
    ).
    return (positionAt(ship, apoapsisTime) - positionAt(mun, apoapsisTime)):mag.
}

function altitudeAt { // Returns vessel altitude above Kerbin
    parameter t.
    return kerbin:altitudeOf(positionAt(ship, t)).
}

function angleToMun { // Returns vector angle between current orbit and Mun
    parameter t.
    return vectorAngle(
        kerbin:position - positionAt(ship, t),
        kerbin:position - positionAt(mun, t)
    ).
}

function ternarySearch { // Ternary Search Algorithm
    parameter f, left, right, absolutePrecision.
    until false {
        if abs(right - left) < absolutePrecision {
            return (left + right) / 2.
        }
        local leftThird is left + (right - left) / 3.
        local rightThird is right - (right - left) / 3.
        if f(leftThird) < f(rightThird) {
            set left to leftThird.
        } else {
            set right to rightThird.
        }
    }
}
// ----------------------------- //

// ---------+Landing+----------- //
function doHoverslam { // Orients vessel for hoverslam maneuver landing
    lock steering to srfRetrograde. // Sets steering to retrograde of the approaching body
    lock pct to stoppingDistance() / distanceToGround(). // Calculates throttle needed to hoverslam
    wait until pct > 1.
    lock throttle to pct.
    when distanceToGround() < 500 then { gear on.}
    wait until ship:verticalSpeed > 0.
    lock throttle to 0.
    lock steering to groundSlope(). // Steers vessel to surface normal of landing zone
    wait 30.
    unlock steering.
}

function stoppingDistance { // Calculates distance needed for hoverslam
    // Stopping distance formula
    // stoppingDistance = v^2 / 2a
    local grav is constant():g * (body:mass / body:radius^2).
    local maxDeceleration is (ship:availableThrust / ship:mass) - grav.
    return ship:verticalSpeed^2 / (2 * maxDeceleration).
}

function distanceToGround { // Calculates distance between vessel and landing surface
    return altitude - body:geopositionOf(ship:position):terrainHeight - 4.7.
}

function groundSlope { // Gets surface normal of the ground 
    local east is vectorCrossProduct(north:vector, up:vector).
    local center is ship:position.

    local a is body:geopositionOf(center + 5 * north:vector).
    local b is body:geopositionOf(center - 3 * north:vector + 4 * east).
    local c is body:geopositionOf(center - 3 * north:vector - 4 * east).

    local a_vec is a:altitudePosition(a:terrainHeight).
    local b_vec is b:altitudePosition(b:terrainHeight).
    local c_vec is c:altitudePosition(c:terrainHeight).

    return vectorCrossProduct(c_vec - a_vec, b_vec - a_vec):normalized. // Returns cross product (surface normal)
}
// ----------------------------- //
