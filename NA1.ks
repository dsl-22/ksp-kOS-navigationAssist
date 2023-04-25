// Name: Navigation Assist 
// Author: dsl-22
// Version: 0.1
// kOS Version: 1.4.0.0
// KSP Version: 1.12.5
// Description:
//      Subroutines assisting and automating orbital navigation
//      (Kerbin Atmostphere Egress, Dynamic Orbit Circularization, 
//      Dynamic Mun Transfer, Dynamic Hoverslam, Surface Normalization).
//
// Notes:
//     - The Ideal Rocket Equation was derived using a summation of
//       engine impulse values in initial vessel stage. KSP provides 
//       impulse values for each engine, and kOS can read them.
//     - Equations used to derive Ideal Rocket Equation:
//          - (dV = isp * g0 * ln(m0 / mf))
//          - (mf = m0 - (fuelFlow * t))
//          - (F = isp * g0 * fuelFlow)

run NAFunctions.ks. // Includes function library

main().

function main {
    doLaunch(). // Launches vessel using doAutoStage
    doAscent(). // Pitches vessel to follow Ideal Rocket Equation motion (see notes)
    until apoapsis > 100000 {
        doAutoStage(). // Continues acceleration from doAscent, automatically stages and deploys to maintain ascent until apoapsis is 100km
    }
    doShutdown(). // Locks throttle to 0, locks steering to prograde marker
    doCircularization(). // Generatively calculates orbital trajectory based on scoring system, plots non-eccentric 100km orbit
    doTransfer(). // Generatively calculates and plots munar collision trajectory using ternery search and scoring system
    doHoverslam(). // Calculates and executes hoverslam burn, vessel normalizes to terrain, steering unlocks
    wait until false.
}
