##
## SimStatus.tcl 
## CarMaker 6.0 ScriptControl Example - IPG Automotive GmbH (www.ipg-automotive.com)
## 
## This example illustrates the use of the SimStatus command.
##
## In Verbose mode, you can see the output in the CarMaker log file.
##
## The Sleep() function are used to slow down execution and make it 
## more viewable. They are normally not necessary. 
##
## $Id: SimStatus.tcl,v 1.5 2017/04/11 09:12:49 vw Exp $

LoadTestRun "Examples/BasicFunctions/Road/ArtificialRoads/SegmentBasedClosedTrack"

set status [SimStatus]
Log "Simulation status: $status\n"

Log "Start simulation"
StartSim

Log "Wait until the simulation runs"
WaitForStatus running
set status [SimStatus]
Log "Simulation status: $status\n"

Log "Simulate 10 seconds"
Sleep 10000

Log "Stop the simulation"
StopSim

Log "Wait until the simulation stops"
WaitForStatus idle
set status [SimStatus]
Log "Simulation status: $status\n"

