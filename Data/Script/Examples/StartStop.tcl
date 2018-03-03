##
## StartStop.tcl 
## CarMaker 6.0 ScriptControl Example - IPG Automotive GmbH (www.ipg-automotive.com)
## 
## Load, start and stop test runs
## 
## $Id: StartStop.tcl,v 1.5 2017/04/11 09:12:49 vw Exp $

Log "* Run 15 seconds of Hockenheim"
LoadTestRun "Examples/BasicFunctions/Road/ArtificialRoads/SegmentBasedClosedTrack"
StartSim
WaitForStatus running
WaitForTime 15

StopSim
WaitForStatus idle 10000

Log "* Run 15 seconds of LaneChangeISO"
LoadTestRun "Examples/VehicleDynamics/Handling/LaneChange_ISO"
StartSim
WaitForStatus running
WaitForTime 15

StopSim

