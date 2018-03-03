##
## TrailerSwingingDVA.tcl
## CarMaker 6.0 ScriptControl Example - IPG Automotive GmbH (www.ipg-automotive.com)
##
## This example shows how to set the value of a variable
## using Direct Variable Access.
## The functions introduced are:
##
## DVAWrite <Name> <Value> <Count> <Mode>
##     <Name>  Quantity name
##     <Value> New Quantity Value
##     <Count> Number of Cycles ... default 1
##     <Mode>  (0 = value, 1 = offset, 2 = factor) ... default 0
##     DVAWrite sets the value of the specified quantity using DVA
##
## DVAReleaseQuants
##     Releases the quantities from DVA control. The quantity is not
##     reset, and unless it is modified by some internal function,
##     it will keep the value it was given.
##

# Declare variables and get access to some predefined globals
QuantSubscribe DM.Steer.Ang

set cycles 2

# Print Test Header
Log screen "\n         *** DVA Example Start ***\n" 

# load the test run
LoadTestRun "Examples/BasicFunctions/TestAutomation/ScriptControl/Straight_TrailerSwingingDVA"

# start the simulation
StartSim

# Wait until the simulation starts 
WaitForStatus running

set t 10
WaitForCondition {$Qu(Time)>=$t || [SimStatus]<0}

for {set i 0} {[SimStatus] >= 0} {incr i} {
    DVAWrite DM.Steer.Ang $i $cycles
    Log screen "Wheel turned to Angle $i rad.\n"

    incr t 10
    WaitForCondition {$Qu(Time)>=$t || [SimStatus]<0}
}

Log screen "
       +++++++++++++++++++++++++++++++++++++++++
       Car went off the road at: 
               
       Time                 =  $Qu(Time) seconds
       Steering Wheel Angle =  [expr $i-1] rad
       Set for Cycles       =  $cycles  
       +++++++++++++++++++++++++++++++++++++++++"

DVAReleaseQuants

Log screen "\n         *** DVA Example End ***\n\n" 

