##
## RunScripts.tcl 
## CarMaker 6.0 ScriptControl Example - IPG Automotive GmbH (www.ipg-automotive.com)
##
## $Id: RunScripts.tcl,v 1.4 2017/04/11 09:12:49 vw Exp $


# A script can specified with a path relative to current script
RunScript StartStop.tcl
RunScript WaitForCondition.tcl
RunScript Math.tcl

# Absolute paths can also be used, but should be avoided.
# Example:
# RunScript /home/hil/Scripts/StartStop.tcl


