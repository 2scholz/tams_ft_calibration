#!/bin/bash
# example of sending calibration offsets and gains via the command line...
# Note the curly braces for arguments ${10} and up.
#
# rosservice call /sunrise_m3207/set_calibration "{ offsets:[ 1,2,3,4,5,6 ], gains:[ 1,1,1,1,2,3 ] }"
#rosservice call set_calibration "{ offsets:[ $1, $2, $3, $4, $5, $6 ], gains:[ $7, $8, $9, ${10}, ${11}, ${12} ] }"
rosservice call set_calibration "{ force_offsets:[ $1, $2, $3], torque_offsets:[$4, $5, $6 ], force_gains:[ $7, $8, $9], torque_gains:[${10}, ${11}, ${12}] }"
#
