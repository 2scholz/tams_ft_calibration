#!/bin/bash
# example of sending calibration offsets and gains via the command line...
# Note the curly braces for arguments ${10} and up.
#
# rosservice call /sunrise_m3207/set_calibration "{ offsets:[ 1,2,3,4,5,6 ], gains:[ 1,1,1,1,2,3 ] }"
rosservice call set_calibration "{ offsets:[ $1, $2, $3, $4, $5, $6 ], gains:[ $7, $8, $9, ${10}, ${11}, ${12} ] }"
#
