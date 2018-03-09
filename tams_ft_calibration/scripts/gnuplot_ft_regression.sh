# Gnuplot script file for robotiq ft 150 sensor f/t plotting
# and calibration via linear regression. This expects two
# input files from phase1 (candle position, joint 6 motion,
# for calibration of Fx, Fy, Tx, Ty, Tz) and phase 2 
# (candle position, joint 5 motion, Fz calibration).
#
# Default filenames are:
# /tmp/sunrise-log-phase1.txt
# /tmp/sunrise-log-phase2.txt
#
file_phase1 = "calib_1"
file_phase2 = "calib_2"

#
# The payload weight is expected to be mounted off-center
# with its center of gravity along the +y sensor direction.
# In the input files, column 1 is the timestamp, columns
# 2..7 are the raw data from the six amplifier channels,
# corresponding to Fx, Fy, Fz, Tx, Ty, Tz.
# Column 8 is unused (reference force from the Maul-Logic
# digital scales).
# Columns 9..14 are position (x,y,z) and rotation (R,P,Y)
# of the sensor frame.
# Columns 15..17 are the projection of the sensor frame
# NOA axes onto the gravity axes (0,0,-1).
#
# Note: for this experiment, total tool load was:
# ca. 650 grams (FT sensor itself)
# ca. 2300 grams (drill chunk and adapter plates)
# load total = (650+2300)/1000 kg * 9.81 m/s^2 = 28.94 N
# Tx/Ty torque is dominated by the metal plate,
# which has 816 grams at dz=0.084m from the sensor plane,
# so torque = 0.816 kg * 9.81 m/s^2* 0.087 m = 0.71 Nm
# drill-chunk: 290 grams at dz=0.035 from the sensor plane,
# so torque = 0.29 * 9.81 * 0.035 = 0.099 Nm
# total torque = 0.71 Nm + 0.099 Nm = 0.81 Nm
# For Tz, sensor and drill-chunk are rotation symmetric,
# and the effective torque is from the assymetry of the
# metal disk. 
# Note: http://www.uni-magdeburg.de/ifme/l-festigkeit/pdf/massentraegheitsmomente.pdf#
# Note: http://me-lrt.de/a43-tragheitsmoment-stab-zylinder
# Note: https://de.wikipedia.org/wiki/Steinerscher_Satz
# F = m * a = m * x_dot_dot
# T = J * alpha = J * phi_dot_dot
# Satz von Steiner: Jp = Js + m*d^2
# m=0.816 gram, d=2cm, also torque_z = 0.816 kg * 0.026 m * 9.81 m/s^2 = 0.208 Nm
#
# 2016.09.27: invert Tx and Tz signs to get correct orientation
#


#
# The payload weight is expected to be mounted off-center
# with its center of gravity along the +y sensor direction.
# In the input files, column 1 is the timestamp, columns
# 2..7 are the raw data from the six amplifier channels,
# corresponding to Fx, Fy, Fz, Tx, Ty, Tz.
# Column 8 is unused (reference force from the Maul-Logic
# digital scales).
# Columns 9..14 are position (x,y,z) and rotation (R,P,Y)
# of the Sunrise sensor frame.
# Columns 15..17 are the projection of the sensor frame
# NOA axes onto the gravity axes (0,0,-1).
#
# Note: for this experiment, total tool load was:
# ca. 650 grams (FT sensor itself)
# ca. 300 grams (bluetooth adapter plate)
# ca. 2300 grams (Robotiq hand)
# ca. 550 grams (small TAMS hammer)
# load total = (650 + 300 + 2300 + 550)/1000 kg * 9.81 m/s^2 = 37.3N
# Tx/Ty torque is dominated by the Robotiq hand,
# which has 2300 grams at dz=0.11m from the sensor plane,
# so torque = 2.3 kg * 9.81 m/s^2* 0.11 m = 2.48 Nm
# bluetooth adapter: ~300 grams at dz=0.035 from the sensor plane,
# so torque = 0.30 * 9.81 * 0.035 = 0.1 Nm
# hammer: z-distance betweed head and F/T frame: 175mm
# so torque = 0.55 * 9.81 * 0.175 = 0.94 Nm
# total torque = 2.48 Nm + 0.1 Nm + 0.94 Nm = 3.52 Nm
#
# For Tz, sensor and drill-chunk are rotation symmetric,
# and the effective torque is from the assymetry of the
# metal disk. 
# Note: http://www.uni-magdeburg.de/ifme/l-festigkeit/pdf/massentraegheitsmomente.pdf#
# Note: http://me-lrt.de/a43-tragheitsmoment-stab-zylinder
# Note: https://de.wikipedia.org/wiki/Steinerscher_Satz
# F = m * a = m * x_dot_dot
# T = J * alpha = J * phi_dot_dot
# Satz von Steiner: Jp = Js + m*d^2
# m=0.816 gram, d=2cm, also torque_z = 0.816 kg * 0.026 m * 9.81 m/s^2 = 0.208 Nm
#
# 2018.03.09: adjust for UR5 + Robotiq setup (no external load yet)
# total mass F/T sensor: 0.65kg (partition base/tool part unknown)
# total mass hand: 2.3 kg
# approx mass Bluetooth adapter: 0.3 kg
# small TAMS hammer: 0.55 kg total (0.5 head 0.05 handle)
#
# torque_z: small imbalance in the 3-finger gripper, ignored for now.
# 500gr hammer head at distance 72mm from gripper centre,
# 50gr hammer handle ignore

load   = (0.65 + 0.3 + 2.3 + 0.55 )*9.81  # F/T + bluetooth + hand
load_z = (0.65 + 0.3 + 2.3 + 0.55 )*9.81  # as above
load_z_phase2 = load               # 

torque   = 3.52                        # see comments above
torque_z = 0.353        # 0.5 * 9.81 * 0.072                 # hammer head only
#
set   autoscale                        # scale axes automatically
#unset log                              # remove any log-scaling
unset label                            # remove any previous labels
set xtic auto                          # set xtics automatically
set ytic auto                          # set ytics automatically
# set key 0.01,100
# set label "Yield Point" at 0.003,260
# set arrow from 0.0028,250 to 0.003,280
# set xr [0.0:0.022]
# set yr [0:325]
set xlabel "raw sensor output"
set ylabel "Force projected along (0,0,-1)"
set style line 1 lc rgb '#ff0000' lt 1 lw 1 pt 1 ps 0.5 
set style line 2 lc rgb '#00ff00' lt 1 lw 1 pt 1 ps 0.5 
set style line 3 lc rgb '#0000ff' lt 1 lw 1 pt 1 ps 0.5 
set style line 4 lc rgb '#777700' lt 1 lw 1 pt 1 ps 0.5 
#
#
#
set title "Robotiq FTS150 Fx calibration (gain, offset)"
set xlabel "Raw sensor output"
set ylabel "Force projected along (0,0,-1)"
set autoscale                        # scale axes automatically
set terminal png size 640,480
set output "fx.png"
fx(x) = gain_fx * (x - offset_fx)
fit fx(x) file_phase1 using 2:($17*load) via gain_fx, offset_fx
title_f(gain_fx,offset_fx) = sprintf('Fx(raw) = %.3f (raw - %.3f)', gain_fx, offset_fx)
plot file_phase1 using 2:($17*load) with points ls 1, fx(x) title title_f(gain_fx,offset_fx)
#
set title "Robotiq FTS150 Fy calibration (gain, offset)"
set autoscale                        # scale axes automatically
set terminal png size 640,480
set output "fy.png"
fy(x) = gain_fy * (x - offset_fy)
fit fy(x) file_phase1 using 3:($16*load) via gain_fy, offset_fy
title_f(gain_fy,offset_fy) = sprintf('Fy(raw) = %.3f (raw - %.3f)', gain_fy, offset_fy)
plot file_phase1 using 3:($16*load) with points ls 2, fy(x) title title_f(gain_fy,offset_fy)
#
set title "Robotiq FTS150 Fz calibration (gain, offset)"
set autoscale                        # scale axes automatically
set terminal png size 640,480
set output "fz.png"
# gnuplot needs a little help here
gain_fz = 0.1
offset_fz = -700
fz(x) = gain_fz * (x - offset_fz)
fit fz(x) file_phase2 using 4:($15*load_z_phase2) via gain_fz, offset_fz
title_f(gain_fz,offset_fz) = sprintf('Fz(raw) = %.3f (raw - %.3f)', gain_fz, offset_fz)
plot file_phase2 using 4:($15*load_z_phase2) with points ls 3, fz(x) title title_f(gain_fz,offset_fz)
#
#
#
set title "Robotiq FTS150 Tx calibration (gain, offset)"
set xlabel "Raw sensor output"
set ylabel "Torque"
set autoscale                        # scale axes automatically
set terminal png size 640,480
set output "tx.png"
tx(x) = gain_tx * (x - offset_tx)
fit tx(x) file_phase1 using 5:($16*torque) via gain_tx, offset_tx
title_f(gain_tx,offset_tx) = sprintf('Tx(raw) = %.6f (raw - %.3f)', gain_tx, offset_tx)
plot file_phase1 using 5:($16*torque) with points ls 1, tx(x) title title_f(gain_tx,offset_tx)
#
set title "Robotiq FTS150 Ty calibration (gain, offset)"
set xlabel "Raw sensor output"
set ylabel "Torque"
set autoscale                        # scale axes automatically
set terminal png size 640,480
set output "ty.png"
ty(x) = gain_ty * (x - offset_ty)
fit ty(x) file_phase1 using 6:($17*torque) via gain_ty, offset_ty
title_f(gain_ty,offset_ty) = sprintf('Ty(raw) = %.6f (raw - %.3f)', gain_ty, offset_ty)
plot file_phase1 using 6:($17*torque) with points ls 2, ty(x) title title_f(gain_ty,offset_ty)
#
set title "Robotiq FTS150 Tz calibration (gain, offset)"
set xlabel "Raw sensor output"
set ylabel "Torque"
set autoscale                        # scale axes automatically
set terminal png size 640,480
set output "tz.png"
tz(x) = gain_tz * (x - offset_tz)
fit tz(x) file_phase1 using 7:($16*torque_z) via gain_tz, offset_tz
title_f(gain_tz,offset_tz) = sprintf('Tz(raw) = %.6f (raw - %.3f)', gain_tz, offset_tz)
plot file_phase1 using 7:($16*torque_z) with points ls 3, tz(x) title title_f(gain_tz,offset_tz)
#

#
# write logfile with regression results. Note that we have
# to invert the signs of gain_tx and gain_tz to get correct
# orientation.
#
foo = sprintf( "rosrun tams_sunrise_driver set_calibration.sh %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f  %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f", offset_fx, offset_fy, offset_fz, offset_tx, offset_ty, offset_tz, gain_fx, gain_fy, gain_fz, (-1.0*gain_tx), gain_ty, (-1.0*gain_tz) )
print foo
update "ft-regression.dat"
set term x11
