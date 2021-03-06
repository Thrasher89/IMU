#
# Makefile for the px4fmu_default configuration
#

#
# Use the configuration's ROMFS.
#
ROMFS_ROOT	 = $(PX4_BASE)/ROMFS/px4fmu_minimal
#ROMFS_OPTIONAL_FILES = $(PX4_BASE)/Images/px4io-v1_minimal.bin

#
# Board support modules
#
#MODULES += drivers/stim300
MODULES += modules/stim300
MODULES += modules/s
MODULES += modules/uart

MODULES		+= drivers/device
MODULES		+= drivers/stm32
MODULES		+= drivers/stm32/adc
MODULES		+= drivers/stm32/tone_alarm
MODULES		+= drivers/led
#MODULES		+= drivers/px4io
MODULES		+= drivers/px4fmu
MODULES		+= drivers/boards/px4fmu-v1
#MODULES		+= drivers/l3gd20
#MODULES		+= drivers/mpu6000
#MODULES		+= drivers/hmc5883
#MODULES		+= drivers/ms5611
#MODULES		+= drivers/gps
MODULES		+= drivers/rgbled
#MODULES		+= drivers/airspeed
#MODULES		+= drivers/ets_airspeed
#MODULES		+= drivers/meas_airspeed
#MODULES		+= drivers/frsky_telemetry
#MODULES		+= modules/sensors

#
# System commands
#
#MODULES		+= systemcmds/mtd
MODULES		+= systemcmds/mixer
#MODULES		+= systemcmds/param
#MODULES		+= systemcmds/perf
#MODULES		+= systemcmds/preflight_check
#MODULES		+= systemcmds/pwm
#MODULES		+= systemcmds/esc_calib
MODULES		+= systemcmds/reboot
MODULES		+= systemcmds/top
MODULES		+= systemcmds/config
MODULES		+= systemcmds/nshterm
MODULES		+= systemcmds/dumpfile
MODULES		+= systemcmds/ver

#
# General system control
#
#MODULES		+= modules/mavlink
#MODULES		+= modules/gpio_led

#
# Estimation modules (EKF / other filters)
#
#MODULES		+= modules/attitude_estimator_ekf
#MODULES		+= modules/attitude_estimator_so3
#MODULES		+= modules/ekf_att_pos_estimator
#MODULES		+= modules/position_estimator_inav


#
# Logging
#
#MODULES		+= modules/sdlog2


#
# Library modules
#
MODULES		+= modules/systemlib
MODULES		+= modules/systemlib/mixer
#MODULES		+= modules/controllib
MODULES		+= modules/uORB
#MODULES		+= modules/dataman

#
# Libraries
#
LIBRARIES	+= lib/mathlib/CMSIS
MODULES		+= lib/mathlib
MODULES		+= lib/mathlib/math/filter
MODULES		+= lib/ecl
#MODULES		+= lib/external_lgpl
#MODULES		+= lib/geo
#MODULES		+= lib/geo_lookup
MODULES		+= lib/conversion
MODULES		+= lib/launchdetection


#
# Transitional support - add commands from the NuttX export archive.
#
# In general, these should move to modules over time.
#
# Each entry here is <command>.<priority>.<stacksize>.<entrypoint> but we use a helper macro
# to make the table a bit more readable.
#
define _B
	$(strip $1).$(or $(strip $2),SCHED_PRIORITY_DEFAULT).$(or $(strip $3),CONFIG_PTHREAD_STACK_DEFAULT).$(strip $4)
endef

#                  command                 priority                   stack  entrypoint
BUILTIN_COMMANDS := \
	$(call _B, sercon,                 ,                          2048,  sercon_main                ) \
	$(call _B, serdis,                 ,                          2048,  serdis_main                )
