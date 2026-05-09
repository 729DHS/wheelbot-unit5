cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unit5_app)

target_sources(app PRIVATE
	src/main.c
	src/dm4310_motor.c
	src/shell_commands.c
	src/linkage_kinematics.c
	src/leg_control.c
	src/motor_debug.c
	src/robot_ctrl.c
	src/protocol_service.c
	src/kinematics_test.c
)
