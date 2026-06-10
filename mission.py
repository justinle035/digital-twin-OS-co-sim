# Day la file mission chay trong app Ubuntu 22.04.5 LTS

from pymavlink import mavutil
import time
import math

print(" Flight Mission Started")

master = mavutil.mavlink_connection('udpin:127.0.0.1:14540', source_system=255)
master.wait_heartbeat(timeout=10)

def set_param(param_id, param_value, param_type):
    master.mav.param_set_send(1, 1, param_id.encode('utf-8'), param_value, param_type)
    time.sleep(0.1)

print("[SYS] Bypassing PX4 Constraints...")
set_param("COM_RC_IN_MODE", 4.0, 6)

# Mở khóa các hạn chế vận tốc/hướng
set_param("MPC_XY_CRUISE", 30.0, 9)       # tốc độ cruising mong muốn 30 m/s
set_param("MPC_XY_VEL_MAX", 40.0, 9)      # tốc độ tối đa 40 m/s
set_param("MPC_TILTMAX_AIR", 70.0, 9)     # cho phép pitch lớn hơn
set_param("VT_ARSP_TRANS", 14.0, 9)       # vận tốc commit transition ~14 m/s
set_param("VT_TRANS_MIN_TM", 3.0, 9)      # thời gian tối thiểu trước khi thả điều khiển
set_param("VT_FWD_THRUST_SC", 2.0, 9)     # nhân scale thrust pusher
set_param("VT_F_TRANS_THR", 1.2, 9)       # tăng giới hạn thrust trong transition
set_param("FW_AIRSPD_TRIM", 20.0, 9)      # airspeed trim cao hơn

g_alt_rel, g_vf = 0.0, 0.0

def update_telemetry():
    global g_alt_rel, g_vf
    master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if not msg:
            return
        g_alt_rel = msg.relative_alt / 1000.0
        g_vf = math.sqrt((msg.vx/100.0)**2 + (msg.vy/100.0)**2)

print("[SYS] Waiting for GPS lock...")
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg and msg.lat != 0:
        home_lat, home_lon = msg.lat/1e7, msg.lon/1e7
        target_alt = msg.alt/1000.0 + 15.0  # Mốc 15m
        break

print("\n[STEP 1] TAKEOFF TO 15M")
master.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(1)
master.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, home_lat, home_lon, target_alt)

while True:
    update_telemetry()
    print(f"   [TAKEOFF] Alt: {g_alt_rel:.1f}m | Vel: {g_vf:.1f}m/s".ljust(60), end='\r')
    if g_alt_rel >= 14.0:
        break
    time.sleep(0.05)

print("\n[STEP 2] LONG FORWARD FLIGHT FOR DATA MATCHING")
master.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, 0, 4, 0, 0, 0, 0, 0, 0)
time.sleep(0.5)
master.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 40.0, 0, 0, 0, home_lat + 0.0200, home_lon, target_alt)  # bay xa hơn

start_time = time.time()
while True:
    update_telemetry()
    elapsed = time.time() - start_time
    # In ra vận tốc và độ cao mỗi 0.05 s
    print(f"   [FW] Time: {elapsed:.1f}s | Vel: {g_vf:.1f}m/s | Alt: {g_alt_rel:.1f}m".ljust(70), end='\r')
    # tiếp tục bay trong 30–40 s hoặc tới khi vượt 35 m/s
    if elapsed >= 35.0 or g_vf >= 35.0:
        break
    time.sleep(0.05)

print("\n\n[SYS] Collection complete. Disarming...")
master.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

print("\ncd /home/nh4tnguyen/PX4-Autopilot/build/px4_sitl_default/rootfs/log/\ncd 2026-04-xx\nls -lt *.ulg\nulog2csv xx_yy_zz.ulg\ncp *vehicle_local_position_0.csv /mnt/c/Users/Acer/Desktop/")