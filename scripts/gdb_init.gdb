# GDB 初始化脚本 — DM4310 电机调试
# 由 scripts/debug_gdb.sh 加载

target extended-remote :3333
monitor reset init

echo === DM4310 电机调试 GDB 会话 ===\n
echo \n
echo 命令会自动 resume → 执行 → halt，直接用：\n
echo   motor_enable 1         使能 M1\n
echo   motor_enable 1 2 3 4   使能全部\n
echo   motor_disable 2        失能 M2\n
echo   motor_disable 0        失能全部\n
echo   motor_zero 1 2 3 4     全部置零\n
echo   motor_status           查看状态\n
echo   motor_csv              单次角度\n
echo   motor_kp 1 0.5         设置 KP\n
echo   motor_kd 2 0.05        设置 KD\n
echo   motor_stop             紧急停止\n
echo   motor_go               手动恢复运行\n
echo ================================\n

# 内部辅助: resume → 等主循环处理 → halt 读状态 → 立即 resume
# 目标只在 interrupt→continue 之间暂停几十 ms，串口几乎不断流
define _motor_exec
    continue&
    shell sleep 0.5
    interrupt
    continue&
end

define motor_go
    continue&
    printf "目标继续运行\n"
end

define motor_enable
    if $argc < 1
        printf "用法: motor_enable <1-4> [...]  或 0=all\n"
    else
        set $_i = 0
        while $_i < $argc
            if $_i == 0
                set $_id = $arg0
            end
            if $_i == 1
                set $_id = $arg1
            end
            if $_i == 2
                set $_id = $arg2
            end
            if $_i == 3
                set $_id = $arg3
            end
            if $_id >= 1 && $_id <= 4
                set g_gdb_cmd[$_id - 1] = 1
            else
                if $_id == 0
                    set $j = 0
                    while $j < 4
                        set g_gdb_cmd[$j] = 1
                        set $j = $j + 1
                    end
                end
            end
            set $_i = $_i + 1
        end
        _motor_exec
        printf "ENABLE 已执行\n"
        motor_status
    end
end

define motor_disable
    if $argc < 1
        printf "用法: motor_disable <1-4> [...]  或 0=all\n"
    else
        set $_i = 0
        while $_i < $argc
            if $_i == 0
                set $_id = $arg0
            end
            if $_i == 1
                set $_id = $arg1
            end
            if $_i == 2
                set $_id = $arg2
            end
            if $_i == 3
                set $_id = $arg3
            end
            if $_id >= 1 && $_id <= 4
                set g_gdb_cmd[$_id - 1] = 2
            else
                if $_id == 0
                    set $j = 0
                    while $j < 4
                        set g_gdb_cmd[$j] = 2
                        set $j = $j + 1
                    end
                end
            end
            set $_i = $_i + 1
        end
        _motor_exec
        printf "DISABLE 已执行\n"
        motor_status
    end
end

define motor_zero
    if $argc < 1
        printf "用法: motor_zero <1-4> [...]  或 0=all\n"
    else
        set $_i = 0
        while $_i < $argc
            if $_i == 0
                set $_id = $arg0
            end
            if $_i == 1
                set $_id = $arg1
            end
            if $_i == 2
                set $_id = $arg2
            end
            if $_i == 3
                set $_id = $arg3
            end
            if $_id >= 1 && $_id <= 4
                set g_gdb_cmd[$_id - 1] = 3
            else
                if $_id == 0
                    set $j = 0
                    while $j < 4
                        set g_gdb_cmd[$j] = 3
                        set $j = $j + 1
                    end
                end
            end
            set $_i = $_i + 1
        end
        _motor_exec
        printf "ZERO 已执行\n"
        motor_status
    end
end

define motor_kp
    if $argc < 2
        printf "用法: motor_kp <1-4> <value>\n"
    else
        set g_dm4310.hold_kp[$arg0 - 1] = (float)$arg1
        set g_dm4310.hold_updates = 1
        printf "M%d KP = %.3f\n", $arg0, $arg1
    end
end

define motor_kd
    if $argc < 2
        printf "用法: motor_kd <1-4> <value>\n"
    else
        set g_dm4310.hold_kd[$arg0 - 1] = (float)$arg1
        set g_dm4310.hold_updates = 1
        printf "M%d KD = %.3f\n", $arg0, $arg1
    end
end

define motor_online
    printf "online_mask = 0x%x  bringup_done = %d  loops = %u\n", \
        g_dm4310.online_mask, g_dm4310.bringup_done, g_dm4310.loops
end

define motor_status
    motor_online
    set $i = 0
    while $i < 4
        set $m = &g_dm4310.motor[$i]
        printf "  M%d: %s  pos=%+.4f rad (%+.1f°)  vel=%+.4f  t_mos=%d t_coil=%d  rx=%u  kp=%.3f kd=%.3f\n", \
            $i + 1, \
            $m->online ? "ON " : "OFF", \
            (double)$m->pos_rad, \
            (double)$m->pos_rad * 180.0 / 3.1415926535, \
            (double)$m->vel_radps, \
            $m->mos_temp, $m->coil_temp, $m->rx_count, \
            (double)g_dm4310.hold_kp[$i], \
            (double)g_dm4310.hold_kd[$i]
        set $i = $i + 1
    end
end

define motor_csv
    printf "%u,%.4f,%.4f,%.4f,%.4f\n", \
        g_dm4310.loops, \
        (double)g_dm4310.motor[0].pos_rad, \
        (double)g_dm4310.motor[1].pos_rad, \
        (double)g_dm4310.motor[2].pos_rad, \
        (double)g_dm4310.motor[3].pos_rad
end

define motor_stop
    set $j = 0
    while $j < 4
        set g_gdb_cmd[$j] = 2
        set $j = $j + 1
    end
    _motor_exec
    printf "紧急停止: 全部 DISABLE 已执行\n"
end
