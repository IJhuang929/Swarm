# Simulation 驗證清單

## 1. h(x) 是否全程 > 0，最小值是多少
即時監控（飛行中）：
```
# 監看單架無人機的 h_min，Ctrl+C 停止後看最小值
rostopic echo /drone_1_traj_server/cbf_debug
```
```
# 只萃取 h_min 欄位
rostopic echo /drone_1_traj_server/cbf_debug | grep -oP 'h_min=\S+'
```
飛行結束後（最可靠）：
```
cat /home/ubuntu/catkin_ws/src/Swarm-Formation/1_vaj.txt | grep global_h_min
# 期望看到 global_h_min >= 0（負值代表曾穿透 r_safe）
# 輸出為 -1.0 代表全程無鄰機
```
## 2. replan 觸發頻率與 infeasible fallback
觸發頻率：
```
# 計算 [REPLAN] 訊息出現速率（需要 rqt_console 或 grep rosout）
rostopic echo /rosout | grep "\[REPLAN\]"
```
```
# 分觸發原因統計（飛行中）
rostopic echo /rosout | grep "\[REPLAN\]" | grep -oP 'trigger=\S+' | sort | uniq -c
```
infeasible fallback：
```
# 即時看 infeasible=1 何時出現
rostopic echo /drone_1_traj_server/cbf_debug | grep 'infeasible=1'
```
```
# 飛行結束後查總次數
grep 'cbf_infeasible\|cbf_triggers' /home/ubuntu/catkin_ws/src/Swarm-Formation/1_vaj.txt
```
## 3. 有無 CBF 的執行時間比較
步驟：
1. 在 launch file 或 param 設 cbf/enabled: false，跑一次並記錄 result file
2. 改回 cbf/enabled: true，跑相同場景
3. 比較兩份 result file：
```
# 無 CBF
grep cmd_exec_ms /home/ubuntu/catkin_ws/src/Swarm-Formation/1_vaj.txt
# 範例輸出：cmd_exec_ms=0.08(mean) 0.42(max)
```
```
# 有 CBF（QP 求解會拉高 mean 與 max）
grep cmd_exec_ms /home/ubuntu/catkin_ws/src/Swarm-Formation/1_vaj.txt
# 範例輸出：cmd_exec_ms=0.23(mean) 1.80(max)
```
## 4. 速度指令有沒有被修改
```
# 當 triggered=1 時，vel_after 應 < vel_before
rostopic echo /drone_1_traj_server/cbf_debug
```
```
# 篩出有介入的瞬間確認 vel_after < vel_before
rostopic echo /drone_1_traj_server/cbf_debug | grep 'triggered=1'
```
同時對照實際指令：
```
# 看 position_cmd 的 velocity 欄位（CBF 介入時應該跟 vel_after 一致）
rostopic echo /position_cmd | grep -A3 velocity
```
## 5. 編隊任務是否完成
```
# 即時等待每架無人機的 finish 訊號（全部為 true 才算完成）
rostopic echo /drone_0_traj_server/planning/finish
rostopic echo /drone_1_traj_server/planning/finish
rostopic echo /drone_2_traj_server/planning/finish
```
```
# 從 terminal 看 goal reached 訊息（白底黑字）
rostopic echo /rosout | grep "reached goal"
```
```
# 飛行結束後確認各架總飛行時間和平均規劃時間
cat /home/ubuntu/catkin_ws/src/Swarm-Formation/ego_swarm.txt
# 格式：drone_id  total_time  average_plan_time
```
快速一行總覽（多架同時監控）
```
# 所有無人機 cbf_debug 同時輸出（需要開多個 terminal 或用 terminator）
for i in 0 1 2; do
  rostopic echo /drone_${i}_traj_server/cbf_debug &
done
```

source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash; for i in 0 1 2 3 4 5 6; do echo "===== drone_$i ====="; for t in /drone_${i}_planning/trajectory /drone_${i}_planning/start /drone_${i}_planning/finish /drone_${i}_planning/pos_cmd /drone_${i}_visual_slam/odom; do echo "--- $t"; rostopic info "$t" 2>/dev/null | sed 's/^/    /'; done; done; echo "===== trigger topic ====="; rostopic info /move_base_simple/goal | sed 's/^/    /'

source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash; for i in 0 1 2 3 4 5 6; do echo "===== drone_$i message check ====="; for t in /drone_${i}_planning/start /drone_${i}_planning/trajectory /drone_${i}_planning/finish; do if timeout 2 rostopic echo -n 1 "$t" >/dev/null 2>&1; then echo "$t : HAS_MSG"; else echo "$t : NO_MSG_WITHIN_2S"; fi; done; echo; done
