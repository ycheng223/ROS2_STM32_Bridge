#!/bin/bash

SESSION="main_view"

tmux kill-session -t $SESSION || true

tmux set-option -g pane-border-status top
tmux set-option -g pane-border-format " #{pane_index} | #{pane_title} "

tmux new-session -d -s $SESSION -n monitor
tmux split-window -h -t $SESSION:monitor
sleep 0.2
tmux split-window -h -t $SESSION:monitor.0
sleep 0.2
tmux new-window -t $SESSION -n detect

tmux select-layout -t $SESSION:monitor even-horizontal

tmux select-pane -t $SESSION:monitor.0 -T "Gyro Data"
tmux select-pane -t $SESSION:monitor.1 -T "Accel Data"
tmux select-pane -t $SESSION:monitor.2 -T "Velocity Cmd"
tmux select-pane -t $SESSION:detect.0 -T "Detections"

tmux send-keys -t $SESSION:monitor.0 "ros2 topic echo /camera/camera/gyro/sample" C-m
tmux send-keys -t $SESSION:monitor.1 "ros2 topic echo /camera/camera/accel/sample" C-m
tmux send-keys -t $SESSION:monitor.2 "ros2 topic echo /nav/cmd_vel" C-m 
tmux send-keys -t $SESSION:detect.0 "ros2 topic echo /detections" C-m

tmux attach-session -t $SESSION