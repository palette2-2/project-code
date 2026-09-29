#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")"

actual_human_height="${ACTUAL_HUMAN_HEIGHT:-1.6}"
lookback_ms="${LOOKBACK_MS:-25}"
log_interval_s="${LOG_INTERVAL_S:-1}"
lcm_url="${LCM_URL:-udpm://239.255.76.67:7667?ttl=255}"
lcm_channel="${LCM_CHANNEL:-ref_upper_dof_pos_channel}"
publish_pico_rc="${PUBLISH_PICO_RC:-1}"
pico_rc_channel="${PICO_RC_CHANNEL:-pico_rc_command}"
publish_dex3_hand="${PUBLISH_DEX3_HAND:-0}"
dex3_pose_config="${DEX3_POSE_CONFIG:-config/dex3_hand_poses.json}"
dex3_hand_channel="${DEX3_HAND_CHANNEL:-hand_action}"
dex3_hand_transition_s="${DEX3_HAND_TRANSITION_S:-0.5}"

cmd=(
    python xrobot_teleop_to_pose_zmq_server.py
    --robot unitree_g1
    --actual_human_height "${actual_human_height}"
    --ctrl_fps 50
    --lookback_ms "${lookback_ms}"
    --retarget_buffer_window_s 0.5
    --log_interval_s "${log_interval_s}"
    --req_bind_addr tcp://*:28701
    --rep_bind_addr tcp://*:28702
    --ctrl_bind_addr tcp://*:28703
    --lcm_url "${lcm_url}"
    --lcm_channel "${lcm_channel}"
    --min_link_height 0.0
    --min_link_height_align_strategy startup_fixed
    --min_link_height_bootstrap_frames 10
    --visualize
    --vis_fps 5
)

if [[ "${publish_pico_rc}" != "0" ]]; then
    cmd+=(
        --enable_lcm_rc
        --rc_lcm_channel "${pico_rc_channel}"
    )
fi

if [[ "${publish_dex3_hand}" != "0" ]]; then
    cmd+=(
        --enable_lcm_hand
        --hand_lcm_channel "${dex3_hand_channel}"
        --dex3_pose_config "${dex3_pose_config}"
        --hand_transition_s "${dex3_hand_transition_s}"
    )
fi

"${cmd[@]}" "$@"
