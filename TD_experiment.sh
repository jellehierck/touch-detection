#!/bin/bash

# shellcheck disable=SC1091
# shellcheck disable=SC1090

# -----------------------------------------------------
# Constants
# -----------------------------------------------------

# Supported clock synchronzation types
readonly CLOCK_SYNC_TIMESYNCD="timesyncd"
readonly CLOCK_SYNC_CHRONY_LOCAL="chrony_local"

# Supported offset values
readonly OFFSET_0S="0s"
readonly OFFSET_15MS="15ms"
readonly OFFSET_45MS="45ms"
readonly OFFSET_150MS="150ms"
readonly OFFSET_1S="1s"

# PC names
readonly ROBOT_PC="robot"
readonly AI_PC="ai"
readonly VISION_PC="vision"
readonly FORCE_PC="force"

# Local NTP layout
readonly LOCAL_NTP_SERVER_PC="${AI_PC}"
readonly LOCAL_NTP_SERVER_IP="10.4.0.15"
readonly OFFSET_CLIENT_PC="${FORCE_PC}"
readonly DATA_COLLECTION_PC="${FORCE_PC}"
# readonly DATA_COLLECTION_PC="${VISION_PC}"

# ROS Domain ID specifications
readonly ROS_DOMAIN_ID_FORCE_TO_VISION=12 # Clock burst
readonly ROS_DOMAIN_ID_FORCE_TO_ROBOT=13  # Clock burst
readonly ROS_DOMAIN_ID_VISION_TO_ROBOT=23 # Clock burst
readonly ROS_DOMAIN_ID_EXPERIMENT=40      # Experiment nodes

# Time to wait after local sync setup
readonly DEFAULT_CLOCK_SYNC_SETTLE_TIME_SECONDS=$((5 * 60))

# -----------------------------------------------------
# Informative print functions
# -----------------------------------------------------

# Function to display usage/help
print_help() {
    cat <<EOF
Usage: $0 [OPTIONS]

Set up TD experiments.

Informational options (optional):
    -c, --conditions        Print all possible conditions and corresponding 
                            experiment numbers and exit.

    -h, --help              Show this help message and exit.

Input options (required for some execution options):
    --pc NAME               [REQUIRED] Name of this PC. The PC name will dictate
                            some options set for the synchronization, offset
                            monitoring and data collection.

    -e, --exp EXP_NR        Experiment number to set up. Execution options list
                            whether this flag is required.

    -r, --rep REP_NR        Repetition number to set up. Execution options list
                            whether this flag is required.

Modifier options (optional):
    --demo                  Start in demo mode, which will change filenames and 
                            not record clock offsets.

Execution options (choose one):
    --setup-sync            Set up NTP clock synchronization. Requires 
                            '-e'/'--exp'. This will start the clock sync method 
                            configured for the experiment. It will also start a 
                            timer to settle the sync method for ${DEFAULT_CLOCK_SYNC_SETTLE_TIME_SECONDS} s.

    --setup-exp             Set up clock monitoring nodes in the background and 
                            nodes which persist between experiments (such as 
                            camera and robot control) in the foreground. 
                            Requires '-e'/'--exp' on all PCs, also requires
                            '-r'/'--rep' on the data collection PC 
                            ('${DATA_COLLECTION_PC}').

    --start-rep             Start an experiment with a specific repetition. 
                            Requires '-e'/'--exp' AND '-r'/'--rep'. This only 
                            has an effect on the data collection PC 
                            ('${DATA_COLLECTION_PC}').

    --record-bag            Record rosbag data for a specific repetition.
                            The recording is started in the foreground and must
                            be manually stopped with Ctrl+C when finished.
                            Requires '-e'/'--exp' AND '-r'/'--rep'. This only 
                            has an effect on the data collection PC 
                            ('${DATA_COLLECTION_PC}').

    --empty-rep             Empty all stored data recorded for a repetition.
                            Requires '-e'/'--exp' AND '-r'/'--rep'. This only 
                            has an effect on the data collection PC 
                            ('${DATA_COLLECTION_PC}').

    --force-empty-rep       Same as '--empty-rep', but asks for sudo permissions
                            to forcefully remove the repetitions samples. Use
                            this for lingering samples which have the wrong 
                            owner due to Docker settings.

    --clean-exp             Clean up all nodes related to an experiment.
                            Requires '-e'/'--exp'.

    --clean-sync            Clean up synchronization and revert to default NTP
                            ('timesyncd'). Requires '-e'/'--exp'.

Set up TD experiments.

Only one of the execution options can be set

EOF
}

print_conditions() {
    cat <<EOF
TD conditions:

| Exp nr | Sync type    | Offset |
| ------ | ------------ | ------ |
| 1      | timesyncd    | 0s     |
| 2      | chrony_local | 0s     |
| 3      | chrony_local | 15ms   |
| 4      | chrony_local | 45ms   |
| 5      | chrony_local | 150ms  |
| 6      | chrony_local | 1s     |

EOF
}

# -----------------------------------------------------
# Helper functions
# -----------------------------------------------------

# Ensure that exactly one flag is true
ensure_exactly_one_true() {
    local count=0
    for val in "$@"; do
        [[ "$val" == "true" ]] && ((count++))
    done

    [[ $count -eq 1 ]]
}

# Show a stopwatch on screen which can be interrupted with ctrl+C
stopwatch() {
    start=$(date +%s)
    trap 'echo; exit' SIGINT # Print newline on Ctrl+C
    while true; do
        # Print time
        time="$(($(date +%s) - start))"
        printf '\rElapsed time: %s' "$(date -u -d "@$time" +%H:%M:%S)"
        sleep 0.1
    done
    echo -e "\nStopwatch interrupted."
    echo # Print an extra blank line for clarity
}

# -----------------------------------------------------
# Argument parsing
# -----------------------------------------------------

pc_name=
exp_nr=
rep_nr=
setup_sync="false"
setup_exp="false"
start_rep="false"
clean_exp="false"
clean_sync="false"
empty_rep="false"
force_empty_rep="false"
demo="false"

# Iterate all arguments to check for flags
while test $# -gt 0; do
    case "$1" in

    --pc)
        shift
        if [[ -z "$1" ]] || [[ "$1" == -* ]]; then
            echo "ERROR: Missing NAME argument after --pc"
            exit 1
        fi
        pc_name="$1"
        shift
        ;;

    -e | --exp)
        shift
        if [[ -z "$1" ]] || [[ "$1" == -* ]]; then
            echo "ERROR: Missing EXP_NR argument after '-e'/'--exp'"
            exit 1
        fi
        exp_nr="$1"
        shift
        ;;

    -r | --rep)
        shift
        if [[ -z "$1" ]] || [[ "$1" == -* ]]; then
            echo "ERROR: Missing REP_NR argument after '-r'/'--rep'"
            exit 1
        fi
        rep_nr="$1"
        shift
        ;;

    --setup-sync)
        setup_sync="true"
        shift
        ;;

    --setup-exp)
        setup_exp="true"
        shift
        ;;

    --start-rep)
        start_rep="true"
        shift
        ;;

    --empty-rep)
        empty_rep="true"
        shift
        ;;

    --force-empty-rep)
        empty_rep="true"
        force_empty_rep="true"
        shift
        ;;

    --clean-exp)
        clean_exp="true"
        shift
        ;;

    --clean-sync)
        clean_sync="true"
        shift
        ;;

    # Demo modifier
    --demo)
        demo="true"
        shift
        ;;

    # Display conditions
    -c | --conditions)
        print_conditions
        exit 0
        ;;

    # Display help
    -h | --help)
        print_help
        exit 0
        ;;

    # Error if invalid option is provided
    *)
        echo ""
        echo "ERROR: Invalid argument: $1"
        echo "Try '$0 --help' for more information."
        exit 1
        ;;
    esac
done

# -----------------------------------------------------
# Process input arguments
# -----------------------------------------------------

# Check that script is NOT run with sudo
if [[ "${EUID}" -eq 0 ]]; then
    echo ""
    echo "ERROR: $0 must be run without sudo privileges."
    echo "       It will ask for a sudo password when needed."
    exit 1
fi

# Check that experiment number is passed
if [[ -v "${exp_nr}" && ("${setup_sync}" == "true" || "${setup_exp}" == "true" || "${start_rep}" == "true" || "${empty_rep}" == "true") ]]; then
    echo ""
    echo "ERROR: Option '-e'/'--exp' is required with '--setup-sync', '--setup-exp', '--empty-rep' or '--start-rep'."
    exit 1
fi

# Check that the repetition number is set under the right conditions
if [[ -v "${rep_nr}" && ("${start_rep}" == "true" || "${empty_rep}" == "true" || ("${pc_name}" == "${DATA_COLLECTION_PC}" && "${setup_exp}" = "true")) ]]; then
    echo ""
    echo "ERROR: Option '-r'/'--rep' is required with '--empty-rep' or '--start-rep', and also with '--setup-exp' only on the data collection PC (${DATA_COLLECTION_PC})."
    exit 1
fi

# Obtain the conditions associated to this experiment number
case "${exp_nr}" in
"1")
    readonly sync_type="${CLOCK_SYNC_TIMESYNCD}"
    readonly offset="${OFFSET_0S}"
    ;;
"2")
    readonly sync_type="${CLOCK_SYNC_CHRONY_LOCAL}"
    readonly offset="${OFFSET_0S}"
    ;;
"3")
    readonly sync_type="${CLOCK_SYNC_CHRONY_LOCAL}"
    readonly offset="${OFFSET_15MS}"
    ;;
"4")
    readonly sync_type="${CLOCK_SYNC_CHRONY_LOCAL}"
    readonly offset="${OFFSET_45MS}"
    ;;
"5")
    readonly sync_type="${CLOCK_SYNC_CHRONY_LOCAL}"
    readonly offset="${OFFSET_150MS}"
    ;;
"6")
    readonly sync_type="${CLOCK_SYNC_CHRONY_LOCAL}"
    readonly offset="${OFFSET_1S}"
    ;;
*)
    echo ""
    echo "ERROR: Invalid experiment number: $exp_nr"
    echo "       Provide an experiment number with '--exp EXP_NR'."
    echo "       Try '$0 --conditions' for conditions and valid experiment numbers."
    exit 1
    ;;
esac

# Convert offset duration string to fractional seconds
case "${offset}" in
"${OFFSET_0S}")
    readonly offset_sec="0.0"
    ;;
"${OFFSET_15MS}")
    readonly offset_sec="0.015"
    ;;
"${OFFSET_45MS}")
    readonly offset_sec="0.045"
    ;;
"${OFFSET_150MS}")
    readonly offset_sec="0.150"
    ;;
"${OFFSET_1S}")
    readonly offset_sec="1.0"
    ;;
esac

if [[ "${demo}" == "false" ]]; then
    # Infer the experiment name from the experiment conditions and repetition number
    readonly EXPERIMENT_NAME="td_exp_${exp_nr}_${sync_type}_offset_${offset}_rep_${rep_nr}"
else
    # Use a special experiment name for demo
    readonly EXPERIMENT_NAME="td_demo_${exp_nr}_${sync_type}_offset_${offset}_rep_${rep_nr}"
fi
# -----------------------------------------------------
# Obtain used paths from PCs
# -----------------------------------------------------

# Check PC name and set PC-specific paths
case "${pc_name}" in
"${ROBOT_PC}")
    # Where script for clock sync setup is located
    readonly SETUP_SYNC_SCRIPT_PATH="${HOME}/clock_sync_ws/src/clock_synchronization/chrony/setup_local_network_chrony.sh"

    # Where the docker-compose file for clock monitoring is located
    readonly CLOCK_SYNC_DOCKER_COMPOSE_PATH="${HOME}/clock_sync_ws/src/clock_synchronization/docker-compose.yml"

    # Where the docker-compose file for the robot launch file
    readonly ROBOT_LAUNCH_DOCKER_COMPOSE_PATH="${HOME}/nakama_ws/src/touch_detection_robot/docker-compose.yml"
    ;;

# Vision PC is disabled as TD experiment only needs 2 PCs
# "${VISION_PC}")
#     # Where script for clock sync setup is located
#     readonly SETUP_SYNC_SCRIPT_PATH="${HOME}/git/clock_sync_ws/src/clock_synchronization/chrony/setup_local_network_chrony.sh"

#     # Where the docker-compose file for clock monitoring is located
#     readonly CLOCK_SYNC_DOCKER_COMPOSE_PATH="${HOME}/git/clock_sync_ws/src/clock_synchronization/docker-compose.yml"

#     # Get the clock burst output folder (and make sure it is created)
#     readonly CLOCK_BURST_OUTPUT_FOLDER_PATH="${HOME}/git/clock_sync_ws/src/clock_synchronization/output/${EXPERIMENT_NAME}"

#     # Get the folder where bag files should be stored
#     readonly BAG_FILE_OUTPUT_FOLDER_PATH="${HOME}/git/clock_sync_ws/src/touch-detection/recording/output"
#     ;;

"${FORCE_PC}")
    # Where script for clock sync setup is located
    readonly SETUP_SYNC_SCRIPT_PATH="${HOME}/thesis/nakama_ws/src/clock_synchronization/chrony/setup_local_network_chrony.sh"

    # Where the docker-compose file for clock monitoring is located
    readonly CLOCK_SYNC_DOCKER_COMPOSE_PATH="${HOME}/thesis/nakama_ws/src/clock_synchronization/docker-compose.yml"

    # Get the clock burst output folder
    readonly CLOCK_BURST_OUTPUT_FOLDER_PATH="${HOME}/thesis/nakama_ws/src/clock_synchronization/output/${EXPERIMENT_NAME}"

    # Where the docker-compose file for the force sensor is located
    readonly FORCE_SENSOR_DOCKER_COMPOSE_PATH="${HOME}/thesis/Simple_Senseone_eth_container/docker-compose.yaml"

    # Where the docker-compose file for the bagfile recording is located
    readonly BAGFILE_RECORDING_DOCKER_COMPOSE_PATH="${HOME}/thesis/nakama_ws/src/touch-detection/recording/docker-compose.yml"

    # Get the folder where bag files should be stored
    readonly BAG_FILE_OUTPUT_FOLDER_PATH="${HOME}/thesis/nakama_ws/src/touch-detection/recording/output"
    ;;

"${AI_PC}")
    # Where script for clock sync setup is located
    readonly SETUP_SYNC_SCRIPT_PATH="${HOME}/Documents/git/clock_sync_ws/src/clock_synchronization/chrony/setup_local_network_chrony.sh"

    # No other paths are necessary for the AI PC
    ;;

*)
    echo ""
    echo "ERROR: Invalid PC name: $pc_name"
    exit 1
    ;;
esac

# -----------------------------------------------------
# Set up synchronization
# -----------------------------------------------------

# Define the function to set up synchroniation
execute_setup_sync() {
    # Ask for sudo password once upfront
    echo ""
    echo "Setting up synchronization requires sudo privileges."
    echo "You will be prompted for your password."
    sudo -v || {
        echo "Failed to authenticate with sudo"
        exit 1
    }

    echo ""
    # Determine which arguments will be passed to the clock sync setup script
    setup_clock_sync_arguments=()
    case "${sync_type}" in
    "${CLOCK_SYNC_TIMESYNCD}")
        setup_clock_sync_arguments=(--restore-timesyncd)
        ;;
    "${CLOCK_SYNC_CHRONY_LOCAL}")
        # Check if a PC needs special treatment
        case "${pc_name}" in
        "${LOCAL_NTP_SERVER_PC}")
            setup_clock_sync_arguments=(--server --server-ip "${LOCAL_NTP_SERVER_IP}")
            echo "Local NTP Server, serving on IP ${LOCAL_NTP_SERVER_IP}"
            ;;

        *)
            setup_clock_sync_arguments=(--client --server-ip "${LOCAL_NTP_SERVER_IP}")
            echo "Local NTP Client, connecting to Local Server on IP ${LOCAL_NTP_SERVER_IP}"

            # If the offset is nonzero, apply it to the offset Client PC
            if (($(echo "${offset_sec} > 0.0" | bc -l))) && [[ "${pc_name}" = "${OFFSET_CLIENT_PC}" ]]; then
                setup_clock_sync_arguments+=(--offset "${offset_sec}")
                echo "Applying offset of ${offset_sec} seconds"
            fi
            ;;

        esac
        ;;
    esac

    # Execute the synchronization setup script
    printf -v args_str "%q " "${setup_clock_sync_arguments[@]}"
    echo "Calling synchronization setup script with the following signature: 'sudo ${SETUP_SYNC_SCRIPT_PATH} ${args_str}'"
    sudo "${SETUP_SYNC_SCRIPT_PATH}" "${setup_clock_sync_arguments[@]}"

    # Show a countdown timer until the clocks have settled
    # https://serverfault.com/a/532564

    echo ""
    echo "Letting the clock sync settle for ${DEFAULT_CLOCK_SYNC_SETTLE_TIME_SECONDS} seconds."
    echo "This script will close automatically when the timer runs out. You can exit early with Ctrl+C."
    settle_time_seconds="${DEFAULT_CLOCK_SYNC_SETTLE_TIME_SECONDS}"
    while [[ "${settle_time_seconds}" -gt 0 ]]; do
        echo -ne "Time left to settle: ${settle_time_seconds} s\033[0K\r"
        sleep 1
        : $((settle_time_seconds--))
    done

    echo ""
    echo "DONE: Clock sync has settled for ${DEFAULT_CLOCK_SYNC_SETTLE_TIME_SECONDS} seconds."
}

# -----------------------------------------------------
# Setup experiment
# -----------------------------------------------------

# Define the function to set up experiment
execute_setup_exp() {
    echo ""
    echo "Stopping any previous clock monitoring nodes..."
    docker container ls -qa --filter name=clock_synchronization* | xargs -r docker rm -f

    # Check PC name and start nodes specific to that PC
    case "${pc_name}" in
    "${ROBOT_PC}")
        echo ""
        echo "Starting clock monitoring nodes in the background..."

        # # Vision PC to Robot PC (second node)
        # docker compose -f "${CLOCK_SYNC_DOCKER_COMPOSE_PATH}" \
        #     run --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VISION_TO_ROBOT}" --detach clock_sync \
        #     ros2 run clock_sync_cpp bounce_clocks --ros-args -r __name:=bounce_clocks2 -r /bounce_input:=/bounce -r /bounce_output:=/bounce -p nr_bounces:=100

        # Force PC to Robot PC (second node)
        docker compose -f "${CLOCK_SYNC_DOCKER_COMPOSE_PATH}" \
            run --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID_FORCE_TO_ROBOT}" --detach clock_sync \
            ros2 run clock_sync_cpp bounce_clocks --ros-args -r __name:=bounce_clocks2 -r /bounce_input:=/bounce -r /bounce_output:=/bounce -p nr_bounces:=100

        # Start robot-specific launch file in the foreground
        echo ""
        echo "Starting robot launch"
        docker compose -f "${ROBOT_LAUNCH_DOCKER_COMPOSE_PATH}" \
            run --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID_EXPERIMENT}" --rm nakama_handeye_robot \
            ros2 launch touch_detection_bringup bringup.launch.py robot_ip:=172.16.0.2 load_gripper:=false use_rviz:=false use_plotjuggler:=false

        # Clean up after experiment is stopped
        execute_clean_exp
        ;;

    # Vision PC is disabled as TD experiment only needs 2 PCs
    # "${VISION_PC}")
    #     echo ""
    #     echo "Starting clock monitoring nodes in the background..."

    #     # Vision PC to Robot PC (first node)
    #     docker compose -f "${CLOCK_SYNC_DOCKER_COMPOSE_PATH}" \
    #         run --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VISION_TO_ROBOT}" --detach clock_sync \
    #         ros2 run clock_sync_cpp bounce_clocks --ros-args -r __name:=bounce_clocks1 -r /bounce_input:=/bounce -r /bounce_output:=/bounce -p nr_bounces:=100

    #     # Force PC to Vision PC (second node)
    #     docker compose -f "${CLOCK_SYNC_DOCKER_COMPOSE_PATH}" \
    #         run --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID_FORCE_TO_VISION}" --detach clock_sync \
    #         ros2 run clock_sync_cpp bounce_clocks --ros-args -r __name:=bounce_clocks2 -r /bounce_input:=/bounce -r /bounce_output:=/bounce -p nr_bounces:=100

    #     # Start clock burst processing nodes
    #     echo ""
    #     echo "Restarting any previously running clock bounce processing nodes"
    #     docker container ls -qa --filter label=process_bounced_clocks | xargs -r docker rm -f

    #     docker compose -f "${CLOCK_SYNC_DOCKER_COMPOSE_PATH}" \
    #         run --volume "${CLOCK_BURST_OUTPUT_FOLDER_PATH}:/output" --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID_FORCE_TO_ROBOT}" --label process_bounced_clocks=true --detach clock_sync \
    #         ros2 run clock_sync_py process_bounced_clocks --ros-args -p filepath:=/output/"${EXPERIMENT_NAME}"_bounce.csv -p auto_process_delay_sec:=0.1 -p auto_process_index:=100 -p only_summary:=true

    #     docker compose -f "${CLOCK_SYNC_DOCKER_COMPOSE_PATH}" \
    #         run --volume "${CLOCK_BURST_OUTPUT_FOLDER_PATH}:/output" --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID_FORCE_TO_VISION}" --label process_bounced_clocks=true --detach clock_sync \
    #         ros2 run clock_sync_py process_bounced_clocks --ros-args -p filepath:=/output/"${EXPERIMENT_NAME}"_bounce.csv -p auto_process_delay_sec:=0.1 -p auto_process_index:=100 -p only_summary:=true

    #     docker compose -f "${CLOCK_SYNC_DOCKER_COMPOSE_PATH}" \
    #         run --volume "${CLOCK_BURST_OUTPUT_FOLDER_PATH}:/output" --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VISION_TO_ROBOT}" --label process_bounced_clocks=true --detach clock_sync \
    #         ros2 run clock_sync_py process_bounced_clocks --ros-args -p filepath:=/output/"${EXPERIMENT_NAME}"_bounce.csv -p auto_process_delay_sec:=0.1 -p auto_process_index:=100 -p only_summary:=true

    #     # Start rosbag recording in de foreground
    #     docker compose -f "${CLOCK_SYNC_DOCKER_COMPOSE_PATH}" \
    #         run --rm --remove-orphans --volume "${BAG_FILE_OUTPUT_FOLDER_PATH}":/output touch_detection_recording \
    #         ros2 bag record /franka_robot_state_broadcaster/robot_state /bota_sensor_node/wrench --output /output/"${EXPERIMENT_NAME}"

    #     # Clean up after experiment is stopped
    #     execute_clean_exp
    #     ;;

    "${FORCE_PC}")
        echo ""
        echo "Starting clock monitoring nodes in the background..."

        # Force PC to Robot PC (first node)
        docker compose -f "${CLOCK_SYNC_DOCKER_COMPOSE_PATH}" \
            run --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID_FORCE_TO_ROBOT}" --detach clock_sync \
            ros2 run clock_sync_cpp bounce_clocks --ros-args -r __name:=bounce_clocks1 -r /bounce_input:=/bounce -r /bounce_output:=/bounce -p nr_bounces:=100

        # # Force PC to Vision PC (first node)
        # docker compose -f "${CLOCK_SYNC_DOCKER_COMPOSE_PATH}" \
        #     run --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID_FORCE_TO_VISION}" --detach clock_sync \
        #     ros2 run clock_sync_cpp bounce_clocks --ros-args -r __name:=bounce_clocks1 -r /bounce_input:=/bounce -r /bounce_output:=/bounce -p nr_bounces:=100

        # Start force sensor in the foreground
        docker compose -f "${FORCE_SENSOR_DOCKER_COMPOSE_PATH}" \
            run --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID_EXPERIMENT}" --rm ethercat_senseone_container \
            bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && source install/setup.bash  && ros2 run senseone_eth_node senseone_eth_node --ros-args -p eth_interface_name:=enx00e04c680bc3 -p sinc_length:=64"

        # Clean up after experiment is stopped
        execute_clean_exp
        ;;

    *)
        echo ""
        echo "ERROR: No experiment setup is configured for PC with name '${pc_name}'."
        echo "       Did you mean to call '$0 --pc ${pc_name} --setup-sync' instead?."
        exit 1
        ;;
    esac

}

# -----------------------------------------------------
# Start experiment repetition
# -----------------------------------------------------

# Define the function to start a repetition
execute_start_rep() {

    # Check PC name and start nodes specific to that PC
    case "${pc_name}" in
    "${DATA_COLLECTION_PC}")
        if [[ -d "${CLOCK_BURST_OUTPUT_FOLDER_PATH}" && -n "$(find "${CLOCK_BURST_OUTPUT_FOLDER_PATH}" -mindepth 1 -print -quit)" || -d "${BAG_FILE_OUTPUT_FOLDER_PATH}/${EXPERIMENT_NAME}" ]]; then
            echo ""
            echo "ERROR: This repetition already has data and will not be overwritten. This script will exit."
            echo "       The following folder is nonempty: '${CLOCK_BURST_OUTPUT_FOLDER_PATH}'"
            echo "       OR a bagfile already exists in '${BAG_FILE_OUTPUT_FOLDER_PATH}/${EXPERIMENT_NAME}'."
            echo "       If you want to remove the data, use the '--empty-rep' flag."
            exit 1
        fi

        mkdir -p "${CLOCK_BURST_OUTPUT_FOLDER_PATH}"

        if [[ "${demo}" == "false" ]]; then
            # Clean up any previously running nodes
            echo ""
            echo "Stopping any previously running clock bounce trigger nodes"
            docker container ls -qa --filter label=clock_bounce_trigger | xargs -r docker rm -f

            # Force PC to Robot PC
            docker compose -f "${CLOCK_SYNC_DOCKER_COMPOSE_PATH}" \
                run --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID_FORCE_TO_ROBOT}" --label clock_bounce_trigger=true --detach clock_sync \
                ros2 service call /bounce_clocks1/start_bounce std_srvs/srv/Trigger --rate 1.0
            sleep 0.2

            # Vision PC is disabled as TD experiment only needs 2 PCs
            # # Force PC to Vision PC
            # docker compose -f "${CLOCK_SYNC_DOCKER_COMPOSE_PATH}" \
            #     run --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID_FORCE_TO_VISION}" --label clock_bounce_trigger=true --detach clock_sync \
            #     ros2 service call /bounce_clocks1/start_bounce std_srvs/srv/Trigger --rate 1.0
            # sleep 0.2

            # Vision PC is disabled as TD experiment only needs 2 PCs
            # # Vision PC to Robot PC
            # docker compose -f "${CLOCK_SYNC_DOCKER_COMPOSE_PATH}" \
            #     run --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VISION_TO_ROBOT}" --label clock_bounce_trigger=true --detach clock_sync \
            #     ros2 service call /bounce_clocks1/start_bounce std_srvs/srv/Trigger --rate 1.0
            # sleep 0.2
        fi

        # Start rosbag recording in de background
        docker compose -f "${BAGFILE_RECORDING_DOCKER_COMPOSE_PATH}" \
            run --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID_EXPERIMENT}" --volume "${BAG_FILE_OUTPUT_FOLDER_PATH}":/output --label bag_recording=true --detach touch_detection_recording \
            ros2 bag record /franka_robot_state_broadcaster/robot_state /bota_sensor_node/wrench --output /output/"${EXPERIMENT_NAME}"



        # Clean up clock bounce trigger nodes and bag recording container
        docker container ls -qa --filter label=clock_bounce_trigger | xargs -r docker rm -f
        docker container ls -qa --filter label=bag_recording | xargs -r docker rm -f
        ;;

    *)
        echo ""
        echo "ERROR: PC with name '${pc_name}' does not need to start a specific repetition."
        echo "       Did you mean to call '$0 --pc ${pc_name} --setup-exp' instead?."
        exit 1
        ;;
    esac

    # Repetition is done
    echo ""
    echo "DONE: Repetition ${rep_nr} of experiment ${exp_nr} (sync type: ${sync_type}, offset: ${offset})."
}

# -----------------------------------------------------
# Empty repetition
# -----------------------------------------------------

execute_empty_rep() {
    # Check if the clock burst output folder exists and contains items, OR the bag file output folder for this experiment exists
    if [[ (-d "${CLOCK_BURST_OUTPUT_FOLDER_PATH}" && -n "$(find "${CLOCK_BURST_OUTPUT_FOLDER_PATH}" -mindepth 1 -print -quit)") || -d "${BAG_FILE_OUTPUT_FOLDER_PATH}/${EXPERIMENT_NAME}" ]]; then

        # Check if we should force remove, i.e. use sudo
        if [[ "${force_empty_rep}" == "true" ]]; then
            echo ""
            echo "Force removing the repetition requires sudo privileges."
            echo "You will be prompted for your password."
            sudo -v || {
                echo "Failed to authenticate with sudo"
                exit 1
            }

            # Using sudo, remove all items inside the clock sync folder but not the output folder itself
            if ! sudo find "${CLOCK_BURST_OUTPUT_FOLDER_PATH}" -mindepth 1 -delete; then
                echo ""
                echo "ERROR: Failed to delete contents of '${CLOCK_BURST_OUTPUT_FOLDER_PATH}'."
                exit 1
            fi

            # Also remove bag file
            if ! sudo find "${BAG_FILE_OUTPUT_FOLDER_PATH}/${EXPERIMENT_NAME}" -mindepth 0 -delete; then
                echo ""
                echo "ERROR: Failed to delete '${BAG_FILE_OUTPUT_FOLDER_PATH}/${EXPERIMENT_NAME}'."
                exit 1
            fi

        else
            # No force remove
            # Remove all items inside the folder but not the output folder itself
            if ! find "${CLOCK_BURST_OUTPUT_FOLDER_PATH}" -mindepth 1 -delete; then
                echo ""
                echo "ERROR: Failed to delete contents of '${CLOCK_BURST_OUTPUT_FOLDER_PATH}'."
                echo "       Try calling with '--force-empty-rep' instead."
                exit 1
            fi

            # Also remove bag file
            if ! find "${BAG_FILE_OUTPUT_FOLDER_PATH}/${EXPERIMENT_NAME}" -mindepth 0 -delete; then
                echo ""
                echo "ERROR: Failed to delete '${BAG_FILE_OUTPUT_FOLDER_PATH}/${EXPERIMENT_NAME}'."
                exit 1
            fi
        fi
        echo ""
        echo "Repetition data is REMOVED, deleted everything in '${CLOCK_BURST_OUTPUT_FOLDER_PATH}' and '${BAG_FILE_OUTPUT_FOLDER_PATH}/${EXPERIMENT_NAME}'."

    else
        # No repetition files found, so we don't need to do anything
        echo ""
        echo "Repetition not emptied: no files found in '${CLOCK_BURST_OUTPUT_FOLDER_PATH}' or '${BAG_FILE_OUTPUT_FOLDER_PATH}/${EXPERIMENT_NAME}'."

    fi
}

# -----------------------------------------------------
# Clean up experiment
# -----------------------------------------------------

# Define the function to clean experiments
execute_clean_exp() {
    echo ""
    echo "Cleaning up clock monitoring nodes for experiment ${exp_nr}."

    # Stop all nodes related to clock monitoring
    docker container ls -qa --filter name=clock_synchronization* | xargs -r docker rm -f
    docker container ls -qa --filter label=clock_bounce_trigger | xargs -r docker rm -f

    echo ""
    echo "DONE: Clock monitoring is cleaned up."
}

# -----------------------------------------------------
# Clean up clock sync
# -----------------------------------------------------

# Define the function to clean synchronizatoin
execute_clean_sync() {
    echo ""
    echo "Cleaning up clock synchronization"

    # Ask for sudo password once upfront
    echo "Restoring synchronization requires sudo privileges."
    echo "You will be prompted for your password."
    sudo -v || {
        echo "Failed to authenticate with sudo"
        exit 1
    }

    # Execute the synchronization setup script
    echo ""
    echo "Calling synchronization setup script with the following signature: 'sudo ${SETUP_SYNC_SCRIPT_PATH} --restore-timesyncd'"
    sudo "${SETUP_SYNC_SCRIPT_PATH}" --restore-timesyncd

    echo ""
    echo "DONE: Clock synchronization is restored to the default."
}

# -----------------------------------------------------
# Execution functions
# -----------------------------------------------------

if [[ "${setup_sync}" = "true" ]]; then
    execute_setup_sync
    exit 0
fi

if [[ "${setup_exp}" = "true" ]]; then
    execute_setup_exp
    exit 0
fi

if [[ "${start_rep}" = "true" ]]; then
    execute_start_rep
    exit 0
fi

if [[ "${empty_rep}" = "true" ]]; then
    execute_empty_rep
    exit 0
fi

if [[ "${clean_exp}" = "true" ]]; then
    execute_clean_exp
    exit 0
fi

if [[ "${clean_sync}" = "true" ]]; then
    execute_clean_sync
    exit 0
fi
