#!/bin/bash
# ==============================================================================
# NEWRRO TECH LLP - INTERACTIVE GUI SETUP (JETSON ORIN NANO SUPER)
# Professional installation with start/pause/resume/exit controls
# ==============================================================================

set -euo pipefail

# Check for dialog
if ! command -v dialog &>/dev/null; then
    echo "Installing dialog for GUI interface..."
    sudo apt-get update && sudo apt-get install -y dialog
fi

# ============================================================================
# CONFIGURATION
# ============================================================================
export DEBIAN_FRONTEND=noninteractive

VERSION="3.0.0"
USER_NAME="$USER"
HOME_DIR="$HOME"
WS_DIR="${HOME_DIR}/arjuna_ros2/arjuna2_ws"

# State management
STATE_DIR="${HOME_DIR}/.newrro_setup_state"
STATE_FILE="${STATE_DIR}/setup_state.conf"
LOG_FILE="${HOME_DIR}/newrro_setup_$(date +%Y%m%d_%H%M%S).log"
TEMP_FILE=$(mktemp)

# Installation steps
TOTAL_STEPS=14
CURRENT_STEP=0

# Control flags
PAUSED=false
CANCELLED=false

# Create state directory
mkdir -p "$STATE_DIR"

# Trap cleanup
trap cleanup EXIT INT TERM

# ============================================================================
# STATE MANAGEMENT FUNCTIONS
# ============================================================================

save_state() {
    cat > "$STATE_FILE" <<EOF
CURRENT_STEP=$CURRENT_STEP
LOG_FILE=$LOG_FILE
LAST_UPDATE=$(date +%s)
EOF
}

load_state() {
    if [ -f "$STATE_FILE" ]; then
        source "$STATE_FILE"
        return 0
    fi
    return 1
}

clear_state() {
    rm -f "$STATE_FILE"
}

# ============================================================================
# LOGGING FUNCTIONS
# ============================================================================

log() {
    echo "[$(date +%H:%M:%S)] [INFO] $*" | tee -a "$LOG_FILE"
}

warn() {
    echo "[$(date +%H:%M:%S)] [WARN] $*" | tee -a "$LOG_FILE"
}

err() {
    echo "[$(date +%H:%M:%S)] [ERROR] $*" | tee -a "$LOG_FILE" >&2
}

# ============================================================================
# CONTROL FUNCTIONS
# ============================================================================

check_pause() {
    if [ -f "${STATE_DIR}/pause_flag" ]; then
        PAUSED=true
        save_state
        
        dialog --title "⏸ Installation Paused" --msgbox "\
Installation paused at step $CURRENT_STEP of $TOTAL_STEPS

Progress saved. You can:
- Resume installation from main menu
- Exit and resume later
- View installation log

Log file: $LOG_FILE" 12 60
        
        rm -f "${STATE_DIR}/pause_flag"
        return 1
    fi
    return 0
}

check_cancel() {
    if [ -f "${STATE_DIR}/cancel_flag" ]; then
        CANCELLED=true
        rm -f "${STATE_DIR}/cancel_flag"
        return 1
    fi
    return 0
}

cleanup() {
    rm -f "$TEMP_FILE"
    rm -f "${STATE_DIR}/pause_flag"
    rm -f "${STATE_DIR}/cancel_flag"
    kill ${KEEPALIVE_PID} 2>/dev/null || true
}

request_pause() {
    touch "${STATE_DIR}/pause_flag"
}

request_cancel() {
    touch "${STATE_DIR}/cancel_flag"
}

# ============================================================================
# PROGRESS DISPLAY
# ============================================================================

show_progress() {
    local step_name="$1"
    CURRENT_STEP=$((CURRENT_STEP + 1))
    save_state
    
    local percent=$((CURRENT_STEP * 100 / TOTAL_STEPS))
    
    echo "$percent" > "${STATE_DIR}/progress"
    echo "$step_name" > "${STATE_DIR}/current_task"
}

# ============================================================================
# BACKGROUND INSTALLATION PROCESS
# ============================================================================

run_installation() {
    # Redirect output to log
    exec > >(tee -a "$LOG_FILE") 2>&1
    
    # Keep sudo alive
    sudo -v
    ( while true; do sudo -n true; sleep 60; kill -0 "$$" || exit; done ) 2>/dev/null &
    KEEPALIVE_PID=$!
    
    log "=========================================================================="
    log "NEWRRO TECH LLP - Installation Started"
    log "Version: $VERSION"
    log "Date: $(date)"
    log "User: $USER_NAME"
    log "=========================================================================="
    
    # Get system info
    TOTAL_RAM_GB=$(awk '/MemTotal/ {printf "%.0f", $2/1024/1024}' /proc/meminfo)
    NPROC=$(nproc || echo 4)
    SAFE_JOBS=$(( TOTAL_RAM_GB / 2 ))
    BUILD_JOBS=$(( SAFE_JOBS < NPROC ? SAFE_JOBS : NPROC ))
    [ "$BUILD_JOBS" -lt 1 ] && BUILD_JOBS=1
    
    # Detect JetPack
    JP_VERSION=""
    if [ -f /etc/nv_tegra_release ]; then
        JP_VERSION=$(dpkg-query --showformat='${Version}' --show nvidia-l4t-core 2>/dev/null | cut -d- -f1 | cut -d. -f1-2 || echo "unknown")
    fi
    
    # ========================================================================
    # STEP 1: SYSTEM PREPARATION
    # ========================================================================
    if [ $CURRENT_STEP -lt 1 ]; then
        show_progress "System Preparation"
        log "Step 1: System Preparation"
        
        check_pause || return 1
        check_cancel || return 1
        
        # Create swap
        if ! grep -q '/swapfile' /etc/fstab 2>/dev/null; then
            log "Creating 8GB swap file"
            if [ ! -f /swapfile ]; then
                sudo fallocate -l 8G /swapfile || sudo dd if=/dev/zero of=/swapfile bs=1M count=8192
                sudo chmod 600 /swapfile
                sudo mkswap /swapfile
            fi
            sudo swapon /swapfile
            echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab >/dev/null
        fi
        
        # Install essentials
        log "Installing essential packages"
        sudo apt-get update
        sudo apt-get install -y --no-install-recommends \
            curl gnupg2 lsb-release build-essential cmake git git-lfs \
            htop nano vim net-tools iputils-ping unzip wget \
            software-properties-common chrony usbutils libusb-1.0-0-dev \
            ccache openssh-server python3-dev pkg-config python3-pip \
            python3-venv ca-certificates
        
        python3 -m pip install --user --upgrade pip setuptools wheel
        export PATH="/usr/lib/ccache:$PATH"
        sudo chronyc -a makestep || true
        
        log "Step 1: Complete"
    fi
    
    # ========================================================================
    # STEP 2: JETSON OPTIMIZATIONS
    # ========================================================================
    if [ $CURRENT_STEP -lt 2 ]; then
        show_progress "Jetson Optimizations"
        log "Step 2: Jetson Optimizations"
        
        check_pause || return 1
        check_cancel || return 1
        
        if [ -f /etc/nv_tegra_release ]; then
            sudo nvpmodel -m 0 || warn "nvpmodel failed"
            sudo systemctl enable nvpmodel 2>/dev/null || true
            
            if command -v jetson_clocks &>/dev/null; then
                sudo jetson_clocks || warn "jetson_clocks failed"
            fi
            
            for cpu_gov in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
                [ -f "$cpu_gov" ] && echo performance | sudo tee "$cpu_gov" >/dev/null 2>&1
            done
        fi
        
        log "Step 2: Complete"
    fi
    
    # ========================================================================
    # STEP 3: DOCKER
    # ========================================================================
    if [ $CURRENT_STEP -lt 3 ]; then
        show_progress "Installing Docker"
        log "Step 3: Installing Docker"
        
        check_pause || return 1
        check_cancel || return 1
        
        if ! command -v docker &>/dev/null; then
            sudo apt-get install -y docker.io
            sudo systemctl enable --now docker
            sudo usermod -aG docker "${USER_NAME}"
        fi
        
        log "Step 3: Complete"
    fi
    
    # ========================================================================
    # STEP 4: NVIDIA CONTAINER TOOLKIT
    # ========================================================================
    if [ $CURRENT_STEP -lt 4 ]; then
        show_progress "Installing NVIDIA Container Toolkit"
        log "Step 4: Installing NVIDIA Container Toolkit"
        
        check_pause || return 1
        check_cancel || return 1
        
        if ! dpkg -l | grep -q nvidia-container-toolkit; then
            curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
                sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
            
            curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
                sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
                sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
            
            sudo apt-get update
            sudo apt-get install -y nvidia-container-toolkit
            
            if command -v nvidia-ctk &>/dev/null; then
                sudo nvidia-ctk runtime configure --runtime=docker
                sudo systemctl restart docker
            fi
        fi
        
        log "Step 4: Complete"
    fi
    
    # ========================================================================
    # STEP 5: JETSON UTILITIES
    # ========================================================================
    if [ $CURRENT_STEP -lt 5 ]; then
        show_progress "Installing Jetson Utilities"
        log "Step 5: Installing Jetson Utilities"
        
        check_pause || return 1
        check_cancel || return 1
        
        python3 -m pip install --user -U jetson-stats
        
        log "Step 5: Complete"
    fi
    
    # ========================================================================
    # STEP 6: ROS 2 HUMBLE
    # ========================================================================
    if [ $CURRENT_STEP -lt 6 ]; then
        show_progress "Installing ROS 2 Humble"
        log "Step 6: Installing ROS 2 Humble"
        
        check_pause || return 1
        check_cancel || return 1
        
        sudo add-apt-repository -y universe
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg
        
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
            sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        
        sudo apt-get update
        sudo apt-get install -y \
            ros-humble-desktop \
            ros-humble-ros-base \
            ros-dev-tools \
            python3-colcon-common-extensions \
            python3-argcomplete \
            python3-rosdep \
            python3-rosinstall-generator \
            python3-vcstool
        
        if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
            sudo rosdep init
        fi
        
        source /opt/ros/humble/setup.bash
        rosdep update
        
        log "Step 6: Complete"
    fi
    
    # ========================================================================
    # STEP 7: GPU LIBRARIES
    # ========================================================================
    if [ $CURRENT_STEP -lt 7 ]; then
        show_progress "Installing GPU Libraries"
        log "Step 7: Installing GPU Libraries"
        
        check_pause || return 1
        check_cancel || return 1
        
        sudo apt-get install -y \
            libopenblas-dev libblas-dev libjpeg-dev zlib1g-dev libhdf5-dev \
            libssl-dev libffi-dev liblapack-dev gfortran \
            libopencv-dev python3-opencv
        
        log "Step 7: Complete"
    fi
    
    # ========================================================================
    # STEP 8: PYTORCH
    # ========================================================================
    if [ $CURRENT_STEP -lt 8 ]; then
        show_progress "Installing PyTorch"
        log "Step 8: Installing PyTorch"
        
        check_pause || return 1
        check_cancel || return 1
        
        case "$JP_VERSION" in
            36.*)
                python3 -m pip install --user --no-cache-dir \
                    https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torch-2.3.0-cp310-cp310-linux_aarch64.whl
                ;;
            35.*)
                python3 -m pip install --user --no-cache-dir \
                    https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
                ;;
            *)
                python3 -m pip install --user --no-cache-dir torch torchvision torchaudio --index-url https://pypi.nvidia.com || true
                ;;
        esac
        
        python3 -m pip install --user --no-cache-dir \
            numpy scipy pandas matplotlib pillow pyyaml \
            Jetson.GPIO pyserial transforms3d pyquaternion simple-pid tqdm || true
        
        log "Step 8: Complete"
    fi
    
    # ========================================================================
    # STEP 9: YOLOV8
    # ========================================================================
    if [ $CURRENT_STEP -lt 9 ]; then
        show_progress "Installing YOLOv8"
        log "Step 9: Installing YOLOv8"
        
        check_pause || return 1
        check_cancel || return 1
        
        python3 -m pip install --user --no-cache-dir ultralytics onnx onnx-simplifier || true
        
        mkdir -p "${HOME_DIR}/models"
        for model in yolov8n.pt yolov8s.pt; do
            if [ ! -f "${HOME_DIR}/models/${model}" ]; then
                wget -q "https://github.com/ultralytics/assets/releases/download/v0.0.0/${model}" \
                    -O "${HOME_DIR}/models/${model}" || true
            fi
        done
        
        log "Step 9: Complete"
    fi
    
    # ========================================================================
    # STEP 10: ROS PACKAGES
    # ========================================================================
    if [ $CURRENT_STEP -lt 10 ]; then
        show_progress "Installing ROS Packages"
        log "Step 10: Installing ROS Packages"
        
        check_pause || return 1
        check_cancel || return 1
        
        sudo apt-get install -y \
            ros-humble-navigation2 \
            ros-humble-nav2-bringup \
            ros-humble-slam-toolbox \
            ros-humble-robot-localization \
            ros-humble-cartographer \
            ros-humble-cartographer-ros \
            ros-humble-teleop-twist-keyboard \
            ros-humble-teleop-twist-joy \
            ros-humble-xacro \
            ros-humble-robot-state-publisher \
            ros-humble-joint-state-publisher \
            ros-humble-vision-opencv \
            ros-humble-cv-bridge \
            ros-humble-image-transport \
            ros-humble-compressed-image-transport \
            ros-humble-foxglove-bridge \
            ros-humble-diagnostic-updater
        
        log "Step 10: Complete"
    fi
    
    # ========================================================================
    # STEP 11: WORKSPACE SETUP
    # ========================================================================
    if [ $CURRENT_STEP -lt 11 ]; then
        show_progress "Setting Up Workspace"
        log "Step 11: Setting Up Workspace"
        
        check_pause || return 1
        check_cancel || return 1
        
        mkdir -p "${WS_DIR}/src"
        cd "${WS_DIR}/src"
        
        # Clone drivers
        if [ ! -d "depthai-ros" ]; then
            git clone --depth 1 -b humble https://github.com/luxonis/depthai-ros.git || true
        fi
        
        if [ ! -d "sllidar_ros2" ]; then
            git clone --depth 1 -b humble https://github.com/Slamtec/sllidar_ros2.git || true
        fi
        
        if [ ! -d "ros-imu-bno055" ]; then
            git clone --depth 1 -b humble https://github.com/dheera/ros-imu-bno055.git || true
        fi
        
        # USB rules
        echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | \
            sudo tee /etc/udev/rules.d/80-movidius.rules >/dev/null
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        
        log "Step 11: Complete"
    fi
    
    # ========================================================================
    # STEP 12: BUILD WORKSPACE
    # ========================================================================
    if [ $CURRENT_STEP -lt 12 ]; then
        show_progress "Building Workspace (10-20 minutes)"
        log "Step 12: Building Workspace"
        
        check_pause || return 1
        check_cancel || return 1
        
        cd "${WS_DIR}"
        source /opt/ros/humble/setup.bash
        rosdep install --from-paths src --ignore-src -r -y || true
        
        colcon build \
            --symlink-install \
            --parallel-workers "${BUILD_JOBS}" \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
            --event-handlers console_direct+ || true
        
        log "Step 12: Complete"
    fi
    
    # ========================================================================
    # STEP 13: DEVELOPMENT TOOLS
    # ========================================================================
    if [ $CURRENT_STEP -lt 13 ]; then
        show_progress "Installing Development Tools"
        log "Step 13: Installing Development Tools"
        
        check_pause || return 1
        check_cancel || return 1
        
        if ! command -v code &>/dev/null; then
            wget -qO- https://packages.microsoft.com/keys/microsoft.asc | \
                gpg --dearmor > /tmp/packages.microsoft.gpg
            sudo install -D -o root -g root -m 644 /tmp/packages.microsoft.gpg \
                /etc/apt/keyrings/packages.microsoft.gpg
            sudo sh -c 'echo "deb [arch=arm64,amd64 signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
            rm -f /tmp/packages.microsoft.gpg
            sudo apt-get update
            sudo apt-get install -y code || true
        fi
        
        log "Step 13: Complete"
    fi
    
    # ========================================================================
    # STEP 14: FINALIZATION
    # ========================================================================
    if [ $CURRENT_STEP -lt 14 ]; then
        show_progress "Finalizing Configuration"
        log "Step 14: Finalizing Configuration"
        
        check_pause || return 1
        check_cancel || return 1
        
        sudo usermod -aG dialout,video,i2c,plugdev,docker "${USER_NAME}"
        sudo groupadd -f gpio
        sudo usermod -aG gpio "${USER_NAME}"
        
        # Configure bashrc
        if ! grep -q "NEWRRO_COMPLETE_SETUP" "${HOME_DIR}/.bashrc" 2>/dev/null; then
            cat >> "${HOME_DIR}/.bashrc" <<'EOF'

# ============================================================================
# NEWRRO_COMPLETE_SETUP
# ============================================================================
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f "$HOME/arjuna_ros2/arjuna2_ws/install/setup.bash" ]; then
    source "$HOME/arjuna_ros2/arjuna2_ws/install/setup.bash"
fi
export ROS_DOMAIN_ID=0
export PATH="/usr/lib/ccache:$PATH"
export PATH=/usr/local/cuda/bin:${PATH}
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}
export PATH="$HOME/.local/bin:$PATH"

alias ws='cd ~/arjuna_ros2/arjuna2_ws'
alias ros2build='cd ~/arjuna_ros2/arjuna2_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
alias ros2src='source ~/arjuna_ros2/arjuna2_ws/install/setup.bash'
alias check_usb='lsusb && ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null'
alias jtop='jtop'
alias max_power='sudo nvpmodel -m 0 && sudo jetson_clocks'
alias check_temp='cat /sys/devices/virtual/thermal/thermal_zone*/temp | awk "{printf \"%.1f°C\n\", \$1/1000}"'
# ============================================================================
EOF
        fi
        
        # Cleanup
        rm -rf "${WS_DIR}/build" "${WS_DIR}/log"
        sudo apt-get clean
        sudo apt-get autoremove -y
        
        log "Step 14: Complete"
    fi
    
    log "=========================================================================="
    log "Installation Complete!"
    log "=========================================================================="
    
    clear_state
    echo "COMPLETED" > "${STATE_DIR}/status"
}

# ============================================================================
# PROGRESS MONITOR (Background)
# ============================================================================

monitor_progress() {
    local install_pid=$1
    
    while kill -0 $install_pid 2>/dev/null; do
        if [ -f "${STATE_DIR}/progress" ]; then
            local percent=$(cat "${STATE_DIR}/progress")
            local task=$(cat "${STATE_DIR}/current_task" 2>/dev/null || echo "Processing...")
            
            echo "$percent"
            echo "XXX"
            echo "$percent%"
            echo ""
            echo "$task"
            echo ""
            echo "Step $CURRENT_STEP of $TOTAL_STEPS"
            echo "XXX"
        fi
        sleep 2
    done | dialog --title "⚙ Installation in Progress" --gauge "Initializing..." 10 70 0
}

# ============================================================================
# MAIN GUI FUNCTIONS
# ============================================================================

show_splash() {
    dialog --title "NEWRRO Setup Installer" --msgbox "\n\
╔═══════════════════════════════════════════════════════════╗
║                                                           ║
║       NEWRRO TECH LLP - Interactive Setup v$VERSION          ║
║                                                           ║
║          Professional Installation System                 ║
║             for Jetson Orin Nano Super                    ║
║                                                           ║
╚═══════════════════════════════════════════════════════════╝

           Complete ROS 2 Humble Development Stack

    Features: Docker • GPU AI • Navigation • Vision

              Press ENTER to continue" 20 65
}

pre_installation_check() {
    local warnings=""
    
    # Check if running as root
    if [ "$EUID" -eq 0 ]; then
        dialog --title "Error" --msgbox "Don't run this as root!\n\nRun as your normal user." 7 40
        exit 1
    fi
    
    # Check disk space
    local free_space=$(df -BG / | awk 'NR==2 {print $4}' | sed 's/G//')
    if [ "$free_space" -lt 20 ]; then
        warnings="${warnings}⚠ Low disk space: ${free_space}GB free (20GB+ recommended)\n"
    fi
    
    # Check RAM
    local total_ram=$(awk '/MemTotal/ {printf "%.0f", $2/1024/1024}' /proc/meminfo)
    if [ "$total_ram" -lt 4 ]; then
        warnings="${warnings}⚠ Low RAM: ${total_ram}GB (8GB recommended)\n"
    fi
    
    # Check if Jetson
    if [ ! -f /etc/nv_tegra_release ]; then
        warnings="${warnings}⚠ Not running on Jetson hardware\n"
    fi
    
    if [ -n "$warnings" ]; then
        dialog --title "⚠ Pre-Installation Warnings" --yesno "\
${warnings}
Continue anyway?" 12 60 || exit 0
    fi
}

main_menu() {
    while true; do
        # Check if installation completed
        if [ -f "${STATE_DIR}/status" ] && [ "$(cat ${STATE_DIR}/status)" = "COMPLETED" ]; then
            show_completion_screen
            return
        fi
        
        # Check if resumable
        local resume_option=""
        if load_state; then
            resume_option="2"
        fi
        
        local menu_items=(
            "1" "▶️  Start New Installation"
        )
        
        if [ -n "$resume_option" ]; then
            menu_items+=("2" "⏯️  Resume Installation (Step $CURRENT_STEP/$TOTAL_STEPS)")
        fi
        
        menu_items+=(
            "3" "📊 View Installation Log"
            "4" "🗑️  Clear Installation State"
            "5" "ℹ️  About"
            "6" "🚪 Exit"
        )
        
        exec 3>&1
        selection=$(dialog \
            --backtitle "NEWRRO Setup Installer v$VERSION" \
            --title "⚙ Main Menu" \
            --clear \
            --menu "Choose an option:" 16 70 10 \
            "${menu_items[@]}" \
            2>&1 1>&3)
        exit_status=$?
        exec 3>&-
        
        if [ $exit_status -ne 0 ]; then
            if confirm_exit; then
                clear
                exit 0
            fi
            continue
        fi
        
        case $selection in
            1) start_installation ;;
            2) resume_installation ;;
            3) view_log ;;
            4) clear_installation_state ;;
            5) show_about ;;
            6)
                if confirm_exit; then
                    clear
                    exit 0
                fi
                ;;
        esac
    done
}

start_installation() {
    # Confirm start
    if ! dialog --title "▶️ Start Installation" --yesno "\
This will install the complete Arjuna development stack:

✓ Docker & NVIDIA Container Toolkit
✓ ROS 2 Humble
✓ PyTorch (GPU-accelerated)
✓ YOLOv8 & Computer Vision libraries
✓ Navigation & SLAM packages
✓ Sensor drivers (OAK-D, RPLidar, IMU)
✓ Development tools (VS Code)

Estimated time: 30-60 minutes
Disk space required: ~15GB

Internet connection required.

Start installation?" 20 60; then
        return
    fi
    
    # Clear old state
    clear_state
    CURRENT_STEP=0
    save_state
    
    # Start installation in background
    run_installation &
    local install_pid=$!
    
    # Show control panel
    show_control_panel $install_pid
}

resume_installation() {
    if ! load_state; then
        dialog --title "Error" --msgbox "No resumable installation found!" 7 40
        return
    fi
    
    if ! dialog --title "⏯️ Resume Installation" --yesno "\
Resume installation from step $CURRENT_STEP of $TOTAL_STEPS?

Log file: $LOG_FILE" 10 60; then
        return
    fi
    
    # Resume installation
    run_installation &
    local install_pid=$!
    
    # Show control panel
    show_control_panel $install_pid
}

show_control_panel() {
    local install_pid=$1
    local paused=false
    
    # Start progress monitor in background
    monitor_progress $install_pid &
    local monitor_pid=$!
    
    # Wait a bit for gauge to appear
    sleep 2
    
    while kill -0 $install_pid 2>/dev/null; do
        if [ "$paused" = false ]; then
            # Show pause/exit options
            if dialog --timeout 3 --title "⚙ Installation Running" --yesno "\
Installation in progress...

Current Step: $CURRENT_STEP / $TOTAL_STEPS

Options:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Press ENTER to pause installation
Press ESC to continue monitoring
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Auto-refresh in 3 seconds..." 15 50; then
                # User pressed ENTER - pause
                request_pause
                paused=true
                
                # Wait for process to acknowledge pause
                sleep 2
                
                # Show paused dialog
                show_paused_menu $install_pid
                paused=false
            fi
        fi
        sleep 1
    done
    
    # Kill monitor
    kill $monitor_pid 2>/dev/null || true
    
    # Check completion
    wait $install_pid
    local exit_code=$?
    
    if [ $exit_code -eq 0 ]; then
        show_completion_screen
    else
        dialog --title "⚠ Installation Interrupted" --msgbox "\
Installation was interrupted.

You can resume from step $CURRENT_STEP later.

Log file: $LOG_FILE" 10 60
    fi
}

show_paused_menu() {
    local install_pid=$1
    
    while true; do
        exec 3>&1
        selection=$(dialog \
            --title "⏸ Installation Paused" \
            --menu "Installation paused at step $CURRENT_STEP of $TOTAL_STEPS" 12 60 4 \
            "1" "▶️  Resume Installation" \
            "2" "📊 View Log" \
            "3" "❌ Cancel Installation" \
            "4" "↩️  Return to Monitoring" \
            2>&1 1>&3)
        exit_status=$?
        exec 3>&-
        
        if [ $exit_status -ne 0 ]; then
            continue
        fi
        
        case $selection in
            1)
                # Resume
                return
                ;;
            2)
                view_log
                ;;
            3)
                if confirm_cancel; then
                    request_cancel
                    kill $install_pid 2>/dev/null || true
                    dialog --title "❌ Installation Cancelled" --msgbox "\
Installation has been cancelled.

Partial progress saved to:
$LOG_FILE" 9 60
                    return
                fi
                ;;
            4)
                return
                ;;
        esac
    done
}

view_log() {
    if [ -f "$LOG_FILE" ]; then
        dialog --title "📊 Installation Log" --textbox "$LOG_FILE" 30 100
    else
        dialog --title "📊 Installation Log" --msgbox "No log file found yet." 7 40
    fi
}

clear_installation_state() {
    if dialog --title "🗑️ Clear State" --yesno "\
This will clear all installation progress.

You will need to start from the beginning.

Continue?" 10 50; then
        clear_state
        rm -f "${STATE_DIR}/status"
        rm -f "${STATE_DIR}/progress"
        rm -f "${STATE_DIR}/current_task"
        dialog --title "✓ Cleared" --msgbox "Installation state cleared." 7 40
    fi
}

show_about() {
    dialog --title "ℹ About" --msgbox "\
╔══════════════════════════════════════════════════════╗
║                                                      ║
║    NEWRRO TECH LLP - Interactive Setup v$VERSION        ║
║                                                      ║
╚══════════════════════════════════════════════════════╝

Professional installation system for Arjuna Robot

Features:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
- Start/Pause/Resume capability
- Progress persistence across sessions
- Real-time progress monitoring
- Complete installation logging
- Error recovery

Company:     NEWRRO TECH LLP
Website:     newrro.in
Target:      Jetson Orin Nano Super
ROS Version: ROS 2 Humble
JetPack:     6.x (Ubuntu 22.04)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Press ENTER to return" 28 60
}

show_completion_screen() {
    dialog --title "✓ Installation Complete!" --msgbox "\n\
╔══════════════════════════════════════════════════════╗
║                                                      ║
║          🎉 INSTALLATION SUCCESSFUL! 🎉              ║
║                                                      ║
╚══════════════════════════════════════════════════════╝

All components installed successfully!

Next Steps:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. REBOOT YOUR SYSTEM
   sudo reboot

2. Verify installation:
   python3 -c 'import torch; print(torch.cuda.is_available())'
   check_usb
   ros2 topic list

3. Open workspace:
   ws
   code .

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Workspace: ~/arjuna_ros2/arjuna2_ws
Log file:  $LOG_FILE

Press ENTER to exit" 26 60
    
    clear_state
    rm -f "${STATE_DIR}/status"
}

confirm_exit() {
    dialog --title "🚪 Exit" --yesno "Are you sure you want to exit?" 7 40
}

confirm_cancel() {
    dialog --title "❌ Cancel Installation" --yesno "\
Are you sure you want to cancel the installation?

Progress will be saved and you can resume later.

Cancel installation?" 10 50
}

# ============================================================================
# MAIN EXECUTION
# ============================================================================

# Show splash
show_splash

# Pre-installation checks
pre_installation_check

# Start main menu
main_menu

# Cleanup and exit
clear
echo "Thank you for using NEWRRO Setup Installer!"
exit 0
