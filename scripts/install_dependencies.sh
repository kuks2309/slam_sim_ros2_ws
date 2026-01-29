#!/bin/bash
#
# SLAM Simulation ROS2 Workspace - 의존성 설치 스크립트
#
# 사용법:
#   chmod +x install_dependencies.sh
#   ./install_dependencies.sh          # 일반 설치
#   ./install_dependencies.sh --check  # 상태 분석만
#   ./install_dependencies.sh --clean  # 제거 후 재설치
#   ./install_dependencies.sh --fix    # 문제 있는 패키지만 재설치
#

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 패키지 그룹 정의
X11_PACKAGES=(
    "x11-apps"
    "x11-utils"
    "x11-xserver-utils"
    "mesa-utils"
    "libgl1-mesa-glx"
    "libgl1-mesa-dri"
)

SLAM_PACKAGES=(
    "ros-humble-slam-toolbox"
    "ros-humble-cartographer"
    "ros-humble-cartographer-ros"
    "ros-humble-rtabmap"
    "ros-humble-rtabmap-ros"
)

NAV_PACKAGES=(
    "ros-humble-nav2-map-server"
    "ros-humble-nav2-lifecycle-manager"
)

GAZEBO_PACKAGES=(
    "ros-humble-ros-gz-sim"
    "ros-humble-ros-gz-bridge"
    "ros-humble-ros-gz-image"
)

VIZ_PACKAGES=(
    "ros-humble-rviz2"
    "ros-humble-rqt-robot-steering"
    "ros-humble-rqt-tf-tree"
    "ros-humble-rqt-graph"
    "ros-humble-tf2-ros"
    "ros-humble-tf2-tools"
)

PYTHON_PACKAGES=(
    "python3-pyqt5"
)

# 모든 패키지 통합
ALL_PACKAGES=(
    "${X11_PACKAGES[@]}"
    "${SLAM_PACKAGES[@]}"
    "${NAV_PACKAGES[@]}"
    "${GAZEBO_PACKAGES[@]}"
    "${VIZ_PACKAGES[@]}"
    "${PYTHON_PACKAGES[@]}"
)

# 패키지 상태 확인 함수
check_package_status() {
    local pkg=$1
    local status=$(dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null || echo "not-installed")

    if [[ "$status" == "install ok installed" ]]; then
        echo "installed"
    elif [[ "$status" == *"deinstall"* ]] || [[ "$status" == *"config-files"* ]]; then
        echo "broken"
    else
        echo "not-installed"
    fi
}

# 패키지 버전 확인 함수
get_package_version() {
    local pkg=$1
    dpkg-query -W -f='${Version}' "$pkg" 2>/dev/null || echo "N/A"
}

# WSL 환경 감지 함수
is_wsl() {
    grep -qi "Microsoft\|WSL" /proc/version 2>/dev/null
}

# WSLg 지원 여부 확인 (Windows 11+)
has_wslg() {
    [[ -d "/mnt/wslg" ]] || [[ -S "/tmp/.X11-unix/X0" ]]
}

# DISPLAY 환경 설정 상태 확인
check_display_config() {
    local issues=()

    if is_wsl; then
        # DISPLAY 설정 확인
        if ! grep -q "export DISPLAY=" ~/.bashrc 2>/dev/null; then
            issues+=("DISPLAY 미설정")
        fi

        # LIBGL_ALWAYS_SOFTWARE 확인
        if ! grep -q "LIBGL_ALWAYS_SOFTWARE" ~/.bashrc 2>/dev/null; then
            issues+=("LIBGL_ALWAYS_SOFTWARE 미설정")
        fi

        # WAYLAND_DISPLAY 확인 (WSLg)
        if has_wslg && ! grep -q "WAYLAND_DISPLAY" ~/.bashrc 2>/dev/null; then
            issues+=("WAYLAND_DISPLAY 미설정 (WSLg)")
        fi
    fi

    echo "${issues[@]}"
}

# WSL 환경 상태 분석
analyze_wsl_environment() {
    echo ""
    if ! is_wsl; then
        echo -e "${BLUE}[i]${NC} WSL 환경: 아님 (네이티브 Linux)"
        return 0
    fi

    echo -e "${BLUE}[i]${NC} WSL 환경: 감지됨"

    local has_issues=false

    # WSLg 확인
    if has_wslg; then
        echo -e "    ${GREEN}✓${NC} WSLg 지원: 활성화됨"
    else
        echo -e "    ${YELLOW}!${NC} WSLg 지원: 비활성화 (외부 X서버 필요)"
    fi

    # DISPLAY 설정 확인
    if grep -q "export DISPLAY=" ~/.bashrc 2>/dev/null; then
        local display_val=$(grep "export DISPLAY=" ~/.bashrc | tail -1 | cut -d'=' -f2)
        echo -e "    ${GREEN}✓${NC} DISPLAY 설정됨: $display_val"
    else
        echo -e "    ${RED}✗${NC} DISPLAY 미설정"
        has_issues=true
    fi

    # LIBGL_ALWAYS_SOFTWARE 확인
    if grep -q "LIBGL_ALWAYS_SOFTWARE" ~/.bashrc 2>/dev/null; then
        echo -e "    ${GREEN}✓${NC} LIBGL_ALWAYS_SOFTWARE 설정됨"
    else
        echo -e "    ${YELLOW}!${NC} LIBGL_ALWAYS_SOFTWARE 미설정 (Gazebo 렌더링 문제 가능)"
        has_issues=true
    fi

    # 현재 세션 DISPLAY 확인
    if [[ -n "$DISPLAY" ]]; then
        echo -e "    ${GREEN}✓${NC} 현재 세션 DISPLAY: $DISPLAY"
    else
        echo -e "    ${YELLOW}!${NC} 현재 세션 DISPLAY 없음 (새 터미널 필요)"
    fi

    if $has_issues; then
        echo ""
        echo -e "    ${YELLOW}[권장]${NC} --fix 옵션으로 WSL 환경 설정을 복구하세요."
        return 1
    fi

    return 0
}

# 패키지 그룹 상태 분석 함수
analyze_package_group() {
    local group_name=$1
    shift
    local packages=("$@")

    local installed=0
    local broken=0
    local missing=0
    local broken_list=()
    local missing_list=()

    for pkg in "${packages[@]}"; do
        local status=$(check_package_status "$pkg")
        case $status in
            "installed")
                ((installed++))
                ;;
            "broken")
                ((broken++))
                broken_list+=("$pkg")
                ;;
            *)
                ((missing++))
                missing_list+=("$pkg")
                ;;
        esac
    done

    local total=${#packages[@]}

    echo ""
    if [[ $installed -eq $total ]]; then
        echo -e "${GREEN}[✓]${NC} $group_name: 모두 설치됨 ($installed/$total)"
    elif [[ $broken -gt 0 ]]; then
        echo -e "${RED}[✗]${NC} $group_name: 문제 발견 (설치: $installed, 손상: $broken, 미설치: $missing)"
        for pkg in "${broken_list[@]}"; do
            echo -e "    ${RED}손상:${NC} $pkg"
        done
        for pkg in "${missing_list[@]}"; do
            echo -e "    ${YELLOW}미설치:${NC} $pkg"
        done
    elif [[ $missing -gt 0 ]]; then
        echo -e "${YELLOW}[!]${NC} $group_name: 일부 미설치 (설치: $installed, 미설치: $missing)"
        for pkg in "${missing_list[@]}"; do
            echo -e "    ${YELLOW}미설치:${NC} $pkg"
        done
    fi

    # 반환값: "installed broken missing"
    echo "$installed $broken $missing" > /tmp/pkg_status_$$
}

# 전체 상태 분석 함수
analyze_all() {
    echo "=============================================="
    echo " 의존성 상태 분석"
    echo "=============================================="

    local total_installed=0
    local total_broken=0
    local total_missing=0

    analyze_package_group "X11/디스플레이" "${X11_PACKAGES[@]}"
    read i b m < /tmp/pkg_status_$$
    total_installed=$((total_installed + i))
    total_broken=$((total_broken + b))
    total_missing=$((total_missing + m))

    analyze_package_group "SLAM" "${SLAM_PACKAGES[@]}"
    read i b m < /tmp/pkg_status_$$
    total_installed=$((total_installed + i))
    total_broken=$((total_broken + b))
    total_missing=$((total_missing + m))

    analyze_package_group "Navigation" "${NAV_PACKAGES[@]}"
    read i b m < /tmp/pkg_status_$$
    total_installed=$((total_installed + i))
    total_broken=$((total_broken + b))
    total_missing=$((total_missing + m))

    analyze_package_group "Gazebo" "${GAZEBO_PACKAGES[@]}"
    read i b m < /tmp/pkg_status_$$
    total_installed=$((total_installed + i))
    total_broken=$((total_broken + b))
    total_missing=$((total_missing + m))

    analyze_package_group "시각화/도구" "${VIZ_PACKAGES[@]}"
    read i b m < /tmp/pkg_status_$$
    total_installed=$((total_installed + i))
    total_broken=$((total_broken + b))
    total_missing=$((total_missing + m))

    analyze_package_group "Python" "${PYTHON_PACKAGES[@]}"
    read i b m < /tmp/pkg_status_$$
    total_installed=$((total_installed + i))
    total_broken=$((total_broken + b))
    total_missing=$((total_missing + m))

    rm -f /tmp/pkg_status_$$

    # WSL 환경 분석
    local wsl_issues=false
    analyze_wsl_environment || wsl_issues=true

    local total=$((total_installed + total_broken + total_missing))

    echo ""
    echo "=============================================="
    echo " 요약"
    echo "=============================================="
    echo -e "  전체 패키지: $total"
    echo -e "  ${GREEN}설치됨:${NC} $total_installed"
    echo -e "  ${RED}손상:${NC} $total_broken"
    echo -e "  ${YELLOW}미설치:${NC} $total_missing"
    echo ""

    if [[ $total_broken -gt 0 ]]; then
        echo -e "${RED}[권장]${NC} 손상된 패키지가 있습니다. --fix 또는 --clean 옵션을 사용하세요."
        return 2
    elif [[ $total_missing -gt 0 ]]; then
        echo -e "${YELLOW}[권장]${NC} 미설치 패키지가 있습니다. 스크립트를 실행하여 설치하세요."
        return 1
    elif $wsl_issues; then
        echo -e "${YELLOW}[권장]${NC} WSL 환경 설정이 필요합니다. --fix 옵션을 사용하세요."
        return 1
    else
        echo -e "${GREEN}[완료]${NC} 모든 의존성이 올바르게 설치되어 있습니다."
        return 0
    fi
}

# 패키지 제거 함수
remove_packages() {
    local packages=("$@")
    local to_remove=()

    for pkg in "${packages[@]}"; do
        local status=$(check_package_status "$pkg")
        if [[ "$status" != "not-installed" ]]; then
            to_remove+=("$pkg")
        fi
    done

    if [[ ${#to_remove[@]} -gt 0 ]]; then
        echo "제거할 패키지: ${to_remove[*]}"
        sudo apt remove -y "${to_remove[@]}" || true
        sudo apt autoremove -y || true
    fi
}

# 손상된 패키지만 재설치 함수
fix_broken_packages() {
    echo "=============================================="
    echo " 손상된 패키지 복구"
    echo "=============================================="

    local broken_packages=()
    local missing_packages=()

    for pkg in "${ALL_PACKAGES[@]}"; do
        local status=$(check_package_status "$pkg")
        case $status in
            "broken")
                broken_packages+=("$pkg")
                ;;
            "not-installed")
                missing_packages+=("$pkg")
                ;;
        esac
    done

    if [[ ${#broken_packages[@]} -gt 0 ]]; then
        echo ""
        echo -e "${YELLOW}손상된 패키지 재설치 중...${NC}"
        sudo apt remove -y "${broken_packages[@]}" || true
        sudo dpkg --configure -a || true
        sudo apt install -y "${broken_packages[@]}"
    fi

    if [[ ${#missing_packages[@]} -gt 0 ]]; then
        echo ""
        echo -e "${YELLOW}미설치 패키지 설치 중...${NC}"
        sudo apt install -y "${missing_packages[@]}"
    fi

    if [[ ${#broken_packages[@]} -eq 0 ]] && [[ ${#missing_packages[@]} -eq 0 ]]; then
        echo -e "${GREEN}[✓]${NC} 복구할 패키지가 없습니다."
    else
        echo ""
        echo -e "${GREEN}[✓]${NC} 패키지 복구 완료"
    fi

    # WSL 환경 설정 복구
    if is_wsl; then
        echo ""
        echo "=============================================="
        echo " WSL 환경 설정 복구"
        echo "=============================================="
        configure_wsl_display
    fi
}

# WSL DISPLAY 설정 함수
configure_wsl_display() {
    local changed=false

    # WSLg 지원 여부에 따라 DISPLAY 설정
    if has_wslg; then
        echo -e "${BLUE}[i]${NC} WSLg 환경 감지됨 (Windows 11+)"

        # DISPLAY 설정 (WSLg)
        if ! grep -q "export DISPLAY=" ~/.bashrc 2>/dev/null; then
            echo "" >> ~/.bashrc
            echo "# WSL2 Display 설정 (WSLg)" >> ~/.bashrc
            echo "export DISPLAY=:0" >> ~/.bashrc
            echo -e "${GREEN}[✓]${NC} DISPLAY=:0 추가됨"
            changed=true
        else
            echo -e "${GREEN}[✓]${NC} DISPLAY 이미 설정됨"
        fi

        # WAYLAND_DISPLAY 설정
        if ! grep -q "WAYLAND_DISPLAY" ~/.bashrc 2>/dev/null; then
            echo "export WAYLAND_DISPLAY=wayland-0" >> ~/.bashrc
            echo -e "${GREEN}[✓]${NC} WAYLAND_DISPLAY=wayland-0 추가됨"
            changed=true
        fi
    else
        echo -e "${BLUE}[i]${NC} 외부 X서버 모드 (VcXsrv/X410 등 필요)"

        # Windows 호스트 IP 가져오기
        local win_host=$(cat /etc/resolv.conf 2>/dev/null | grep nameserver | awk '{print $2}' | head -1)

        if [[ -n "$win_host" ]]; then
            # DISPLAY 설정 (외부 X서버)
            if ! grep -q "export DISPLAY=" ~/.bashrc 2>/dev/null; then
                echo "" >> ~/.bashrc
                echo "# WSL2 Display 설정 (외부 X서버)" >> ~/.bashrc
                echo "export DISPLAY=${win_host}:0.0" >> ~/.bashrc
                echo -e "${GREEN}[✓]${NC} DISPLAY=${win_host}:0.0 추가됨"
                changed=true
            else
                # 기존 설정이 있으면 업데이트 여부 확인
                local current_display=$(grep "export DISPLAY=" ~/.bashrc | tail -1 | cut -d'=' -f2)
                echo -e "${YELLOW}[i]${NC} 현재 DISPLAY 설정: $current_display"
                echo -e "${YELLOW}[i]${NC} 권장 설정: ${win_host}:0.0"
            fi
        else
            echo -e "${RED}[!]${NC} Windows 호스트 IP를 가져올 수 없습니다."
            echo "    수동으로 DISPLAY 설정이 필요합니다."
        fi
    fi

    # LIBGL_ALWAYS_SOFTWARE 설정
    if ! grep -q "LIBGL_ALWAYS_SOFTWARE" ~/.bashrc 2>/dev/null; then
        echo "" >> ~/.bashrc
        echo "# WSL2 Gazebo 렌더링 호환성 (소프트웨어 렌더링)" >> ~/.bashrc
        echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
        echo -e "${GREEN}[✓]${NC} LIBGL_ALWAYS_SOFTWARE=1 추가됨"
        changed=true
    else
        echo -e "${GREEN}[✓]${NC} LIBGL_ALWAYS_SOFTWARE 이미 설정됨"
    fi

    # XDG_RUNTIME_DIR 설정 (일부 애플리케이션에 필요)
    if ! grep -q "XDG_RUNTIME_DIR" ~/.bashrc 2>/dev/null; then
        echo "export XDG_RUNTIME_DIR=/tmp/runtime-\$USER" >> ~/.bashrc
        echo "mkdir -p \$XDG_RUNTIME_DIR 2>/dev/null || true" >> ~/.bashrc
        echo -e "${GREEN}[✓]${NC} XDG_RUNTIME_DIR 추가됨"
        changed=true
    fi

    if $changed; then
        echo ""
        echo -e "${YELLOW}[!]${NC} 새 터미널을 열거나 다음 명령을 실행하세요:"
        echo "    source ~/.bashrc"
    else
        echo ""
        echo -e "${GREEN}[✓]${NC} WSL 환경 설정이 이미 완료되어 있습니다."
    fi
}

# 전체 제거 후 재설치 함수
clean_install() {
    echo "=============================================="
    echo " 전체 제거 후 재설치"
    echo "=============================================="
    echo ""
    echo -e "${YELLOW}경고: 모든 관련 패키지를 제거하고 다시 설치합니다.${NC}"
    echo ""
    read -p "계속하시겠습니까? (y/N): " confirm
    if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
        echo "취소됨."
        exit 0
    fi

    echo ""
    echo "[1/3] 기존 패키지 제거 중..."
    remove_packages "${ALL_PACKAGES[@]}"

    echo ""
    echo "[2/3] APT 정리 중..."
    sudo apt autoremove -y
    sudo apt clean

    echo ""
    echo "[3/3] 재설치 시작..."
    # install_all 함수 호출 (아래에서 정의)
    DO_INSTALL=true
}

# 명령줄 인자 처리
MODE="install"
DO_INSTALL=false

case "${1:-}" in
    --check)
        MODE="check"
        ;;
    --clean)
        MODE="clean"
        ;;
    --fix)
        MODE="fix"
        ;;
    --help|-h)
        echo "사용법: $0 [옵션]"
        echo ""
        echo "옵션:"
        echo "  (없음)    일반 설치"
        echo "  --check   상태 분석만 (설치하지 않음)"
        echo "  --clean   모든 패키지 제거 후 재설치"
        echo "  --fix     손상되거나 미설치된 패키지만 복구"
        echo "  --help    이 도움말 표시"
        exit 0
        ;;
    "")
        DO_INSTALL=true
        ;;
    *)
        echo "알 수 없는 옵션: $1"
        echo "도움말: $0 --help"
        exit 1
        ;;
esac

echo "=============================================="
echo " SLAM Simulation ROS2 의존성 설치"
echo "=============================================="
echo ""

# ROS2 Humble 확인
if [ -z "$ROS_DISTRO" ]; then
    echo "[!] ROS2 환경이 설정되지 않았습니다."
    echo "    먼저 실행: source /opt/ros/humble/setup.bash"
    exit 1
fi

if [ "$ROS_DISTRO" != "humble" ]; then
    echo "[!] ROS2 Humble이 필요합니다. 현재: $ROS_DISTRO"
    exit 1
fi

echo "[✓] ROS2 Humble 확인됨"
echo ""

# 모드별 실행
case "$MODE" in
    "check")
        analyze_all
        exit $?
        ;;
    "fix")
        sudo apt update
        fix_broken_packages
        echo ""
        echo "최종 상태 확인:"
        analyze_all
        exit 0
        ;;
    "clean")
        clean_install
        # DO_INSTALL이 true로 설정되어 아래 설치 과정 진행
        ;;
esac

# 설치 모드가 아니면 종료
if [[ "$DO_INSTALL" != "true" ]]; then
    exit 0
fi

# APT 업데이트
echo "[1/8] APT 저장소 업데이트..."
sudo apt update

# X11 및 디스플레이 패키지
echo ""
echo "[2/8] X11 및 디스플레이 패키지 설치..."
sudo apt install -y \
    x11-apps \
    x11-utils \
    x11-xserver-utils \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri

# SLAM 패키지
echo ""
echo "[3/8] SLAM 패키지 설치..."
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros

# Navigation 패키지
echo ""
echo "[4/8] Navigation 패키지 설치..."
sudo apt install -y \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager

# Gazebo 연동 패키지
echo ""
echo "[5/8] Gazebo 연동 패키지 설치..."
sudo apt install -y \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image

# 시각화 및 도구
echo ""
echo "[6/8] 시각화 및 도구 설치..."
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rqt-robot-steering \
    ros-humble-rqt-tf-tree \
    ros-humble-rqt-graph \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools

# Python 패키지
echo ""
echo "[7/8] Python 패키지 설치..."
sudo apt install -y python3-pyqt5

# WSL2 환경 설정 (DISPLAY 및 렌더링 호환성)
echo ""
echo "[8/8] WSL2 환경 설정..."
if is_wsl; then
    configure_wsl_display

    # 현재 세션에도 적용
    export LIBGL_ALWAYS_SOFTWARE=1
    if has_wslg; then
        export DISPLAY=:0
        export WAYLAND_DISPLAY=wayland-0
    else
        WIN_HOST=$(cat /etc/resolv.conf 2>/dev/null | grep nameserver | awk '{print $2}' | head -1)
        [[ -n "$WIN_HOST" ]] && export DISPLAY="${WIN_HOST}:0.0"
    fi
    export XDG_RUNTIME_DIR="/tmp/runtime-$USER"
    mkdir -p "$XDG_RUNTIME_DIR" 2>/dev/null || true
else
    echo "WSL2 환경이 아님. 건너뜀."
fi

echo ""
echo "=============================================="
echo " 설치 완료 - 상태 확인"
echo "=============================================="
analyze_all || true

echo ""
echo "=============================================="
echo " 설치 완료!"
echo "=============================================="
echo ""
echo "다음 단계:"
echo "  1. 워크스페이스 빌드:"
echo "     cd ~/slam_sim_ros2_ws"
echo "     colcon build --symlink-install"
echo ""
echo "  2. 환경 설정:"
echo "     source install/setup.bash"
echo ""
echo "  3. Gazebo 실행:"
echo "     ros2 launch tm_gazebo gazebo.launch.py"
echo ""
echo "  4. SLAM Manager 실행:"
echo "     ros2 run slam_manager slam_manager"
echo ""
