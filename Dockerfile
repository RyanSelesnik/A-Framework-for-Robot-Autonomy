FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# ══════════════════════════════════════════════════════════════════════════════
# SLOW / STABLE layers (rarely change, cached)
# ══════════════════════════════════════════════════════════════════════════════

# ── System packages (dev tools + build deps) ─────────────────────────────────
RUN apt-get update && apt-get install -y \
    locales git curl wget unzip zip \
    build-essential cmake pkg-config \
    vim tmux zsh fzf ripgrep tree htop \
    net-tools iputils-ping dnsutils openssh-client \
    python3-pip python3-venv \
    nodejs npm \
    jq less man-db sudo ca-certificates gnupg \
    software-properties-common \
    # C++ libs for A-Framework
    libopenblas-dev libeigen3-dev libarmadillo-dev \
    libgflags-dev libgoogle-glog-dev libgtest-dev \
    liblapack-dev libsuitesparse-dev libcxsparse3 \
    # VNC for browser-based RViz
    xvfb x11vnc novnc \
    && locale-gen en_US.UTF-8 \
    && add-apt-repository -y ppa:neovim-ppa/unstable \
    && apt-get update && apt-get install -y neovim \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ── Ceres Solver (required by VINS-Fusion) ───────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends libceres-dev \
    && rm -rf /var/lib/apt/lists/*

# ── ROS packages ─────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-catkin \
    ros-noetic-message-generation ros-noetic-message-runtime \
    ros-noetic-nav-msgs ros-noetic-geometry-msgs \
    ros-noetic-sensor-msgs ros-noetic-std-msgs \
    ros-noetic-visualization-msgs ros-noetic-tf ros-noetic-tf2-ros \
    ros-noetic-nodelet ros-noetic-cv-bridge ros-noetic-image-transport \
    ros-noetic-pcl-ros ros-noetic-pcl-conversions \
    ros-noetic-rviz \
    ros-noetic-ddynamic-reconfigure \
    ros-noetic-mavros \
    ros-noetic-roslint \
    && rm -rf /var/lib/apt/lists/*

# ── Python deps (CPU-only PyTorch — slowest layer) ──────────────────────────
COPY src/consdred_smpc_ros/requirements.txt /tmp/
RUN pip3 install --no-cache-dir 'typing-extensions<4.13' \
    && pip3 install --no-cache-dir torch==2.4.1 --index-url https://download.pytorch.org/whl/cpu \
    && grep -v '^torch' /tmp/requirements.txt | pip3 install --no-cache-dir -r /dev/stdin

# ══════════════════════════════════════════════════════════════════════════════
# FAST / CHANGING layers (user config, entrypoint — quick to rebuild)
# ══════════════════════════════════════════════════════════════════════════════

# ── User setup ───────────────────────────────────────────────────────────────
RUN useradd -m -s /bin/zsh -G sudo -u 1005 ryan && \
    echo "ryan ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ryan
WORKDIR /home/ryan

# Oh My Zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended

# LazyVim
RUN git clone https://github.com/LazyVim/starter ~/.config/nvim && \
    rm -rf ~/.config/nvim/.git

# Claude Code
RUN curl -fsSL https://claude.ai/install.sh | bash
RUN echo 'export PATH="$HOME/.claude/bin:$PATH"' >> ~/.zshrc

# ── Catkin workspace ─────────────────────────────────────────────────────────
USER root
RUN mkdir -p /catkin_ws/src && chown -R ryan:ryan /catkin_ws
USER ryan

# ROS env in zsh
RUN echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc \
    && echo '[ -f /catkin_ws/devel/setup.zsh ] && source /catkin_ws/devel/setup.zsh' >> ~/.zshrc

# ── Entrypoint (changes here only rebuild this layer) ────────────────────────
COPY --chown=ryan:ryan entrypoint.sh /entrypoint.sh
USER root
RUN chmod +x /entrypoint.sh
USER ryan

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/zsh"]
