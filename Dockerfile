FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# ── Dev tools (mirroring golden base, adapted for Ubuntu 20.04) ──────────────
RUN apt-get update && apt-get install -y \
    locales git curl wget unzip zip \
    build-essential cmake pkg-config \
    vim tmux zsh fzf ripgrep tree htop \
    net-tools iputils-ping dnsutils openssh-client \
    python3-pip python3-venv \
    nodejs npm \
    jq less man-db sudo ca-certificates gnupg \
    software-properties-common \
    && locale-gen en_US.UTF-8 \
    && add-apt-repository -y ppa:neovim-ppa/unstable \
    && apt-get update && apt-get install -y neovim \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ── C++ libs for A-Framework compilation ─────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopenblas-dev libeigen3-dev libarmadillo-dev \
    # Ceres Solver deps (required by VINS-Fusion)
    libgflags-dev libgoogle-glog-dev libgtest-dev \
    liblapack-dev libsuitesparse-dev libcxsparse3 \
    && rm -rf /var/lib/apt/lists/*

# Ceres Solver (required by VINS-Fusion in A-Framework)
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

# ── Python deps (CPU-only PyTorch 2.4.1 — last version for Python 3.8) ──────
COPY src/consdred_smpc_ros/requirements.txt /tmp/
RUN pip3 install --no-cache-dir 'typing-extensions<4.13' \
    && pip3 install --no-cache-dir torch==2.4.1 --index-url https://download.pytorch.org/whl/cpu \
    && grep -v '^torch' /tmp/requirements.txt | pip3 install --no-cache-dir -r /dev/stdin

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

# ── Entrypoint ───────────────────────────────────────────────────────────────
COPY --chown=ryan:ryan entrypoint.sh /entrypoint.sh
USER root
RUN chmod +x /entrypoint.sh
USER ryan

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/zsh"]
