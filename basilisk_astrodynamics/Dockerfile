# SPDX-License-Identifier: Apache-2.0
FROM osrf/space-ros:jazzy-2026.07.0

USER root

ENV ACADOS_SOURCE_DIR=/opt/acados
ENV ACADOS_INSTALL_DIR=/opt/acados
ENV PATH=/opt/acados/bin:${PATH}
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/acados/lib:/opt/acados/build/lib
ENV VIZARD_HOME=/opt/vizard

RUN apt-get update && apt-get install -y \
    ca-certificates \
    unzip \
    gfortran \
    libblas-dev \
    liblapack-dev \
    libeigen3-dev \
    libzmq3-dev \
    pkg-config \
    python3-matplotlib \
    python3-idna \
    python3-urllib3 \
    cython3 \
    ros-jazzy-rviz2 \
    ros-jazzy-rviz-default-plugins \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p "${VIZARD_HOME}" \
    && curl -fL --retry 3 \
       https://hanspeterschaub.info/bskFiles/Vizard_Linux.zip \
       -o /tmp/Vizard_Linux.zip \
    && unzip -q /tmp/Vizard_Linux.zip -d "${VIZARD_HOME}" \
    && VIZARD_BIN="$(find "${VIZARD_HOME}" -type f -name Vizard.x86_64 -print -quit)" \
    && test -n "${VIZARD_BIN}" \
    && chmod +x "${VIZARD_BIN}" \
    && ln -s "${VIZARD_BIN}" /usr/local/bin/Vizard.x86_64 \
    && rm -f /tmp/Vizard_Linux.zip

RUN PIP_BREAK_SYSTEM_PACKAGES=1 python3 -m pip install \
    --no-cache-dir \
    --no-deps \
    "bsk==2.12.0" \
    casadi \
    orjson \
    requests \
    certifi \
    charset-normalizer \
    future-fstrings \
    colorama \
    tqdm \
    pooch \
    platformdirs

RUN git clone --depth 1 --branch v0.5.1 \
    https://github.com/acados/acados.git /opt/acados \
    && cd /opt/acados \
    && git submodule update --init --recursive \
    && mkdir build \
    && cd build \
    && cmake \
       -DCMAKE_BUILD_TYPE=Release \
       -DCMAKE_INSTALL_PREFIX=/opt/acados \
       -DACADOS_WITH_QPOASES=ON \
       .. \
    && make -j"$(nproc)" \
    && make install

RUN export HOME=/root \
    && export CARGO_HOME=/root/.cargo \
    && export RUSTUP_HOME=/root/.rustup \
    && curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal \
    && git clone --depth 1 \
       https://github.com/acados/tera_renderer.git \
       /tmp/tera_renderer \
    && cd /tmp/tera_renderer \
    && /root/.cargo/bin/cargo build --release \
    && mkdir -p /opt/acados/bin \
    && cp target/release/t_renderer /opt/acados/bin/t_renderer

RUN PIP_BREAK_SYSTEM_PACKAGES=1 python3 -m pip install \
    --no-cache-dir \
    --no-deps \
    -e /opt/acados/interfaces/acados_template

RUN mkdir -p /opt/basilisk_ws/src \
    && git clone --depth 1 --branch v0.1.0 \
       https://github.com/DISCOWER/bsk-msgs.git \
       /opt/basilisk_ws/src/bsk-msgs \
    && git clone --depth 1 --branch v1.2.0 \
       https://github.com/DISCOWER/bsk-ros2-bridge.git \
       /opt/basilisk_ws/src/bsk-ros2-bridge \
    && git clone --depth 1 --branch space-ros-demo \
       https://github.com/DISCOWER/bsk-ros2-mpc.git \
       /opt/basilisk_ws/src/bsk-ros2-mpc

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
      rosdep init; \
    fi \
    && rosdep update \
    && apt-get update \
    && cd /opt/basilisk_ws \
    && PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install \
       --from-paths src \
       --ignore-src \
       --rosdistro jazzy \
       -r \
       -y \
    && rm -rf /var/lib/apt/lists/*

RUN bash -lc "\
    source /opt/ros/jazzy/setup.bash && \
    source \"${SPACEROS_DIR}/setup.bash\" && \
    cd /opt/basilisk_ws && \
    colcon build --symlink-install"

RUN mkdir -p /opt/basilisk_astrodynamics

COPY scenarios/scenarioRosMultiAgentOrbit.py /opt/basilisk_astrodynamics

WORKDIR /opt/basilisk_astrodynamics

CMD ["bash"]
