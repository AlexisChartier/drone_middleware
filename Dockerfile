# -------------------------------------------------------------------------
# 0. Base image
# -------------------------------------------------------------------------
    FROM gitlab.polytech.umontpellier.fr:5050/eii/ros/ros-vanilla:main

    # -------------------------------------------------------------------------
    # 1.  Mettre à jour la clé GPG ROS 2 et nettoyer les anciens fichiers list
    # -------------------------------------------------------------------------
    RUN set -e \
     && apt-get update && apt-get install -y --no-install-recommends curl gnupg lsb-release \
     # — supprimer l’ancien dépôt (clé expirée) s’il existe
     && rm -f /etc/apt/sources.list.d/ros2-latest.list /etc/apt/sources.list.d/ros2.list || true \
     # — récupérer la nouvelle clé
     && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
          | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
     # — ré-créer l’entrée apt unique
     && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
              http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
          > /etc/apt/sources.list.d/ros2.list
    
    # -------------------------------------------------------------------------
    # 2.  Dépendances système
    # -------------------------------------------------------------------------
    RUN apt-get update && apt-get install -y --no-install-recommends \
            build-essential cmake git libasio-dev \
            liboctomap-dev ros-humble-octomap-msgs \
            iw iputils-ping \
        && rm -rf /var/lib/apt/lists/*
    
    # -------------------------------------------------------------------------
    # 3.  (copie code + colcon build + entrypoint, inchangé)
    # -------------------------------------------------------------------------
    WORKDIR /ros_ws
    COPY . /ros_ws
    RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
                      colcon build --packages-select drone_middleware"
# ─────── 4.  Entrypoint local (overlay ROS + exec) ──────────────
# on copie le script et on lui ajoute le +x
COPY entrypoint.sh /ros_entrypoint_local.sh
RUN chmod 755 /ros_entrypoint_local.sh            # ← clé !

# remplace l’ENTRYPOINT de l’image ros-vanilla
ENTRYPOINT ["/ros_entrypoint_local.sh"]
CMD ["ros2", "launch", "drone_middleware", "dmw.launch.py"]
    