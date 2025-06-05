#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash

# ---------- commande passée au conteneur ----------
cmd=( "$@" )

# si la var d'env existe on l'ajoute correctement
if [[ -n "${TRANSPORT_URL}" ]]; then
  cmd+=( --ros-args -p "transport_url:=${TRANSPORT_URL}" )
fi

exec "${cmd[@]}"
