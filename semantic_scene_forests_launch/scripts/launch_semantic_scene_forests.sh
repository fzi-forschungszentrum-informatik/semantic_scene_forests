#!/bin/bash

CATMUX_CONFIG="$(ros2 pkg prefix semantic_scene_forests_launch)/share/semantic_scene_forests_launch/etc/semantic_scene_forests.catmux.yaml"

catmux_create_session "$CATMUX_CONFIG"
