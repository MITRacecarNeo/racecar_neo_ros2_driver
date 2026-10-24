# RACECAR Neo: where the car's data goes (racecar_neo_ros2_driver/data_root.py).
# Installed to /etc/profile.d/racecar-data.sh by scripts/setup_services.sh.
# racecar-data.service writes the path once per boot.
if [ -r /run/racecar/data_root ]; then
    RACECAR_DATA_DIR="$(cat /run/racecar/data_root)"
else
    RACECAR_DATA_DIR="$HOME/data"
fi
export RACECAR_DATA_DIR
