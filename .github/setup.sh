#!/bin/bash
set -e

export DEBIAN_FRONTEND=noninteractive

apt-get update

apt-get install -y curl

curl https://www.postgresql.org/media/keys/ACCC4CF8.asc | gpg --dearmor | tee /etc/apt/trusted.gpg.d/apt.postgresql.org.gpg >/dev/null
echo "deb http://apt.postgresql.org/pub/repos/apt $(lsb_release -cs)-pgdg-testing main 15" | tee /etc/apt/sources.list.d/pgdg-testing.list

apt-get update \
   && apt-get -y install --no-install-recommends \
   postgresql-14 postgresql-14-postgis-3 \
   python3-pip \
   git-lfs \
   libpq-dev \
   && apt-get autoremove -y

python3 setup.py egg_info
pip3 install -r postgis_ros_bridge.egg-info/requires.txt

rosdep update
rosdep install --from-paths . --ignore-src -y
