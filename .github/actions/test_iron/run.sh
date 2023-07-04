#!/bin/bash
set -e

./.github/setup.sh
# TODO: better method of avoiding linting / testing in build, install, log, ...
mkdir -p .build_ws/src
cp -rpa * .build_ws/src/
cd .build_ws
mkdir -p build install log
chown postgres:postgres build install log -R
su -c ../.github/build.sh postgres
su -c ../.github/test.sh postgres
