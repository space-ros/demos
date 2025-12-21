#!/bin/bash

# Source the package we have built, so that we can just run our launchfiles directly
source install/setup.bash
# since we install new ros packages, source the base ros install
source "/opt/ros/${ROS_DISTRO}/setup.sh"
# source spaceros specific install
source "${SPACEROS_DIR}/setup.bash"

echo "                                         _.oo.   "
echo "                 _.u[[/;:,.         .odMMMMMM'   "
echo "              .o888UU[[[/;:-.  .o@P^    MMM^     "
echo "             oN88888UU[[[/;::-.        dP^       "
echo "            dNMMNN888UU[[[/;:--.   .o@P^         "
echo "           ,MMMMMMN888UU[[/;::-. o@^             "
echo "           NNMMMNN888UU[[[/~.o@P^                "
echo "           888888888UU[[[/o@^-..                 "
echo "          oI8888UU[[[/o@P^:--..                  "
echo "       .@^  YUU[[[/o@^;::---..                   "
echo "     oMP     ^/o@P^;:::---..                     "
echo "  .dMMM    .o@^ ^;::---...                       "
echo " dMMMMMMM@^`       `^^^^                         "
echo "YMMMUP^                                          "
echo " ^^                                              "
echo "                   Welcome!                      "

exec "$@"
