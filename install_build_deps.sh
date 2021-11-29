#!/bin/bash

# list all your dependencies here
DEPS="libmodal_json openmp libvoxl_cutils ceres-solver nlopt voxl-mpa-tools libmodal_pipe voxblox"


# variables
OPKG_CONF=/etc/opkg/opkg.conf
STABLE=http://voxl-packages.modalai.com/stable
DEV=http://voxl-packages.modalai.com/dev


# make sure opkg config file exists
if [ ! -f ${OPKG_CONF} ]; then
	echo "ERROR: missing ${OPKG_CONF}"
	echo "are you not running in voxl-emulator or voxl-cross?"
	exit 1
fi


# parse dev or stable option
if [ "$1" == "stable" ]; then
	echo "using stable repository"
	PKG_STRING="src/gz stable ${STABLE}"

elif [ "$1" == "dev" ]; then
	echo "using development repository"
	PKG_STRING="src/gz dev ${DEV}"

else
	echo ""
	echo "Please specify if the build dependencies should be pulled from"
	echo "the stable or development modalai opkg package repos."
	echo "If building the master branch you should specify stable."
	echo "For development branches please specify dev."
	echo ""
	echo "./install_build_deps.sh stable"
	echo "./install_build_deps.sh dev"
	echo ""
	exit 1
fi

# delete any existing repository entries
sudo sed -i '/voxl-packages.modalai.com/d' ${OPKG_CONF}

# write in the new entry
sudo echo ${PKG_STRING} >> ${OPKG_CONF}
sudo echo "" >> ${OPKG_CONF}

## make sure we have the latest package index
sudo opkg update


# install/update each dependency
for i in ${DEPS}; do
	# this will also update if already installed!
	sudo opkg install --nodeps $i
done

exit 0
