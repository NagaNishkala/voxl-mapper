#!/bin/bash
################################################################################
# Copyright (c) 2022 ModalAI, Inc. All rights reserved.
################################################################################

# Script to install build dependencies in voxl-cross docker image

# list all your dependencies here. Note for packages that have AMD64 equivalents
# in the ubuntu repositories you should specify the arm64 architecture to make
# sure the correct one is installed in voxl-cross.
DEPS="
libmodal-json
libvoxl-cutils
ceres-solver
nlopt
voxl-mpa-tools
libmodal-pipe
voxblox"


print_usage(){
	echo ""
	echo " Install build dependencies from a specified repository."
	echo " For \"dev\" and \"stable\" repos the packages will be pulled as IPKs"
	echo " Otherwise debs will be pulled with apt."
	echo ""
	echo " Usage:"
	echo ""
	echo "  ./install_build_deps.sh dev"
	echo "        Install from 820 development repo."
	echo ""
	echo "  ./install_build_deps.sh stable"
	echo "        Install from 820 stable repo."
	echo ""
	echo "  ./install_build_deps.sh dev-deb"
	echo "        Install from 865 development repo."
	echo ""
	echo ""
}

# make sure one argument was given
if [ "$#" -ne 1 ]; then
	print_usage
	exit 1
fi
REPO=$1
MODE="DEB"


# check for ipk-based repos
if [ "$REPO" == "dev" ] || [ "$REPO" == "stable" ]; then
	MODE="IPK"
fi


# install deb packages with apt
if [ "$MODE" == "DEB" ]; then
	echo "using $REPO DEB repo"
	# write in the new entry
	DPKG_FILE="/etc/apt/sources.list.d/modalai.list"
	LINE="deb [trusted=yes] http://voxl-packages.modalai.com/${REPO}/ ./"
	sudo echo "${LINE}" > ${DPKG_FILE}

	## make sure we have the latest package index
	## only pull from voxl-packages to save time
	sudo apt-get update -o Dir::Etc::sourcelist="sources.list.d/modalai.list" -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0"

	## install the user's list of dependencies
	echo "installing: "
	echo $DEPS
	sudo apt-get install -y $DEPS

# install IPK packages with opkg
else
	echo "using $REPO IPK repo"
	OPKG_CONF="/etc/opkg/opkg.conf"
	# delete any existing repository entries
	sudo sed -i '/voxl-packages.modalai.com/d' ${OPKG_CONF}

	# write in the new entry
	sudo echo ${PKG_STRING} >> ${OPKG_CONF}
	LINE="src/gz ${REPO} http://voxl-packages.modalai.com/${REPO}"
	sudo echo "$LINE" >> ${OPKG_CONF}

	## make sure we have the latest package index
	sudo opkg update

	echo "installing: "
	echo $DEPS

	# install/update each dependency
	for i in ${DEPS}; do
		# this will also update if already installed!
		sudo opkg install --nodeps $i
	done

fi

echo ""
echo "Done installing dependencies"
echo ""
exit 0
