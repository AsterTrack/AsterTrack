#!/bin/bash

SEARCH_QUERY=$1
REPO_ID=$2
DOWNLOAD_PATH=$3

if [[ $SEARCH_QUERY = "help" || $SEARCH_QUERY = "h" ]]; then
	echo "Usage: sudo search_packages.sh SEARCH_QUERY REPO_ID [DOWNLOAD_PATH]
	SEARCH_QUERY: grep formatted term to search for in packages filetree
    REPO_ID: id of repository, e.g. '13.x/armv6' from http://www.tinycorelinux.net/13.x/armv6/tcz
	DOWNLOAD_PATH: folder to download packages to"
	exit 0
fi

if [[ -z "$DOWNLOAD_PATH" ]]; then
	DOWNLOAD_PATH=search_repo_$REPO_ID
fi

mkdir -p $DOWNLOAD_PATH

pushd $DOWNLOAD_PATH > /dev/null

# Update package list of all used repositories
REPO_URL="http://www.tinycorelinux.net/$REPO_ID/tcz/"
if [[ ! -f "repo.txt" ]]; then
    wget -q $REPO_URL -O "repo.txt"
fi

PACKAGES="$PACKAGES $(cat "repo.txt")"
for pkg in $PACKAGES; do

	# Download package if not already there
    if [[ ! -f $pkg.list ]]; then
        url="$REPO_URL/$pkg.list"
	    wget -N -q --show-progress $url
    fi
    if [[ ! -f $pkg.list ]]; then
        echo "Failed to download $pkg list from the repo ($url)!"
        exit 1
    fi

	query=$(grep $SEARCH_QUERY $pkg.list)
    if [[ "$query" != "" ]]; then
        echo "Found results in package $pkg (in $REPO_ID repository):"
        echo $query
    fi
done

popd > /dev/null