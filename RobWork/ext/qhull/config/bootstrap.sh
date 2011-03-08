#!/bin/sh

run () {
  echo -n Running `$1 --version | sed q`...
  $* > /dev/null
  echo " done"
}


if test ! -f config/configure.ac ; then
  echo "$0: This script must be run from the Qhull top directory."
  exit 1
fi

echo -n Copying autoconf and automake files...
cp config/configure.ac .
cp config/Makefile-am-main Makefile.am
for d in src html eg ; do
  cp config/Makefile-am-$d $d/Makefile.am
done
echo " done"

run aclocal \
  && run libtoolize --force --copy \
  && run automake --foreign --add-missing --force-missing --copy \
  && run autoconf
