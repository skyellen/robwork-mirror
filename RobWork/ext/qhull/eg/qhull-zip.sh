#!/bin/sh
#
# qhull-zip.sh version -- Make zip and tgz files for Qhull release
#
# wzzip from http://www.winzip.com/wzcline.htm
#   can not use path with $zip_file 
#   odd error messages if can't locate directory
#
# $Id: //main/2005/road/road-bash/local/bin/qhull-zip.sh#18 $$Change: 1141 $
# $DateTime: 2010/01/03 12:13:50 $$Author: bbarber $

if [[ $# -eq 0 ]]; then
        echo 'Missing date stamp, e.g., qhull-zip.sh 2007.1' 
        exit
fi
version=$1

err_program=qhull-zip
err_log=/var/tmp/qhull-zip.log
[[ -e $HOME/bash/etc/road-script.sh ]] && source $HOME/bash/etc/road-script.sh \
    || source /etc/road-script.sh

check_err_log $LINENO "$err_log"
check_err_log $LINENO "$err_step_log"
log_step $LINENO "Logging to $err_log\n... and $err_step_log"

log_note $LINENO "Find Qhull directory" 
if [[ ! -d qhull/eg && ! -d ../qhull/eg && -d ../../qhull/eg ]]; then
    err_exit $LINENO "qhull/eg directory not found at or above $PWD"
fi
if [[ ! -d qhull/eg ]]; then
    if [[ -d ../qhull/eg ]]; then
        cd ..
    else
        cd ../..
    fi
fi
root_dir=$(pwd)

TEMP_DIR="$TMP/qhull-zip-$(ro_today2)"
TEMP_FILE="$TMP/qhull-zip-$(ro_today2).txt"

qhull_zip_file=qhull-$version.zip # no path or spaces
qhull_tgz_file=qhull-$version-src.tgz
qhullmd5_file=qhull-$version.md5sum

exit_if_fail $LINENO "rm -f $qhull_zip_file $qhull_tgz_file $qhullmd5_file"

#############################
log_step $LINENO "Check environment"
#############################

[[ $(type -p md5sum) ]] || exit_err $LINENO "md5sum is missing"
[[ $(cp --help || grep '[-]-parents') ]] ||  exit_err $LINENO "cp does not have --parents option"

#############################
log_step $LINENO "Define functions"
#############################

function check_zip_file #zip_file
{
    local zip_file=$1
    local HERE=$(ro_here)
    log_note $HERE "Check $zip_file"
    ls -l $zip_file >>$err_log
    exit_if_err $HERE "Did not create $zip_file"
    wzunzip -ybc -t $zip_file | grep -E -v -e '( OK|Zip)' >>$err_log
    exit_if_err $HERE "Error while checking $zip_file"
}

function check_tgz_file #tgz_file
{
    local tgz_file=$1
    local HERE=$(ro_here)
    log_note $HERE "Check $tgz_file"
    ls -l $tgz_file >>$err_log
    exit_if_err $HERE "Did not create $tgz_file"
    tar -tzf $tgz_file >/dev/null 2>>$err_log
    exit_if_err $HERE "Can not extract -- tar -tzf $tgz_file"
}

function convert_to_unix #dir -- convert files to Unix, preserving modtime from PWD
{
    local temp_dir=$1
    local HERE=$(ro_here)
    log_note $HERE "Convert files to unix format in $1"
    for f in $(find $temp_dir -type f | grep -E '^([^.]*|.*\.(bashrc|c|cfg|cpp|css|d|h|htm|html|man|pl|pro|profile|sh|sql|termcap|txt|xml|xsd|xsl))$'); do
        exit_if_fail $HERE "d2u '$f' && touch -r '$root_dir/${f#$temp_dir/}' '$f'"
    done
}

function create_md5sum #md5_file -- create md5sum of current directory
{
    local md5_file=$1
    local HERE=$(ro_here)
    log_step $HERE "Compute $md5_file"
    exit_if_fail $HERE "rm -f $md5_file"
    find . -type f | sed 's|^\./||' | sort | xargs md5sum >>$md5_file
    exit_if_err $HERE "md5sum failed"
    log_note $HERE "$(md5sum $md5_file)"
}

#############################
log_step $LINENO "Configure $0 for $(pwd)/qhull"
#############################

md5_zip_file=qhull-$version.zip.md5sum
md5_tgz_file=qhull-$version-src.tgz.md5sum

# recursive 
qhull_dirs="qhull/cpp qhull/eg qhull/html qhull/src"
qhull_files="qhull/project/*.pro qhull/project/*.sln qhull/project/*/*.vcproj qhull/project/*/*.pro \
    qhull/project/libqhull.vcproj \
    qhull/Announce.txt qhull/CMakeLists.txt qhull/COPYING.txt qhull/File_id.diz qhull/QHULL-GO.pif \
    qhull/README.txt qhull/REGISTER.txt qhull/index.htm \
    qhull/qconvex.exe qhull/qdelaunay.exe qhull/qhalf.exe \
    qhull/qhull.exe qhull/qvoronoi.exe qhull/rbox.exe"
qhull_ufiles="$qhull_dirs qhull/project/*.pro qhull/project/*.sln qhull/project/*/*.vcproj qhull/project/*/*.pro \
    qhull/project/libqhull.vcproj \
    qhull/Announce.txt qhull/CMakeLists.txt qhull/COPYING.txt qhull/File_id.diz qhull/QHULL-GO.pif \
    qhull/README.txt qhull/REGISTER.txt qhull/index.htm"

set noglob

if [[ -e /bin/msysinfo && $(type -p wzzip) && $(type -p wzunzip) ]]; then

    #############################
    log_step $LINENO "Build zip directory, $TEMP_DIR/qhull"
    #############################

    ls -l $qhull_files $qhull_dirs >>$err_log 
    exit_if_err $LINENO "Missing files for zip directory"

    log_note $LINENO "Copy \$qhull_files \$qhull_dirs to $TEMP_DIR/qhull"
    exit_if_fail $LINENO "rm -rf $TEMP_DIR && mkdir $TEMP_DIR"
    exit_if_fail $LINENO "cp -r -p --parents $qhull_files $qhull_dirs $TEMP_DIR"

    #############################
    log_step $LINENO "Write md5sum to $md5_tgz_file"
    #############################

    exit_if_fail $LINENO "pushd $TEMP_DIR/qhull"
    create_md5sum $md5_zip_file

    #############################
    log_step $LINENO "Write $qhull_zip_file"
    #############################

    log_note $LINENO "Write \$qhull_files to $qhull_zip_file"
    exit_if_fail $LINENO "cd .. && mv qhull qhull-$version && md5sum qhull-$version/$md5_zip_file >>$root_dir/$qhullmd5_file"
    wzzip -P -r -u $qhull_zip_file qhull-$version >>$err_log
    exit_if_err $LINENO "wzzip does not exist or error while zipping files"
    check_zip_file $qhull_zip_file
    exit_if_fail $LINENO "popd"
    exit_if_fail $LINENO "mv $TEMP_DIR/$qhull_zip_file ."
fi

#############################
log_step $LINENO "Build tgz directory, $TEMP_DIR/qhull"
#############################

log_note $LINENO "Archive these files as $qhull_tgz_file"
ls -l $qhull_ufiles >>$err_log 
exit_if_err $LINENO "Missing files for tgz"

exit_if_fail $LINENO "rm -rf $TEMP_DIR && mkdir -p $TEMP_DIR"
exit_if_fail $LINENO "cp -r -p --parents $qhull_ufiles $TEMP_DIR"

if [[ $IS_WINDOWS && $(type -p d2u) ]]; then
    convert_to_unix "$TEMP_DIR"
fi
exit_if_fail $LINENO "mv $TEMP_DIR/qhull/src/Makefile.txt $TEMP_DIR/qhull/src/Makefile"

#############################
log_step $LINENO "Write md5sum to $md5_tgz_file"
#############################

exit_if_fail $LINENO "pushd $TEMP_DIR && cd qhull"
create_md5sum $md5_tgz_file

exit_if_fail $LINENO "cd .. && mv qhull qhull-$version && md5sum qhull-$version/$md5_tgz_file >>$root_dir/$qhullmd5_file"

#############################
log_step $LINENO "Write $qhull_tgz_file"
#############################

exit_if_fail $LINENO "tar -zcf $root_dir/$qhull_tgz_file * && popd"
check_tgz_file $qhull_tgz_file

log_note $LINENO "md5sum of zip and tgz files"

for f in $qhull_zip_file $qhull_tgz_file; do
    if [[ -r $f ]]; then
        exit_if_fail $LINENO "md5sum $f >>$qhullmd5_file"
    fi
done

#############################
log_step $LINENO "Extract zip and tgz files to ($TEMP_DIR)"
#############################

exit_if_fail $LINENO "rm -rf $TEMP_DIR"
if [[ -r $root_dir/$qhull_zip_file ]]; then
    exit_if_fail $LINENO "mkdir -p $TEMP_DIR/zip && cd $TEMP_DIR/zip"
    if [[ -r $root_dir/$qhull_zip_file ]]; then
        exit_if_fail $LINENO "wzunzip -yb -d $root_dir/$qhull_zip_file"
    fi
    log_step $LINENO "Search for date stamps to Dates.txt"
    find . -type f | grep -v '/bin/' | xargs grep '\-20' | grep -v -E '(page=|ISBN|sql-2005|utility-2000|written 2002-2003|tail -20|Spinellis|WEBSIDESTORY|D:06-5-2007|server-2005)' >Dates.txt
    find . -type f | grep -v '/bin/' | xargs grep -i 'qhull *20' >>Dates.txt
fi
if [[ -r $root_dir/$qhull_tgz_file ]]; then
    exit_if_fail $LINENO "mkdir -p $TEMP_DIR/tgz && cd $TEMP_DIR/tgz"
    if [[ -r $root_dir/$qhull_tgz_file ]]; then
        exit_if_fail $LINENO "tar -zxf $root_dir/$qhull_tgz_file"
    fi
fi

#############################
log_step $LINENO "Compare previous zip release, Dates.txt, and md5sum.  Check for virus."
log_step $LINENO "Search xml files for UNDEFINED. Check page links"
log_step $LINENO "Finished successfully"
#############################
        