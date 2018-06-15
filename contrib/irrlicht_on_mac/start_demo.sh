#
#	Script to start an Irrlicht3D based Mac App
#
if [ $(uname) != "Darwin" ] ; then
	echo "You don't need this tool on your system!"
	exit
fi
if [ $# > 0 ] ; then
	if [ -d $1 ] ; then
		APPNAME=$( echo $1 | sed "s/\app.*$/app/" )
		TOOLNAME=$( basename $APPNAME | sed "s/\.app$//" )
		TOOLPATH=${APPNAME}/Contents/MacOS/${TOOLNAME}
		shift
		$TOOLPATH $@
	else
		echo "App >" $1 " does not exist!"
	fi
else
	echo "usage: $0 IrrlichtBased.app [ args ]"
fi