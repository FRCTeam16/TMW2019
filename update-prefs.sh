#!/bin/sh
FIND="find src/main -type f -print0"
XSED="xargs -0 sed -i"
BSPREFH="Util\/BSPrefs.h"

# Includes
$FIND | $XSED 's/<frc\/Preferences.h>/"Util\/BSPrefs.h"/g'
$FIND | $XSED 's/"frc\/Preferences.h"/"Util\/BSPrefs.h"/g'
$FIND | $XSED 's/"Util\/PrefUtil.h"/"Util\/BSPrefs.h"/g'

echo
$FIND | $XSED 's/frc::Preferences::GetInstance()/BSPrefs::GetInstance()/g'
$FIND | $XSED 's/Preferences::GetInstance()/BSPrefs::GetInstance()/g'
# $FIND | $XSED 's/frc::Preferences *prefs/auto prefs/g'

echo
$FIND | $XSED 's/PrefUtil::getSetInt/BSPrefs::GetInstance()->GetInt/g'
$FIND | $XSED 's/PrefUtil::getSetBool/BSPrefs::GetInstance()->GetBool/g'
$FIND | $XSED 's/PrefUtil::getSet/BSPrefs::GetInstance()->GetDouble/g'
$FIND | $XSED 's/>GetFloat/>GetDouble/g'

$FIND | $XSED 's/frc::Preferences/BSPrefs/g'
