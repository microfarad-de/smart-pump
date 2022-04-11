#!/bin/bash
# Updated the version and date information within the main executable
# and generate a zip archive with the entire source code including submodules.

option="$1"
file_name="smart-pump"
version_file="version.h"

version_major=$(cat $version_file | grep VERSION_MAJOR | awk -F' ' '{print $3}')
version_minor=$(cat $version_file | grep VERSION_MINOR | awk -F' ' '{print $3}')
version_maint=$(cat $version_file | grep VERSION_MAINT | awk -F' ' '{print $3}')

version="$version_major.$version_minor.$version_maint"

if [[ -z $version ]]; then
  echo "Error: No version string specified"
  exit 1
fi

sed -i -e "s/.* * Version:.*/ * Version: $version/" "$file_name.ino"
sed -i -e "s/.* * Date:.*/ * Date:    $(date '+%B %d, %Y')/" "$file_name.ino"
rm -rf "$file_name.ino-e"

rm -rf "$file_name-"*"-full"*

if [[ "$option" != "clean" ]]; then
  zip -r "$file_name-$version-full.zip" . -x '*.git*' '*.vscode*' '*private*' '*-build*' '*.DS_Store*'
fi

exit 0
