#!/bin/bash

VERSION=$(awk -F "= " '/version/ {print $2}' < aica-package.toml)
VERSION=$(echo "${VERSION}" | tr -d '"')

DIFF_TYPE=''
DECREMENT=false
DRY_RUN=false
COMMIT=false

HELP_MESSAGE="Usage: ./update_version.sh [OPTIONS]

Update the project version number by one increment.
For the version number in the format \"major.minor.patch\",
the patch number will be incremented by default.

Supplying additional options to the script will change
the behaviour, either to set a specific version, update
the major or minor numbers, or to decrement instead.

This script modifies the version in the following files:
  ./source/modulo_components/package.xml
  ./source/modulo_controllers/package.xml
  ./source/modulo_core/package.xml
  ./source/modulo_interfaces/package.xml
  ./source/modulo_utils/package.xml
  ./doxygen/doxygen.conf
  ./aica-package.toml

Options:
  --major                  Update the major version number.
  --minor                  Update the minor version number.

  -v, --version <x.y.z>    Set a specific version number.

  -d, --downgrade          Decrement instead of increment.

  -c, --commit             Automatically commit the change
                           to git source control.

  -n, --dry-run            Echo the new version but do not
                           write changes to any files.

  -h, --help               Show this help message."

while [ "$#" -gt 0 ]; do
  case "$1" in
    -v | --version) NEW_VERSION=$2; DIFF_TYPE=fixed; shift 2;;
    --major) DIFF_TYPE=major; shift 1;;
    --minor) DIFF_TYPE=minor; shift 1;;
    -d | --downgrade) DECREMENT=true; shift 1;;
    -c | --commit) COMMIT=true; shift 1;;
    -n | --dry-run) DRY_RUN=true; shift 1;;
    -h|--help) echo "${HELP_MESSAGE}"; exit 0;;

    -*) echo "Unknown option: $1" >&2; echo "${HELP_MESSAGE}"; exit 1;;
  esac
done

if [ "${COMMIT}" == true ]; then
  STAGED_CHANGES=$(git diff --cached)
  if [ "${STAGED_CHANGES}" ]; then
    echo "Staged changes detected! Resolve to a clean working state before using the --commit option."
    exit 1
  fi
fi

function update_field() {
  local FIELD=$1
  if [ "${DECREMENT}" == true ]; then
    if [ "${FIELD}" -ne "0" ]; then
      FIELD=$(( FIELD - 1 ))
    fi
  else
    FIELD=$(( FIELD + 1 ))
  fi
  echo "${FIELD}"
}

function update_version() {
  # break down the version number into its components
  regex="([0-9]+).([0-9]+).([0-9]+)"
  if [[ "$VERSION" =~ ${regex} ]]; then
    MAJOR="${BASH_REMATCH[1]}"
    MINOR="${BASH_REMATCH[2]}"
    BUILD="${BASH_REMATCH[3]}"
  fi

  # check parameter to see which number to increment
  if [[ "${DIFF_TYPE}" == "major" ]]; then
    MAJOR=$(update_field "${MAJOR}")
  elif [[ "${DIFF_TYPE}" == "minor" ]]; then
    MINOR=$(update_field "${MINOR}")
  else
    BUILD=$(update_field "${BUILD}")
  fi

  echo "${MAJOR}.${MINOR}.${BUILD}"
}

if [[ "${DIFF_TYPE}" != "fixed" ]]; then
  NEW_VERSION=$(update_version)
fi

echo "Updating version from ${VERSION} to ${NEW_VERSION}"
if [ "${DRY_RUN}" == true ]; then
  echo "Dry run complete. Exiting without changing any files"
  exit 0
fi

SED_STR_PACKAGE="s/<version>${VERSION}<\/version>/<version>${NEW_VERSION}<\/version>/g"
SED_STR_DOXYGEN="s/PROJECT_NUMBER = ${VERSION}/PROJECT_NUMBER = ${NEW_VERSION}/g"
SED_STR_TOML="s/version = \"${VERSION}\"/version = \"${NEW_VERSION}\"/g"

if [[ "$OSTYPE" == "darwin"* ]]; then
  sed -i '' "${SED_STR_PACKAGE}" ./source/modulo_components/package.xml
  sed -i '' "${SED_STR_PACKAGE}" ./source/modulo_controllers/package.xml
  sed -i '' "${SED_STR_PACKAGE}" ./source/modulo_core/package.xml
  sed -i '' "${SED_STR_PACKAGE}" ./source/modulo_interfaces/package.xml
  sed -i '' "${SED_STR_PACKAGE}" ./source/modulo_utils/package.xml
  sed -i '' "${SED_STR_DOXYGEN}" ./doxygen/doxygen.conf
  sed -i '' "${SED_STR_TOML}" ./aica-package.toml
else
  sed -i "${SED_STR_PACKAGE}" ./source/modulo_components/package.xml
  sed -i "${SED_STR_PACKAGE}" ./source/modulo_controllers/package.xml
  sed -i "${SED_STR_PACKAGE}" ./source/modulo_core/package.xml
  sed -i "${SED_STR_PACKAGE}" ./source/modulo_interfaces/package.xml
  sed -i "${SED_STR_PACKAGE}" ./source/modulo_utils/package.xml
  sed -i "${SED_STR_DOXYGEN}" ./doxygen/doxygen.conf
  sed -i "${SED_STR_TOML}" ./aica-package.toml
fi

if [ "${COMMIT}" == true ]; then
  echo "Committing changes to source control"
  git add ./source/modulo_components/package.xml ./source/modulo_controllers/package.xml \
      ./source/modulo_core/package.xml ./source/modulo_interfaces/package.xml ./source/modulo_utils/package.xml \
      ./doxygen/doxygen.conf aica-package.toml
  git commit -m "${VERSION} -> ${NEW_VERSION}"
fi