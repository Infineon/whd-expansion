#!/bin/bash
git submodule add https://github.com/Infineon/wpa3-external-supplicant.git External/wpa3-external-supplicant
git submodule add https://github.com/Infineon/wifi-resources.git External/wifi-resources
git submodule add https://github.com/Mbed-TLS/mbedtls.git External/mbedtls

git submodule init
git submodule update

# Get the absolute path of the directory where the script resides
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Path to the patch file (in the same directory as the script)
PATCH_FILE="$SCRIPT_DIR/External/patches/wpa3_ext_supp_whd-e.patch"

# Path to the target Git repository (one level up from "patches")
TARGET_DIR="$SCRIPT_DIR/External/wpa3-external-supplicant"

MBEDTLS="$SCRIPT_DIR/External/mbedtls"

WR="$SCRIPT_DIR/External/wifi-resources"

cd "$MBEDTLS"
git fetch --tags
git checkout mbedtls-2.25.0

cd "$WR"
git fetch --tags
git checkout release-v2.0.0

cd "$TARGET_DIR"
git fetch --tags
git checkout release-v1.2.3

cp "$PATCH_FILE" "$TARGET_DIR"
PATCH_FILE="$TARGET_DIR/wpa3_ext_supp_whd-e.patch"

cd "$TARGET_DIR"

# Apply the patch
if git apply "$PATCH_FILE"; then
    echo "Patch applied successfully!"
else
    echo "Error: Failed to apply the patch."
    exit 1
fi
