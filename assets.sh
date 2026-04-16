#!/bin/bash
git submodule add https://github.com/Infineon/wifi-resources.git External/wifi-resources
git submodule add https://github.com/Infineon/wpa3-external-supplicant.git External/wpa3-external-supplicant
git submodule add https://github.com/Mbed-TLS/mbedtls.git External/mbedtls
git submodule add https://github.com/Infineon/wifi-connection-manager.git External/wifi-connection-manager
git submodule add https://github.com/Infineon/secure-sockets.git External/secure-sockets
git submodule add https://github.com/Infineon/command-console.git External/command-console
git submodule add https://github.com/Infineon/wifi-cert.git External/wifi-cert

git submodule init
git submodule update

# Get the absolute path of the directory where the script resides
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Path to the patch file (in the same directory as the script)
PATCH_DIR="$SCRIPT_DIR/External/patches"
WPA3_EXT_PATCH_FILE="wpa3_ext_supp_whd-e.patch"
WCM_PATCH_FILE="wcm_whd-e.patch"
SS_PATCH_FILE="secure_sockets_whd-e.patch"
CONSOLE_PATCH_FILE="console_whd-e.patch"
CERT_PATCH_FILE="cert_whd-e.patch"

# Path to the target Git repository (one level up from "patches")
WR="$SCRIPT_DIR/External/wifi-resources"
WPA3_EXT="$SCRIPT_DIR/External/wpa3-external-supplicant"
MBEDTLS="$SCRIPT_DIR/External/mbedtls"
WCM="$SCRIPT_DIR/External/wifi-connection-manager"
SS="$SCRIPT_DIR/External/secure-sockets"
CONSOLE="$SCRIPT_DIR/External/command-console"
CERT="$SCRIPT_DIR/External/wifi-cert"

# unzip patches
cd "$PATCH_DIR"
unzip ifx_assets_patches.zip

# fetch tag for wifi-resources
cd "$WR"
git fetch --tags
git checkout release-v2.0.0

# fetch tag and apply patch for wpa3-external-supplicant
cd "$WPA3_EXT"
git fetch --tags
git checkout release-v1.2.3

cp "$PATCH_DIR/$WPA3_EXT_PATCH_FILE" .
if git apply "$WPA3_EXT_PATCH_FILE"; then
    echo "$WPA3_EXT_PATCH_FILE applied successfully!"
else
    echo "Error: Failed to apply $WPA3_EXT_PATCH_FILE."
    exit 1
fi
rm "$WPA3_EXT_PATCH_FILE"

# fetch tag for mbedtls
cd "$MBEDTLS"
git fetch --tags
git checkout mbedtls-2.25.0

# fetch tag and apply patch for wifi-connection-manager
cd "$WCM"
git fetch --tags
git checkout release-v4.0.1

cp "$PATCH_DIR/$WCM_PATCH_FILE" .
if git apply "$WCM_PATCH_FILE"; then
    echo "$WCM_PATCH_FILE applied successfully!"
else
    echo "Error: Failed to apply $WCM_PATCH_FILE."
    exit 1
fi
rm "$WCM_PATCH_FILE"

# fetch tag and apply patch for secure-sockets
cd "$SS"
git fetch --tags
git checkout release-v3.12.0

cp "$PATCH_DIR/$SS_PATCH_FILE" .
if git apply "$SS_PATCH_FILE"; then
    echo "$SS_PATCH_FILE applied successfully!"
else
    echo "Error: Failed to apply $SS_PATCH_FILE."
    exit 1
fi
rm "$SS_PATCH_FILE"

# fetch tag and apply patch for command-console
cd "$CONSOLE"
git fetch --tags
git checkout release-v6.3.0

cp "$PATCH_DIR/$CONSOLE_PATCH_FILE" .
if git apply "$CONSOLE_PATCH_FILE"; then
    echo "$CONSOLE_PATCH_FILE applied successfully!"
else
    echo "Error: Failed to apply $CONSOLE_PATCH_FILE."
    exit 1
fi
rm "$CONSOLE_PATCH_FILE"

# fetch tag and apply patch for wifi-cert
cd "$CERT"
git fetch --tags
git checkout release-v5.1.6

cp "$PATCH_DIR/$CERT_PATCH_FILE" .
if git apply "$CERT_PATCH_FILE"; then
    echo "$CERT_PATCH_FILE applied successfully!"
else
    echo "Error: Failed to apply $CERT_PATCH_FILE."
    exit 1
fi
rm "$CERT_PATCH_FILE"