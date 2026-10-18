#!/bin/bash
#
# build.sh - Arduino CLI wrapper for the fogbox firmware
#
# Builds, uploads and monitors the two fogbox sketches: fog_controller (the
# box) and fog_trigger (the upstream trigger node). MODULE selects which one;
# build defaults to both, upload/monitor/deploy need a single MODULE.
#

HERE=$(dirname $(readlink -f $BASH_SOURCE))
MODULES="fog_controller fog_trigger"
MODULE=${MODULE:-}
DEV_PLATFORM=${DEV_PLATFORM:-esp32:esp32@3.3.0}
DEV_BOARD=${DEV_BOARD:-esp32:esp32:esp32c3}
DEV_PORT=${DEV_PORT:-/dev/ttyACM0}
DEV_BOARD_BAUDRATE=${DEV_BOARD_BAUDRATE:-115200}
export ARDUINO_DIRECTORIES_DATA=${ARDUINO_DIRECTORIES_DATA:-$HERE/.arduino15}
export TMPDIR=/var/tmp

function do_update(){
    DEV_URLS=${DEV_URLS:-https://dl.espressif.com/dl/package_esp32_index.json}
    [ "${DEV_UPDATE:-0}" = 1 ] && {
        arduino-cli --additional-urls "$DEV_URLS" update
        arduino-cli --additional-urls "$DEV_URLS" core install "${DEV_PLATFORM}"
        arduino-cli --additional-urls "$DEV_URLS" board list
    }
}

function need_module(){
    if [ -z "$MODULE" ]; then
        echo "Set MODULE to one of: $MODULES"
        exit 1
    fi
}

function do_build(){
    DEV_EXTRA_FLAGS="-DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1"
    if [ "${DEBUG:-0}" = "1" ]; then
        DEV_EXTRA_FLAGS="$DEV_EXTRA_FLAGS -DDEBUG"
    fi
    if [ ! -z "${EXTRA_FLAGS}" ]; then
        DEV_EXTRA_FLAGS="$DEV_EXTRA_FLAGS ${EXTRA_FLAGS}"
    fi
    for m in ${MODULE:-$MODULES}; do
        arduino-cli -b ${DEV_BOARD} compile \
            --output-dir dist/$m \
            --jobs 16 \
            --build-property compiler.cpp.extra_flags="$DEV_EXTRA_FLAGS" \
            --build-property compiler.c.extra_flags="$DEV_EXTRA_FLAGS" \
            --build-property build.extra_flags="$DEV_EXTRA_FLAGS" \
            $HERE/$m \
            || exit $?
    done
}

function do_upload(){
    need_module
    arduino-cli -b ${DEV_BOARD} upload -p ${DEV_PORT} --input-dir dist/$MODULE $HERE/$MODULE
}

function do_monitor(){
    arduino-cli -b ${DEV_BOARD} monitor -p ${DEV_PORT} -c baudrate=${DEV_BOARD_BAUDRATE}
}

function show_usage(){
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  build|compile   Build the sketch(es) into dist/<module>"
    echo "  update          Update the ESP32 platform only (no build)"
    echo "  upload          Upload the compiled firmware to the device"
    echo "  monitor         Open a serial monitor to the device"
    echo "  deploy          Build, upload and monitor"
    echo "  help            Show this usage information"
    echo ""
    echo "Environment variables:"
    echo "  MODULE                    fog_controller or fog_trigger (build default: both)"
    echo "  DEV_PLATFORM              Arduino platform (default: esp32:esp32@3.3.0)"
    echo "  DEV_BOARD                 Board identifier (default: esp32:esp32:esp32c3)"
    echo "  DEV_PORT                  Serial port (default: /dev/ttyACM0)"
    echo "  DEV_BOARD_BAUDRATE        Monitor baudrate (default: 115200)"
    echo "  DEBUG                     Set to 1 to add -DDEBUG flag"
    echo "  EXTRA_FLAGS               Extra compiler flags"
    echo ""
    echo "Examples:"
    echo "  $0 build"
    echo "  MODULE=fog_controller DEV_PORT=/dev/ttyACM0 $0 deploy"
    echo "  MODULE=fog_trigger $0 deploy"
}

case $1 in
    upload)
        do_upload
        ;;
    monitor)
        do_monitor
        ;;
    build|compile)
        DEV_UPDATE=${DEV_UPDATE:-0} do_update
        do_build
        ;;
    deploy|all)
        need_module
        DEV_UPDATE=${DEV_UPDATE:-0} do_update
        do_build
        do_upload
        do_monitor
        ;;
    update)
        DEV_UPDATE=1 do_update
        ;;
    help|--help|-h|"")
        show_usage
        ;;
    *)
        echo "Unknown command: $1"
        echo ""
        show_usage
        exit 1
        ;;
esac
