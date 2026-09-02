#!/bin/bash

PROJECT_DIR="$HOME/sp_vision_25_for_sentry"
DESKTOP_SRC="$PROJECT_DIR/rm_auto_start/sp_vision.desktop"
DESKTOP_DIR="$HOME/.config/autostart"
DESKTOP_DST="$DESKTOP_DIR/sp_vision.desktop"

mkdir -p "$DESKTOP_DIR"

if [ ! -f "$DESKTOP_SRC" ]; then
    echo "错误: 找不到 $DESKTOP_SRC"
    exit 1
fi

cp "$DESKTOP_SRC" "$DESKTOP_DST"
chmod +x "$DESKTOP_DST"

echo "自启动已安装: $DESKTOP_DST"
echo "下次登录桌面后自动运行。"
