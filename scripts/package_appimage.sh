#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

BUILD_DIR="$REPO_ROOT/build"
APPDIR="$BUILD_DIR/AppDir"
DIST_DIR="$REPO_ROOT/dist"

mkdir -p "$BUILD_DIR" "$DIST_DIR"

DO_CLEAN=false
for arg in "$@"; do
  if [[ "$arg" == "--clean" ]]; then DO_CLEAN=true; break; fi
done
if [[ "${CLEAN_BUILD:-}" == "1" ]] || [[ "${CLEAN_BUILD:-}" == "true" ]] || [[ "$DO_CLEAN" == true ]]; then
  DO_CLEAN=true
fi

echo ">> Building JCToolKit..."
if [[ "$DO_CLEAN" == true ]]; then
  echo ">> Clean build: removing build directory..."
  ./scripts/build.sh -c || true
fi
./scripts/build.sh

BIN_SRC="$BUILD_DIR/JCToolKit"
if [[ ! -x "$BIN_SRC" ]]; then
  echo "Error: expected executable at $BIN_SRC (make sure CMake outputs this target name)" >&2
  exit 1
fi

echo ">> Preparing AppDir at $APPDIR"
rm -rf "$APPDIR"
mkdir -p \
  "$APPDIR/usr/bin" \
  "$APPDIR/usr/share/applications" \
  "$APPDIR/usr/share/metainfo" \
  "$APPDIR/usr/share/icons/hicolor/256x256/apps" \
  "$APPDIR/jctool/original_res"

echo ">> Copying executable..."
cp "$BIN_SRC" "$APPDIR/usr/bin/JCToolKit"

echo ">> Copying resources..."
if [[ -d "$REPO_ROOT/jctool/original_res" ]]; then
  cp -r "$REPO_ROOT/jctool/original_res/." "$APPDIR/jctool/original_res/"
else
  echo "Warning: resource directory jctool/original_res not found; AppImage may miss UI assets" >&2
fi

ICON_SRC_SVG="$REPO_ROOT/packaging/assets/JCToolKit.svg"
ICON_DEST_ROOT_SVG="$APPDIR/JCToolKit.svg"
ICON_DEST_SCALABLE_SVG="$APPDIR/usr/share/icons/hicolor/scalable/apps/JCToolKit.svg"
if [[ -f "$ICON_SRC_SVG" ]]; then
  mkdir -p "$(dirname "$ICON_DEST_SCALABLE_SVG")"
  cp "$ICON_SRC_SVG" "$ICON_DEST_ROOT_SVG"
  cp "$ICON_SRC_SVG" "$ICON_DEST_SCALABLE_SVG"
else
  echo "Warning: icon $ICON_SRC_SVG not found; no application icon will be bundled" >&2
fi

echo ">> Writing AppRun..."
cat > "$APPDIR/AppRun" << 'EOF'
#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
# exec -a sets the process name so the taskbar/DE groups this with the JCToolKit desktop entry
exec -a JCToolKit "./usr/bin/JCToolKit" "$@"
EOF
chmod +x "$APPDIR/AppRun"

echo ">> Writing desktop file..."
DESKTOP_FILE="$APPDIR/JCToolKit.desktop"
cat > "$DESKTOP_FILE" << 'EOF'
[Desktop Entry]
Type=Application
Name=Joy-Con Toolkit
Exec=JCToolKit
Icon=JCToolKit
StartupWMClass=JCToolKit
Categories=Utility;
Terminal=false
EOF

APPDATA_ID="com._jon_dez.jctoolkit"
cp "$DESKTOP_FILE" "$APPDIR/usr/share/applications/${APPDATA_ID}.desktop"

echo ">> Writing AppStream metadata..."
APPDATA_SRC="$REPO_ROOT/packaging/metainfo/JCToolKit.appdata.xml"
APPDATA_DEST="$APPDIR/usr/share/metainfo/${APPDATA_ID}.appdata.xml"
if [[ -f "$APPDATA_SRC" ]]; then
  cp "$APPDATA_SRC" "$APPDATA_DEST"
else
  echo "Warning: AppStream metadata $APPDATA_SRC not found" >&2
fi

echo ">> Building AppImage..."
APPIMAGE_NAME="JCToolKit-x86_64.AppImage"
APPIMAGE_TMP="$BUILD_DIR/$APPIMAGE_NAME"

export APPIMAGE_EXTRACT_AND_RUN=1

if command -v appimagetool-x86_64.AppImage >/dev/null 2>&1; then
  appimagetool-x86_64.AppImage "$APPDIR" "$APPIMAGE_TMP"
elif command -v appimagetool >/dev/null 2>&1; then
  appimagetool "$APPDIR" "$APPIMAGE_TMP"
elif [[ -x "/opt/appimage/appimagetool-x86_64.AppImage" ]]; then
  "/opt/appimage/appimagetool-x86_64.AppImage" "$APPDIR" "$APPIMAGE_TMP"
else
  echo "Error: appimagetool not found in PATH or at /opt/appimage/appimagetool-x86_64.AppImage" >&2
  exit 1
fi

mv "$APPIMAGE_TMP" "$DIST_DIR/$APPIMAGE_NAME"
echo ">> AppImage created at $DIST_DIR/$APPIMAGE_NAME"

