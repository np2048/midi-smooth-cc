# midi-smooth-cc/docs/INSTALL.md

# Build & Install

## Dependencies

### Arch Linux / Manjaro

```bash
sudo pacman -S --needed base-devel cmake lv2
```

### Debian / Ubuntu

```bash
sudo apt update
sudo apt install build-essential cmake lv2-dev
```

### Fedora

```bash
sudo dnf install @development-tools cmake lv2-devel
```

## Build

```bash
cd midi-smooth-cc
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

The LV2 bundle is produced at `build/midi-smooth-cc.lv2/`.

## Run Tests

```bash
cd build
ctest --output-on-failure
```

## Install to ~/.lv2

```bash
cmake --install build
```

Override the install location if needed:

```bash
cmake -S . -B build -DLV2_INSTALL_DIR=/usr/local/lib/lv2
cmake --install build
```

## Rescan Plugins in Carla

1. Open Carla.
2. `Edit -> Rescan Plugins` (or restart Carla).
3. Search for `MIDI Smooth CC` in the plugin list.

## Verify Discovery via `lv2ls`

```bash
lv2ls | grep midi-smooth-cc
```
