# Coral M.2 (PCIe) migration

The kit moved from the Coral USB accelerator to the M.2 (PCIe) Coral, the Apex
device `1ac1:089a`. On a Raspberry Pi 5 this needs a kernel driver, a
device-tree change, and a group for non-root access. uav-neo was the first
successful build and deployment; racecar-neo runs the same Pi 5 / BCM2712, so
the same recipe is ported here (see Provenance below).

## Problem

The stock gasket/apex driver does not build on kernel 6.8, and even when built it
fails on the Pi 5 with:

```
apex 0000:03:00.0: Couldn't initialize interrupts: -28
```

The Apex requests 13 MSI vectors. The Pi 5 external PCIe (`pcie@110000`) defaults
its `msi-parent` to the small `mip1` MSI peripheral, whose GIC interrupt slots are
too few for 13, especially when an NVMe shares the same controller. The failure
is non-fatal but leaves the device "lamed" (no completion interrupts), so
inference never runs.

## Fix components

1. Driver build: the feranick fork of gasket-driver (adds the kernel 6.8 fixes,
   e.g. `eventfd_signal`, `class_create`).
2. Interrupt allocation: `scripts/gasket-msi-fallback.patch` swaps
   `pci_enable_msix_exact` for `pci_alloc_irq_vectors(PCI_IRQ_MSIX | PCI_IRQ_MSI)`
   so it falls back to MSI. The Pi 5's `pcie1` MSI controller supplies MSI, not
   MSI-X.
3. `coral-msi` overlay (`scripts/coral-msi.dts`): repoints `pcie@110000`
   `msi-parent` from `mip1` to `pcie1`'s own MSI controller, which has enough
   vectors. This is the key fix; equivalent to `dtoverlay=pineboards-hat-ai`, but
   board-agnostic.
4. Access group: udev rule `SUBSYSTEM=="apex", GROUP="apex"` (shipped by the
   `.deb`) plus the current user in the `apex` group.

Prerequisites, both already true on the kit image: 4K pages (`getconf PAGESIZE`
= 4096) and kernel <= 6.11 (kernel 6.12+ breaks gasket separately).

## Reproduction

```
bash scripts/setup_coral.sh
sudo reboot
```

`setup_coral.sh` auto-detects the Apex on the PCI bus and installs `dkms` +
matching `linux-headers` + `device-tree-compiler`, the gasket DKMS `.deb`
(`depend/gasket-dkms_*.deb`), the `apex` group, and the compiled overlay plus the
`dtoverlay=coral-msi` line in `/boot/firmware/config.txt`. DKMS rebuilds the
module on kernel updates. Without the Apex on the bus, the same script installs
only the Coral USB access path (`99-racecar.rules`, no reboot).

## Verification

After reboot, with no manual steps:

```
lsmod | grep apex                 # apex + gasket loaded (modalias auto-load)
xxd /proc/device-tree/axi/pcie@110000/msi-parent   # ...00 00 00 67 (pcie1)
grep -c apex /proc/interrupts      # 13
dmesg | grep apex                  # no "-28"
python3 -c "from pycoral.utils.edgetpu import list_edge_tpus; print(list_edge_tpus())"
# -> [{'type': 'pci', 'path': '/dev/apex_0'}]
```

`edgetpu_node` logs `EdgeTPU found: pci at /dev/apex_0` and publishes to
`/edgetpu/inference`. Reference benchmark: mobilenet_v2_224 at ~3 ms/inference.

## Provenance and port

Ported from `uav_neo_ros2_driver` PR #8 (uav-neo, done and deployed). racecar-neo
is the same Pi 5 / BCM2712, so the recipe transfers directly. The port copies
into this repo:

- `depend/gasket-dkms_*.deb`
- `scripts/coral-msi.dts` and `scripts/gasket-msi-fallback.patch`
- the M.2 branch of `setup_coral.sh`

The overlay targets `&pcie1` and the DTB fixup resolves it per-board, so no
phandle editing is needed. After reboot, confirm `msi-parent` reads `pcie1`'s
phandle.

`depend/gasket-dkms_*.deb` is built from the feranick fork of
`google/gasket-driver` (GPLv2, package version 1.0-18.4) with
`scripts/gasket-msi-fallback.patch` applied. To rebuild: apply the patch to the
fork's `src/`, then `debuild -us -uc -tc -b`.

## Reversibility

- Driver: `sudo dkms remove gasket/1.0 --all` and `sudo apt remove gasket-dkms`.
- Overlay: remove the `dtoverlay=coral-msi` line from
  `/boot/firmware/config.txt`, then reboot.
