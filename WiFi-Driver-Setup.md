# Wi‑Fi Setup Script for Jetson Orin NX (JetPack 6.2)

This README explains the **exact** commands required to enable an Intel‑based M.2 Key‑E Wi‑Fi module (e.g. 8265/8260/AX200/AX210) on a Jetson Orin NX flashed with **JetPack 6.2**.

> **Scope:** Only the commands listed here are needed—no additional utilities or diagnostics are included.

---

## 1  Update APT Metadata

```bash
sudo apt update
```

---

## 2  Install Kernel Build Prerequisites

The driver will be compiled against the running JetPack kernel via DKMS. Install the build toolchain first:

```bash
sudo apt install build-essential dkms
```

---

## 3  Install the Intel Wi‑Fi Driver Bundle

`backport‑iwlwifi‑dkms` automatically builds and installs **iwlwifi**, **iwlmvm**, and supporting modules for your current kernel.

```bash
sudo apt install backport-iwlwifi-dkms
```

---

## 4  Install `rfkill` Utilities (and Kernel Module)

The user‑space `rfkill` tool also pulls in the missing **rfkill.ko** kernel module required by the Wi‑Fi stack:

```bash
sudo apt install rfkill
```

---

## 5  Load the Driver Immediately

Without rebooting, load **iwlwifi** so the interface appears right away:

```bash
sudo modprobe iwlwifi
```

---

## 6  Ensure the Driver Loads on Every Boot

Create a one‑line file that the init system reads during startup:

```bash
echo iwlwifi | sudo tee /etc/modules-load.d/iwlwifi.conf
```

---

## 7  (Optional) Reboot & Verify

A reboot is *not* strictly necessary, but it confirms the modules auto‑load:

```bash
sudo reboot
```

After the system comes back up, check that the interface exists:

```bash
nmcli device
```

You should see something like `wlp1p1s0  wifi  disconnected`—ready to scan and connect.

---

### Troubleshooting

If the Wi‑Fi interface does not show up after completing **all** steps above, verify the following:

1. **Hardware visibility** – `lspci | grep -i network` should list the Intel device.
2. **Driver loaded** – `lsmod | grep iwlwifi` should list `iwlwifi` and `iwlmvm`.
3. **Firmware present** – `dmesg | grep iwlwifi` should show firmware loaded successfully.

If any of these checks fail, repeat the steps or consult NVIDIA’s Jetson forums for board‑specific quirks.

---

© 2025 Your Name / Your Organization. Feel free to copy or modify this guide for your own Jetson projects.
