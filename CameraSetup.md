# IMX477 “Soft / Blurry Preview” Fixes on Jetson Orin-NX  
A concise, one-page recipe that cures the most common causes of edge-softness or smearing when you use the Arducam IMX477 with JetPack 6 (kernel 5.15 / L4T 36.x).

---

## 0 . Prerequisites
* JetPack **6.0 DP – 6.2** running on a Jetson Orin-NX  
  (`uname -r` should print `5.15.148-tegra-36.4.3` or newer).
* The Arducam driver package you just installed:
  `arducam-nvidia-l4t-kernel-t234-nx-5.15.148-tegra-36.4.3-xxxx_imx477.deb`

---

## 1 . (Only once) Register & load the driver module
```bash
sudo depmod -a                      # rebuild module dependency map
sudo modprobe nv_imx477             # load the IMX477 driver
sudo systemctl restart nvargus-daemon
```
> **Tip:** Add `nv_imx477` to `/etc/modules-load.d/imx477.conf` so it autoloads on every boot.

---

## 2 . (Quick test) Run the camera with ISP sharpening **OFF**
```bash
gst-launch-1.0 \
  nvarguscamerasrc sensor-id=0 sensor-mode=2 \
                   tnr-mode=0  tnr-strength=0 \
                   ee-mode=0   ee-strength=0 \
  ! 'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' \
  ! nvvidconv flip-method=0  interpolation-method=5 \
  ! 'video/x-raw(memory:NVMM),width=960,height=720' \
  ! nvegltransform ! nveglglessink -e
```

What the pipeline does  
| Block | Purpose |
|-------|---------|
| `sensor-mode=2` | Uses the IMX477’s native 1920 × 1080 pixel-binned mode — no full-frame resize. |
| `tnr-* = 0`     | Disables temporal-noise-reduction, a frequent source of smearing. |
| `ee-* = 0`      | Disables the ISP’s edge-enhancer. |
| `interpolation-method=5` | Lanczos resampler — the highest-quality scaler in `nvvidconv`. |

If the preview is now sharp everywhere, you can stop here.

---

## 3 . (Permanent fix) Disable TNR & EE globally
Create a tiny override file for the ISP:

```bash
sudo tee /var/nvidia/nvcam/settings/camera_overrides.isp >/dev/null <<'EOF'
tnr.v1.enable      = 0;      # turn off temporal noise reduction
sharpness.v5.enable = FALSE; # turn off edge enhancer
EOF
sudo chmod 644 /var/nvidia/nvcam/settings/camera_overrides.isp
sudo systemctl restart nvargus-daemon
```

All applications (GStreamer, OpenCV, ROS, etc.) will now inherit the sharper output without extra flags.

---

## 4 . Troubleshooting checklist
| Symptom | Quick check | Fix |
|---------|-------------|-----|
| `modprobe nv_imx477` fails | `dmesg | tail -20` | Kernel mismatch – flash the matching JetPack or rebuild the module. |
| Corners still soft | Refocus and verify back-focus spacers on the C/CS-mount lens. |
| Noise or tear in preview | Ribbon cable pinched or too long; re-seat or shorten the cable. |
| Can’t find `camera_overrides.isp` | Path is case-sensitive (`nvcam/settings`). Create the dirs if missing. |

---

## 5 . Credits & references
* **NVIDIA L4T 36.x ISP guide:** explains `tnr-mode`, `ee-mode`, and `interpolation-method`.  
* **Arducam Jetson driver repo:** <https://github.com/ArduCAM/MIPI_Camera> (release notes list supported JetPack builds).  
* **Community thread “Blurry IMX477 on JP-6”** (Mar 2025) — the origin of the override trick.

---

Happy capturing!  🎥
```
