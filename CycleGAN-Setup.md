# img2img-turbo Setup Guide

A quick step-by-step README to get your img2img-turbo environment up and running.

---

## 1. Create & Activate the Conda Environment

```bash
# From the directory containing `environment.yml`:
conda env create -f environment.yml

# Then activate it:
conda activate img2img-turbo
```

---

## 2. Install xFormers

```bash
pip install "xformers>=0.0.20"
```

---

## 3. Install Prebuilt PyTorch & TorchVision Wheels
Follow for better installation: (https://docs.ultralytics.com/guides/nvidia-jetson/#flash-jetpack-to-nvidia-jetson)
```bash
pip3 install \
  https://github.com/ultralytics/assets/releases/download/v0.0.0/torch-2.5.0a0+872d972e41.nv24.08-cp310-cp310-linux_aarch64.whl \
  https://github.com/ultralytics/assets/releases/download/v0.0.0/torchvision-0.20.0a0+afc54f7-cp310-cp310-linux_aarch64.whl
```

---

## 4. Patch Out Conflicting Imports

If you run into errors about missing PyTorch symbols, open any scripts that do:

```python
from torch import autograd, cudnn, …
```

and temporarily **comment out** those problematic lines.

---

## 5. Make Attention Fallback Robust

In `inference_unpaired.py`, replace the direct call

```python
model.unet.enable_xformers_memory_efficient_attention()
```

with this try/except snippet:

```python
try:
    model.unet.enable_xformers_memory_efficient_attention()
    print("✅ xFormers enabled.")
except Exception:
    try:
        model.unet.enable_torch_memory_efficient_attention()
        print("✅ PyTorch memory-efficient attention enabled.")
    except AttributeError:
        print("⚠️ Default attention in use.")
```

This ensures you’ll get whichever efficient-attention backend is available.

---

## 6. Verify Your Setup

```bash
# Check key packages
pip list | grep -E "torch|xformers|diffusers"
```

You should see:
- **torch** (2.5.0a0+…)
- **torchvision** (0.20.0a0+…)
- **xformers** (>=0.0.20)
- **diffusers**, **transformers**, etc.

---

🎉 You’re done! Now you can run your img2img-turbo scripts without import or attention errors.
