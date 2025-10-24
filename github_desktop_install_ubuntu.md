# 🧩 Install GitHub Desktop on Ubuntu (via Flatpak & Flathub)

This guide explains how to install **GitHub Desktop** on Ubuntu using **Flatpak** and **Flathub**.

---

## 🧰 Step 1. Install Flatpak

First, install **Flatpak** package manager:

```bash
sudo apt install flatpak -y
```

---

## 🧩 Step 2. Enable Flathub Repository

Flathub is the main repository for Flatpak applications.

```bash
sudo apt install gnome-software-plugin-flatpak -y
flatpak remote-add --if-not-exists flathub https://dl.flathub.org/repo/flathub.flatpakrepo
```

---

## 🔁 Step 3. Restart Your System

To apply Flatpak changes, **restart your computer**:

```bash
sudo reboot
```

---

## 💻 Step 4. Install GitHub Desktop

After reboot, install **GitHub Desktop** from Flathub:

```bash
flatpak install flathub io.github.shiftey.Desktop -y
```

---

## 🚀 Step 5. Run GitHub Desktop

To start GitHub Desktop, run:

```bash
flatpak run io.github.shiftey.Desktop
```

Or find **GitHub Desktop** in your application menu.

---

## ✅ Done!

You now have **GitHub Desktop** installed on Ubuntu via Flatpak 🎉

---

**Optional:** To update GitHub Desktop later:

```bash
flatpak update
```
