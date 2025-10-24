# Create a `fan` Command for Jetson Without Password

This guide explains how to create a custom command called **`fan`** that runs  
`sudo jetson_clocks --fan` without requiring a password.

---

## 🧩 Step 1: Create the `fan` Script

Open a terminal and create a new script file:

```bash
sudo nano /usr/local/bin/fan
```

Paste the following content inside:

```bash
#!/bin/bash
sudo /usr/bin/jetson_clocks --fan
```

Save and exit (`Ctrl + O`, `Enter`, then `Ctrl + X`).

Then make it executable:

```bash
sudo chmod +x /usr/local/bin/fan
```

---

## ⚙️ Step 2: Allow Running Without Password

Edit the sudoers file safely:

```bash
sudo visudo
```

Scroll to the bottom and add this line (replace `YOUR_USERNAME` with your actual username):

```
ituarc ALL=(ALL) NOPASSWD: /usr/bin/jetson_clocks
```

Save and exit (`Ctrl + O`, `Enter`, `Ctrl + X`).

---

## 🚀 Step 3: Test the Command

Run:

```bash
fan
```

If everything is set correctly, your Jetson’s fan control will activate without asking for a password.

---

✅ **Done!**
You now have a quick `fan` command for enabling Jetson clocks and fan mode instantly.
