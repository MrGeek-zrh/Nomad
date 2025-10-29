#!/bin/bash

# Check if the script is run as root
if [ "$(id -u)" -ne "0" ]; then
    echo "This script must be run as root" 1>&2
    exit 1
fi

# Define the directory where the kernel images are stored
KERNEL_DIR="/boot"

# List available kernel versions
echo "Available kernel versions:"
ls $KERNEL_DIR/vmlinuz-* | awk -F/ '{print $NF}' | sed 's/vmlinuz-//' | sort -V
echo ""

# Prompt the user to select a kernel version
read -p "Enter the kernel version you want to switch to: " kernel_version

# Check if the selected kernel version exists
if [ ! -e "$KERNEL_DIR/vmlinuz-$kernel_version" ]; then
    echo "Kernel version $kernel_version does not exist"
    exit 1
fi

MID=$(grep -F "submenu 'Advanced options for Ubuntu'" /boot/grub/grub.cfg | head -1 | awk '{print $(NF-1)}' | tr -d "'")
if [ -z "$MID" ]; then
    echo "无法找到 GRUB 高级菜单 ID" 1>&2
    exit 1
fi

KID=$(grep -F "menuentry 'Ubuntu, with Linux $kernel_version' " /boot/grub/grub.cfg | grep -v recovery | head -1 | awk '{print $(NF-1)}' | tr -d "'")
if [ -z "$KID" ]; then
    echo "无法找到与 $kernel_version 对应的 GRUB 菜单 ID" 1>&2
    exit 1
fi

# update-grub

sed -i -E "s|^GRUB_DEFAULT=.*|GRUB_DEFAULT=\"$MID>$KID\"|" /etc/default/grub


if [ $kernel_version = "5.15.19-htmm" ];then
    sed -i '/^[^#].*memmap/ s/^/#/' /etc/default/grub   
else
    sed -i '/^#.*memmap/ s/^#//' /etc/default/grub
fi


update-grub

# Update saved_entry in grubenv to match GRUB_DEFAULT
# This ensures the saved entry matches the kernel we want to boot
saved_entry_text=$(grep -F "menuentry 'Ubuntu, with Linux $kernel_version' " /boot/grub/grub.cfg | grep -v recovery | head -1 | sed -E "s/.*menuentry '([^']*)'.*/\1/")
if [ -n "$saved_entry_text" ]; then
    grub-editenv - set "saved_entry=$saved_entry_text"
    echo "Updated saved_entry to: $saved_entry_text"
else
    echo "Warning: Could not find saved_entry for kernel $kernel_version"
fi

echo -e "\e[31mPlease reboot machine\e[0m"

