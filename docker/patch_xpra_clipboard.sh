#!/bin/bash

# Path to the file you want to patch
TARGET_FILE="/usr/lib/python3/dist-packages/xpra/x11/gtk/clipboard.py"

# Check if the file exists
if [ ! -f "$TARGET_FILE" ]; then
    echo "Error: File not found!"
    exit 1
fi

# Backup the original file
cp "$TARGET_FILE" "${TARGET_FILE}.bak"

# Use sed to find and replace the target block
sed -i '/if self._can_receive:/{
N;
s/self\.targets = tuple(bytestostr(x) for x in (targets or ()))$/if target_data:\
                targets = list(targets) + list(target_data.keys())\
            self.targets = tuple(bytestostr(x) for x in (targets or ()))/;
}' "$TARGET_FILE"

# Confirm the patch was applied
if grep -q "if target_data:" "$TARGET_FILE"; then
    echo "Patch applied successfully."
else
    echo "Error: Patch failed."
    # Restore from backup if patch failed
    mv "${TARGET_FILE}.bak" "$TARGET_FILE"
fi