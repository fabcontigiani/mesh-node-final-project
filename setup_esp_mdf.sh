#!/bin/bash

# Create the components directory if it doesn't exist
mkdir -p ./components

# Loop through all items in esp-mdf/components/
for item in esp-mdf/components/*; do
    name=$(basename "$item")
    target="./components/$name"
    # Remove existing file or symlink if present
    [ -e "$target" ] && rm -rf "$target"
    # Create the symlink
    ln -s "../$item" "$target"
done

echo "Symlinks created in ./components/"