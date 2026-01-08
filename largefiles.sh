#!/bin/bash
# List all files committed to git larger than 50KB (excluding LFS files)

lfs_files=$(git lfs ls-files -n 2>/dev/null)

git ls-files -z | while IFS= read -r -d '' file; do
    # Skip LFS-tracked files
    if echo "$lfs_files" | grep -qxF "$file"; then
        continue
    fi
    if [[ -f "$file" ]]; then
        size=$(stat -c%s "$file" 2>/dev/null)
        if [[ $size -gt 51200 ]]; then
            printf "%8d KB  %s\n" $((size / 1024)) "$file"
        fi
    fi
done | sort -rn
