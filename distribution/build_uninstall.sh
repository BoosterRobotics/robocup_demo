#!/bin/sh
# 获取当前脚本文件的绝对路径
script_path=$(readlink -f "$0")
echo "Script path: $script_path"


# 提取父目录路径作为根路径
root_path=$(dirname $(dirname $script_path))
echo "root_path: $root_path"

cd `dirname $0`
cd ..


git_commit_id=$(git rev-parse --short=6 HEAD)

package_dir="$root_path/distribution/packages/robocup_uninstall_$git_commit_id"
if [ ! -d "$package_dir" ]; then
    mkdir -p "$package_dir"
fi

cp "$root_path/distribution/uninstall.sh" "$package_dir"
chmod +x "$root_path/distribution/uninstall.sh"

echo "Copy success"

makeself "$package_dir" "$root_path/distribution/packages/robocup_uninstall.run" "Booster Robocup UnInstaller" "./uninstall.sh"
echo "UnInstaller generated."
