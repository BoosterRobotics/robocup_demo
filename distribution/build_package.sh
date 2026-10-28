#!/bin/sh
# 获取当前脚本文件的绝对路径
script_path=$(readlink -f "$0")
echo "Script path: $script_path"


# 提取父目录路径作为根路径
root_path=$(dirname $(dirname $script_path))
echo "root_path: $root_path"

cd `dirname $0`
cd ..

rm -rf build/ install/ log/

colcon build
echo "Build success"

git_commit_id=$(git rev-parse --short=6 HEAD)

package_dir="$root_path/distribution/packages/robocup_$git_commit_id"
if [ ! -d "$package_dir" ]; then
    mkdir -p "$package_dir"
fi

vision_model_src_dir="$package_dir/src/vision/model/"
if [ ! -d "$vision_model_src_dir" ]; then    # 注意这里修复了原来的语法错误
    mkdir -p "$vision_model_src_dir"
fi

vision_config_src_dir="$package_dir/src/vision/config/"
if [ ! -d "$vision_config_src_dir" ]; then    # 注意这里修复了原来的语法错误
    mkdir -p "$vision_config_src_dir"
fi

rsync -aL "$root_path/install/" "$package_dir/install/"
rsync -aL "$root_path/scripts/" "$package_dir/scripts/"
rsync -aL "$root_path/utils/" "$package_dir/utils/"
rsync -aL "$root_path/configs/" "$package_dir/configs/"
rsync -aL "$root_path/src/vision/model/" "$vision_model_src_dir"
rsync -aL "$root_path/src/vision/config/" "$vision_config_src_dir"
cp "$root_path/distribution/install.sh" "$package_dir"
chmod +x "$root_path/distribution/install.sh"

echo "Copy success"

makeself --tar-format pax "$package_dir" "$root_path/distribution/packages/robocup_$git_commit_id.run" "Booster Robocup Installer" "./install.sh"
echo "Installer generated."