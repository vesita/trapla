#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import subprocess
import sys
import re
import getpass
import time

def run_command(command, timeout=30):
    """运行命令并返回结果"""
    try:
        result = subprocess.run(command, shell=True, capture_output=True, text=True, timeout=timeout)
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return -2, "", f"命令执行超时: {command}"
    except Exception as e:
        return -1, "", str(e)

def is_git_repo():
    """检查当前目录是否为Git仓库"""
    return os.path.exists(".git")

def get_current_branch():
    """获取当前分支名"""
    returncode, stdout, stderr = run_command("git branch --show-current")
    if returncode == 0:
        return stdout.strip()
    else:
        # 尝试获取提交哈希
        returncode, stdout, stderr = run_command("git rev-parse --short HEAD")
        if returncode == 0:
            return stdout.strip()
        else:
            return None

def has_changes():
    """检查是否有未提交的更改"""
    # 检查暂存区更改
    returncode, _, _ = run_command("git diff --cached --quiet")
    has_staged_changes = (returncode != 0)
    
    # 检查未暂存更改
    returncode, _, _ = run_command("git diff --quiet")
    has_unstaged_changes = (returncode != 0)
    
    return has_staged_changes or has_unstaged_changes

def add_all_changes():
    """添加所有更改到暂存区"""
    return run_command("git add -A")

def commit_changes():
    """提交更改"""
    return run_command('git commit -m "WIP sync commit"')

def create_and_switch_branch(branch_name):
    """创建并切换到新分支"""
    return run_command(f"git switch -c {branch_name}")

def push_branch(branch_name):
    """推送分支到远程"""
    # 增加重试机制
    for attempt in range(3):
        returncode, stdout, stderr = run_command(f"git push origin {branch_name}", timeout=60)
        if returncode == 0:
            return returncode, stdout, stderr
        elif returncode == -2:  # 超时情况
            print(f"推送超时，第 {attempt + 1} 次重试...")
            time.sleep(2 ** attempt)  # 指数退避
        else:
            # 其他错误，不重试
            break
    return returncode, stdout, stderr

def switch_branch(branch_name):
    """切换到指定分支"""
    return run_command(f"git switch {branch_name}")

def reset_last_commit():
    """撤销最后一次提交，但保持工作区不变"""
    return run_command("git reset --soft HEAD~1")

def delete_local_branch(branch_name):
    """删除本地分支"""
    return run_command(f"git branch -D {branch_name}")

def delete_remote_branch(branch_name):
    """删除远程分支"""
    # 增加重试机制
    for attempt in range(3):
        returncode, stdout, stderr = run_command(f"git push origin --delete {branch_name}", timeout=30)
        if returncode == 0:
            return returncode, stdout, stderr
        elif returncode == -2:  # 超时情况
            print(f"删除远程分支超时，第 {attempt + 1} 次重试...")
            time.sleep(2 ** attempt)  # 指数退避
        else:
            # 其他错误，不重试
            break
    return returncode, stdout, stderr

def validate_branch_name(branch_name):
    """验证分支名是否有效"""
    # Git分支名不能以.或/开头或结尾，不能包含连续的.或/，不能包含特殊字符
    if not re.match(r"^(?!/)(?!.*(?:\.$|/\.|//|@\{))[^\000-\037\177 ~^:?*\[\\]+$", branch_name):
        return False
    if branch_name in [".", "..", "/", "HEAD", "ORIG_HEAD", "FETCH_HEAD", "MERGE_HEAD", "CHERRY_PICK_HEAD"]:
        return False
    return True

def get_github_username():
    """获取GitHub用户名"""
    # 获取远程仓库URL
    returncode, stdout, stderr = run_command("git remote get-url origin")
    if returncode != 0:
        raise Exception("无法获取远程仓库URL")
    
    remote_url = stdout.strip()
    
    # 从远程URL中提取用户名
    # 支持SSH和HTTPS格式
    # SSH: git@github.com:username/repo.git
    # HTTPS: https://github.com/username/repo.git
    # SSH also could be: ssh://git@github.com/username/repo.git
    
    ssh_pattern = r"git@github\.com:([^/]+)/"
    ssh_pattern2 = r"ssh://git@github\.com/([^/]+)/"
    https_pattern = r"https://github\.com/([^/]+)/"
    
    ssh_match = re.search(ssh_pattern, remote_url)
    ssh_match2 = re.search(ssh_pattern2, remote_url)
    https_match = re.search(https_pattern, remote_url)
    
    if ssh_match:
        return ssh_match.group(1)
    elif ssh_match2:
        return ssh_match2.group(1)
    elif https_match:
        return https_match.group(1)
    else:
        raise Exception(f"无法从远程URL中提取GitHub用户名: {remote_url}")

def get_sync_branch_name():
    """获取同步分支名称"""
    try:
        username = get_github_username()
    except Exception as e:
        print(f"警告: 无法获取GitHub用户名: {e}")
        print("使用系统用户名作为备选方案")
        username = getpass.getuser()
    return f"sync-of-{username}"

def cleanup_on_failure(current_branch, sync_branch):
    """失败时的清理操作"""
    print("执行清理操作...")
    # 切换回主分支
    switch_branch(current_branch)
    # 撤销主分支上的提交
    reset_last_commit()
    # 删除本地同步分支
    delete_local_branch(sync_branch)
    print("清理完成")

def main():
    print("=== May 项目进度推送脚本 ===")
    
    # 检查是否在Git仓库中
    if not is_git_repo():
        print("错误: 当前目录不是Git仓库")
        sys.exit(1)
    
    # 获取当前分支
    current_branch = get_current_branch()
    if current_branch is None:
        print("无法获取当前分支信息")
        sys.exit(1)
    
    print(f"当前分支: {current_branch}")
    
    # 检查是否有更改
    if not has_changes():
        print("没有更改需要提交")
        sys.exit(0)
    
    # 获取同步分支名
    sync_branch = get_sync_branch_name()
    
    # 验证分支名
    if not validate_branch_name(sync_branch):
        print(f"错误: 同步分支名无效: {sync_branch}")
        sys.exit(1)
    
    print(f"将使用同步分支: {sync_branch}")
    
    # 添加所有更改到暂存区
    print("添加所有更改到暂存区...")
    returncode, stdout, stderr = add_all_changes()
    if returncode != 0:
        print("执行 git add 失败")
        print(stderr)
        sys.exit(1)
    
    # 在当前分支上提交更改（不推送）
    print("在当前分支上提交更改...")
    returncode, stdout, stderr = commit_changes()
    if returncode != 0:
        print("提交更改失败")
        print(stderr)
        sys.exit(1)
    
    # 删除远程同步分支（如果存在）
    print("删除远程同步分支（如果存在）...")
    delete_remote_branch(sync_branch)
    
    # 创建并切换到同步分支（基于当前提交）
    print(f"创建同步分支: {sync_branch}")
    returncode, stdout, stderr = create_and_switch_branch(sync_branch)
    if returncode != 0:
        print("创建同步分支失败")
        # 尝试撤销主分支上的提交
        cleanup_on_failure(current_branch, sync_branch)
        sys.exit(1)
    
    # 推送同步分支到远程
    print("推送同步分支到远程仓库...")
    returncode, stdout, stderr = push_branch(sync_branch)
    if returncode != 0:
        print("推送分支失败")
        if returncode == -2:
            print("推送操作超时，请检查网络连接")
        else:
            print(stderr)
        # 切换回主分支并撤销提交
        cleanup_on_failure(current_branch, sync_branch)
        sys.exit(1)
    
    # 切换回主分支
    print(f"切换回原分支: {current_branch}")
    returncode, stdout, stderr = switch_branch(current_branch)
    if returncode != 0:
        print(f"警告：无法切换回原分支 {current_branch}")
        # 尝试清理
        delete_local_branch(sync_branch)
        sys.exit(1)
    
    # 撤销主分支上的提交，保持工作区状态不变
    print("撤销主分支上的提交...")
    returncode, stdout, stderr = reset_last_commit()
    if returncode != 0:
        print("警告：无法撤销主分支上的提交")
        print(stderr)
    
    # 删除本地同步分支
    print("删除本地同步分支...")
    returncode, stdout, stderr = delete_local_branch(sync_branch)
    if returncode != 0:
        print(f"警告：无法删除本地同步分支 {sync_branch}")
    
    print("\n进度推送完成！")
    print("")
    print("请在其他PC上运行 get-sync.py 脚本获取更改")

if __name__ == "__main__":
    main()