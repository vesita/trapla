#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import subprocess
import sys
import datetime
import getpass
import re
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
        
def get_previous_backup_branch():
    """获取上一次的备份分支"""
    returncode, stdout, stderr = run_command("git branch")
    if returncode != 0:
        return None
    
    branches = stdout.strip().split('\n')
    backup_branches = [b.strip().replace('*', '').strip() for b in branches if b.strip().replace('*', '').strip().startswith('backup_')]
    
    if not backup_branches:
        return None
    
    # 返回最新的备份分支（按名称排序，最后一个应该是最新的）
    return sorted(backup_branches)[-1]

def delete_branch(branch_name):
    """删除指定分支"""
    return run_command(f"git branch -D {branch_name}")

def create_backup_branch():
    """创建备份分支以保存当前状态"""
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_branch = f"backup_{timestamp}"
    return run_command(f"git switch -c {backup_branch}")
    
def switch_branch(branch_name):
    """切换到指定分支"""
    return run_command(f"git switch {branch_name}")

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

def fetch_updates():
    """获取远程更新"""
    # 增加重试机制
    for attempt in range(3):
        returncode, stdout, stderr = run_command("git fetch", timeout=60)
        if returncode == 0:
            return returncode, stdout, stderr
        elif returncode == -2:  # 超时情况
            print(f"获取远程更新超时，第 {attempt + 1} 次重试...")
            time.sleep(2 ** attempt)  # 指数退避
        else:
            # 其他错误，不重试
            break
    return returncode, stdout, stderr

def remote_branch_exists(branch_name):
    """检查远程分支是否存在"""
    returncode, _, _ = run_command(f"git rev-parse --verify origin/{branch_name}")
    return returncode == 0

def checkout_branch(branch_name):
    """检出分支"""
    return run_command(f"git switch -c {branch_name} origin/{branch_name}")
    
def merge_branch(branch_name):
    """合并分支"""
    return run_command(f"git merge {branch_name}")

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

def reset_last_commit():
    """撤销最后一次提交，但保持工作区不变"""
    return run_command("git reset --soft HEAD~1")


def validate_branch_name(branch_name):
    """验证分支名是否有效"""
    import re
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

def list_remote_sync_branches():
    """列出所有远程sync-of-*分支"""
    returncode, stdout, stderr = run_command("git branch -r")
    if returncode != 0:
        return []
    
    branches = stdout.strip().split('\n')
    sync_branches = []
    for branch in branches:
        branch = branch.strip()
        if branch.startswith('origin/sync-of-'):
            # 提取不带origin/前缀的分支名
            branch_name = branch.replace('origin/', '')
            sync_branches.append(branch_name)
    
    return sync_branches

def select_sync_branch(default_branch):
    """让用户选择同步分支"""
    sync_branches = list_remote_sync_branches()
    
    # 如果没有远程sync分支，直接返回默认分支
    if not sync_branches:
        return default_branch
    
    # 将默认分支放在列表的开头
    if default_branch in sync_branches:
        sync_branches.remove(default_branch)
        sync_branches.insert(0, default_branch)
    else:
        # 如果默认分支不在列表中，则添加到开头
        sync_branches.insert(0, default_branch)
    
    print("\n找到以下远程同步分支:")
    for i, branch in enumerate(sync_branches, 1):
        if branch == default_branch:
            print(f"  {i}. {branch} (默认)")
        else:
            print(f"  {i}. {branch}")
    
    print(f"  {len(sync_branches) + 1}. 取消")
    
    while True:
        try:
            choice = input(f"\n请选择要同步的分支 (1-{len(sync_branches) + 1})，或直接按回车使用默认分支: ").strip()
            if choice == "":
                return default_branch
            choice = int(choice)
            if 1 <= choice <= len(sync_branches):
                return sync_branches[choice - 1]
            elif choice == len(sync_branches) + 1:
                return None
            else:
                print(f"请输入 1 到 {len(sync_branches) + 1} 之间的数字")
        except ValueError:
            print("请输入有效的数字或直接按回车使用默认分支")

def is_default_branch(branch_name, default_branch):
    """判断是否为默认分支"""
    return branch_name == default_branch

def main():
    print("=== May 项目进度拉取脚本 ===")
    
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
    
    # 获取默认同步分支名
    default_sync_branch = get_sync_branch_name()
    
    # 验证分支名
    if not validate_branch_name(default_sync_branch):
        print(f"错误: 默认同步分支名无效: {default_sync_branch}")
        sys.exit(1)
    
    # 获取远程更新
    print("获取远程更新...")
    returncode, stdout, stderr = fetch_updates()
    if returncode != 0:
        print("获取远程更新失败")
        if returncode == -2:
            print("获取远程更新超时，请检查网络连接")
        else:
            print(stderr)
        sys.exit(1)
    
    # 检查默认分支是否存在
    print(f"默认同步分支: {default_sync_branch}")
    if remote_branch_exists(default_sync_branch):
        # 默认分支存在，直接使用
        sync_branch = default_sync_branch
        print(f"默认分支存在，直接使用: {sync_branch}")
    else:
        # 默认分支不存在，让用户选择其他同步分支
        print("默认分支不存在，可选择其他同步分支")
        sync_branch = select_sync_branch(default_sync_branch)
        
        if sync_branch is None:
            print("操作已取消")
            sys.exit(0)
        
        # 检查所选分支是否存在
        if not remote_branch_exists(sync_branch):
            print(f"错误: 远程分支 {sync_branch} 不存在")
            print("请确保在另一台电脑上运行了推送脚本 (put-sync.py)")
            sys.exit(1)
    
    print(f"将使用同步分支: {sync_branch}")
    
    # 检查本地是否有未提交的更改
    local_has_changes = has_changes()
    backup_branch = None
    if local_has_changes:
        print("检测到本地有未提交的更改，将自动保存到备份分支")
        
        # 删除上一次的备份分支（如果存在）
        previous_backup = get_previous_backup_branch()
        if previous_backup:
            print(f"删除上一次的备份分支: {previous_backup}")
            delete_branch(previous_backup)
        
        # 创建备份分支
        print("创建备份分支以保存当前状态...")
        returncode, stdout, stderr = create_backup_branch()
        if returncode != 0:
            print("创建备份分支失败")
            print(stderr)
            sys.exit(1)
        
        # 提取备份分支名
        backup_branch = stdout.strip().split()[-1] if "Switched to a new branch" in stdout else None
        if backup_branch:
            print(f"当前状态已备份到分支: {backup_branch}")
        
        # 切换回原分支
        returncode, stdout, stderr = switch_branch(current_branch)
        if returncode != 0:
            print(f"切换回 {current_branch} 分支失败")
            sys.exit(1)
        
        print("本地更改已保存到备份分支，将继续同步远程更改")
    
    # 创建本地跟踪分支
    print(f"检出同步分支: {sync_branch}")
    returncode, stdout, stderr = checkout_branch(sync_branch)
    if returncode != 0:
        print("检出同步分支失败")
        print(stderr)
        sys.exit(1)
    
    # 切换回原分支
    print(f"切换回原分支: {current_branch}")
    returncode, stdout, stderr = switch_branch(current_branch)
    if returncode != 0:
        print(f"切换到 {current_branch} 分支失败")
        sys.exit(1)
    
    # 合并到当前分支
    print(f"将更改合并到 {current_branch} 分支...")
    returncode, stdout, stderr = merge_branch(sync_branch)
    if returncode != 0:
        print("合并分支失败")
        print(stderr)
        print("\n提示: 可能存在冲突，请手动解决后重新运行脚本")
        sys.exit(1)
    
    # 撤销合并提交，保持工作区有更改但未提交的状态
    print("重置合并提交，保持更改在工作区...")
    returncode, stdout, stderr = reset_last_commit()
    if returncode != 0:
        print("重置提交失败")
        print(stderr)
        sys.exit(1)
    
    # 删除本地同步分支
    print("清理本地同步分支...")
    returncode, stdout, stderr = delete_local_branch(sync_branch)
    if returncode != 0:
        print(f"删除本地同步分支 {sync_branch} 失败")
        print(stderr)
    
    # 只有当使用默认分支时才删除远程分支，防止删除他人的分支
    if is_default_branch(sync_branch, default_sync_branch):
        print("删除远程同步分支...")
        returncode, stdout, stderr = delete_remote_branch(sync_branch)
        if returncode != 0:
            print(f"删除远程同步分支 {sync_branch} 失败")
            if returncode == -2:
                print("删除远程同步分支超时，请稍后手动删除")
            else:
                print(stderr)
            print("您可以稍后手动执行以下命令删除远程分支:")
            print(f"  git push origin --delete {sync_branch}")
        else:
            print(f"远程同步分支 {sync_branch} 已删除")
    else:
        print(f"注意: 由于使用了非默认分支 {sync_branch}，不会自动删除远程分支")
        print(f"如需手动删除，请运行: git push origin --delete {sync_branch}")
    
    print("\n进度同步完成！")
    print("工作区现在处于有更改但未提交的状态，与推送端推送前的状态一致。")
    
    if backup_branch:
        print(f"注意: 您之前的更改已备份到分支 '{backup_branch}'")
        print("如需恢复之前的更改，可以使用以下命令:")
        print(f"  git checkout {backup_branch}")
        print("查看更改后，您可以选择将更改合并回当前分支或手动处理。")

if __name__ == "__main__":
    main()