import os
import shutil
import hashlib
import subprocess


class Updater:
    def __init__(self, updatercfg):
        self.repo_url = updatercfg['repo_url']
        self.temp_path = updatercfg['temp_path']

    def __call__(self, repo_path='.'):
        remote_hash = self.get_remote_hash()
        local_hash = self.get_local_hash(repo_path)
        if remote_hash == local_hash:
            print("Already up to date")
        else:
            try:
                subprocess.run(["git", "-C", repo_path, "pull"], check=True)
            except subprocess.CalledProcessError as e:
                raise Exception(f"Update failed: {e.stderr}") from e
        # if os.path.exists(self.temp_path):
        #     shutil.rmtree(self.temp_path)
        # self.run_command(f"git clone {self.repo_url} {self.temp_path}")

        # # 遍历临时文件夹中的文件
        # for root, dirs, files in os.walk(self.temp_path):
        #     # 忽略.git文件夹
        #     if '.git' in dirs:
        #         dirs.remove('.git')
        #     for file_name in files:
        #         temp_file_path = os.path.join(root, file_name)
        #         relative_path = os.path.relpath(temp_file_path, self.temp_path)
        #         local_file_path = os.path.join('./', relative_path)
        #         # 检查本地文件是否存在
        #         if os.path.exists(local_file_path):
        #             # 单独处理config.json文件以不覆盖用户已有配置
        #             if file_name != 'config.json':
        #                 # 计算本地文件和临时文件的MD5值
        #                 local_md5 = self.calculate_md5(local_file_path)
        #                 temp_md5 = self.calculate_md5(temp_file_path)
        #                 # 比较MD5值
        #                 if local_md5 != temp_md5:
        #                     # 如果MD5值不匹配，替换本地文件
        #                     print(f"Updating {local_file_path}")
        #                     shutil.copy2(temp_file_path, local_file_path)
        #                 else:
        #                     print(f"{local_file_path} is up to date")
        #         else:
        #             # 如果本地文件不存在，复制新文件
        #             print(f"Adding new file {local_file_path}")
        #             os.makedirs(os.path.dirname(local_file_path), exist_ok=True)
        #             shutil.copy2(temp_file_path, local_file_path)
        # shutil.rmtree(self.temp_path)

    # def run_command(self, command):
    #     result = subprocess.run(command, shell=True, capture_output=True, text=True)
    #     if result.returncode != 0:
    #         print(f"Error: {result.stderr}")
    #     else:
    #         print(result.stdout)
    #     return result

    # def calculate_md5(self, file_path):
    #     hash_md5 = hashlib.md5()
    #     with open(file_path, "rb") as f:
    #         for chunk in iter(lambda: f.read(4096), b""):
    #             hash_md5.update(chunk)
    #     return hash_md5.hexdigest()

    def get_remote_hash(self):
        try:
            result = subprocess.run(
                ["git", "ls-remote", self.repo_url, "HEAD"],
                capture_output=True,
                text=True,
                check=True
            )
            # 解析第一行的哈希值（格式：哈希值\tHEAD）
            remote_hash = result.stdout.split()[0]
            return remote_hash.strip()
        except subprocess.CalledProcessError as e:
            raise Exception(f"Get remote hash failed: {e.stderr}") from e

    def get_local_hash(self, repo_path="."):
        try:
            result = subprocess.run(
                ["git", "-C", repo_path, "rev-parse", "HEAD"],
                capture_output=True,
                text=True,
                check=True
            )
            return result.stdout.strip()
        except subprocess.CalledProcessError as e:
            raise Exception(f"Get local hash failed: {e.stderr}") from e