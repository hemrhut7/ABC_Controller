import os
import shutil
import subprocess
import argparse

def run_scratchpad_task(filename, content, use_shell=False):
    """
    建立臨時環境、執行腳本並保證清理
    """
    scratchpad_dir = ".agent_scratchpad"
    filepath = os.path.join(scratchpad_dir, filename)
    
    try:
        # 1. 確保專案內環境初始化
        os.makedirs(scratchpad_dir, exist_ok=True)
        
        # 2. 寫入內容 (使用 UTF-8 確保相容性)
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(content)
        
        # 3. 判斷執行環境
        if use_shell or filename.endswith(".sh"):
            cmd = ["bash", filepath]
        else:
            # 優先搜尋專案內的 .venv
            python_path = "./.venv/bin/python" if os.path.exists("./.venv") else "python"
            cmd = [python_path, "-u", filepath]
            
        # 4. 執行並捕獲輸出
        process = subprocess.run(cmd, capture_output=True, text=True)
        return f"--- 執行結果 ---\nSTDOUT:\n{process.stdout}\nSTDERR:\n{process.stderr}"
        
    finally:
        # 5. 實現反饋循環中的「確定性清理」
        if os.path.exists(scratchpad_dir):
            shutil.rmtree(scratchpad_dir)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", required=True, help="Filename for the script")
    parser.add_argument("--content", required=True, help="Full code content")
    parser.add_argument("--shell", action="store_true", help="Execute as shell script")
    args = parser.parse_args()
    print(run_scratchpad_task(args.file, args.content, args.shell))