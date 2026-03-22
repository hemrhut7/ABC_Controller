---
name: managing-git-commits
description: 使用 git_operations API 分析 Git 暫存區變更、生成符合規範的提交訊息並執行 commit。於用戶要求提交程式碼或完成任務後需存檔時使用。
---

# Git 提交管理工作流

## 核心規則
- **自動化優先**：優先調用 `scripts/git_helper.py` 獲取變更內容。
- **格式合規**：訊息必須符合 [FORMAT.md](FORMAT.md) 的 Conventional Commits 規範。

## 執行工作流

1. **提取變更**：運行 `python .agents/execution-runtime/api/git_operations.py diff --staged`。
2. **撰寫草稿**：根據 diff 內容，參考 [FORMAT.md](FORMAT.md) 生成合規的提交訊息。
3. **驗證與修訂**：檢查是否保留專業術語（如 `EKF`, `DMA`）。
4. **執行提交**：運行 `python .agents/execution-runtime/api/git_operations.py commit --message "[訊息內容]"`。

## 錯誤處理
- 若無暫存變更（No staged changes），先執行 `git add`（運行 `python .agents/execution-runtime/api/git_operations.py add .`）。
- 若發生錯誤，分析返回的 `stderr` 並修復。