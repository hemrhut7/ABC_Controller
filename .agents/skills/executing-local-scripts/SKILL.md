---
name: executing-local-scripts
description: 在專案根目錄的隱藏資料夾中建立、執行並自動清理臨時測試腳本。用於診斷問題或執行臨時驗證，避開外部目錄權限限制。
---

# 本地專案腳本執行器

## 核心原則
- **環境隔離**：僅限於專案根目錄下的 `.agent_scratchpad/` 執行，嚴禁訪問 `/tmp` 或系統路徑。
- **確定性清理**：無論執行成功或失敗，必須確保資料夾被完整刪除。
- **安全第一**：執行前需簡短說明腳本意圖。

## 執行工作流
建議遵循以下清單以確保安全與清理：

```text
任務進度：
- [ ] 步驟 1：撰寫測試程式碼並進行靜態檢查
- [ ] 步驟 2：使用 scripts/scratchpad_util.py 部署並執行
- [ ] 步驟 3：捕獲輸出並彙整結果
- [ ] 步驟 4：確認 .agent_scratchpad/ 已移除
```

## 操作指令
優先使用預製腳本以減少手動操作錯誤：

## 執行 Python 腳本：
`python scripts/scratchpad_executor.py --file test.py --content "[程式碼內容]"`

## 執行 Bash 腳本：
`python scripts/scratchpad_executor.py --file check.sh --content "[指令內容]" --shell`

## 報告規範
- 執行結果必須使用 繁體中文 彙整。
- 必須明確聲明：「臨時檔案與目錄已依規定清理完畢」。