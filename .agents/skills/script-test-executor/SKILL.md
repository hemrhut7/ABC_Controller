---
name: script-test-executor
description: Creates, executes, and cleans up temporary test scripts within a hidden directory inside the current project root to avoid external folder permission issues.
---

# Local Project Test Executor Skill

This skill ensures that all temporary diagnostic or test scripts are self-contained within the project workspace, bypassing OS-level permission prompts for directories like `/tmp`.

## Operational Logic

1. **Workspace Localization**:
   - Identify the project root directory.
   - Create a hidden temporary folder named `.agent_scratchpad/` in the project root.

2. **Script Deployment**:
   - Write the required test script (e.g., `.py`, `.sh`, `.cpp`) into `.agent_scratchpad/`.
   - **Mandatory**: Use a unique filename to avoid overwriting existing files.

3. **Execution (Follow Rules)**:
   - For Python: Execute using the `python-venv-executor` skill (via `./.venv`).
   - For C++: Use the local compiler or `make` within the project context.

4. **Guaranteed Cleanup**:
   - Regardless of execution success or failure, the `.agent_scratchpad/` directory and all its contents **MUST** be deleted immediately after results are captured.

## Instructions
- **Prohibition**: Never use `/tmp`, `C:\Temp`, or any path outside the currently trusted workspace.
- **Reporting**: Capture the output, summarize in **Traditional Chinese**, and confirm that the local temporary files have been removed.

## Examples
### Running a local Python diagnostic
1. `mkdir -p .agent_scratchpad`
2. Write `diag.py` to `.agent_scratchpad/diag.py`
3. `./.venv/bin/python -u .agent_scratchpad/diag.py`
4. `rm -rf .agent_scratchpad`