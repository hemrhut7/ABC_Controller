---
name: run-python
description: Executes Python commands strictly through the project's `.venv`. Includes output monitoring to prevent terminal hanging during high-volume logging.
---

# Python Venv Executor (Monitored)

## Operational Logic
1. **Environment Detection**: Locate `.venv` in the project root. 
   - Linux: `./.venv/bin/python`
   - Windows: `.\.venv\Scripts\python.exe`

2. **Execution Strategy (Anti-Hang)**:
   - **Unbuffered Mode**: Always use the `-u` flag (e.g., `python -u script.py`) to ensure real-time log flushing.
   - **Output Limitation**: For scripts with high-frequency data (e.g., sensor logs), append `| tail -n 100` to the command to prevent buffer overflow.
   - **Polling Mechanism**: For tasks exceeding 10 seconds, use asynchronous execution and poll the process status every 5 seconds.

## Instructions
- **Command Transformation**: Replace `python <args>` with the absolute path to the venv executable + `-u`.
- **Dependency Management**: Use the venv's `pip` for all package installations.
- **Error Analysis**: If execution fails, retrieve only the last 50 lines of `stderr` for troubleshooting.

## Examples
### Ubuntu (High-frequency Log)
`./.venv/bin/python -u src/main.py | tail -n 50`

### Windows (Standard Execution)
`.\.venv\Scripts\python.exe -u tests/ekf_validation.py`