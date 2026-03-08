---
description: research the issue on the github repo, and try to fix it
---

1. Issue Scoping & Sync
Identify: Use list_issues / get_issue via MCP to define the bug or feature.

Sync: Align requirements with EKF models and Target Metrics (Step 1 & 3).

2. Branch Isolation
Branch: git checkout -b fix/issue-[ID]-name to isolate your ptimization.

Action: Create the branch both locally and on the remote repository.

3. Implementation & Compile
Develop: Execute Step 4 to 6 (Update code and logging).

Validate: Try to compile the project and fix any error in the process of comiling

4. Commits & Remote Sync
Commit: Use descriptive messages (e.g., git commit -m "Add EKF innovation logs").

Push: git push origin <branch_name> to sync progress with GitHub.

5. Verification & Pull Request
PR: Use create_pull_request via MCP

6. Issue Conclusion & Analysis
Reply: Use add_issue_comment to post a summary