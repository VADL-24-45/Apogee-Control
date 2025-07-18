This is a simple beginner's guide to using Git to work on files in a repository.

1. **Clone the repository** (if you don’t have it locally yet):

   ```
   git clone <repository-url>
   ```
---
2. **Pull latest before switching or creating a branch:**
   ```
   git pull
   ```
   This prevents working from an outdated base:
---   
3. **Check which branch you’re on / switch branch:**

   ```
   git branch
   git checkout <branch-name>
   git checkout -b <branch-name>
   ```
   Creating a new branch lets you work on changes safely, without affecting the main code. The methods for working with branches are with the listed commands:
   - ```git branch``` will list all local branches available to switch to
   - ```git checkout <branch-name>``` will switch to a local branch
   - ```git checkout -b <branch-name>``` will switch to a **new** local branch
---
4. ***Optional:*** **Pull the latest changes from remote:**

   ```
   git pull
   ```
   You most likely would want to pull again if you're working on a branch with someone else.
---
5. **Edit your file(s)**
   *(Make your changes using a text editor or IDE)*
---
6. **Check the status of changes:**

   ```
   git status
   ```
   This will list all files affected in your work.
---
7. **Stage the changes:**

   ```
   git add <file-name>  
   ```

   Or all files:

   ```
   git add .
   ```
   Staging a change is telling Git that you would like these changes to be in this next commit.
---
8. **Commit the changes with a message:**

   ```
   git commit -m "Describe your changes"
   ```
   A commit is a snapshot of your current progress. While your local computer will save your changes, a commit is proof of your work and is good for documentation. Your commit message should be meaningful and give a quick description on what your work is.
---
9. **Push your changes to the remote repository:**

   ```
   git push
   ```
   Pushing uploads your local commits to the remote repo and keeps your local branch and the corresponding remote branch in sync. Without git push, your commits stay on your machine only — nobody else can see them.
---

### Optional Extras:

* **If collaborating and pulling in changes before pushing:**

  ```
  git pull --rebase
  ```
  Helps avoid unnecessary merge commits. However, if you're not comfortable with rebase, stick with ```git pull``` until you're more familiar with resolving conflicts.
* **Using Visual Studio Code’s UI Instead of the Terminal**
You don’t have to use terminal commands for everything in Git. Visual Studio Code has built-in Source Control tools that can handle most common actions:

  **Clone a Repository:**

   - Go to the Command Palette (Ctrl + Shift + P → “Git: Clone”).

   - Paste the repository URL.

  **Switch Branches / Create Branches:**

   - Bottom-left corner shows the current branch. Click it to switch or create a new branch.

  **Pull Changes:**

  - Click the sync icon in the Source Control sidebar, or use Ctrl + Shift + P → “Git: Pull.”

  **Stage and Commit Changes:**

  - The Source Control sidebar shows all changed files.

  - Click the + icon next to each file to stage.

  - Type a commit message in the text box and click the checkmark to commit.

  **Push Changes:**

  - After committing, the sync icon will show pending changes. Click it to push.

  - ⚠️ Note: Behind the scenes, VS Code is just running the same git commands. Knowing both methods (UI and terminal) is helpful, especially if you switch between different tools or need finer control.

---
### Troubleshooting:

**When ```git pull``` Makes a Pull Request Instead of Pulling**
Sometimes, when working with shared repositories—especially ones with protected branches like main—you might notice:

- You try to pull or push, and instead of pulling changes directly, GitHub suggests making a pull request.

Why This Happens:
- You don’t have direct push permissions to the main branch.

- Your local branch has diverged from the remote branch.

- The repository uses a workflow that requires all changes to go through pull requests for code review and safety.

**How to Handle It:**
1. Run ```git pull```
2. Open the repo
3. Open Pull Requests and find your branch
4. Verify the Pull Request. If unconfident in your work, have others review the pull request and check off on it
5. Merge the pull request once reviewed.
6. Update your local main branch after the pull request is merged:
  ```
  git checkout main
  git pull
  ```