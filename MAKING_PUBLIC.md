# Making the Showcase Folder Public on GitHub

This guide explains how to make the `showcase/` folder publicly accessible on GitHub while keeping the rest of the repository private.

## Option 1: GitHub Sparse Checkout (Recommended)

This approach allows you to create a separate public repository that only contains the showcase folder.

### Steps:

1. **Create a new public repository** on GitHub (e.g., `rpo-simulation-showcase`)

2. **Clone your private repository** (if not already cloned):
   ```bash
   git clone <your-private-repo-url>
   cd rpo-simulation
   ```

3. **Add the public repository as a remote**:
   ```bash
   git remote add public <your-public-repo-url>
   ```

4. **Use sparse checkout to only include the showcase folder**:
   ```bash
   # Create a new branch for the public repo
   git checkout -b public-showcase
   
   # Configure sparse checkout
   git config core.sparseCheckout true
   echo "showcase/*" > .git/info/sparse-checkout
   
   # Remove all files except showcase
   git read-tree -mu HEAD
   git rm -r --cached .
   git add showcase/
   git commit -m "Add showcase folder for public repository"
   
   # Push to public repository
   git push public public-showcase:main
   ```

5. **Update the public repository** when showcase changes:
   ```bash
   # From your main repository
   git checkout public-showcase
   git merge main  # or cherry-pick specific commits
   git add showcase/
   git commit -m "Update showcase materials"
   git push public public-showcase:main
   ```

## Option 2: Manual Copy (Simpler)

If you prefer a simpler approach, you can manually copy the showcase folder to a separate public repository.

### Steps:

1. **Create a new public repository** on GitHub (e.g., `rpo-simulation-showcase`)

2. **Clone the public repository**:
   ```bash
   git clone <your-public-repo-url>
   cd rpo-simulation-showcase
   ```

3. **Copy showcase files** from your private repository:
   ```bash
   # From your private repo
   cp -r showcase/* <path-to-public-repo>/
   ```

4. **Commit and push**:
   ```bash
   cd <path-to-public-repo>
   git add .
   git commit -m "Add showcase materials"
   git push origin main
   ```

5. **Update when needed**: Repeat steps 3-4 when showcase materials are updated.

## Option 3: GitHub Actions (Automated)

You can set up a GitHub Action to automatically sync the showcase folder to a public repository.

### Steps:

1. **Create a new public repository** on GitHub

2. **Add a GitHub Personal Access Token** with repo permissions to your private repository's secrets:
   - Go to Settings → Secrets → Actions
   - Add a secret named `PUBLIC_REPO_TOKEN` with your GitHub token

3. **Create `.github/workflows/sync-showcase.yml`** in your private repository:
   ```yaml
   name: Sync Showcase to Public Repo
   
   on:
     push:
       paths:
         - 'showcase/**'
       branches:
         - main
     workflow_dispatch:
   
   jobs:
     sync:
       runs-on: ubuntu-latest
       steps:
         - uses: actions/checkout@v3
         - name: Sync to public repo
           run: |
             git config --global user.name "GitHub Actions"
             git config --global user.email "actions@github.com"
             git clone https://${{ secrets.PUBLIC_REPO_TOKEN }}@github.com/<your-username>/rpo-simulation-showcase.git public-repo
             cp -r showcase/* public-repo/
             cd public-repo
             git add .
             git commit -m "Update showcase from private repo" || exit 0
             git push
   ```

## Recommendation

**Option 2 (Manual Copy)** is the simplest and most straightforward for occasional updates. Use **Option 3 (GitHub Actions)** if you want automatic syncing whenever the showcase folder changes.

## Notes

- The showcase folder contains only documentation and visualization files (no source code)
- All images and documentation in the showcase folder are safe to make public
- Remember to update the showcase folder when significant features are added or documentation is updated
- Consider adding a note in the public repository README pointing back to the main (private) repository for code access

