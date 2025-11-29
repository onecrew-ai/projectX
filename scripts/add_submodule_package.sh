#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 2 ]; then
  echo "Usage: $0 <repo-url> <package_name>"
  echo "Example: $0 git@github.com:yourorg/my_pkg.git my_pkg"
  exit 1
fi

REPO_URL="$1"
PKG_NAME="$2"
DEST_PATH="src/${PKG_NAME}"

echo "Adding submodule ${PKG_NAME} at ${DEST_PATH} from ${REPO_URL}"
git submodule add "${REPO_URL}" "${DEST_PATH}"
git submodule update --init "${DEST_PATH}"

echo "Staging submodule changes..."
git add .gitmodules "${DEST_PATH}"

echo "Committing submodule addition"
if git commit -m "Add submodule: ${PKG_NAME}"; then
  echo "Committed. Don't forget to push: git push"
else
  echo "Commit failed (possibly no user.name/email set). Please run 'git commit -m "Add submodule: ${PKG_NAME}"' manually."
fi

echo "Done."
