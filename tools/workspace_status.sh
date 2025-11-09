#!/bin/sh
# workspace_status.sh - Cung cap thong tin trang thai cho Bazel

# Lay commit hien tai (neu dang trong repo Git)
GIT_COMMIT=$(git rev-parse --short HEAD 2>/dev/null || echo "unknown")

# Kiem tra neu co thay doi chua commit
GIT_DIRTY=$(git diff --quiet 2>/dev/null || echo "-dirty")

# Thoi gian build (dinh dang ISO 8601)
BUILD_TIME=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

# Ten nguoi build (neu co)
USER_NAME=$(whoami 2>/dev/null || echo "unknown")

# Xuat ra cac bien de Bazel ghi vao stable-status.txt
echo "STABLE_GIT_COMMIT ${GIT_COMMIT}${GIT_DIRTY}"
echo "STABLE_BUILD_TIME ${BUILD_TIME}"
echo "STABLE_USER ${USER_NAME}"

exit 0
