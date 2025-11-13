#!/usr/bin/env bash

[ -z "$PS1" ] && return

RED="\[\e[0;31m\]"
GREEN="\[\e[0;32m\]"
YELLOW="\[\e[0;33m\]"
BLUE="\[\e[0;34m\]"
CYAN="\[\e[0;36m\]"
RESET="\[\e[0m\]"

parse_git_branch() {
    git branch 2>/dev/null | sed -n '/\* /s///p'
}

export PS1="${CYAN}\u${RESET}@${YELLOW}\h${RESET}:${BLUE}\w${RESET} \$( \
  branch=\$(parse_git_branch); \
  if [ -n \"\$branch\" ]; then \
    echo \"(${GREEN}\$branch${RESET})\"; \
  fi \
) \$ "
