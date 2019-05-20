## Default script header
```
BLACK=$(tput setaf 0)
RED=$(tput setaf 1)
GREEN=$(tput setaf 2)
YELLOW=$(tput setaf 3)
LIME_YELLOW=$(tput setaf 190)
POWDER_BLUE=$(tput setaf 153)
BLUE=$(tput setaf 4)
MAGENTA=$(tput setaf 5)
CYAN=$(tput setaf 6)
WHITE=$(tput setaf 7)
BRIGHT=$(tput bold)
NORMAL=$(tput sgr0)
BLINK=$(tput blink)
REVERSE=$(tput smso)
UNDERLINE=$(tput smul)

DIR=$(dirname "$(readlink -f "$0")")

if [[ -z "$1" || "$1" == "-h" ]]; then
    printf -- "HELP MESSAGE\n"
    printf "${RED}%s${NORMAL}\n" "RED HELP MESSAGE"
    exit -1
fi
```

## Gotchas
- .bashrc cannot be sourced in non-interactive shell (Check first few lines of ~/.bashrc)
- [Bash functions cannot return strings (or anything other than integers)](https://stackoverflow.com/questions/3236871/how-to-return-a-string-value-from-a-bash-function)

## Resources
- <https://github.com/progrium/bashstyle>
- <https://github.com/niieani/bash-oo-framework>
- <https://github.com/kvz/bash3boilerplate>
- <https://github.com/Bash-it/bash-it>

