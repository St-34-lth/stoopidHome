# ~/.bashrc

# -------------------------------
# Pyenv Configuration
# -------------------------------

# Set the location of pyenv
export PYENV_ROOT="$HOME/.pyenv"

# Add pyenv executable to PATH
export PATH="$PYENV_ROOT/bin:$PATH"

# Initialize pyenv only if it's installed
if command -v pyenv 1>/dev/null 2>&1; then
    # For login shells: (adjust if needed)
    eval "$(pyenv init --path)"
    # For interactive shells:
    eval "$(pyenv init -)"
    eval "$(pyenv virtualenv-init -)"
fi