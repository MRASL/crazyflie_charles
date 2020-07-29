# Exit immediately if a command exits with a non-zero status.
set -e

ROOT=$PWD

echo "Installing VENV"
pip install virtualenv

# Create virtual env
cd $ROOT
virtualenv .venv
source .venv/bin/activate

# Upgrade pip and install packages
pip install -U pip
pip install -r requirements.txt

echo "Installation done"
