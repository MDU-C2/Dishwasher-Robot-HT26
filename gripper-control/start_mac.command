#!/bin/zsh
cd "$(dirname "$0")" || exit 1
if [[ ! -f ".venv/bin/python" ]]; then
  echo "Kör först installationsstegen i README.md"
  read -k 1 '?Tryck valfri tangent för att stänga…'
  exit 1
fi
source .venv/bin/activate
python main.py
