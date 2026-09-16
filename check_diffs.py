import os

diffs_file = r"C:\Users\maxim\.gemini\antigravity-ide\brain\e7e1041f-5fcc-4fc8-93ec-622bb5881b01\scratch\code_diffs.txt"

if not os.path.exists(diffs_file):
    print("code_diffs.txt not found")
    exit(1)

with open(diffs_file, "r", encoding="utf-8") as f:
    lines = f.readlines()

diff_indices = [i for i, l in enumerate(lines) if l.startswith("--- aidwear")]

for i, idx in enumerate(diff_indices):
    start = idx
    end = diff_indices[i + 1] if i + 1 < len(diff_indices) else len(lines)
    file_lines = lines[start:end]
    print(f"{lines[idx].strip()} ({len(file_lines)} lines)")
